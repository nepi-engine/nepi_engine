#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy
import os
import copy
import time
import numpy as np
import math
import threading
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2

from std_msgs.msg import UInt8, Float32, Bool, Empty, String
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import ObjectCount, BoundingBox, BoundingBoxes, ImageClassifierStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_img


EXAMPLE_DETECTION_DICT_ENTRY = {
    'name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'prob': .3, # Probability of detection
    'xmin': 10,
    'ymin': 10,
    'xmax': 100,
    'ymax': 100
}

MIN_THRESHOLD = 0.01
MAX_THRESHOLD = 1.0
DEFUALT_THRESHOLD = 0.3

MIN_MAX_RATE = 1
MAX_MAX_RATE = 20
DEFAULT_MAX_RATE = 5

DEFAULT_IMG_TILING = False
DEFAULT_LABELS_OVERLAY = True
DEFAULT_CLF_OVERLAY = False
DEFAULT_IMG_OVERLAY = False



class AiNodeIF:

    MODEL_CONFIG_FOLDER = "/mnt/nepi_storage/user_cfg/ros" 

    node_namespace = ""
    config_file_path = ""
    self_managed = True
    model_name = "None"

    img_connected = False

    img_acquire = False
    img_msg = None
    last_img_msg = None
    img_lock = threading.Lock()

    last_time = time.time()
    img_subs_dict = dict()


    init_img_topics = []
    init_img_tiling = DEFAULT_IMG_TILING
    init_overlay_labels = DEFAULT_LABELS_OVERLAY
    init_overlay_clf_name = DEFAULT_CLF_OVERLAY
    init_overlay_img_name = DEFAULT_IMG_OVERLAY
    init_img_topic = "None"
    init_threshold = DEFUALT_THRESHOLD
    init_max_rate = DEFAULT_MAX_RATE


    save_cfg_if = None

    state = 'Loading'
    enabled = False
    mgr_registered = False

    threshold_active_sub = None
    max_rate_active_sub = None
    set_img_topic_sub = None
    overlay_clf_name_active_sub = None
    overlay_img_name_active_sub = None
    reset_factory_active_sub = None

    def __init__(self, model_name, mgr_namespace, classes_list, defualt_config_dict, processDetectionFunction):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ##############################  
        if mgr_namespace.find(self.node_name) == -1:
            self.self_managed = False
        # Create a message image to publish when not running
        message = "NOT ENABLED"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_not_enabled_img = nepi_img.cv2img_to_rosimg(cv2_img) 
        message = "IMAGE SOURCE NOT SELECTED"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_no_img_topic_img = nepi_img.cv2img_to_rosimg(cv2_img)
        message = "WAITING FOR IMAGE STREAM"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_no_img_img = nepi_img.cv2img_to_rosimg(cv2_img)


        get_folder_name_service = self.base_namespace + 'system_storage_folder_query'
        nepi_msg.publishMsgInfo(self,"Waiting for system automation scripts folder query service " + get_folder_name_service)
        rospy.wait_for_service(get_folder_name_service)
        nepi_msg.publishMsgInfo(self,"Calling system automation scripts folder query service " + get_folder_name_service)
        try:
            folder_query_service = rospy.ServiceProxy(get_folder_name_service, SystemStorageFolderQuery)
            time.sleep(1)
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain folder query service " + str(e))
        try:
            nepi_msg.publishMsgInfo(self,"Getting AI Model config folder query service " + get_folder_name_service)
            response = folder_query_service("user_cfg/ros")
            nepi_msg.publishMsgInfo(self,"Got AI Model config folder path" + response.folder_path)
            self.MODEL_CONFIG_FOLDER = response.folder_path
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain AI Model config folder, falling back to: " + self.MODEL_CONFIG_FOLDER + " " + str(e))


        self.model_name = model_name
        self.defualt_config_dict = defualt_config_dict
        if mgr_namespace[-1] == "/":
            mgr_namespace = mgr_namespace[:-1]
        self.mgr_namespace = mgr_namespace
        self.processDetection = processDetectionFunction

        self.classes_list = classes_list
        self.classes_color_list = self.get_classes_color_list(classes_list)
                 
        nepi_msg.publishMsgInfo(self,"Starting IF setup")

        # Load/Create classifier config params
        self.node_namespace = os.path.join(self.base_namespace,"ai",self.node_name)
        self.config_file_path = os.path.join(self.MODEL_CONFIG_FOLDER,self.node_name)
        nepi_ros.load_config_file(self.config_file_path, defualt_config_dict, self.node_namespace)
        nepi_msg.publishMsgWarn(self,"Loaded AI Classifier params")
        nepi_ros.print_node_params(self)
        


        self.status_pub = rospy.Publisher("~status", ImageClassifierStatus,  queue_size = 1, latch = True)
        
        # Create AI Node Publishers
        #self.found_object_pub = rospy.Publisher("~found_object", ObjectCount,  queue_size = 1)
        #self.bounding_boxes_pub = rospy.Publisher("~bounding_boxes", BoundingBoxes, queue_size = 1)
        #self.detection_image_pub = rospy.Publisher("~detection_image", Image,  queue_size = 1)
        
        # Create AI Node Subscribers
        rospy.Subscriber('~add_img_topic', String, self.addImageTopicCb, queue_size=10)
        rospy.Subscriber('~remove_img_topic', String, self.removeImageTopicCb, queue_size=1)
        rospy.Subscriber('~set_img_topic', String, self.setImageTopicCb, queue_size=1)
        rospy.Subscriber('~set_image_tiling', Bool, self.setTileImgCb, queue_size=1)
        rospy.Subscriber('~set_overlay_labels', Bool, self.setOverlayLabelsCb, queue_size=1)
        rospy.Subscriber('~set_overlay_clf_name', Bool, self.setOverlayClfNameCb, queue_size=1)
        rospy.Subscriber('~set_overlay_img_name', Bool, self.setOverlayImgNameCb, queue_size=1)
        rospy.Subscriber('~set_threshold', Float32, self.setThresholdCb, queue_size=1)
        rospy.Subscriber('~set_max_rate', Float32, self.setMaxRateCb, queue_size=10)
        rospy.Subscriber('~set_enable', Bool, self.setEnbaleClassifierCb, queue_size=1)
        rospy.Subscriber('~reset_factory', Empty, self.resetFactoryCb, queue_size=1) # start local callback
        rospy.Subscriber('~save_config', Empty, self.saveConfigCb, queue_size=1) # start local callback
        rospy.Subscriber('~reset_config', Empty, self.resetConfigCb, queue_size=1) # start local callback


        # Create AI Mgr Node Publishers

        self.found_object_pub = rospy.Publisher(self.mgr_namespace + '/found_object', ObjectCount,  queue_size = 1)
        self.bounding_boxes_pub = rospy.Publisher(self.mgr_namespace + '/bounding_boxes', BoundingBoxes, queue_size = 1)
        self.detection_image_pub = rospy.Publisher(self.mgr_namespace + '/detection_image', Image,  queue_size = 1)

        if self.mgr_namespace != self.node_namespace:
            self.mgr_registered = True
            self.status_mgr_pub = rospy.Publisher(self.mgr_namespace + '/active_classifier/status', ImageClassifierStatus,  queue_size = 1)

        rospy.Timer(rospy.Duration(1), self.publishStatusCb)
        rospy.Timer(rospy.Duration(.1), self.updaterCb, oneshot = True)
        rospy.Timer(rospy.Duration(.1), self.updateDetectionCb, oneshot = True)

        self.state = 'Loaded'
        nepi_msg.publishMsgInfo(self,"IF Initialization Complete")
        
    def saveConfigCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Save Config")
        nepi_ros.save_config_file(self.config_file_path,self.node_namespace) 

    def resetConfigCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Reset Config")
        self.resetParamServer()


    def resetFactoryCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recived Factory Reset")
        self.resetFactory()

    def resetFactory(self):
        nepi_msg.publishMsgInfo(self,"Factory Reseting Param Server")
        rospy.set_param('~img_topics', [])
        rospy.set_param('~img_topic', "None")
        rospy.set_param('~img_tiling', DEFAULT_IMG_TILING)
        rospy.set_param('~overlay_labels', DEFAULT_LABELS_OVERLAY)
        rospy.set_param('~overlay_clf_name', DEFAULT_CLF_OVERLAY)
        rospy.set_param('~overlay_img_name', DEFAULT_IMG_OVERLAY)
        rospy.set_param('~threshold', self.defualt_config_dict['threshold'])
        rospy.set_param('~max_rate', self.defualt_config_dict['max_rate'])
        self.enabled = False

    def resetParamServer(self,do_updates = False):
        nepi_msg.publishMsgInfo(self,"Reseting Param Server")
        rospy.set_param('~img_topics', self.init_img_topics)
        rospy.set_param('~img_topic', self.init_img_topic)
        rospy.set_param('~img_tiling', self.init_img_tiling)
        rospy.set_param('~overlay_labels',self.init_overlay_labels)
        rospy.set_param('~overlay_clf_name', self.init_overlay_clf_name)
        rospy.set_param('~overlay_img_name', self.init_overlay_img_name)
        rospy.set_param('~threshold', self.init_threshold)
        rospy.set_param('~max_rate', self.init_max_rate)
        if do_updates:
            self.updateFromParamServer()

    def initializeParamServer(self,do_updates = False):
        nepi_msg.publishMsgInfo(self,"Initializing Param Server")
        self.init_img_topics = rospy.get_param('~img_topics', [])
        self.init_img_topic = rospy.get_param('~img_topic', "None")
        self.init_img_tiling = rospy.get_param('~img_tiling', DEFAULT_IMG_TILING)
        self.init_overlay_labels = rospy.get_param('~overlay_labels', DEFAULT_LABELS_OVERLAY)
        self.init_overlay_clf_name = rospy.get_param('~overlay_clf_name', DEFAULT_CLF_OVERLAY)
        self.init_overlay_img_name = rospy.get_param('~overlay_img_name', DEFAULT_IMG_OVERLAY)
        self.init_threshold = rospy.get_param('~threshold', self.defualt_config_dict['threshold'])
        self.init_max_rate = rospy.get_param('~max_rate', self.defualt_config_dict['max_rate'])
        self.resetParamServer(do_updates)

    def updateFromParamServer(self):
        pass # Handled by config_if


    def addImageTopicCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received Add Image Topic: " + msg.data)
        img_topic = msg.data
        self.addImageTopic(img_topic)

    def addImageTopic(self,img_topic):   
        img_topics = rospy.get_param('~img_topics', self.init_img_topics)
        if img_topic not in img_topics:
            img_topics.append(img_topic)
        rospy.set_param('~img_topics', img_topics)
        self.publishStatus()

    def removeImageTopicCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received Remove Image Topic: " + msg.data)
        img_topic = msg.data
        self.removeImageTopic(img_topic)

    def removeImageTopic(self,img_topic):         
        img_topics = rospy.get_param('~img_topics', self.init_img_topics)
        if img_topic in img_topics:
            img_topics.remove(img_topic)
        rospy.set_param('~img_topics', img_topics)
        self.publishStatus()



    def setImageTopicCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received Activate Image Topic: " + msg.data)
        img_topic = msg.data
        rospy.set_param('~img_topic', img_topic)
        self.publishStatus()

    def setTileImgCb(self,msg):
        rospy.set_param('~img_tiling', msg.data)
        self.publishStatus()

    def setOverlayLabelsCb(self,msg):
        rospy.set_param('~overlay_labels', msg.data)
        self.publishStatus()

    def setOverlayClfNameCb(self,msg):
        rospy.set_param('~overlay_clf_name', msg.data)
        self.publishStatus()

    def setOverlayImgNameCb(self,msg):
        rospy.set_param('~overlay_img_name', msg.data)
        self.publishStatus()


    def setThresholdCb(self,msg):
        threshold = msg.data
        nepi_msg.publishMsgInfo(self,"Received Threshold Update: " + str(threshold))
        if threshold <  MIN_THRESHOLD:
            threshold = MIN_THRESHOLD
        elif threshold > MAX_THRESHOLD:
            threshold = MAX_THRESHOLD
        rospy.set_param('~threshold', threshold)
        self.publishStatus()

    def setMaxRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        rospy.set_param('~max_rate', max_rate)
        self.publishStatus()

    def setEnbaleClassifierCb(self,msg):
        #nepi_msg.publishMsgInfo(self,"Received Enable Update: " + str(msg.data))
        #nepi_msg.publishMsgInfo(self,"Current Enable state: " + str(self.enabled))
        if self.enabled != msg.data:
            self.enabled = msg.data
            self.publishStatus()

    def statusPublishCb(self,timer):
        self.publishStatus()


    def imageCb(self,image_msg, args):  
        self.img_connected = True
        source_img_topic = args 
        sel_img_topic = rospy.get_param('~img_topic', self.init_img_topic)
        get_image = self.enabled and (source_img_topic == sel_img_topic)
        if get_image == True:
            self.img_lock.acquire()
            self.img_msg = copy.deepcopy(image_msg)
            self.img_lock.release()
            #self.last_img_msg = copy.deepcopy(image_msg)
            



    def updaterCb(self,timer):
        enabled = self.enabled
        #nepi_msg.publishMsgWarn(self,"Updating with image topic: " +  self.img_topic)
        img_topics = rospy.get_param('~img_topics', self.init_img_topics)
        img_topic = rospy.get_param('~img_topic', self.init_img_topic)
        if img_topic != "None":
            img_topic = nepi_ros.find_topic(img_topic)
            if img_topic != '' and img_topic in img_topics and img_topic not in self.img_subs_dict.keys() :
                nepi_msg.publishMsgWarn(self,'Subsribing to image topic: ' + img_topic)
                img_sub = rospy.Subscriber(img_topic, Image, self.imageCb, queue_size=1, callback_args=(img_topic))
                self.img_subs_dict[img_topic] = img_sub
                time.sleep(1)
                rospy.set_param('~img_topic', img_topic)
            elif img_topic not in img_topics:
                rospy.set_param('~img_topic', "None")
        else:
            self.img_connected = False
        for img_topic in img_topics:
            if img_topic not in self.img_subs_dict.keys():
                nepi_msg.publishMsgWarn(self,'Subsribing to image topic: ' + img_topic)
                img_sub = rospy.Subscriber(img_topic, Image, self.imageCb, queue_size=1, callback_args=(img_topic))
                self.img_subs_dict[img_topic] = img_sub
        purge_list = []
        for img_topic in self.img_subs_dict.keys():
            if img_topic not in img_topics:
                purge_list.append(img_topic)
        for img_topic in purge_list:
            nepi_msg.publishMsgWarn(self,'Unregistering image topic: ' + img_topic)
            img_sub = self.img_subs_dict[img_topic]
            img_sub.unregister()
            del self.img_subs_dict[img_topic]
        if enabled == True and len(self.img_subs_dict.keys()) > 0:
            if self.img_connected == True:
                self.state = "Running"
            else:
                self.state = "Waiting"
        else:
            self.state = "Loaded"
        rospy.Timer(rospy.Duration(.01), self.updaterCb, oneshot = True)



    def updateDetectionCb(self,timer):
        if self.enabled == True:
            threshold = rospy.get_param('~threshold', self.init_threshold)
            max_rate = rospy.get_param('~max_rate', self.init_max_rate)
            delay_time = float(1) / max_rate
            current_time = time.time()
            timer = current_time - self.last_time

            #nepi_msg.publishMsgWarn(self,"Delay and Timer: " + str(delay_time) + " " + str(timer))
            if timer > delay_time:
                self.last_time = current_time
                detect_dict_list = None
                img_in_msg = None
                self.img_lock.acquire()
                img_in_msg = copy.deepcopy(self.img_msg) 
                self.img_msg = None # Clear the last image        
                self.img_lock.release()
                if img_in_msg is not None:
                    ros_img_header = img_in_msg.header
                    detect_img_msg = img_in_msg
                    cv2_img = nepi_img.rosimg_to_cv2img(img_in_msg)
                    cv2_shape = cv2_img.shape
                    img_width = cv2_shape[1] 
                    img_height = cv2_shape[0] 
                    if nepi_img.is_gray(cv2_img) == True:
                        cv2_img = nepi_img.grayscale_to_rgb(cv2_img)
                    try:
                        detect_dict_list = self.processDetection(cv2_img,threshold) 
                        #nepi_msg.publishMsgInfo(self,"AIF got back detect_dict: " + str(detect_dict_list))
                        success = True
                    except Exception as e:
                        nepi_msg.publishMsgWarn(self,"Failed to process detection img with exception: " + str(e))

                    if detect_dict_list is not None:
                        # Publish image first for consumers
                        if len(detect_dict_list) > 0:
                            #nepi_msg.publishMsgWarn(self,"Starting detect image: " + str(cv2_img.shape))
                            cv2_detect_img = self.apply_detection_overlay(detect_dict_list,cv2_img)
                            #nepi_msg.publishMsgWarn(self,"Return detect image: " + str(cv2_detect_img.shape))
                            detect_img_msg = nepi_img.cv2img_to_rosimg(cv2_detect_img, encoding="bgr8")
                            if not rospy.is_shutdown() and self.detection_image_pub is not None:
                                    self.detection_image_pub.publish(detect_img_msg)
                        self.publishDetectionData(detect_dict_list,ros_img_header)
                        # Now create and publish detection image

                        #else:
                            #nepi_msg.publishMsgWarn(self,"No detections to add to image")
                    else:
                        nepi_ros.signal_shutdown("Something went wrong in detection process call")
                        nepi_ros.sleep(2)

        rospy.Timer(rospy.Duration(.01), self.updateDetectionCb, oneshot = True)


    def getImgShortName(self, img_topic):
        short_name = img_topic.replace(self.base_namespace,"")
        if short_name.find("idx") != -1:
            short_name = short_name.replace("/idx","")
        return short_name
   

    def apply_detection_overlay(self,detect_dict_list,cv2_img):
        cv2_detect_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 

                

        for detect_dict in detect_dict_list:
            ###### Apply Image Overlays and Publish Image ROS Message
            # Overlay adjusted detection boxes on image 
            class_name = detect_dict['name']
            xmin = detect_dict['xmin'] + 5
            ymin = detect_dict['ymin'] + 5
            xmax = detect_dict['xmax'] - 5
            ymax = detect_dict['ymax'] - 5
            start_point = (xmin, ymin)
            end_point = (xmax, ymax)

            # Overlay text data on OpenCV image
            font                   = cv2.FONT_HERSHEY_DUPLEX
            fontScale, thickness  = nepi_img.optimal_font_dims(cv2_detect_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
            fontColor = (255, 255, 255)
            fontColorBk = (0,0,0)
            lineType = cv2.LINE_AA


            class_color = (255,0,0)
            
            if class_name in self.classes_list:
                class_ind = self.classes_list.index(class_name)
                if class_ind < len(self.classes_color_list):
                    class_color = tuple(self.classes_color_list[class_ind])
            class_color =  [int(c) for c in class_color]
            img_size = cv2_img.shape[:2]
            line_thickness = 2

            if xmax <= img_size[1] and ymax <= img_size[0]:

                success = False
                try:
                    cv2.rectangle(cv2_detect_img, start_point, end_point, class_color, thickness=thickness)
                    success = True
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to create bounding box rectangle: " + str(e))

                # Overlay text data on OpenCV image
                if success == True:


                    ## Overlay Labels
                    overlay_labels =  rospy.get_param('~overlay_labels',self.init_overlay_labels)
                    if overlay_labels:
                        text2overlay=class_name
                        text_size = cv2.getTextSize(text2overlay, 
                            font, 
                            fontScale,
                            thickness)
                        #nepi_msg.publishMsgWarn(self,"Text Size: " + str(text_size))
                        line_height = text_size[0][1]
                        line_width = text_size[0][0]
                        padding = int(line_height*0.4)
                        bottomLeftCornerOfText = (xmin + (line_thickness * 2), ymin + (line_thickness * 2) + line_height)
                        # Create Text Background Box

                        start_point = (bottomLeftCornerOfText[0] + line_thickness-padding, bottomLeftCornerOfText[1]-line_height-padding)
                        end_point = (bottomLeftCornerOfText[0]+ line_thickness + line_width + padding, bottomLeftCornerOfText[1]+ (padding *2) )
                        box_color = [0,0,0]

                        try:
                            cv2.rectangle(cv2_detect_img, start_point, end_point, box_color , -1)
                            cv2.putText(cv2_detect_img,text2overlay, 
                                bottomLeftCornerOfText, 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                lineType)
                        except Exception as e:
                            nepi_msg.publishMsgWarn(self,"Failed to apply overlay text: " + str(e))

                        # Start name overlays    
                        x_start = int(img_width * 0.05)
                        y_start = int(img_height * 0.05)
                        ## Overlay Classifier Name
                        overlay_clf_name = rospy.get_param('~overlay_clf_name', self.init_overlay_clf_name)
                        if overlay_clf_name:
                            text2overlay=self.model_name
                            text_size = cv2.getTextSize(text2overlay, 
                                font, 
                                fontScale,
                                thickness)
                            #nepi_msg.publishMsgWarn(self,"Text Size: " + str(text_size))
                            line_height = text_size[0][1]
                            line_width = text_size[0][0]
                            bottomLeftCornerOfText = (x_start,y_start)
                            try:
                                cv2.putText(cv2_detect_img,text2overlay, 
                                    bottomLeftCornerOfText, 
                                    font, 
                                    fontScale,
                                    fontColorBk,
                                    thickness*2,
                                    lineType)
                                cv2.putText(cv2_detect_img,text2overlay, 
                                    bottomLeftCornerOfText, 
                                    font, 
                                    fontScale,
                                    fontColor,
                                    thickness,
                                    lineType)
                            except Exception as e:
                                nepi_msg.publishMsgWarn(self,"Failed to apply overlay text: " + str(e))
                            y_start = y_start + int(line_height * 1.5)
                    ## Overlay Image Name
                    overlay_img_name = rospy.get_param('~overlay_img_name', self.init_overlay_img_name)
                    img_topic = rospy.get_param('~img_topic', self.init_img_topic)
                    if overlay_img_name:
                        text2overlay=self.getImgShortName(img_topic)
                        text_size = cv2.getTextSize(text2overlay, 
                            font, 
                            fontScale,
                            thickness)
                        #nepi_msg.publishMsgWarn(self,"Text Size: " + str(text_size))
                        line_height = text_size[0][1]
                        line_width = text_size[0][0]
                        bottomLeftCornerOfText = (x_start,y_start)
                        try:
                            cv2.putText(cv2_detect_img,text2overlay, 
                                bottomLeftCornerOfText, 
                                font, 
                                fontScale,
                                fontColorBk,
                                thickness*2,
                                lineType)
                            cv2.putText(cv2_detect_img,text2overlay, 
                                bottomLeftCornerOfText, 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                lineType)
                        except Exception as e:
                            nepi_msg.publishMsgWarn(self,"Failed to apply overlay text: " + str(e))


            else:
                nepi_msg.publishMsgWarn(self,"xmax or ymax out of range: " + str(img_size)) 
        return cv2_detect_img

    def publishDetectionData(self,detect_dict_list,ros_img_header):

        if len(detect_dict_list) > 0:
            bounding_box_msg_list = []
            for detect_dict in detect_dict_list:
                try:
                    bounding_box_msg = BoundingBox()
                    bounding_box_msg.Class = detect_dict['name']
                    bounding_box_msg.id = detect_dict['id']
                    bounding_box_msg.uid = detect_dict['uid']
                    bounding_box_msg.probability = detect_dict['prob']
                    bounding_box_msg.xmin = detect_dict['xmin']
                    bounding_box_msg.ymin = detect_dict['ymin']
                    bounding_box_msg.xmax = detect_dict['xmax']
                    bounding_box_msg.ymax = detect_dict['ymax']
                    area_pixels = (detect_dict['xmax'] - detect_dict['xmin']) * (detect_dict['ymax'] - detect_dict['ymin'])
                    img_area = detect_dict['width_pixels'] * detect_dict['height_pixels']
                    if img_area > 1:
                        area_ratio = area_pixels / img_area
                    else:
                        area_ratio = -999
                    bounding_box_msg.area_pixels = detect_dict['area_pixels']
                    bounding_box_msg.area_ratio = detect_dict['area_ratio']
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to get all data from detect dict: " + str(e)) 
                bounding_box_msg_list.append(bounding_box_msg)
            bounding_boxes_msg = BoundingBoxes()
            bounding_boxes_msg.header.stamp = ros_img_header.stamp
            bounding_boxes_msg.classifier_name = self.node_name
            bounding_boxes_msg.image_header = ros_img_header
            bounding_boxes_msg.image_topic = rospy.get_param('~img_topic', self.init_img_topic)
            bounding_boxes_msg.image_width = detect_dict['width_pixels']
            bounding_boxes_msg.image_height = detect_dict['height_pixels']
            bounding_boxes_msg.bounding_boxes = bounding_box_msg_list
            if not rospy.is_shutdown() and self.bounding_boxes_pub is not None:
                    self.bounding_boxes_pub.publish(bounding_boxes_msg)
        found_object_msg = ObjectCount()
        found_object_msg.header.stamp = ros_img_header.stamp
        found_object_msg.count = len(detect_dict_list)
        if not rospy.is_shutdown() and self.found_object_pub is not None:
                self.found_object_pub.publish(found_object_msg)



    def optimal_font_dims(self, img, font_scale = 2e-3, thickness_scale = 5e-3):
        h, w, _ = img.shape
        font_scale = min(w, h) * font_scale
        thickness = math.ceil(min(w, h) * thickness_scale)
        return font_scale, thickness
                    

    def get_classes_color_list(self,classes_str_list):
        rgb_list = []
        if len(classes_str_list) > 0:
            cmap = plt.get_cmap('viridis')
            color_list = cmap(np.linspace(0, 1, len(classes_str_list))).tolist()
            for color in color_list:
                rgb = []
                for i in range(3):
                    rgb.append(int(color[i]*255))
                rgb_list.append(rgb)
        return rgb_list

    def publishStatusCb(self,timer):
        self.publishStatus()


    def publishStatus(self):
        status_msg = ImageClassifierStatus()
        status_msg.classifier_name = self.model_name
        status_msg.classifier_state = self.state
        status_msg.classifier_enabled = self.enabled
        status_msg.classifier_classes = self.classes_list

        status_msg.img_topics = rospy.get_param('~img_topics', self.init_img_topics)
        status_msg.img_tiling = rospy.get_param('~img_tiling', self.init_img_tiling)
        status_msg.overlay_labels = rospy.get_param('~overlay_labels',self.init_overlay_labels)
        status_msg.overlay_clf_name = rospy.get_param('~overlay_clf_name', self.init_overlay_clf_name)
        status_msg.overlay_img_name = rospy.get_param('~overlay_img_name', self.init_overlay_img_name)
        status_msg.img_topic = rospy.get_param('~img_topic', self.init_img_topic)
        status_msg.img_connected = self.img_connected

        status_msg.threshold = rospy.get_param('~threshold', self.init_threshold)
        status_msg.max_rate_hz = rospy.get_param('~max_rate', self.init_max_rate)
        #nepi_msg.publishMsgWarn(self,"Sending Status Msg: " + str(status_msg))
        if not rospy.is_shutdown():
            self.status_pub.publish(status_msg)
            if self.enabled == True:
                if status_msg.img_topic == "None":
                    self.ros_no_img_topic_img.header.stamp = nepi_ros.time_now()
                    self.detection_image_pub.publish(self.ros_no_img_topic_img)
                elif self.img_connected == False:
                    self.ros_no_img_img.header.stamp = nepi_ros.time_now()
                    self.detection_image_pub.publish(self.ros_no_img_img)

                if self.mgr_registered == True:
                    self.status_mgr_pub.publish(status_msg)


