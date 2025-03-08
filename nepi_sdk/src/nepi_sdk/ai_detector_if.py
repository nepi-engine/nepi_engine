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
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes, AiModelInfo, AiDetectorStatus, AiDetectorsStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_img
from nepi_sdk import nepi_save

from nepi_sdk.save_data_if import SaveDataIF


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



class AiDetectorIF:

    MODEL_CONFIG_FOLDER = "/mnt/nepi_storage/user_cfg/ros" 
    
    data_products = ['bounding_boxes','detection_image']

    node_namespace = ""
    config_file_path = ""
    self_managed = True
    model_name = "None"

    img_connected = False

    img_acquire = False
    img_msg = None
    last_img_msg = None
    img_msg_topic = ""
    img_msg_header = None
    cv2_img = None
    img_lock = threading.Lock()

    last_time = time.time()
    imgs_pub_sub_dict = dict()
    imgs_pub_sub_lock = threading.Lock()
    img_info_dict = dict()


    init_selected_img_topics = []
    init_img_tiling = DEFAULT_IMG_TILING
    init_overlay_labels = DEFAULT_LABELS_OVERLAY
    init_overlay_clf_name = DEFAULT_CLF_OVERLAY
    init_overlay_img_name = DEFAULT_IMG_OVERLAY
    init_selected_img_topic = "None"
    init_threshold = DEFUALT_THRESHOLD
    init_max_rate = DEFAULT_MAX_RATE
    init_enabled = False


    save_cfg_if = None

    state = 'Loading'

    threshold_active_sub = None
    max_rate_active_sub = None
    set_img_topic_sub = None
    overlay_clf_name_active_sub = None
    overlay_img_name_active_sub = None
    reset_factory_active_sub = None

    detect_time = 0
    
    def __init__(self, model_name, framework, description, img_height, img_width,  classes_list, defualt_config_dict, all_namespace, processDetectionFunction):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ##############################  
        if all_namespace.find(self.node_name) == -1:
            self.self_managed = False
        # Create a message image to publish when not running
        message = ""
        cv2_img = nepi_img.create_message_image(message)
        self.ros_not_enabled_img = nepi_img.cv2img_to_rosimg(cv2_img) 
        message = "NOT ENABLED"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_blank_img = nepi_img.cv2img_to_rosimg(cv2_img) 
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
        self.model_framework = framework
        self.model_type = 'detection'
        self.model_description = description
        self.model_img_height = img_height
        self.model_img_width = img_width
        self.defualt_config_dict = defualt_config_dict
        if all_namespace[-1] == "/":
            all_namespace = all_namespace[:-1]
        self.all_namespace = all_namespace
        self.processDetection = processDetectionFunction

        self.classes_list = classes_list
        self.classes_color_list = self.get_classes_color_list(classes_list)
                 
        nepi_msg.publishMsgInfo(self,"Starting IF setup")

        # Load/Create model config params
        self.node_namespace = os.path.join(self.base_namespace,"ai",self.node_name)
        self.config_file_path = os.path.join(self.MODEL_CONFIG_FOLDER,self.node_name)
        nepi_ros.load_config_file(self.config_file_path, defualt_config_dict, self.node_namespace)
        nepi_msg.publishMsgWarn(self,"Loaded AI Detector params")
        nepi_ros.print_node_params(self)
        
        # Create AI Mgr Node Publishers

        self.found_object_pub = rospy.Publisher('~found_object', ObjectCount,  queue_size = 1)
        self.bounding_boxes_pub = rospy.Publisher('~bounding_boxes', BoundingBoxes, queue_size = 1)
        self.detection_image_pub = rospy.Publisher('~detection_image', Image,  queue_size = 1)
        self.detection_trigger_pub = rospy.Publisher('~detection_trigger', Bool,  queue_size = 1)
        self.detection_state_pub = rospy.Publisher('~detection_state', Bool,  queue_size = 1)

        self.all_namespace = os.path.join(self.base_namespace,'ai/all_detectors')
        nepi_msg.publishMsgInfo(self,"Staring all detectors on namespace " + self.all_namespace)
        self.found_object_all_pub = rospy.Publisher(self.all_namespace + '/found_object', ObjectCount,  queue_size = 1)
        self.bounding_boxes_all_pub = rospy.Publisher(self.all_namespace + '/bounding_boxes', BoundingBoxes, queue_size = 1)
        self.detection_image_all_pub = rospy.Publisher(self.all_namespace + '/detection_image', Image,  queue_size = 1)
        self.detection_trigger_all_pub = rospy.Publisher(self.all_namespace + '/detection_trigger', Bool,  queue_size = 1)
  

        self.status_pub = rospy.Publisher("~status", AiDetectorsStatus,  queue_size = 1, latch = True)

        time.sleep(1)

        # Create AI Node Subscribers
        rospy.Subscriber('~add_img_topic', String, self.addImageTopicCb, queue_size=10)
        rospy.Subscriber('~add_img_topics', StringArray, self.addImageTopicsCb, queue_size=10)
        rospy.Subscriber('~remove_img_topic', String, self.removeImageTopicCb, queue_size=10)
        rospy.Subscriber('~remove_img_topics', StringArray, self.removeImageTopicsCb, queue_size=10)
        rospy.Subscriber('~set_detection_img_topic', String, self.setDetectImageTopicCb, queue_size=10)
        #rospy.Subscriber('~set_image_tiling', Bool, self.setTileImgCb, queue_size=10)
        rospy.Subscriber('~set_overlay_labels', Bool, self.setOverlayLabelsCb, queue_size=10)
        rospy.Subscriber('~set_overlay_clf_name', Bool, self.setOverlayClfNameCb, queue_size=10)
        rospy.Subscriber('~set_overlay_img_name', Bool, self.setOverlayImgNameCb, queue_size=10)
        rospy.Subscriber('~set_threshold', Float32, self.setThresholdCb, queue_size=10)
        rospy.Subscriber('~set_max_rate', Float32, self.setMaxRateCb, queue_size=10)
        rospy.Subscriber('~reset_factory', Empty, self.resetFactoryCb, queue_size=10) # start local callback
        rospy.Subscriber('~save_config', Empty, self.saveConfigCb, queue_size=10) # start local callback
        rospy.Subscriber('~reset_config', Empty, self.resetConfigCb, queue_size=10) # start local callback
        rospy.Subscriber('~enable', Bool, self.enableCb, queue_size=10) # start local callback


        # Setup Data Saving
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'detection_image' in self.data_products:
            factory_data_rates['detection_image'] = [1.0, 0.0, 100.0] 
        self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)

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
        nepi_ros.set_param(self,'~img_topics', [])
        nepi_ros.set_param(self,'~selected_img_topic', "None")
        nepi_ros.set_param(self,'~img_tiling', DEFAULT_IMG_TILING)
        nepi_ros.set_param(self,'~overlay_labels', DEFAULT_LABELS_OVERLAY)
        nepi_ros.set_param(self,'~overlay_clf_name', DEFAULT_CLF_OVERLAY)
        nepi_ros.set_param(self,'~overlay_img_name', DEFAULT_IMG_OVERLAY)
        nepi_ros.set_param(self,'~threshold', self.defualt_config_dict['threshold'])
        nepi_ros.set_param(self,'~max_rate', self.defualt_config_dict['max_rate'])
        nepi_ros.set_param(self,'~enabled', False)

    def resetParamServer(self,do_updates = False):
        nepi_msg.publishMsgInfo(self,"Reseting Param Server")
        nepi_ros.set_param(self,'~img_topics', self.init_selected_img_topics)
        nepi_ros.set_param(self,'~selected_img_topic', self.init_selected_img_topic)
        nepi_ros.set_param(self,'~img_tiling', self.init_img_tiling)
        nepi_ros.set_param(self,'~overlay_labels',self.init_overlay_labels)
        nepi_ros.set_param(self,'~overlay_clf_name', self.init_overlay_clf_name)
        nepi_ros.set_param(self,'~overlay_img_name', self.init_overlay_img_name)
        nepi_ros.set_param(self,'~threshold', self.init_threshold)
        nepi_ros.set_param(self,'~max_rate', self.init_max_rate)
        nepi_ros.set_param(self,'~enabled', self.init_enabled)
        if do_updates:
            self.updateFromParamServer()

    def initializeParamServer(self,do_updates = False):
        nepi_msg.publishMsgInfo(self,"Initializing Param Server")
        self.init_selected_img_topics = rospy.get_param('~img_topics', [])
        self.init_selected_img_topic = rospy.get_param('~selected_img_topic', "None")
        self.init_img_tiling = rospy.get_param('~img_tiling', DEFAULT_IMG_TILING)
        self.init_overlay_labels = rospy.get_param('~overlay_labels', DEFAULT_LABELS_OVERLAY)
        self.init_overlay_clf_name = rospy.get_param('~overlay_clf_name', DEFAULT_CLF_OVERLAY)
        self.init_overlay_img_name = rospy.get_param('~overlay_img_name', DEFAULT_IMG_OVERLAY)
        self.init_threshold = rospy.get_param('~threshold', self.defualt_config_dict['threshold'])
        self.init_max_rate = rospy.get_param('~max_rate', self.defualt_config_dict['max_rate'])
        self.init_enabled = nepi_ros.get_param(self,'~enabled', Fasle)
        self.resetParamServer(do_updates)

    def updateFromParamServer(self):
        pass # Handled by config_if

    def addImageTopicCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received Add Image Topics: " + msg.data)
        img_topic = msg.data
        self.addImageTopic(img_topic)


    def addImageTopicsCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received Add Image Topics: " + str(msg))
        img_topic_list = msg.entries
        for img_topic in img_topic_list:
            self.addImageTopic(img_topic)


    def addImageTopic(self,img_topic):   
        img_topics = rospy.get_param('~img_topics', self.init_selected_img_topics)
        if img_topic not in img_topics:
            img_topics.append(img_topic)
        nepi_ros.set_param(self,'~img_topics', img_topics)
        self.publishStatus()

    def removeImageTopicCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received Remove Image Topics: " + str(msg))
        img_topic = msg.data
        self.removeImageTopic(img_topic)


    def removeImageTopicsCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received Remove Image Topic: " + str(msg))
        img_topic_list = msg.entries
        for img_topic in img_topic_list:
            self.removeImageTopic(img_topic)

    def removeImageTopic(self,img_topic):         
        img_topics = rospy.get_param('~img_topics', self.init_selected_img_topics)
        if img_topic in img_topics:
            img_topics.remove(img_topic)
        nepi_ros.set_param(self,'~img_topics', img_topics)
        self.publishStatus()



    def setDetectImageTopicCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received Set Detection Image Topic: " + msg.data)
        img_topic = msg.data
        nepi_ros.set_param(self,'~selected_img_topic', img_topic)
        self.publishStatus()

    def setTileImgCb(self,msg):
        nepi_ros.set_param(self,'~img_tiling', msg.data)
        self.publishStatus()

    def setOverlayLabelsCb(self,msg):
        nepi_ros.set_param(self,'~overlay_labels', msg.data)
        self.publishStatus()

    def setOverlayClfNameCb(self,msg):
        nepi_ros.set_param(self,'~overlay_clf_name', msg.data)
        self.publishStatus()

    def setOverlayImgNameCb(self,msg):
        nepi_ros.set_param(self,'~overlay_img_name', msg.data)
        self.publishStatus()


    def setThresholdCb(self,msg):
        threshold = msg.data
        nepi_msg.publishMsgInfo(self,"Received Threshold Update: " + str(threshold))
        if threshold <  MIN_THRESHOLD:
            threshold = MIN_THRESHOLD
        elif threshold > MAX_THRESHOLD:
            threshold = MAX_THRESHOLD
        nepi_ros.set_param(self,'~threshold', threshold)
        self.publishStatus()

    def setMaxRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        nepi_ros.set_param(self,'~max_rate', max_rate)
        self.publishStatus()

    def enableCb(self,msg):
        enabled = msg.data
        nepi_ros.set_param(self,'~enabled', enabled)
        self.publishStatus()
        time.sleep(1)
        if msg.data == False and not rospy.is_shutdown():
            self.ros_blank_img.header.stamp = nepi_ros.time_now()
            self.detection_image_pub.publish(self.ros_blank_img)
            self.detection_image_all_pub.publish(self.ros_blank_img)


    def statusPublishCb(self,timer):
        self.publishStatus()

            

    def subscribeImgTopic(self,img_topic):
        if img_topic == "None" or img_topic == "":
            return False
        else:
            nepi_msg.publishMsgInfo(self,'Subsribing to image topic: ' + img_topic)
            #nepi_msg.publishMsgWarn(self,'have base namespace: ' + self.base_namespace)
            img_name = img_topic.replace(self.base_namespace,"")
            #nepi_msg.publishMsgWarn(self,'Subsribing to image name: ' + img_name)
            pub_sub_namespace = os.path.join(self.node_namespace,img_name)
            nepi_msg.publishMsgWarn(self,'Publishing to image topic: ' + pub_sub_namespace)
            found_object_pub = self.found_object_pub = rospy.Publisher(pub_sub_namespace + '/found_object', ObjectCount,  queue_size = 1)
            bounding_box_pub = self.bounding_boxes_pub = rospy.Publisher(pub_sub_namespace + '/bounding_boxes', BoundingBoxes, queue_size = 1)
            detection_image_pub = rospy.Publisher(pub_sub_namespace + '/detection_image', Image,  queue_size = 1)
            detection_trigger_pub = rospy.Publisher(pub_sub_namespace + '/detection_trigger', Bool,  queue_size = 1)
            detection_state_pub = rospy.Publisher(pub_sub_namespace + '/detection_state', Bool,  queue_size = 1)
            status_pub = rospy.Publisher(pub_sub_namespace + '/status', AiDetectorStatus,  queue_size = 1, latch = True)
            img_sub = img_sub = rospy.Subscriber(img_topic, Image, self.imageCb, queue_size=1, callback_args=(img_topic))
            time.sleep(1)
            self.img_info_dict[img_topic] = dict()  
            self.img_info_dict[img_topic]['connected'] = False  
            self.img_info_dict[img_topic]['detect_time'] = 0  
            self.imgs_pub_sub_lock.acquire()
            self.imgs_pub_sub_dict[img_topic] = {'img_sub': img_sub,
                                            'pub_sub_namespace': pub_sub_namespace,
                                            'found_object_pub': found_object_pub,
                                            'bounding_box_pub': bounding_box_pub,
                                            'detection_image_pub': detection_image_pub,
                                            'detection_trigger_pub': detection_trigger_pub,
                                            'detection_state_pub': detection_state_pub,
                                            'status_pub': status_pub,
                                            }
            
            self.imgs_pub_sub_lock.release()
            nepi_msg.publishMsgWarn(self,'Registered : ' + img_topic +  ' ' + str(self.imgs_pub_sub_dict[img_topic]))
            time.sleep(1)
            self.ros_no_img_img.header.stamp = nepi_ros.time_now()
            detection_image_pub.publish(self.ros_no_img_img)
            found_object_pub.publish(ObjectCount())
            bounding_box_pub.publish(BoundingBoxes())

            return True
        

    def unsubscribeImgTopic(self,img_topic):
        self.imgs_pub_sub_lock.acquire()
        if img_topic in self.imgs_pub_sub_dict.keys():
            nepi_msg.publishMsgWarn(self,'Unregistering image topic: ' + img_topic)
            img_pub_sub_dict = self.imgs_pub_sub_dict[img_topic]
            del self.imgs_pub_sub_dict[img_topic]
            for pub_sub_name in img_pub_sub_dict.keys():
                if pub_sub_name != 'pub_sub_namespace':
                    nepi_msg.publishMsgWarn(self,'Unregistering topic: ' + pub_sub_name)
                    img_pub_sub_dict[pub_sub_name].unregister

            del self.img_info_dict[img_topic]
        self.imgs_pub_sub_lock.release()
        return True

            


    def updaterCb(self,timer):
        enabled = nepi_ros.get_param(self,'~enabled', self.init_enabled)
        #nepi_msg.publishMsgWarn(self,"Updating with image topic: " +  self.img_topic)
        img_topics = rospy.get_param('~img_topics', self.init_selected_img_topics)
        self.imgs_pub_sub_lock.acquire()
        imgs_pub_sub_keys = self.imgs_pub_sub_dict.keys()
        self.imgs_pub_sub_lock.release()
        for i,img_topic in enumerate(img_topics):
            img_topic = nepi_ros.find_topic(img_topic)
            if img_topic != '':
                img_topics[i] = img_topic
                if img_topic not in imgs_pub_sub_keys:
                    success = self.subscribeImgTopic(img_topic)          
        purge_list = []
        for img_topic in imgs_pub_sub_keys:
            if img_topic not in img_topics:
                purge_list.append(img_topic)
        #nepi_msg.publishMsgWarn(self,'Purging image topics: ' + str(purge_list))
        for topic in purge_list:
            nepi_msg.publishMsgWarn(self,'Will unsubscribe topics: ' + topic)
            success = self.unsubscribeImgTopic(topic)

        if len(imgs_pub_sub_keys) == 0:
            self.img_connected = False

        if enabled == True and len(imgs_pub_sub_keys) > 0:
            if self.img_connected == True:
                self.state = "Running"
            else:
                self.state = "Waiting"
        else:
            self.state = "Loaded"
                 
        rospy.Timer(rospy.Duration(.01), self.updaterCb, oneshot = True)



    def imageCb(self,image_msg, args):  
        self.img_connected = True
        img_topic = args 
        sel_img_topic = rospy.get_param('~selected_img_topic', self.init_selected_img_topic)
        enabled = nepi_ros.get_param(self,'~enabled', self.init_enabled)
        self.imgs_pub_sub_lock.acquire()
        imgs_pub_sub_keys = self.imgs_pub_sub_dict.keys()
        self.imgs_pub_sub_lock.release()
        if img_topic != 'None' and img_topic != 'All' and img_topic in imgs_pub_sub_keys:
             self.img_info_dict[img_topic]['connected'] = True
        get_image = enabled and ((img_topic == sel_img_topic or sel_img_topic == 'All'))
        if get_image == True:
            self.img_lock.acquire()
            self.img_msg = image_msg
            self.img_msg_header = image_msg.header
            self.img_msg_topic = img_topic
            self.cv2_img = nepi_img.rosimg_to_cv2img(image_msg)
            self.img_lock.release()
            #self.last_img_msg = copy.deepcopy(image_msg)


    def updateDetectionCb(self,timer):
        enabled = nepi_ros.get_param(self,'~enabled', self.init_enabled)
        if enabled == True:
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
                ros_img_header = self.img_msg_header
                img_topic = self.img_msg_topic    
                self.img_msg_topic = ""

                cv2_img = copy.deepcopy(self.cv2_img) 
                self.cv2_img = None # Clear the last image   

                self.img_lock.release()
                img_topics = rospy.get_param('~img_topics', self.init_selected_img_topics)
                img_not_none = (cv2_img is not None) and (img_topic in img_topics)
                #nepi_msg.publishMsgWarn(self,'Got Image not None: ' + str(img_not_none))
                if img_not_none == True:
                    cv2_shape = cv2_img.shape
                    img_width = cv2_shape[1] 
                    img_height = cv2_shape[0] 
                    if nepi_img.is_gray(cv2_img) == True:
                        cv2_img = nepi_img.grayscale_to_rgb(cv2_img)
                    try:
                        [detect_dict_list, detect_time] = self.processDetection(cv2_img,threshold) 
                        self.detect_time = detect_time
                        self.img_info_dict[img_topic]['detect_time'] = detect_time
                        #nepi_msg.publishMsgInfo(self,"AIF got back detect_dict: " + str(detect_dict_list))
                        success = True
                    except Exception as e:
                        self.img_msg = None
                        nepi_msg.publishMsgWarn(self,"Failed to process detection img with exception: " + str(e))

                    if detect_dict_list is not None:
                        # Publish image first for consumers
                        if len(detect_dict_list) > 0:
                            #nepi_msg.publishMsgWarn(self,"Starting detect image: " + str(cv2_img.shape))
                            cv2_detect_img = self.apply_detection_overlay(img_topic, detect_dict_list,cv2_img)
                        else:
                            cv2_detect_img = cv2_img
                        #nepi_msg.publishMsgWarn(self,"Return detect image: " + str(cv2_detect_img.shape))
                        detect_img_msg = nepi_img.cv2img_to_rosimg(cv2_detect_img, encoding="bgr8")
                        detect_img_msg.header.stamp = nepi_ros.time_now()
                        if not rospy.is_shutdown() and self.detection_image_pub is not None:
                            self.detection_image_pub.publish(detect_img_msg)
                            self.detection_image_all_pub.publish(detect_img_msg)

                            if img_topic in self.imgs_pub_sub_dict.keys():
                                img_dict = self.imgs_pub_sub_dict[img_topic]
                                #nepi_msg.publishMsgWarn(self,"Got Img Dict: " + str(img_dict))
                                detection_image_pub = img_dict['detection_image_pub']
                                detection_image_pub.publish(detect_img_msg)
                        self.publishDetectionData(img_topic, detect_dict_list,ros_img_header)


                        # Save Image Data if needed
                        data_product = 'detection_image'
                        image_text = img_topic.replace(self.base_namespace,"")
                        image_text = image_text.replace('/idx',"")
                        image_text = image_text.replace('/','_')
                        ros_timestamp = ros_img_header.stamp
                        nepi_save.save_ros_img2file(self,data_product,img_in_msg,ros_timestamp, add_text = image_text)
                    else:
                        nepi_ros.signal_shutdown("Something went wrong in detection process call")
                        nepi_ros.sleep(2)

        rospy.Timer(rospy.Duration(.01), self.updateDetectionCb, oneshot = True)


    def getImgShortName(self, img_topic):
        short_name = img_topic.replace(self.base_namespace,"")
        if short_name.find("idx") != -1:
            short_name = short_name.replace("/idx","")
        return short_name
   

    def apply_detection_overlay(self,img_topic, detect_dict_list,cv2_img):
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
                        ## Overlay Detector Name
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
                    img_topic = img_topic
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

    def publishDetectionData(self,img_topic, detect_dict_list,ros_img_header):

        if len(detect_dict_list) > 0:
            bounding_box_msg_list = []
            for detect_dict in detect_dict_list:
                try:
                    bb_msg = BoundingBox()
                    bb_msg.Class = detect_dict['name']
                    bb_msg.id = detect_dict['id']
                    bb_msg.uid = detect_dict['uid']
                    bb_msg.probability = detect_dict['prob']
                    bb_msg.xmin = detect_dict['xmin']
                    bb_msg.ymin = detect_dict['ymin']
                    bb_msg.xmax = detect_dict['xmax']
                    bb_msg.ymax = detect_dict['ymax']
                    area_pixels = (detect_dict['xmax'] - detect_dict['xmin']) * (detect_dict['ymax'] - detect_dict['ymin'])
                    img_area = detect_dict['width_pixels'] * detect_dict['height_pixels']
                    if img_area > 1:
                        area_ratio = area_pixels / img_area
                    else:
                        area_ratio = -999
                    bb_msg.area_pixels = detect_dict['area_pixels']
                    bb_msg.area_ratio = detect_dict['area_ratio']
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to get all data from detect dict: " + str(e)) 
                bounding_box_msg_list.append(bb_msg)
            bbs_msg = BoundingBoxes()
            bbs_msg.header.stamp = ros_img_header.stamp
            bbs_msg.model_name = self.node_name
            bbs_msg.image_header = ros_img_header
            bbs_msg.image_topic = img_topic
            bbs_msg.image_width = detect_dict['width_pixels']
            bbs_msg.image_height = detect_dict['height_pixels']
            bbs_msg.bounding_boxes = bounding_box_msg_list
            if not rospy.is_shutdown() and self.bounding_boxes_pub is not None:
                    self.bounding_boxes_pub.publish(bbs_msg)
                    self.bounding_boxes_all_pub.publish(bbs_msg)
                    self.detection_trigger_pub.publish()
                    self.detection_trigger_all_pub.publish()                                
                    if img_topic in self.imgs_pub_sub_dict.keys():
                        img_dict = self.imgs_pub_sub_dict[img_topic]
                        bounding_box_pub = img_dict['bounding_box_pub']
                        bounding_box_pub.publish(bbs_msg)
                        detection_trigger_pub = img_dict['detection_trigger_pub']
                        detection_trigger_pub.publish()
                        detection_state_pub = img_dict['detection_state_pub']
                        detection_state_pub.publish(True)
                        
        found_object_msg = ObjectCount()
        found_object_msg.header.stamp = ros_img_header.stamp
        found_object_msg.model_name = self.node_name
        found_object_msg.image_header = ros_img_header
        found_object_msg.header.stamp = ros_img_header.stamp
        count = len(detect_dict_list)
        found_object_msg.count = count
        if not rospy.is_shutdown() and self.found_object_pub is not None:
            self.found_object_pub.publish(found_object_msg)
            self.found_object_all_pub.publish(found_object_msg)
            if img_topic in self.imgs_pub_sub_dict.keys():
                img_dict = self.imgs_pub_sub_dict[img_topic]
                found_object_pub = img_dict['found_object_pub']
                found_object_pub.publish(found_object_msg)
                if count == 0:
                    self.detection_state_pub.publish(False)
                    if img_topic in self.imgs_pub_sub_dict.keys():
                        img_dict = self.imgs_pub_sub_dict[img_topic]
                        detection_state_pub = img_dict['detection_state_pub']
                        detection_state_pub.publish(False)

        # Save Bounding Data if needed
        image_text = img_topic.replace(self.base_namespace,"")
        image_text = image_text.replace('/idx',"")
        image_text = image_text.replace('/','_')
        if len(detect_dict_list) > 0:

            data_product = 'bounding_boxes'

            ros_timestamp = bbs_msg.header.stamp
            bbs_dict = dict()
            bbs_dict['timestamp'] =  nepi_ros.get_datetime_str_from_stamp(bbs_msg.header.stamp)
            bbs_dict['model_name'] = bbs_msg.model_name
            bbs_dict['image_topic'] = bbs_msg.image_topic
            bbs_dict['image_height'] = bbs_msg.image_height
            bbs_dict['image_width'] = bbs_msg.image_width

            bb_list = []
            for ind, bb_msg in enumerate(bbs_msg.bounding_boxes):
                bb_dict = dict()
                bb_dict['class'] = bb_msg.Class
                bb_dict['id'] = bb_msg.id
                bb_dict['uid'] = bb_msg.uid
                bb_dict['probability'] = bb_msg.probability
                bb_dict['xmin'] = bb_msg.xmin
                bb_dict['ymin'] = bb_msg.ymin
                bb_dict['xmax'] = bb_msg.xmax
                bb_dict['ymax'] = bb_msg.ymax
                bb_dict['area_pixels'] = bb_msg.area_pixels
                bb_dict['area_ratio'] = bb_msg.area_ratio
                bb_list.append(bb_dict)
            bbs_dict['bounding_boxes'] = bb_list
            nepi_save.save_dict2file(self,data_product,bbs_dict,ros_timestamp,add_text = image_text)

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




    def getModelInfo(self):
        info_msg = AiModelInfo()
        info_msg.name = self.model_name
        info_msg.framework = self.model_framework
        info_msg.type = self.model_type
        info_msg.description = self.model_description
        info_msg.img_height = self.model_img_height
        info_msg.img_width = self.model_img_width
        info_msg.classes = self.classes_list
        return info_msg
    

    def publishStatusCb(self,timer):
        self.publishStatus()

    def publishStatus(self):
        enabled = nepi_ros.get_param(self,'~enabled', self.init_enabled)
  
       
        status_msg = AiDetectorsStatus()
        status_msg.model_name = self.model_name
        status_msg.model_info = self.getModelInfo()

        status_msg.namespace = self.node_namespace
        status_msg.state = self.state
        status_msg.enabled = enabled

        img_source_topics = []
        img_det_namespaces = []
        self.imgs_pub_sub_lock.acquire()
        for img_topic in self.imgs_pub_sub_dict.keys():
            img_source_topics.append(img_topic)
            img_det_namespaces.append(self.imgs_pub_sub_dict[img_topic]['pub_sub_namespace'])
        self.imgs_pub_sub_lock.release()

        status_msg.image_source_topics = img_source_topics
        status_msg.image_detect_namespaces = img_det_namespaces
        status_msg.images_connected = self.img_connected

        selected_img_topic = rospy.get_param('~selected_img_topic', self.init_selected_img_topic)
        status_msg.selected_image_topic = selected_img_topic
        status_msg.img_tiling = rospy.get_param('~img_tiling', self.init_img_tiling)
        status_msg.overlay_labels = rospy.get_param('~overlay_labels',self.init_overlay_labels)
        status_msg.overlay_clf_name = rospy.get_param('~overlay_clf_name', self.init_overlay_clf_name)
        status_msg.overlay_img_name = rospy.get_param('~overlay_img_name', self.init_overlay_img_name)


        status_msg.threshold = rospy.get_param('~threshold', self.init_threshold)
        status_msg.max_rate_hz = rospy.get_param('~max_rate', self.init_max_rate)

        status_msg.detect_time = self.detect_time

        #nepi_msg.publishMsgWarn(self,"Sending Status Msg: " + str(status_msg))
        if not rospy.is_shutdown():
            self.status_pub.publish(status_msg)

        #nepi_msg.publishMsgWarn(self,"Sending Status Msg: " + str(status_msg))
        if not rospy.is_shutdown():
            if enabled == True:
                if selected_img_topic == "None":
                    self.ros_no_img_topic_img.header.stamp = nepi_ros.time_now()
                    self.detection_image_pub.publish(self.ros_no_img_topic_img)
                    self.detection_image_all_pub.publish(self.ros_no_img_topic_img)
                elif self.img_connected == False:
                    self.ros_no_img_img.header.stamp = nepi_ros.time_now()
                    self.detection_image_pub.publish(self.ros_no_img_img)
                    self.detection_image_all_pub.publish(self.ros_no_img_img)
            else:
                self.ros_not_enabled_img.header.stamp = nepi_ros.time_now()
                self.detection_image_pub.publish(self.ros_not_enabled_img)

        # Send individual img detector status messages
        self.imgs_pub_sub_lock.acquire()
        for img_topic in self.imgs_pub_sub_dict.keys():
            img_pub_sub_dict = self.imgs_pub_sub_dict[img_topic]
            #nepi_msg.publishMsgWarn(self,"Got img pub sub dict keys: " + str(img_pub_sub_dict.keys()))
            img_detector_namespace = img_pub_sub_dict['pub_sub_namespace']
            img_status_pub = img_pub_sub_dict['status_pub']
            img_connected = self.img_info_dict[img_topic]['connected']
            img_detect_time = self.img_info_dict[img_topic]['detect_time']
            status_msg = AiDetectorStatus()
            status_msg.model_name = self.model_name
            status_msg.model_info = self.getModelInfo()

            status_msg.namespace = img_detector_namespace
            status_msg.enabled = enabled
            status_msg.state = self.state

            status_msg.image_source_topic = img_topic
            status_msg.image_connected = img_connected

            status_msg.img_tiling = rospy.get_param('~img_tiling', self.init_img_tiling)
            status_msg.overlay_labels = rospy.get_param('~overlay_labels',self.init_overlay_labels)
            status_msg.overlay_clf_name = rospy.get_param('~overlay_clf_name', self.init_overlay_clf_name)
            status_msg.overlay_img_name = rospy.get_param('~overlay_img_name', self.init_overlay_img_name)


            status_msg.threshold = rospy.get_param('~threshold', self.init_threshold)
            status_msg.max_rate_hz = rospy.get_param('~max_rate', self.init_max_rate)

            status_msg.detect_time = img_detect_time
            #nepi_msg.publishMsgWarn(self,"Sending Img Status Msg: " + str(status_msg))
            if not rospy.is_shutdown():
                img_status_pub.publish(status_msg)
        self.imgs_pub_sub_lock.release()


