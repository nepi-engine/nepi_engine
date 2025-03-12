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
import sys
import copy
import time
import numpy as np
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2

from std_msgs.msg import UInt8, Float32, Bool, Empty, String
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes, AiDetectorStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, AiDetectorInfoQuery, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_ais
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


class AiDetectorIF:

    data_products = ['bounding_boxes','detection_image']

    det_dict = dict()

    dets_pub_sub_dict = dict()
    dets_pub_sub_lock = threading.Lock()
    dets_info_dict = dict()
    imgs_lock_dict = dict()

    img_dict = None
    img_dict_lock = threading.Lock()

    first_detect_complete = False

    save_cfg_if = None

    

    defualt_config_dict = {'threshold': 0.3,'max_rate': 5}

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "detector_img" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_namespace = self.base_namespace + self.node_name
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
        ##############################

        self.detector_namespace = nepi_ros.get_param(self,"~detector_namespace","")
        if self.detector_namespace = "":
            sys.exit(-1)
        else:
            if self.detector_namespace[-1] == '/':
                self.detector_namespace[0:-1]
            nepi_msg.publishMsgInfo(self,"Connecting to Detector: " + self.detector_namespace)
            success = nepi_ais.register_detector(self,self.detector_namespace)
            if success:
                # Create a message image to publish when not running
                message = ""
                cv2_img = nepi_img.create_message_image(message)
                self.ros_not_enabled_img = nepi_img.cv2img_to_rosimg(cv2_img) 
                message = "NOT ENABLED"
                cv2_img = nepi_img.create_message_image(message)
                self.ros_blank_img = nepi_img.cv2img_to_rosimg(cv2_img) 
                message = "IMAGE SOURCE NOT SELECTED"
                cv2_img = nepi_img.create_message_image(message)
                self.ros_no_det_namespace_img = nepi_img.cv2img_to_rosimg(cv2_img)
                message = "WAITING FOR IMAGE STREAM"
                cv2_img = nepi_img.create_message_image(message)
                self.ros_no_img_img = nepi_img.cv2img_to_rosimg(cv2_img)

                # Create Class Colors
                classes_list = self.detector_info.classes
                self.classes_color_list = self.get_classes_color_list(classes_list)
          
                # Create Publishers
                self.detection_image_pub = rospy.Publisher(self.detector_namespace + '/detection_image', Image,  queue_size = 1)
                self.all_namespace = os.path.join(self.base_namespace,'ai/all_detectors')
                self.detection_image_all_pub = rospy.Publisher(self.all_namespace + '/detection_image', Image,  queue_size = 1)
                time.sleep(1)

                # Setup Data Saving
                factory_data_rates= {}
                for d in self.data_products:
                    factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
                if 'detection_image' in self.data_products:
                    factory_data_rates['detection_image'] = [1.0, 0.0, 100.0] 
                self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)

                rospy.Timer(rospy.Duration(.1), self.updaterCb, oneshot = True)
                rospy.Timer(rospy.Duration(.1), self.processImgageCb, oneshot = True)
                
                nepi_msg.publishMsgInfo(self,"Initialization Complete")
        

            

    def subscribeDetNamespace(self,det_namespace):
        if det_namespace == "None" or det_namespace == "":
            return False
        else:
            nepi_msg.publishMsgWarn(self,'Pub Sub to detector namespace: ' + det_namespace)
            detection_image_pub = rospy.Publisher(det_namespace + '/detection_image', Image,  queue_size = 1)
            found_object_sub = rospy.Subscriber(det_namespace + '/found_object', ObjectCount,  self.foundObjectCb, queue_size=1, callback_args=(det_namespace))
            bounding_box_sub = rospy.Subscriber(det_namespace + '/bounding_boxes', BoundingBoxes, self.boundingBoxCb, queue_size = 1, callback_args=(det_namespace))

            ns_ind = self.image_detect_namespaces.index(det_namespace)
            img_topic = self.image_source_topics[ns_ind]
            nepi_msg.publishMsgWarn(self,'Subsribing to image topic: ' + image_topic)
            source_img_sub = rospy.Subscriber(img_topic, Image, self.imageCb, queue_size=1, callback_args=(det_namespace))
            time.sleep(1)


            # Create img info dict
            self.img_dict[img_topic] = det_namespace
            self.dets_info_dict[det_namespace] = dict()  
            self.dets_info_dict[det_namespace]['det_connected'] = False
            self.dets_info_dict[det_namespace]['img_connected'] = False 
            self.dets_info_dict[det_namespace]['last_detection'] = []
            self.dets_info_dict[det_namespace]['cur_detection'] = []

            self.imgs_lock_dict[det_namespace] = threading.Lock()
       
            # Create imag sub pub dict
            self.dets_pub_sub_lock.acquire()
            self.dets_pub_sub_dict[det_namespace] = {'source_image_sub': source_img_sub,
                                            'found_object_sub': found_object_sub,
                                            'bounding_box_sub': bounding_box_sub,
                                            'detection_image_pub': detection_image_pub,
                                            }
            
            self.dets_pub_sub_lock.release()
            nepi_msg.publishMsgWarn(self,'Registered : ' + det_namespace +  ' ' + str(self.dets_pub_sub_dict[det_namespace]))
            time.sleep(1)
            self.ros_no_img_img.header.stamp = nepi_ros.ros_time_now()
            detection_image_pub.publish(self.ros_no_img_img)
            return True
        

    def unsubscribeDetNamespace(self,det_namespace):
        self.dets_pub_sub_lock.acquire()
        if det_namespace in self.dets_pub_sub_dict.keys():
            nepi_msg.publishMsgWarn(self,'Unregistering detector: ' + det_namespace)
            det_pub_sub_dict = self.dets_pub_sub_dict[det_namespace]
            del self.dets_pub_sub_dict[det_namespace]
            for pub_sub_name in det_pub_sub_dict.keys():
                    nepi_msg.publishMsgWarn(self,'Unregistering detector: ' + pub_sub_name)
                    det_pub_sub_dict[pub_sub_name].unregister
            if det_namespace in self.dets_info_dict.keys():
                del self.dets_info_dict[det_namespace]
        self.dets_pub_sub_lock.release()
        return True

            
    def updaterCb(self,timer):
        purge_list = []
        for det_namespace in self.det_dict.keys():
            if det_namespace not in self.image_detect_namespaces:
                purge_list.append(det_namespace)
        for det_namespace in purge_list:
            self.unsubscribeDetNamespace(det_namespace)
        for det_namespace in self.image_detect_namespaces:
            if det_namespace not in self.det_dict.keys():
                self.subscribeDetNamespace(det_namespace)

      
        det_namespaces = self.image_detect_namespaces
        self.dets_pub_sub_lock.acquire()
        dets_pub_sub_keys = self.dets_pub_sub_dict.keys()
        self.dets_pub_sub_lock.release()
        for det_namespace in det_namespaces:
            if det_namespace not in dets_pub_sub_keys:
                nepi_msg.publishMsgWarn(self,'Will register detect namespace: ' + namespace)
                success = self.subscribeDetNamespace(det_namespace)          
        purge_list = []
        for det_namespace in dets_pub_sub_keys:
            if det_namespace not in det_namespaces:
                purge_list.append(det_namespace)
        #nepi_msg.publishMsgWarn(self,'Purging image topics: ' + str(purge_list))
        for namespace in purge_list:
            nepi_msg.publishMsgWarn(self,'Will unregister detect namespace: ' + namespace)
            success = self.unsubscribeDetNamespace(namespace)

        dets_info_dict = copy.deepcopy(self.dets_info_dict)
        img_connects = []
        for det_namespace in dets_info_dict.keys():
            img_connects.append(dets_info_dict[det_namespace]['img_connected'])
        img_selected = len(img_connects) > 0
        img_connected = True in img_connects
       
        if not rospy.is_shutdown():
            if self.enabled == True:
                if img_selected == 0:
                    self.ros_no_det_namespace_img.header.stamp = nepi_ros.ros_time_now()
                    self.detection_image_pub.publish(self.ros_no_det_namespace_img)
                    self.detection_image_all_pub.publish(self.ros_no_det_namespace_img)
                elif img_connected == False:
                    self.ros_no_img_img.header.stamp = nepi_ros.ros_time_now()
                    self.detection_image_pub.publish(self.ros_no_img_img)
                    self.detection_image_all_pub.publish(self.ros_no_img_img)
            else: # Loaded, but not enabled
                self.ros_not_enabled_img.header.stamp = nepi_ros.ros_time_now()
                self.detection_image_pub.publish(self.ros_not_enabled_img)

        rospy.Timer(rospy.Duration(.1), self.updaterCb, oneshot = True)



    def imageCb(self,image_msg, args):      
        start_time = nepi_ros.get_time_sec()   
        det_namespace = args 
        if det_namespace in self.dets_info_dict.keys():
            self.dets_info_dict[det_namespace]['img_connected'] = True
            enabled = nepi_ros.get_param(self,'~detector/enabled', self.init_enabled)
            if enabled == True:
                get_image = (det_namespace == self.get_det_namespace)
                #nepi_msg.publishMsgWarn(self,"Callback got image from topic:  " + det_namespace + " with get topic " + self.get_det_namespace)
                if get_image == True:
                    self.get_det_namespace = "None"

                    #nepi_msg.publishMsgWarn(self,"Processing img for topic:  " + det_namespace)
                    ##############################
                    ### Preprocess Image
                    
                    options_dict = dict()
                    options_dict['tile'] = nepi_ros.get_param(self,'~detector/img_tiling', self.init_img_tiling)
                    cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                    '''
                    img_dict = dict()
                    img_dict['cv2_img'] = cv2_img
                    '''
                    img_dict = self.preprocessImage(cv2_img, options_dict)
                    img_dict['ros_det_namespace'] = det_namespace
                    img_dict['ros_img_header'] = image_msg.header
                    img_dict['ros_img_stamp'] = image_msg.header.stamp
                    ##############################

                    self.img_dict_lock.acquire()
                    self.img_dict = img_dict
                    self.img_dict_lock.release()
                    self.got_detect_det_namespace = det_namespace
                    self.got_post_det_namespace = det_namespace

                    preprocess_time = round( (nepi_ros.get_time_sec() - start_time) , 3)
                    self.dets_info_dict[det_namespace]['preprocess_time'] = preprocess_time
            


    def updateDetectionTopicCb(self,timer):
        start_time = nepi_ros.get_time_sec()
        enabled = nepi_ros.get_param(self,'~detector/enabled', self.init_enabled)
        if enabled == True:
            det_namespaces = nepi_ros.get_param(self,'~detector/det_namespaces', self.init_selected_det_namespaces)
            connected_list = []
            for topic in det_namespaces:
                if topic in self.dets_info_dict.keys():
                    if self.dets_info_dict[topic]['img_connected'] == True:
                        connected_list.append(topic)
            if len(connected_list) == 0:
                self.get_det_namespace = "None"
            else:
                # check timer
                max_rate = nepi_ros.get_param(self,'~detector/max_rate', self.init_max_rate)
                delay_time = float(1) / max_rate 
                current_time = nepi_ros.get_time_sec()
                timer = round((current_time - self.last_detect_time), 3)
                #nepi_msg.publishMsgWarn(self,"Delay and Timer: " + str(delay_time) + " " + str(timer))

                # Get image topic info
                det_namespace = self.cur_det_namespace
                got_detect_det_namespace = self.got_detect_det_namespace

                # Setup Next Img if needed
                num_connected_list = len(connected_list)
                if num_connected_list > 0:
                    if det_namespace in connected_list:
                        next_img_ind = connected_list.index(det_namespace) + 1
                        if next_img_ind >= num_connected_list:
                            next_det_namespace = connected_list[0]
                        else:
                            next_det_namespace = connected_list[next_img_ind]
                    else:
                        next_det_namespace = connected_list[0]
                else:
                    next_det_namespace = "None"
                #nepi_msg.publishMsgWarn(self,"Next Image Topic set to: " + next_det_namespace)

                # Check if current image topic is None
                if det_namespace == "None" and next_det_namespace != "None":
                    self.get_det_namespace = next_det_namespace
                    self.cur_det_namespace = next_det_namespace
                    self.last_detect_time = nepi_ros.get_time_sec()

                ##############################
                # Check for non responding image streams                   
                if timer > (delay_time + GET_IMAGE_TIMEOUT_SEC):
                    #nepi_msg.publishMsgWarn(self,"Topic " + det_namespace + " timed out. Setting next topic to: " +  next_det_namespace)
                    if det_namespace is not None and det_namespace in self.dets_info_dict.keys():
                        self.dets_info_dict[det_namespace]['img_connected'] = False
                    self.get_det_namespace = next_det_namespace
                    self.cur_det_namespace = next_det_namespace
                    self.last_detect_time = nepi_ros.get_time_sec()

                elif timer > delay_time: 
                    #nepi_msg.publishMsgWarn(self,"Setting next topic to: " +  next_det_namespace)
                    self.cur_det_namespace = next_det_namespace
                    self.get_det_namespace = next_det_namespace

                    #nepi_msg.publishMsgWarn(self,"Timer over delay check, looking for image topic: " +  det_namespace)
                    if det_namespace != "None" and det_namespace in connected_list and det_namespace == got_detect_det_namespace :

                        #nepi_msg.publishMsgWarn(self,"Got image topic: " +  det_namespace)
                        self.got_detect_det_namespace = "None"
                        self.last_detect_time = nepi_ros.get_time_sec()

                        # Process got image
                        #nepi_msg.publishMsgWarn(self,"Copying img_dict from topic callback:  " + det_namespace)
                        self.img_dict_lock.acquire()
                        img_dict = copy.deepcopy(self.img_dict)
                        self.img_dict_lock.release()
                        #nepi_msg.publishMsgWarn(self,"Copying img_dict from topic callback:  " + det_namespace)




                        if img_dict is None:
                            nepi_msg.publishMsgWarn(self,"Callback provided None img_dict, :  " + det_namespace)
                        else:
                            if img_dict['cv2_img'] is None:
                                nepi_msg.publishMsgWarn(self,"Callback provided None cv2_img, :  " + det_namespace)
                                pass
                            else:
                                #nepi_msg.publishMsgWarn(self,"Detector got img_dict from topic callback:  " + det_namespace + " with img size: " + str(img_dict['cv2_img'].shape[:2]))

                                ##############################
                                # Process Detections
                                detect_dict_list = []
                                try:
                                    threshold = nepi_ros.get_param(self,'~detector/threshold', self.init_threshold)
                                    detect_dict_list = self.processDetection(img_dict,threshold) 
                                    #nepi_msg.publishMsgWarn(self,"AIF got back detect_dict: " + str(detect_dict_list))
                                    success = True
                                    self.first_detect_complete = True
                                except Exception as e:
                                    nepi_msg.publishMsgWarn(self,"Failed to process detection img with exception: " + str(e))
                                ros_timestamp = img_dict['ros_img_stamp']
                                self.publishDetectionData(det_namespace,detec_dict_list,ros_timestamp)
                                
                                self.dets_info_dict[det_namespace]['cur_detection'] = detect_dict_list
                                self.dets_info_dict[det_namespace]['last_detection'] = detect_dict_list



                            detect_time = round( (nepi_ros.get_time_sec() - start_time) , 3)
                            self.dets_info_dict[det_namespace]['detect_time'] = detect_time
                            #nepi_msg.publishMsgInfo(self,"Detect Time: {:.2f}".format(detect_time))

                        current_time = nepi_ros.ros_time_now()
                        ros_timestamp = img_dict['ros_img_stamp']
                        latency = (current_time.to_sec() - ros_timestamp.to_sec())
                        self.dets_info_dict[det_namespace]['detect_latency_time'] = latency
                        #nepi_msg.publishMsgInfo(self,"Detect Pub Latency: {:.2f}".format(latency))
                    
                                
        rospy.Timer(rospy.Duration(0.001), self.updateDetectionTopicCb, oneshot = True)



    def processImgageCb(self,timer):
        start_time = nepi_ros.get_time_sec()
        enabled = nepi_ros.get_param(self,'~detector/enabled', self.init_enabled)
        if enabled == True:
            if self.got_post_det_namespace != "None":
                self.img_dict_lock.acquire()
                img_dict = copy.deepcopy(self.img_dict)
                self.img_dict_lock.release()
                self.got_post_det_namespace = "None"

                if img_dict is not None:
                    det_namespace = img_dict['ros_det_namespace']
                    
                    wait_for_detect = nepi_ros.get_param(self,'~detector/wait_for_detect', self.init_wait_for_detect)
                    if wait_for_detect == True:
                        detect_dict_list = None
                        while detect_dict_list is None and not rospy.is_shutdown():
                            nepi_ros.sleep(.01)
                            detect_dict_list = copy.deepcopy(self.dets_info_dict[det_namespace]['cur_detection'])
                        self.dets_info_dict[det_namespace]['cur_detection'] = None
                    else:
                        detect_dict_list = self.dets_info_dict[det_namespace]['last_detection']
                    #nepi_msg.publishMsgInfo(self,"Postprocessing Image Topic: " + det_namespace)
                    # Postprocess Image with Last Detection
                    if img_dict['cv2_img'] is not None:
                        #nepi_msg.publishMsgWarn(self,"Postprocessor got img_dict from topic callback:  " + det_namespace + " with img size: " + str(img_dict['cv2_img'].shape[:2]))
                        try:
                            success = self.postProcessDetectionImage(det_namespace,img_dict, detect_dict_list)
                        except:
                            pass
                        postprocess_time = round( (nepi_ros.get_time_sec() - start_time) , 3)
                        self.dets_info_dict[det_namespace]['postprocess_time'] = postprocess_time
                        #nepi_msg.publishMsgInfo(self,"Detect Time: {:.2f}".format(detect_time))

                        current_time = nepi_ros.ros_time_now()
                        ros_timestamp = img_dict['ros_img_stamp']
                        latency = (current_time.to_sec() - ros_timestamp.to_sec())
                        self.dets_info_dict[det_namespace]['image_latency_time'] = latency
                        #nepi_msg.publishMsgInfo(self,"Image Pub Latency: {:.2f}".format(latency))

        rospy.Timer(rospy.Duration(0.001), self.processImgageCb, oneshot = True)


    def postProcessDetectionImage(self,det_namespace, img_dict, detect_dict_list):
        if 'cv2_img' not in img_dict.keys():
            return False
        if img_dict['cv2_img'] is None:
            return False
        cv2_img = img_dict['cv2_img']
        ros_timestamp = img_dict['ros_img_stamp']
        # Post process image with overlays
        if detect_dict_list is not None:
            # Publish image first for consumers
            if len(detect_dict_list) > 0:
                #nepi_msg.publishMsgWarn(self,"Starting detect image: " + str(cv2_img.shape))
                cv2_detect_img = self.apply_detection_overlay(det_namespace, detect_dict_list,cv2_img)
            else:
                cv2_detect_img = cv2_img
            #nepi_msg.publishMsgWarn(self,"Return detect image: " + str(cv2_detect_img.shape))
            detect_img_msg = nepi_img.cv2img_to_rosimg(cv2_detect_img, encoding="bgr8")
            detect_img_msg.header.stamp = nepi_ros.ros_time_now()
            if not rospy.is_shutdown() and self.detection_image_pub is not None:
                self.detection_image_pub.publish(detect_img_msg)
                
                self.detection_image_all_pub.publish(detect_img_msg)
        
                if det_namespace in self.dets_pub_sub_dict.keys():
                    dets_pub_sub_dict = self.dets_pub_sub_dict[det_namespace]
                    #nepi_msg.publishMsgWarn(self,"Got Img Dict: " + str(dets_pub_sub_dict))
                    detection_image_pub = dets_pub_sub_dict['detection_image_pub']
                    detection_image_pub.publish(detect_img_msg)
                
            # Save Image Data if needed
            data_product = 'detection_image'
            image_text = det_namespace.replace(self.base_namespace,"")
            image_text = image_text.replace('/idx',"")
            image_text = image_text.replace('/','_')
            nepi_save.save_ros_img2file(self,data_product,detect_img_msg,ros_timestamp, add_text = image_text)
        return True


    def getImgShortName(self, det_namespace):
        short_name = det_namespace.replace(self.base_namespace,"")
        if short_name.find("idx") != -1:
            short_name = short_name.replace("/idx","")
        return short_name
   

    def apply_detection_overlay(self,det_namespace, detect_dict_list,cv2_img):
        cv2_detect_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 



        # Overlay text data on OpenCV image
        font                   = cv2.FONT_HERSHEY_DUPLEX
        fontScale, thickness  = nepi_img.optimal_font_dims(cv2_detect_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
        fontColor = (255, 255, 255)
        fontColorBk = (0,0,0)
        lineType = cv2.LINE_AA

        if self.first_detect_complete == False:

            text2overlay="WAITING FOR FIRST DETECTION"
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

        else:
            for detect_dict in detect_dict_list:
                img_size = cv2_img.shape[:2]
                ###### Apply Image Overlays and Publish Image ROS Message
                # Overlay adjusted detection boxes on image 
                class_name = detect_dict['name']
                xmin = detect_dict['xmin']
                ymin = detect_dict['ymin']
                xmax = detect_dict['xmax']
                ymax = detect_dict['ymax']

                if xmin <= 0:
                    xmin = 5
                if ymin <= 0:
                    ymin = 5
                if xmax >= img_size[1]:
                    xmax = img_size[1] - 5
                if ymax >= img_size[0]:
                    ymax = img_size[0] - 5


                start_point = (xmin, ymin)
                end_point = (xmax, ymax)

                class_color = (255,0,0)
                
                if class_name in self.classes_list:
                    class_ind = self.classes_list.index(class_name)
                    if class_ind < len(self.classes_color_list):
                        class_color = tuple(self.classes_color_list[class_ind])
                class_color =  [int(c) for c in class_color]

                line_thickness = 2


                success = False
                try:
                    cv2.rectangle(cv2_detect_img, start_point, end_point, class_color, thickness=thickness)
                    success = True
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to create bounding box rectangle: " + str(e))

                # Overlay text data on OpenCV image
                if success == True:


                    ## Overlay Labels
                    overlay_labels =  nepi_ros.get_param(self,'~detector/overlay_labels',self.init_overlay_labels)
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
                        overlay_clf_name = nepi_ros.get_param(self,'~detector/overlay_clf_name', self.init_overlay_clf_name)
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
                    overlay_img_name = nepi_ros.get_param(self,'~detector/overlay_img_name', self.init_overlay_img_name)
                    det_namespace = det_namespace
                    if overlay_img_name:
                        text2overlay=self.getImgShortName(det_namespace)
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


        return cv2_detect_img

    def publishDetectionData(self,det_namespace, detect_dict_list,ros_img_header):

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
            bbs_msg.image_topic = det_namespace
            bbs_msg.image_width = detect_dict['width_pixels']
            bbs_msg.image_height = detect_dict['height_pixels']
            bbs_msg.bounding_boxes = bounding_box_msg_list
            if not rospy.is_shutdown() and self.bounding_boxes_pub is not None:
                    self.bounding_boxes_pub.publish(bbs_msg)
                    self.detection_trigger_pub.publish()
                    self.detection_trigger_all_pub.publish()    
                                    
                    self.bounding_boxes_all_pub.publish(bbs_msg)   

                    if det_namespace in self.dets_pub_sub_dict.keys():
                        dets_pub_sub_dict = self.dets_pub_sub_dict[det_namespace]
                        bounding_box_pub = dets_pub_sub_dict['bounding_box_pub']
                        bounding_box_pub.publish(bbs_msg)
                        detection_trigger_pub = dets_pub_sub_dict['detection_trigger_pub']
                        detection_trigger_pub.publish()
                        detection_state_pub = dets_pub_sub_dict['detection_state_pub']
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

            if det_namespace in self.dets_pub_sub_dict.keys():
                dets_pub_sub_dict = self.dets_pub_sub_dict[det_namespace]
                found_object_pub = dets_pub_sub_dict['found_object_pub']
                found_object_pub.publish(found_object_msg)
                if count == 0:
                    self.detection_state_pub.publish(False)
                    if det_namespace in self.dets_pub_sub_dict.keys():
                        dets_pub_sub_dict = self.dets_pub_sub_dict[det_namespace]
                        detection_state_pub = dets_pub_sub_dict['detection_state_pub']
                        detection_state_pub.publish(False)
            

        # Save Bounding Data if needed
        image_text = det_namespace.replace(self.base_namespace,"")
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




    def handleInfoRequest(self,_):
        resp = AiDetectorInfoQueryResponse()
        resp.name = self.model_name
        resp.framework = self.model_framework
        resp.type = self.model_type
        resp.description = self.model_description
        resp.proc_img_height = self.model_proc_img_height
        resp.proc_img_width = self.model_proc_img_width
        resp.classes = self.classes_list
        resp.has_img_tiling = self.has_img_tiling
        return resp
    

    def publishStatusCb(self,timer):
        self.publishStatus()

    def publishStatus(self):
        enabled = nepi_ros.get_param(self,'~detector/enabled', self.init_enabled)
  
       
        status_msg = AiDetectorStatus()
        status_msg.name = self.model_name

        status_msg.namespace = self.node_namespace
        status_msg.state = self.state
        status_msg.enabled = enabled

        status_msg.wait_for_detect = nepi_ros.get_param(self,'~detector/wait_for_detect', self.init_wait_for_detect)
        status_msg.has_img_tiling = self.has_img_tiling
        status_msg.img_tiling = nepi_ros.get_param(self,'~detector/img_tiling', self.init_img_tiling)

        status_msg.overlay_labels = nepi_ros.get_param(self,'~detector/overlay_labels',self.init_overlay_labels)
        status_msg.overlay_clf_name = nepi_ros.get_param(self,'~detector/overlay_clf_name', self.init_overlay_clf_name)
        status_msg.overlay_img_name = nepi_ros.get_param(self,'~detector/overlay_img_name', self.init_overlay_img_name)


        status_msg.threshold = nepi_ros.get_param(self,'~detector/threshold', self.init_threshold)
        status_msg.max_rate_hz = nepi_ros.get_param(self,'~detector/max_rate', self.init_max_rate)

        img_source_topics = []
        img_det_namespaces = []
        img_connects = []
        img_det_lat_times = []
        img_pub_lat_times = []
        img_pre_times = []
        img_detect_times = []
        img_post_times = []
        dets_info_dict = copy.deepcopy(self.dets_info_dict)
        for det_namespace in dets_info_dict.keys():
            img_source_topics.append(det_namespace)
            img_det_namespaces.append(dets_info_dict[det_namespace]['det_namespace'])
            img_connects.append(dets_info_dict[det_namespace]['img_connected'])
            img_det_lat_times.append(dets_info_dict[det_namespace]['detect_latency_time'])
            img_pub_lat_times.append(dets_info_dict[det_namespace]['image_latency_time'])
            img_pre_times.append(dets_info_dict[det_namespace]['preprocess_time'])
            img_detect_times.append(dets_info_dict[det_namespace]['detect_time'])
            img_post_times.append(dets_info_dict[det_namespace]['postprocess_time'])
        status_msg.image_source_topics = img_source_topics
        status_msg.image_detect_namespaces = img_det_namespaces
        status_msg.images_connected = img_connects
        status_msg.detect_latency_times = img_det_lat_times
        status_msg.image_latency_times = img_pub_lat_times
        status_msg.preprocess_times = img_pre_times
        status_msg.detect_times = img_detect_times
        status_msg.postprocess_times = img_post_times

        img_selected = len(img_connects) > 0
        status_msg.image_selected = img_selected
        img_connected = True in img_connects
        status_msg.image_connected = img_connected

        det_lat_time = 0.0
        img_lat_time = 0.0
        pre_time = 0.0
        detect_time = 0.0
        post_time = 0.0
        if img_connected:
            det_lat_time = sum(img_det_lat_times) / len(img_det_lat_times)
            img_lat_time = sum(img_pub_lat_times) / len(img_pub_lat_times)
            pre_time = sum(img_pre_times) / len(img_pre_times)
            detect_time = sum(img_detect_times) / len(img_detect_times)
            post_time = sum(img_post_times) / len(img_post_times)
        status_msg.detect_latency_time = det_lat_time
        status_msg.image_latency_time = img_lat_time
        status_msg.preprocess_time = pre_time
        status_msg.detect_time = detect_time
        status_msg.postprocess_time = post_time

        #nepi_msg.publishMsgWarn(self,"Sending Status Msg: " + str(status_msg))
        if not rospy.is_shutdown():
            self.status_pub.publish(status_msg)



