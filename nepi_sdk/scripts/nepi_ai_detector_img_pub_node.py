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
from nepi_ros_interfaces.msg import AiDetectrorInfo
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, 
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img
from nepi_sdk import nepi_save

from nepi_sdk.save_data_if import SaveDataIF
from nepi_sdk.ai_detection_if import AiDetectionIF


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

    CLEAR_IMG_TIME = 1 # Clear after no bounding box updates
    first_det_time = False
    last_det_time = nepi_ros.get_time()

    data_products = ['bounding_boxes','detection_image']

    

    save_cfg_if = None

   

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

        detector_namespace = nepi_ros.get_param(self,"~pub_sub_namespace","")
        if self.detector_namespace = "":
            nepi_msg.publishMsgError(self,"Failed to read pub_sub_namespace from param server")
            nepi_msg.publishMsgError(self,"Shutting Down None")
            time.sleep
            sys.exit(-1)
        else:
            if detector_namespace[-1] == '/':
                detector_namespace[0:-1]
        self.detector_namespace = detector_namespace

        aif = AiDetectionIF(self, 
                                    connect_timeout = 180,
                                    det_namespace = detector_namespace,
                                    auto_connect_source_image = True,
                                    throttle_get_image_callback = True,
                                    imagePreprocessFunction = None
                                    ):
        # Get AI IF State Info
        self.ai_mgr_connected = aif.ai_mgr_connected
        self.det_connected = aif.det_connected
        self.img_subscribed = aif.img_subscribed
        self.img_topic = aif.img_topic

        self.classes_color_list = aif.get_classes_color_list(classes_list)

        # Setup Data Saving
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'detection_image' in self.data_products:
            factory_data_rates['detection_image'] = [1.0, 0.0, 100.0] 
        self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates, pub_sub_namespace = detector_namespace)

        # Status message images
        message = "DETECTOR_NOT_ENABLED"
        cv2_img = nepi_img.create_message_image(message,image_size = aif.status_img_size)
        self.ros_not_enb_img = nepi_img.cv2img_to_rosimg(cv2_img)
        message = "WAITING FOR IMAGE STREAM"
        cv2_img = nepi_img.create_message_image(message,image_size = aif.status_img_size)
        self.ros_no_img_img = nepi_img.cv2img_to_rosimg(cv2_img)
        self.ros_no_det_namespace_img = nepi_img.cv2img_to_rosimg(cv2_img)
        message = "WAITING FOR DETECTION DATA"
        cv2_img = nepi_img.create_message_image(message,image_size = aif.status_img_size)
        self.ros_no_det_img = nepi_img.cv2img_to_rosimg(cv2_img)

        # Create Publishers
        det_img_topic = os.path.join(ais.det_namespace,'detection_image')
        self.det_img_pub = rospy.Publisher(det_img_topic, Image,  queue_size = 1)
        det_img_all_topic = os.path.join(ais.det_base_namespace,'detection_image')
        self.det_img_all_pub = rospy.Publisher(det_img_all_topic, Image,  queue_size = 1)


        rospy.Timer(rospy.Duration(.1), self.imgUpdateCb, oneshot = True)
        
        nepi_msg.publishMsgInfo(self,"Initialization Complete")



    def imgUpdateCb(self,timer):
        if aif.det_status_msg is not None:
            enabled = aif.det_enabled
            if enabled == True:

                max_rate = aif.det_status_msg.max_rate * 2
                delay_time = float(1) / max_rate 
                current_time = nepi_ros.get_time_sec()
                timer = round((current_time - self.last_detect_time), 3)
                #nepi_msg.publishMsgWarn(self,"Delay and Timer: " + str(delay_time) + " " + str(timer))

                timer > delay_time: 
                    if aif.got_img is True:
                        # Process got image
                        #nepi_msg.publishMsgWarn(self,"Copying img_dict from topic callback:  " + image_topi)
                        self.img_dict_lock.acquire()
                        img_dict = copy.deepcopy(aif.img_dict)
                        aif.img_dict = None
                        self.img_dict_lock.release()
                        #nepi_msg.publishMsgWarn(self,"Copying img_dict from topic callback:  " + image_topi)

                        # Request new img
                        aif.get_img = True

                        if img_dict is None:
                            nepi_msg.publishMsgWarn(self,"Callback provided None img_dict, :  " + image_topi)
                        else:
                            if img_dict['cv2_img'] is None:
                                nepi_msg.publishMsgWarn(self,"Callback provided None cv2_img, :  " + image_topi)
                                pass
                            else:
                                img_topic = aif.img_topic
                                det_dict_list = copy.deepcopy(aif.det_dict_list)
                                success = self.processDetImage(img_topic, img_dict, det_dict_list)
                                    
        rospy.Timer(rospy.Duration(0.001), self.updateDetImageCb, oneshot = True)



    def processDetImage(self,image_topic, img_dict, detect_dict_list):
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
                cv2_det_img = self.apply_detection_overlay(image_topic, detect_dict_list, cv2_img)
            else:
                cv2_det_img = cv2_img
            #nepi_msg.publishMsgWarn(self,"Return detect image: " + str(cv2_det_img.shape))
            det_img_msg = nepi_img.cv2img_to_rosimg(cv2_det_img, encoding="bgr8")
            det_img_msg.header.stamp = nepi_ros.ros_time_now()
            if not rospy.is_shutdown():
                self.det_img_pub.publish(det_img_msg)
                self.det_img_all_pub.publish(det_img_msg)
        
            # Save Image Data if needed
            data_product = 'detection_image'
            image_text = image_topic.replace(self.base_namespace,"")
            image_text = image_text.replace('/idx',"")
            image_text = image_text.replace('/','_')
            nepi_save.save_ros_img2file(self,data_product,det_img_msg,ros_timestamp, add_text = image_text)
        return True



   

    def apply_detection_overlay(self,image_topi, detect_dict_list,cv2_img):
        cv2_det_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 



        # Overlay text data on OpenCV image
        font                   = cv2.FONT_HERSHEY_DUPLEX
        fontScale, thickness  = nepi_img.optimal_font_dims(cv2_det_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
        fontColor = (255, 255, 255)
        fontColorBk = (0,0,0)
        lineType = cv2.LINE_AA


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
                cv2.rectangle(cv2_det_img, start_point, end_point, box_color , -1)
                cv2.putText(cv2_det_img,text2overlay, 
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
                    cv2.rectangle(cv2_det_img, start_point, end_point, class_color, thickness=thickness)
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
                            cv2.rectangle(cv2_det_img, start_point, end_point, box_color , -1)
                            cv2.putText(cv2_det_img,text2overlay, 
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
                                cv2.putText(cv2_det_img,text2overlay, 
                                    bottomLeftCornerOfText, 
                                    font, 
                                    fontScale,
                                    fontColorBk,
                                    thickness*2,
                                    lineType)
                                cv2.putText(cv2_det_img,text2overlay, 
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
                     if overlay_img_name:
                        text2overlay=self.getImgShortName(image_topi)
                        text_size = cv2.getTextSize(text2overlay, 
                            font, 
                            fontScale,
                            thickness)
                        #nepi_msg.publishMsgWarn(self,"Text Size: " + str(text_size))
                        line_height = text_size[0][1]
                        line_width = text_size[0][0]
                        bottomLeftCornerOfText = (x_start,y_start)
                        try:
                            cv2.putText(cv2_det_img,text2overlay, 
                                bottomLeftCornerOfText, 
                                font, 
                                fontScale,
                                fontColorBk,
                                thickness*2,
                                lineType)
                            cv2.putText(cv2_det_img,text2overlay, 
                                bottomLeftCornerOfText, 
                                font, 
                                fontScale,
                                fontColor,
                                thickness,
                                lineType)
                        except Exception as e:
                            nepi_msg.publishMsgWarn(self,"Failed to apply overlay text: " + str(e))


        return cv2_det_img

                    







