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
import cv2


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img
from nepi_sdk import nepi_save

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_ros_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryResponse

from nepi_sdk.save_data_if import SaveDataIF
from nepi_sdk.ai_manager_if import AiManagerIF
from nepi_sdk.ai_detection_if import AiDetectionIF
from nepi_sdk.image_if import ImageIF


EXAMPLE_AI_MGR_STATUS_DICT = {
    'ai_frameworks': [],

    'ai_models': [],
    'ai_models_frameworks': [],
    'ai_models_types': [],

    'active_ai_framework': 'framework_name',

    'active_ai_models': [],
    'active_ai_models_frameworks': [],
    'active_ai_models_types': [],
    'active_ai_models_nodes': [],
    'active_ai_models_namespaces': [],

    'all_namespace': '/base_namespace/ai/all'
}

EXAMPLE_BOXES_INFO_DICT_ENTRY = {
    'model_name': 'test_model',
    'image_header': Header(),
    'image_topic': 'my/test_topic',
    'src_height': 600,
    'src_width': 1000,
    'prc_height': 300,
    'prc_width': 500,
}


EXAMPLE_BOX_DICT_ENTRY = {
    'name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'prob': .3, # Probability of detection
    'xmin': 10,
    'ymin': 10,
    'xmax': 100,
    'ymax': 100,
    'area_ratio': 0.054,
    'area_pixels': 8100
}


NONE_IMG_DICT = {       
    'cv2_img': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_ros.get_time(),
    'ros_img_topic': 'None',
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp
}


BLANK_SIZE_DICT = { 'h': 350, 'w': 700, 'c': 3}

BLANK_CV2_IMAGE = nepi_img.create_blank_image((BLANK_SIZE_DICT['h'],BLANK_SIZE_DICT['w'],BLANK_SIZE_DICT['c']))

BLANK_IMG_DICT = {       
    'cv2_img': BLANK_CV2_IMAGE,
    'width': 0,
    'height': 0,
    'timestamp': nepi_ros.get_time(),
    'ros_img_topic': 'None',
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp
}


class AiDetectorImgPub:


    last_time = nepi_ros.get_time()

    last_img_dict = None

    data_products = ['bounding_boxes','detection_image']

    save_cfg_if = None

    det_if = None
    img_if = None

    pub_inactive_img = True


    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "ai_detector_img_pub" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_namespace = self.base_namespace + self.node_name
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
        ##############################

        ##############################
        # Get detector base namespace from param server
        success = True
        # Get detector namespace from param server
        det_namespace = nepi_ros.get_param(self,self.node_namespace + "/det_namespace","")
        if det_namespace == "":
            nepi_msg.publishMsgWarn(self,"Failed to read detector namespace from param server")
            success == False
        self.det_namespace = nepi_ros.clear_end_slash(det_namespace)

        # Get detector base namespace from param server
        det_base_namespace = nepi_ros.get_param(self,self.node_namespace + "/base_namespace","")
        if det_base_namespace == "":
            nepi_msg.publishMsgWarn(self,"Failed to read detector base namespace from param server")
            success == False
        self.det_base_namespace = nepi_ros.clear_end_slash(det_base_namespace)

        # Get detector pub sub namespace from param server
        det_all_namespace = nepi_ros.get_param(self,self.node_namespace + "/all_namespace","")
        if det_all_namespace == "":
            nepi_msg.publishMsgWarn(self,"Failed to read detector all namespace from param server")
            success == False
        self.det_all_namespace = nepi_ros.clear_end_slash(det_all_namespace)

        if success == False:
            nepi_msg.publishMsgWarn(self,"Failed to read required params from param server")
            nepi_msg.publishMsgWarn(self,"Shutting Down None")
            time.sleep
            sys.exit(-1)

        ##############################
        # Connect to AI Detector
        self.det_if = AiDetectionIF()

        success = self.det_if.register_detector(det_namespace, det_base_namespace, timeout = 180)
        if success == False:
            msg = str("Failed to connect to detector: " + det_namespace )
            nepi_msg.publishMsgWarn(self,msg)
            nepi_ros.signal_shutdown(msg)
        else:
            time.sleep(1)
            self.det_connected = self.det_if.connected
            self.img_topic = self.det_if.src_img_topic
            self.classes_list = self.det_if.classes
            self.classes_color_list = self.det_if.classes_colors
            nepi_msg.publishMsgInfo(self,"Detector provided source image topc: " + str(self.img_topic))

            ##############################
            # Connect to Source Image
            self.img_if = ImageIF( throttle_get_image_callback = True,
                                imagePreprocessFunction = self.preprocessImageFunction
                                )
            success = self.img_if.connectImgTopic(self.img_topic,timeout = 60)
            if success == False:
                msg = str("Failed to connect to image topic: " + str(self.img_topic) )
                nepi_msg.publishMsgWarn(self,msg)
                nepi_ros.signal_shutdown(msg)
            else:
                time.sleep(1)
                pass

        ##############################
        # Setup Data Saving
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'detection_image' in self.data_products:
            factory_data_rates['detection_image'] = [1.0, 0.0, 100.0] 
        self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates, pub_sub_namespace = det_namespace) 

        ##############################
        # Create Status Message Images

        message = "IMAGE_NOT_ACTIVE"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_not_act_img = nepi_img.cv2img_to_rosimg(cv2_img)

        message = "DETECTOR_NOT_ENABLED"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_not_enb_img = nepi_img.cv2img_to_rosimg(cv2_img)

        message = "WAITING FOR IMAGE STREAM"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_no_img_img = nepi_img.cv2img_to_rosimg(cv2_img)

        message = "WAITING FOR DETECTION DATA"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_no_det_img = nepi_img.cv2img_to_rosimg(cv2_img)

        ##############################
        # Create Publishers
        img_pub_topic = os.path.join(det_namespace,'detection_image')
        self.img_pub = rospy.Publisher(img_pub_topic, Image,  queue_size = 1, latch = True)
        det_img_pub_topic = os.path.join(det_base_namespace,'detection_image')
        self.det_img_pub = rospy.Publisher(det_img_pub_topic, Image,  queue_size = 1, latch = True)
        det_img_all_pub_topic = os.path.join(det_all_namespace,'detection_image')
        self.det_img_all_pub = rospy.Publisher(det_img_all_pub_topic, Image,  queue_size = 1, latch = True)

        ##############################
        # Create Subscribers
        pass

        ##############################
        # Launch Timer Processes
        rospy.Timer(rospy.Duration(.1), self.imgUpdateCb, oneshot = True)

        #########################################################
        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################


    def preprocessImageFunction(self,cv2_img):      
        # Convert BW image to RGB
        if nepi_img.is_gray(cv2_img):
            cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_GRAY2BGR)
        return cv2_img


    def imgUpdateCb(self,timer):
        if self.det_if.status_msg is not None:
            max_rate = self.det_if.status_msg.max_rate_hz
            delay_time = float(1) / max_rate 
            current_time = nepi_ros.get_time()
            timer = round((current_time - self.last_time), 3)
            #nepi_msg.publishMsgWarn(self,"Delay and Timer: " + str(delay_time) + " " + str(timer))
            
            got_img = copy.deepcopy(self.img_if.got_img)

            if timer > delay_time: 
                enabled = self.det_if.status_dict['enabled']
                #nepi_msg.publishMsgWarn(self,"Time to Pub with get_img: " + str(get_img) + " & got_img: " + str(got_img))
                if self.det_if.active == False:
                    if self.pub_inactive_img == True:
                        self.pub_inactive_img = False
                        #nepi_msg.publishMsgWarn(self,"Publishing not enabled msg img")
                        if not rospy.is_shutdown():
                            det_img_msg = self.ros_not_act_img 
                            det_img_msg.header.stamp = nepi_ros.ros_time_now()
                            self.img_pub.publish(det_img_msg)
                            self.det_img_pub.publish(det_img_msg)
                            self.det_img_all_pub.publish(det_img_msg)

                else:
                    self.pub_inactive_img = True
                    if enabled == False:
                        #nepi_msg.publishMsgWarn(self,"Publishing not enabled msg img")
                        if not rospy.is_shutdown():
                            det_img_msg = self.ros_not_enb_img
                            det_img_msg.header.stamp = nepi_ros.ros_time_now()
                            self.img_pub.publish(det_img_msg)
                            self.det_img_pub.publish(det_img_msg)
                            self.det_img_all_pub.publish(det_img_msg)

                            # Reset Timer
                            self.last_time = current_time
                
                    elif self.img_if.img_connected == False:
                        #nepi_msg.publishMsgWarn(self,"Publishing no img msg img")
                        if not rospy.is_shutdown():
                            det_img_msg = self.ros_no_img_img
                            det_img_msg.header.stamp = nepi_ros.ros_time_now()
                            self.img_pub.publish(det_img_msg)
                            self.det_img_pub.publish(det_img_msg)
                            self.det_img_all_pub.publish(det_img_msg)

                            # Reset Timer
                            self.last_time = current_time

                    elif got_img is True:
                        # Process got image
                        nepi_msg.publishMsgWarn(self,"Got_img is True: Copying img_dict from ImageIF")
                        self.img_if.img_dict_lock.acquire()
                        #img_dict = copy.deepcopy(self.last_img_dict) # Use n-1 image to better match detection image
                        #self.last_img_dict = copy.deepcopy(self.img_if.img_dict)
                        img_dict = copy.deepcopy(self.img_if.img_dict)
                        self.img_if.img_dict = None
                        self.img_if.img_dict_lock.release()
                        #nepi_msg.publishMsgWarn(self,"Copied last_img_dict Class. Got last size:  " + str(self.last_img_dict['cv2_img'].shape))
                        #nepi_msg.publishMsgWarn(self,"Copied img_dict from ImageIF. Got last size:  " + str(img_dict['cv2_img'].shape))

                        # Reset Timer
                        self.last_time = current_time

                         # Request new img
                        self.img_if.get_img = True

                        if img_dict is None:
                            nepi_msg.publishMsgWarn(self,"Got None img_dict")
                        else:
                            if img_dict['cv2_img'] is None:
                                nepi_msg.publishMsgWarn(self,"Got None cv2_img")
                                pass
                            else:
                                img_topic = self.img_if.img_topic
                                det_dict_list = copy.deepcopy(self.det_if.bboxes_list)
                                success = self.processDetImage(img_topic, img_dict, det_dict_list)
                    
                    elif got_img == False:
                        if self.img_if.get_img == False:
                        #nepi_msg.publishMsgWarn(self,"Reseting get_img to True")
                        self.img_if.get_img = True
                                
        rospy.Timer(rospy.Duration(0.001), self.imgUpdateCb, oneshot = True)



    def processDetImage(self,image_topic, img_dict, detect_dict_list):
        if 'cv2_img' not in img_dict.keys():
            return False
        if img_dict['cv2_img'] is None:
            return False
        
        
        cv2_img = img_dict['cv2_img']
        nepi_msg.publishMsgWarn(self,"Process image Got image: " + str(cv2_img.shape))
        ros_timestamp = img_dict['ros_img_stamp']
        # Post process image with overlays
        if detect_dict_list is not None:
            # Publish image first for consumers
            if len(detect_dict_list) > 0:
                #nepi_msg.publishMsgWarn(self,"Starting detect image: " + str(cv2_img.shape))
                cv2_det_img = self.apply_detection_overlay(image_topic, detect_dict_list, cv2_img)
            else:
                cv2_det_img = cv2_img
            nepi_msg.publishMsgWarn(self,"Return detect image: " + str(cv2_det_img.shape))
            det_img_msg = nepi_img.cv2img_to_rosimg(cv2_det_img, encoding="bgr8")
            det_img_msg.header.stamp = nepi_ros.ros_time_now()
            if not rospy.is_shutdown():
                self.img_pub.publish(det_img_msg)
                self.det_img_pub.publish(det_img_msg)
                self.det_img_all_pub.publish(det_img_msg)
        
            # Save Image Data if needed
            data_product = 'detection_image'
            if image_topic is not None:
                image_text = image_topic.replace(self.base_namespace,"")
                image_text = image_text.replace('/idx',"")
                image_text = image_text.replace('/','_')
            else:
                imagee_text = ""
            nepi_save.save_ros_img2file(self,data_product,det_img_msg,ros_timestamp, add_text = image_text)
        return True


    def apply_detection_overlay(self,image_topic, detect_dict_list, cv2_img):
        cv2_det_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 

        if self.det_if.status_msg is not None:
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
                    overlay_labels =  self.det_if.status_dict['overlay_labels']
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
                        overlay_clf_name = self.det_if.status_dict['overlay_clf_name']
                        if overlay_clf_name:
                            text2overlay=self.det_if.det_dict['name']
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
                    overlay_img_name = self.det_if.status_dict['overlay_img_name']
                    if overlay_img_name:
                        text2overlay=nepi_img.getImgShortName(image_topic)
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

                    
#########################################
# Main
#########################################
if __name__ == '__main__':
  node = AiDetectorImgPub()






