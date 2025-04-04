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
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img
from nepi_sdk import nepi_save

from std_msgs.msg import UInt8, Int32 Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image

from nepi_ros_interfaces.msg import ImageStatus


from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_ros_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryResponse

from nepi_api.node_if_cfg import NodeClassIF
from nepi_api.sys_if_msg import MsgIF
from nepi_api.save_data_if import SaveDataIF
from nepi_api.connect_ai_if_detector import ConnectAiDetectorIF
from nepi_api.connect_data_if_image import ConnectImageIF


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
    det_base_namespace = None
    det_all_namespace = None
    det_status_dict = None
    det_img_dict = dict()
    save_data_if_dict = dict()
    data_products = ['detection_image']

    det_connected = False
    img_publishing = False
    last_img_time = nepi_ros.get_time()
    last_img_dict = None


    sub_det_if = None
    sub_img_if = None
    pub_img_if = None
    pub_img_det_if = None
    pub_img_all_if = None

    pub_inactive_img = True

    last_det_dict_list = []

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "ai_detector_img_pub" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        # Get detector base namespace from param server
        success = True
        self.det_base_namespace = self.node_namespace

        # Get detector pub sub namespace from param server
        det_all_namespace = nepi_ros.get_param(self,self.node_namespace + "/all_namespace","")
        if det_all_namespace == "":
            self.msg_if.pub_warn("Failed to read detector all namespace from param server")
            success == False
        self.det_all_namespace = nepi_utils.clear_end_slash(det_all_namespace)

        if success == False:
            self.msg_if.pub_warn("Failed to read required params from param server")
            self.msg_if.pub_warn("Shutting Down None")
            time.sleep
            sys.exit(-1)

        ##############################
        # Connect to AI Detector
        self.sub_det_if = ConnectAiDetectorIF()

        success = self.sub_det_if.register_detector(det_img_namespace, det_base_namespace, timeout = 180)
        if success == False:
            msg = str("Failed to connect to detector: " + det_img_namespace )
            nepi_msg.publishMsgWarn(self,msg)
            nepi_ros.signal_shutdown(msg)
        else:
            time.sleep(1)
            self.det_connected = self.sub_det_if.wait_for_connection()

            self.img_topic = self.sub_det_if.get_source_img_topic()
            self.msg_if.pub_info("Detector provided source image topc: " + str(self.img_topic))

            self.classes_list = self.sub_det_if.get_classes()
            self.msg_if.pub_warn("Detector provided classes list: " + str(self.classes_list))

            self.classes_color_list = self.sub_det_if.get_classes_colors()
            #self.msg_if.pub_warn("Detector provided classes color list: " + str(self.classes_color_list))

            ##############################
            # Connect to Source Image
            self.sub_img_if = ConnectImageIF( throttle_get_image_callback = True,
                                imagePreprocessFunction = self.preprocessImageFunction
                                )
            success = self.sub_img_if.connect_image_topic(self.img_topic,timeout = 60)
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
        self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates, pub_sub_namespace = det_img_namespace) 


        ##############################
        # Create Image Publishers and Messages
        img_pub_topic = os.path.join(det_img_namespace,'detection_image')
        self.pub_img_if = ImageIF(img_pub_topic)
        ready = self.pub_img_if.wait_for_ready()

        det_img_pub_topic = os.path.join(det_base_namespace,'detection_image')
        self.pub_img_det_if = ImageIF(det_img_pub_topic)
        ready = self.pub_img_det_if.wait_for_ready()


        det_img_all_pub_topic = os.path.join(det_all_namespace,'detection_image')
        self.pub_img_all_if = ImageIF(det_img_all_pub_topic)
        ready = self.pub_img_all_if.wait_for_ready()

        time.sleep(1)

        # Create Img Status Message
        self.not_enabled_msg = "DETECTOR NOT ENABLED"
        self.not_connected_msg = "IMAGE NOT CONNECTED"
        self.waiting_img_msg = "WAITING FOR IMAGE"


        ##############################
        # Create Subscribers
        pass

        ##############################
        # Launch Timer Processes
        rospy.Timer(rospy.Duration(.1), self.imgUpdateCb, oneshot = True)

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################


    def preprocessImageFunction(self,cv2_img):      
        # Convert BW image to RGB
        if nepi_img.is_gray(cv2_img):
            cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_GRAY2BGR)
        return cv2_img



    def imgUpdateCb(self,timer):
        self.det_status_dict = self.sub_det_if.get_status_dict()
        if self.det_status_dict is not None:
            max_rate = self.det_status_dict['max_rate_hz']
            delay_time = float(1) / max_rate 
            current_time = nepi_ros.get_time()
            timer = round((current_time - self.last_img_time), 3)
            #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))
            
            img_connected = self.sub_img_if.check_connection()
            [get_img,got_img] = self.sub_img_if.read_get_got_img_states()

            if timer > delay_time: 
                enabled = self.det_status_dict['enabled']
                #self.msg_if.pub_warn("Time to Pub with get_img: " + str(get_img) + " & got_img: " + str(got_img))
                if self.sub_det_if.get_active_state() == False:
                    if self.pub_inactive_img == True:
                        self.pub_inactive_img = False
                        #self.msg_if.pub_warn("Publishing not connected msg img")
                        if not rospy.is_shutdown():
                            self.pub_img_if.publish_cv2_msg_img(self.not_connected_msg)

                else:
                    self.pub_inactive_img = True
                    if enabled == False:
                        #self.msg_if.pub_warn("Publishing not enabled msg img")
                        if not rospy.is_shutdown():
                            self.pub_img_if.publish_cv2_msg_img(self.not_enabled_msg)
                            # Reset Timer
                            self.last_img_time = current_time
                
                    elif img_connected == False:
                        #self.msg_if.pub_warn("Publishing waiting for img msg img")
                        if not rospy.is_shutdown():
                            det_img_msg = self.ros_no_img_img
                            det_img_msg.header.stamp = nepi_ros.ros_time_now()
                            self.pub_img_if.publish_cv2_msg_img(self.waiting_img_msg)
                            self.pub_img_det_if.publish_cv2_msg_img(self.waiting_img_msg)

                            # Reset Timer
                            self.last_img_time = current_time

                    elif got_img is True:
                        # Process got image
                        #self.msg_if.pub_warn("Got_img is True: Copying img_dict from ImageIF")
                        img_dict = self.sub_img_if.get_img_dict()
                        #self.msg_if.pub_warn("Copied last_img_dict Class. Got last size:  " + str(self.last_img_dict['cv2_img'].shape))
                        #self.msg_if.pub_warn("Copied img_dict from ImageIF. Got last size:  " + str(img_dict['cv2_img'].shape))

                        # Reset Timer
                        self.last_img_time = current_time

                         # Request new img
                        self.sub_img_if.set_get_img(True)
                        det_dict_list = []
                        if img_dict is None:
                            self.msg_if.pub_warn("Got None img_dict")
                        else:
                            if img_dict['cv2_img'] is None:
                                self.msg_if.pub_warn("Got None cv2_img")
                                pass
                            else:

                                img_topic = self.sub_img_if.img_topic
                                det_dict = self.sub_det_if.get_clear_boxes_dict()
                                if det_dict is None:
                                    #self.msg_if.pub_warn("Got None det_dict")
                                    det_dict_list = self.last_det_dict_list
                                else:
                                    if 'boxes' in det_dict.keys():
                                        det_dict_list = det_dict['boxes']
                                        self.last_det_dict_list = det_dict_list
                                    else:
                                        det_dict_list = []
                                success = self.processDetImage(img_topic, img_dict, det_dict_list)
                    
                    elif timer > 2 * delay_time: # Reset collection call
                        [get_img,got_img] = self.sub_img_if.read_get_got_img_states()
                        if got_img == False and get_img == False:
                            #self.msg_if.pub_warn("Reseting get_img to True")
                            self.sub_img_if.set_get_img(True)
                                
        rospy.Timer(rospy.Duration(0.001), self.imgUpdateCb, oneshot = True)



    def processDetImage(self,image_topic, img_dict, detect_dict_list):
        if 'cv2_img' not in img_dict.keys():
            return False
        if img_dict['cv2_img'] is None:
            return False
        
        
        cv2_img = img_dict['cv2_img']
        #self.msg_if.pub_warn("Process image Got image: " + str(cv2_img.shape))
        ros_timestamp = img_dict['ros_img_stamp']
        # Post process image with overlays
        if detect_dict_list is not None:
            # Publish image first for consumers
            #self.msg_if.pub_warn("Starting detect image: " + str(cv2_img.shape))
            cv2_det_img = self.apply_detection_overlay(image_topic, detect_dict_list, cv2_img)
            #self.msg_if.pub_warn("Return detect image: " + str(cv2_det_img.shape)
            if not rospy.is_shutdown():
                self.pub_img_if.publish_cv2_img(cv2_det_img, ros_timestamp = ros_timestamp)
                self.pub_img_det_if.publish_cv2_img(cv2_det_img, ros_timestamp = ros_timestamp)
                self.pub_img_all_if.publish_cv2_img(cv2_det_img, ros_timestamp = ros_timestamp)
        
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



        if self.det_status_dict is not None:
            for detect_dict in detect_dict_list:
                img_size = cv2_img.shape[:2]

                # Overlay text data on OpenCV image
                font = cv2.FONT_HERSHEY_DUPLEX
                fontScale, font_thickness  = nepi_img.optimal_font_dims(cv2_det_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
                fontColor = (255, 255, 255)
                fontColorBk = (0,0,0)
                lineType = cv2.LINE_AA


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


                bot_left_box = (xmin, ymin)
                top_right_box = (xmax, ymax)

                class_color = (255,0,0)
                if class_name in self.classes_list:
                    class_ind = self.classes_list.index(class_name)
                    #self.msg_if.pub_warn("Got Class Index: " + str(class_ind))
                    if class_ind < len(self.classes_color_list):
                        class_color = tuple(self.classes_color_list[class_ind])
                        #self.msg_if.pub_warn("Got Class Color: " + str(class_color))
                class_color =  [int(c) for c in class_color]

                line_thickness = font_thickness


                success = False
                try:
                    cv2.rectangle(cv2_det_img, bot_left_box, top_right_box, class_color, thickness=line_thickness)
                    success = True
                except Exception as e:
                    self.msg_if.pub_warn("Failed to create bounding box rectangle: " + str(e))

                # Overlay text data on OpenCV image
                if success == True:


                    ## Overlay Labels
                    overlay_labels =  self.det_status_dict['overlay_labels']
                    if overlay_labels:
                        text2overlay=class_name
                        text_size = cv2.getTextSize(text2overlay, 
                            font, 
                            fontScale,
                            font_thickness)
                        #self.msg_if.pub_warn("Text Size: " + str(text_size))
                        line_height = text_size[0][1]
                        line_width = text_size[0][0]
                        x_padding = int(line_height*0.4)
                        y_padding = int(line_height*0.4)
                        bot_left_text = (xmin + (line_thickness * 2) + x_padding , ymin + line_height + (line_thickness * 2) + y_padding)
                        # Create Text Background Box

                        bot_left_box =  (bot_left_text[0] - x_padding , bot_left_text[1] + y_padding)
                        top_right_box = (bot_left_text[0] + line_width + x_padding, bot_left_text[1] - line_height - y_padding )
                        box_color = [0,0,0]

                        try:
                            cv2.rectangle(cv2_det_img, bot_left_box, top_right_box, box_color , -1)
                            cv2.putText(cv2_det_img,text2overlay, 
                                bot_left_text, 
                                font, 
                                fontScale,
                                fontColor,
                                font_thickness,
                                lineType)
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to apply overlay text: " + str(e))

                        # Start name overlays    
                        x_start = int(img_width * 0.05)
                        y_start = int(img_height * 0.05)
                        ## Overlay Detector Name
                        overlay_clf_name = self.det_status_dict['overlay_clf_name']
                        if overlay_clf_name:
                            text2overlay=self.det_status_dict['name']
                            text_size = cv2.getTextSize(text2overlay, 
                                font, 
                                fontScale,
                                font_thickness)
                            #self.msg_if.pub_warn("Text Size: " + str(text_size))
                            line_height = text_size[0][1]
                            line_width = text_size[0][0]
                            bot_left_text = (x_start,y_start)
                            try:
                                cv2.putText(cv2_det_img,text2overlay, 
                                    bot_left_text, 
                                    font, 
                                    fontScale,
                                    fontColorBk,
                                    font_thickness*2,
                                    lineType)
                                cv2.putText(cv2_det_img,text2overlay, 
                                    bot_left_text, 
                                    font, 
                                    fontScale,
                                    fontColor,
                                    font_thickness,
                                    lineType)
                            except Exception as e:
                                self.msg_if.pub_warn("Failed to apply overlay text: " + str(e))
                            y_start = y_start + int(line_height * 1.5)
                    ## Overlay Image Name
                    overlay_img_name = self.det_status_dict['overlay_img_name']
                    if overlay_img_name:
                        text2overlay=nepi_img.getImgShortName(image_topic)
                        text_size = cv2.getTextSize(text2overlay, 
                            font, 
                            fontScale,
                            font_thickness)
                        #self.msg_if.pub_warn("Text Size: " + str(text_size))
                        line_height = text_size[0][1]
                        line_width = text_size[0][0]
                        bot_left_text = (x_start,y_start)
                        try:
                            cv2.putText(cv2_det_img,text2overlay, 
                                bot_left_text, 
                                font, 
                                fontScale,
                                fontColorBk,
                                font_thickness*2,
                                lineType)
                            cv2.putText(cv2_det_img,text2overlay, 
                                bot_left_text, 
                                font, 
                                fontScale,
                                fontColor,
                                font_thickness,
                                lineType)
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to apply overlay text: " + str(e))


        return cv2_det_img

                    
#########################################
# Main
#########################################
if __name__ == '__main__':
  node = AiDetectorImgPub()






