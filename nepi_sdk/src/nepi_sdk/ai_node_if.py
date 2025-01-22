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
from nepi_ros_interfaces.msg import ObjectCount, BoundingBox, BoundingBoxes

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_ais
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

class AiNodeIF:

    img_width = 0 # Updated on receipt of first image
    img_height = 0 # Updated on receipt of first image
    img_area = 0


    img_acquire = False
    img_msg = None
    last_img_msg = None
    img_lock = threading.Lock()

    last_time = time.time()

    def __init__(self,node_name, source_img_topic, pub_sub_namespace, classes_list, \
                setThresholdFunction, setMaxRateFunction, getMaxRateFunction, \
                processDetectionFunction):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ##############################    


        self.node_name = node_name
        if pub_sub_namespace[-1] == "/":
            pub_sub_namespace = pub_sub_namespace[:-1]
        self.pub_sub_namespace = pub_sub_namespace
        self.setThreshold = setThresholdFunction
        self.setMaxRate = setMaxRateFunction
        self.getMaxRate = getMaxRateFunction
        self.processDetection = processDetectionFunction
        self.source_img_topic = source_img_topic

        self.classes_list = classes_list
        self.classes_color_list = self.get_classes_color_list(classes_list)
                 
        nepi_msg.publishMsgInfo(self,"Starting IF setup")
        
        # Create AI Node Publishers

        
        FOUND_OBJECT_TOPIC = self.pub_sub_namespace + "/found_object"
        self.found_object_pub = rospy.Publisher(FOUND_OBJECT_TOPIC, ObjectCount,  queue_size = 1)

        BOUNDING_BOXES_TOPIC = self.pub_sub_namespace + "/bounding_boxes"
        self.bounding_boxes_pub = rospy.Publisher(BOUNDING_BOXES_TOPIC, BoundingBoxes, queue_size = 1)

        DETECTION_IMAGE_TOPIC = self.pub_sub_namespace + "/detection_image"
        self.detection_image_pub = rospy.Publisher(DETECTION_IMAGE_TOPIC, Image,  queue_size = 1)

        time.sleep(1)

        # Create AI Node Subscribers
        THRESHOLD_SUB_TOPIC = self.pub_sub_namespace + '/set_threshold'
        self.set_threshold_sub = rospy.Subscriber(THRESHOLD_SUB_TOPIC, Float32, self.setThresholdCb, queue_size=1)
        MAX_RATE_SUB_TOPIC = self.pub_sub_namespace + '/set_max_rate'
        self.set_threshold_sub = rospy.Subscriber(MAX_RATE_SUB_TOPIC, Float32, self.setMaxRateCb, queue_size=1)

        IMAGE_SUB_TOPIC = self.source_img_topic
        self.set_threshold_sub = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, self.imageCb, queue_size=1)
        	
        rospy.Timer(rospy.Duration(.1), self.updateDetectionCb, oneshot = True)

        nepi_msg.publishMsgInfo(self,"IF Initialization Complete")
        
                
    def setThresholdCb(self,msg):
        threshold = msg.data
        nepi_msg.publishMsgInfo(self,"Received Threshold Update: " + str(threshold))
        if (threshold < 0):
            threshold = 0
        elif (threshold > 1):
            threshold = 1
        self.setThreshold(threshold)

    def setMaxRateCb(self,msg):
        rate = msg.data
        nepi_msg.publishMsgInfo(self,"Received Max Rate Update: " + str(rate))
        if (rate > 0):
            self.setMaxRate(rate)



    def imageCb(self,image_msg):   
        self.img_lock.acquire()
        self.img_msg = copy.deepcopy(self.last_img_msg)
        self.img_lock.release()
        self.last_img_msg = copy.deepcopy(image_msg)


    def updateDetectionCb(self,timer):
        current_rate = self.getMaxRate()
        delay_time = float(1) / current_rate
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
                self.img_width = cv2_shape[1] 
                self.img_height = cv2_shape[0] 
                try:
                    detect_dict_list = self.processDetection(cv2_img) 
                    #nepi_msg.publishMsgInfo(self,"AIF got back detect_dict: " + str(detect_dict_list))
                    success = True
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to process detection img with exception: " + str(e))

                if detect_dict_list is not None:
                    self.publishDetectionData(detect_dict_list,ros_img_header)
                    # Now create and publish detection image
                    if len(detect_dict_list) > 0:
                        #nepi_msg.publishMsgWarn(self,"Starting detect image: " + str(cv2_img.shape))
                        cv2_detect_img = self.apply_detection_overlay(detect_dict_list,cv2_img)
                        #nepi_msg.publishMsgWarn(self,"Return detect image: " + str(cv2_detect_img.shape))
                        detect_img_msg = nepi_img.cv2img_to_rosimg(cv2_detect_img, encoding="bgr8")
                    #else:
                        #nepi_msg.publishMsgWarn(self,"No detections to add to image")
                else:
                    nepi_ros.signal_shutdown("Something went wrong in detection process call")
                    nepi_ros.sleep(2)
                if not rospy.is_shutdown():
                    self.detection_image_pub.publish(detect_img_msg)
        rospy.Timer(rospy.Duration(.01), self.updateDetectionCb, oneshot = True)

   

    def apply_detection_overlay(self,detect_dict_list,cv2_img):
        cv2_detect_img = copy.deepcopy(cv2_img)
        cv2_shape = cv2_img.shape
        if cv2_shape[2] == 1:
            cv2_detect_img = cv2.cvtColor(cv2_detect_img,cv2.COLOR_GRAY2BGR)
                

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
                    cv2.rectangle(cv2_detect_img, start_point, end_point, class_color, thickness=line_thickness)
                    success = True
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to create bounding box rectangle: " + str(e))

                # Overlay text data on OpenCV image
                if success == True:
                    # Overlay text data on OpenCV image
                    font                   = cv2.FONT_HERSHEY_DUPLEX
                    fontScale, thickness  = nepi_img.optimal_font_dims(cv2_detect_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
                    fontColor = (255, 255, 255)
                    lineType = 1

                    ## Overlay Label
                    text2overlay=class_name
                    text_size = cv2.getTextSize(text2overlay, 
                        font, 
                        fontScale,
                        thickness)
                    #nepi_msg.publishMsgWarn(self,"Text Size: " + str(text_size))
                    line_height = text_size[0][1]
                    line_width = text_size[0][0]
                    bottomLeftCornerOfText = (xmin,ymin - line_thickness * 2 - line_height)
                    # Create Text Background Box
                    padding = int(line_height*0.4)
                    start_point = (bottomLeftCornerOfText[0]-padding, bottomLeftCornerOfText[1]-line_height-padding)
                    end_point = (bottomLeftCornerOfText[0]+line_width+padding, bottomLeftCornerOfText[1]+padding)
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
                    if self.img_area > 1:
                        area_ratio = area_pixels / self.img_area
                    else:
                        area_ratio = -999
                    bounding_box_msg.area_pixels = detect_dict['area_pixels']
                    bounding_box_msg.area_ratio = detect_dict['area_ratio']
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Failed to get all data from detect dict: " + str(e)) 
                bounding_box_msg_list.append(bounding_box_msg)
            bounding_boxes_msg = BoundingBoxes()
            bounding_boxes_msg.header.stamp = ros_img_header.stamp
            bounding_boxes_msg.image_header = ros_img_header
            bounding_boxes_msg.image_topic = self.source_img_topic
            bounding_boxes_msg.image_width = self.img_width
            bounding_boxes_msg.image_height = self.img_height
            bounding_boxes_msg.bounding_boxes = bounding_box_msg_list
            if not rospy.is_shutdown():
                self.bounding_boxes_pub.publish(bounding_boxes_msg)
        found_object_msg = ObjectCount()
        found_object_msg.header.stamp = ros_img_header.stamp
        found_object_msg.count = len(detect_dict_list)
        if not rospy.is_shutdown():
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
