#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img

import rospy
import os
import time
import cv2
import threading

from rospy_message_converter import message_converter

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image





#You must use the classes img_dict_lock.aquire() and img_dict_lock.release() thread save functions 
#When accessing the classes img_dict
NONE_IMG_DICT = {       
    'cv2_img': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_ros.get_time(),
    'ros_img_topic': 'None',
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp
}



# Overlay text data on OpenCV image
font                   = cv2.FONT_HERSHEY_DUPLEX
fontScale, thickness  = nepi_img.optimal_font_dims(cv2_det_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
fontColor = (255, 255, 255)
fontColorBk = (0,0,0)
lineType = cv2.LINE_AA

class ImageIF:

    # Img Vars
    img_sub = None
    img_subscribed = False
    img_connected = False

    img_topic = None
    cur_img_topic = None

    img_info_dict = dict()

    img_dict = BLANK_IMG_DICT
    img_dict_lock = threading.Lock()

    get_img = False
    got_img = False

    status_img_size = (350, 700, 3)

    #######################
    ### IF Initialization
    log_name = "ImageIF"
    def __init__(self, 
                throttle_get_image_callback = False,
                imagePreprocessFunction = None
                ):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting Initialization Processes")
        ##############################  
        self.throttle = throttle_get_image_callback 
        self.imagePreprocessFunction = imagePreprocessFunction
        #################################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Initialization Complete")
    

    #######################
    # Class Public Methods


    def connectImgTopic(self,img_topic, timeout = 5):
        success = False
        if img_topic is None or img_topic == "None":
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Can't subscribe to None topic")
        else:
            # Try to find img topic
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for image on topic: " + str(img_topic))
            found_topic = nepi_ros.wait_for_topic(img_topic, timeout = timeout)
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Found topic: " + found_topic)
            if found_topic != "":
                # Subscribe to topic
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Subscribing to image topic: " + img_topic)
                img_subscribed = self.subscribeImgTopic(img_topic)
                self.img_subscribed = img_subscribed
                if img_subscribed == True:
                    # Wait for image
                    nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for image")
                    connected = False
                    timer = 0
                    time_start = nepi_ros.ros_time_now()
                    while connected == False and timer < timeout and not rospy.is_shutdown():
                        nepi_ros.sleep(.2)
                        connected = self.img_connected
                        timer = nepi_ros.ros_time_now() - time_start
                    
                    if connected == True:
                        success = True
                        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Image Connected")
                    else:
                        self.unsubscribeImgTopic()
                        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
                else:
                    #################################
                    nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Subscribe")
            else:
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to find image topic: " + img_topic)
        return success


    def disconnectImgTopic(self):
        success = False
        if self.img_subscribed == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Already unsubsribed")
        else:
            #################################
            ## Subscribe to image if requested
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Unsubscribing to image topic: " + str(self.img_topic))
            img_unsubscribed = self.unsubscribeImgTopic()

            if img_unsubscribed == True:
                success = True
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Unsubsribe Succeeded")
            else:
                #################################
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Unsubscribe")


    ###############################
    # Class Private Methods

    def subscribeImgTopic(self,img_topic):
            if img_topic == "None" or img_topic == "":
                return False
            if self.img_sub is not None:
                success = self.unsubscribeImgTopic(img_topic)

            self.img_connected = False 
            # Create img info dict
            self.img_info_dict = dict()  
            self.img_info_dict['has_mmap'] = False
            self.img_info_dict['mmap_id'] = ""
            self.img_info_dict['mmap_info_dict'] = dict()
            self.img_info_dict['image_latency_time'] = 0
            self.img_info_dict['preprocess_time'] = 0 
            self.img_info_dict['get_latency_time'] = 0
            self.img_info_dict['pub_latency_time'] = 0

            nepi_msg.publishMsgInfo(self,'Subsribing to image topic: ' + img_topic)
            self.img_sub = rospy.Subscriber(img_topic, Image, self.imageCb, queue_size=1, callback_args=(img_topic))
            self.img_topic = img_topic
            time.sleep(1)
  
            return True
        

    def unsubscribeImgTopic(self):
        success = False
        self.img_connected = False
        if self.img_sub is not None:
            nepi_msg.publishMsgWarn(self,'Unregistering topic: ' + str(self.img_topic))
            try:
                self.img_connected = False 
                self.img_sub.unregister()
                time.sleep(1)
                self.img_topic = None
                self.img_subscribed = False
                self.img_connected = False 
                self.img_sub = None
                self.img_dict = None
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to unregister img_sub:  " + str(e))
        return success


    def imageCb(self,image_msg, args):      
        self.img_connected = True
        img_topic = args
        #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got img for topic:  " + img_topic)
        '''
        # Check if topic has memory mapping
        has_mmap = False
        if self.img_info_dict['has_mmap'] is None:
            mmap_id = nepi_mmap.get_mmap_id_from_topic(img_topic)
            mmap_exists = nepi_mmap.check_for_mmap(mmap_id)
            if mmap_exists == True:
                [success, msg, info_dict]  = nepi_mmap.get_cv2img_mmap_info(mmap_id)
                if info_dict is not None:
                    has_mmap = True
                    self.img_info_dict['has_mmap'] = True
                    self.img_info_dict['mmap_id'] = mmap_id
                    self.img_info_dict['mmap_info_dict'] = info_dict
        else:
            has_mmap = self.img_info_dict['has_mmap']
        if has_mmap == True:
            # Start get mmap image thread
            pass
        else:
        '''
        # Process ros image message
        current_time = nepi_ros.ros_time_now()
        ros_timestamp = image_msg.header.stamp
        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.img_info_dict['get_latency_time'] = latency
        #nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Get Img Latency: {:.2f}".format(latency))

        start_time = nepi_ros.get_time()   


        get_image = (self.get_img == True or self.throttle == False)
        #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got Img with get_img: " + str(self.get_img) + " got_img: " + str(self.got_img))
        if get_image == True:

            nepi_msg.publishMsgWarn(self,":" + self.log_name + ":Got get_img flag. Processing img for topic:  " + img_topic)
            ##############################
            ### Preprocess Image
            
            cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

            if self.imagePreprocessFunction is not None:
                try:
                    cv2_img = self.imagePreprocessFunction(cv2_img)
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Provided Image Preprocess Function failed:  " + str(e))

            cv2_img = self.imagePreprocessFunction(cv2_img)
            img_dict = dict()
            img_dict['cv2_img'] = cv2_img
            height, width = cv2_img.shape[:2]
            img_dict['width'] = width 
            img_dict['height'] = height 
            ros_timestamp = image_msg.header.stamp
            img_dict['timestamp'] = nepi_ros.sec_from_ros_time(ros_timestamp)
            img_dict['ros_img_topic'] = img_topic
            img_dict['ros_img_header'] = image_msg.header
            img_dict['ros_img_stamp'] = ros_timestamp
            ##############################

            self.img_dict_lock.acquire()
            self.img_dict = img_dict
            self.img_dict_lock.release()
            self.get_img = False
            self.got_img = True

            preprocess_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.img_info_dict['preprocess_time'] = preprocess_time

        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.img_info_dict['pub_latency_time'] = latency
        #nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Img Pub Latency: {:.2f}".format(latency))

    '''
    def imageMmapThread(self,img_topic, mmap_id):    
        mmap_list = nepi_mmap.get_mmap_list()
        while img_topic in self.imgs_pub_sub_dict.keys() and mmap_id in mmap_list and not rospy.is_shutdown():
            current_time = nepi_ros.get_time_now()
            mmap_list = nepi_mmap.get_mmap_list()

            start_time = nepi_ros.get_time()   

            get_image = (self.get_img == True or self.throttle_image == False)
            #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Callback got image from topic:  " + img_topic + " with get topic " + str(self.get_img))
            if get_image == True:
                self.get_img = False

                if mmap_id in mmap_list:
                    # Get cv2 data
                    unlocked = nepi_mmap.wait_for_unlock_mmap(mmap_id, timeout = 1)
                    if unlocked == True:
                        locked = nepi_mmap.lock_mmap(mmap_id)
                        if locked == True:
                            mmap_read_response = nepi_mmap.read_cv2img_mmap_data(mmap_id)
                            [success, msg, cv2_img, img_encoding, timestamp, latency_sec] = mmap_read_response
                            if success:
                                latency = current_time - timestamp
                                self.img_info_dict['image_latency_time'] = latency
                                #nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Detect Pub Latency: {:.2f}".format(latency))



                                ### Preprocess Image
                                
                                cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                                if self.imagePreprocessFunction is not None:
                                    try:
                                        cv2_img = self.imagePreprocessFunction(cv2_img)
                                    except Exception as e:
                                        nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Provided Image Preprocess Function failed:  " + str(e))

                                cv2_img = self.imagePreprocessFunction(cv2_img)
                                img_dict = dict()
                                img_dict['cv2_img'] = cv2_img
                                height, width = cv2_img.shape[:2]
                                img_dict['width'] = width 
                                img_dict['height'] = height 

                                ros_header = Header()
                                ros_header.stamp = nepi_ros.ros_stamp_from_sec(timestamp)
                                img_dict = self.imagePreprocessFunction(cv2_img, options_dict)
                                img_dict['timestamp'] = timestamp
                                img_dict['ros_img_topic'] = img_topic
                                img_dict['ros_img_header'] = ros_header
                                img_dict['ros_img_stamp'] = ros_header.stamp
                                ##############################

                                self.img_dict_lock.acquire()
                                self.img_dict = img_dict
                                self.img_dict_lock.release()
                                self.got_img = True

                                preprocess_time = round( (nepi_ros.get_time() - start_time) , 3)
                                self.img_info_dict['preprocess_time'] = preprocess_time

                                unlocked = nepi_mmap.unlock_mmap(mmap_id)
                            else:
                                self.img_info_dict['has_mmap'] = True
                                break
                        else:
                        time.sleep(.01)
    '''





