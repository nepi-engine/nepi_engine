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
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img

import rospy
import os
import time
import copy
import cv2
import threading



from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image

from nepi_ros_interfaces.msg import ImageStatus


#You must use the classes img_dict_lock.aquire() and img_dict_lock.release() thread save functions 
#When accessing the classes img_dict


EXAMPLE_IMG_INFO_DICT = {
    'has_mmap': False,
    'mmap_id': '',
    'mmap_info_dict': dict(),
    'depth_map_topic': None,
    'pointcloud_topic': None,
    'get_latency_time':0,
    'pub_latency_time':0,
    'process_time':0,
}

EXAMPLE_PC_DICT = {       
    'cv2_img': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_ros.get_time(),
    'ros_img_topic': 'None',
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp,
}




class ConnectImageIF:

    connected = False
    status_connected = False

    img_sub = None
    img_subscribed = False

    img_topic = None

    img_status_msg = None

    img_info_dict = None

    img_dict = None
    img_dict_lock = threading.Lock()

    get_img = False
    got_img = False



    #######################
    ### IF Initialization
    log_name = "ConnectImageIF"
    def __init__(self, 
                getImageCallbackFunction = None,
                imagePreprocessFunction = None,
                ):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting Initialization Processes")
        ##############################  
        self.getImageCallbackFunction = getImageCallbackFunction
        self.imagePreprocessFunction = imagePreprocessFunction
        #################################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Initialization Complete")

    

    #######################
    # Class Public Methods
    #######################

    def connect_image_topic(self,img_topic, timeout = 5):
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
                img_subscribed = self._subscribeImgTopic(img_topic)
                self.img_subscribed = img_subscribed
                if img_subscribed == True:
                    # Wait for image
                    nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for image")
                    connected = False
                    timer = 0
                    time_start = nepi_ros.get_time()
                    while connected == False and timer < timeout and not rospy.is_shutdown():
                        nepi_ros.sleep(.2)
                        connected = self.connected
                        timer = nepi_ros.get_time() - time_start
                    
                    if connected == True:
                        success = True
                        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Image Connected")
                    else:
                        self._unsubscribeImgTopic()
                        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
                else:
                    #################################
                    nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Subscribe")
            else:
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to find image topic: " + img_topic)
        return success


    def disconnect_image_topic(self):
        success = False
        if self.img_subscribed == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Already unsubsribed")
        else:
            #################################
            ## Subscribe to image if requested
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Unsubscribing to image topic: " + str(self.img_topic))
            img_unsubscribed = self._unsubscribeImgTopic()

            if img_unsubscribed == True:
                success = True
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Unsubsribe Succeeded")
            else:
                #################################
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Unsubscribe")

    def get_img_topic(self):
        return self.img_topic

    def get_info_dict(self):
        return self.img_info_dict

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Connected")
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for status connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.status_connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.status_connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to connect to status msg")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Status Connected")
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_ros.convert_msg2dict(self.img_status_msg)
        return self.img_status_dict



    

    def read_get_got_img_states(self):
        return self.get_img, self.got_img

    def set_get_img(self,state):
        self.get_img = state
        return True

    def get_img_dict(self):
        self.img_dict_lock.acquire()
        img_dict = copy.deepcopy(self.img_dict)
        self.img_dict = None
        self.img_dict_lock.release()
        return img_dict


    ###############################
    # Class Private Methods
    ###############################

    def _subscribeImgTopic(self,img_topic):
            if img_topic == "None" or img_topic == "":
                return False
            if self.img_sub is not None:
                success = self._unsubscribeImgTopic(img_topic)

            self.connected = False 
            # Create img info dict
            self.img_info_dict = dict()  
            self.img_info_dict['has_mmap'] = False
            self.img_info_dict['mmap_id'] = ""
            self.img_info_dict['mmap_info_dict'] = dict()

            self.img_info_dict['depth_map_topic'] = nepi_img.get_img_depth_map_topic(img_topic)
            self.img_info_dict['pointcloud_topic'] = nepi_img.get_img_pointcloud_topic(img_topic)

            self.img_info_dict['get_latency_time'] = 0
            self.img_info_dict['pub_latency_time'] = 0
            self.img_info_dict['process_time'] = 0 

          
            nepi_msg.publishMsgInfo(self,'Subsribing to image topic: ' + img_topic)
            self.img_sub = rospy.Subscriber(img_topic, Image, self._imageCb, queue_size=1, callback_args=(img_topic))
            self.img_topic = img_topic
            # Setup Image Status Topic Subscriber in case there is one
            self.status_connected = False
            self.status_msg = None
            status_topic = os.path.join(img_topic,'status')
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Subscribing to image status topic: " + status_topic)
            img_subscribed = rospy.Subscriber(status_topic, ImageStatus, self._imageStatusCb, queue_size=1, callback_args=(img_topic))
            time.sleep(1)
  
            return True
        

    def _unsubscribeImgTopic(self):
        success = False
        self.connected = False
        if self.img_sub is not None:
            nepi_msg.publishMsgWarn(self,'Unregistering topic: ' + str(self.img_topic))
            try:
                self.connected = False 
                self.img_sub.unregister()
                time.sleep(1)
                self.img_topic = None
                self.img_subscribed = False
                self.connected = False 
                self.img_sub = None
                self.img_dict = None
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to unregister img_sub:  " + str(e))
        return success


    def _imageCb(self,image_msg, args):      
        self.connected = True
        img_topic = args
        #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got img for topic:  " + img_topic)

        # Process ros image message
        current_time = nepi_ros.ros_time_now()
        ros_timestamp = image_msg.header.stamp
        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.img_info_dict['get_latency_time'] = latency
        #nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Get Img Latency: {:.2f}".format(latency))

        start_time = nepi_ros.get_time()   


        get_image = (self.getImageCallbackFunction is not None or self.get_img == True)
        #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got Img with get_img: " + str(self.get_img) + " got_img: " + str(self.got_img))
        if get_image == True:
            self.get_img = False
           # nepi_msg.publishMsgWarn(self,":" + self.log_name + ":Got get_img flag. Processing img for topic:  " + img_topic)
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

            if self.getImageCallbackFunction is not None:
                self.getImageCallbackFunction(img_dict)
            else:
                self.img_dict_lock.acquire()
                self.img_dict = img_dict
                self.img_dict_lock.release()
                self.got_img = True

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.img_info_dict['process_time'] = process_time

        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.img_info_dict['pub_latency_time'] = latency
        #nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Img Pub Latency: {:.2f}".format(latency))


    def _imageStatusCb(self,status_msg, args):      
        self.status_connected = True
        img_topic = args
        self.img_status_msg = status_msg