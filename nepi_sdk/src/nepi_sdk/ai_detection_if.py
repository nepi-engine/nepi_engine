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

from rospy_message_converter import message_converter

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes, AiDetectorStatus
from nepi_ros_interfaces.msg import AiDetectrorInfo
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, 
from nepi_ros_interfaces.srv import AiMgrActiveModelsInfoQuery, AiMgrActiveModelsInfoQueryResponse
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img



#You must use the classes img_dict_lock.aquire() and img_dict_lock.release() thread save functions 
#When accessing the classes img_dict

BLANK_IMG_DICT = {       
    'cv2_img': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_ros.get_time(),
    'ros_img_topic': 'None'
    'ros_img_header': Header()
    'ros_img_stamp': Header().stamp
}

EXAMPLE_DETECTION_DICT_ENTRY = {
    'name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'prob': .3, # Probability of detection
    'xmin': 10,
    'ymin': 10,
    'xmax': 100,
    'ymax': 100,
    'area_ratio': 20.2,
    'area_pixels': 8100
}

class AiDetectionIF:

    # AI Mgr Vars
    AI_MGR_NODE_NAME = 'ai_model_mgr'
    MODEL_TYPE_LIST = ['detection']

    ai_mgr_connected = False
    ai_mgr_status_msg = None
    ai_models_info_query_service = None
    ai_models_info_response = None
    ai_models_info_dict = dict()
    ai_dets_info_dict = dict()



    # AI Detector Vars
    det_connected = False
    det_namespace = "None"
    det_base_namespace = "None"


    det_status_sub = None
    det_status_msg = None
    det_info_msg = None
    det_info_dict = dict()

    det_img_topic = None
    det_img_width = 0
    det_img_height = 0
    det_count = 0
    det_dict_list = None

    found_obj_sub = None
    found_object_msg = None
    found_object_count = 0
    bounding_boxes_sub = None
    bounding_boxes_msg = None

    first_det_time = None
    last_det_time = None

    # Img Vars
    img_subscribed = False
    img_connected = False
    img_topic = None
    cur_img_topic = None
    img_sub = None
    img_connected = False

    img_info_dict = dict()

    img_dict = BLANK_IMG_DICT
    img_dict_lock = threading.Lock()

    get_img = False
    got_img = False

    status_img_size = (350, 700, 3)

    #######################
    ### IF Initialization
    def __init__(self, 
                connect_timeout = float('inf'),
                det_namespace = None,
                auto_connect_source_image = True,
                throttle_get_image_callback = False,
                imagePreprocessFunction = None
                ):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ##############################  
        self.connect_timeout = connect_timeout
        self.connect_image = auto_connect_source_image
        self.throttle_image = throttle_get_image_callback 
        self.imagePreprocessFunction = imagePreprocessFunction
                
        #################################
        ## Connect to AI model manager
        mgr_connected = False
        nepi_msg.publishMsgInfo(self,"Initializing AI Model Mgr Connection")
        mgr_connected = self.init_ai_mgr_connection(timeout = self.connect_timeout)
        if mgr_connected == False:
            nepi_msg.publishMsgError(self,"Failed connect to AI Model Mgr. Exiting Detection IF")
        self.mgr_connected = mgr_connected
                
        #################################
        ## Connect to Detector if requested
        det_connected = False
        if det_namespace is not None:
            nepi_msg.publishMsgInfo(self,"Initializing Detector: " + det_namespace)
            det_connected = self.register_detector(det_namespace, timeout = self.connect_timeout)
            if det_connected == False:
                nepi_msg.publishMsgError(self,"Failed connect to Detector. Exiting Detection IF: " + str(det_namespace))
        self.det_connceted = det_connected

        #################################
        ## Subscribe to image if requested
        img_subscribed = False
        if self.connect_image == True:
            if self.det_img_topic is None:
                nepi_msg.publishMsgInfo(self,"No image topic found for Detector: " + det_namespace)
            nepi_msg.publishMsgInfo(self,"Subscribing to image Detector: " + self.det_img_topic)
            img_subscribed = self.subscribeImgTopic(self.det_img_topic, timeout = self.connect_timeout)
            if img_subscribed == False:
                nepi_msg.publishMsgError(self,"Failed connect to Image Topic. " + str(self.det_img_topic))
        self.img_subscribed = img_subscribed

        #################################
        nepi_msg.publishMsgInfo(self,"Detection IF Initialization Complete")
        


    #######################
    # AI Model Manager IF Functions


    def init_ai_mgr_connection(self, timeout = 5):
        self.ai_mgr_namespace = AI_MGR_NODE_NAME
        nepi_msg.publishMsgInfo(self,"Initialization AI Manager Processes")
        mgr_namespace = os.path.join(self.base_namespace,self.ai_mgr_namespace)

        # Subscribe to status topic
        status_topic = os.path.join(mgr_namespace,"status")
        nepi_msg.publishMsgInfo(self,"Waiting for AI Model Mgr status topic: " + status_topic)
        nepi_ros.wait_for_topic(status_topic, timeout )
        nepi_msg.publishMsgInfo(self,"Starting AI Model Mgr status")
        self.ai_mgr_status_msg = None
        rospy.Subscriber(status_topic, AiModelMgrStatus, self.ai_mgr_status_callback, queue_size=None)
        nepi_msg.publishMsgInfo(self,"Waiting for AI Model Mgr status message")
        while self.ai_mgr_status_msg is None and not rospy.is_shutdown():
            nepi_ros.sleep(1)

        # Create Model Info Service Listener
        get_info_service = os.path.join(det_namespace,'active_models_info_query')
        nepi_msg.publishMsgInfo(self,"Waiting for AI Model Mgr active models info query service " + get_info_service)
        rospy.wait_for_service(get_info_service, timeout )
        nepi_msg.publishMsgInfo(self,"Calling AI Model Mgr active models info query service " + get_info_service)
        try:
            self.ai_models_info_query_service = rospy.ServiceProxy(get_info_service, AiDetectorInfoQuery)
            time.sleep(1)
            success = True
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain AI Model Mgr active models info query service " + str(e))
            success = False
        if success = True
            nepi_msg.publishMsgInfo(self,"Calling AI Model Mgr active models info query service " + get_info_service)
            req = AiMgrActiveModelsInfoQueryRequest()
            try:
                response = self.ai_models_info_query_service(req)
                nepi_msg.publishMsgInfo(self,"Got AI Model Mgr active models info response" + response)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to obtain AI Model Mgr active models info response " + str(e))
                return False
        rospy.Timer(rospy.Duration(1), self.aiMgrModelInfoServiceCb, oneshot = True)

        nepi_msg.publishMsgInfo(self,"AI Model Mgr Initialization Complate")
        return True


    def ai_mgr_status_callback(self,status_msg):
        self.ai_mgr_status_msg = status_msg
        self.ai_mgr_status_dict = message_converter.convert_ros_message_to_dictionary(status_msg)


    def aiMgrModelInfoServiceCb(self,_):
        if self.ai_models_info_query_service is not None:
            req = AiMgrActiveModelsInfoQueryRequest()
            try:
                response = self.ai_models_info_query_service(req)
                self.ai_models_info_response = response
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to obtain active models info response " + str(e))
            if response is not None:
                if "detector_name_list" in response.keys(detector_name_list):
                    det_namespaces = response["detector_name_list"]
                    for i, namespace in enumerate(det_namesapces):
                        self.ai_dets_info_dict[namespace] = response["detector_info_list"][i]
        rospy.Timer(rospy.Duration(5), self.aiMgrModelInfoServiceCb, oneshot = True)


    #######################
    # Detection IF Functions

    def get_available_detector_namespaces(self):
        namespaces = []
        if self.ai_dets_info_dict is not None:
            for base_namespace in self.ai_dets_info_dict.keys():
                model_info_msg = self.ai_dets_info_dict[base_namespace]
                namespace = namespace + model_info_msg['image_detector_namespaces']
        return namespaces

    def get_detector_base_namespace(self,det_namespace):
        det_base_namespace = None
        if self.dets_info_dict is not None:
            namespace_list = list(self.dets_info_dict.keys())
        for namespace in namespace_list:
            if det_namespace.find(namespace) != -1:
                det_base_namespace = namespace
        return det_base_namespace

    def register_detector(self, det_namespace, timeout = 5):
        nepi_msg.publishMsgInfo(self,"Initialization Detection Processes")
        success = False
        if det_namespace[-1] == '/'
            det_namespace = det_namespace[0:-1]
        
        avail_dets = self.get_available_detector_namespaces()
        found_det = False:
            for det in avail_dets:
                if det.find(det_namespace) != -1:
                    found_det = True
        if found_det == False:
            nepi_msg.publishMsgWarn(self,"Specified Detector not valid, Exiting registration process")
            nepi_msg.publishMsgWarn(self,"Current Valid Detectors are: " + str(avail_dets))
            return False


        try:
            check_namespace = self.det_namesapce
            det_exists = True
        except:
            det_exists = False

        if det_exists == True:
            if det_namespace == self.det_namespace:
                nepi_msg.publishMsgWarn(self,"Specified Detector already connected, Exiting registration process")
                return True
            else:
                nepi_msg.publishMsgWarn(self,"Different Detector already connected, unregistering current detector first")
                success = unregister_detector(self.det_namespace)
                if success == False:
                    nepi_msg.publishMsgWarn(self,"Failed to unregister current detector: " str(self.det_namespace))

        
        det_base_namespace = self.get_det_namespace(det_namespace)
        if det_base_namespace is None:
            nepi_msg.publishMsgWarn(self,"Failed to find base namespace for detector: " str(det_namespace))
            return False


        if det_namespace == det_base_namespace:
            nepi_msg.publishMsgWarn(self,"Detector Namespace can not match Detector Base Namespace " + str(det_base_namespace))
            valid_dets = []
            for det in avail_dets:
                if det.find(det_namesapce) != -1:
                    valid_dets.append(det)
            nepi_msg.publishMsgWarn(self,"Current Valid Detectors for Specified Detector are: " + str(valid_dets))
            if len(valid_dets) > 0:
                nepi_msg.publishMsgWarn(self,"Will connect to first available Detector: " + str(valid_dets[0]))
                det_namespace = valid_dets[0]
            else:
                nepi_msg.publishMsgWarn(self,"Selected Namespace has no valid Detectors")
                return False


        self.det_namespace = det_namespace
        self.det_base_namespace = det_base_namespace

        ## Setup Settings Callback
        status_topic = os.path.join(det_base_namespace,"status")
        nepi_msg.publishMsgInfo(self,"Waiting for topic: " + status_topic)
        nepi_ros.wait_for_topic(status_topic, timeout )
        nepi_msg.publishMsgInfo(self,"Starting detector status scubscriber callback")
        self.det_status_sub = rospy.Subscriber(status_topic, AiDetectorStatus, detector_status_callback, queue_size=None, callback_args=(det_base_namespace))
        nepi_msg.publishMsgInfo(self,"Waiting for status message")
        success = False
        timer = 0
        time_start = time.ros_time_now()
        connected = False
        while connected == False and timer > timeout and not rospy.is_shutdown():
            nepi_ros.sleep(.2)
            connected = self.det_connected
            timer = time.ros_time_now() - time_start
        if self.det_connected == False:
            nepi_msg.publishMsgWarn(self,"Status msg topic subscribe timedout " + str(status_topic))
            nepi_msg.publishMsgWarn(self,"Unregistering subscribe topic " + str(det_base_namespace))
            unregister_detector(self,det_base_namespace)
            time.sleep(1)
            return False

        # Call Detector Info Service Query
        get_info_service = os.path.join(det_base_namespace,'detector_info_query')
        nepi_msg.publishMsgInfo(self,"Waiting for detector info query service " + get_info_service)
        rospy.wait_for_service(get_info_service, timeout )
        nepi_msg.publishMsgInfo(self,"Calling detector info query service " + get_info_service)
        try:
            info_query_service = rospy.ServiceProxy(get_info_service, AiDetectorInfoQuery)
            time.sleep(1)
            success = True
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain detector info query service " + str(e))
            success = False
        if success = True
            nepi_msg.publishMsgInfo(self,"Getting detector info query service " + get_info_service)
            req = AiDetectorInfoQueryRequest()
            try:
                response = info_query_service(req)
                self.det_info_msg = response
                self.det_img_width = response.proc_img_width
                self.det_img_height = response.proc_img_height
                self.status_img_size = (self.det_img_height, self.det_img_width, 3)
                self.det_classes = response.classes
                nepi_msg.publishMsgInfo(self,"Got detector info response" + response)
            except Exception as e:
                self.det_info_msg = None
                nepi_msg.publishMsgWarn(self,"Failed to obtain detector info service response " + str(e))
                nepi_msg.publishMsgWarn(self,"Unregistering subscribe topic " + str(det_base_namespace))
                unregister_detector(self,det_namespace)
                time.sleep(1)
                return False

        #Setup Det Subscribers
        FOUND_OBJECT_TOPIC = os.path.join(det_namespace,"found_object")
        self.found_obj_sub = rospy.Subscriber(FOUND_OBJECT_TOPIC, ObjectCount, foundObjectCb, queue_size = 1)
        BOUNDING_BOXES_TOPIC = sos.path.join(det_namespace,"bounding_boxes")
        self.bounding_boxes_sub = rospy.Subscriber(BOUNDING_BOXES_TOPIC, BoundingBoxes, objectDetectedCb, queue_size = 1)

        self.img_topic = self.det_status_msg.
        if self.auto_connect_source_image == True:

        
        nepi_msg.publishMsgInfo(self,"Detection Registration Complete")
        return success



    def unregister_detector(self,det_namespace)
        try:
            if self.det_status_sub is not None:
                self.det_connected = False

                self.det_status_msg = None
                self.det_status_sub.unregister()
                self.found_obj_msg = None
                self.found_obj_sub.unregister()
                self.bounding_boxes_msg = None
                self.bounding_boxes_sub.unregister()
                time.sleep(1)
                self.det_status_sub = None
                self.found_obj_sub = None
                self.bounding_boxes_sub = None
                
            success = True
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to unregister detector: " + str(e))
            success = False
        return success


    def detector_status_callback(self, status_msg):
        self.det_connected = True
        self.det_status_msg = status_msg

        # Create some useful class objects from status message
        self.det_namespaces = self.det_status_msg.image_detector_namespaces
        self.det_img_topics = self.det_status_msg.image_source_topics
        det_index = dets_self.det_namespaces.index(self.det_namespace)
        if det_index == -1:
            det_img_topic = None
            nepi_msg.publishMsgWarn(self,"Failed to detector namespace in status detect namespaces: " + str(self.det_namespaces))
        else:
            self.det_img_topic = self.det_img_topics[det_index]
        
        # Convert the status msg to a staus dict
        dets_status_dict = None
        try:
            dets_status_dict = message_converter.convert_ros_message_to_dictionary(status_msg)
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to convert detector status msg to dict: " + str(e))
        if dets_status_dict is not None:
            det_status_dict = dict()
            for key in dets_status_dict:
                entry = dets_status_dict[key]
                det_status_dict[key] = entry




    ### Monitor Output of AI detector to clear detection status
    def foundObjectCb(self,found_obj_msg):
        self.found_obj_msg = found_obj_msg
        self.det_count = found_obj_msg.found_object_count

    def objectDetectedCb(self,bounding_boxes_msg):
        self.object_detect_msg = bounding_boxes_msg
        bb_list = bounding_boxes_msg.bounding_boxes
        det_dict_list = []
        for bb in bb_list:
            bb_dict = {
                'name': bb.Class, 
                'id': bb.id, 
                'uid': bb.uid, 
                'prob': bb.probability, 
                'xmin': bb.xmin,
                'ymin': bb.ymin,
                'xmax': bb.xmax,
                'ymax': bb.ymax,
                'area_ratio': bb.area_ratio,
                'area_pixels': bb.area_pixels
            }
            det_dict_list.append(bb_dict)
        self.det_dict_list = det_dict_list

        det_time = nepi_ros.get_time()
        if self.first_det_time is None:
            self.first_det_time = det_time
        self.last_det_time = det_time

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


    #######################
    # Source Image IF Functions


    def subscribeImgTopic(self,img_topic, timeout = 5):
        if img_topic is not None:
            if img_topic == "None" or img_topic == "":
                return False
            if self.img_sub is not None:
                success = self.unsubscribeImgTopic(img_topic)

            self.img_connected = False 
            # Create img info dict
            self.img_info_dict = dict()  
            self.img_info_dict['has_mmap'] = None
            self.img_info_dict['mmap_id'] = ""
            self.img_info_dict['mmap_info_dict'] = dict()
            self.img_info_dict['image_latency_time'] = 0
            self.img_info_dict['preprocess_time'] = 0 

            nepi_msg.publishMsgInfo(self,'Subsribing to image topic: ' + img_topic)
            self.img_sub = rospy.Subscriber(img_topic, Image, self.imageCb, queue_size=1, callback_args=(img_topic))
            time.sleep(1)
            self.last_img_topic = img_topic
  
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
                self.img_sub = None
                self.img_dict = BLANK_IMG_DICT
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to unregister img_sub:  " + str(e))
        return success

   def preprocessImage(self,cv2_img):
        if self.det_img_width > 0 and self.det_img_height > 0:
            cv2_img = nepi_img.resize_proportionally(cv2_img, self.det_img_width,self.det_img_height,interp = cv2.INTER_NEAREST)
        
        # Convert BW image to RGB
        if nepi_img.is_gray(cv2_img):
            cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_GRAY2BGR)

        return cv2_img


    def imageCb(self,image_msg, args):      
        self.img_connected = True
        img_topic = args
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
        self.img_info_dict['image_latency_time'] = latency
        #nepi_msg.publishMsgInfo(self,"Detect Pub Latency: {:.2f}".format(latency))

        start_time = nepi_ros.get_time()   


        get_image = (self.get_img == True or self.throttle_image == False)
        #nepi_msg.publishMsgWarn(self,"Callback got image from topic:  " + img_topic + " with get topic " + str(self.get_img))
        if get_image == True:
            self.get_img = False

            #nepi_msg.publishMsgWarn(self,"Processing img for topic:  " + img_topic)
            ##############################
            ### Preprocess Image
            
            cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

            if self.imagePreprocessFunction is not None:
                try:
                    cv2_img = self.imagePreprocessFunction(cv2_img)
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,"Provided Image Preprocess Function failed:  " + str(e))

            cv2_img = self.preprocessImage(cv2_img)
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
            self.got_img = True

            preprocess_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.img_info_dict['preprocess_time'] = preprocess_time

    '''
    def imageMmapThread(self,img_topic, mmap_id):    
        mmap_list = nepi_mmap.get_mmap_list()
        while img_topic in self.imgs_pub_sub_dict.keys() and mmap_id in mmap_list and not rospy.is_shutdown():
            current_time = nepi_ros.get_time_now()
            mmap_list = nepi_mmap.get_mmap_list()

            start_time = nepi_ros.get_time()   

            get_image = (self.get_img == True or self.throttle_image == False)
            #nepi_msg.publishMsgWarn(self,"Callback got image from topic:  " + img_topic + " with get topic " + str(self.get_img))
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
                                #nepi_msg.publishMsgInfo(self,"Detect Pub Latency: {:.2f}".format(latency))



                                ### Preprocess Image
                                
                                cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                                if self.imagePreprocessFunction is not None:
                                    try:
                                        cv2_img = self.imagePreprocessFunction(cv2_img)
                                    except Exception as e:
                                        nepi_msg.publishMsgWarn(self,"Provided Image Preprocess Function failed:  " + str(e))

                                cv2_img = self.preprocessImage(cv2_img)
                                img_dict = dict()
                                img_dict['cv2_img'] = cv2_img
                                height, width = cv2_img.shape[:2]
                                img_dict['width'] = width 
                                img_dict['height'] = height 

                                ros_header = Header()
                                ros_header.stamp = nepi_ros.ros_stamp_from_sec(timestamp)
                                img_dict = self.preprocessImage(cv2_img, options_dict)
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

    def getImgShortName(self, det_namespace):
        short_name = det_namespace.replace(self.base_namespace,"")
        if short_name.find("idx") != -1:
            short_name = short_name.replace("/idx","")
        return short_name



