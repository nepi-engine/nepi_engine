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

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes, AiDetectorStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, AiDetectorInfoQuery, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_img
from nepi_sdk import nepi_mmap
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


DEFAULT_WAIT_FOR_DETECT = False

DEFAULT_IMG_TILING = False

DEFAULT_LABELS_OVERLAY = True
DEFAULT_CLF_OVERLAY = False
DEFAULT_IMG_OVERLAY = False



GET_IMAGE_TIMEOUT_SEC = 1 

class AiDetectorIF:

    BACKUP_MODEL_CONFIG_FOLDER = "/mnt/nepi_storage/user_cfg/ros" 
    model_config_folder = BACKUP_MODEL_CONFIG_FOLDER

    BACKUP_SDK_LIB_FOLDER = "/opt/nepi/ros/lib/nepi_sdk"
    sdk_lib_folder = BACKUP_SDK_LIB_FOLDER

    data_products = ['bounding_boxes','detection_image']

    node_namespace = ""
    config_file_path = ""
    self_managed = True
    model_name = "None"


    init_selected_img_topics = []
    init_wait_for_detect = DEFAULT_WAIT_FOR_DETECT

    init_img_tiling = DEFAULT_IMG_TILING

    init_overlay_labels = DEFAULT_LABELS_OVERLAY
    init_overlay_clf_name = DEFAULT_CLF_OVERLAY
    init_overlay_img_name = DEFAULT_IMG_OVERLAY
    init_threshold = DEFUALT_THRESHOLD
    init_max_rate = DEFAULT_MAX_RATE

    init_enabled = False

    last_detect_time = nepi_ros.get_time_sec()
    imgs_pub_sub_dict = dict()
    imgs_pub_sub_lock = threading.Lock()
    imgs_info_dict = dict()
    imgs_lock_dict = dict()

    has_tiling = False
    save_cfg_if = None

    state = 'Loading'

    cur_img_topic = "None"
    get_img_topic = "None"
    last_get_image_time = 0
    got_img_topic = "None"

    img_dict = None
    img_dict_lock = threading.Lock()

    first_detect_complete = False

    
    def __init__(self, 
                model_name, 
                framework, 
                description, 
                proc_img_height, 
                proc_img_width,  
                classes_list, 
                defualt_config_dict, 
                all_namespace, 
                preprocessImageFunction, 
                processDetectionFunction,
                has_img_tiling = False):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ##############################  
        if all_namespace.find(self.node_name) == -1:
            self.self_managed = False

        # Set up folders
        get_folder_name_service = self.base_namespace + 'system_storage_folder_query'
        nepi_msg.publishMsgInfo(self,"Waiting for system folder query service " + get_folder_name_service)
        rospy.wait_for_service(get_folder_name_service)
        nepi_msg.publishMsgInfo(self,"Calling system folder query service " + get_folder_name_service)
        try:
            folder_query_service = rospy.ServiceProxy(get_folder_name_service, SystemStorageFolderQuery)
            time.sleep(1)
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain folder query service " + str(e))
        try:
            nepi_msg.publishMsgInfo(self,"Getting User Config folder query service " + get_folder_name_service)
            response = folder_query_service("user_cfg/ros")
            nepi_msg.publishMsgInfo(self,"Got User Config config folder path" + response.folder_path)
            self.model_config_folder = response.folder_path
        except Exception as e:
            self.model_config_folder = self.BACKUP_MODEL_CONFIG_FOLDER
            nepi_msg.publishMsgWarn(self,"Failed to obtain AI Model config folder, falling back to: " + self.BACKUP_MODEL_CONFIG_FOLDER + " " + str(e))
        try:
            nepi_msg.publishMsgInfo(self,"Getting NEPI SDK Lib folder query service " + get_folder_name_service)
            response = folder_query_service("sdk")
            nepi_msg.publishMsgInfo(self,"Got NEPI SDK Lib folder path" + response.folder_path)
            self.sdk_lib_folder = response.folder_path
        except Exception as e:
            self.sdk_lib_folder = self.BACKUP_SDK_LIB_FOLDER
            nepi_msg.publishMsgWarn(self,"Failed to obtain NEPI SDK Lib folder, falling back to: " + self.BACKUP_SDK_LIB_FOLDER + " " + str(e))


        self.model_name = model_name
        self.model_framework = framework
        self.model_type = 'detection'
        self.model_description = description
        self.model_proc_img_height = proc_img_height
        self.model_proc_img_width = proc_img_width
        self.defualt_config_dict = defualt_config_dict
        if all_namespace[-1] == "/":
            all_namespace = all_namespace[:-1]
        self.all_namespace = all_namespace
        self.preprocessImage = preprocessImageFunction
        self.processDetection = processDetectionFunction
        self.classes_list = classes_list

        # Create a blank image to initialize detector
        nepi_msg.publishMsgWarn(self,"Initializing detector with blank image")
        options_dict = dict()
        cv2_test_imge = nepi_img.create_cv2_test_img(1000,600)
        img_dict = self.preprocessImage(cv2_test_imge, options_dict)
        img_dict['ros_img_topic'] = 'Test_Img'
        img_dict['ros_img_header'] = ""
        img_dict['ros_img_stamp'] = nepi_ros.ros_time_now()
        detect_dict_list = self.processDetection(img_dict, 0.5)
        nepi_msg.publishMsgWarn(self,"Detector initializing complete")
                 
        nepi_msg.publishMsgInfo(self,"Starting IF setup")

        # Load/Create model config params
        self.node_namespace = os.path.join(self.base_namespace,"ai",self.node_name)
        self.config_file_path = os.path.join(self.model_config_folder,self.node_name)
        nepi_ros.load_config_file(self.config_file_path, defualt_config_dict, self.node_namespace)
        nepi_msg.publishMsgWarn(self,"Loaded AI Detector params")
        nepi_ros.print_node_params(self)
        
        # Create Publishers

        self.found_object_pub = rospy.Publisher('~found_object', ObjectCount,  queue_size = 1)
        self.bounding_boxes_pub = rospy.Publisher('~bounding_boxes', BoundingBoxes, queue_size = 1)
        self.detection_trigger_pub = rospy.Publisher('~detection_trigger', Bool,  queue_size = 1)
        self.detection_state_pub = rospy.Publisher('~detection_state', Bool,  queue_size = 1)
        self.all_namespace = os.path.join(self.base_namespace,'ai/all_detectors')
        nepi_msg.publishMsgInfo(self,"Staring all detectors on namespace " + self.all_namespace)
        self.found_object_all_pub = rospy.Publisher(self.all_namespace + '/found_object', ObjectCount,  queue_size = 1)
        self.bounding_boxes_all_pub = rospy.Publisher(self.all_namespace + '/bounding_boxes', BoundingBoxes, queue_size = 1)
        self.detection_trigger_all_pub = rospy.Publisher(self.all_namespace + '/detection_trigger', Bool,  queue_size = 1)
  
        self.status_pub = rospy.Publisher("~status", AiDetectorStatus,  queue_size = 1, latch = True)
        time.sleep(1)


    # Create Services
        rospy.Service('detector_info_query', AiDetectorInfoQuery,
                self.handleInfoRequest)

        # Create Subscribers
        rospy.Subscriber('~add_img_topic', String, self.addImageTopicCb, queue_size=10)
        rospy.Subscriber('~add_img_topics', StringArray, self.addImageTopicsCb, queue_size=10)
        rospy.Subscriber('~remove_img_topic', String, self.removeImageTopicCb, queue_size=10)
        rospy.Subscriber('~remove_img_topics', StringArray, self.removeImageTopicsCb, queue_size=10)
        self.has_img_tiling = has_img_tiling
        if self.has_img_tiling == True:
            rospy.Subscriber('~set_img_tiling', Bool, self.setTileImgCb, queue_size=10)
        rospy.Subscriber('~set_overlay_labels', Bool, self.setOverlayLabelsCb, queue_size=10)
        rospy.Subscriber('~set_overlay_clf_name', Bool, self.setOverlayClfNameCb, queue_size=10)
        rospy.Subscriber('~set_overlay_img_name', Bool, self.setOverlayImgNameCb, queue_size=10)          
        rospy.Subscriber('~set_threshold', Float32, self.setThresholdCb, queue_size=10)
        rospy.Subscriber('~set_max_rate', Float32, self.setMaxRateCb, queue_size=10)

        rospy.Subscriber('~set_sleep_enable', Bool, self.setSleepEnableCb, queue_size=10) 
        rospy.Subscriber('~set_sleep_suspend_sec', Int32, self.setSleepSuspendTimeCb, queue_size=10)
        rospy.Subscriber('~set_sleep_run_sec', Int32, self.setsetSleepSuspendTimeCb, queue_size=10)

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
        rospy.Timer(rospy.Duration(.1), self.updateDetectionTopicCb, oneshot = True)

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
        nepi_ros.set_param(self,'~detector/img_topics', [])

        nepi_ros.set_param(self,'~detector/img_tiling', DEFAULT_IMG_TILING)

        nepi_ros.set_param(self,'~detector/overlay_labels', DEFAULT_LABELS_OVERLAY)
        nepi_ros.set_param(self,'~detector/overlay_clf_name', DEFAULT_CLF_OVERLAY)
        nepi_ros.set_param(self,'~detector/overlay_img_name', DEFAULT_IMG_OVERLAY)
        
        nepi_ros.set_param(self,'~detector/threshold', self.defualt_config_dict['threshold'])
        nepi_ros.set_param(self,'~detector/max_rate', self.defualt_config_dict['max_rate'])
        nepi_ros.set_param(self,'~detector/enabled', False)

    def resetParamServer(self,do_updates = False):
        nepi_msg.publishMsgInfo(self,"Reseting Param Server")
        nepi_ros.set_param(self,'~detector/img_topics', self.init_selected_img_topics)

        nepi_ros.set_param(self,'~detector/wait_for_detect', self.init_wait_for_detect)
        nepi_ros.set_param(self,'~detector/img_tiling', self.init_img_tiling)

        nepi_ros.set_param(self,'~detector/overlay_labels',self.init_overlay_labels)
        nepi_ros.set_param(self,'~detector/overlay_clf_name', self.init_overlay_clf_name)
        nepi_ros.set_param(self,'~detector/overlay_img_name', self.init_overlay_img_name)

        nepi_ros.set_param(self,'~detector/threshold', self.init_threshold)
        nepi_ros.set_param(self,'~detector/max_rate', self.init_max_rate)
        nepi_ros.set_param(self,'~detector/enabled', self.init_enabled)
        if do_updates:
            self.updateFromParamServer()

    def initializeParamServer(self,do_updates = False):
        nepi_msg.publishMsgInfo(self,"Initializing Param Server")
        self.init_selected_img_topics = nepi_ros.get_param(self,'~detector/img_topics', [])

        self.init_wait_for_detect = nepi_ros.get_param(self,'~detector/wait_for_detect',DEFAULT_WAIT_FOR_DETECT)
        self.init_img_tiling = nepi_ros.get_param(self,'~detector/img_tiling', DEFAULT_IMG_TILING)

        self.init_overlay_labels = nepi_ros.get_param(self,'~detector/overlay_labels', DEFAULT_LABELS_OVERLAY)
        self.init_overlay_clf_name = nepi_ros.get_param(self,'~detector/overlay_clf_name', DEFAULT_CLF_OVERLAY)
        self.init_overlay_img_name = nepi_ros.get_param(self,'~detector/overlay_img_name', DEFAULT_IMG_OVERLAY)

        self.init_threshold = nepi_ros.get_param(self,'~detector/threshold', self.defualt_config_dict['threshold'])
        self.init_max_rate = nepi_ros.get_param(self,'~detector/max_rate', self.defualt_config_dict['max_rate'])
        self.init_enabled = nepi_ros.get_param(self,'~detector/enabled', Fasle)
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
        img_topics = nepi_ros.get_param(self,'~detector/img_topics', self.init_selected_img_topics)
        if img_topic not in img_topics:
            img_topics.append(img_topic)
        nepi_ros.set_param(self,'~detector/img_topics', img_topics)
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
        img_topics = nepi_ros.get_param(self,'~detector/img_topics', self.init_selected_img_topics)
        if img_topic in img_topics:
            img_topics.remove(img_topic)
        nepi_ros.set_param(self,'~detector/img_topics', img_topics)
        self.publishStatus()

    def setTileImgCb(self,msg):
        nepi_ros.set_param(self,'~detector/img_tiling', msg.data)
        self.publishStatus()

    def setOverlayLabelsCb(self,msg):
        nepi_ros.set_param(self,'~detector/overlay_labels', msg.data)
        self.publishStatus()

    def setOverlayClfNameCb(self,msg):
        nepi_ros.set_param(self,'~detector/overlay_clf_name', msg.data)
        self.publishStatus()

    def setOverlayImgNameCb(self,msg):
        nepi_ros.set_param(self,'~detector/overlay_img_name', msg.data)
        self.publishStatus()




    def setThresholdCb(self,msg):
        threshold = msg.data
        nepi_msg.publishMsgInfo(self,"Received Threshold Update: " + str(threshold))
        if threshold <  MIN_THRESHOLD:
            threshold = MIN_THRESHOLD
        elif threshold > MAX_THRESHOLD:
            threshold = MAX_THRESHOLD
        nepi_ros.set_param(self,'~detector/threshold', threshold)
        self.publishStatus()

    def setMaxRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        nepi_ros.set_param(self,'~detector/max_rate', max_rate)
        self.publishStatus()

    def enableCb(self,msg):
        enabled = msg.data
        nepi_ros.set_param(self,'~detector/enabled', enabled)
        self.publishStatus()
        time.sleep(1)
        if msg.data == False and not rospy.is_shutdown():
            self.get_img_topic = "None"


    def statusPublishCb(self,timer):
        self.publishStatus()

    def updaterCb(self,timer):
        enabled = nepi_ros.get_param(self,'~detector/enabled', self.init_enabled)
        #nepi_msg.publishMsgWarn(self,"Updating with image topic: " +  self.img_topic)
        img_topics = nepi_ros.get_param(self,'~detector/img_topics', self.init_selected_img_topics)
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

        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        img_connects = []
        for img_topic in imgs_info_dict.keys():
            img_connects.append(imgs_info_dict[img_topic]['connected'])
        img_selected = len(img_connects) > 0
        img_connected = True in img_connects
       
        if not rospy.is_shutdown():
            if enabled == True:
                if img_selected == 0:
                    self.state = "Waiting"
                elif img_connected == False:
                    self.state = "Listening"
                else:
                    self.state = "Running"
    
            else: # Loaded, but not enabled
                self.state = "Loaded"

        rospy.Timer(rospy.Duration(.1), self.updaterCb, oneshot = True)


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
            found_object_pub = rospy.Publisher(pub_sub_namespace + '/found_object', ObjectCount,  queue_size = 1)
            bounding_box_pub = rospy.Publisher(pub_sub_namespace + '/bounding_boxes', BoundingBoxes, queue_size = 1)
            detection_trigger_pub = rospy.Publisher(pub_sub_namespace + '/detection_trigger', Bool,  queue_size = 1)
            detection_state_pub = rospy.Publisher(pub_sub_namespace + '/detection_state', Bool,  queue_size = 1)
            img_sub = img_sub = rospy.Subscriber(img_topic, Image, self.imageCb, queue_size=1, callback_args=(img_topic))
            time.sleep(1)


            # Create img info dict
            self.imgs_info_dict[img_topic] = dict()  
            self.imgs_info_dict[img_topic]['pub_sub_namespace'] = pub_sub_namespace
            self.imgs_info_dict[img_topic]['has_mmap'] = None
            self.imgs_info_dict[img_topic]['mmap_id'] = ""
            self.imgs_info_dict[img_topic]['mmap_info_dict'] = dict()
            self.imgs_info_dict[img_topic]['connected'] = False 
            self.imgs_info_dict[img_topic]['image_latency_time'] = 0
            self.imgs_info_dict[img_topic]['detect_latency_time'] = 0
            self.imgs_info_dict[img_topic]['preprocess_time'] = 0 
            self.imgs_info_dict[img_topic]['detect_time'] = 0  


            self.imgs_lock_dict[img_topic] = threading.Lock()
       
            # Create imag sub pub dict
            self.imgs_pub_sub_lock.acquire()
            self.imgs_pub_sub_dict[img_topic] = {'img_sub': img_sub,
                                            'found_object_pub': found_object_pub,
                                            'bounding_box_pub': bounding_box_pub,
                                            'detection_trigger_pub': detection_trigger_pub,
                                            'detection_state_pub': detection_state_pub
                                            }
            
            self.imgs_pub_sub_lock.release()
            nepi_msg.publishMsgWarn(self,'Registered : ' + img_topic +  ' ' + str(self.imgs_pub_sub_dict[img_topic]))
            time.sleep(1)
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
                    nepi_msg.publishMsgWarn(self,'Unregistering topic: ' + pub_sub_name)
                    img_pub_sub_dict[pub_sub_name].unregister
            if img_topic in self.imgs_info_dict.keys():
                del self.imgs_info_dict[img_topic]
        self.imgs_pub_sub_lock.release()
        return True


    def imageCb(self,image_msg, args):      
        img_topic = args
        # Check if topic has memory mapping
        has_mmap = False
        if self.imgs_info_dict[img_topic]['has_mmap'] is None:
            mmap_id = get_mmap_id_from_topic(img_topic)
            mmap_exists = nepi_mmap.check_for_mmap(mmap_id)
            if mmap_exists == True:
                [success, msg, info_dict]  = nepi_mmap.get_cv2img_mmap_info(mmap_id)
                if info_dict is not None:
                    has_mmap = True
                    self.imgs_info_dict[img_topic]['has_mmap'] = True
                    self.imgs_info_dict[img_topic]['mmap_id'] = mmap_id
                    self.imgs_info_dict[img_topic]['mmap_info_dict'] = info_dict
        else:
            has_mmap = self.imgs_info_dict[img_topic]['has_mmap']
        if has_mmap == True:
            # Start get mmap image thread
            pass
        else:
            # Process ros image message
            current_time = nepi_ros.ros_time_now()
            ros_timestamp = img_msg.header.stamp
            latency = (current_time.to_sec() - ros_timestamp.to_sec())
            self.imgs_info_dict[img_topic]['image_latency_time'] = latency
            #nepi_msg.publishMsgInfo(self,"Detect Pub Latency: {:.2f}".format(latency))

            start_time = nepi_ros.get_time_sec()   
    
            if img_topic in self.imgs_info_dict.keys():
                self.imgs_info_dict[img_topic]['connected'] = True
                enabled = nepi_ros.get_param(self,'~detector/enabled', self.init_enabled)
                if enabled == True:
                    get_image = (img_topic == self.get_img_topic)
                    #nepi_msg.publishMsgWarn(self,"Callback got image from topic:  " + img_topic + " with get topic " + self.get_img_topic)
                    if get_image == True:
                        self.get_img_topic = "None"

                        #nepi_msg.publishMsgWarn(self,"Processing img for topic:  " + img_topic)
                        ##############################
                        ### Preprocess Image
                        
                        options_dict = dict()
                        options_dict['tile'] = nepi_ros.get_param(self,'~detector/img_tiling', self.init_img_tiling)
                        cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                        '''
                        img_dict = dict()
                        img_dict['cv2_img'] = cv2_img
                        '''
                        ros_timestamp = image_msg.header.stamp
                        img_dict = self.preprocessImage(ros_timestamp)
                        img_dict['timestamp'] = nepi_ros.sec_from_ros_stamp()
                        img_dict['ros_img_topic'] = img_topic
                        img_dict['ros_img_header'] = image_msg.header
                        img_dict['ros_img_stamp'] = ros_timestamp
                        ##############################

                        self.img_dict_lock.acquire()
                        self.img_dict = img_dict
                        self.img_dict_lock.release()
                        self.got_img_topic = img_topic

                        preprocess_time = round( (nepi_ros.get_time_sec() - start_time) , 3)
                        self.imgs_info_dict[img_topic]['preprocess_time'] = preprocess_time

    def imageMmapThread(self,img_topic, mmap_id):    
        mmap_list = nepi_mmap.get_mmap_list()
        while img_topic in self.imgs_pub_sub_dict.keys() and mmap_id in mmap_list and not rospy.is_shutdown():
            current_time = nepi_ros.get_time_now()
            mmap_list = nepi_mmap.get_mmap_list()

            start_time = nepi_ros.get_time_sec()   

            if img_topic in self.imgs_info_dict.keys():
                self.imgs_info_dict[img_topic]['connected'] = True
                enabled = nepi_ros.get_param(self,'~detector/enabled', self.init_enabled)
                if enabled == True:
                    get_image = (img_topic == self.get_img_topic)
                    #nepi_msg.publishMsgWarn(self,"Callback got image from topic:  " + img_topic + " with get topic " + self.get_img_topic)
                    if get_image == True:
                        self.get_img_topic = "None"

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
                                        self.imgs_info_dict[img_topic]['image_latency_time'] = latency
                                        #nepi_msg.publishMsgInfo(self,"Detect Pub Latency: {:.2f}".format(latency))



                                        #nepi_msg.publishMsgWarn(self,"Processing img for topic:  " + img_topic)
                                        ##############################
                                        ### Preprocess Image
                                        
                                        options_dict = dict()
                                        options_dict['tile'] = nepi_ros.get_param(self,'~detector/img_tiling', self.init_img_tiling)
                                        cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                                        '''
                                        img_dict = dict()
                                        img_dict['cv2_img'] = cv2_img
                                        '''
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
                                        self.got_img_topic = img_topic

                                        preprocess_time = round( (nepi_ros.get_time_sec() - start_time) , 3)
                                        self.imgs_info_dict[img_topic]['preprocess_time'] = preprocess_time

                                        unlocked = nepi_mmap.unlock_mmap(mmap_id)
                                    else:
                                        self.imgs_info_dict[img_topic]['has_mmap'] = True
                                        break
                                else:
                                    time.sleep(.01)



    def updateDetectionTopicCb(self,timer):
        start_time = nepi_ros.get_time_sec()
        enabled = nepi_ros.get_param(self,'~detector/enabled', self.init_enabled)
        if enabled == True:
            img_topics = nepi_ros.get_param(self,'~detector/img_topics', self.init_selected_img_topics)
            connected_list = []
            for topic in img_topics:
                if topic in self.imgs_info_dict.keys():
                    if self.imgs_info_dict[topic]['connected'] == True:
                        connected_list.append(topic)
            if len(connected_list) == 0:
                self.get_img_topic = "None"
            else:
                # check timer
                max_rate = nepi_ros.get_param(self,'~detector/max_rate', self.init_max_rate)
                delay_time = float(1) / max_rate 
                current_time = nepi_ros.get_time_sec()
                timer = round((current_time - self.last_detect_time), 3)
                #nepi_msg.publishMsgWarn(self,"Delay and Timer: " + str(delay_time) + " " + str(timer))

                # Get image topic info
                img_topic = self.cur_img_topic

                # Setup Next Img if needed
                num_connected_list = len(connected_list)
                if num_connected_list > 0:
                    if img_topic in connected_list:
                        next_img_ind = connected_list.index(img_topic) + 1
                        if next_img_ind >= num_connected_list:
                            next_img_topic = connected_list[0]
                        else:
                            next_img_topic = connected_list[next_img_ind]
                    else:
                        next_img_topic = connected_list[0]
                else:
                    next_img_topic = "None"
                #nepi_msg.publishMsgWarn(self,"Next Image Topic set to: " + next_img_topic)

                # Check if current image topic is None
                if img_topic == "None" and next_img_topic != "None":
                    self.get_img_topic = next_img_topic
                    self.cur_img_topic = next_img_topic
                    self.last_detect_time = nepi_ros.get_time_sec()

                ##############################
                # Check for non responding image streams                   
                if timer > (delay_time + GET_IMAGE_TIMEOUT_SEC):
                    #nepi_msg.publishMsgWarn(self,"Topic " + img_topic + " timed out. Setting next topic to: " +  next_img_topic)
                    if img_topic is not None and img_topic in self.imgs_info_dict.keys():
                        self.imgs_info_dict[img_topic]['connected'] = False
                    self.get_img_topic = next_img_topic
                    self.cur_img_topic = next_img_topic
                    self.last_detect_time = nepi_ros.get_time_sec()

                elif timer > delay_time: 


                    if len(connected_list) > 0 and self.get_img_topic == "None" :
                        #Request new image before publishing current
                        #nepi_msg.publishMsgWarn(self,"Setting next topic to: " +  next_img_topic)
                        self.cur_img_topic = next_img_topic
                        self.get_img_topic = next_img_topic
                        self.last_detect_time = nepi_ros.get_time_sec()

                    #nepi_msg.publishMsgWarn(self,"Timer over delay check, looking for image topic: " +  img_topic)
                    if img_topic != "None" and img_topic in connected_list and img_topic == self.got_img_topic :

                        #nepi_msg.publishMsgWarn(self,"Got image topic: " +  img_topic)
                        self.got_img_topic = "None"
                        
                        # Process got image
                        #nepi_msg.publishMsgWarn(self,"Copying img_dict from topic callback:  " + img_topic)
                        self.img_dict_lock.acquire()
                        img_dict = copy.deepcopy(self.img_dict)
                        self.img_dict_lock.release()
                        #nepi_msg.publishMsgWarn(self,"Copying img_dict from topic callback:  " + img_topic)

                        if img_dict is None:
                            nepi_msg.publishMsgWarn(self,"Callback provided None img_dict, :  " + img_topic)
                        else:
                            if img_dict['cv2_img'] is None:
                                nepi_msg.publishMsgWarn(self,"Callback provided None cv2_img, :  " + img_topic)
                                pass
                            else:
                                #nepi_msg.publishMsgWarn(self,"Detector got img_dict from topic callback:  " + img_topic + " with img size: " + str(img_dict['cv2_img'].shape[:2]))

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
                                ros_img_header = img_dict['ros_img_header']

                                self.publishDetectionData(img_topic,detect_dict_list,ros_img_header)
                                
                            detect_time = round( (nepi_ros.get_time_sec() - start_time) , 3)
                            self.imgs_info_dict[img_topic]['detect_time'] = detect_time
                            #nepi_msg.publishMsgInfo(self,"Detect Time: {:.2f}".format(detect_time))

                        current_time = nepi_ros.ros_time_now()
                        ros_timestamp = img_dict['ros_img_stamp']
                        latency = (current_time.to_sec() - ros_timestamp.to_sec())
                        self.imgs_info_dict[img_topic]['detect_latency_time'] = latency
                        #nepi_msg.publishMsgInfo(self,"Detect Pub Latency: {:.2f}".format(latency))
                       
        rospy.Timer(rospy.Duration(0.01), self.updateDetectionTopicCb, oneshot = True)

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
                    self.detection_trigger_pub.publish()
                    self.detection_trigger_all_pub.publish()    
                                    
                    self.bounding_boxes_all_pub.publish(bbs_msg)   

                    if img_topic in self.imgs_pub_sub_dict.keys():
                        imgs_pub_sub_dict = self.imgs_pub_sub_dict[img_topic]
                        bounding_box_pub = imgs_pub_sub_dict['bounding_box_pub']
                        bounding_box_pub.publish(bbs_msg)
                        detection_trigger_pub = imgs_pub_sub_dict['detection_trigger_pub']
                        detection_trigger_pub.publish()
                        detection_state_pub = imgs_pub_sub_dict['detection_state_pub']
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
                imgs_pub_sub_dict = self.imgs_pub_sub_dict[img_topic]
                found_object_pub = imgs_pub_sub_dict['found_object_pub']
                found_object_pub.publish(found_object_msg)
                if count == 0:
                    self.detection_state_pub.publish(False)
                    if img_topic in self.imgs_pub_sub_dict.keys():
                        imgs_pub_sub_dict = self.imgs_pub_sub_dict[img_topic]
                        detection_state_pub = imgs_pub_sub_dict['detection_state_pub']
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
        img_img_lat_times = []
        img_det_lat_times = []
        img_pre_times = []
        img_detect_times = []
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        for img_topic in imgs_info_dict.keys():
            img_source_topics.append(img_topic)
            img_det_namespaces.append(imgs_info_dict[img_topic]['pub_sub_namespace'])
            img_connects.append(imgs_info_dict[img_topic]['connected'])
            img_img_lat_times.append(imgs_info_dict[img_topic]['image_latency_time'])
            img_det_lat_times.append(imgs_info_dict[img_topic]['detect_latency_time'])
            img_pre_times.append(imgs_info_dict[img_topic]['preprocess_time'])
            img_detect_times.append(imgs_info_dict[img_topic]['detect_time'])
        status_msg.image_source_topics = img_source_topics
        status_msg.image_detect_namespaces = img_det_namespaces
        status_msg.images_connected = img_connects
        status_msg.image_latency_times = img_img_lat_times
        status_msg.detect_latency_times = img_det_lat_times
        status_msg.preprocess_times = img_pre_times
        status_msg.detect_times = img_detect_times

        img_selected = len(img_connects) > 0
        status_msg.image_selected = img_selected
        img_connected = True in img_connects
        status_msg.image_connected = img_connected

        img_lat_time = 0.0
        det_lat_time = 0.0
        pre_time = 0.0
        detect_time = 0.0
        if img_connected:
            img_lat_time = sum(img_img_lat_times) / len(img_img_lat_times)
            det_lat_time = sum(img_det_lat_times) / len(img_det_lat_times)
            pre_time = sum(img_pre_times) / len(img_pre_times)
            detect_time = sum(img_detect_times) / len(img_detect_times)
        status_msg.image_latency_time = img_lat_time
        status_msg.detect_latency_time = det_lat_time
        status_msg.preprocess_time = pre_time
        status_msg.detect_time = detect_time


        #nepi_msg.publishMsgWarn(self,"Sending Status Msg: " + str(status_msg))
        if not rospy.is_shutdown():
            self.status_pub.publish(status_msg)



