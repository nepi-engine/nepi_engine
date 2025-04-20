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
import cv2

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image

from nepi_ros_interfaces.msg import ImageStatus
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_ros_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_ais
from nepi_sdk import nepi_img
from nepi_sdk import nepi_save

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeParamsIF, NodePublishersIF, NodeSubscribersIF, NodeClassIF
from nepi_api.system_if import SaveDataIF, StatesIF, TriggersIF

from nepi_api.mgr_if_system import MgrSystemIF
from nepi_api.mgr_if_config import MgrConfigIF




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

GET_IMAGE_TIMEOUT_SEC = 1 

SDK_LIB_FOLDER = "/opt/nepi/ros/lib/nepi_sdk"
AIFS_SHARE_PATH = "/opt/nepi/ros/share/nepi_aifs"
USER_CFG_FOLDER = '/mnt/nepi_storage/user_cfg/ros'

class AiDetectorIF:



    det_img_pub_file = 'nepi_ai_detector_img_pub_node.py'

    data_products = ['bounding_boxes','detection_image']

    node_namespace = ""
    config_file_path = ""
    self_managed = True
    model_name = "None"

    last_detect_time = nepi_ros.get_time()
    imgs_pub_sub_dict = dict()
    imgs_pub_sub_lock = threading.Lock()
    imgs_info_dict = dict()
    imgs_lock_dict = dict()
    imgs_img_proc_dict = dict()
    imgs_img_pub_if_dict = dict()

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
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################  
        # Init Class Variables 

        all_pubs_config_dict = None
        all_pubs_if = None


        ##############################  
        # Create NodeClassIF Class  

        # Configs Dict ########################
        self.CONFIGS_DICT = {
                'init_callback': None,
                'reset_callback': None,
                'factory_reset_callback': None,
                'init_configs': True,
                'namespace': '~',
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'img_topics': {
                'namespace': self.namespace,
                'factory_val': []
            },
            'wait_for_detect': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_WAIT_FOR_DETECT
            },
            'img_tiling': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_IMG_TILING
            },
            'overlay_labels': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_LABELS_OVERLAY
            },
            'overlay_clf_name': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_CLF_OVERLAY
            },
            'threshold': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_THRESHOLD
            },
            'max_rate': {
                'namespace': self.namespace,
                'factory_val': DEFAULT_MAX_RATE
            },
            'enabled': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'sleep_enabled': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'sleep_suspend_time': {
                'namespace': self.namespace,
                'factory_val': -1
            },
            'sleep_run_time': {
                'namespace': self.namespace,
                'factory_val': 1
            },
        }



        # Services Config Dict ####################
        self.SRVS_DICT = {
            'info_query': {
                'namespace': self.namespace,
                'topic': 'detector_info_query',
                'srv': AiDetectorInfoQuery,
                'req': AiDetectorInfoQueryRequest(),
                'resp': AiDetectorInfoQueryResponse(),
                'callback': self.handleInfoRequest
            }
        }


        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'found_object': {
                'msg': ObjectCount,
                'namespace': self.namespace,
                'topic': 'found_object',
                'qsize': 1,
                'latch': False
            },
            'bounding_boxes': {
                'msg': BoundingBoxes,
                'namespace': self.namespace,
                'topic': 'bounding_boxes',
                'qsize': 1,
                'latch': False
            },
            'status': {
                'msg': AiDetectorStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
            
        }


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'update': {
                'msg': Setting,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 1,
                'callback': self._updateSettingCb
            },
            'publish_settings': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'publish_setting',
                'qsize': 1,
                'callback': self._publishSettingsCb
            },
            'reset': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 1,
                'callback': self._resetInitSettingCb
            },
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(self,
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.node_if.wait_for_ready()
        
        ##############################
        ## Setup All Detector Pubs and Subs

        if all_namespace.find(self.node_name) == -1:
            self.self_managed = False
            # starting all detector pubs
            self.all_namespace = all_namespace
            self.all_pubs_config_dict = self.PUBS_CONFIG_DICT
            del self.all_pubs_config_dict.['pubs_dict']['status']
            self.all_pubs_config_dict['namespace'] = self.all_namespace
    
            self.all_pubs_if = NodePublishersIF(self,
                            pubs_config_dict = all_pubs_config_dict,
                            log_class_name = False
            )


        ##############################
        ## Get folder info
        mgr_sys_if = MgrSystemIF()
        success = mgr_sys_if.wait_for_status()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Status Msg")

        self.sdk_lib_folder = SDK_LIB_FOLDER
        self.msg_if.pub_info("Using SDK Share Folder: " + str(self.sdk_lib_folder))

 
        #self.aifs_param_folder = mgr_sys_if.get_sys_folder_path("aifs",AIFS_SHARE_PATH)
        #self.msg_if.pub_info("Using AIF Share Folder: " + str(self.aifs_param_folder))

        self.user_cfg_folder = mgr_sys_if.get_sys_folder_path('user_cfg/ros',USER_CFG_FOLDER)
        self.msg_if.pub_info("Using User Config Folder: " + str(self.user_cfg_folder))
        
        # Wait for Config Manager
        mgr_cfg_if = MgrConfigIF()
        success = mgr_cfg_if.wait_for_status()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get Config Status Msg")
        
        ##############################
        # Create Image Publishers and Messages
        det_img_pub_topic = os.path.join(det_base_namespace,'detection_image')
        self.pub_img_det_if = ImageIF(det_img_pub_topic)
        ready = self.pub_img_det_if.wait_for_ready()

        det_img_all_pub_topic = os.path.join(det_all_namespace,'detection_image')
        self.pub_img_all_if = ImageIF(det_img_all_pub_topic)
        ready = self.pub_img_all_if.wait_for_ready()

        time.sleep(1)

        # Create Img Status Message
        self.not_enabled_msg = "DETECTOR NOT ENABLED"
        self.no_img_msg = "NO IMAGES CONNECTED"
        self.img_pub_loading_msg = "IMAGE_PUBLISHER_LOADING"



        ##########################

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
        self.msg_if.pub_warn("Initializing detector with blank image")
        options_dict = dict()
        cv2_test_imge = nepi_img.create_cv2_test_img(1000,600)
        img_dict = self.preprocessImage(cv2_test_imge, options_dict)
        img_dict['ros_img_topic'] = 'Test_Img'
        img_dict['ros_img_header'] = ""
        img_dict['ros_img_stamp'] = nepi_ros.ros_time_now()
        detect_dict_list = self.processDetection(img_dict, 0.5)
        self.msg_if.pub_warn("Detector initializing complete")
                 
        self.msg_if.pub_info("Starting IF setup")

        # Load/Create model config params
        self.node_namespace = os.path.join(self.base_namespace,"ai",self.node_name)
        self.config_file_path = os.path.join(self.user_cfg_folder,self.node_name)
        nepi_ros.load_config_file(self.config_file_path, defualt_config_dict, self.node_namespace)
        self.msg_if.pub_warn("Loaded AI Detector params")
        node_params = self.node_if.get_node_params()
        
        # Create Publishers


        time.sleep(1)


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

        rospy.Subscriber('~set_threshold', Float32, self.setThresholdCb, queue_size=10)
        rospy.Subscriber('~set_max_rate', Int32, self.setMaxRateCb, queue_size=10)

        rospy.Subscriber('~set_sleep_enable', Bool, self.setSleepEnableCb, queue_size=10) 
        rospy.Subscriber('~set_sleep_suspend_sec', Int32, self.setSleepSuspendTimeCb, queue_size=10)
        rospy.Subscriber('~set_sleep_run_sec', Int32, self.setSleepSuspendTimeCb, queue_size=10)

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
        rospy.Timer(rospy.Duration(.1), self.updateDetectCb, oneshot = True)

        self.state = 'Loaded'

        self.msg_if.pub_info("IF Initialization Complete")
        
    def saveConfigCb(self, msg):
        self.msg_if.pub_info("Recived Save Config")
        nepi_ros.save_params_to_file(self.config_file_path,self.node_namespace) 

    def resetConfigCb(self, msg):
        self.msg_if.pub_info("Recived Reset Config")
        self.resetParamServer()


    def resetFactoryCb(self, msg):
        self.msg_if.pub_info("Recived Factory Reset")
        self.resetFactory()

    def addImageTopicCb(self,msg):
        self.msg_if.pub_info("Received Add Image Topics: " + msg.data)
        img_topic = msg.data
        self.addImageTopic(img_topic)


    def addImageTopicsCb(self,msg):
        self.msg_if.pub_info("Received Add Image Topics: " + str(msg))
        img_topic_list = msg.entries
        for img_topic in img_topic_list:
            self.addImageTopic(img_topic)


    def addImageTopic(self,img_topic):   
        img_topics = self.node_if.get_param('img_topics')
        if img_topic not in img_topics:
            img_topics.append(img_topic)
        self.node_if.set_param('img_topics', img_topics)
        self.publishStatus()

    def removeImageTopicCb(self,msg):
        self.msg_if.pub_info("Received Remove Image Topics: " + str(msg))
        img_topic = msg.data
        self.removeImageTopic(img_topic)


    def removeImageTopicsCb(self,msg):
        self.msg_if.pub_info("Received Remove Image Topic: " + str(msg))
        img_topic_list = msg.entries
        for img_topic in img_topic_list:
            self.removeImageTopic(img_topic)

    def removeImageTopic(self,img_topic):         
        img_topics = self.node_if.get_param('img_topics')
        if img_topic in img_topics:
            img_topics.remove(img_topic)
        self.node_if.set_param('img_topics', img_topics)
        self.publishStatus()

    def setTileImgCb(self,msg):
        self.node_if.set_param('img_tiling', msg.data)
        self.publishStatus()

    def setOverlayLabelsCb(self,msg):
        self.node_if.set_param('overlay_labels', msg.data)
        self.publishStatus()

    def setOverlayClfNameCb(self,msg):
        self.node_if.set_param('overlay_clf_name', msg.data)
        self.publishStatus()




    def setThresholdCb(self,msg):
        threshold = msg.data
        self.msg_if.pub_info("Received Threshold Update: " + str(threshold))
        if threshold <  MIN_THRESHOLD:
            threshold = MIN_THRESHOLD
        elif threshold > MAX_THRESHOLD:
            threshold = MAX_THRESHOLD
        self.node_if.set_param('threshold', threshold)
        self.publishStatus()

    def setMaxRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        self.node_if.set_param('max_rate', max_rate)
        self.publishStatus()

    def enableCb(self,msg):
        enabled = msg.data
        self.node_if.set_param('enabled', enabled)
        self.publishStatus()
        time.sleep(1)
        if msg.data == False and not rospy.is_shutdown():
            self.get_img_topic = "None"


    def setSleepEnableCb(self,msg):
        self.node_if.set_param('sleep_enabled', msg.data)
        self.publishStatus()


    def setSleepSuspendTimeCb(self,msg):
        data = msg.data
        if data > 1 or suspend_time == -1:
            self.node_if.set_param('sleep_suspend_time', data)
        self.publishStatus()

    def setSleepSuspendTimeCb(self,msg):
        data = msg.data
        if data > 1:
            self.node_if.set_param('sleep_run_time', data)
        self.publishStatus()


    def statusPublishCb(self,timer):
        self.publishStatus()

    def updaterCb(self,timer):
        enabled = self.node_if.get_param('enabled')
        #self.msg_if.pub_warn("Updating with image topic: " +  self.img_topic)
        img_topics = self.node_if.get_param('img_topics')
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
                    self.pub_img_det_if.publish_img_msg(self.no_img_msg)
                elif img_connected == False:
                    self.state = "Listening"
                else:
                    self.state = "Running"
    
            else: # Loaded, but not enabled
                self.state = "Loaded"
                self.pub_img_det_if.publish_img_msg(self.not_enabled_msg)

        rospy.Timer(rospy.Duration(.1), self.updaterCb, oneshot = True)


    def subscribeImgTopic(self,img_topic):
        if img_topic == "None" or img_topic == "":
            return False
        else:
            imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
            if img_topic in imgs_info_dict.keys():
                if self.imgs_pub_sub_dict[img_topic]
                    nepi_msg.publishMsgInfo(self,'Subsribing to image topic: ' + img_topic)  
                    # Reregister img_sub
                    self.imgs_pub_sub_lock.acquire()
                    self.imgs_pub_sub_dict[img_topic]['subs_if'].register('img_sub')
                    self.imgs_pub_sub_lock.release()
                    nepi_msg.publishMsgWarn(self,'Registered : ' + img_topic +  ' ' + str(self.imgs_pub_sub_dict[img_topic]))
                    # Set back to active
                    self.imgs_info_dict[img_topic]['active'] = True
            else:
                    
                self.imgs_lock_dict[img_topic] = threading.Lock()

                img_name = img_topic.replace(self.base_namespace,"")
                pub_sub_namespace = os.path.join(self.node_namespace,img_name)

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
                self.imgs_info_dict[img_topic]['active'] = True
                self.imgs_info_dict[img_topic]['img_publishing'] = False

                # Create img sub pubs dict
                nepi_msg.publishMsgInfo(self,'Subsribing to image topic: ' + img_topic)

                self.img_pubs_config_dict = self.PUBS_CONFIG_DICT
                del self.img_pubs_config_dict.['pubs_dict']['status']
                self.img_pubs_config_dict['namespace'] = pub_sub_namespace
        
                self.img_pubs_if = NodePublishersIF(self,
                                pubs_config_dict = img_pubs_config_dict,
                                log_class_name = False
                )

                # Image Subs Config Dict ####################

                self.IMG_SUBS_DICT = {
                    'sub_name': {
                        'msg': Image,
                        'topic': image_name,
                        'qsize': 1,
                        'callback': self.imageCb,
                        'callback_args': (img_topic)
                    }
                }

                self.IMG_SUBS_CONFIG_DICT = {
                    'namespace': self.base_namespace,
                    'sub_dict': IMG_SUBS_DICT
                }


                self.img_subs_if = NodeSubscribersIF(self,
                                subs_config_dict = self.IMG_SUBS_CONFIG_DICT,
                                log_class_name = False
                )
                self.imgs_pub_sub_lock.acquire()
                self.imgs_pub_sub_dict[img_topic] = {'pubs_if': img_pubs_if,
                                                'subs_if': img_subs_if
                                                }   

                self.imgs_pub_sub_lock.release()
                nepi_msg.publishMsgWarn(self,'Registered : ' + img_topic +  ' ' + str(self.imgs_pub_sub_dict[img_topic]))


                time.sleep(1)
                # Publish blank msg to prime topics
                found_object_pub.publish(ObjectCount())
                bounding_box_pub.publish(BoundingBoxes())


                # Launch detection img pub node
                
                pkg_name = 'nepi_sdk'

                sdk_lib_folder = self.sdk_lib_folder  #+ '/scripts'
                node_file_folder = sdk_lib_folder
                img_pub_file = self.det_img_pub_file
                img_pub_file_path = os.path.join(node_file_folder,img_pub_file)
            
                pub_process = None
                if os.path.exists(img_pub_file_path) == False:
                    self.msg_if.pub_warn("Could not find det img pub node file at: " + img_pub_file_path)
                elif img_topic not in self.imgs_img_proc_dict.keys():
                    node_name = self.node_name + "_img_pub"
                    self.imgs_info_dict[img_topic]['node_name'] = node_name


                    # Store det img sub pub namespace for node in param server
                    IMG_PARAMS_DICT = {
                        'det_namespace': {
                            'factory_val': pub_sub_namespace
                        },
                        'base_namespace': {
                            'factory_val': self.node_namespace
                        },
                        'all_namespace': {
                            'factory_val': self.all_namespace
                        }
                    }

                    IMG_NODE_NAMESPACE = os.path.join(self.base_namespace,node_name)
                    IMG_PARAMS_CONFIG_DICT = {
                            'save_callback': None,
                            'reset_callback': None,
                            'factory_reset_callback': None,
                            'namespace': IMG_NODE_NAMESPACE,
                            'params_dict': IMG_PARAMS_DICT
                    }

                    params_if = NodeParamsIF(params_config_dict = IMG_PARAMS_CONFIG_DICT)


                    #Try and launch node
                    self.msg_if.pub_warn("Launching Detector Img Pub node: " + node_name)
                    [success, msg, pub_process] = nepi_ros.launch_node(pkg_name, img_pub_file, node_name)
                    self.imgs_img_proc_dict[img_topic] = pub_process
                    self.msg_if.pub_warn("Img Pub Launch return msg: " + msg)
                                    # Create image pub for img detector
                    pub_img_if = ImageIF(pub_sub_namespace, create_status_pub = False)
                    ready = self.pub_img_if.wait_for_ready()
                    if ready == True:
                        success = pub_img_if.publish_cv2_msg_img(self.img_pub_loading_msg)
                    self.imgs_img_pub_if_dict[img_topic] = pub_img_if
    

    
            return True
        

    def unsubscribeImgTopic(self,img_topic):
        self.imgs_pub_sub_lock.acquire()
        if img_topic in self.imgs_pub_sub_dict.keys():
            nepi_msg.publishMsgWarn(self,'Unregistering image topic: ' + img_topic)
            if 'subs_if' in img_pub_sub_dict.keys():
                self.imgs_pub_sub_dict[img_topic]['subs_if'].unregister('img_sub')
        self.imgs_pub_sub_lock.release()

        #Leave img pub running in case it is switched back on
        '''
        nepi_msg.publishMsgWarn(self,'killing node: ' + pub_sub_name)
        node_name = self.imgs_info_dict[img_topic]['node_name'] 
        sub_process = img_pub_sub_dict[pub_sub_name]
        if sub_process is not None:
            success = nepi_ros.kill_node_process(node_name,sub_process)
        '''
    
        # Clear info dict
        self.imgs_info_dict[img_topic]['active'] = False
        self.imgs_info_dict[img_topic]['connected'] = False 
        self.imgs_info_dict[img_topic]['image_latency_time'] = 0
        self.imgs_info_dict[img_topic]['detect_latency_time'] = 0
        self.imgs_info_dict[img_topic]['preprocess_time'] = 0 
        self.imgs_info_dict[img_topic]['detect_time'] = 0 

        return True


    def imageCb(self,image_msg, args):     
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict) 
        img_topic = args


        start_time = nepi_ros.get_time()   

        if img_topic in imgs_info_dict.keys():
            if imgs_info_dict[img_topic]['active'] == True:
                # Process ros image message
                current_time = nepi_ros.ros_time_now()
                ros_timestamp = image_msg.header.stamp
                latency = (current_time.to_sec() - ros_timestamp.to_sec())
                imgs_info_dict[img_topic]['image_latency_time'] = latency
                #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency))


                imgs_info_dict[img_topic]['connected'] = True
                enabled = self.node_if.get_param('enabled')
                if enabled == True:
                    get_image = (img_topic == self.get_img_topic)
                    #self.msg_if.pub_warn("Callback got image from topic:  " + img_topic + " with get topic " + self.get_img_topic)
                    if get_image == True:
                        self.get_img_topic = "None"

                        #self.msg_if.pub_warn("Processing img for topic:  " + img_topic)
                        ##############################
                        ### Preprocess Image
                        
                        options_dict = dict()
                        options_dict['tile'] = self.node_if.get_param('img_tiling')
                        cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                        '''
                        img_dict = dict()
                        img_dict['cv2_img'] = cv2_img
                        '''
                        ros_timestamp = image_msg.header.stamp
                        img_dict = self.preprocessImage(cv2_img,options_dict)
                        img_dict['topic'] = img_topic
                        img_dict['timestamp'] = nepi_ros.sec_from_ros_time(ros_timestamp)
                        img_dict['ros_img_topic'] = img_topic
                        img_dict['ros_img_header'] = image_msg.header
                        img_dict['ros_img_stamp'] = ros_timestamp
                        ##############################

                        self.img_dict_lock.acquire()
                        self.img_dict = img_dict
                        self.img_dict_lock.release()
                        self.got_img_topic = img_topic

                        preprocess_time = round( (nepi_ros.get_time() - start_time) , 3)
                        imgs_info_dict[img_topic]['preprocess_time'] = preprocess_time

                if img_topic in self.imgs_info_dict.keys():
                    try:
                        self.imgs_info_dict[img_topic] = imgs_info_dict[img_topic]
                    except:
                        pass


    def updateDetectCb(self,timer):
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        start_time = nepi_ros.get_time()
        enabled = self.node_if.get_param('enabled')
        if enabled == True:
            img_topics = self.node_if.get_param('img_topics')
            connected_list = []
            for topic in img_topics:
                if topic in imgs_info_dict.keys():
                    if imgs_info_dict[topic]['connected'] == True:
                        connected_list.append(topic)
            if len(connected_list) == 0:
                self.get_img_topic = "None"
            else:
                # check timer
                max_rate = self.node_if.get_param('max_rate')
                delay_time = float(1) / max_rate 
                current_time = nepi_ros.get_time()
                timer = round((current_time - self.last_detect_time), 3)
                #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))

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
                #self.msg_if.pub_warn("Next Image Topic set to: " + next_img_topic)

                # Check if current image topic is None
                if img_topic == "None" and next_img_topic != "None":
                    self.get_img_topic = next_img_topic
                    self.cur_img_topic = next_img_topic
                    self.last_detect_time = nepi_ros.get_time()

                ##############################
                # Check for non responding image streams                   
                if timer > (delay_time + GET_IMAGE_TIMEOUT_SEC):
                    #self.msg_if.pub_warn("Topic " + img_topic + " timed out. Setting next topic to: " +  next_img_topic)
                    if img_topic is not None and img_topic in imgs_info_dict.keys():
                        imgs_info_dict[img_topic]['connected'] = False
                    self.get_img_topic = next_img_topic
                    self.cur_img_topic = next_img_topic
                    self.last_detect_time = nepi_ros.get_time()

                elif timer > delay_time: 


                    if len(connected_list) > 0 and self.get_img_topic == "None" :
                        #Request new image before publishing current
                        #self.msg_if.pub_warn("Setting next topic to: " +  next_img_topic)
                        self.cur_img_topic = next_img_topic
                        self.get_img_topic = next_img_topic
                        self.last_detect_time = nepi_ros.get_time()

                    #self.msg_if.pub_warn("Timer over delay check, looking for image topic: " +  img_topic)
                    if img_topic != "None" and img_topic in connected_list and img_topic == self.got_img_topic :

                        #self.msg_if.pub_warn("Got image topic: " +  img_topic)
                        self.got_img_topic = "None"
                        
                        # Process got image
                        #self.msg_if.pub_warn("Copying img_dict from topic callback:  " + img_topic)
                        self.img_dict_lock.acquire()
                        img_dict = copy.deepcopy(self.img_dict)
                        self.img_dict_lock.release()
                        #self.msg_if.pub_warn("Copying img_dict from topic callback:  " + img_topic)

                        if img_dict is None:
                            self.msg_if.pub_warn("Callback provided None img_dict, :  " + img_topic)
                        else:
                            if img_dict['cv2_img'] is None:
                                self.msg_if.pub_warn("Callback provided None cv2_img, :  " + img_topic)
                                pass
                            else:
                                #self.msg_if.pub_warn("Detector got img_dict from topic callback:  " + img_topic + " with img size: " + str(img_dict['cv2_img'].shape[:2]))

                                ##############################
                                # Process Detections
                                detect_dict_list = []
                                try:
                                    threshold = self.node_if.get_param('threshold')
                                    detect_dict_list = self.processDetection(img_dict,threshold) 
                                    #self.msg_if.pub_warn("AIF got back detect_dict: " + str(detect_dict_list))
                                    success = True
                                    self.first_detect_complete = True
                                except Exception as e:
                                    self.msg_if.pub_warn("Failed to process detection img with exception: " + str(e))
                                ros_img_header = img_dict['ros_img_header']

                                self.publishDetectionData(img_dict,detect_dict_list,ros_img_header)
                                
                            detect_time = round( (nepi_ros.get_time() - start_time) , 3)
                            imgs_info_dict[img_topic]['detect_time'] = detect_time
                            #self.msg_if.pub_info("Detect Time: {:.2f}".format(detect_time))

                        current_time = nepi_ros.ros_time_now()
                        ros_timestamp = img_dict['ros_img_stamp']
                        latency = (current_time.to_sec() - ros_timestamp.to_sec())
                        imgs_info_dict[img_topic]['detect_latency_time'] = latency
                        #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency))

                        if img_topic in self.imgs_info_dict.keys():
                            try:
                                self.imgs_info_dict[img_topic] = imgs_info_dict[img_topic]
                            except:
                                pass


                       
        rospy.Timer(rospy.Duration(0.01), self.updateDetectCb, oneshot = True)

    def publishDetectionData(self,img_dict, detect_dict_list,ros_img_header):
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        img_topic = img_dict['topic']
        if imgs_info_dict[img_topic]['active'] == True:
            bb_msg_list = []
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
                    img_area = img_dict['prc_width']* img_dict['prc_height']
                    if img_area > 1:
                        area_ratio = area_pixels / img_area
                    else:
                        area_ratio = -999
                    bb_msg.area_pixels = detect_dict['area_pixels']
                    bb_msg.area_ratio = detect_dict['area_ratio']
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get all data from detect dict: " + str(e)) 
                bb_msg_list.append(bb_msg)
            bbs_msg = BoundingBoxes()
            bbs_msg.header.stamp = ros_img_header.stamp
            bbs_msg.model_name = self.node_name
            bbs_msg.image_header = ros_img_header
            bbs_msg.image_topic = img_topic
            bbs_msg.src_width = img_dict['src_width']
            bbs_msg.src_height = img_dict['src_height']
            bbs_msg.prc_width = img_dict['prc_width']
            bbs_msg.prc_height = img_dict['prc_height']
            bbs_msg.bounding_boxes = bb_msg_list
            if not rospy.is_shutdown() and self.bounding_boxes_pub is not None:
                    self.bounding_boxes_pub.publish(bbs_msg)
                    self.detection_trigger_pub.publish()
                    self.detection_trigger_all_pub.publish()    
                                    
                    self.bounding_boxes_all_pub.publish(bbs_msg)   


                    self.imgs_pub_sub_lock.acquire()
                    if img_topic in self.imgs_pub_sub_dict.keys():
                        imgs_pub_sub_dict = self.imgs_pub_sub_dict[img_topic]
                        bounding_box_pub = imgs_pub_sub_dict['pub_if']['bounding_boxes']
                        bounding_box_pub.publish(bbs_msg)
                        detection_trigger_pub = imgs_pub_sub_dict['detection_trigger_pub']
                        detection_trigger_pub.publish()
                        detection_state_pub = imgs_pub_sub_dict['detection_state_pub']
                        detection_state_pub.publish(True)
                    self.imgs_pub_sub_lock.release()
                        
                            
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

                self.imgs_pub_sub_lock.acquire()
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
                self.imgs_pub_sub_lock.release()
                

            # Save Bounding Data if needed
            image_text = img_topic.replace(self.base_namespace,"")
            image_text = image_text.replace('/idx',"")
            image_text = image_text.replace('/','_')
            if len(detect_dict_list) > 0:

                data_product = 'bounding_boxes'

                ros_timestamp = bbs_msg.header.stamp
        
                bbs_dict = nepi_ais.get_boxes_info_from_msg(bbs_msg)
                bb_dict_list = nepi_ais.get_boxes_list_from_msg(bbs_msg)
                bbs_dict['bounding_boxes']=bb_dict_list
                nepi_save.save_dict2file(self,data_product,bbs_dict,ros_timestamp,add_text = image_text)


    def handleInfoRequest(self,_):
        resp = AiDetectorInfoQueryResponse().detector_info
        resp.name = self.model_name
        resp.framework = self.model_framework
        resp.node_name = self.node_name
        resp.namespace = self.node_namespace
        resp.type = self.model_type
        resp.description = self.model_description
        resp.proc_img_height = self.model_proc_img_height
        resp.proc_img_width = self.model_proc_img_width
        resp.classes = self.classes_list
        resp.has_img_tiling = self.has_img_tiling

        img_source_topics = []
        img_det_namespaces = []
        img_det_states = []
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        for img_topic in imgs_info_dict.keys():
                img_source_topics.append(img_topic)
                img_det_namespaces.append(imgs_info_dict[img_topic]['pub_sub_namespace'])
                img_det_states.append( imgs_info_dict[img_topic]['active'])
        resp.image_source_topics = img_source_topics
        resp.image_detector_namespaces = img_det_namespaces
        resp.image_detector_states = img_det_states

        return resp
    

    def publishStatusCb(self,timer):
        self.publishStatus()

    def publishStatus(self):
        enabled = self.node_if.get_param('enabled')
  
       
        status_msg = AiDetectorStatus()
        status_msg.name = self.model_name

        status_msg.namespace = self.node_namespace
        status_msg.state = self.state
        status_msg.enabled = enabled

        status_msg.sleep_enabled = self.node_if.get_param('sleep_enabled')
        status_msg.sleep_suspend_sec = self.node_if.get_param('sleep_suspend_time')
        status_msg.sleep_run_sec = self.node_if.get_param('sleep_run_time')

        status_msg.has_img_tiling = self.has_img_tiling
        status_msg.img_tiling = self.node_if.get_param('img_tiling')

        status_msg.overlay_labels = self.node_if.get_param('overlay_labels')
        status_msg.overlay_clf_name = self.node_if.get_param('overlay_clf_name')

        status_msg.threshold = self.node_if.get_param('threshold')
        status_msg.max_rate_hz = self.node_if.get_param('max_rate')

        img_source_topics = []
        img_det_namespaces = []
        img_det_states = []
        img_connects = []
        img_img_lat_times = []
        img_det_lat_times = []
        img_pre_times = []
        img_detect_times = []
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        for img_topic in imgs_info_dict.keys():
                img_source_topics.append(img_topic)
                img_det_namespaces.append(imgs_info_dict[img_topic]['pub_sub_namespace'])
                img_det_states.append( imgs_info_dict[img_topic]['active'])
                img_connects.append(imgs_info_dict[img_topic]['connected'])
                img_img_lat_times.append(imgs_info_dict[img_topic]['image_latency_time'])
                img_det_lat_times.append(imgs_info_dict[img_topic]['detect_latency_time'])
                img_pre_times.append(imgs_info_dict[img_topic]['preprocess_time'])
                img_detect_times.append(imgs_info_dict[img_topic]['detect_time'])
        status_msg.image_source_topics = img_source_topics
        status_msg.image_detector_namespaces = img_det_namespaces
        status_msg.image_detector_states = img_det_states
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


        #self.msg_if.pub_warn("Sending Status Msg: " + str(status_msg))
        if not rospy.is_shutdown():
            self.node_if.publish('status_pub',status_msg)



