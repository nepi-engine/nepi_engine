#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import copy
import time
import numpy as np
import math
import threading
import cv2

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image

from nepi_ros_interfaces.msg import AiDetectorMgrStatus
from nepi_ros_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_ros_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_ais
from nepi_sdk import nepi_img

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodePublishersIF, NodeSubscribersIF, NodeClassIF
from nepi_api.system_if import SaveDataIF, StatesIF, TriggersIF

from nepi_api.connect_mgr_if_system import ConnectMgrSystemServicesIF




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
DEFAULT_THRESHOLD = 0.3

MIN_MAX_RATE = 1
MAX_MAX_RATE = 20
DEFAULT_MAX_PROC_RATE = 5
DEFAULT_MAX_IMG_RATE = DEFAULT_MAX_PROC_RATE * 2


DEFAULT_WAIT_FOR_DETECT = False

DEFAULT_IMG_TILING = False

DEFAULT_LABELS_OVERLAY = True
DEFAULT_CLF_OVERLAY = False
DEFAULT_IMG_OVERLAY = False

GET_IMAGE_TIMEOUT_SEC = 1 

API_LIB_FOLDER = "/opt/nepi/ros/lib/nepi_api"
AIFS_SHARE_PATH = "/opt/nepi/ros/share/nepi_aifs"
USER_CFG_FOLDER = '/mnt/nepi_storage/user_cfg/ros'

class AiDetectorIF:
    
    IMAGE_DATA_PRODUCT = 'detection_image'

    STATES_DICT = dict()
    TRIGGERS_DICT = dict()

    det_img_pub_file = 'nepi_ai_detector_img_pub_node.py'

    data_products = ['bounding_boxes',IMAGE_DATA_PRODUCT]

    det_pub_names = ['found_object', 'bounding_boxes']

    self_managed = True
    model_name = "None"

    last_detect_time = nepi_ros.get_time()
    img_ifs_dict = dict()
    img_ifs_lock = threading.Lock()
    imgs_info_dict = dict()
    imgs_img_proc_dict = dict()


    has_img_tiling = False
    save_cfg_if = None

    state = 'Loading'

    cur_img_topic = "None"
    get_img_topic = "None"
    last_get_image_time = 0
    got_img_topic = "None"

    img_dict = None
    img_dict_lock = threading.Lock()

    first_detect_complete = False

    detection_state = False
    
    time_list = [0,0,0,0,0,0,0,0,0,0]
    sleep_state = False
    def __init__(self, 
                model_name, 
                framework, 
                description, 
                proc_img_height, 
                proc_img_width,  
                classes_list, 
                default_config_dict, 
                all_namespace, 
                preprocessImageFunction, 
                processDetectionFunction,
                has_img_tiling = False,
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.msg_if.pub_debug("Starting Node Class IF Initialization Processes")

        ##############################  
        # Init Class Variables 
        self.has_img_tiling = has_img_tiling

        ## Get folder info
        mgr_sys_srv_if = ConnectMgrSystemServicesIF()
        success = self.mgr_sys_srv_if.wait_for_services()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Status Msg")

        #self.api_lib_folder = mgr_sys_srv_if.get_sys_folder_path('api_lib',API_LIB_FOLDER)
        #self.msg_if.pub_info("Using User Config Folder: " + str(self.api_lib_folder))

        self.api_lib_folder = API_LIB_FOLDER
        self.msg_if.pub_info("Using SDK Share Folder: " + str(self.api_lib_folder))
 
 

        self.status_msg = AiDetectorMgrStatus()
        self.det_status_msg = AiDetectorStatus()


        self.model_name = model_name
        self.model_framework = framework
        self.model_type = 'detection'
        self.model_description = description
        self.model_proc_img_height = proc_img_height
        self.model_proc_img_width = proc_img_width
        self.default_config_dict = default_config_dict
        if all_namespace[-1] == "/":
            all_namespace = all_namespace[:-1]
        self.all_namespace = all_namespace
        self.preprocessImage = preprocessImageFunction
        self.processDetection = processDetectionFunction
        self.classes = classes_list
        self.msg_if.pub_warn("Detector provided classes list: " + str(self.classes))
        num_colors = len(self.classes)
        self.classes_colors = nepi_img.create_bgr_jet_colormap_list(num_colors)
        self.msg_if.pub_warn("Created classes color list: " + str(self.classes_colors))

        self.has_sleep = False

        self.img_data_product = self.IMAGE_DATA_PRODUCT

        ###############################
        # Launch detection img pub node that handles detection image publishing
        pkg_name = 'nepi_api'
        node_file_folder = self.api_lib_folder
        img_pub_file = self.det_img_pub_file
        img_pub_file_path = os.path.join(node_file_folder,img_pub_file)
    
        if os.path.exists(img_pub_file_path) == False:
            self.msg_if.pub_warn("Could not find det img pub node file at: " + img_pub_file_path)
        else: 
           #Try and launch node
            img_pub_node_name = self.node_name + "_img_pub"
            img_pub_namespace = self.node_namespace + "_img_pub"
            all_pub_namespace = os.path.join(self.base_namespace,"ai","all_detectors")
            self.msg_if.pub_warn("Launching Detector Img Pub Node: " + img_pub_node_name)

            # Pre Set Img Pub Params
            dp_param_ns = nepi_ros.create_namespace(img_pub_node_name,'data_product')
            nepi_ros.set_param(dp_param_ns,self.img_data_product)

            det_param_ns = nepi_ros.create_namespace(img_pub_node_name,'det_namespace')
            nepi_ros.set_param(det_param_ns,self.node_namespace)

            all_param_ns = nepi_ros.create_namespace(img_pub_node_name,'all_namespace')
            nepi_ros.set_param(all_param_ns,self.all_namespace)
                     

            [success, msg, pub_process] = nepi_ros.launch_node(pkg_name, img_pub_file, img_pub_node_name)

            self.msg_if.pub_warn("Detector Img Pub Node launch return msg: " + msg)
        


        ##############################  
        # Create NodeClassIF Class  

        # Configs Dict ########################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.node_namespace,
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'enabled': {
                'namespace': self.node_namespace,
                'factory_val': False
            },
            'selected_img_topics': {
                'namespace': self.node_namespace,
                'factory_val': []
            },
            'wait_for_detect': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_WAIT_FOR_DETECT
            },
            'selected_classes': {
                'namespace': self.node_namespace,
                'factory_val': classes_list
            },
            'img_tiling': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_IMG_TILING
            },
            'overlay_labels': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_LABELS_OVERLAY
            },
            'overlay_clf_name': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_CLF_OVERLAY
            },
            'overlay_img_name': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_IMG_OVERLAY
            },
            'threshold': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_THRESHOLD
            },
            'max_proc_rate_hz': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_MAX_PROC_RATE
            },
            'max_img_rate_hz': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_MAX_IMG_RATE
            },
            'sleep_enabled': {
                'namespace': self.node_namespace,
                'factory_val': False
            },
            'sleep_suspend_sec': {
                'namespace': self.node_namespace,
                'factory_val': -1
            },
            'sleep_run_sec': {
                'namespace': self.node_namespace,
                'factory_val': 1
            },
        }



        # Services Config Dict ####################
        self.SRVS_DICT = {
            'info_query': {
                'namespace': self.node_namespace,
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
                'namespace': self.node_namespace,
                'topic': 'found_object',
                'qsize': 1,
                'latch': False
            },
            'bounding_boxes': {
                'msg': BoundingBoxes,
                'namespace': self.node_namespace,
                'topic': 'bounding_boxes',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg': AiDetectorMgrStatus,
                'namespace': self.node_namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
            'det_status_pub': {
                'msg': AiDetectorStatus,
                'namespace': self.node_namespace,
                'topic': 'detector_status',
                'qsize': 1,
                'latch': True
            },
        }

        if all_namespace.find(self.node_name) == -1:
            self_managed = False
            self.PUBS_DICT['found_object_all'] = {
                'msg': ObjectCount,
                'namespace': self.all_namespace,
                'topic': 'found_object',
                'qsize': 1,
                'latch': False
            },
            self.PUBS_DICT['bounding_boxes_all'] = {
                'msg': BoundingBoxes,
                'namespace': self.all_namespace,
                'topic': 'bounding_boxes',
                'qsize': 1,
                'latch': False
            },


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'enable': {
                'namespace': self.node_namespace,
                'topic': 'enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.enableCb, 
                'callback_args': ()
            },
            'add_img_topic': {
                'namespace': self.node_namespace,
                'topic': 'add_img_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.addImageTopicCb, 
                'callback_args': ()
            },
            'add_img_topics': {
                'namespace': self.node_namespace,
                'topic': 'add_img_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.addImageTopicsCb, 
                'callback_args': ()
            },
            'remove_img_topic': {
                'namespace': self.node_namespace,
                'topic': 'remove_img_topic',
                'msg': String,
                'qsize': 10,
                'callback': self.removeImageTopicCb, 
                'callback_args': ()
            },
            'remove_img_topics': {
                'namespace': self.node_namespace,
                'topic': 'remove_img_topics',
                'msg': StringArray,
                'qsize': 10,
                'callback': self.removeImageTopicsCb, 
                'callback_args': ()
            },
            'add_all_classes': {
                'namespace': self.node_namespace,
                'topic': 'add_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.addAllClassesCb, 
                'callback_args': ()
            },
            'remove_all_classes': {
                'namespace': self.node_namespace,
                'topic': 'remove_all_classes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.removeAllClassesCb, 
                'callback_args': ()
            },
            'add_class': {
                'namespace': self.node_namespace,
                'topic': 'add_class',
                'msg': String,
                'qsize': 10,
                'callback': self.addClassCb, 
                'callback_args': ()
            },
            'remove_class': {
                'namespace': self.node_namespace,
                'topic': 'remove_class',
                'msg': String,
                'qsize': 10,
                'callback': self.removeClassCb, 
                'callback_args': ()
            },
            'set_threshold': {
                'namespace': self.node_namespace,
                'topic': 'set_threshold',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setThresholdCb, 
                'callback_args': ()
            },
            'set_img_tiling': {
                'namespace': self.node_namespace,
                'topic': 'set_img_tiling',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setTileImgCb, 
                'callback_args': ()
            },
            'set_overlay_labels': {
                'namespace': self.node_namespace,
                'topic': 'set_overlay_labels',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setOverlayLabelsCb, 
                'callback_args': ()
            },
            'set_overlay_clf_name': {
                'namespace': self.node_namespace,
                'topic': 'set_overlay_clf_name',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setOverlayClfNameCb, 
                'callback_args': ()
            },
            'set_overlay_img_name': {
                'namespace': self.node_namespace,
                'topic': 'set_overlay_img_name',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setOverlayImgNameCb, 
                'callback_args': ()
            },
            'set_max_proc_rate': {
                'namespace': self.node_namespace,
                'topic': 'set_max_proc_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxProcRateCb, 
                'callback_args': ()
            },
            'set_max_img_rate': {
                'namespace': self.node_namespace,
                'topic': 'set_max_img_rate',
                'msg': Float32,
                'qsize': 10,
                'callback': self.setMaxImgRateCb, 
                'callback_args': ()
            },
            'set_sleep_enable': {
                'namespace': self.node_namespace,
                'topic': 'set_sleep_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setSleepEnableCb, 
                'callback_args': ()
            },
            'set_sleep_suspend_sec': {
                'namespace': self.node_namespace,
                'topic': 'set_sleep_suspend_sec',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setSleepSuspendTimeCb, 
                'callback_args': ()
            },
            'set_sleep_run_sec': {
                'namespace': self.node_namespace,
                'topic': 'set_sleep_run_sec',
                'msg': Int32,
                'qsize': 10,
                'callback': self.setSleepSuspendTimeCb, 
                'callback_args': ()
            },
        }



        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT
        )

        self.node_if.wait_for_ready()

        ###############################
        # Create System IFs

        # Setup States IF
        self.states_dict = {
                        "running": {
                            "name":"running",
                            "node_name": self.node_name,
                            "description": "Current detection running state",
                            "type":"Bool",
                            "options": [],
                            "value":"False"
                            },
                        "detection": {
                            "name":"detection",
                            "node_name": self.node_name,
                            "description": "Current detection state, cleared every 1 sec",
                            "type":"Bool",
                            "options": [],
                            "value":"False"
                            }
        }
        self.states_if = StatesIF(get_states_dict_function = self.get_states_dict_function,
                        msg_if = self.msg_if)


        # Setup Triggers IF
        self.triggers_dict = {
                        "detection_trigger": {
                            "name":"detection_trigger",
                            "node_name": self.node_name,
                            "description": "Triggered on AI detection",
                            "data_str_list":["None"],
                            "time":nepi_utils.get_time()
                            }

        }

        self.triggers_if = TriggersIF(triggers_dict = self.triggers_dict,
                        msg_if = self.msg_if)

        
        # Setup Save Data IF
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if self.img_data_product in self.data_products:
            factory_data_rates[self.img_data_product] = [1.0, 0.0, 100.0] 

        self.save_data_if = SaveDataIF(data_products = self.data_products, factory_rate_dict = factory_data_rates)
        
        time.sleep(1)


        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_ros.start_timer_process((1.0), self.publishStatusCb)
        nepi_ros.start_timer_process((1.0), self.updateDetectionState)
        nepi_ros.start_timer_process((0.1), self.updaterCb, oneshot = True)
        nepi_ros.start_timer_process((0.1), self.updateDetectCb, oneshot = True)

        self.state = 'Loaded'
        ##########################
        self.msg_if.pub_info("IF Initialization Complete")
        ##########################


    def create_classes_colors_msg(self,classes_list):
        colors_msg_list = []
        for class_name in classes_list:
            color_msg  = ColorRGBA()
            class_ind = classes_list.index(class_name)
            if len(self.classes_colors) >= class_ind:
                color = self.classes_colors[class_ind]
                color_msg.r = color[2]
                color_msg.g = color[1]
                color_msg.b = color[0]
            else:
                color_msg.r = 0
                color_msg.g = 200
                color_msg.b = 0
            colors_msg_list.append(color_msg)
        return colors_msg_list




    def get_states_dict_function(self):
        return self.states_dict


    def initCb(self,do_updates = False):
        self.msg_if.pub_info(" Setting init values to param values")
        if do_updates == True:
            self.resetCb(do_updates)


    def resetCb(self):
        self.publish_status()

    def factoryResetCb(self):
        self.last_image_topic = ""
        self.publish_status()



    def enableCb(self,msg):
        enabled = msg.data
        self.node_if.set_param('enabled', enabled)
        self.publish_status()
        time.sleep(1)
        if msg.data == False and not nepi_ros.is_shutdown():
            self.get_img_topic = "None"

    def addAllClassesCb(self,msg):
        self.addAllClasses()

    def addAllClasses(self):
        ##self.msg_if.pub_info(msg)

        self.det_status_msg.selected_classes = self.classes
        self.det_status_msg.selected_classes = self.create_classes_colors_msg(self.classes)
        self.publish_status(do_updates = False) # Updated Here

        self.node_if.set_param('selected_classes', self.classes)


    def removeAllClassesCb(self,msg):
        ##self.msg_if.pub_info(msg)
        self.det_status_msg.selected_classes = []
        self.det_status_msg.selected_classes = []
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('selected_classes',[])


    def addClassCb(self,msg):
        ##self.msg_if.pub_info(msg)
        class_name = msg.data
        if class_name in self.classes:
            sel_classes = self.node_if.get_param('selected_classes')
            if class_name not in sel_classes:
                sel_classes.append(class_name)
            self.det_status_msg.selected_classes = sel_classes
            self.det_status_msg.selected_classes_colors = self.create_classes_colors_msg(sel_classes)
            self.publish_status(do_updates = False) # Updated Here
            self.node_if.set_param('selected_classes', sel_classes)


    def removeClassCb(self,msg):
        ##self.msg_if.pub_info(msg)
        class_name = msg.data
        sel_classes = self.node_if.get_param('selected_classes')
        if class_name in sel_classes:
            sel_classes.remove(class_name)
        self.det_status_msg.selected_classes = sel_classes
        self.det_status_msg.selected_classes_colors = self.create_classes_colors_msg(sel_classes)
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('selected_classes', sel_classes)



    def addImageTopicCb(self,msg):
        self.msg_if.pub_info("Received Add Image Topic: " + msg.data)
        img_topic = msg.data
        self.addImageTopic(img_topic)


    def addImageTopicsCb(self,msg):
        self.msg_if.pub_info("Received Add Image Topics: " + str(msg))
        img_topic_list = msg.entries
        for img_topic in img_topic_list:
            self.addImageTopic(img_topic)


    def addImageTopic(self,img_topic):   
        self.msg_if.pub_info("Adding Image Topic: " + img_topic)
        img_topics = self.node_if.get_param('selected_img_topics')
        if img_topic not in img_topics:
            img_topics.append(img_topic)
        self.det_status_msg.selected_img_topics = img_topics
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('selected_img_topics', img_topics)


    def removeImageTopicCb(self,msg):
        self.msg_if.pub_info("Received Remove Image Topic: " + str(msg))
        img_topic = msg.data
        self.removeImageTopic(img_topic)


    def removeImageTopicsCb(self,msg):
        self.msg_if.pub_info("Received Remove Image Topic: " + str(msg))
        img_topic_list = msg.entries
        for img_topic in img_topic_list:
            self.removeImageTopic(img_topic)

    def removeImageTopic(self,img_topic):
        self.msg_if.pub_info("Removing Image Topic: " + img_topic)         
        img_topics = self.node_if.get_param('selected_img_topics')
        if img_topic in img_topics:
            img_topics.remove(img_topic)
        
        self.det_status_msg.selected_img_topics = img_topics
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('selected_img_topics', img_topics)




    def setOverlayLabelsCb(self,msg):
        self.det_status_msg.overlay_labels = msg.data
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('overlay_labels', msg.data)


    def setOverlayClfNameCb(self,msg):
        self.det_status_msg.overlay_clf_name = msg.data
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('overlay_clf_name', msg.data)


    def setOverlayImgNameCb(self,msg):
        self.det_status_msg.overlay_img_name = msg.data
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('overlay_img_name', msg.data)


    def setMaxProcRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        self.det_status_msg.max_proc_rate_hz = max_rate
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('max_proc_rate_hz', max_rate)


    def setMaxImgRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        self.det_status_msg.max_img_rate_hz = max_rate
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('max_img_rate_hz', max_rate)


    def setTileImgCb(self,msg):
        self.status_msg.img_tiling = msg.data
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('img_tiling', msg.data)






    def setThresholdCb(self,msg):
        threshold = msg.data
        self.msg_if.pub_info("Received Threshold Update: " + str(threshold))
        if threshold <  MIN_THRESHOLD:
            threshold = MIN_THRESHOLD
        elif threshold > MAX_THRESHOLD:
            threshold = MAX_THRESHOLD
        self.det_status_msg.threshold = threshold
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('threshold', threshold)





    def setSleepEnableCb(self,msg):
        self.status_msg.sleep_enabled = msg.data
        self.publish_status(do_updates = False) # Updated Here
        self.node_if.set_param('sleep_enabled', msg.data)



    def setSleepSuspendTimeCb(self,msg):
        data = msg.data
        if data > 1 or suspend_time == -1:
            self.status_msg.sleep_suspend_sec = data
            self.publish_status(do_updates = False) # Updated Here
            self.node_if.set_param('sleep_suspend_sec', data)


    def setSleepSuspendTimeCb(self,msg):
        data = msg.data
        if data > 1:
            self.status_msg.sleep_run_sec = data
            self.publish_status(do_updates = False) # Updated Here
            self.node_if.set_param('sleep_run_sec', data)



    def statusPublishCb(self,timer):
        self.publish_status()

    def getActiveImgTopics(self):
        active_img_topics = []
        img_topics = self.imgs_info_dict.keys()
        for i,img_topic in enumerate(img_topics):
            img_active =  self.imgs_info_dict[img_topic]['active']
            if img_active == True:
                active_img_topics.append(img_topic)
        return active_img_topics


    def updaterCb(self,timer):
        #self.msg_if.pub_warn("Updating with image topic: " +  self.img_topic)
        img_topics = self.node_if.get_param('selected_img_topics')
        self.img_ifs_lock.acquire()
        registered_img_topics = self.img_ifs_dict.keys()
        active_img_topics = self.getActiveImgTopics()
        self.img_ifs_lock.release()
        # Update Image subscribers
        for i,img_topic in enumerate(img_topics):
            img_topic = nepi_ros.find_topic(img_topic)
            if img_topic != '':
                img_topics[i] = img_topic
                if img_topic not in active_img_topics:
                    success = self.subscribeImgTopic(img_topic)    
          
        purge_list = []
        for img_topic in active_img_topics:
            if img_topic not in img_topics:
                purge_list.append(img_topic)
        #self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
        for topic in purge_list:
            self.msg_if.pub_warn('Will unsubscribe from image topic: ' + topic)
            success = self.unsubscribeImgTopic(topic)

        # Set Image connected state
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        img_connects = []
        for img_topic in imgs_info_dict.keys():
            img_connects.append(imgs_info_dict[img_topic]['connected'])
        img_selected = len(img_connects) > 0
        img_connected = True in img_connects
       
        # Set Detector State
        enabled = self.det_status_msg.enabled
        if enabled == True:
            if img_selected == 0:
                self.state = "Waiting"
            elif img_connected == False:
                self.state = "Listening"
            elif self.sleep_state == True:
                self.state = "Sleeping"
            else:
                self.state = "Running"
        else: # Loaded, but not enabled
            self.state = "Loaded"

        nepi_ros.start_timer_process((.1), self.updaterCb, oneshot = True)


    def subscribeImgTopic(self,img_topic):
        if img_topic == "None" or img_topic == "":
            return False
        else:

            img_sub_dict = {
                        'namespace': img_topic,
                        'msg': Image,
                        'topic': '',
                        'qsize': 1,
                        'callback': self.imageCb,
                        'callback_args': (img_topic)
                    }

            imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
            # Check if exists
            if img_topic in imgs_info_dict.keys():
                if self.img_ifs_dict[img_topic]:
                    self.msg_if.pub_info('Subsribing to image topic: ' + img_topic)  
                    # Try and Reregister img_sub
                    self.img_ifs_lock.acquire()
                    self.img_ifs_dict[img_topic]['subs_if'].register_sub('img_sub',img_sub_dict)
                    self.img_ifs_lock.release()
                    self.msg_if.pub_warn('Registered : ' + img_topic +  ' ' + str(self.img_ifs_dict[img_topic]))
                    # Set back to active
                    self.imgs_info_dict[img_topic]['active'] = True
            else:
                    
                # Create register new image topic
                self.msg_if.pub_info('Registering to image topic: ' + img_topic)
                img_name = img_topic.replace(self.base_namespace,"")
                pub_namespace = os.path.join(self.node_namespace,img_name)

                ####################
                # Create img info dict
                self.imgs_info_dict[img_topic] = dict()  
                self.imgs_info_dict[img_topic]['namespace'] = pub_namespace
                self.imgs_info_dict[img_topic]['connected'] = False 
                self.imgs_info_dict[img_topic]['active'] = True
                self.imgs_info_dict[img_topic]['image_latency_time'] = 0
                self.imgs_info_dict[img_topic]['detect_latency_time'] = 0
                self.imgs_info_dict[img_topic]['preprocess_time'] = 0 
                self.imgs_info_dict[img_topic]['detect_time'] = 0  


                self.msg_if.pub_info('Subsribing to image topic: ' + img_topic)
                ####################
                # Pubs Config Dict 
                IMG_PUBS_DICT = {
                    'found_object': {
                        'msg': ObjectCount,
                        'namespace': pub_namespace,
                        'topic': 'found_object',
                        'qsize': 1,
                        'latch': False
                    },
                    'bounding_boxes': {
                        'msg': BoundingBoxes,
                        'namespace': pub_namespace,
                        'topic': 'bounding_boxes',
                        'qsize': 1,
                        'latch': False
                    }
                }

                img_pubs_if = NodePublishersIF(
                                pubs_dict = IMG_PUBS_DICT
                )

                ####################
                # Pubs Config Dict 
                SUBS_DICT = {
                    'img_sub': img_sub_dict
                }

                ####################
                # Subs Config Dict 
                img_subs_if = NodeSubscribersIF(
                                subs_dict = SUBS_DICT,
                                log_class_name = False
                )


                time.sleep(1)

                ####################
                # Add Img Subs and Pubs IFs to Img IFs Dict
                self.img_ifs_lock.acquire()
                self.img_ifs_dict[img_topic] = {
                                                'pubs_if': img_pubs_if,
                                                'subs_if': img_subs_if
                                                }   

                self.img_ifs_lock.release()
                self.msg_if.pub_warn('Registered : ' + img_topic)


                ####################
                # Publish blank msg to prime topics
                img_pubs_if.publish_pub('found_object',ObjectCount())
                img_pubs_if.publish_pub('bounding_boxes',BoundingBoxes())    
    
            return True

    def publishDetMsg(self,img_topic, pub_name, msg):
        self.node_if.publish_pub(pub_name,msg)
        pub_name_all = pub_name + "_all"
        self.node_if.publish_pub(pub_name_all,msg)
        if img_topic in self.img_ifs_dict.keys():
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[img_topic]['pubs_if'].publish_pub(pub_name,msg)
            self.img_ifs_lock.release()
                 

    def unsubscribeImgTopic(self,img_topic):
        if img_topic in self.img_ifs_dict.keys():
            self.msg_if.pub_warn('Unregistering image topic: ' + img_topic)
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[img_topic]['subs_if'].unregister_sub('img_sub')
            self.img_ifs_lock.release()
        #Leave img pub running in case it is switched back on
    
        # Clear info dict
        if img_topic in self.imgs_info_dict.keys():
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
                imgs_info_dict[img_topic]['connected'] = True
                enabled = self.det_status_msg.enabled
                if enabled == True and self.sleep_state == False:
                    # Process ros image message
                    current_time = nepi_ros.ros_time_now()
                    ros_timestamp = image_msg.header.stamp
                    latency = (current_time.to_sec() - ros_timestamp.to_sec())
                    imgs_info_dict[img_topic]['image_latency_time'] = latency
                    #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency))


                    get_image = (img_topic == self.get_img_topic)
                    #self.msg_if.pub_warn("Callback got image from topic:  " + img_topic + " with get topic " + self.get_img_topic)
                    if get_image == True:
                        self.get_img_topic = "None"

                        #self.msg_if.pub_warn("Processing img for topic:  " + img_topic)
                        ##############################
                        ### Preprocess Image
                        
                        options_dict = dict()
                        options_dict['tile'] = self.status_msg.img_tiling
                        cv2_img = nepi_img.rosimg_to_cv2img(image_msg)

                        '''
                        img_dict = dict()
                        img_dict['cv2_img'] = cv2_img
                        '''
                        ros_timestamp = image_msg.header.stamp
                        img_dict = self.preprocessImage(cv2_img,options_dict)
                        img_dict['topic'] = img_topic
                        img_dict['timestamp'] = nepi_ros.sec_from_ros_time(ros_timestamp)
                        img_dict['msg_header'] = image_msg.header
                        img_dict['msg_stamp'] = ros_timestamp
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
        enabled = self.det_status_msg.enabled
        if enabled == True:
            img_topics = self.det_status_msg.image_source_topics
            connected_list = []
            for topic in img_topics:
                if topic in imgs_info_dict.keys():
                    if imgs_info_dict[topic]['connected'] == True:
                        connected_list.append(topic)
            if len(connected_list) == 0:
                self.get_img_topic = "None"
            else:
                # check timer
                max_rate = self.det_status_msg.max_proc_rate_hz
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
                        detect_delay = nepi_utils.get_time() - self.last_detect_time
                        self.time_list.pop(0)
                        self.time_list.append(detect_delay)
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
                                    threshold = self.det_status_msg.threshold
                                    detect_dicts = self.processDetection(img_dict,threshold) 
                                    #self.msg_if.pub_warn("AIF got back detect_dict: " + str(detect_dict_list))
                                    success = True
                                    self.first_detect_complete = True



                                except Exception as e:
                                    self.msg_if.pub_warn("Failed to process detection img with exception: " + str(e))
                                ros_img_header = img_dict['msg_header']
                                # Filter selected classes
                                sel_classes = self.det_status_msg.selected_classes
                                sel_detect_ind = []
                                for i, detect in enumerate(detect_dicts):
                                    if detect['name'] in sel_classes:
                                        sel_detect_ind.append(i)
                                for ind in sel_detect_ind:
                                    detect_dict_list.append(detect_dicts[ind])

                                self.publishDetectionData(img_dict,detect_dict_list,ros_img_header)
                                
                            detect_time = round( (nepi_ros.get_time() - start_time) , 3)
                            imgs_info_dict[img_topic]['detect_time'] = detect_time
                            #self.msg_if.pub_info("Detect Time: {:.2f}".format(detect_time))

                        current_time = nepi_ros.ros_time_now()
                        ros_timestamp = img_dict['msg_stamp']
                        latency = (current_time.to_sec() - ros_timestamp.to_sec())
                        imgs_info_dict[img_topic]['detect_latency_time'] = latency
                        #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency))

                        if img_topic in self.imgs_info_dict.keys():
                            try:
                                self.imgs_info_dict[img_topic] = imgs_info_dict[img_topic]
                            except:
                                pass


                       
        nepi_ros.start_timer_process((0.01), self.updateDetectCb, oneshot = True)

    def publishDetectionData(self,img_dict, detect_dict_list,ros_img_header):
        det_count = len(detect_dict_list)
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
                    bb_msg_list.append(bb_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get all data from detect dict: " + str(e)) 
            bbs_msg = BoundingBoxes()
            bbs_msg.header.stamp = nepi_ros.ros_time_now()
            bbs_msg.model_name = self.node_name
            bbs_msg.image_header = ros_img_header
            bbs_msg.image_topic = img_topic
            bbs_msg.image_width = img_dict['image_width']
            bbs_msg.image_height = img_dict['image_height']
            bbs_msg.prc_width = img_dict['prc_width']
            bbs_msg.prc_height = img_dict['prc_height']
            bbs_msg.bounding_boxes = bb_msg_list

            self.publishDetMsg(img_topic,'bounding_boxes',bbs_msg)
                                       
            found_object_msg = ObjectCount()
            found_object_msg.header.stamp = nepi_ros.ros_time_now()
            found_object_msg.model_name = self.node_name
            found_object_msg.image_header = ros_img_header
            found_object_msg.image_topic = img_topic
            found_object_msg.count = det_count

            self.publishDetMsg(img_topic,'found_object',found_object_msg)

            if det_count > 0:
                trigger_dict = self.triggers_dict['detection_trigger']
                trigger_dict['time']=nepi_utils.get_time()
                self.triggers_if.publish_trigger(trigger_dict)

                self.detection_state = True
             

            # Save Bounding Data if needed
            image_text = img_topic.replace(self.base_namespace,"")
            image_text = image_text.replace('/','_')
            if len(detect_dict_list) > 0:
                data_product = 'bounding_boxes'
                ros_timestamp = bbs_msg.header.stamp
                bbs_dict = nepi_ais.get_boxes_info_from_msg(bbs_msg)
                bb_dict_list = nepi_ais.get_boxes_list_from_msg(bbs_msg)
                bbs_dict['bounding_boxes']=bb_dict_list
                self.save_data_if.save(data_product,'bounding_boxes',ros_img_header.stamp)
            



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
        resp.classes = self.classes
        resp.has_img_tiling = self.has_img_tiling

        img_source_topics = []
        img_det_namespaces = []
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        for img_topic in imgs_info_dict.keys():
                state = imgs_info_dict[img_topic]['active']
                if state == True:
                    img_source_topics.append(img_topic)
                    img_det_namespaces.append(imgs_info_dict[img_topic]['namespace'])
        resp.image_source_topics = img_source_topics
        resp.image_detector_namespaces = img_det_namespaces


        return resp
    

    def updateDetectionState(self,timer):
        # Update and clear detection state every second
        self.states_dict['detection']['value'] = str(self.detection_state)
        self.detection_state = False


    def publishStatusCb(self,timer):
        self.publish_status()

    def publish_status(self, do_updates = True):

        # Pub Status
        self.status_msg.name = self.model_name
        self.status_msg.has_sleep = self.has_sleep

        if do_updates == True and not nepi_ros.is_shutdown():
            self.status_msg.sleep_enabled = self.node_if.get_param('sleep_enabled')
            self.status_msg.sleep_suspend_sec = self.node_if.get_param('sleep_suspend_sec')
            self.status_msg.sleep_run_sec = self.node_if.get_param('sleep_run_sec')
            self.status_msg.sleep_state = self.sleep_state

            self.status_msg.img_tiling = self.node_if.get_param('img_tiling')

        #self.msg_if.pub_warn("Sending Status Msg: " + str(self.det_status_msg))
        self.node_if.publish_pub('status_pub',self.status_msg)

        # Pub Deteccton Status

        self.det_status_msg.name = self.model_name
        self.det_status_msg.namespace = self.node_namespace
        self.det_status_msg.state = self.state
        self.det_status_msg.available_classes = self.classes


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
                state = imgs_info_dict[img_topic]['active']
                if state == True:
                    img_source_topics.append(img_topic)
                    img_det_namespaces.append(imgs_info_dict[img_topic]['namespace'])
                    img_connects.append(imgs_info_dict[img_topic]['connected'])
                    img_img_lat_times.append(imgs_info_dict[img_topic]['image_latency_time'])
                    img_det_lat_times.append(imgs_info_dict[img_topic]['detect_latency_time'])
                    img_pre_times.append(imgs_info_dict[img_topic]['preprocess_time'])
                    img_detect_times.append(imgs_info_dict[img_topic]['detect_time'])
        self.det_status_msg.image_source_topics = img_source_topics
        self.det_status_msg.image_detector_namespaces = img_det_namespaces
        self.det_status_msg.images_connected = img_connects
        self.det_status_msg.image_latency_times = img_img_lat_times
        self.det_status_msg.detect_latency_times = img_det_lat_times
        self.det_status_msg.preprocess_times = img_pre_times
        self.det_status_msg.detect_times = img_detect_times

        img_selected = len(img_connects) > 0
        self.det_status_msg.image_selected = img_selected
        img_connected = True in img_connects
        self.det_status_msg.image_connected = img_connected

        img_lat_time = 0.0
        det_lat_time = 0.0
        pre_time = 0.0
        detect_time = 0.0
        if img_connected:
            img_lat_time = sum(img_img_lat_times) / len(img_img_lat_times)
            det_lat_time = sum(img_det_lat_times) / len(img_det_lat_times)
            pre_time = sum(img_pre_times) / len(img_pre_times)
            detect_time = sum(img_detect_times) / len(img_detect_times)
        self.det_status_msg.image_latency_time = img_lat_time
        self.det_status_msg.detect_latency_time = det_lat_time
        self.det_status_msg.preprocess_time = pre_time
        self.det_status_msg.detect_time = detect_time



        avg_rate = 0
        avg_time = sum(self.time_list) / len(self.time_list)
        if avg_time > .01:
            avg_rate = float(1) / avg_time
       
        self.det_status_msg.avg_rate_hz = avg_rate


        if do_updates == True and not nepi_ros.is_shutdown():
            self.det_status_msg.enabled = self.node_if.get_param('enabled')
            sel_classes = self.node_if.get_param('selected_classes')
            self.det_status_msg.selected_classes = sel_classes
            self.det_status_msg.selected_classes_colors = self.create_classes_colors_msg(sel_classes)
               

            self.status_msg.sleep_enabled = self.node_if.get_param('sleep_enabled')
            self.status_msg.sleep_suspend_sec = self.node_if.get_param('sleep_suspend_sec')
            self.status_msg.sleep_run_sec = self.node_if.get_param('sleep_run_sec')
            self.status_msg.sleep_state = self.sleep_state

            self.status_msg.img_tiling = self.node_if.get_param('img_tiling')

            self.det_status_msg.overlay_labels = self.node_if.get_param('overlay_labels')
            self.det_status_msg.overlay_clf_name = self.node_if.get_param('overlay_clf_name')
            self.det_status_msg.overlay_img_name = self.node_if.get_param('overlay_img_name')

            self.det_status_msg.threshold = self.node_if.get_param('threshold')
            self.det_status_msg.max_proc_rate_hz = self.node_if.get_param('max_proc_rate_hz')
            self.det_status_msg.max_img_rate_hz = self.node_if.get_param('max_img_rate_hz')

            self.det_status_msg.selected_img_topics = self.node_if.get_param('selected_img_topics')

        #self.msg_if.pub_warn("Sending Detection Status Msg: " + str(self.det_status_msg))
        self.node_if.publish_pub('det_status_pub',self.det_status_msg)



