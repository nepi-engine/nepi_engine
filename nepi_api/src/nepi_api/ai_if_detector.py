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
import copy
import numpy as np
import math
import threading
import cv2

from std_msgs.msg import UInt8, Int32, Float32, Bool, Empty, String, Header
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image

from nepi_interfaces.msg import MgrAiDetectorStatus
from nepi_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes
from nepi_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_interfaces.srv import SystemStorageFolderQuery
from nepi_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_ais
from nepi_sdk import nepi_img

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodePublishersIF, NodeSubscribersIF, NodeClassIF
from nepi_api.system_if import SaveDataIF, StatesIF, TriggersIF






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
DEFAULT_MAX_PROC_RATE = 1
DEFAULT_MAX_IMG_RATE = 5
DEFAULT_USE_LAST_IMAGE = True

DEFAULT_WAIT_FOR_DETECT = False

DEFAULT_IMG_TILING = False

DEFAULT_LABELS_OVERLAY = True
DEFAULT_CLF_OVERLAY = False
DEFAULT_IMG_OVERLAY = False

GET_IMAGE_TIMEOUT_SEC = 1 



class AiDetectorIF:
    
    IMAGE_DATA_PRODUCT = 'detection_image'
    IMAGE_SUB_MSG = 'Waiting for Source Image'
    IMAGE_PUB_MSG = 'Loading Image Publisher'

    BLANK_SIZE_DICT = { 'h': 350, 'w': 700, 'c': 3}
    BLANK_CV2_IMAGE = nepi_img.create_blank_image((BLANK_SIZE_DICT['h'],BLANK_SIZE_DICT['w'],BLANK_SIZE_DICT['c']))

    namespace = '~'
    all_namespace = None

    states_dict = None
    triggers_dict = dict()

    node_if = None

    

    data_products = ['bounding_boxes',IMAGE_DATA_PRODUCT]

    det_pub_names = [ 'bounding_boxes']

    self_managed = True
    model_name = "None"

    last_detect_time = nepi_sdk.get_time()
    img_ifs_dict = dict()
    img_ifs_lock = threading.Lock()
    imgs_info_dict = dict()
    imgs_img_proc_dict = dict()


    has_img_tiling = False

    state = 'Loading'

    cur_img_topic = "None"

    get_img_topic = "None"
    got_img_topic = "None"

    get_img_file = False
    got_img_file = False

    imgs_dict = dict()
    imgs_has_subs_dict = dict()

    first_detect_complete = False

    detection_state = False
    detect_latencies = [0,0,0,0,0,0,0,0,0,0]
    detect_delays = [0,0,0,0,0,0,0,0,0,0]
    detect_times = [0,0,0,0,0,0,0,0,0,0]
    sleep_state = False


    enabled = False
    selected_classes = []
    sleep_enabled = False
    sleep_suspend_sec = 0
    sleep_run_sec = 0
    img_tiling = False
    overlay_labels = True
    overlay_targeting = True
    overlay_clf_name = False
    overlay_img_name = False
    threshold = DEFAULT_THRESHOLD
    max_proc_rate_hz = DEFAULT_MAX_PROC_RATE
    max_img_rate_hz = DEFAULT_MAX_IMG_RATE
    use_last_image = DEFAULT_USE_LAST_IMAGE
    selected_img_topics = []

    pub_image_enabled=True
    launch_node_process=None
    pub_img_node_name = ""
    pub_img_namepace = ""

    img_folder_path=None
    img_file_path=None

    next_image_topic="None"

    def __init__(self, 
                namespace,
                model_name, 
                framework, 
                description, 
                proc_img_height, 
                proc_img_width,  
                classes_list, 
                default_config_dict, 
                processDetectionFunction,
                all_namespace = None,
                has_img_tiling = False,
                enable_image_pub = True,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_debug("Starting Node Class IF Initialization Processes", log_name_list = self.log_name_list)


 
        ##############################
        # Get for System Folders
        self.msg_if.pub_warn("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        self.msg_if.pub_warn("Got system folders: " + str(system_folders))
       
        self.api_lib_folder = system_folders['api_lib']
        self.msg_if.pub_info("Using SDK Share Folder: " + str(self.api_lib_folder))
 

        ##############################  
        # Init Class Variables 

        if namespace is None:
            namespace = self.node_namespace
        namespace = nepi_sdk.create_namespace(namespace,'settings')
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.enable_image_pub = enable_image_pub
        self.has_img_tiling = has_img_tiling

        
        ## Init Status Messages
        self.status_msg = MgrAiDetectorStatus()
        self.det_status_msg = AiDetectorStatus()


        self.model_name = model_name
        self.model_framework = framework
        self.model_type = 'detection'
        self.model_description = description
        self.model_proc_img_height = proc_img_height
        self.model_proc_img_width = proc_img_width
        self.default_config_dict = default_config_dict
        if all_namespace is None:
            all_namespace = ''
        else:
            if all_namespace[-1] == "/":
                all_namespace = all_namespace[:-1]
        self.all_namespace = all_namespace
        self.processDetection = processDetectionFunction
        self.classes = classes_list
        self.msg_if.pub_warn("Detector provided classes list: " + str(self.classes))

        self.has_sleep = False

        self.img_data_product = self.IMAGE_DATA_PRODUCT

        self.selected_classes = self.classes

     
        self.initCb(do_updates = False)

        ##############################
        ### Setup Node

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
                'factory_val': self.selected_classes
            },
            'img_tiling': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_IMG_TILING
            },
            'overlay_labels': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_LABELS_OVERLAY
            },
            'overlay_targeting': {
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
            'use_last_image': {
                'namespace': self.node_namespace,
                'factory_val': DEFAULT_USE_LAST_IMAGE
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
            'pub_image_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.pub_image_enabled
            }
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
            'bounding_boxes_all': {
                'msg': BoundingBoxes,
                'namespace': self.node_namespace,
                'topic': 'bounding_boxes',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg': MgrAiDetectorStatus,
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

        if self.enable_image_pub == True:
            self.PUBS_DICT['image_pub'] = {
                'msg': Image,
                'namespace': self.node_namespace,
                'topic': 'detection_image',
                'qsize': 1,
                'latch': False
            }

        if all_namespace != '':
            self_managed = False
            self.PUBS_DICT['bounding_boxes_all'] = {
                'msg': BoundingBoxes,
                'namespace': self.all_namespace,
                'topic': 'bounding_boxes',
                'qsize': 1,
                'latch': False
            }


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
            'select_img_file': {
                'namespace': self.node_namespace,
                'topic': 'select_img_file',
                'msg': String,
                'qsize': 10,
                'callback': self.selectImageFileCb, 
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
            'set_image_pub': {
                'namespace': self.node_namespace,
                'topic': 'set_image_pub',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setPubImageCb, 
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
            'set_overlay_targeting': {
                'namespace': self.node_namespace,
                'topic': 'set_overlay_targeting',
                'msg': Bool,
                'qsize': 10,
                'callback': self.setOverlayTargetingCb, 
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
            'set_use_last_image': {
                'namespace': self.node_namespace,
                'topic': 'set_use_last_image',
                'msg':Bool,
                'qsize': 10,
                'callback': self.setUseLastImageCb, 
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
            }
        }



        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        #self.node_if.wait_for_ready()
        nepi_sdk.sleep(1)

        self.initCb(do_updates = True)

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
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )



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
            factory_data_rates[d] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
        if self.img_data_product in self.data_products:
            factory_data_rates[self.img_data_product] = [1.0, 0.0, 100] 

        self.save_data_if = SaveDataIF(data_products = self.data_products, factory_rate_dict = factory_data_rates,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        
        time.sleep(1)


        ##########################
        # Complete Initialization

        # Start Timer Processes
        nepi_sdk.start_timer_process((1.0), self.publishStatusCb)
        nepi_sdk.start_timer_process((1.0), self.updateStatesCb)
        nepi_sdk.start_timer_process((0.1), self.updateImgSubsCb, oneshot = True)
        nepi_sdk.start_timer_process((0.1), self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process((0.1), self.updateDetectCb, oneshot = True)

        self.state = 'Loaded'
        ##########################
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ##########################


    def launch_image_pub_node(self):
            node_name = self.node_name + "_img_pub"
            launch_namespace=os.path.dirname(self.node_namespace)
            node_namespace = self.node_namespace + "_img_pub"
            pkg_name = 'nepi_api'
            node_file_folder = self.api_lib_folder
            node_file_name = 'nepi_ai_detector_img_pub_node.py'
           

            self.msg_if.pub_warn("Launching Detction Img Node with with settings " + str([pkg_name, node_file_name, node_name]))          
            ###############################
            # Launch Node
            node_file_path = os.path.join(node_file_folder,node_file_name)
            if self.launch_node_process is not None:
                self.msg_if.pub_warn("Node Already Launched: " + node_name)
            elif os.path.exists(node_file_path) == False or self.enable_image_pub == False:
                self.msg_if.pub_warn("Could not find Node File at: " + node_file_path)
            else: 
                #Try and launch node
                
                all_pub_namespace = os.path.join(self.base_namespace,"ai","all_detectors")
                self.msg_if.pub_warn("Launching Node: " + node_name)

                # Pre Set Img Pub Params
                param_ns = nepi_sdk.create_namespace(node_namespace,'data_product')
                nepi_sdk.set_param(param_ns,self.img_data_product)

                param_ns = nepi_sdk.create_namespace(node_namespace,'det_namespace')
                nepi_sdk.set_param(param_ns,self.node_namespace)

                param_ns = nepi_sdk.create_namespace(node_namespace,'all_namespace')
                nepi_sdk.set_param(param_ns,self.all_namespace)
                        

                [success, msg, sub_process] = nepi_sdk.launch_node(pkg_name, node_file_name, node_name, namespace=launch_namespace)
                if success == True:
                    self.launch_node_process = sub_process
                    self.pub_img_node_name = node_name
                    self.pub_img_namepace = node_namespace
                self.msg_if.pub_warn("Node launch return msg: " + str(msg))

    def kill_image_pub_node(self):
        if self.launch_node_process is None:
            self.msg_if.pub_warn("Node Not Running")
        else:
            self.msg_if.pub_warn("Killing Node")
            success = nepi_sdk.kill_node_process(self.pub_img_node_name, self.launch_node_process)
            if success == True:
                self.launch_node_process = None
                self.pub_img_node_name = ""
                self.pub_img_namepace = ""
                self.msg_if.pub_warn("Node Killed")
            else:
                self.msg_if.pub_warn("Failed to Kill Node")


    def get_states_dict_function(self):    
        return self.states_dict


    def initCb(self,do_updates = False):
        self.msg_if.pub_info("Setting init values to param values", log_name_list = self.log_name_list)
        if self.node_if is not None:
            self.pub_image_enabled = self.node_if.get_param('pub_image_enabled')
            self.msg_if.pub_warn("Launcing Image Pub Node")
            self.launch_image_pub_node()

            self.enabled = self.node_if.get_param('enabled')
            self.selected_classes = self.node_if.get_param('selected_classes')
            self.sleep_enabled = self.node_if.get_param('sleep_enabled')
            self.sleep_suspend_sec = self.node_if.get_param('sleep_suspend_sec')
            self.sleep_run_sec = self.node_if.get_param('sleep_run_sec')
            self.img_tiling = self.node_if.get_param('img_tiling')
            self.overlay_labels = self.node_if.get_param('overlay_labels')
            self.overlay_targeting = self.node_if.get_param('overlay_targeting')
            self.overlay_clf_name = self.node_if.get_param('overlay_clf_name')
            self.overlay_img_name = self.node_if.get_param('overlay_img_name')
            self.threshold = self.node_if.get_param('threshold')
            self.max_proc_rate_hz = self.node_if.get_param('max_proc_rate_hz')
            self.max_img_rate_hz = self.node_if.get_param('max_img_rate_hz')
            self.use_last_image = self.node_if.get_param('use_last_image')
            self.selected_img_topics = self.node_if.get_param('selected_img_topics')
            self.msg_if.pub_info("Init selected images: " + str(self.selected_img_topics), log_name_list = self.log_name_list)

            self.node_if.save_config()
        if do_updates == True:
            pass
        self.publish_status()

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass # self.node_if.reset_params()
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        if self.node_if is not None:
            pass # self.node_if.factory_reset_params()
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)



    def enableCb(self,msg):
        self.msg_if.pub_warn("Received AI enable msg: " + str(msg))
        enabled = msg.data
        self.enabled = enabled
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('enabled',self.enabled)
            #self.node_if.save_config()
        if msg.data == False and not nepi_sdk.is_shutdown():
            self.next_image_topic = "None"

    def addAllClassesCb(self,msg):
        self.msg_if.pub_info('Got add all classes msg: ' + str(msg))
        self.addAllClasses()

    def addAllClasses(self):
        self.publish_status(do_updates = False) # Updated Here
        self.selected_classes = self.classes
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('selected_classes', self.classes)
            self.node_if.save_config()


    def removeAllClassesCb(self,msg):
        self.msg_if.pub_info('Got remove all classes msg: ' + str(msg))
        self.selected_classes = []
        self.publish_status(do_updates = False) # Updated Here
        if self.node_if is not None:
            self.node_if.set_param('selected_classes',[])
            self.node_if.save_config()


    def addClassCb(self,msg):
        self.msg_if.pub_info('Got add class msg: ' + str(msg))
        class_name = msg.data
        if class_name in self.classes:
            sel_classes = copy.deepcopy(self.selected_classes)
            if class_name not in sel_classes:
                sel_classes.append(class_name)
            self.selected_classes = sel_classes
            self.publish_status(do_updates = False) # Updated Here
            if self.node_if is not None:
                self.node_if.set_param('selected_classes', sel_classes)
                self.node_if.save_config()


    def removeClassCb(self,msg):
        self.msg_if.pub_info('Got remove class msg: ' + str(msg))
        class_name = msg.data
        sel_classes = copy.deepcopy(self.selected_classes)
        if class_name in sel_classes:
            sel_classes.remove(class_name)
        self.selected_classes = sel_classes
        self.publish_status(do_updates = False) # Updated Here
        if self.node_if is not None:
            self.node_if.set_param('selected_classes', sel_classes)
            self.node_if.save_config()



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
        img_topics = copy.deepcopy(self.selected_img_topics)
        if img_topic not in img_topics:
            img_topics.append(img_topic)
        elif os.path.isdir(img_topic) == True:
            if self.img_folder_path is not None:
                if self.img_folder_path in img_topics:
                    img_topics.remove(self.img_folder_path)
            img_topics.append(img_topic)
            self.img_folder_path=img_topic
        else:
            self.msg_if.pub_warn('Selected image topic not found as image or folder')
        self.selected_img_topics = img_topics
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('selected_img_topics',self.selected_img_topics)


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
        img_topics = copy.deepcopy(self.selected_img_topics)
        if img_topic in img_topics:
            img_topics.remove(img_topic)
        self.selected_img_topics = img_topics
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('selected_img_topics',self.selected_img_topics)
            self.node_if.save_config()

    def selectImageFileCb(self,msg):
        #self.msg_if.pub_info("Received Process Image File: " + str(msg))
        if self.img_folder_path is not None:
            img_file_path = os.path.join(self.img_folder_path,self.msg.data)
            if os.path.exists(img_file_path) == True:
                self.img_file_path=img_file_path
            else:
                self.msg_if.pub_warn('Process image file failed, File not found: ' + str(img_file_path))
                
        else:
            self.msg_if.pub_warn('Process image file failed, Folder not set as Image Topic')


    def setOverlayLabelsCb(self,msg):
        self.overlay_labels = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('overlay_labels',self.overlay_labels)
            self.node_if.save_config()

    def setOverlayTargetingCb(self,msg):
        self.overlay_targeting = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('overlay_targeting',self.overlay_targeting)
            self.node_if.save_config()


    def setOverlayClfNameCb(self,msg):
        self.overlay_clf_name = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('overlay_clf_name',self.overlay_clf_name)
            self.node_if.save_config()



 


    def setOverlayImgNameCb(self,msg):
        self.overlay_img_name = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('overlay_img_name',self.overlay_img_name)
            self.node_if.save_config()

 



    def setMaxProcRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        self.max_proc_rate_hz = max_rate
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('max_proc_rate_hz',self.max_proc_rate_hz)
            self.node_if.save_config()

 


    def setMaxImgRateCb(self,msg):
        max_rate = msg.data
        if max_rate <  MIN_MAX_RATE:
            max_rate = MIN_MAX_RATE
        elif max_rate > MAX_MAX_RATE:
            max_rate = MAX_MAX_RATE
        self.max_img_rate_hz = max_rate
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('max_img_rate_hz',self.max_img_rate_hz)
            self.node_if.save_config()

    def setUseLastImageCb(self,msg):
        self.use_last_image = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('use_last_image',self.use_last_image)
            self.node_if.save_config()



    def setTileImgCb(self,msg):
        self.img_tiling = msg.data
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('img_tiling',self.img_tiling)
            self.node_if.save_config()

 

    def setThresholdCb(self,msg):
        threshold = msg.data
        self.msg_if.pub_info("Received Threshold Update: " + str(threshold))
        if threshold <  MIN_THRESHOLD:
            threshold = MIN_THRESHOLD
        elif threshold > MAX_THRESHOLD:
            threshold = MAX_THRESHOLD
        self.threshold = threshold
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('threshold',self.threshold)
            self.node_if.save_config()



    def setSleepEnableCb(self,msg):
        self.sleep_enabled = msg.data
        self.publish_status()

        if self.node_if is not None:
            self.node_if.set_param('sleep_enabled',self.sleep_enabled)
            self.node_if.save_config()



    def setSleepSuspendTimeCb(self,msg):
        data = msg.data
        if data > 1 or suspend_time == -1:
            self.sleep_suspend_sec = data
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('sleep_suspend_sec',self.sleep_suspend_sec)
                self.node_if.save_config()


    def setSleepSuspendTimeCb(self,msg):
        data = msg.data
        if data > 1:
            self.sleep_run_sec = data
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('sleep_run_sec',self.sleep_run_sec)
                self.node_if.save_config()


    def setPubImageCb(self,msg):
        enable = msg.data
        self.set_pub_image(enable)




    def set_pub_image(self,enable):
        self.pub_image_enabled = enable
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('pub_image_enabled',self.pub_image_enabled)
            self.node_if.save_config()

    #######################################
    # Class Functions

    def statusPublishCb(self,timer):
        self.publish_status()



    def getActiveImgTopics(self):
        active_img_topics = []
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        for img_topic in imgs_info_dict.keys():
            img_active =  imgs_info_dict[img_topic]['active']
            if img_active == True:
                active_img_topics.append(img_topic)
        return active_img_topics


    def updaterCb(self,timer):
        #self.msg_if.pub_warn("Updating with image topic: " +  self.img_topic)
        selected_img_topics = self.selected_img_topics
        active_img_topics = self.getActiveImgTopics()


        #self.msg_if.pub_warn("")
        #self.msg_if.pub_warn("Updating with image topics: " +  str(selected_img_topics))
        #self.msg_if.pub_warn("Updating with active image topics: " +  str(active_img_topics))
        purge_list = []
        # Update Image subscribers
        found_img_topics = []
        for img_topic in selected_img_topics:
            if img_topic == self.img_folder_path:
                if os.path.isdir(img_topic) != True:
                    img_topic = ''
            else:
                img_topic = nepi_sdk.find_topic(img_topic, exact = True)
            if img_topic != '':
                found_img_topics.append(img_topic)
                if img_topic not in active_img_topics:
                    self.msg_if.pub_warn('Will subscribe to image topic: ' + img_topic)
                    success = self.subscribeImgTopic(img_topic)              
        # Update Image Subs purge list
        for img_topic in active_img_topics:
            if img_topic not in found_img_topics:
                purge_list.append(img_topic)
        if len(purge_list) > 0:
            self.msg_if.pub_warn('Purging image topics: ' + str(purge_list))
        for topic in purge_list:
            self.msg_if.pub_warn('Will unsubscribe from image topic: ' + topic)
            success = self.unsubscribeImgTopic(topic)

        # Check Image connected state
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        img_connects = []
        for img_topic in imgs_info_dict.keys():
            img_connects.append(imgs_info_dict[img_topic]['connected'])
        img_selected = len(img_connects) > 0
        img_connected = True in img_connects
       
        # Set Detector State
        enabled = self.enabled
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

        nepi_sdk.start_timer_process((.1), self.updaterCb, oneshot = True)


    def subscribeImgTopic(self,img_topic):
        if img_topic == "None" or img_topic == "":
            return False
        else:

            ####################
            # Create Pubs and Subs IF Dict 

            img_name = img_topic.replace(self.base_namespace,"")
            pub_namespace = nepi_sdk.create_namespace(self.node_namespace,img_name)

            # Pubs Config Dict 
            img_pubs_dict = {
                'bounding_boxes': {
                    'msg': BoundingBoxes,
                    'namespace': pub_namespace,
                    'topic': 'bounding_boxes',
                    'qsize': 1,
                    'latch': False
                }
            }

            pub_namespaces = []
            ns=os.path.join(pub_namespace,'bounding_boxes')
            pub_namespaces.append(ns)


            if self.enable_image_pub == True:
                img_pubs_dict['image_pub'] = {
                    'msg': Image,
                    'namespace': pub_namespace,
                    'topic': 'detection_image',
                    'qsize': 1,
                    'latch': False
                }
                ns=os.path.join(pub_namespace,'detection_image')
                pub_namespaces.append(ns)

            

            if img_topic == self.img_folder_path:
                img_sub_dict = {
                            'namespace': self.node_namespace,
                            'msg': String,
                            'topic': '',
                            'qsize': 1,
                            'callback': self.setImageFileCb,
                            'callback_args': (img_topic)
                        }
                connected=True
            else:
                img_sub_dict = {
                            'namespace': img_topic,
                            'msg': Image,
                            'topic': '',
                            'qsize': 1,
                            'callback': self.imageCb,
                            'callback_args': (img_topic)
                        }
                connected=False

            imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
            # Check if exists
            if img_topic in imgs_info_dict.keys():
                if self.img_ifs_dict[img_topic]:
                    self.msg_if.pub_info('Subsribing to image topic: ' + img_topic)  
                    # Try and Reregister img_sub
                    self.img_ifs_lock.acquire()
                    self.img_ifs_dict[img_topic]['subs_if'].register_sub('img_sub',img_sub_dict)
                    self.img_ifs_dict[img_topic]['pubs_if'].register_pubs(img_pubs_dict)
                    self.img_ifs_lock.release()
                    self.msg_if.pub_warn('Registered : ' + img_topic +  ' ' + str(self.img_ifs_dict[img_topic]))
                    # Set back to active
                    self.imgs_info_dict[img_topic]['active'] = True
            else:
                    
                # Create register new image topic
                self.msg_if.pub_warn('Registering to image topic: ' + img_topic)
                self.msg_if.pub_warn('Publishing on namespace: ' + pub_namespace)

                ####################
                # Create img info dict
                imgs_info_dict = dict()  
                imgs_info_dict['namespace'] = pub_namespace
                imgs_info_dict['pub_namespaces'] = pub_namespaces    
                imgs_info_dict['connected'] = connected 
                imgs_info_dict['active'] = True
                imgs_info_dict['image_latency_time'] = 0
                imgs_info_dict['detect_latency_time'] = 0
                imgs_info_dict['preprocess_time'] = 0 
                imgs_info_dict['detect_time'] = 0  
                self.imgs_info_dict[img_topic] = imgs_info_dict

                self.msg_if.pub_info('Subsribing to image topic: ' + img_topic)


                img_pubs_if = NodePublishersIF(
                                pubs_dict = img_pubs_dict,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
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
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )





                ####################
                # Add Img Subs and Pubs IFs to Img IFs Dict
                self.img_ifs_lock.acquire()
                self.img_ifs_dict[img_topic] = {
                                                'pubs_if': img_pubs_if,
                                                'subs_if': img_subs_if
                                                }   

                self.img_ifs_lock.release()
                self.msg_if.pub_warn('Registered : ' + img_topic)

                time.sleep(1)
                ####################
                # Publish blank msg to prime topics
                img_pubs_if.publish_pub('bounding_boxes',BoundingBoxes())   

                if self.enable_image_pub == True:
                    cv2_img = nepi_img.overlay_text(self.BLANK_CV2_IMAGE, self.IMAGE_SUB_MSG)
                    ros_img = nepi_img.cv2img_to_rosimg(cv2_img)
                    img_pubs_if.publish_pub('image_pub',ros_img) 

                ####################
                # Create Img Sub Dict
                self.imgs_has_subs_dict[img_topic] = False

                ####################
                # Create Img Dict
                img_dict = dict()
                img_dict['topic'] = img_topic
                img_dict['timestamp'] = nepi_utils.get_time()
                img_dict['msg_header'] = "None" 
                img_dict['msg_stamp'] = "None" 

                self.imgs_dict[img_topic]=img_dict
    
            return True

    def publishDetMsg(self,img_topic, pub_name, msg):
        #self.msg_if.pub_warn("Publishing topic: " + str(pub_name))
        self.node_if.publish_pub(pub_name,msg)
        pub_name_all = pub_name + "_all"
        self.node_if.publish_pub(pub_name_all,msg)
        if img_topic in self.img_ifs_dict.keys():
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[img_topic]['pubs_if'].publish_pub(pub_name,msg)
            self.img_ifs_lock.release()
                 

    def unsubscribeImgTopic(self,img_topic):

        # Unsubsribe from Pubs and Subs
        if img_topic in self.img_ifs_dict.keys():
            self.msg_if.pub_warn('Unregistering image topic: ' + img_topic)
            self.img_ifs_lock.acquire()
            self.img_ifs_dict[img_topic]['subs_if'].unregister_sub('img_sub')
            self.img_ifs_dict[img_topic]['pubs_if'].unregister_pubs()
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

        # Clear Img Dict
        nepi_sdk.sleep(1)
        img_dict = dict()
        self.cv2_img = None
        img_dict['topic'] = img_topic
        img_dict['timestamp'] = nepi_utils.get_time()
        img_dict['msg_header'] = "None" 
        self.imgs_dict[img_topic] = img_dict


        # Clear Img Subs Dict
        self.imgs_has_subs_dict[img_topic] = False

        return True


    def imageCb(self,image_msg, args):     
        img_topic = args
        was_connected = self.imgs_info_dict[img_topic]['connected']
        self.imgs_info_dict[img_topic]['connected'] = True
        if img_topic == self.get_img_topic:           
            # Reset image get flags
            self.get_img_topic = "None"
            self.got_img_topic = img_topic

            img_info_dict = self.imgs_info_dict[img_topic]   
            if img_info_dict['active'] == True and img_topic in self.imgs_info_dict.keys() and self.enabled == True and self.sleep_state == False:
                #self.msg_if.pub_warn("Processing img for topic:  " + img_topic)
                start_time = nepi_sdk.get_time() 

                ##############################
                ### Get CV2 Image
                timestamp = start_time
                msg_header = image_msg.header
                msg_stamp = image_msg.header.stamp

                get_msg_time = nepi_sdk.get_msg_stamp()
                get_msg_stampstamp = image_msg.header.stamp
                latency = (get_msg_time.to_sec() - get_msg_stampstamp.to_sec())

                cv2_img = nepi_img.rosimg_to_cv2img(image_msg)
                preprocess_time = round( (nepi_sdk.get_time() - start_time) , 3)
                ##############################


                # Update img_dict
                img_dict = dict()
                img_dict['topic'] = img_topic
                img_dict['timestamp'] = timestamp
                img_dict['msg_header'] = msg_header
                img_dict['msg_stamp'] = msg_stamp
                self.imgs_dict[img_topic] = img_dict                

                # Update Image Pre Process Info
                if img_topic in self.imgs_info_dict.keys():
                    try:
                        self.imgs_info_dict[img_topic]['image_latency_time'] = latency
                        self.imgs_info_dict[img_topic]['preprocess_time'] = preprocess_time
                    except:
                        pass

                #################################
                ### Run Detection
                if cv2_img is not None:



                    ##############################
                    # Process Detections
                    det_start_time = nepi_sdk.get_time()

                    detect_dict_list = []
                    detect_dicts = []
                    threshold = self.threshold
                    try:
                        ##################################
                        #self.msg_if.pub_warn("AIF init img_dict: " + str(img_dict))
                        [detect_dicts, img_dict] = self.processDetection(cv2_img, img_dict, threshold = threshold, resize = False, verbose = False) 
                        #self.msg_if.pub_warn("AIF got img_dict: " + str(img_dict))
                        #self.msg_if.pub_warn("AIF got back detect_dict: " + str(detect_dicts))
                        self.imgs_dict[img_topic] = img_dict  
                        ##################################
                        success = True
                        self.first_detect_complete = True
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to process detection img with exception: " + str(e))


                    ##############################
                    # Publish Detections
                    # Filter selected classes
                    sel_classes = copy.deepcopy(self.selected_classes)
                    sel_detect_ind = []
                    for i, detect in enumerate(detect_dicts):
                        if detect['name'] in sel_classes:
                            sel_detect_ind.append(i)
                    for ind in sel_detect_ind:
                        detect_dict_list.append(detect_dicts[ind])

                    ##################################
                    self.publishDetectionData(img_topic, img_dict, detect_dict_list)
                    ##################################


                    ##############################
                    # Upate Detection Info

                    cur_time = nepi_sdk.get_time()

                    detect_time = round( (cur_time - det_start_time) , 3)
                    self.detect_times.pop(0)
                    self.detect_times.append(detect_time)

                    #self.msg_if.pub_warn("Updated Detect Times: " + str(self.detect_times))

                    detect_delay = round( (cur_time - self.last_detect_time) , 3)
                    self.detect_delays.pop(0)
                    self.detect_delays.append(detect_delay)
                    self.last_detect_time = cur_time

                    #self.msg_if.pub_warn("Updated Detect Delays: " + str(self.detect_delays))
                    
                    cur_msg_time = nepi_sdk.get_msg_stamp()
                    get_msg_stampstamp = img_dict['msg_stamp']
                    latency = round( (cur_msg_time.to_sec() - get_msg_stampstamp.to_sec()) , 3)
                    self.detect_latencies.pop(0)
                    self.detect_latencies.append(latency)

                    
                    #self.msg_if.pub_info("Detect Pub Latency: {:.2f}".format(latency))
                    if img_topic in self.imgs_info_dict.keys():
                        try:
                            self.imgs_info_dict[img_topic]['detect_time'] = detect_time
                            self.imgs_info_dict[img_topic]['detect_latency_time'] = latency
                        except:
                            pass

                    ##############################
                    # Publish Update Image for image pub node
                    if was_connected == False and self.enable_image_pub == True:
                        if img_topic in self.img_ifs_dict.keys():
                            cv2_img = nepi_img.overlay_text(self.BLANK_CV2_IMAGE, self.IMAGE_PUB_MSG)
                            ros_img = nepi_img.cv2img_to_rosimg(cv2_img)
                            self.img_ifs_lock.acquire()
                            self.img_ifs_dict[img_topic]['pubs_if'].publish_pub('image_pub',ros_img) 
                            self.img_ifs_lock.release()







    def setImageFileCb(self,str_msg, args):    
        img_topic = args
        was_connected = self.imgs_info_dict[img_topic]['connected']
        self.imgs_info_dict[img_topic]['connected'] = True
        if img_topic == self.get_img_topic:           
            # Reset image get flags
            self.get_img_topic = "None"
            self.got_img_topic = img_topic

            img_info_dict = self.imgs_info_dict[img_topic]   
            if img_info_dict['active'] == True and img_topic in self.imgs_info_dict.keys() and self.enabled == True and self.sleep_state == False:
                #self.msg_if.pub_warn("Processing img for topic:  " + img_topic)
                start_time = nepi_sdk.get_time() 


                ##############################
                ### Get CV2 Image
                img_file=os.path.join(img_topic,str_msg)
                if os.path.exists(img_file) == False:
                    self.msg_if.pub_warn("Process Image File Failed. Image File Not Found:  " + img_file)
                    return 

                timestamp = start_time
                msg_header = nepi_sdk.get_msg_header()
                msg_stamp = nepi_sdk.get_msg_stamp()
                latency = 0.0 #(start_time.to_sec() - get_msg_stampstamp.to_sec())
                cv2_img = cv2.imread(img_file)
                preprocess_time = round( (nepi_sdk.get_time() - start_time) , 3)
                ##############################


                # Update img_dict
                img_dict['topic'] = os.path.join(img_topic,img_file)
                img_dict['timestamp'] = timestamp
                img_dict['msg_header'] = msg_header
                img_dict['msg_stamp'] = msg_stamp
                self.imgs_dict[img_topic] = img_dict                

                # Update Image Pre Process Info
                if img_topic in self.imgs_info_dict.keys():
                    try:
                        self.imgs_info_dict[img_topic]['image_latency_time'] = latency
                        self.imgs_info_dict[img_topic]['preprocess_time'] = preprocess_time
                    except:
                        pass

                #################################
                ### Run Detection
                if cv2_img is not None:

                    self.got_img_topic = img_topic  






    def updateDetectCb(self,timer):
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        enabled = self.enabled
        if enabled == True:
            img_topics = self.selected_img_topics
            connected_list = []
            for topic in img_topics:
                if topic in imgs_info_dict.keys():
                    if imgs_info_dict[topic]['connected'] == True:
                        connected_list.append(topic)
            if len(connected_list) == 0:
                #self.msg_if.pub_warn("No Connected Image Topics")
                self.next_image_topic = "None"
            else:
                # check timer
                max_rate = self.max_proc_rate_hz
                delay_time = float(1) / max_rate 
                start_time = nepi_sdk.get_time()
                timer = round((start_time - self.last_detect_time), 3)
                #self.msg_if.pub_warn("Delay and Timer: " + str(delay_time) + " " + str(timer))

                # Get image topic info
                cur_img_topic = copy.deepcopy(self.cur_img_topic)

                # Setup Next Img if needed
                num_connected_list = len(connected_list)
                if num_connected_list > 0:
                    if cur_img_topic in connected_list:
                        next_img_ind = connected_list.index(cur_img_topic) + 1
                        if next_img_ind >= num_connected_list:
                            self.next_image_topic = connected_list[0]
                        else:
                            self.next_image_topic = connected_list[next_img_ind]
                    else:
                        self.next_image_topic = connected_list[0]
                else:
                    self.next_image_topic = "None"
                #self.msg_if.pub_warn("Next Image Topic set to: " + self.next_image_topic)

                # Check if image topic has been initialized.
                if cur_img_topic == "None" and self.next_image_topic != "None":
                    self.msg_if.pub_warn("Initializing get topic to: " +  self.next_image_topic)
                    self.got_img_topic = None
                    self.cur_img_topic = copy.deepcopy(self.next_image_topic)
                    self.get_img_topic = copy.deepcopy(self.next_image_topic)
                    #self.last_detect_time = nepi_sdk.get_time()


                ##############################
                # Check for non responding image streams                   
                if self.got_img_topic is None and timer > (delay_time + GET_IMAGE_TIMEOUT_SEC):
                    #self.msg_if.pub_warn("Topic " + cur_img_topic + " timed out. Setting next topic to: " +  self.next_image_topic)
                    if cur_img_topic != self.img_folder_path:
                        if cur_img_topic is not None and cur_img_topic in imgs_info_dict.keys():
                            imgs_info_dict[cur_img_topic]['connected'] = False
                        self.cur_img_topic = self.next_image_topic
                        #self.last_detect_time = nepi_sdk.get_time()

                elif self.got_img_topic is not None and timer > delay_time: 
                        # Set Next Image Topic on Delay
                        self.cur_img_topic = copy.deepcopy(self.next_image_topic)
                        self.get_img_topic = copy.deepcopy(self.next_image_topic)
                        self.got_img_topic = None

                        #self.msg_if.pub_warn("Reset Current and Get Image Topic: " +  self.cur_img_topic)
                        #self.msg_if.pub_warn("With Delay and Timer: " + str(delay_time) + " " + str(timer))

                        
                       
        nepi_sdk.start_timer_process((0.01), self.updateDetectCb, oneshot = True)

    def publishDetectionData(self, img_topic, img_dict, detect_dict_list):
        #self.msg_if.pub_warn("Publisher got img_dict: " + str(img_dict))
        det_count = len(detect_dict_list)
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        img_topic = img_topic
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
            bbs_msg.header.stamp = nepi_sdk.get_msg_stamp()
            bbs_msg.model_name = self.node_name
            ros_img_header=img_dict['msg_header']
            if ros_img_header is None:
                ros_img_header = bbs_msg.header
            bbs_msg.image_header = ros_img_header
            bbs_msg.image_topic = img_topic
            bbs_msg.image_width = img_dict['image_width']
            bbs_msg.image_height = img_dict['image_height']
            bbs_msg.prc_width = img_dict['prc_width']
            bbs_msg.prc_height = img_dict['prc_height']
            bbs_msg.bounding_boxes = bb_msg_list

            self.publishDetMsg(img_topic,'bounding_boxes',bbs_msg)
    
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
                get_msg_stampstamp = bbs_msg.header.stamp
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

        #self.msg_if.pub_warn("Returning Detector Info Response: " + str(resp))
        return resp
    

    def updateStatesCb(self,timer):
        # Update and clear detection state every second
        self.states_dict['detection']['value'] = str(self.detection_state)
        self.detection_state = False
        
    def updateImgSubsCb(self,timer):
        # Check for data subscribers every second
        has_subs_list = []
        # Find active img topics
        imgs_info_dict = copy.deepcopy(self.imgs_info_dict)
        img_topics = imgs_info_dict.keys()
        active_topics = []
        for img_topic in img_topics:
            if imgs_info_dict[img_topic]['active'] == True:
                active_topics.append(img_topic)

        
        # Check if for all topic subscribers
        filters = ['detection_image']
        topic_names = []
        namespace = os.path.join(self.base_namespace, 'bounding_boxes')
        topic_names.append(namespace)
        namespace = os.path.join(self.base_namespace, 'detection_image')
        topic_names.append(namespace)
        namespace = os.path.join(self.base_namespace, 'bounding_boxes')
        topic_names.append(namespace)
        namespace = os.path.join(self.base_namespace, 'detection_image')
        topic_names.append(namespace)

        [has_subs,has_subs_dict] = nepi_sdk.find_subscribers(topic_names,filters, log_name_list = self.log_name_list)

        # Check if save_data_if needs data
        ds_dict = self.save_data_if.data_products_should_save_dict()
        for ds in ds_dict.keys():
            if ds == True:
                has_subs = has_subs or ds

        if has_subs == True:
            self.img_ifs_lock.acquire()
            for img_topic in img_topics:
                if img_topic not in active_topics:
                    self.imgs_has_subs_dict[img_topic] = False
                else:
                    self.imgs_has_subs_dict[img_topic] = True
            self.img_ifs_lock.release()
        else:           
            # Check if for image topic subscribers
            for img_topic in img_topics:
                if img_topic not in active_topics:
                    self.imgs_has_subs_dict[img_topic] = False
                else:
                    filters = ['detection_image']
                    topic_names = []      
                    if img_topic in self.imgs_info_dict.keys():
                        topic_names = self.imgs_info_dict[img_topic]['pub_namespaces']
                        has_subs = nepi_sdk.find_subscribers(topic_names,filters, log_name_list = self.log_name_list)

                    self.imgs_has_subs_dict[img_topic] = has_subs
               

        nepi_sdk.start_timer_process((0.1), self.updateImgSubsCb, oneshot = True)


    def publishStatusCb(self,timer):
        self.publish_status()

    def publish_status(self, do_updates = True):

        # Pub Status
        self.status_msg.name = self.model_name
        self.status_msg.has_sleep = self.has_sleep

        self.status_msg.sleep_enabled = self.sleep_enabled
        self.status_msg.sleep_suspend_sec = int(self.sleep_suspend_sec)
        self.status_msg.sleep_run_sec = int(self.sleep_run_sec)
        self.status_msg.sleep_state = self.sleep_state

        self.status_msg.img_tiling = self.img_tiling



        #self.msg_if.pub_warn("Sending Status Msg: " + str(self.det_status_msg), throttle_s = 5.0)
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub',self.status_msg)

        # Pub Detection Status

        self.det_status_msg.name = self.model_name
        self.det_status_msg.namespace = self.node_namespace
        self.det_status_msg.enabled = self.enabled
        self.det_status_msg.state = self.state
        detection_state = False
        if self.states_dict is not None:
            detection_state = copy.deepcopy(self.states_dict['detection']['value']) == 'True'
        self.det_status_msg.detection_state = detection_state

        self.det_status_msg.available_classes = self.classes
        sel_classes = copy.deepcopy(self.selected_classes)
        self.det_status_msg.selected_classes = sel_classes


        self.det_status_msg.pub_image_enabled = self.pub_image_enabled
        self.det_status_msg.overlay_labels = self.overlay_labels
        self.det_status_msg.overlay_targeting = self.overlay_targeting
        self.det_status_msg.overlay_clf_name = self.overlay_clf_name
        self.det_status_msg.overlay_img_name = self.overlay_img_name

        self.det_status_msg.threshold_filter = self.threshold
        self.det_status_msg.max_proc_rate_hz = self.max_proc_rate_hz
        self.det_status_msg.max_img_rate_hz = self.max_img_rate_hz
        self.det_status_msg.use_last_image = self.use_last_image

        self.det_status_msg.selected_img_topics = self.selected_img_topics


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




        avg_time = sum(self.detect_delays) / len(self.detect_delays)
        self.det_status_msg.avg_latency_sec = avg_time

        avg_rate = 0
        avg_times = sum(self.detect_latencies) / len(self.detect_latencies)
        if avg_time > .005:
            avg_rate = float(1) / avg_time
        self.det_status_msg.avg_rate_hz = avg_rate


        max_rate = 0
        max_time = sum(self.detect_times) / len(self.detect_times)
        if max_time > .005:
            max_rate = float(1) / max_time
        self.det_status_msg.max_rate_hz = max_rate       

        #self.msg_if.pub_warn("Sending Detection Status Msg: " + str(self.det_status_msg))
        if self.node_if is not None:
            self.node_if.publish_pub('det_status_pub',self.det_status_msg)



