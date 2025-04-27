#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import sys
import os
import os.path
import copy
import errno
import glob
import subprocess
import yaml
import time
import numpy as np
import cv2

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils



from std_msgs.msg import Empty, Float32, String, Bool, Int32
from nepi_ros_interfaces.msg import SystemStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nepi_ros_interfaces.msg import UpdateState, AiModelMgrStatus
from nepi_ros_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_ros_interfaces.msg import BoundingBoxes, ObjectCount

from nepi_ros_interfaces.srv import SystemStorageFolderQuery
from nepi_ros_interfaces.srv import AiMgrActiveModelsInfoQuery, AiMgrActiveModelsInfoQueryRequest, AiMgrActiveModelsInfoQueryResponse
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryResponse, AiDetectorInfoQueryRequest


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.connect_mgr_if_system import ConnectMgrSystemIF
from nepi_api.connect_mgr_if_config import ConnectMgrConfigIF


DEFAULT_FRAMEWORK = 'yolov5'


AIFS_PARAM_FOLDER = '/opt/nepi/ros/share/nepi_api'
AIFS_API_FOLDER = '/opt/nepi/ros/lib/python3/dist-packages/nepi_api'
AIFS_INSTALL_FOLDER = '/mnt/nepi_storage/install/ai_frameworks'
AI_MODELS_FOLDER = '/mnt/nepi_storage/ai_models/'

class AIDetectorManager:   

    MODEL_TYPE_LIST = ['detection']
    MODEL_INFO_INTERVAL = 5

    data_products = ['bounding_boxes','detection_image']

    aif_classes_dict = dict()

    save_cfg_if = None

    running_models_list = []
    model_namespace_dict = dict()

    detector_info_dict = dict()
    
    all_namespace = "None"
    #######################
    ### Node Initialization


    DEFAULT_NODE_NAME = "ai_model_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### MGR NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        
        ##############################
        # Initialize Params
        self.initCb(do_updates = False)


        ##############################
        ## Wait for NEPI core managers to start
        # Wait for System Manager
        mgr_sys_if = ConnectMgrSystemIF()
        success = mgr_sys_if.wait_for_status()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Status Msg")

        self.aifs_param_folder = mgr_sys_if.get_sys_folder_path("aifs",AIFS_PARAM_FOLDER)
        self.msg_if.pub_info("Using AI Frameworks Params Folder: " + str(self.aifs_param_folder))

        self.aifs_api_folder = mgr_sys_if.get_sys_folder_path("api",AIFS_API_FOLDER)
        self.msg_if.pub_info("Using AI Frameworks API Folder: " + str(self.aifs_api_folder))

        self.aifs_install_folder = mgr_sys_if.get_sys_folder_path('install/ai_frameworks',AIFS_INSTALL_FOLDER )
        self.msg_if.pub_info("Using AI Frameworks Install Folder: " + str(self.aifs_install_folder))
        
        self.ai_models_folder = mgr_sys_if.get_sys_folder_path("ai_models",AI_MODELS_FOLDER)
        self.msg_if.pub_info("Using AI Models Folder: " + str(self.ai_models_folder))
        
        # Wait for Config Manager
        mgr_cfg_if = ConnectMgrConfigIF()
        success = mgr_cfg_if.wait_for_status()
        if success == False:
            nepi_ros.signal_shutdown(self.node_name + ": Failed to get Config Status Msg")
        
        ###########################


        message = "NO MODELS ENABLED"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_no_models_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        message = "LOADING MODELS"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_loading_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        message = "WAITING FOR MODELS TO PUBLISH"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_waiting_img = nepi_img.cv2img_to_rosimg(cv2_img) 


    ##############################
    ### Setup Node

    # Configs Config Dict ####################
    self.CFGS_DICT = {
        'init_callback': self.initCb,
        'reset_callback': self.resetCb,
        'factory_reset_callback': self.factoryResetCb,
        'init_configs': True,
        'namespace': self.node_namespace
    }


    # Params Config Dict ####################
    self.PARAMS_DICT = {
        'aifs_dict': {
            'namespace': self.node_namespace,
            'factory_val': dict()
        },   
        'active_aifs': {
            'namespace': self.node_namespace,
            'factory_val': []
        },   
        'models_dict': {
            'namespace': self.node_namespace,
            'factory_val': models_dict
        }

    }


    # Services Config Dict ####################
    self.SRVS_DICT = {
        'active_models_info_query': {
            'namespace': self.node_namespace,
            'topic': 'active_models_info_query',
            'svr': AiMgrActiveModelsInfoQuery,
            'req': AiMgrActiveModelsInfoQueryRequest(),
            'resp': AiMgrActiveModelsInfoQueryResponse(),
            'callback': self.handleInfoRequest
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': '/all_detectors/detection_image', #self.all_namespace + '/all_detectors/detection_image
            'msg': Image,
            'qsize': 1,
            'latch': True
        },
    }  


    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'refresh_frameworks': {
            'namespace': self.node_namespace,
            'topic': 'refresh_frameworks',
            'msg': Empty,
            'qsize': 10,
            'callback': self.refreshFrameworksCb, 
            'callback_args': ()
        },
        'update_framework_state': {
            'namespace': self.node_namespace,
            'topic': 'update_framework_state',
            'msg': UpdateState,
            'qsize': 10,
            'callback': self.updateFrameworkStateCb, 
            'callback_args': ()
        },
        'disable_all_frameworks': {
            'namespace': self.node_namespace,
            'topic': 'disable_all_frameworks',
            'msg': Empty,
            'qsize': 10,
            'callback': self.disableAllFwsCb, 
            'callback_args': ()
        },
        'enable_all_models': {
            'namespace': self.node_namespace,
            'topic': 'enable_all_models',
            'msg': UpdateState,
            'qsize': 10,
            'callback': self.enableAllModelsCb, 
            'callback_args': ()
        },
        'disable_all_models': {
            'namespace': self.node_namespace,
            'topic': 'disable_all_models',
            'msg': UpdateState,
            'qsize': 10,
            'callback': self.disableAllModelsCb, 
            'callback_args': ()
        },
        'update_model_state': {
            'namespace': self.node_namespace,
            'topic': 'update_model_state',
            'msg': UpdateState,
            'qsize': 10,
            'callback': self.updateModelStateCb, 
            'callback_args': ()
        },
        'detection_img': {
            'namespace': self.node_namespace,
            'topic': '/all_detectors/detection_image', #self.all_namespace + "/all_detectors/detection_img"
            'msg': Image,
            'qsize': 10,
            'callback': self.detectionImageCb, 
            'callback_args': ()
        },
        'bounding_boxes': {
            'namespace': self.node_namespace,
            'topic': '/all_detectors/bounding_boxes', #self.all_namespace + "/all_detectors/bounding_boxes"
            'msg': UpdateState,
            'qsize': 10,
            'callback': self.updateModelStateCb, 
            'callback_args': ()
        }

    }



    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT,
                    log_class_name = True
    )

    ready = self.node_if.wait_for_ready()




    ###########################
    # Set up save data and save config services ########################################################
    factory_data_rates= {}
    for d in self.data_products:
        factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
    if 'detection_image' in self.data_products:
        factory_data_rates['detection_image'] = [1.0, 0.0, 100.0] 
    self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)



    ###########################
    self.refreshFrameworks()
    # Update and save settings
    self.saveSettings() # Save config

    self.initCb(do_updates = True)


    ###########################


    self.all_namespace = os.path.join(self.base_namespace,'ai')
    self.msg_if.pub_info("Staring all detectors on namespace " + self.all_namespace)
    self.ros_loading_img.header.stamp = nepi_ros.ros_time_now()
    self.node_if.publish_pub('detection_image_pub', self.ros_loading_img)


    self.msg_if.pub_info("Staring AI Framework and Model update process")
    nepi_ros.timer(nepi_ros.ros_duration(1), self.updaterCb, oneshot = True)
    self.msg_if.pub_info("Staring AI Model Info update process")
    nepi_ros.timer(nepi_ros.ros_duration(1), self.modelsInfoUpdaterCb, oneshot = True)

    self.ros_waiting_img.header.stamp = nepi_ros.ros_time_now()
    self.node_if.publish_pub('detection_image_pub', self.ros_waiting_img)
    #########################################################
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    # Spin forever (until object is detected)
    nepi_ros.spin()
    #########################################################




    ####################
    # Wait for System and Config Statuses Callbacks
    def systemStatusCb(self,msg):
        self.sys_status = True

    def configStatusCb(self,msg):
        self.cfg_status = True
        


    #######################
    ### Mgr Config Functions

    def refreshFrameworksCb(self,msg):
        self.refreshFrameworks()

    def refreshFrameworks(self):
        ## Find AI Frameworks
        # Get ai framework dict form param server and update
        #self.msg_if.pub_warn("Got latest ais dict " + str(get_aifs_dict))

        # Update AI Frameworks Dict
        aifs_dict = self.node_if.get_param('aifs_dict')
        last_aifs_keys = aifs_dict.keys()
        #self.msg_if.pub_warn("Got init aifs " + str(last_aifs_keys.keys()))
        aifs_dict = nepi_aifs.refreshAIFsDict(self.aifs_param_folder,self.aifs_api_folder,aifs_dict)
        self.node_if.set_param('aifs_dict', aifs_dict)
        cur_aifs_keys = aifs_dict.keys()
        #self.msg_if.pub_warn("Got updated aifs " + str(cur_aifs_keys.keys()))
        if last_aifs_keys != cur_aifs_keys:
             self.msg_if.pub_warn("Got updated ai framework list: " + str(cur_aifs_keys))

        # Update Active AI Frameworks List
        active_aifs = self.node_if.get_param('active_aifs')
        last_active_aifs = copy.deepcopy(active_aifs)
        #self.msg_if.pub_warn("Got last active aifs " + str(last_active_aifs))
        active_aifs = self.node_if.get_param('active_aifs')
        ### RESTRICT TO SINGLE FRAMEWORK FOR NOW #########
        if len(active_aifs) > 1:
            active_aifs = [active_aifs[0]]
        else:
            active_aifs = active_aifs
        ###########################
        self.node_if.set_param('active_aifs',active_aifs)
        cur_active_aifs = copy.deepcopy(active_aifs)
        #self.msg_if.pub_warn("Got updated active aifs " + str(cur_active_aifs))
        if last_active_aifs != cur_active_aifs:
            self.msg_if.pub_warn("Got updated active ai framework list: " + str(active_aifs))


        models_dict = dict()
        for aif_name in aifs_dict.keys():
            aif_dict = aifs_dict[aif_name]
            self.msg_if.pub_info("Processing ais dict for ai name " + aif_name + " " + str(aif_dict))

            self.msg_if.pub_info("Updating ai dict for framework: " + str(aif_name))
            file_name = aif_dict['if_file_name']
            file_path = aif_dict['api_path']
            module_name = aif_dict['if_module_name']
            class_name = aif_dict['if_class_name']
            sys.path.append(file_path)
            [success, msg, aif_class] = nepi_aifs.importAIFClass(file_name,file_path,module_name,class_name)
            if success == False:
                self.msg_if.pub_warn("Failed to import ai framework if file " + file_name)
                continue
            else:
                success = False
                try:
                    self.msg_if.pub_info("Instantiating IF class for framework type: " + str(aif_name))
                    launch_namespace = os.path.join(self.base_namespace, "ai")
                    all_namespace = os.path.join(self.base_namespace, self.node_name)
                    aif_if_class_instance = aif_class(aif_dict,launch_namespace,all_namespace,self.ai_models_folder)
                    success = True
                    time.sleep(1) # Give some time for publishers to set in class init
                except Exception as e:
                    self.msg_if.pub_warn("Failed to instantiate ai framework class " + class_name + " " + str(e))
                aif_models_dict = dict()
                if success:
                    try:
                        aif_models_dict = aif_if_class_instance.getModelsDict()
                        for model_name in aif_models_dict.keys():
                            model_dict = aif_models_dict[model_name]
                            model_dict['active'] = False
                            models_dict[model_name] = model_dict
                            self.aif_classes_dict[model_name] = aif_if_class_instance
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to get models from class " + class_name + " " + str(e))
                        continue
                    
                    if (len(models_dict.keys()) < 1):
                        self.msg_if.pub_warn("No models found for this ai framework: " + aif_name)
                    else:
                        self.msg_if.pub_info("Got models for framework type: " + str(aif_name) + " from param server " + str(models_dict.keys()))

        param_dict = self.node_if.get_param('models_dict')
        # refresh active states from stored values
        for model_name in models_dict.keys():
            if model_name in param_dict.keys():
                active = param_dict[model_name]['active']
                models_dict[model_name]['active'] =  active

        
        self.node_if.set_param('models_dict', models_dict)
        

    def printModelsDict(self,models_dict):
        self.msg_if.pub_warn("" )
        for model_name in models_dict.keys():
            model_dict = models_dict[model_name]
            self.printModelDict(model_dict)
        self.msg_if.pub_warn("" )


    def printModelDict(self,model_dict):
        #self.msg_if.pub_warn("" )
        for key in model_dict.keys():    
            value = model_dict[key]
            self.msg_if.pub_warn("Model key: " + str(key) + " value: " + str(value))
        #self.msg_if.pub_warn("" )
 
    def initCb(self, do_updates = False):
        if do_updates == True:
            self.resetCb()

    def resetCb(self):
        self.publish_status()

    def factoryResetCb(self):
        pass

    def setCurrentSettingsAsDefault(self):
        self.msg_if.pub_info("Setting current values as default params")


    def updaterCb(self,timer):
        active_aifs = self.node_if.get_param('active_aifs')
        models_dict = self.node_if.reset_param("models_dict")
        active_models_list = nepi_aifs.getModelsActiveSortedList(models_dict)

        for model_name in self.running_models_list:
            model_aif = models_dict[model_name]['framework']
            if model_name not in active_models_list or model_aif not in active_aifs:
                self.msg_if.pub_warn("Killing model: " + model_name)
                try:
                    del self.detector_info_dict[model_name]
                except:
                    pass
                models_dict[model_name]['active'] = False
                self.running_models_list.remove(model_name)
                self.killModel(model_name)
                nepi_ros.sleep(1)

        active_models_list = nepi_aifs.getModelsActiveSortedList(models_dict)
        for model_name in active_models_list:
            model_aif = models_dict[model_name]['framework']
            if model_name not in self.running_models_list and model_aif in active_aifs:
                self.msg_if.pub_warn("Loading model: " + model_name)
                model_dict = models_dict[model_name]
                self.msg_if.pub_warn("Loading model with model dict for model: " + model_name)
                self.printModelDict(model_dict)
                success = self.loadModel(model_name, model_dict)
                if success == True:
                    self.msg_if.pub_warn("Model loaded successfully, adding to running models list: " + model_name)
                    self.running_models_list.append(model_name)
                    nepi_ros.sleep(1)
                    model_type = models_dict[model_name]['type']
                    if model_type == "detection":
                        self.detector_info_dict[model_name] = None # Gets updated in modelsInfoUpdateCb
                else:
                    self.msg_if.pub_warn("Model failed to load: " + model_name)
                    self.msg_if.pub_warn("Setting model to disabled: " + model_name)
                    models_dict[model_name]['active'] = False
                    nepi_ros.set_param("~models_dict",models_dict)
        # Get Updated Models Dict
        models_dict = self.node_if.reset_param("models_dict")
        nepi_ros.set_param("~models_dict",models_dict)
        if len(active_models_list) == 0:
            self.ros_no_models_img.header.stamp = nepi_ros.ros_time_now()
            self.node_if.publish_pub('detection_image_pub', self.ros_no_models_img)
        self.publish_status()
        nepi_ros.timer(nepi_ros.ros_duration(1), self.updaterCb, oneshot = True)

    def modelsInfoUpdaterCb(self,timer):
        # Update detector info
        for model_name in self.detector_info_dict.keys():
            if self.detector_info_dict[model_name] is None:
                # Check for service
                service_namespace = os.path.join(self.base_namespace,'detector_info_query')
                service_exists = nepi_ros.check_for_service(service_namespace)
                if service_exists == True:
                    try:
                        self.msg_if.pub_info("Getting model info service " + service_namespace)
                        info_service = nepi_ros.create_service(service_namespace, AiDetectorInfoQuery)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to obtain model info service: " + str(e))
                    try:
                        self.msg_if.pub_info("Requesting model info for service" + service_namespace)
                        request = AiDetectorInfoQueryRequest()
                        response = folder_query_service(request)
                        self.msg_if.pub_info("Got model info response: " + str(response))
                        self.detector_info_dict[model_name] = response
                    except Exception as e:
                        self.msg_if.pub_info("Failed to obtain model info, will try again in " + str(self.MODEL_INFO_INTERVAL) + " secs")
        nepi_ros.timer(nepi_ros.ros_duration(self.MODEL_INFO_INTERVAL), self.modelsInfoUpdaterCb, oneshot = True)


    def loadModel(self, model_name, model_dict):       
        success = False
        if model_name != "None": 
            try:
                aif_class = self.aif_classes_dict[model_name]
                self.msg_if.pub_warn("Creating model node " + model_name)
                [success,node_namespace] = aif_class.loadModel(model_dict)
                self.msg_if.pub_warn("Model Loaded with namespace: " + node_namespace)
            except Exception as e:
                self.msg_if.pub_warn("Failed to load model: " + model_name + " Removing from models dict list :" + str(e))

            if success == False:
                self.msg_if.pub_warn("Failed to load model:" + model_name)
            else:

                # Just Assume Running for now
                self.msg_if.pub_warn("Node Found: " + model_name)
                self.model_namespace_dict[model_name] = node_namespace
                nepi_ros.sleep(1)

                ''' Future check
                # Try and Wait for model status message
                status_topic = os.path.join(node_namespace,'status')
                model_size = model_dict['size']
                load_time = model_dict['load_time']

                self.msg_if.pub_warn("Model " + model_name + " has model_size: " + str(model_size) + " and estimated load time of: " + str(load_time))
                timeout = round(load_time * 2, 2)
                if timeout < 60:
                    timeout = 60
                self.msg_if.pub_warn("Waiting for model " + model_name + " to publish status on topic: " + status_topic)
                self.msg_if.pub_warn("Model " + model_name + " status wait timeout set to " + str(timeout))
                got_topic = nepi_ros.wait_for_topic(status_topic,timeout = timeout)
                if got_topic == "":
                    self.msg_if.pub_warn("Model status timed out for: " + model_name)
                else:
                    self.msg_if.pub_warn("Got model status: " + model_name)

                # Check node is active
                self.msg_if.pub_warn("Checking that node is active for model " + model_name )
                node_running = nepi_ros.check_for_node(model_name)
                if node_running == True:
                    self.msg_if.pub_warn("Node Found: " + model_name)
                    self.model_namespace_dict[model_name] = node_namespace
                else: 
                    self.msg_if.pub_warn("Node Not Found" + model_name)
                    success = False
                '''




            self.saveSettings() # Save config
            self.publish_status()
            return success
        
    def killModel(self, model_name):
        if model_name != "None": 
            models_dict = self.node_if.reset_param("models_dict")
            if not (model_name in models_dict.keys()):
                self.msg_if.pub_warn("Unknown model model requested: " + model_name)
                return
            # Start the model
            aif_class = self.aif_classes_dict[model_name]
            self.msg_if.pub_info("Killing model " + model_name)
            del self.model_namespace_dict[model_name]
            aif_class.killModel(model_name)
  
    def saveSettings(self):
        # Save framework and model dictionaries
        self.save_cfg_if.save() # Save config after initialization for drvt time


    def updateFrameworkStateCb(self,msg):
        self.msg_if.pub_warn("Recieved Framework State Update: " + str(msg))
        framework_name = msg.name
        new_active_state = msg.active_state
        current_aifs = self.node_if.get_param('active_aifs')
        aifs_dict = self.node_if.get_param('aifs_dict')
        if new_active_state == True:
            if framework_name not in current_aifs:
                self.msg_if.pub_warn("Setting AI Framework: " + str(framework_name) + " Active")
                if framework_name in aifs_dict.keys():
                    #current_aifs.append(framework_name)
                    current_aifs = [framework_name] # Just one framework for now
                    self.node_if.set_param('active_aifs', current_aifs)
                    self.refreshFrameworks()
                    self.saveSettings() # Save config
        else:
            if framework_name in current_aifs:
                self.msg_if.pub_warn("Setting AI Framework: " + str(framework_name) + " Inctive")
                current_aifs.remove(framework_name)
                self.node_if.set_param('active_aifs', current_aifs)
                self.refreshFrameworks()
                self.saveSettings() # Save config

        self.publish_status()
        


    def disableAllFwsCb(self,msg):
        self.node_if.set_param('active_aifs', [])
        self.refreshFrameworks()
        self.saveSettings() # Save config
        self.publish_status()

    '''
    def enableAllFwsCb(self,msg):
        aifs_dict = self.node_if.reset_param("aifs_dict")
        aifs_dict = nepi_aifs.activateAllFws(aifs_dict)
        nepi_ros.set_param("~aifs_dict",aifs_dict)
        self.refreshFrameworks()
        self.saveSettings() # Save config
        self.publish_status()


    def disableAllFwsCb(self,msg):
        aifs_dict = self.node_if.reset_param("aifs_dict")
        aifs_dict = nepi_aifs.disableAllFws(aifs_dict)
        nepi_ros.set_param("~aifs_dict",aifs_dict)
        self.refreshFrameworks()
        self.saveSettings() # Save config
        self.publish_status()

    def updateFwStateCb(self,msg):
        aif_name = msg.name
        new_active_state = msg.active_state
        aifs_dict = self.node_if.reset_param("aifs_dict")
        if aif_name in aifs_dict.keys():
            app = aifs_dict[aif_name]
            active_state = app['active']
            if new_active_state != active_state:
                if new_active_state == True:
                    aifs_dict = nepi_aifs.activateFw(aif_name,aifs_dict)
                else:
                    aifs_dict = nepi_aifs.disableFw(aif_name,aifs_dict)
        nepi_ros.set_param("~aifs_dict",aifs_dict)
        self.refreshFrameworks()
        self.saveSettings() # Save config
        self.publish_status()
    '''

    def enableAllModelsCb(self,msg):
        self.msg_if.pub_warn("Recieved Enable All Models msg: " + str(msg))
        aif_name = msg.name
        new_active_state = msg.active_state
        active_aifs = self.node_if.get_param('active_aifs')
        if aif_name not in active_aifs:
            self.msg_if.pub_warn("Ignoring request. AI Framework: " + str(aif_name) + " not enabled")
        else:
            models_dict = self.node_if.reset_param("models_dict")
            for model_name in models_dict.keys():
                model_dict = models_dict[model_name]
                if model_dict['framework'] == aif_name:
                    models_dict[model_name]['active'] = True
        nepi_ros.set_param("~models_dict",models_dict)
        self.saveSettings() # Save config
        self.publish_status()

    def disableAllModelsCb(self,msg):
        self.msg_if.pub_warn("Recieved Disable All Models msg: " + str(msg))
        aif_name = msg.name
        new_active_state = msg.active_state
        active_aifs = self.node_if.get_param('active_aifs')
        if aif_name not in active_aifs:
            self.msg_if.pub_warn("Ignoring request. AI Framework: " + str(aif_name) + " not enabled")
        else:
            models_dict = self.node_if.reset_param("models_dict")
            for model_name in models_dict.keys():
                model_dict = models_dict[model_name]
                if model_dict['framework'] == aif_name:
                    models_dict[model_name]['active'] = False
        nepi_ros.set_param("~models_dict",models_dict)
        self.saveSettings() # Save config
        self.publish_status()

    def updateModelStateCb(self,msg):
        self.msg_if.pub_warn("Recieved Model State Update: " + str(msg))
        model_name = msg.name
        new_active_state = msg.active_state
        models_dict = self.node_if.reset_param("models_dict")
        if model_name in models_dict.keys():
            model_dict = models_dict[model_name]
            model_aif = model_dict['framework']
            active_aifs = self.node_if.get_param('active_aifs')
            if model_aif not in active_aifs:
                self.msg_if.pub_warn("Ignoring request. Model's AI Framework: " + str(model_aif) + " not enabled")
            else:
                active_state = model_dict['active']
                if new_active_state != active_state:
                    if new_active_state == True:
                        self.msg_if.pub_warn("Changing Model State to: True")
                        models_dict[model_name]['active'] = True
                    else:
                        self.msg_if.pub_warn("Changing Model State to: False")
                        models_dict[model_name]['active'] = False

        nepi_ros.set_param("~models_dict",models_dict)
        self.saveSettings() # Save config
        self.publish_status()

    def detectionImageCb(self,img_msg):
        data_product = 'detection_image'
        ros_timestamp = img_msg.header.stamp
        nepi_save.save_ros_img2file(self,data_product,img_msg,ros_timestamp)

    def boundingBoxesCb(self,bbs_msg):
        data_product = 'bounding_boxes'
        ros_timestamp = bbs_msg.header.stamp
        bbs_dict = nepi_ros.convert_msg2dict(bbs_msg)
        nepi_save.save_dict2file(self,data_product,bbs_dict,ros_timestamp)

    def handleInfoRequest(self,_):
        resp = AiMgrActiveModelsInfoQueryResponse()
        model_name_list = []
        model_info_list = []
        for model_name in self.detector_info_dict.key():
            if self.detector_info_dict[model_name] is not None:
                model_name_list.append(model_name)
                model_info_list.append(self.detector_info_dict[model_name])
        resp.detector_name_list = model_name_list
        resp.detector_info_list = model_info_list
        return resp


    def publish_status(self):
        aifs_dict = self.node_if.reset_param("aifs_dict",)
        models_dict = self.node_if.reset_param("models_dict")

        status_msg = AiModelMgrStatus()
        status_msg.ai_frameworks = nepi_aifs.getAIFsSortedList(aifs_dict)


        ai_models = nepi_aifs.getModelsSortedList(models_dict)
        status_msg.ai_models = ai_models

        aif_list = []
        type_list = []
        info_list = []
        state_list = []
        for model_name in ai_models:
            type_list.append(models_dict[model_name]['type'])
            aif_list.append(models_dict[model_name]['framework'])
            state_list.append(models_dict[model_name]['active'])
        status_msg.ai_models_frameworks = aif_list
        status_msg.ai_models_types = type_list
        status_msg.ai_models_states = state_list


        active_aifs = self.node_if.get_param('active_aifs')
        model_folders = []
        for aif_name in active_aifs:
            model_folders.append(aifs_dict[aif_name]['models_folder_name'])
        status_msg.active_ai_frameworks = active_aifs
        status_msg.active_ai_frameworks_folders = model_folders

        active_models = nepi_aifs.getModelsActiveSortedList(models_dict)
        active_list = []
        type_list = []
        node_list = []
        namespace_list = []

        for model_name in active_models:
            if model_name in self.model_namespace_dict.keys():
                active_list.append(model_name)
                type_list.append(models_dict[model_name]['type'])
                node_list.append(model_name)
                namespace_list.append(self.model_namespace_dict[model_name])

        status_msg.active_ai_models = active_list
        status_msg.active_ai_models_types = type_list
        status_msg.active_ai_models_nodes = node_list
        status_msg.active_ai_models_namespaces = namespace_list


        status_msg.all_namespace = self.all_namespace

        #self.msg_if.pub_warn("Sending Model Mgr Status Msg: " + str(status_msg))
        if not nepi_ros.is_shutdown():
            self.node_if.publish_pub('status_pub', status_msg)

    def publishDetectorStatus(self):
        status_msg = AiDetectorStatus()
        status_msg.model_name = "None"
        status_msg.state = "Stopped"
        #self.msg_if.pub_warn("Sending Model Status Msg: " + str(status_msg))

        if not nepi_ros.is_shutdown():
            self.node_if.publish_pub('status_model_pub', status_msg)

if __name__ == '__main__':
    AIDetectorManager()
