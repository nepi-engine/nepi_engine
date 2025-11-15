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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_img


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import MgrSystemStatus

from nepi_interfaces.msg import UpdateState, MgrAiModelsStatus
from nepi_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_interfaces.msg import BoundingBoxes, ObjectCount

from nepi_interfaces.srv import SystemStorageFolderQuery
from nepi_interfaces.srv import AiModelsInfoQuery, AiModelsInfoQueryRequest, AiModelsInfoQueryResponse
from nepi_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryResponse, AiDetectorInfoQueryRequest


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


DEFAULT_FRAMEWORK = 'yolov5'


class AIDetectorManager:   

    MODEL_TYPE_LIST = ['detection']
    MODEL_INFO_INTERVAL = 10

    aif_classes_dict = dict()

    node_if = None

    aifs_classes_dict = dict()
    aifs_dict = dict() 
    models_dict = dict()

    running_models_list = []
    model_namespace_dict = dict()

    detector_info_dict = dict()
    
    all_namespace = 'ai'
    #######################
    ### Node Initialization


    DEFAULT_NODE_NAME = "ai_model_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### MGR NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        nepi_sdk.sleep(1)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        
        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got system folders: " + str(system_folders))

        self.aifs_param_folder = system_folders['aifs_param']
        self.msg_if.pub_info("Using AI Frameworks Params Folder: " + str(self.aifs_param_folder))

        self.aifs_api_folder = system_folders['api_pkg']
        self.msg_if.pub_info("Using AI Frameworks API Folder: " + str(self.aifs_api_folder))

        self.aifs_install_folder = system_folders['aifs_install']
        self.msg_if.pub_info("Using AI Frameworks Install Folder: " + str(self.aifs_install_folder))
        
        self.ai_models_folder = system_folders['aifs_models']
        self.msg_if.pub_info("Using AI Models Folder: " + str(self.ai_models_folder))

        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()

        self.msg_if.pub_info("Waiting for driver manager to start")
        active_drivers = nepi_system.get_active_drivers(log_name_list = [self.node_name])
        nepi_sdk.sleep(5) # Some extra time for drivers to load
        
       
        ##############################
        # Initialize Variables


        message = "NO MODELS ENABLED"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_no_models_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        message = "LOADING MODELS"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_loading_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        message = "WAITING FOR MODELS TO PUBLISH"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_waiting_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        self.initCb(do_updates = False)

        
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
            'models_dict': {
                'namespace': self.node_namespace,
                'factory_val': dict()
            }

        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'active_models_info_query': {
                'namespace': self.node_namespace,
                'topic': 'active_models_info_query',
                'srv': AiModelsInfoQuery,
                'req': AiModelsInfoQueryRequest(),
                'resp': AiModelsInfoQueryResponse(),
                'callback': self.handleInfoRequest
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': MgrAiModelsStatus,
                'qsize': 1,
                'latch': True
            }
        }  


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'refresh_frameworks': {
                'namespace': self.node_namespace,
                'topic': 'refresh_frameworks',
                'msg': Empty,
                'qsize': 10,
                'callback': self.refreshCb, 
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
            'enable_all_frameworks': {
                'namespace': self.node_namespace,
                'topic': 'enable_all_frameworks',
                'msg': Empty,
                'qsize': 10,
                'callback': self.enableAllFwsCb, 
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
            }

        }



        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        msg_if = self.msg_if
        )


        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()


        ###########################

        self.initCb(do_updates = True)


        ###########################


        self.all_namespace = os.path.join(self.base_namespace,'ai')
        self.msg_if.pub_info("Staring all detectors on namespace " + self.all_namespace)
        self.ros_loading_img.header.stamp = nepi_sdk.get_msg_stamp()


        self.msg_if.pub_info("Staring AI Framework and Model update process")
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)
        self.msg_if.pub_info("Staring AI Model Info update process")
        nepi_sdk.start_timer_process(1.0, self.modelsInfoUpdaterCb, oneshot = True)

        self.ros_waiting_img.header.stamp = nepi_sdk.get_msg_stamp()
        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################




    ####################
    # Wait for System and Config Statuses Callbacks
    def systemStatusCb(self,msg):
        self.sys_status = True

    def configStatusCb(self,msg):
        self.cfg_status = True
        


    #######################
    ### Mgr Config Functions

    def refreshCb(self,msg):
        self.refresh()

    def refresh(self):
        success = False
        if self.node_if is not None:
            ## Find AI Frameworks
            # Get ai framework dict form param server and update
            #self.msg_if.pub_warn("Refreshing latest ais dict " + str(self.aifs_dict), throttle_s = 5.0)

            # Update AI Frameworks Dict
            aifs_dict = copy.deepcopy(self.aifs_dict)
            #self.msg_if.pub_warn("Refreshing")
            #self.msg_if.pub_warn("Refreshing aifs dict with keys: " + str(self.aifs_dict.keys()))
            #self.msg_if.pub_warn("Refreshing active aifs list: " + str(self.getActiveAifs())) 
            last_aifs_keys = aifs_dict.keys()
            #self.msg_if.pub_warn("Got last aifs " + str(last_aifs_keys), throttle_s = 5.0)
            aifs_dict = nepi_aifs.refreshAIFsDict(self.aifs_param_folder,self.aifs_api_folder,aifs_dict)
            #self.msg_if.pub_warn("Refreshed aifs dict with keys: " + str(self.aifs_dict.keys()))
            #self.msg_if.pub_warn("Refreshed active aifs list: " + str(self.getActiveAifs())) 
            cur_aifs_keys = aifs_dict.keys()
            #self.msg_if.pub_warn("Got updated aifs " + str(cur_aifs_keys), throttle_s = 5.0)
            if last_aifs_keys != cur_aifs_keys:
                self.msg_if.pub_warn("Got updated ai framework list: " + str(cur_aifs_keys), throttle_s = 5.0)

            # Refresh Frameworks if Needed
            for aif_name in aifs_dict.keys():
                aif_dict = aifs_dict[aif_name]
                #self.msg_if.pub_info("Processing ais dict for aif name " + aif_name)
                success = False
                if aif_name not in self.aifs_classes_dict.keys():
                    #self.msg_if.pub_info("Updating ai dict for framework: " + str(aif_name), throttle_s = 5.0)
                    file_name = aif_dict['if_file_name']
                    file_path = aif_dict['api_path']
                    module_name = aif_dict['if_module_name']
                    class_name = aif_dict['if_class_name']
                    self.msg_if.pub_warn("Got aif dict: " + aif_name + " : " + str(aif_dict), throttle_s = 5.0)
                    sys.path.append(file_path)
                    [success, msg, aif_class] = nepi_aifs.importAIFClass(file_name,file_path,module_name,class_name)
                    if success == True:
                        success = False
                        try:
                            self.msg_if.pub_warn("Instantiating IF class for framework type: " + str(aif_name), throttle_s = 5.0)
                            launch_namespace = os.path.join(self.base_namespace, "ai")
                            all_namespace = os.path.join(self.base_namespace, self.node_name)
                            aif_if_class_instance = aif_class(aif_dict,launch_namespace,all_namespace,self.ai_models_folder)
                            self.aifs_classes_dict[aif_name] = aif_if_class_instance
                            self.msg_if.pub_warn("Created aif instantiated if class: " + aif_name , throttle_s = 5.0)
                            success = True
                            time.sleep(1) # Give some time for publishers to set in class init
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to instantiate ai framework if class " + class_name + " " + str(e), throttle_s = 5.0)
                else:
                    aif_if_class_instance = self.aifs_classes_dict[aif_name]
                    #self.msg_if.pub_warn("Got aif instantiated class: " + aif_name , throttle_s = 5.0)
                    success = True


                # Refresh Models if Needed
                if success == True:
                    success = False
                    cur_models_dict = copy.deepcopy(self.models_dict)
                    aif_models_dict = aif_if_class_instance.getModelsDict()
                    models_dict = dict()
                    if cur_models_dict != aif_models_dict:
                        #self.msg_if.pub_warn("Got models dict for aif: " + aif_name + " " + str(aif_models_dict.keys()), throttle_s = 5.0)
                        if (len(aif_models_dict.keys()) < 1):
                            self.msg_if.pub_warn("No models found for this ai framework: " + aif_name)
                        else:
                            #self.msg_if.pub_info("Got models for framework: " + str(aif_name) + " from param server " + str(aif_models_dict.keys()))
                            pass
                        for model_name in aif_models_dict.keys():
                            try:
                                if model_name not in self.aif_classes_dict.keys():
                                    model_dict = aif_models_dict[model_name]
                                    if model_name in cur_models_dict.keys():
                                        model_dict['active'] = cur_models_dict[model_name]['active']
                                    else:
                                        model_dict['active'] = False
                                    models_dict[model_name] = model_dict
                                    #self.msg_if.pub_info("Got update models dict for framework: " + str(aif_name) + " from param server " + str(models_dict.keys()))
                                    self.aif_classes_dict[model_name] = aif_if_class_instance
                                    success = True
                            except Exception as e:
                                self.msg_if.pub_warn("Failed to get models from class " + class_name + " " + str(e))
                                continue

            # refresh active states from stored values
            #self.msg_if.pub_warn("Refreshing models dict with keys: " + str(self.models_dict.keys()))
            #self.msg_if.pub_warn("Refreshing active models list: " + str(self.getActiveModels()))

            for model in models_dict.keys():
                if model not in cur_models_dict.keys():
                    cur_models_dict[model] = models_dict[model]

            last_dict = copy.deepcopy(self.models_dict)
            self.aifs_dict = aifs_dict
            self.models_dict =  cur_models_dict
            self.node_if.set_param('aifs_dict', self.aifs_dict)
            self.node_if.set_param('models_dict', self.models_dict)
            if self.models_dict != last_dict:
                self.save_config() # Save config
                self.publish_status()
       
            #self.msg_if.pub_warn("Refreshed models dict with keys: " + str(self.models_dict.keys()))
            #self.msg_if.pub_warn("Refreshed active models list: " + str(self.getActiveModels()))
        return success
        

        

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
      if self.node_if is not None:
        self.aifs_dict = self.node_if.get_param('aifs_dict')
        self.msg_if.pub_warn("Init")
        self.msg_if.pub_warn("Init aifs dict with keys: " + str(self.aifs_dict.keys()))
        self.msg_if.pub_warn("Init active aifs list: " + str(self.getActiveAifs()))
        self.models_dict = self.node_if.get_param('models_dict')
        self.msg_if.pub_warn("Init models dict with keys: " + str(self.models_dict.keys()))
        self.msg_if.pub_warn("Init active models list: " + str(self.getActiveModels()))
        self.node_if.save_config()

      if do_updates == True:
        self.refresh()
      self.publish_status()



    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        self.aifs_classes_dict = dict()
        self.aif_classes_dict = dict()
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def getActiveAifs(self):
        aaifs = []
        aifs_dict = copy.deepcopy(self.aifs_dict)
        for aif in aifs_dict.keys():
            if aifs_dict[aif]['active'] == True:
                aaifs.append(aif)
        return aaifs

    def getActiveModels(self):
        ams = []
        models_dict = copy.deepcopy(self.models_dict)
        for m in models_dict.keys():
            if models_dict[m]['active'] == True:
                ams.append(m)
        return ams

    def updaterCb(self,timer):
        active_aifs = self.getActiveAifs()
        #self.msg_if.pub_warn("Update")
        #self.msg_if.pub_warn("Updating aifs dict with keys: " + str(self.aifs_dict.keys()))
        #self.msg_if.pub_warn("Updating active aifs list: " + str(self.getActiveAifs()))        
        models_dict = copy.deepcopy(self.models_dict)
        active_models_list = self.getActiveModels()
        #self.msg_if.pub_warn("Updating models dict with keys: " + str(self.models_dict.keys()))
        #self.msg_if.pub_warn("Updating active models list: " + str(self.getActiveModels()))
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
                nepi_sdk.wait()

       
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
                    nepi_sdk.wait()
                    model_type = models_dict[model_name]['type']
                    if model_type == "detection":
                        self.detector_info_dict[model_name] = None # Gets updated in modelsInfoUpdateCb
                else:
                    self.msg_if.pub_warn("Model failed to load: " + model_name)
                    self.msg_if.pub_warn("Setting model to disabled: " + model_name)
                    self.models_dict[model_name]['active'] = False
                    self.models_dict = models_dict

        #self.msg_if.pub_warn("Updated models dict with keys: " + str(self.models_dict.keys()))
        #self.msg_if.pub_warn("Updated active models list: " + str(self.getActiveModels()))
        if len(active_models_list) == 0:
            self.ros_no_models_img.header.stamp = nepi_sdk.get_msg_stamp()
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param("models_dict",self.models_dict)
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)


    def modelsInfoUpdaterCb(self,timer):
        #success = self.refresh()
        models_dict = copy.deepcopy(self.models_dict)
        model_names = models_dict.keys()
        #self.msg_if.pub_warn("Calling model info services for models: " + str(model_names))
        # Update detector info
        detector_info_dict = dict()
        model_namespace_dict = copy.deepcopy(self.model_namespace_dict)
        for model_name in model_namespace_dict.keys():
            namespace = model_namespace_dict[model_name]
            model_type = models_dict[model_name]['type']
            #self.msg_if.pub_warn("Calling model info service for model namespace: " + str(namespace) + " " + str(model_type)) 
            if namespace is not None and model_type == 'detection':
                # Check for service
                service_namespace = os.path.join(namespace,'detector_info_query')
                #self.msg_if.pub_warn("Looking for det info services topic: " + str(service_namespace) )
                service_exists = True # nepi_sdk.check_for_service(service_namespace)
                #self.msg_if.pub_warn("Got check for service reply: " + str(service_exists) )
                if service_exists == True:
                    try:
                        #self.msg_if.pub_info("Connecting to model info service " + service_namespace)
                        info_service = nepi_sdk.connect_service(service_namespace, AiDetectorInfoQuery)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to call model info service: " + service_namespace + " " + str(e))
                    try:
                        #self.msg_if.pub_info("Requesting model info for service" + service_namespace)
                        request = AiDetectorInfoQueryRequest()
                        response = nepi_sdk.call_service(info_service, request)
                        #self.msg_if.pub_info("Got model info response: " + str(response))
                        detector_info_dict[model_name] = response.detector_info
                    except Exception as e:
                        self.msg_if.pub_info("Waiting for model to load: " + str(model_name) + " , will try again in " + str(self.MODEL_INFO_INTERVAL) + " secs")
                else:
                    self.msg_if.pub_warn("Failed to find model info service: " + model_name + " " + namespace)
        self.detector_info_dict = detector_info_dict
        nepi_sdk.start_timer_process(self.MODEL_INFO_INTERVAL, self.modelsInfoUpdaterCb, oneshot = True)


    def handleInfoRequest(self,_):
        resp = AiModelsInfoQueryResponse()
        model_name_list = []
        model_info_list = []
        for model_name in self.detector_info_dict.keys():
            if self.detector_info_dict[model_name] is not None:
                model_name_list.append(model_name)
                model_info_list.append(self.detector_info_dict[model_name])
        resp.detector_name_list = model_name_list
        resp.detector_info_list = model_info_list
        #self.msg_if.pub_warn("Respose for model info query " + str(resp))
        return resp


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
                nepi_sdk.wait()

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
                got_topic = nepi_sdk.wait_for_topic(status_topic,timeout = timeout)
                if got_topic == "":
                    self.msg_if.pub_warn("Model status timed out for: " + model_name)
                else:
                    self.msg_if.pub_warn("Got model status: " + model_name)

                # Check node is active
                self.msg_if.pub_warn("Checking that node is active for model " + model_name )
                node_running = nepi_sdk.check_for_node(model_name)
                if node_running == True:
                    self.msg_if.pub_warn("Node Found: " + model_name)
                    self.model_namespace_dict[model_name] = node_namespace
                else: 
                    self.msg_if.pub_warn("Node Not Found" + model_name)
                    success = False
                '''

            self.save_config() # Save config
            self.publish_status()
            return success
        
    def killModel(self, model_name):
        if model_name != "None": 
            models_dict = copy.deepcopy(self.models_dict)
            if not (model_name in models_dict.keys()):
                self.msg_if.pub_warn("Unknown model model requested: " + model_name)
                return
            # Start the model
            aif_class = self.aif_classes_dict[model_name]
            self.msg_if.pub_info("Killing model " + model_name)
            del self.model_namespace_dict[model_name]
            aif_class.killModel(model_name)
            # Kill Img Pub Node if running
            nepi_sdk.kill_node(model_name + '/img_pub')
  
    def save_config(self):
        # Save framework and model dictionaries
        self.node_if.save_config() # Save config after initialization


    def updateFrameworkStateCb(self,msg):
        self.msg_if.pub_warn("Recieved Framework State Update: " + str(msg))
        framework_name = msg.name
        state = msg.active_state
        active_aifs = self.getActiveAifs()
        aifs_dict = copy.deepcopy(self.aifs_dict)
        if state == True:
            if framework_name not in active_aifs:
                self.msg_if.pub_warn("Setting AI Framework: " + str(framework_name) + " Active")
                if framework_name in aifs_dict.keys():
                    aifs_dict[framework_name]['active'] = True
                    self.aifs_dict = aifs_dict
                    self.publish_status()
                    if self.node_if is not None:
                        self.node_if.set_param('aifs_dict', aifs_dict)
                        self.save_config() # Save config
        else:
            if framework_name in active_aifs:
                self.msg_if.pub_warn("Setting AI Framework: " + str(framework_name) + " Inctive")
                aifs_dict[framework_name]['active'] = False
                self.aifs_dict = aifs_dict
                self.publish_status()
                if self.node_if is not None:
                    self.node_if.set_param('aifs_dict', aifs_dict)
                    self.save_config() # Save config
        
        


    def disableAllFwsCb(self,msg):
        aifs_dict = copy.deepcopy(self.aifs_dict)
        for aif in aifs_dict.keys():
            aifs_dict[aif]['active'] = False
        self.aifs_dict = aifs_dict
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('aifs_dict', aifs_dict)
            self.save_config() # Save config


    def enableAllFwsCb(self,msg):
        aifs_dict = copy.deepcopy(self.aifs_dict)
        available_frameworks = list(aifs_dict.keys())
        for aif in aifs_dict.keys():
            aifs_dict[aif]['active'] = True
        self.aifs_dict = aifs_dict
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('aifs_dict', aifs_dict)
            self.save_config() # Save config


    def enableAllModelsCb(self,msg):
        self.msg_if.pub_warn("Recieved Enable All Models msg: " + str(msg))
        aif_name = msg.name
        state = msg.active_state
        active_aifs = self.getActiveAifs()
        if aif_name not in active_aifs:
            self.msg_if.pub_warn("Ignoring request. AI Framework: " + str(aif_name) + " not enabled")
        else:
            models_dict = copy.deepcopy(self.models_dict)
            for model_name in models_dict.keys():
                model_dict = models_dict[model_name]
                if model_dict['framework'] == aif_name:
                    models_dict[model_name]['active'] = True
        self.models_dict = models_dict
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param("models_dict",models_dict)
            self.save_config() # Save config
 

    def disableAllModelsCb(self,msg):
        self.msg_if.pub_warn("Recieved Disable All Models msg: " + str(msg))
        aif_name = msg.name
        state = msg.active_state
        active_aifs = self.getActiveAifs()
        if aif_name not in active_aifs:
            self.msg_if.pub_warn("Ignoring request. AI Framework: " + str(aif_name) + " not enabled")
        else:
            models_dict = copy.deepcopy(self.models_dict)
            for model_name in models_dict.keys():
                model_dict = models_dict[model_name]
                if model_dict['framework'] == aif_name:
                    models_dict[model_name]['active'] = False
        self.models_dict = models_dict
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param("models_dict",models_dict)
            self.save_config() # Save config

    def updateModelStateCb(self,msg):
        self.msg_if.pub_warn("Recieved Model State Update: " + str(msg))
        model_name = msg.name
        state = msg.active_state
        models_dict = copy.deepcopy(self.models_dict)
        if model_name in models_dict.keys():
            model_dict = models_dict[model_name]
            model_aif = model_dict['framework']
            active_aifs = self.getActiveAifs()
            if model_aif not in active_aifs:
                self.msg_if.pub_warn("Ignoring request. Model's AI Framework: " + str(model_aif) + " not enabled")
            else:
                active_state = model_dict['active']
                if state != active_state:
                    if state == True:
                        self.msg_if.pub_warn("Changing Model State to: True")
                        models_dict[model_name]['active'] = True
                    else:
                        self.msg_if.pub_warn("Changing Model State to: False")
                        models_dict[model_name]['active'] = False
        if self.models_dict != models_dict:
            self.models_dict = models_dict
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param("models_dict",models_dict)
                self.save_config() # Save config




    def publish_status(self, do_updates = False):
        aifs_dict = copy.deepcopy(self.aifs_dict)
        models_dict = copy.deepcopy(self.models_dict)

        status_msg = MgrAiModelsStatus()
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


        active_aifs = self.getActiveAifs()
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
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)

if __name__ == '__main__':
    AIDetectorManager()
