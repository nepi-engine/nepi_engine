#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
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

from nepi_interfaces.msg import UpdateBool, MgrAiModelsStatus
from nepi_interfaces.msg import AiDetectorInfo, AiDetectorStatus
from nepi_interfaces.msg import AiBoundingBoxes, ObjectCount

from nepi_interfaces.srv import SystemStorageFolderQuery
from nepi_interfaces.srv import AiModelsInfoQuery, AiModelsInfoQueryRequest, AiModelsInfoQueryResponse
from nepi_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryResponse, AiDetectorInfoQueryRequest


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF


DEFAULT_FRAMEWORK = 'yolov8'


class AIDetectorManager:   

    MODEL_TYPE_LIST = ['detection']
    MODEL_INFO_INTERVAL = 2


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

        # self.msg_if.pub_info("Waiting for driver manager to start")
        # active_drivers = nepi_system.get_active_drivers(log_name_list = [self.node_name])
        # nepi_sdk.sleep(5) # Some extra time for drivers to load
        
       
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
                'msg': UpdateBool,
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
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.enableAllModelsCb, 
                'callback_args': ()
            },
            'disable_all_models': {
                'namespace': self.node_namespace,
                'topic': 'disable_all_models',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.disableAllModelsCb, 
                'callback_args': ()
            },
            'update_model_state': {
                'namespace': self.node_namespace,
                'topic': 'update_model_state',
                'msg': UpdateBool,
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

        success = self.initCb(do_updates = True)


        ###########################


        self.msg_if.pub_info("Staring AI Status Pub process")
        nepi_sdk.start_timer_process(1.0, self.statusPubCb)
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


    def initCb(self, do_updates = False):
      success = True
      if self.node_if is not None:
        self.msg_if.pub_warn("Initializing Node Variables")
        self.aifs_dict = self.node_if.get_param('aifs_dict')

        self.models_dict = self.node_if.get_param('models_dict')

        self.msg_if.pub_warn("Calling Refresh Process")
        success = self.refresh()
        # self.msg_if.pub_warn("Init models dict with keys: " + str(self.models_dict.keys()))
        # for key in self.models_dict.keys():
        #     self.msg_if.pub_warn("Init model dict with keys: " + str(key) + ' : ' + str(self.models_dict[key].keys()))
        # self.msg_if.pub_warn("**********************")
        # self.msg_if.pub_warn(" ")

        #self.msg_if.pub_warn("Init active models list: " + str(self.getActiveModels()))

        #self.msg_if.pub_warn("Saving Updated Config")
        #self.node_if.save_config()

        #self.msg_if.pub_warn("Publishing Status")
        #self.publish_status()
      return success



    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)


    def factoryResetCb(self,do_updates = True):
        self.aifs_classes_dict = dict()
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.initCb(do_updates = do_updates)



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
        #self.msg_if.pub_warn("Starting Refresh with models dict with keys: " + str(self.models_dict.keys()))
        self.msg_if.pub_warn("Stopping all running models")
        for model_name in self.models_dict.keys():
                if self.models_dict[model_name]['active'] == True:
                  self.killModel(model_name)
        time.sleep(1)
        aif_classes = list(self.aifs_classes_dict.keys())
        for aif_class in aif_classes:
            del self.aifs_classes_dict[aif_class]


        if self.node_if is not None:

            ## Find AI Frameworks
            # Get ai framework dict form param server and update
            #self.msg_if.pub_warn("Refreshing latest ais dict " + str(self.aifs_dict))

            # Update AI Frameworks Dict
            orig_aifs_dict = copy.deepcopy(self.aifs_dict)
            cur_aif_keys = orig_aifs_dict.keys()
            #self.msg_if.pub_warn("Got last aifs " + str(cur_aif_keuys))
            aifs_dict = nepi_aifs.getAIFsDict(self.aifs_param_folder, self.aifs_api_folder)
            #self.msg_if.pub_warn("Refreshed aifs dict with keys: " + str(self.aifs_dict.keys()))
            #self.msg_if.pub_warn("Refreshed active aifs list: " + str(self.getActiveAifs())) 
            new_aif_keys = aifs_dict.keys()
            #self.msg_if.pub_warn("Got updated aifs " + str(new_aif_keys))
            if cur_aif_keys != new_aif_keys:
                self.msg_if.pub_warn("Got updated ai framework list: " + str(new_aif_keys))

            orig_models_dict = copy.deepcopy(self.models_dict)
            models_dict = dict()

            # Refresh Frameworks if Needed
            for aif_name in aifs_dict.keys():
                aif_dict = aifs_dict[aif_name]

                self.msg_if.pub_info(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> " + aif_name)
                self.msg_if.pub_info("Processing ais dict for aif name " + aif_name)
                success = False
                if aif_name not in self.aifs_classes_dict.keys():
                    self.msg_if.pub_info("Updating ai dict for framework: " + str(aif_name))
                    file_name = aif_dict['if_file_name']
                    file_path = aif_dict['api_path']
                    module_name = aif_dict['if_module_name']
                    class_name = aif_dict['if_class_name']
                    #self.msg_if.pub_warn("Got aif dict: " + aif_name + " : " + str(aif_dict))
                    sys.path.append(file_path)
                    self.msg_if.pub_warn("Importing AIF Class File: " + str(file_path)  + " " + file_name + " from module " + module_name)
                    [success, msg, aif_class] = nepi_aifs.importAIFClass(file_name,file_path,module_name,class_name)
                    if success == True:
                        success = False
                        try:
                            self.msg_if.pub_warn("Instantiating IF class for framework: " + str(aif_name))
                            launch_namespace = self.base_namespace #os.path.join(self.base_namespace, "ai")
                            aif_if_class_instance = aif_class(aif_dict, launch_namespace, self.ai_models_folder)
                            self.aifs_classes_dict[aif_name] = aif_if_class_instance
                            self.msg_if.pub_warn("Created aif instantiated if class: " + aif_name )
                            success = True
                            time.sleep(1) # Give some time for publishers to set in class init
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to instantiate ai framework if class " + class_name + " " + str(e))
                    else:
                        self.msg_if.pub_warn("Failed to Import ai framework if class " + class_name + " " + str(e))

                else:
                    aif_if_class_instance = self.aifs_classes_dict[aif_name]
                    self.msg_if.pub_warn("Got aif instantiated class: " + aif_name)
                    success = True




                # Refresh Models Dict
                if success == True:
                    if aif_name in orig_aifs_dict.keys():
                        aifs_dict[aif_name]['active'] = orig_aifs_dict[aif_name]['active']
                    else:
                        aifs_dict[aif_name]['active'] = False
                    try:
                        aif_models_dict = aif_if_class_instance.getModelsDict()
                    
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to Get Model Info for ai framework: " + aif_name + " " + str(e))
                    if (len(aif_models_dict.keys()) < 1):
                        self.msg_if.pub_warn("No models found for this ai framework: " + aif_name)
                    else:
                        self.msg_if.pub_info("Got models for framework: " + str(aif_name) + " from param server " + str(aif_models_dict.keys()))
                        pass
                    for model_name in aif_models_dict.keys():
                        try:
                            if model_name in orig_models_dict.keys():
                                aif_models_dict[model_name]['active'] = orig_models_dict[model_name]['active']
                            else: 
                                aif_models_dict[model_name]['active'] = False

                            models_dict[model_name] = aif_models_dict[model_name]
                          
                            success = True
                        except Exception as e:
                            self.msg_if.pub_warn("Failed to get models from class " + class_name + " " + str(e))
                            continue
                        
                        

                   
    
           
            self.aifs_dict = aifs_dict
            self.node_if.set_param('aifs_dict', aifs_dict)
            self.models_dict = models_dict
            self.node_if.set_param('models_dict',models_dict)      


            #self.msg_if.pub_warn("Refreshing models dict with keys: " + str(self.models_dict.keys()))
            # for key in self.models_dict.keys():
            #     self.msg_if.pub_warn( str(key) + ' : ' + str(self.models_dict[key].keys()))
            # self.msg_if.pub_warn("**********************")
            # self.msg_if.pub_warn(" ")
            # self.msg_if.pub_warn(" ")

            # nepi_sdk.sleep(1)
            # self.save_config() # Save config
            # self.publish_status()
       
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
 

    def getActiveAifs(self):
        aaifs = []
        aifs_dict = copy.deepcopy(self.aifs_dict)
        for aif_name in aifs_dict.keys():
            if 'active' in aifs_dict[aif_name].keys():
                if aifs_dict[aif_name]['active'] == True:
                    aaifs.append(aif_name)
        return aaifs

    def getActiveModels(self):
        active_models = []
        models_dict = copy.deepcopy(self.models_dict)
        for model_name in models_dict.keys():
            if 'active' in models_dict[model_name].keys():
                if models_dict[model_name]['active'] == True:
                    active_models.append(model_name)
        return active_models


    def updaterCb(self,timer):
        #self.msg_if.pub_warn("Starting Updater with models dict with keys: " + str(self.models_dict.keys()))
        # for key in self.models_dict.keys():
        #     self.msg_if.pub_warn("Starting Updater with g model dict with keys: " + str(key) + ' : ' + str(self.models_dict[key].keys()))
        # self.msg_if.pub_warn("**********************")
        # self.msg_if.pub_warn(" ")

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
                self.msg_if.pub_warn("Launching Node for model: " + model_name)
                model_dict = models_dict[model_name]
                [success,node_namespace] = self.loadModel(model_name, model_dict)
                if success == True:
                    self.running_models_list.append(model_name)
                    nepi_sdk.wait()
                    model_type = models_dict[model_name]['type']
                    if model_type == "detection":
                        self.detector_info_dict[model_name] = None # Gets updated in modelsInfoUpdateCb
                else:
                    self.msg_if.pub_warn("Model failed to launch: " + model_name)
                    self.msg_if.pub_warn("Setting model to disabled: " + model_name)
                    self.models_dict[model_name]['active'] = False
    

        # self.msg_if.pub_warn("Updated models dict with keys: " + str(self.models_dict.keys()))
        # for key in self.models_dict.keys():
        #     self.msg_if.pub_warn("Updated model dict with keys: " + str(key) + ' : ' + str(self.models_dict[key].keys()))

        #self.msg_if.pub_warn("Updated active models list: " + str(self.getActiveModels()))
        if len(active_models_list) == 0:
            self.ros_no_models_img.header.stamp = nepi_sdk.get_msg_stamp()
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param("models_dict",self.models_dict)


        #self.msg_if.pub_warn("Ending Updater with models dict with keys: " + str(self.models_dict.keys()))
        # for key in self.models_dict.keys():
        #     self.msg_if.pub_warn("Ending Updater with g model dict with keys: " + str(key) + ' : ' + str(self.models_dict[key].keys()))
        # self.msg_if.pub_warn("**********************")
        # self.msg_if.pub_warn(" ")

        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)


    def modelsInfoUpdaterCb(self,timer):
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
                        #self.msg_if.pub_info("Waiting for model to load: " + str(model_name) + " , will try again in " + str(self.MODEL_INFO_INTERVAL) + " secs")
                        pass
                else:
                    self.msg_if.pub_warn("Failed to find model info service: " + model_name + " " + namespace)
        self.detector_info_dict = detector_info_dict

        # self.msg_if.pub_warn("Ending Info Updater with models dict with keys: " + str(self.models_dict.keys()))
        # for key in self.models_dict.keys():
        #     self.msg_if.pub_warn("Ending Info Updater with g model dict with keys: " + str(key) + ' : ' + str(self.models_dict[key].keys()))
        # self.msg_if.pub_warn("**********************")
        # self.msg_if.pub_warn(" ")

        nepi_sdk.start_timer_process(self.MODEL_INFO_INTERVAL, self.modelsInfoUpdaterCb, oneshot = True)




    def loadModel(self, model_name, model_dict):       
        success = False
        if model_name != "None": 
            aif_name=self.models_dict[model_name]['framework']
            node_namespace ='None'
            if aif_name not in self.aifs_classes_dict.keys():
                self.msg_if.pub_warn("Model Framework Class not instantiated")
            else:
                try:
                    aif_class = self.aifs_classes_dict[aif_name]
                    self.msg_if.pub_warn("Launching Node for Model " + model_name)
                    [success,node_namespace] = aif_class.launchModel(model_dict)
                    self.msg_if.pub_warn("Model Node Launched with namespace: " + node_namespace)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to Launch Node for model: " + model_name + " : " + str(e))

                if success == True:
                    # Just Assume Running for now
                    #self.msg_if.pub_warn("Node Found: " + model_name)
                    self.model_namespace_dict[model_name] = node_namespace
                    nepi_sdk.wait()

                self.save_config() # Save config
                self.publish_status()
            return [success,node_namespace]
        
    def killModel(self, model_name):
        if model_name != "None": 
            models_dict = copy.deepcopy(self.models_dict)
            if model_name not in models_dict.keys():
                self.msg_if.pub_warn("Unknown model model requested: " + model_name)
            else:
                aif_name=self.models_dict[model_name]['framework']
                if aif_name not in self.aifs_classes_dict.keys():
                    self.msg_if.pub_warn("Model Framework Class not instantiated")
                else:
                    # Call Kill model
                    aif_class = self.aifs_classes_dict[aif_name]
                    self.msg_if.pub_info("Killing model " + model_name)
                    if model_name in list(self.model_namespace_dict.keys()):
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
        state = msg.value
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
        state = msg.value
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
        state = msg.value
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
        state = msg.value
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



    def handleInfoRequest(self,_):
        self.msg_if.pub_warn("Handling Request for model info query ")
        resp = AiModelsInfoQueryResponse()
        model_name_list = []
        model_info_list = []
        for model_name in self.detector_info_dict.keys():
            if self.detector_info_dict[model_name] is not None:
                model_name_list.append(model_name)
                model_info_list.append(self.detector_info_dict[model_name])
        resp.detector_name_list = model_name_list
        resp.detector_info_list = model_info_list
        self.msg_if.pub_warn("Respose for model info query " + str(resp))
        return resp

    def statusPubCb(self,timer):
        self.publish_status()

    def publish_status(self, do_updates = False):
        aifs_dict = copy.deepcopy(self.aifs_dict)
        models_dict = copy.deepcopy(self.models_dict)

        #self.msg_if.pub_warn("Status Using AIFs Dict: " + str(aifs_dict.keys()))
        # self.msg_if.pub_warn("Status Using Models Dict: " + str(models_dict.keys()))

        status_msg = MgrAiModelsStatus()

        status_msg.ai_frameworks = nepi_aifs.getAIFsSortedList(aifs_dict)

        
        #self.msg_if.pub_warn("Updated Model Mgr Status Msg: " + str(status_msg))

        active_aifs = self.getActiveAifs()
        model_folders = []
        for aif_name in active_aifs:
            model_folders.append(aifs_dict[aif_name]['models_folder_name'])
        status_msg.active_ai_frameworks = active_aifs
        status_msg.active_ai_frameworks_folders = model_folders



        [ai_models,ai_names] = nepi_aifs.getModelsSortedList(models_dict)
        #self.msg_if.pub_warn("Status got sourced models: " + str( [ai_models,ai_names]))
        status_msg.ai_models = ai_models
        status_msg.ai_models_names = ai_names

        #self.msg_if.pub_warn("Updated Model Mgr Status Msg: " + str(status_msg))

        aif_list = []
        type_list = []
        state_list = []

        active_model_list = []
        active_name_list = []
        active_type_list = []
        active_node_list = []
        active_namespace_list = []
        for model_name in ai_models:
            type_list.append(models_dict[model_name]['type'])
            aif_list.append(models_dict[model_name]['framework'])
            active = False
            if 'active' in models_dict[model_name].keys():
                active = models_dict[model_name]['active']
                state_list.append(active)
            else:
                state_list.append(False)
            if active == True:
                active_model_list.append(model_name)
                active_name_list.append(models_dict[model_name]['name'])
                active_type_list.append(models_dict[model_name]['type'])
                active_node_list.append(model_name)
                if model_name in self.model_namespace_dict.keys():
                    active_namespace_list.append(self.model_namespace_dict[model_name])
                else:
                    active_namespace_list.append('')

                


        status_msg.ai_models_frameworks = aif_list
        status_msg.ai_models_types = type_list
        status_msg.ai_models_states = state_list


        status_msg.active_ai_models = active_model_list
        status_msg.active_ai_models_names = active_name_list
        status_msg.active_ai_models_types = active_type_list
        status_msg.active_ai_models_nodes = active_node_list
        status_msg.active_ai_models_namespaces = active_namespace_list



      




        status_msg.all_namespace = self.all_namespace

        #self.msg_if.pub_warn("Sending Model Mgr Status Msg: " + str(status_msg))
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)

if __name__ == '__main__':
    AIDetectorManager()
