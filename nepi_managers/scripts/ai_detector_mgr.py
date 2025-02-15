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
import errno
import glob
import subprocess
import yaml
import time
import rospy
import numpy as np
import cv2

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_save
from nepi_sdk import nepi_aifs
from nepi_sdk import nepi_msg 
from nepi_sdk import nepi_img


from std_msgs.msg import Empty, Float32, String, Bool, Int32
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from nepi_ros_interfaces.msg import UpdateState, AiMgrStatus, ImageClassifierStatus
from nepi_ros_interfaces.srv import SystemStorageFolderQuery
from nepi_ros_interfaces.msg import BoundingBoxes, ObjectCount,ClassifierSelection


from nepi_sdk.save_cfg_if import SaveCfgIF
from nepi_sdk.save_data_if import SaveDataIF


class AIDetectorManager:   

    AIFS_SHARE_PATH = '/opt/nepi/ros/share/nepi_aifs'
    AI_MODELS_PATH = '/mnt/nepi_storage/ai_models/'


    data_products = ['bounding_boxes','detection_image']

    init_aifs_dict = dict()
    init_models_dict = dict()
    models_dict = dict()
    aif_classes_dict = dict()

    save_cfg_if = None

    init_active_classifier = "None"

    running_classifiers_list = []
    classifier_namespace_dict = dict()

    active_classifier = "None"

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "aif_detector_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### MGR NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
        ##############################

        # Create a message image to publish when not running
        message = "NO CLASSIFIER ENABLED"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_no_clf_img = nepi_img.cv2img_to_rosimg(cv2_img) 

        message = "CLASSIFIER LOADING"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_clf_loading = nepi_img.cv2img_to_rosimg(cv2_img) 

        message = "WAITING FOR CLASSIFIER TO PUBLISH"
        cv2_img = nepi_img.create_message_image(message)
        self.ros_wait_clf_img = nepi_img.cv2img_to_rosimg(cv2_img) 



        get_folder_name_service = self.base_namespace + 'system_storage_folder_query'
        nepi_msg.publishMsgInfo(self,"Waiting for system automation scripts folder query service " + get_folder_name_service)
        rospy.wait_for_service(get_folder_name_service)
        nepi_msg.publishMsgInfo(self,"Calling system automation scripts folder query service " + get_folder_name_service)
        try:
            folder_query_service = rospy.ServiceProxy(get_folder_name_service, SystemStorageFolderQuery)
            time.sleep(1)
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain folder query service " + str(e))
        try:
            nepi_msg.publishMsgInfo(self,"Getting AI Frameworks folder query service " + get_folder_name_service)
            response = folder_query_service("aifs")
            nepi_msg.publishMsgInfo(self,"Got AI Frameworks folder path" + response.folder_path)
            self.AIFS_SHARE_PATH = response.folder_path
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain AI Frameworks folder, falling back to: " + self.AIFS_SHARE_PATH + " " + str(e))
        try:
            nepi_msg.publishMsgInfo(self,"Getting AI Models folder query service " + get_folder_name_service)
            response = folder_query_service("ai_models")
            nepi_msg.publishMsgInfo(self,"Got AI Models folder path" + response.folder_path)
            self.AI_MODELS_PATH = response.folder_path
        except Exception as e:
            nepi_msg.publishMsgWarn(self,"Failed to obtain AI Models folder, falling back to: " + self.AI_MODELS_PATH + " " + str(e))





       # Create Node Publishers
        self.status_pub = rospy.Publisher("~status", AiMgrStatus, queue_size=1, latch=True)
        self.status_clf_pub = rospy.Publisher('~active_classifier/status', ImageClassifierStatus,  queue_size = 1, latch=True)
        time.sleep(1)

        self.refreshFrameworks()
       

        # Initialize apps_mgr param server
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, 
                                    paramsModifiedCallback=self.updateFromParamServer)
        self.save_cfg_if.userReset()
        # Set up save data and save config services ########################################################
        factory_data_rates= {}
        for d in self.data_products:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'detection_image' in self.data_products:
            factory_data_rates['detection_image'] = [1.0, 0.0, 100.0] 
        self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)


 
        # Setup Node Subscribers
        app_reset_app_sub = rospy.Subscriber('~reset_factory', Empty, self.resetAppCb, queue_size = 10)

        rospy.Subscriber('~set_active_classifier', String, self.setActiveClassifierCb, queue_size=1)

        ## Mgr ROS Setup 
        #mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
        mgr_reset_sub = rospy.Subscriber('~refreshFrameworks', Empty, self.refreshFrameworksCb, queue_size = 10)

        self.detection_image_pub = rospy.Publisher("~detection_image", Image,queue_size=1, latch=True)
        rospy.Subscriber("~detection_image", Image, self.detectionImageCb, queue_size = 1)
        rospy.Subscriber("~bounding_boxes", BoundingBoxes, self.boundingBoxesCb, queue_size = 1)
        time.sleep(1)

        # Framework Management Scubscirbers
        rospy.Subscriber('~enable_all_frameworks', Empty, self.enableAllFwsCb, queue_size = 10)
        rospy.Subscriber('~disable_all_frameworks', Empty, self.disableAllFwsCb, queue_size = 10)
        rospy.Subscriber('~update_framework_state', UpdateState, self.updateFwStateCb)
        #Model Management Scubscirbers
        rospy.Subscriber('~enable_all_models', Empty, self.enableAllModelsCb, queue_size = 10)
        rospy.Subscriber('~disable_all_models', Empty, self.disableAllModelsCb, queue_size = 10)
        rospy.Subscriber('~update_model_state', UpdateState, self.updateModelStateCb)
        # Create status pub


        time.sleep(1)
       
        # Load default params
        self.updateFromParamServer()
        self.saveSettings() # Save config
        nepi_ros.timer(nepi_ros.duration(1), self.updaterCb)
        #########################################################
        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################


    def refreshFrameworksCb(self,msg):
        self.refreshFrameworks()

    def refreshFrameworks(self):
        ## Find AI Frameworks
        # Get ai framework dict form param server and update
        get_aifs_dict = nepi_aifs.getAIFsDict(self.AIFS_SHARE_PATH)
        #nepi_msg.publishMsgWarn(self,"Got latest ais dict " + str(get_aifs_dict))
        aifs_dict = nepi_ros.get_param(self,'~aifs_dict', get_aifs_dict)
        self.init_aifs_dict = nepi_aifs.refreshAIFsDict(self.AIFS_SHARE_PATH,aifs_dict)
        nepi_ros.set_param(self,'~aifs_dict', self.init_aifs_dict)
        #nepi_msg.publishMsgWarn(self,"Got updated ais dict from param server " + str(self.init_aifs_dict))
        for aif_name in self.init_aifs_dict.keys():
            aif_dict = self.init_aifs_dict[aif_name]
            nepi_msg.publishMsgInfo(self,"Processing ais dict for ai name " + aif_name + " " + str(aif_dict))
            if aif_dict['active']:
                nepi_msg.publishMsgInfo(self,"Updating ai dict for framework: " + str(aif_name))
                file_name = aif_dict['if_file_name']
                file_path = aif_dict['if_path']
                module_name = aif_dict['if_module_name']
                class_name = aif_dict['if_class_name']
                sys.path.append(file_path)
                [success, msg, aif_class] = nepi_aifs.importAIFClass(file_name,file_path,module_name,class_name)
                if success == False:
                    nepi_msg.publishMsgWarn(self,"Failed to import ai framework if file " + file_name)
                    break
                else:
                    success = False
                    try:
                        nepi_msg.publishMsgInfo(self,"Instantiating IF class for framework type: " + str(aif_name))
                        launch_namespace = os.path.join(self.base_namespace, "ai")
                        mgr_namespace = os.path.join(self.base_namespace, self.node_name)
                        class_instance = aif_class(aif_dict,launch_namespace,mgr_namespace,self.AI_MODELS_PATH)
                        success = True
                        time.sleep(1) # Give some time for publishers to set in class init
                    except Exception as e:
                        nepi_msg.publishMsgWarn(self,"Failed to instantiate ai framework class " + class_name + " " + str(e))
                    models_dict = dict()
                    if success:
                        try:
                            models_dict = class_instance.getModelsDict()
                            #nepi_msg.publishMsgWarn(self,"Got Models Dict " + str(models_dict))
                            for model_name in models_dict.keys():
                                model_dict = models_dict[model_name]
                                model_dict['type'] = aif_name
                                model_dict['active'] = False
                                self.models_dict[model_name] = model_dict
                                self.aif_classes_dict[model_name] = class_instance
                        except Exception as e:
                            nepi_msg.publishMsgWarn(self,"Failed to get models from class " + class_name + " " + str(e))
                            break
                        
                        if (len(models_dict.keys()) < 1):
                            nepi_msg.publishMsgWarn(self,"No classiers identified for this system at " + file_path)
                        else:
                            nepi_msg.publishMsgInfo(self,"Got models for framework type: " + str(aif_name) + " from param server " + str(self.models_dict.keys()))
        #nepi_msg.publishMsgWarn(self,"Got models dict from nepi_aifs call " + str(self.models_dict))
        self.init_models_dict = nepi_ros.get_param(self,'~models_dict', self.models_dict)
        nepi_ros.set_param(self,'~models_dict', self.init_models_dict)
        purge_list = []
        for model_name in self.init_models_dict.keys():
            if model_name not in self.models_dict.keys():
                purge_list.append(model_name)
        for model_name in purge_list:
            del self.init_models_dict[model_name]
        for model_name in self.models_dict.keys():
            if model_name not in self.init_models_dict.keys():
                self.init_models_dict[model_name] = self.models_dict[model_name]
        nepi_ros.set_param(self,'~models_dict', self.init_models_dict)
        #nepi_msg.publishMsgWarn(self,"Storing classifier dict in param server: " + str(self.init_models_dict))

    def resetAppCb(self,msg):
        self.resetApp()

    def resetApp(self):
        self.enableClassifier("None")
        self.publish_status()


    def setCurrentSettingsAsDefault(self):
        nepi_msg.publishMsgInfo(self,"Setting current values as default params")
        nepi_ros.set_param(self,'~active_classifier', self.active_classifier)


    def updateFromParamServer(self):
        nepi_msg.publishMsgInfo(self,"Updating from param server")
        self.active_classifier = nepi_ros.get_param(self,'~active_classifier',"None")
        self.publish_status()


    def updaterCb(self,timer):
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        active_models_list = nepi_aifs.getModelsActiveSortedList(models_dict)
        for model_name in self.running_classifiers_list:
            if model_name not in active_models_list:
                self.running_classifiers_list.remove(model_name)
                self.killClassifier(model_name)
        for model_name in active_models_list:
            if model_name not in self.running_classifiers_list:
                self.loadClassifier(model_name)
                self.running_classifiers_list.append(model_name)
        if self.active_classifier == "None" and not nepi_ros.is_shutdown():
            self.ros_no_clf_img.header.stamp = nepi_ros.time_now()
            self.detection_image_pub.publish(self.ros_no_clf_img)

        if self.active_classifier != "None":
            aif_class = self.aif_classes_dict[self.active_classifier]
            aif_class.enableClassifier(self.active_classifier,True)
        self.publish_status()


    def loadClassifier(self, model_name):
        if model_name != "None": 
            models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
            if not (model_name in models_dict.keys()):
                nepi_msg.publishMsgWarn(self,"Unknown classifier model requested: " + model_name)
                return
            # Start the classifier
            success = False
            try:
                aif_class = self.aif_classes_dict[model_name]
                nepi_msg.publishMsgInfo(self,"Loading classifier node " + model_name)
                model_dict = models_dict[model_name]
                [success,node_namespace] = aif_class.loadClassifier(model_dict)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to load classifier: " + model_name + " Removing from models dict list" + str(e))

            if success == True:
                 self.classifier_namespace_dict[model_name] = node_namespace
            else:
                del models_dict[model_name]
                nepi_ros.set_param(self,"~models_dict",models_dict)
            self.saveSettings() # Save config
            self.publish_status()
        
    def killClassifier(self, model_name):
        if model_name != "None": 
            models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
            if not (model_name in models_dict.keys()):
                nepi_msg.publishMsgWarn(self,"Unknown classifier model requested: " + model_name)
                return
            # Start the classifier
            aif_class = self.aif_classes_dict[model_name]
            nepi_msg.publishMsgInfo(self,"Killing classifier " + model_name)
            del self.classifier_namespace_dict[model_name]
            aif_class.killClassifier(model_name)

    def setActiveClassifierCb(self,msg):
        new_classifier = msg.data
        active_classifier = self.active_classifier
        switched_classifier = new_classifier != "None" and new_classifier != active_classifier
        if switched_classifier and not nepi_ros.is_shutdown():
            self.detection_image_pub.publish(self.ros_wait_clf_img) 
        self.enableClassifier(new_classifier)


    def enableClassifier(self,classifier_name):
        #nepi_msg.publishMsgWarn(self,"Got classifier name: " + classifier_name)
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        active_models_list = nepi_aifs.getModelsActiveSortedList(models_dict)
        active_classifier = self.active_classifier
        #nepi_msg.publishMsgWarn(self,"Current classifier list: " + str(active_models_list))
        #nepi_msg.publishMsgWarn(self,"Current classifier name: " + active_classifier)
        if active_classifier != classifier_name:
            # Disable current active classifier
            if active_classifier != "None":
                nepi_msg.publishMsgInfo(self,"Disabling classifier: " + active_classifier)
                aif_class = self.aif_classes_dict[active_classifier]
                aif_class.enableClassifier(active_classifier,False)
            # Disable other classifiers
            for model_name in active_models_list:
                if model_name != classifier_name and model_name != active_classifier:
                    #nepi_msg.publishMsgInfo(self,"Disabling classifier: " + active_classifier)
                    aif_class = self.aif_classes_dict[model_name]
                    aif_class.enableClassifier(model_name,False)
            if classifier_name != "None":
                models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
                if classifier_name in models_dict.keys():
                    nepi_msg.publishMsgInfo(self,"Activating classifier: " + classifier_name)
                    self.active_classifier = classifier_name
                    aif_class = self.aif_classes_dict[classifier_name]
                    aif_class.enableClassifier(classifier_name,True)              
                else:
                    nepi_msg.publishMsgWarn(self,"Classifier not in models dict: " + "None")
                    self.active_classifier = "None"
            else:
                nepi_msg.publishMsgWarn(self,"Setting active to classifier: " + "None")
                self.active_classifier = "None"
        self.publishClfStatus()
  
    def saveSettings(self):
        # Save framework and model dictionaries
        self.save_cfg_if.saveConfig(do_param_updates = False) # Save config after initialization for drvt time


    def enableAllFwsCb(self,msg):
        aifs_dict = nepi_ros.get_param(self,"~aifs_dict",self.init_aifs_dict)
        aifs_dict = nepi_aifs.activateAllFws(aifs_dict)
        nepi_ros.set_param(self,"~aifs_dict",aifs_dict)
        self.refreshFrameworks()
        self.saveSettings() # Save config
        self.publish_status()

    def disableAllFwsCb(self,msg):
        aifs_dict = nepi_ros.get_param(self,"~aifs_dict",self.init_aifs_dict)
        aifs_dict = nepi_aifs.disableAllFws(aifs_dict)
        nepi_ros.set_param(self,"~aifs_dict",aifs_dict)
        self.refreshFrameworks()
        self.saveSettings() # Save config
        self.publish_status()

    def updateFwStateCb(self,msg):
        nepi_msg.publishMsgInfo(self,str(msg))
        aif_name = msg.name
        new_active_state = msg.active_state
        aifs_dict = nepi_ros.get_param(self,"~aifs_dict",self.init_aifs_dict)
        if aif_name in aifs_dict.keys():
            app = aifs_dict[aif_name]
            active_state = app['active']
            if new_active_state != active_state:
                if new_active_state == True:
                    aifs_dict = nepi_aifs.activateFw(aif_name,aifs_dict)
                else:
                    aifs_dict = nepi_aifs.disableFw(aif_name,aifs_dict)
        nepi_ros.set_param(self,"~aifs_dict",aifs_dict)
        self.refreshFrameworks()
        self.saveSettings() # Save config
        self.publish_status()

    def enableAllModelsCb(self,msg):
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        models_dict = nepi_aifs.activateAllModels(models_dict)
        nepi_ros.set_param(self,"~models_dict",models_dict)
        self.saveSettings() # Save config
        self.publish_status()

    def disableAllModelsCb(self,msg):
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        models_dict = nepi_aifs.disableAllModels(models_dict)
        nepi_ros.set_param(self,"~models_dict",models_dict)
        self.saveSettings() # Save config
        self.publish_status()

    def updateModelStateCb(self,msg):
        nepi_msg.publishMsgInfo(self,str(msg))
        model_name = msg.name
        new_active_state = msg.active_state
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)
        if model_name in models_dict.keys():
            app = models_dict[model_name]
            active_state = app['active']
            if new_active_state != active_state:
                if new_active_state == True:
                    models_dict = nepi_aifs.activateModel(model_name,models_dict)
                else:
                    models_dict = nepi_aifs.disableModel(model_name,models_dict)
        nepi_ros.set_param(self,"~models_dict",models_dict)
        self.saveSettings() # Save config
        self.publish_status()

    def detectionImageCb(self,img_msg):
        data_product = 'detection_image'
        ros_timestamp = img_msg.header.stamp
        nepi_save.save_ros_img2file(self,data_product,img_msg,ros_timestamp)

    def boundingBoxesCb(self,bbs_msg):
        data_product = 'bounding_boxes'
        ros_timestamp = bbs_msg.header.stamp
        bbs_dict = dict()
        bbs_dict['timestamp'] =  nepi_ros.get_datetime_str_from_stamp(bbs_msg.header.stamp)
        bbs_dict['classifier_name'] = bbs_msg.classifier_name
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
        nepi_save.save_dict2file(self,data_product,bbs_dict,ros_timestamp)


    def publish_status(self):
        aifs_dict = nepi_ros.get_param(self,"~aifs_dict",self.init_aifs_dict)
        models_dict = nepi_ros.get_param(self,"~models_dict",self.init_models_dict)

        status_msg = AiMgrStatus()
        status_msg.ai_frameworks = nepi_aifs.getAIFsSortedList(aifs_dict)
        status_msg.active_ai_frameworks = nepi_aifs.getAIFsActiveSortedList(aifs_dict)

        status_msg.ai_models = nepi_aifs.getModelsSortedList(models_dict)
        status_msg.active_ai_models = nepi_aifs.getModelsActiveSortedList(models_dict)


        classifiers_list = []
        namespace_list = []
        for name in self.classifier_namespace_dict.keys():
            classifiers_list.append(name)
            namespace_list.append(self.classifier_namespace_dict[name])
        status_msg.ai_classifiers = classifiers_list
        status_msg.ai_classifier_namespaces = namespace_list
        status_msg.active_ai_classifier = self.active_classifier

        if not nepi_ros.is_shutdown():
            self.status_pub.publish(status_msg)

    def publishClfStatus(self):
        status_msg = ImageClassifierStatus()
        status_msg.classifier_name = "None"
        status_msg.classifier_state = "Stopped"
        #nepi_msg.publishMsgWarn(self,"Sending Status Msg: " + str(status_msg))
        if not rospy.is_shutdown():
            self.status_clf_pub.publish(status_msg)

if __name__ == '__main__':
    AIDetectorManager()
