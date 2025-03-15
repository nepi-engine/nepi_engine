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
import time

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image
from nepi_ros_interfaces.srv import AiMgrActiveModelsInfoQuery, AiMgrActiveModelsInfoQueryResponse
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img

EXAMPLE_AI_MGR_STATUS_DICT = {
    'ai_frameworks': [],

    'ai_models': [],
    'ai_models_frameworks': [],
    'ai_models_types': [],

    'active_ai_framework': 'framework_name',

    'active_ai_models': [],
    'active_ai_models_frameworks': [],
    'active_ai_models_types': [],
    'active_ai_models_nodes': [],
    'active_ai_models_namespaces': [],

    'all_namespace': '/base_namespace/ai/all'
}

#You must use the classes img_dict_lock.aquire() and img_dict_lock.release() thread save functions 
#When accessing the classes img_dict

class AiManagerIF:

    # AI Mgr Vars
    AI_MGR_NODE_NAME = 'ai_model_mgr'
    MODEL_TYPE_LIST = ['detection']

    ai_mgr_connected = False
    ai_mgr_status_msg = None
    ai_mgr_status_dict = dict()
    ai_models_info_query_service = None
    ai_models_info_response = None
    ai_models_info_dict = dict()
    ai_detectors_info_dict = dict()


    #######################
    ### IF Initialization
    log_name = "AiMgrIf"
    def __init__(self):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting AI Manager IF Initialization Processes")
        ##############################  
                
        #################################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": AI Manager IF Initialization Complete")
        


    #######################
    # AI Model Manager IF Functions


    def init_ai_mgr_connection(self, timeout = 5):
        self.ai_mgr_namespace = AI_MGR_NODE_NAME
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Initialization AI Manager Processes")
        mgr_namespace = os.path.join(self.base_namespace,self.ai_mgr_namespace)

        # Subscribe to status topic
        status_topic = os.path.join(mgr_namespace,"status")
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for AI Model Mgr status topic: " + status_topic)
        nepi_ros.wait_for_topic(status_topic, timeout )
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting AI Model Mgr status")
        self.ai_mgr_status_msg = None
        rospy.Subscriber(status_topic, AiModelMgrStatus, self.ai_mgr_status_callback, queue_size=None)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for AI Model Mgr status message")
        while self.ai_mgr_status_msg is None and not rospy.is_shutdown():
            nepi_ros.sleep(1)

        # Create Model Info Service Listener
        get_info_service = os.path.join(det_namespace,'active_models_info_query')
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for AI Model Mgr active models info query service " + get_info_service)
        rospy.wait_for_service(get_info_service, timeout )
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Calling AI Model Mgr active models info query service " + get_info_service)
        try:
            self.ai_models_info_query_service = rospy.ServiceProxy(get_info_service, AiDetectorInfoQuery)
            time.sleep(1)
            success = True
        except Exception as e:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to obtain AI Model Mgr active models info query service " + str(e))
            success = False
        if success == True:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Calling AI Model Mgr active models info query service " + get_info_service)
            req = AiMgrActiveModelsInfoQueryRequest()
            try:
                response = self.ai_models_info_query_service(req)
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Got AI Model Mgr active models info response" + response)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to obtain AI Model Mgr active models info response " + str(e))
                return False
        rospy.Timer(rospy.Duration(1), self.aiMgrModelInfoServiceCb, oneshot = True)

        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": AI Model Mgr Initialization Complate")
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
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to obtain active models info response " + str(e))
            if response is not None:
                if "detector_name_list" in response.keys(detector_name_list):
                    det_namespaces = response["detector_name_list"]
                    for i, namespace in enumerate(det_namesapces):
                        self.ai_detectors_info_dict[namespace] = response["detector_info_list"][i]
        rospy.Timer(rospy.Duration(5), self.aiMgrModelInfoServiceCb, oneshot = True)

