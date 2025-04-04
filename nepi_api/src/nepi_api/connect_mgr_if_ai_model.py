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
from nepi_ros_interfaces.msg import AiModelMgrStatus
from nepi_ros_interfaces.srv import AiMgrActiveModelsInfoQuery, AiMgrActiveModelsInfoQueryRequest
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest

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

class ConnectMgrAiModelIF:

    # AI Mgr Vars
    AI_MGR_NODE_NAME = 'ai_model_mgr'
    MODEL_TYPE_LIST = ['detection']

    connected = False

    status_topic_name = 'status'
    status_msg = None
    status_connected = False

    services_dict = {
        'active_models_info_query': {
            'connected': False,
            'msg': AiMgrActiveModelsInfoQuery,
            'req': AiMgrActiveModelsInfoQueryRequest(),
            'psn': None,
            'service': None
        }
    }

    #######################
    ### IF Initialization
    log_name = "AiMgrIf"
    def __init__(self, timeout = float('inf')):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting AI Manager IF Initialization Processes")
        ##############################  
                
        self.mgr_namespace = os.path.join(self.base_namespace,self.NODE_NAME)
        
        status_topic = os.path.join(self.mgr_namespace,self.status_topic_name)
        status_sub = rospy.Subscriber(status_topic, AiModelMgrStatus, self._statusCb, queue_size=10)

        #######################
        # Wait for Status Message
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for status message")
        timer = 0
        time_start = nepi_ros.get_time()
        status_msg = None
        while status_msg is None and timer < timeout and not rospy.is_shutdown():
            nepi_ros.sleep(.2)
            status_msg = self.status_msg
            timer = nepi_ros.get_time() - time_start
        if self.status_msg is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Status msg topic subscribe timed out " + str(status_topic))
            success = False
        else:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got status msg " + str(self.status_msg))

        ##################################
        ### Set up Services
        for service_name in self.services_dict.keys():
            service_dict = self.services_dict[service_name]
            name = 'detector_info_query'
            if service_dict['psn'] is None: 
                service_namespace = os.path.join(self.base_namespace, service_name)
            else:
                service_namespace = os.path.join(self.base_namespace,service_dict['psn'], service_name)
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for " + service_name + " on namespace " + service_namespace)
            ret = nepi_ros.wait_for_service(service_namespace, timeout = float('inf') )
            if ret == "":
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Wait for service: " + service_name + " timed out") 
            else:
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Creating service call for: " + service_name)
                #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Creating service with namespace: " + service_namespace)
                #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Creating service with msg: " + str(service_dict['msg']))
                service = None
                try:
                    service = nepi_ros.create_service(service_namespace, service_dict['msg'])
                    time.sleep(1)
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to get service connection: " + service_name + " " + str(e))  
                if service is not None:
                    self.connected = True
                    self.services_dict[service_name]['service'] = service
                    self.services_dict[service_name]['connected'] = True

        #################################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": AI Manager IF Initialization Complete")
        

    #######################
    # Class Public Methods
    #######################

    def wait_for_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Connected")
        return self.connected    

    def get_status_dict(self):
        status_dict = None
        if status_sub is not None:
            if self.status_msg is not None:
                status_dict = nepi_ros.convert_msg2dict(self.status_msg)
            else:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Status Listener Not connected")
        else:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Manager Not connected")
        return status_dict


    #######################
    # Class Public Service Methods
    #######################

    def get_active_models_info_dict(self):
        service_name = 'active_models_info_query'
        srv_dict = self.services_dict[service_name]
        models_info_dict = None

        if 'service' in srv_dict.keys():
            # Create service request
            request = None
            try:
                    request = srv_dict['req']
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to create service request: " + service_name + " " + str(e))
                
            # Call service
            response = None
            if request is not None:
                response = nepi_ros.call_service(srv_dict['service'],  request)

            # Process Response
            if response is None:
                nepi_msg.publishMsgWarn(self,"Failed to get response for service: " + service_name)
            else:
                models_info_dict = nepi_ros.convert_msg2dict(response)
                #nepi_msg.publishMsgInfo(self,"Got software status response" + str(response) + " for service: " + service_name)
                nepi_msg.publishMsgInfo(self,"Got models info response" + str(status_dict) + " for service: " + models_info_dict)

        return models_info_dict


    def get_active_detector_namespaces(self):
        model_key = 'detector_namespaces'
        service_name = 'detector_info_query'
        request = AiDetectorInfoQueryRequest()
        namespaces_key = 'image_detector_namespaces'
        states_key = image_detector_states

        # Get available detector namespaces
        namespaces = []
        models_info_dict = self.get_active_models_info_dict()
        for namespace in models_info_dict[model_key]:
            # Create Service
            service_namespace = os.path.join(namespace,service_name)
            service = None
            try:
                service = nepi_ros.create_service(service_namespace, service_dict['msg'])
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to get service connection: " + service_name + " " + str(e)) 

            # Call Service
            response = None
            if service is not None:
                response = nepi_ros.call_service(service_name, service, request)

            # Get namespaces
            if response is not None:
                info_dict = nepi_ros.convert_msg2dict(response)

                if namespaces_key in info_dict.keys():
                    got_namespaces = info_dict[namespaces_key]
                    got_states = info_dict[states_key]
                    if got_namespaces is not None:
                        active_namespaces = []
                        for i, state in enumerate(got_states):
                            if state == True:
                                active_namespaces.append(got_namespace[i])
                        namespaces += active_namespaces
        nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got active detector namespaces: " + str(namespaces))
        return namespaces

    #######################
    # Class Private Methods
    #######################

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg
        self.status_connected = True


    def getActiveModelsInfo(self):
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


