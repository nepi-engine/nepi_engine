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
import time

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import AiModelMgrStatus
from nepi_ros_interfaces.srv import AiMgrActiveModelsInfoQuery, AiMgrActiveModelsInfoQueryRequest, AiMgrActiveModelsInfoQueryResponse
from nepi_ros_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img


from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.messages_if import MsgIF

MODEL_TYPE_LIST = ['detection','segmentation','pose']

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

    MGR_NODE_NAME = 'ai_model_mgr'

    ready = False

    status_msg = None

    #######################
    ### IF Initialization
    def __init__(self, timeout = float('inf')):
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
        # Initialize Class Variables
                
        self.mgr_namespace = os.path.join(self.base_namespace,self.MGR_NODE_NAME)
        

        #############################
        # Connect Node IF Setup


        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'namespace': self.mgr_namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'active_models_query': {
                'namespace': self.mgr_namespace,
                'topic': 'active_models_info_query',
                'srv': AiMgrActiveModelsInfoQuery,
                'req': AiMgrActiveModelsInfoQueryRequest(),
                'resp': AiMgrActiveModelsInfoQueryResponse(),
            }
        }


        self.PUBS_DICT = None
        '''
        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'pub_name': {
                'namespace': self.mgr_namespace,
                'topic': 'set_empty',
                'msg': EmptyMsg,
                'qsize': 1,
                'latch': False
            }
        }
        '''

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'sub_name': {
                'namespace': self.mgr_namespace,
                'topic': 'status',
                'msg': AiModelMgrStatus,
                'qsize': 10,
                'callback': self._statusCb, 
                'callback_args': ()
            }
        }


        # Create Node Class ####################

        self.NODE_IF = ConnectNodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        ################################
        # Complete Initialization

        # Wait for Service Message
        nepi_ros.sleep(1)
        self.msg_if.pub_info("Waiting for status message")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.status_msg is None and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.2)
            timer = nepi_ros.get_time() - time_start
        if self.status_msg is None:
            self.msg_if.pub_warn("Status msg topic subscribe timed out " + str(status_topic))
        else:
            #self.msg_if.pub_warn("Got status msg " + str(self.status_msg))
            pass


        #################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        

    #######################
    # Class Public Methods
    #######################
    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Failed to Connect")
        else:
            self.msg_if.pub_info("ready")
        return self.ready

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        else:
            self.msg_if.pub_warn("Status Listener Not ready")
        return status_dict

    def get_active_models_info_dict(self):
        service_name = 'active_models_query'
        models_info_dict = None

        # Create service request
        request = self.NODE_IF.create_request_msg(service_name)
        # Call service
        response = None
        if request is not None:
            response = self.NODE_IF.call_service(service_name, request)

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            models_info_dict = nepi_ros.convert_msg2dict(response)
            #self.msg_if.pub_info("Got models response" + str(response) + " for service: " + service_name)
            #self.msg_if.pub_info("Got models info response" + str(models_info_dict) + " for service: " + service_name)

        return models_info_dict


    #######################
    # Class Private Methods
    #######################

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg


    def getActiveModelsInfo(self):
        if self.ai_models_info_query_service is not None:
            req = AiMgrActiveModelsInfoQueryRequest()
            try:
                response = self.ai_models_info_query_service(req)
                self.ai_models_info_response = response
            except Exception as e:
                self.msg_if.pub_warn("Failed to obtain active models info response " + str(e))
            if response is not None:
                if "detector_name_list" in response.keys(detector_name_list):
                    det_namespaces = response["detector_name_list"]
                    for i, namespace in enumerate(det_namesapces):
                        self.ai_detectors_info_dict[namespace] = response["detector_info_list"][i]


