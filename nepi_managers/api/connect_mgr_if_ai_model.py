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
import copy

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header
from sensor_msgs.msg import Image
from nepi_interfaces.msg import MgrAiModelStatus
from nepi_interfaces.srv import AiModelsInfoQuery, AiModelsInfoQueryRequest, AiModelsInfoQueryResponse
from nepi_interfaces.srv import AiDetectorInfoQuery, AiDetectorInfoQueryRequest, AiDetectorInfoQueryResponse

from nepi_sdk import nepi_sdk
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
    status_connected = False

    #######################
    ### IF Initialization
    def __init__(self, timeout = float('inf'),
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
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)


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
                'srv': AiModelsInfoQuery,
                'req': AiModelsInfoQueryRequest(),
                'resp': AiModelsInfoQueryResponse(),
            }
        }


        self.PUBS_DICT = None


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.mgr_namespace,
                'topic': 'status',
                'msg': MgrAiModelStatus,
                'qsize': 10,
                'callback': self._statusCb, 
                'callback_args': ()
            }
        }


        # Create Node Class ####################

        self.node_if = ConnectNodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        #ready = self.node_if.wait_for_ready()
        nepi_sdk.wait()
        ################################
        # Complete Initialization


        #################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        

    #######################
    # Class Public Methods
    #######################
    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("ready", log_name_list = self.log_name_list)
        return self.ready

    def wait_for_status(self, timeout = float('inf') ):
        self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
        if self.status_connected == False:
            self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return self.status_connected

    def get_status_dict(self):
        status_dict = None

        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        else:
            self.msg_if.pub_info("Status Listener Not connected", log_name_list = self.log_name_list)
        return status_dict


    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        else:
            self.msg_if.pub_warn("Status Listener Not ready", log_name_list = self.log_name_list)
        return status_dict

    def get_active_models_info_dict(self):
        service_name = 'active_models_query'
        models_info_dict = None

        # Create service request
        request = self.node_if.create_request_msg(service_name)
        # Call service
        response = None
        if request is not None:
            response = self.node_if.call_service(service_name, request)

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            models_info_dict = nepi_sdk.convert_msg2dict(response)
            #self.msg_if.pub_info("Got models response" + str(response) + " for service: " + service_name)
            #self.msg_if.pub_info("Got models info response" + str(models_info_dict) + " for service: " + service_name)

        return models_info_dict


    #######################
    # Class Private Methods
    #######################

    # Update System Status
    def _statusCb(self,msg):
        self.status_connected = True
        self.status_msg = msg


    def getActiveModelsInfo(self):
        if self.ai_models_info_query_service is not None:
            req = AiModelsInfoQueryRequest()
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


