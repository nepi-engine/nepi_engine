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

import os
import time
import copy

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_sdk import nepi_sdk


from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.messages_if import MsgIF


class ConnectMgrConfigIF:
 
    MGR_NODE_NAME = 'config_mgr'

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
        
        ##############################
        ### Wait for status to publish
        status_topic = nepi_sdk.create_namespace(self.mgr_namespace,'status')
        self.msg_if.pub_info("Waiting for status topic: " + status_topic, log_name_list = self.log_name_list)
        nepi_sdk.wait_for_topic(status_topic)
        
        #############################
        # Connect Node IF Setup


        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'namespace': self.mgr_namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = None

        ##################################
        ### Create Publishers Topic 


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'save_params': {
                'namespace': self.base_namespace,
                'topic': 'save_params',
                'msg': String,
                'qsize': None,
            },
            'reset_params': {
                'namespace': self.base_namespace,
                'topic': 'reset_params',
                'msg': String,
                'qsize': None,
            },
            'save_params_all': {
                'namespace': self.base_namespace,
                'topic': 'save_params_all',
                'msg': String,
                'qsize': None,
            },
            'factory_save': {
                'namespace': self.base_namespace,
                'topic': 'factory_save',
                'msg': Empty,
                'qsize': None,
            },
            'factory_reset': {
                'namespace': self.base_namespace,
                'topic': 'factory_reset',
                'msg': Empty,
                'qsize': None,
            },
            'factory_clear': {
                'namespace': self.base_namespace,
                'topic': 'factory_clear',
                'msg': Empty,
                'qsize': None,
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.mgr_namespace,
                'topic': 'status',
                'msg': Empty,
                'qsize': 1,
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
        nepi_sdk.wait(1)
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
        self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(.1)
            timer = nepi_sdk.get_time() - time_start
            self.msg_if.pub_info("Waiting for Config Mgr ready: " + str(self.ready), log_name_list = self.log_name_list, throttle_s = 5)
        if self.ready == False:
            self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("ready", log_name_list = self.log_name_list)
        return self.ready

    def wait_for_status(self, timeout = float('inf') ):
        self.wait_for_ready()
        self.msg_if.pub_info("Waiting for Config Mgr status connection", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
            nepi_sdk.sleep(1)
            timer = nepi_sdk.get_time() - time_start
            self.msg_if.pub_info("Waiting for Config Mgr status connection: " + str(self.status_connected), log_name_list = self.log_name_list, throttle_s = 5)
        if self.status_connected == False:
            self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return self.status_connected

    def get_status_dict(self):
        status_dict = dict()

        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        else:
            self.msg_if.pub_info("Status Listener Not connected", log_name_list = self.log_name_list)
        return status_dict

    #######################
    # Class Private Methods
    #######################

    # Update System Status
    def _statusCb(self,msg):
        #self.msg_if.pub_warn("Got Status Msg", log_name_list = self.log_name_list)
        self.status_connected = True
        self.status_msg = None

