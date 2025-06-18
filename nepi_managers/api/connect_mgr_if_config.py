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

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.srv import ParamsReset, ParamsResetRequest, ParamsResetResponse

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils


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
        

        #############################
        # Connect Node IF Setup


        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'namespace': self.mgr_namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'user_reset': {
                'namespace': self.mgr_namespace,
                'topic': 'user_reset',
                'srv': ParamsReset,
                'req': ParamsResetRequest(),
                'resp': ParamsResetResponse(),
            },
            'factory_reset': {
                'namespace': self.mgr_namespace,
                'topic': 'factory_reset',
                'srv': ParamsReset,
                'req': ParamsResetRequest(),
                'resp': ParamsResetResponse(),
            }
        }


        ##################################
        ### Create Publishers Topic 


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'save_all_config': {
                'namespace': self.base_namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': 10,
                'latch': False
            },
            'store_params': {
                'namespace': self.base_namespace,
                'topic': 'store_params',
                'msg': String,
                'qsize': 10,
                'latch': False
            },
            'full_user_reset': {
                'namespace': self.base_namespace,
                'topic': 'full_user_restore',
                'msg': Empty,
                'qsize': 10,
                'latch': False
            },
            'full_factory_reset': {
                'namespace': self.base_namespace,
                'topic': 'factory_restore',
                'msg': Empty,
                'qsize': 10,
                'latch': False
            },
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.mgr_namespace,
                'topic': 'status',
                'msg': Empty,
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

    #######################
    # Class Private Methods
    #######################

    # Update System Status
    def _statusCb(self,msg):
        self.status_connected = True
        self.status_msg = msg

