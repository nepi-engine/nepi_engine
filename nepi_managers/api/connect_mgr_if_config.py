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

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.srv import FileReset, FileResetRequest, FileResetResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils


from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.messages_if import MsgIF


class ConnectMgrConfigIF:
 
    MGR_NODE_NAME = 'config_mgr'

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
            'user_reset': {
                'namespace': self.mgr_namespace,
                'topic': 'user_reset',
                'srv': FileReset,
                'req': FileResetRequest(),
                'resp': FileResetResponse(),
            },
            'factory_reset': {
                'namespace': self.mgr_namespace,
                'topic': 'factory_reset',
                'srv': FileReset,
                'req': FileResetRequest(),
                'resp': FileResetResponse(),
            }
        }


        ##################################
        ### Create Publishers Topic 


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'save_config': {
                'namespace': self.mgr_namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': 10,
                'latch': False
            },
            'store_params': {
                'namespace': self.mgr_namespace,
                'topic': 'store_params',
                'msg': String,
                'qsize': 10,
                'latch': False
            },
            'full_user_restore': {
                'namespace': self.mgr_namespace,
                'topic': 'full_user_restore',
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

        self.NODE_IF = ConnectNodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        ready = self.NODE_IF.wait_for_ready()

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
            self.msg_if.pub_warn("Status msg topic subscribe timed out")
            success = False
        else:
            pass
            #self.msg_if.pub_warn("Got status msg " + str(self.status_msg))


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



    #######################
    # Class Private Methods
    #######################

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg

