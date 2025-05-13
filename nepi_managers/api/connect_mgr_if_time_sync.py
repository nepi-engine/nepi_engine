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

from nepi_ros_interfaces.msg import TimeStatus, TimeUpdate
from nepi_ros_interfaces.srv import TimeStatusQuery, TimeStatusQueryRequest, TimeStatusQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils


from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.messages_if import MsgIF


class ConnectMgrTimeSyncIF:
 
    MGR_NODE_NAME = 'time_sync_mgr'

    ready = False

    status_msg = None

    #######################
    ### IF Initialization
    def __init__(self, time_updated_callback = None, timeout = float('inf')):
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
        self.time_updated_callback = time_updated_callback
        self.mgr_namespace = os.path.join(self.base_namespace,self.MGR_NODE_NAME)
        

        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'namespace': self.mgr_namespace
        }

        # Services Config Dict ####################     
        self.SRVS_DICT = {
            'time_status_query': {
                'namespace': self.base_namespace,
                'topic': 'time_status_query',
                'srv': TimeStatusQuery,
                'req': TimeStatusQueryRequest(),
                'resp': TimeStatusQueryResponse()
            }
        }


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'add_ntp_server': {
                'namespace': self.base_namespace,
                'topic': 'add_ntp_server',
                'msg': String,
                'latch': False,
                'qsize': None
            },
            'remove_ntp_server': {
                'namespace': self.base_namespace,
                'topic': 'remove_ntp_server',
                'msg': String,
                'latch': False,
                'qsize': None
            },
            'set_time': {
                'namespace': self.base_namespace,
                'topic': 'set_time',
                'msg': TimeUpdate,
                'latch': False,
                'qsize': 10
            }
        }



        # Subscribers Config Dict ####################
        if self.time_updated_callback is not None:
            self.SUBS_DICT = {
                'sys_time_updated': {
                    'namespace': self.base_namespace,
                    'topic': 'sys_time_updated',
                    'msg': Empty,
                    'qsize': 3,
                    'callback': self.time_updated_callback, 
                    'callback_args': ()
                }
            }
        else:
            self.SUBS_DICT = None



        # Create Node Class ####################

        self.node_if = ConnectNodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        ready = self.node_if.wait_for_ready()

        ################################
        # Complete Initialization

        # Wait for Service Message
        nepi_ros.sleep(1)
        self.msg_if.pub_info("Waiting for status message")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.status_msg is None and timer < timeout and not nepi_ros.is_shutdown():
            self.get_time_status()
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

    def get_time_status(self):
        service_name = 'time_status_query'
        response = None
        time_dict = None
        try:
            request = self.node_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + " " + str(e))
        try:
            response = self.node_if.call_service(service_name, request)
            self.status_msg = response.time_status
            time_status = response.time_status
            time_dict = nepi_ros.convert_msg2dict(time_status)
            #self.msg_if.pub_info("Got time status dict: " + time_dict)
        except Exception as e:
            self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))

        return time_dict    

    #######################
    # Class Private Methods
    #######################

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg

