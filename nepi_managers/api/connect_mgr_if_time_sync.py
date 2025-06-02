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
    services_connected = False

    #######################
    ### IF Initialization
    def __init__(self, time_updated_callback = None, timeout = float('inf'),
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

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
                    'callback': self._time_updated_callback, 
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
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        #ready = self.node_if.wait_for_ready()
        nepi_ros.wait()

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
        time_start = nepi_ros.get_time()
        while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.ready == False:
            self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("ready", log_name_list = self.log_name_list)
        return self.ready

    def wait_for_services(self, timeout = float('inf') ):
        self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_ros.get_time()
        connected = False
        while connected == False and timer < timeout and not nepi_ros.is_shutdown():
            exists = nepi_ros.check_for_service('time_status_query')
            nepi_ros.wait()
            if exists == True:
                ret = None
                ret = self.get_time_status(verbose = True)
                connected = (ret is not None)
            timer = nepi_ros.get_time() - time_start
        if connected == False:
            self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return connected



    def get_time_status(self, verbose = True):
        service_name = 'time_status_query'
        response = None
        time_dict = None
        try:
            request = self.node_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + " " + str(e))
        try:
            response = self.node_if.call_service(service_name, request, verbose = verbose)
            #self.msg_if.pub_info("Got time status response: " + str(response))
        except Exception as e:
            if verbose == True:
                self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))
            else:
                pass
        if response is not None:
            self.status_msg = response.time_status
            time_status = response.time_status
            time_dict = nepi_ros.convert_msg2dict(time_status)
            #self.msg_if.pub_info("Got time status dict: " + str(time_dict))
        return time_dict    

    #######################
    # Class Private Methods
    #######################

    def _time_updated_callback(self,msg):
        pass