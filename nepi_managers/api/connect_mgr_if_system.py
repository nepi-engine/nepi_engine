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


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_sdk_interfaces.msg import SystemStatus, SystemDefs, WarningFlags, StampedString, SaveDataRate
from nepi_sdk_interfaces.srv import SystemDefsQuery, SystemDefsQueryRequest, SystemDefsQueryResponse
from nepi_sdk_interfaces.srv import OpEnvironmentQuery, OpEnvironmentQueryRequest, OpEnvironmentQueryResponse                      
from nepi_sdk_interfaces.srv import SystemSoftwareStatusQuery, SystemSoftwareStatusQueryRequest, SystemSoftwareStatusQueryResponse
from nepi_sdk_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryRequest, SystemStorageFolderQueryResponse
from nepi_sdk_interfaces.srv import DebugQuery, DebugQueryRequest, DebugQueryResponse
from nepi_sdk_interfaces.srv import SystemStatusQuery, SystemStatusQueryRequest, SystemStatusQueryResponse

from nepi_api.connect_node_if import ConnectNodeServicesIF, ConnectNodeClassIF
from nepi_api.messages_if import MsgIF


MGR_NODE_NAME = 'system_mgr'


class ConnectMgrSystemServicesIF:

    ready = False
    services_connected = False
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
                
        self.mgr_namespace = os.path.join(self.base_namespace,MGR_NODE_NAME)
        

        #############################
        # Connect Node IF Setup


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'sys_storage': {
                'namespace': self.base_namespace,
                'topic': 'system_storage_folder_query',
                'srv': SystemStorageFolderQuery,
                'req': SystemStorageFolderQueryRequest(),
                'resp': SystemStorageFolderQueryResponse(),
            },
            'sys_env': {
                'namespace': self.base_namespace,
                'topic': 'op_environment_query',
                'srv': OpEnvironmentQuery,
                'req': OpEnvironmentQueryRequest(),
                'resp': OpEnvironmentQueryResponse(),
            },
            'sw_status': {
                'namespace': self.base_namespace,
                'topic': 'sw_update_status_query',
                'srv': SystemSoftwareStatusQuery,
                'req': SystemSoftwareStatusQueryRequest(),
                'resp': SystemSoftwareStatusQueryResponse(),
            },
            'sys_defs': {
                'namespace': self.base_namespace,
                'topic': 'system_defs_query',
                'srv': SystemDefsQuery,
                'req': SystemDefsQueryRequest(),
                'resp': SystemDefsQueryResponse(),
            },
            'debug_query': {
                'namespace': self.base_namespace,
                'topic': 'debug_mode_query',
                'srv': DebugQuery,
                'req': DebugQueryRequest(),
                'resp': DebugQueryResponse(),
            },
            'status_query': {
                'namespace': self.base_namespace,
                'topic': 'system_status_query',
                'srv': SystemStatusQuery,
                'req': SystemStatusQueryRequest(),
                'resp': SystemStatusQueryResponse(),
            }
        }


        # Create Services Class ####################

        self.services_if = ConnectNodeServicesIF(
                        services_dict = self.SRVS_DICT,
                        msg_if = self.msg_if
        )

        #ready = self.services_if.wait_for_ready()
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

    def wait_for_services(self, timeout = float('inf') ):
        self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
        timer = 0
        time_start = nepi_sdk.get_time()
        connected = False
        while connected == False and timer < timeout and not nepi_sdk.is_shutdown():
            exists = nepi_sdk.check_for_service('system_status_query')
            nepi_sdk.wait()
            if exists == True:
                ret = self.get_system_status_dict(verbose = False)
                connected = (ret is not None)
            timer = nepi_sdk.get_time() - time_start
        if connected == False:
            self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Services Connected", log_name_list = self.log_name_list)
        return connected
    

    def get_sys_folder_path(self, folder_name, fallback_path = ""):
        service_name = 'sys_storage'
        folder_path = fallback_path
        response = None
        try:
            request = self.services_if.create_request_msg(service_name)
            request.type = folder_name
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))
        try:
            response = self.services_if.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))

        # Process Response
        if response is None or response == "":
            self.msg_if.pub_warn("Returning fallback path: " + fallback_path)
        else:
            folder_path = response.folder_path
            self.msg_if.pub_info("Got folder path: " + folder_path + " for folder request: " + folder_name)

        return folder_path
        

    def get_op_env_str(self):
        service_name = 'sys_env'
        env_str = None
        response = None
        try:
            request = self.services_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))
        try:
            response = self.services_if.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            env_str = response.op_env
            self.msg_if.pub_info("Got system env response " + str(response) + " for service: " + service_name)

        return env_str

    def get_software_status_dict(self):
        service_name = 'sw_status'
        status_dict = None

        response = None
        try:
            request = self.services_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))
        try:
            response = self.services_if.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            status_dict = nepi_sdk.convert_msg2dict(response)
            #self.msg_if.pub_info("Got status response" + str(response) + " for service: " + service_name)
            self.msg_if.pub_info("Got system software response " + str(status_dict) + " for service: " + service_name)

        return status_dict


    def get_system_stats_dict(self):
        service_name = 'sys_defs'
        stats_dict = None
        response = None
        try:
            request = self.services_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))
        try:
            response = self.services_if.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
            return states_dict
        try:
            stats_dict = nepi_sdk.convert_msg2dict(response)['defs']
            #self.msg_if.pub_info("Got status response" + str(response) + " for service: " + service_name)
            self.msg_if.pub_info("Got system stats dict " + str(stats_dict) + " for service: " + service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert service response to dict: " + service_name + " : " + str(response) + " : " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)

        return stats_dict


    def get_system_debug_mode(self):
        service_name = 'debug_query'
        debug_mode = None
        response = None
        try:
            request = self.services_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))
        try:
            response = self.services_if.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            debug_mode = response.debug_enabled
            #self.msg_if.pub_info("Got status response" + str(response) + " for service: " + service_name)
            self.msg_if.pub_info("Got system debug mode " + str(debug_mode) + " for service: " + service_name)

        return debug_mode

    def get_system_status_dict(self, verbose = True):
        service_name = 'status_query'
        status_dict = None
        response = None
        try:
            request = self.services_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))
        try:
            response = self.services_if.call_service(service_name, request, verbose = verbose)
        except Exception as e:
            self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))
        # Process Response
        if response is None:
            if verbose == True:
                self.msg_if.pub_warn("Failed to get response for service: " + service_name)
            else:
                pass
        else:
            status_dict = nepi_sdk.convert_msg2dict(response)['system_status']
            #self.msg_if.pub_info("Got status response " + str(response) + " for service: " + service_name)
            #self.msg_if.pub_info("Got system status dict " + str(status_dict) + " for service: " + service_name)

        return status_dict


    #######################
    # Class Private Methods
    #######################





class ConnectMgrSystemIF:
 
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
                
        self.mgr_namespace = os.path.join(self.base_namespace,MGR_NODE_NAME)
        

        #############################
        # Connect Node IF Setup


        # Configs Config Dict ####################

        # Publishers Config Dict ####################
        self.PUBS_DICT = None
        '''  #  Need to add publishers
        self.save_data_pub = rospy.Publisher(self._get_ns('save_data'), SaveData, queue_size=10)
        self.clear_data_folder_pub = rospy.Publisher(self._get_ns('clear_data_folder'), Empty, queue_size=10)
        self.set_op_environment_pub = rospy.Publisher(self._get_ns('set_op_environment'), String, queue_size=10)
        self.set_device_id_pub = rospy.Publisher(self._get_ns('set_device_id'), String, queue_size=10)
        self.submit_system_error_msg_pub = rospy.Publisher(self._get_ns('submit_system_error_msg'), String, queue_size=10)
        self.install_new_image_pub = rospy.Publisher(self._get_ns('install_new_image'), String, queue_size=10)
        self.switch_active_inactive_rootfs_pub = rospy.Publisher(self._get_ns('switch_active_inactive_rootfs'), Empty, queue_size=10)
        self.archive_inactive_rootfs_pub = rospy.Publisher(self._get_ns('archive_inactive_rootfs'), Empty, queue_size=10)
        self.save_data_prefix_pub = rospy.Publisher(self._get_ns('save_data_prefix'), String, queue_size=10)

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'pub_name': {
                'namespace': self.base_namespace,
                'topic': 'set_empty',
                'msg': EmptyMsg,
                'qsize': 1,
                'latch': False
            }
        }
        '''


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.base_namespace,
                'topic': 'system_status',
                'msg': SystemStatus,
                'qsize': 10,
                'callback': self._statusCb, 
                'callback_args': ()
            }
        }



        # Create Node Class ####################

        self.node_if = ConnectNodeClassIF(
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )


        self.services_if = ConnectMgrSystemServicesIF()

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

    def wait_for_services(self, timeout = float('inf') ):
        self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
        services_connected = self.services_if.wait_for_services_path(timeout = timeout)
        return services_connected

    def get_sys_folder_path(self, folder_name, fallback_path = ""):
        return self.services_if.get_sys_folder_path(folder_name, fallback_path = fallback_path)
        

    def get_op_env_str(self):
        return self.services_if.get_op_env_str()

    def get_software_status_dict(self):
        return self.services_if.get_software_status_dict()


    def get_system_stats_dict(self):
        return self.services_if.get_system_stats_dict()

    def get_system_debug_mode(self):
        return self.services_if.get_system_debug_mode()

    def get_system_status_dict(self):
        return self.services_if.get_system_status_dict()


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

    def _get_ns(self,topic_str):
        return os.path.join(self.base_namespace,topic_str)

    # Update System Status
    def _statusCb(self,msg):
        #self.msg_if.pub_warn("Got System Status Msg", log_name_list = self.log_name_list)
        self.status_msg = msg
        self.status_connected = True


