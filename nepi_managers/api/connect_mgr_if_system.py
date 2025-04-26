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


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.msg import SystemStatus, SystemDefs, WarningFlags, StampedString, SaveDataRate
from nepi_ros_interfaces.srv import SystemDefsQuery, SystemDefsQueryRequest, OpEnvironmentQuery, OpEnvironmentQueryRequest, \
                             SystemSoftwareStatusQuery, SystemSoftwareStatusQueryRequest, SystemStorageFolderQuery, SystemStorageFolderQueryRequest



from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.messages_if import MsgIF

class ConnectMgrSystemIF:
 
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
        self.msg_if = MsgIF(log_name = self.class_name + ": " + img_name)
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
            'sys_storage': {
                'namespace': self.base_namespace,
                'topic': 'system_storage_folder_query',
                'svr': SystemStorageFolderQuery,
                'req': SystemStorageFolderQueryRequest(),
                'resp': SystemStorageFolderQueryResponse(),
            },
            'sys_env': {
                'namespace': self.base_namespace,
                'topic': 'op_environment_query',
                'svr': OpEnvironmentQuery,
                'req': OpEnvironmentQueryRequest(),
                'resp': OpEnvironmentQueryResponse(),
            },
            'sw_status': {
                'namespace': self.base_namespace,
                'topic': 'sw_update_status_query',
                'svr': SystemSoftwareStatusQueryQuery,
                'req': SystemSoftwareStatusQueryRequest(),
                'resp': SystemSoftwareStatusQueryResponse(),
            },
            'sys_defs': {
                'namespace': self.base_namespace,
                'topic': 'system_defs_query',
                'svr': SystemDefsQuery,
                'req': SystemDefsQueryRequest(),
                'resp': SystemDefsQueryResponse(),
            }
        }



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
                'topic': 'system_status',
                'msg': SystemStatus,
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

        # Wait for Status Message
        nepi_ros.sleep(1)
        self.msg_if.pub_info("Waiting for status message")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.status_msg is None and timer < timeout and not rospy.is_shutdown():
            nepi_ros.sleep(.2)
            timer = nepi_ros.get_time() - time_start
        if self.status_msg is None:
            self.msg_if.pub_warn("Status msg topic subscribe timed out " + str(status_topic))
            success = False
        else:
            self.msg_if.pub_warn("Got status msg " + str(self.status_msg))


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


    def get_sys_folder_path(self,folder_path, fallback_path = ""):
        service_name = 'sys_storage'
        folder_path = ""
        request = None
        try:
            request = self.NODE_IF.create_request_msg(service_name)
            request.folder_path = folder_path
            response = self.NODE_IF.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))

        # Process Response
        if response is None or response == "":
            folder_path = fallback_path
            self.msg_if.pub_warn("Returning fallback path: " + fallback_path)
        else:
            folder_path = response.folder_path
            self.msg_if.pub_info("Got folder path: " + folder_path + " for folder request: " + folder_name)

        return folder_path
        

    def get_op_env_str(self):
        service_name = 'sys_env'
        env_str = None

        request = None
        try:
            request = self.NODE_IF.create_request_msg(service_name)
            request.folder_path = folder_path
            response = self.NODE_IF.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            env_str = response.op_env
            self.msg_if.pub_info("Got status response" + str(response) + " for service: " + service_name)

        return env_str

    def get_software_status_dict(self):
        service_name = 'sw_status'
        status_dict = None

        request = None
        try:
            request = self.NODE_IF.create_request_msg(service_name)
            request.folder_path = folder_path
            response = self.NODE_IF.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            status_dict = nepi_ros.convert_msg2dict(response)
            #self.msg_if.pub_info("Got status response" + str(response) + " for service: " + service_name)
            self.msg_if.pub_info("Got system status response" + str(status_dict) + " for service: " + service_name)

        return status_dict


    def get_system_stats_dict(self):
        service_name = 'sys_defs'
        stats_dict = None
        request = None
        try:
            request = self.NODE_IF.create_request_msg(service_name)
            request.folder_path = folder_path
            response = self.NODE_IF.call_service(service_name, request)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            stats_dict = nepi_ros.convert_msg2dict(response)['defs']
            #self.msg_if.pub_info("Got status response" + str(response) + " for service: " + service_name)
            self.msg_if.pub_info("Got stats dict" + str(stats_dict) + " for service: " + service_name)

        return stats_dict



    def wait_for_status(self,timeout = float('inf')):
        self.msg_if.pub_warn("Waiting for system to connect")
        success = self.wait_for_connection(timeout)
        if success == False:
            self.msg_if.pub_info("Manager Not connected")
        else:
            self.msg_if.pub_warn("Waiting for status topic")
            found_topic = nepi_ros.wait_for_topic(self.status_topic, timeout)
            if found_topic == "":
                self.msg_if.pub_info("Failed to get status topic")
            else:
                self.msg_if.pub_warn("Got status topic")
                success = True
        return success



    def get_status_dict(self):
        status_dict = None
        if self.status_sub is not None:
            if self.status_msg is not None:
                status_dict = nepi_ros.convert_msg2dict(self.status_msg)
            else:
                self.msg_if.pub_info("Status Listener Not connected")
        else:
            self.msg_if.pub_info("Manager Not connected")
        return status_dict


    #######################
    # Class Private Methods
    #######################

    def _get_ns(self,topic_str):
        return os.path.join(self.base_namespace,topic_str)

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg
        self.status_connected = True


