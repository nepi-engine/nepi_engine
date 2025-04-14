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
import rospy
import time

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.srv import FileReset, FileResetRequest

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils


class ConnectMgrConfigIF:
 
    NODE_NAME = 'config_mgr'
    PUB_SUB_NAME = ''

    connected = False

    status_topic_name = 'status'
    status_msg_type = Empty
    status_sub = None
    status_connected = False


    services_dict = {
        'factory_reset': {
            'connected': False,
            'msg': FileReset,
            'req': FileResetRequest,
            'psn': "",
            'service': None
        },

        'user_reset': {
            'connected': False,
            'msg': FileReset,
            'req': FileResetRequest,
            'psn': "",
            'service': None
        }
    }



    #######################
    ### IF Initialization
    log_name = "MgrSystemIF"

    def __init__(self):
        #################################
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        ##############################

        self.mgr_namespace = os.path.join(self.base_namespace,self.NODE_NAME)

        self.pub_sub_namespace = os.path.join(self.base_namespace,self.PUB_SUB_NAME)
        self.status_topic = os.path.join(self.mgr_namespace,self.status_topic_name)


        ##################################
        ### Create Publishers Topic 

        self.save_config_pub = rospy.Publisher(self.pub_sub_namespace + 'save_config', Empty, queue_size=10)
        self.store_params_pub = rospy.Publisher(self.pub_sub_namespace + 'store_params', String, queue_size=10)
        self.archive_inactive_rootfs_pub = rospy.Publisher(self.pub_sub_namespace + 'full_user_restore', Empty, queue_size=10)


        time.sleep(1)

        ##################################
        ### Set up Services
        for service_name in self.services_dict.keys():
            service_dict = self.services_dict[service_name]
            name = 'detector_info_query'
            if service_dict['psn'] is None: 
                service_namespace = os.path.join(self.base_namespace, service_name)
            else:
                service_namespace = os.path.join(self.base_namespace,service_dict['psn'], service_name)
            self.msg_if.pub_info("Waiting for " + service_name + " on namespace " + service_namespace)
            ret = nepi_ros.wait_for_service(service_namespace, timeout = float('inf') )
            if ret == "":
                self.msg_if.pub_warn("Wait for service: " + service_name + " timed out") 
            else:
                self.msg_if.pub_info("Creating service call for: " + service_name)
                #self.msg_if.pub_warn("Creating service with namespace: " + service_namespace)
                #self.msg_if.pub_warn("Creating service with msg: " + str(service_dict['msg']))
                service = None
                try:
                    service = nepi_ros.create_service(service_namespace, service_dict['msg'])
                    time.sleep(1)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to get service connection: " + service_name + " " + str(e))  
                if service is not None:
                    self.services_dict[service_name]['service'] = service
                    self.services_dict[service_name]['connected'] = True


        #################################
        self.connected = True
        self.msg_if.pub_info("IF Initialization Complete")
        #################################



    #######################
    # Class Public Sub Methods
    #######################

    def wait_for_connection(self, timout = float('inf') ):
        success = False
        self.msg_if.pub_info("Waiting for connection")
        timer = 0
        time_start = nepi_ros.ros_time_now()
        while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.ros_time_now() - time_start
        if self.connected == False:
            self.msg_if.pub_info("Failed to Connect")
        else:
            self.msg_if.pub_info("Connected")
        return self.connected

    def wait_for_status(self,timeout = float('inf')):
        success = self.wait_for_connection(timeout)
        if success == False:
            self.msg_if.pub_warn("Manager Not connected")
        else:
            self.msg_if.pub_info("Waiting for status msg")
            found_topic = nepi_ros.wait_for_topic(self.status_topic, timeout)
            if found_topic == "":
                self.msg_if.pub_warn("Failed to get status msg")
            else:
                self.msg_if.pub_info("Got status msg")
                success = True
        return success

    #######################
    # Class Public Pub Methods
    #######################

        self.save_config_pub = rospy.Publisher(self.pub_sub_namespace + 'save_config', Empty, queue_size=10)

    def create_node_save_config_service(self,node_namespace):
        topic = os.path.join(node_namespace,'save_config')
        service = rospy.Publisher(topic, Empty, queue_size=10)
        time.sleep(1)
        return service

    def save_node_config(self, config_service):
        success = False
        try:
            config_service.publish(Empty())
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Failed to publish config_service: " + str(config_service) + " " + str(e))

        success = True
        return success


    def save_all_node_configs(self):
        success = False
        if self.connected == True:
            self.save_config_pub.publish(Empty())
            success = True
        else:
            self.msg_if.pub_warn("Manager Not connected")
        return success



    #######################
    # Class Public Service Methods
    #######################

    def factory_reset(self):
        success = False
        service_name = 'factory_reset'
        srv_dict = self.services_dict[service_name]

        folder_path = None

        # Create service request
        request = None
        try:
            request = srv_dict['req']()
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))
            
        # Call service
        response = None
        if request is not None:
            response = nepi_ros.call_service(srv_dict['service'],  request)

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            env_str = response.op_env
            self.msg_if.pub_info("Got status response" + str(response) + " for service: " + service_name)
            success = True

        return success
        

    def user_reset(self):
        success = False
        service_name = 'user_reset'
        srv_dict = self.services_dict[service_name]

        folder_path = None

        # Create service request
        request = None
        try:
            request = srv_dict['req']()
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + service_name + " " + str(e))
            
        # Call service
        response = None
        if request is not None:
            response = nepi_ros.call_service(srv_dict['service'],  request)

        # Process Response
        if response is None:
            self.msg_if.pub_warn("Failed to get response for service: " + service_name)
        else:
            env_str = response.op_env
            self.msg_if.pub_info("Got status response" + str(response) + " for service: " + service_name)
            success = True

        return success




    #######################
    # Class Private Methods
    #######################


