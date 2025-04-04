#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy
import os
import time


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg 

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.msg import SystemStatus, SystemDefs, WarningFlags, StampedString, SaveData
from nepi_ros_interfaces.srv import SystemDefsQuery, SystemDefsQueryRequest, OpEnvironmentQuery, OpEnvironmentQueryRequest, \
                             SystemSoftwareStatusQuery, SystemSoftwareStatusQueryRequest, SystemStorageFolderQuery, SystemStorageFolderQueryRequest

from nepi_ros_interfaces.msg import SystemTrigger, SystemTriggersStatus
from nepi_ros_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse

from nepi_ros_interfaces.msg import SystemState, SystemStatesStatus
from nepi_ros_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse


class ConnectMgrSystemIF:
 
    NODE_NAME = 'config_mgr'

    connected = False

    status_topic_name = 'system_status'
    status_msg_type = SystemStatus
    status_sub = None
    status_connected = False

    triggers_status_sub = None
    triggers_status_msg = None


    states_status_sub = None
    states_status_msg = None


    services_dict = {
        'sys_storage': {
            'topic': 'system_storage_folder_query',
            'msg': SystemStorageFolderQuery,
            'req': SystemStorageFolderQueryRequest(),
        },

        'sys_env': {
            'topic': 'op_environment_query',
            'msg': OpEnvironmentQuery,
            'req': OpEnvironmentQueryRequest(),
        },

        'sw_status': {
            'topic': 'sw_update_status_query',
            'msg': SystemSoftwareStatusQuery,
            'req': SystemSoftwareStatusQueryRequest(),
        },

        'sys_defs': {
            'topic': 'system_defs_query',
            'msg': SystemDefsQuery,
            'req': SystemDefsQueryRequest(),
        }
    }



    #######################
    ### IF Initialization

    def __init__(self):
        #################################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Starting IF Initialization Processes")
        ##############################

        self.mgr_namespace = os.path.join(self.base_namespace,self.NODE_NAME)
        
        self.status_topic = os.path.join(self.base_namespace,self.status_topic_name)

        ##################################
        ### Create Publishers Topic 
        self.save_data_pub = rospy.Publisher(self._get_ns('save_data'), SaveData, queue_size=10)
        self.clear_data_folder_pub = rospy.Publisher(self._get_ns('clear_data_folder'), Empty, queue_size=10)
        self.set_op_environment_pub = rospy.Publisher(self._get_ns('set_op_environment'), String, queue_size=10)
        self.set_device_id_pub = rospy.Publisher(self._get_ns('set_device_id'), String, queue_size=10)
        self.submit_system_error_msg_pub = rospy.Publisher(self._get_ns('submit_system_error_msg'), String, queue_size=10)
        self.install_new_image_pub = rospy.Publisher(self._get_ns('install_new_image'), String, queue_size=10)
        self.switch_active_inactive_rootfs_pub = rospy.Publisher(self._get_ns('switch_active_inactive_rootfs'), Empty, queue_size=10)
        self.archive_inactive_rootfs_pub = rospy.Publisher(self._get_ns('archive_inactive_rootfs'), Empty, queue_size=10)
        self.save_data_prefix_pub = rospy.Publisher(self._get_ns('save_data_prefix'), String, queue_size=10)

        time.sleep(1)

        ##################################
        ### Set up Services
        for service_name in self.services_dict.keys():
            service_dict = self.services_dict[service_name]
            name = 'detector_info_query'
            service_namespace = os.path.join(self.base_namespace, service_dict['topic'])
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Waiting for service: " + service_name + " on namespace " + service_namespace)
            ret = nepi_ros.wait_for_service(service_dict['topic'], timeout = float('inf') )
            if ret == "":
                nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Wait for service: " + service_name + " timed out") 
            else:
                nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Creating service call for: " + service_name)
                #nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Creating service with namespace: " + service_namespace)
                #nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Creating service with msg: " + str(service_dict['msg']))
                service = None
                try:
                    service = nepi_ros.create_service(service_namespace, service_dict['msg'])
                    time.sleep(1)
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Failed to get service connection: " + service_name + " " + str(e))  
                if service is not None:
                    self.services_dict[service_name]['service'] = service
                    self.services_dict[service_name]['connected'] = True



        #################################
        self.connected = True
        nepi_msg.publishMsgInfo(self,":" + self.class_name + ": IF Initialization Complete")
        #################################



    #######################
    # Class Public Sub Methods
    #######################

    def check_connection(self):
        return self.connection

    def wait_for_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Failed to Connect")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Connected")
        return self.connected


    def get_node_name(self):
        return self.NODE_NAME

    def get_node_namespace(self):
        return self.mgr_namespace
    
    def get_pub_sub_namespace(self):
        return self.base_namespace

    #Start Status Subscriber
    def register_status_listener(self,timeout = float('inf')): 
        success = False
        if self.status_sub is not None:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Status listener already running")
            success = True
        else:
            # Wait for system manager status
            nepi_msg.publishMsgInfo(self,"Waiting for topic: " + self.status_topic)
            topic = nepi_ros.wait_for_topic(self.status_topic, timeout)
            if topic != "":
                self.status_msg = None
                self.status_sub = rospy.Subscriber(topic, self.status_msg_type , self._statusCb)
                #######################
                # Wait for Status Message
                nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Waiting for status message")
                timer = 0
                time_start = nepi_ros.get_time()
                while self.status_msg is None and timer < timeout and not nepi_ros.is_shutdown():
                    nepi_ros.sleep(.2)
                    timer = nepi_ros.get_time() - time_start
                if self.status_msg is None:
                    nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Status msg topic subscribe timed out " + str(status_topic))
                else:
                    nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Got status msg " + str(self.status_msg))
                    success = True
        return success


    def unregister_status_listener(self): 
        success = False
        if self.status_sub is None:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Status listener not running")
            success = True
        else:
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Killing status listener")
            try:
                self.status_sub.unregister()
                success = True
            except:
                pass
            time.sleep(1)
            self.status_connected = False
            self.status_dict = dict()
            self.status_msg = None
        return success



    def get_status_dict(self):
        status_dict = None
        if self.status_sub is not None:
            if self.status_msg is not None:
                status_dict = nepi_ros.convert_msg2dict(self.status_msg)
            else:
                nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Status Listener Not connected")
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Manager Not connected")
        return status_dict



    def wait_for_status(self,timeout = float('inf')):
        nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Waiting for system to connect")
        success = self.wait_for_connection(timeout)
        if success == False:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Manager Not connected")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Waiting for status topic")
            found_topic = nepi_ros.wait_for_topic(self.status_topic, timeout)
            if found_topic == "":
                nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Failed to get status topic")
            else:
                nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Got status topic")
                success = True
        return success


    def create_states_status_listener(self, namespace = ""):
        self.states_status_msg = None
        states_status_namespace = os.path.join(namespace, 'system_states_status')
        self.states_status_sub = rospy.Subscriber(states_status_namespace, SystemStatesStatus, self._statusSubCb, queue_size=1)

    def wait_for_states_status_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Waiting for status connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.states_status_msg is None and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.det_connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Status Connection Timed Out")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Status Connected")
        return self.det_connected


    def get_states_status_dict(self):
        states_dict = None
        if self.states_status_sub is not None:
            if self.states_status_msg is not None:
                states_dict = nepi_states.parse_states_states_status_msg(self.states_status_msg)
            else:
                nepi_msg.publishMsgWarn(self,":" + self.class_name + ": State Status listener has not received any data yet: ")
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": State Status listener not created: ")
        return states_dict      



    def create_triggers_triggers_status_listener(self, namespace = ""):
        self.triggers_status_msg = None
        triggers_status_namespace = os.path.join(namespace, 'system_triggers_status')
        self.triggers_status_sub = rospy.Subscriber(triggers_status_namespace, SystemTriggersStatus, self._statusSubCb, queue_size=1)


    def wait_for_triggers_status_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Waiting for status connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.triggers_status_msg is None and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.det_connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Status Connection Timed Out")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.class_name + ": Status Connected")
        return self.det_connected

    def get_triggers_status_dict(self):
        triggers_dict = None
        if self.triggers_status_sub is not None:
            if self.triggers_status_msg is not None:
                triggers_dict = nepi_triggers.parse_triggers_triggers_status_msg(self.triggers_status_msg)
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Trigger Status listener has not received any data yet: ")
        return triggers_dict




    #######################
    # Class Public Pub Methods
    #######################


    def clear_data_folder(self):
        success = False
        if self.connected == True:
            self.clear_data_folder_pub.publish(Empty())
            success = True
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ": Manager Not connected")
        return success

    def set_all_save_prefix(self,prefix_str):
        success = False
        if self.connected == True:
            self.save_data_prefix_pub.publish(prefix_str)
            success = True
        else:
            nepi_msg.publishMsgWarn(self,":" + self.class_name + ":Manager Not connected")
        return success


    #######################
    # Class Public Service Methods
    #######################

    def get_sys_folder_path(self,folder_name, fallback_path = ""):
        service_name = 'sys_storage'
        srv_dict = self.services_dict[service_name]
        
        folder_path = None

        if 'service' in srv_dict.keys():
            # Create service request
            request = None
            try:
                request = folder_name
                response = nepi_ros.call_service(srv_dict['service'],  request)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to create service request: " + service_name + " " + str(e))
                

            # Process Response
            if response is None or response == "":
                folder_path = fallback_path
                nepi_msg.publishMsgWarn(self,"Returning fallback path: " + fallback_path)
            else:
                folder_path = response.folder_path
                nepi_msg.publishMsgInfo(self,"Got folder path: " + folder_path + " for folder request: " + folder_name)

        return folder_path
        

    def get_op_env_str(self):
        service_name = 'sys_env'
        srv_dict = self.services_dict[service_name]

        env_str = None

        if 'service' in srv_dict.keys():
            # Create service request
            request = None
            try:
                request = srv_dict['req']
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to create service request: " + service_name + " " + str(e))
                
            # Call service
            response = None
            if request is not None:
                response = nepi_ros.call_service(srv_dict['service'],  request)

            # Process Response
            if response is None:
                nepi_msg.publishMsgWarn(self,"Failed to get response for service: " + service_name)
            else:
                env_str = response.op_env
                nepi_msg.publishMsgInfo(self,"Got status response" + str(response) + " for service: " + service_name)

        return env_str

    def get_software_status_dict(self):
        service_name = 'sw_status'
        srv_dict = self.services_dict[service_name]
        status_dict = None

        if 'service' in srv_dict.keys():
            # Create service request
            request = None
            try:
                    request = srv_dict['req']
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to create service request: " + service_name + " " + str(e))
                
            # Call service
            response = None
            if request is not None:
                response = nepi_ros.call_service(srv_dict['service'],  request)

            # Process Response
            if response is None:
                nepi_msg.publishMsgWarn(self,"Failed to get response for service: " + service_name)
            else:
                status_dict = nepi_ros.convert_msg2dict(response)
                #nepi_msg.publishMsgInfo(self,"Got status response" + str(response) + " for service: " + service_name)
                nepi_msg.publishMsgInfo(self,"Got system status response" + str(status_dict) + " for service: " + service_name)

        return status_dict


    def get_system_stats_dict(self):
        service_name = 'sys_defs'
        srv_dict = self.services_dict[service_name]
        stats_dict = None

        if 'service' in srv_dict.keys():
            # Create service request
            request = None
            try:
                    request = srv_dict['req']
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to create service request: " + service_name + " " + str(e))
                
            # Call service
            response = None
            if request is not None:
                response = nepi_ros.call_service(srv_dict['service'],  request)

            # Process Response
            if response is None:
                nepi_msg.publishMsgWarn(self,"Failed to get response for service: " + service_name)
            else:
                stats_dict = nepi_ros.convert_msg2dict(response)['defs']
                #nepi_msg.publishMsgInfo(self,"Got status response" + str(response) + " for service: " + service_name)
                nepi_msg.publishMsgInfo(self,"Got stats dict" + str(stats_dict) + " for service: " + service_name)

        return stats_dict




    #######################
    # Class Private Methods
    #######################

    def _get_ns(self,topic_str):
        return os.path.join(self.base_namespace,topic_str)

    # Update System Status
    def _statusCb(self,msg):
        self.status_msg = msg
        self.status_connected = True


