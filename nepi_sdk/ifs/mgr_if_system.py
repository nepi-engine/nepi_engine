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

from std_msgs.msg import String, Empty, Float32
from nepi_ros_interfaces.msg import SystemStatus, SystemDefs, WarningFlags, StampedString, SaveData
from nepi_ros_interfaces.srv import SystemDefsQuery, SystemDefsQueryRequest, OpEnvironmentQuery, OpEnvironmentQueryRequest, \
                             SystemSoftwareStatusQuery, SystemSoftwareStatusQueryRequest, SystemStorageFolderQuery, SystemStorageFolderQueryRequest




class MgrSystemIF():
 
    MGR_NODE_NAME = 'system_mgr'
    mgr_namespace = MGR_NODE_NAME
    pub_sub_namespace = ''

    connected = False


    status_topic_name = 'system_status'
    status_msg_type = SystemStatus
    status_topic = ""
    status_sub = None
    status_connected = False


    services_dict = {
        'system_storage_folder_query': {
            'connected': False,
            'msg': SystemStorageFolderQuery,
            'req': SystemStorageFolderQueryRequest(),
            'psns': pub_sub_namespace,
            'service': None
        },

        'op_environment_query': {
            'connected': False,
            'msg': OpEnvironmentQuery,
            'req': OpEnvironmentQueryRequest(),
            'psns': pub_sub_namespace,
            'service': None
        },

        'sw_update_status_query': {
            'connected': False,
            'msg': SystemSoftwareStatusQuery,
            'req': SystemSoftwareStatusQueryRequest(),
            'psns': pub_sub_namespace,
            'service': None
        },

        'system_defs_query': {
            'connected': False,
            'msg': SystemDefsQuery,
            'req': SystemDefsQueryRequest(),
            'psns': pub_sub_namespace,
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
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting IF Initialization Processes")
        ##############################

        self.mgr_namespace = os.path.join(self.base_namespace,self.MGR_NODE_NAME)
        self.status_topic = os.path.join(self.base_namespace,self.pub_sub_namespace,self.status_topic_name)

        ##################################
        ### Create Publishers Topic 
        self.save_data_pub = rospy.Publisher('save_data', SaveData, queue_size=10)
        self.clear_data_folder_pub = rospy.Publisher('clear_data_folder', Empty, queue_size=10)
        self.set_op_environment_pub = rospy.Publisher('set_op_environment', String, queue_size=10)
        self.set_device_id_pub = rospy.Publisher('set_device_id', String, queue_size=10)
        self.submit_system_error_msg_pub = rospy.Publisher('submit_system_error_msg', String, queue_size=10)
        self.install_new_image_pub = rospy.Publisher('install_new_image', String, queue_size=10)
        self.switch_active_inactive_rootfs_pub = rospy.Publisher('switch_active_inactive_rootfs', Empty, queue_size=10)
        self.archive_inactive_rootfs_pub = rospy.Publisher('archive_inactive_rootfs', Empty, queue_size=10)
        self.save_data_prefix_pub = rospy.Publisher('save_data_prefix', String, queue_size=10)

        time.sleep(1)

        ##################################
        ### Set up Services
        for service_name in self.services_dict.keys():
            service_dict = self.services_dict[service_name]
            name = 'detector_info_query'
            service_namespace = os.path.join(self.base_namespace,service_dict['psns'], service_name)
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for " + service_name + " on namespace " + service_namespace)
            ret = nepi_ros.wait_for_service(service_namespace, timeout = float('inf') )
            if ret == "":
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Wait for service: " + service_name + " timed out") 
            else:
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Creating service call for: " + service_name)
                #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Creating service with namespace: " + service_namespace)
                #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Creating service with msg: " + str(service_dict['msg']))
                service = None
                try:
                    service = nepi_ros.create_service(service_namespace, service_dict['msg'])
                    time.sleep(1)
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to get service connection: " + service_name + " " + str(e))  
            if service is not None:
                self.services_dict[service_name]['service'] = service
                self.services_dict[service_name]['connected'] = True


        #################################
        self.connected = True
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": IF Initialization Complete")
        #################################



    #######################
    # Class Public Sub Methods
    #######################

    def wiat_for_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for connection")
        timer = 0
        time_start = nepi_ros.ros_time_now()
        while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.ros_time_now() - time_start
        if self.connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Connected")
        return self.connected


    #Start Status Subscriber
    def register_status_listener(self,timeout = float('inf')): 
        success = False
        if self.status_sub is not None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Status listener already running")
            success = True
        else:
            # Wait for system manager status
            nepi_msg.publishMsgInfo(self,"Waiting for topic: " + topic)
            topic = nepi_ros.wait_for_topic(self.status_topic, timeout)
            if topic != "":
                self.status_msg = None
                self.status_sub = rospy.Subscriber(topic, self.status_msg_type , self.statusCb)
                #######################
                # Wait for Status Message
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for status message")
                timer = 0
                time_start = nepi_ros.ros_time_now()
                while self.status_msg is None and timer < timeout and not nepi_ros.is_shutdown():
                    nepi_ros.sleep(.2)
                    timer = nepi_ros.ros_time_now() - time_start
                if self.status_msg is None:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Status msg topic subscribe timed out " + str(status_topic))
                else:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got status msg " + str(self.status_msg))
                    success = True
        return success


    def unregister_status_listener(self): 
        success = False
        if self.status_sub is None:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Status listener not running")
            success = True
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Killing status listener")
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
        if status_sub is not None:
            if self.status_msg is not None:
                status_dict = nepi_ros.convert_msg2dict(self.status_msg)
            else:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Status Listener Not connected")
        else:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Manager Not connected")
        return status_dict



    def wait_for_status(self,timeout = float('inf')):
        success = self.wiat_for_connection(timeout)
        if success == False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Manager Not connected")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for status msg")
            found_topic = nepi_ros.wait_for_topic(self.status_topic, timeout)
            if found_topic == "":
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to get status msg")
            else:
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Got status msg")
                success = True
        return success


    #######################
    # Class Public Pub Methods
    #######################


    def clear_data_folder(self):
        success = False
        if self.connected == True:
            self.clear_data_folder_pub.publish(Empty())
            success = True
        else:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Manager Not connected")
        return success

    def set_all_save_prefix(self,prefix_str):
        success = False
        if self.connected == True:
            self.save_data_prefix_pub.publish(prefix_str)
            success = True
        else:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ":Manager Not connected")
        return success


    #######################
    # Class Public Service Methods
    #######################

    def get_sys_folder_path(self,folder_name, fallback_path = ""):
        service_name = 'system_storage_folder_query'
        srv_dict = self.services_dict[service_name]

        folder_path = None

        # Create service request
        request = None
        try:
            request = folder_name
        except Exception as e:
            rospy.logerr("Failed to create service request: " + service_name + " " + str(e))
            
        # Call service
        response = None
        if request is not None:
            response = self.callService(service_name, request)

        # Process Response
        if response is None or response == "":
            folder_path = fallback_param
            nepi_msg.publishMsgWarn(self,"Returning fallback path: " + fallback_path)
        else:
            folder_path = response.folder_path
            nepi_msg.publishMsgInfo(self,"Got folder path: " + folder_path + " for folder request: " + folder_name)

        return folder_path
        

    def get_op_env_str(self):
        service_name = 'op_environment_query'
        srv_dict = self.services_dict[service_name]

        env_str = None

       # Create service request
        request = None
        try:
            request = srv_dict['req']
        except Exception as e:
            rospy.logerr("Failed to create service request: " + service_name + " " + str(e))
            
        # Call service
        response = None
        if request is not None:
            response = self.callService(service_name, request)

        # Process Response
        if response is None:
            nepi_msg.publishMsgWarn(self,"Failed to get response for service: " + service_name)
        else:
            env_str = response.op_env
            nepi_msg.publishMsgInfo(self,"Got status response" + str(response) + " for service: " + service_name)

        return env_str

    def get_software_status_dict(self):
        service_name = 'sw_update_status_query'
        srv_dict = self.services_dict[service_name]
        status_dict = None

      # Create service request
        request = None
        try:
                request = srv_dict['req']
        except Exception as e:
            rospy.logerr("Failed to create service request: " + service_name + " " + str(e))
            
        # Call service
        response = None
        if request is not None:
            response = self.callService(service_name, request)

        # Process Response
        if response is None:
            nepi_msg.publishMsgWarn(self,"Failed to get response for service: " + service_name)
        else:
            status_dict = nepi_ros.convert_msg2dict(response)
            #nepi_msg.publishMsgInfo(self,"Got status response" + str(response) + " for service: " + service_name)
            nepi_msg.publishMsgInfo(self,"Got system status response" + str(status_dict) + " for service: " + service_name)

        return status_dict

    def get_software_status_dict(self):
        service_name = 'op_environment_query'
        srv_dict = self.services_dict[service_name]
        status_dict = None

      # Create service request
        request = None
        try:
                request = srv_dict['req']
        except Exception as e:
            rospy.logerr("Failed to create service request: " + service_name + " " + str(e))
            
        # Call service
        response = None
        if request is not None:
            response = self.callService(service_name, request)

        # Process Response
        if response is None:
            nepi_msg.publishMsgWarn(self,"Failed to get response for service: " + service_name)
        else:
            status_dict = nepi_ros.convert_msg2dict(response)
            #nepi_msg.publishMsgInfo(self,"Got software status response" + str(response) + " for service: " + service_name)
            nepi_msg.publishMsgInfo(self,"Got software status response" + str(status_dict) + " for service: " + service_name)

        return status_dict

    def get_system_stats_dict(self):
        service_name = 'system_defs_query'
        srv_dict = self.services_dict[service_name]
        stats_dict = None

      # Create service request
        request = None
        try:
                request = srv_dict['req']
        except Exception as e:
            rospy.logerr("Failed to create service request: " + service_name + " " + str(e))
            
        # Call service
        response = None
        if request is not None:
            response = self.callService(service_name, request)

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

    # Update System Status
    def statusCb(self,msg):
        self.status_msg = msg
        self.status_connected = True

    def callService(self,service_name,request):
        response = None
        srv_dict = self.services_dict[service_name]
        if srv_dict['connected'] == True:
            try:
                response = srv_dict['service'](request)
            except Exception as e:
                nepi_msg.publishMsgWarn(self,"Failed to get service call response")
        else:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": " + service_name + ": Service Not Connected")
        return response

