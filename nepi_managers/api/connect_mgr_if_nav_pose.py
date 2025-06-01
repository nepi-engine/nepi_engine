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
import sys
import time
import copy
import numpy as np
import math

import tf
import yaml

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion


from nepi_ros_interfaces.msg import NavPose, NavPoseData, SaveDataRate
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, NavPoseQueryResponse
from nepi_ros_interfaces.srv import NavPoseStatusQuery, NavPoseStatusQueryRequest, NavPoseStatusQueryResponse

from nepi_ros_interfaces.msg import NavPoseData

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from nepi_api.messages_if import MsgIF
from nepi_api.connect_system_if import ConnectSaveDataIF
from nepi_api.connect_node_if import ConnectNodeClassIF


class ConnectMgrNavPoseIF:
 
    MGR_NODE_NAME = 'nav_pose_mgr'

    ready = False

    services_connected = False

    status_msg = None
    status_connected = False

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
            'nav_pose_query': {
                'namespace': self.base_namespace,
                'topic': 'nav_pose_query',
                'srv': NavPoseQuery,
                'req': NavPoseQueryRequest(),
                'resp': NavPoseQueryResponse()
            },
            'nav_pose_status_query': {
                'namespace': self.base_namespace,
                'topic': 'nav_pose_status_query',
                'srv': NavPoseStatusQuery,
                'req': NavPoseStatusQueryRequest(),
                'resp': NavPoseStatusQueryResponse()
            },
        }

    

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'set_gps_topic': {
                'namespace': self.base_namespace,
                'topic': 'set_gps_topic',
                'msg': String,
                'latch': False,
                'qsize': None
            },
            'set_pose_topic': {
                'namespace': self.base_namespace,
                'topic': 'set_pose_topic',
                'msg': String,
                'latch': False,
                'qsize': None
            },
            'set_elevation_topic': {
                'namespace': self.base_namespace,
                'topic': 'set_elevation_topic',
                'msg': String,
                'latch': False,
                'qsize': 10
            },
            'set_heading_topic': {
                'namespace': self.base_namespace,
                'topic': 'set_heading_topic',
                'msg': String,
                'latch': False,
                'qsize': 10
            },
            'enable_gps_clock_sync': {
                'namespace': self.base_namespace,
                'topic': 'enable_gps_clock_sync',
                'msg': Bool,
                'latch': False,
                'qsize': 10
            },
            'reinit_navpose_solution': {
                'namespace': self.base_namespace,
                'topic': 'reinit_navpose_solution',
                'msg': Empty,
                'latch': False,
                'qsize': 10
            },
            'pub_rate': {
                'namespace': self.mgr_namespace,
                'topic': 'set_pub_rate',
                'msg': Float32,
                'qsize': 1,
                'latch': False
            },
            '3d_frame': {
                'namespace': self.mgr_namespace,
                'topic': 'set_3d_frame',
                'msg': String,
                'qsize': 1,
                'latch': False

            },
            'alt_frame': {
                'namespace': self.mgr_namespace,
                'topic': 'set_alt_frame',
                'msg': String,
                'qsize': 1,
                'latch': False
            },
            'save_config': {
                'namespace': self.mgr_namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'reset_config': {
                'namespace': self.mgr_namespace,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'factory_reset_config': {
                'namespace': self.mgr_namespace,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            }
        }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'navpose_pub': {
                'namespace': self.mgr_namespace,
                'topic': 'navpose',
                'msg': NavPoseData,
                'qsize': 1,
                'callback': self._navposeDataCb
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
        nepi_ros.wait()

        self.con_save_data_if = ConnectSaveDataIF(namespace = self.mgr_namespace)

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
            exists = nepi_ros.check_for_service('nav_pose_status_query')
            nepi_ros.wait()
            if exists == True:
                ret = None
                try:
                    ret = self.get_time_status(verbose = True)
                except:
                    pass # Don't Print Exception
                connected = (ret is not None)
            timer = nepi_ros.get_time() - time_start
        if connected == False:
            self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return connected


    def get_navpose_solution(self, verbose = True):
        service_name = 'nav_pose_query'
        response = None
        navpose_dict = None
        try:
            request = self.node_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + " " + str(e), log_name_list = self.log_name_list)
        try:
            response = self.node_if.call_service(service_name, request, verbose = verbose)
            #self.msg_if.pub_info("Got time status response: " + str(response), log_name_list = self.log_name_list)
        except Exception as e:
            if verbose == True:
                self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e), log_name_list = self.log_name_list)
            else:
                pass
        try:
            navpose_dict = nepi_nav.convert_navpose_resp2data_dict(response)
        except Exception as e:
            if verbose == True:
                self.msg_if.pub_warn("Failed to convdert navpose query response to dict: " + service_name + " " + str(e), log_name_list = self.log_name_list)
            else:
                pass
        return navpose_dict    


    def get_navpose_status(self, verbose = True):
        service_name = 'nav_pose_status_query'
        response = None
        status_dict = None
        try:
            request = self.node_if.create_request_msg(service_name)
        except Exception as e:
            self.msg_if.pub_warn("Failed to create service request: " + " " + str(e), log_name_list = self.log_name_list)
        try:
            response = self.node_if.call_service(service_name, request, verbose = verbose)
            #self.msg_if.pub_info("Got time status response: " + str(response), log_name_list = self.log_name_list)
            navpose_status = response.status
            status_dict = nepi_ros.convert_msg2dict(time_status)
            #self.msg_if.pub_info("Got time status dict: " + str(navpose_status), log_name_list = self.log_name_list)
        except Exception as e:
            if verbose == True:
                self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e), log_name_list = self.log_name_list)
            else:
                pass
        return status_dict    


    def set_gps_source_topic(self,topic):
        self.node_if.publish_pub('set_gps_topic',topic)

    def set_elevation_source_topic(self,topic):
        self.node_if.publish_pub('set_elevation_topic',topic)

    def set_pose_source_topic(self,topic):
        self.node_if.publish_pub('set_pose_topic',topic)

    def set_heading_source_topic(self,topic):
        self.node_if.publish_pub('set_heading_topic',topic)

    def set_enable_gps_clock_sync(self,enable):
        self.node_if.publish_pub('enable_gps_clock_sync',enable)

    def reinit_navpose_solution(self):
        self.node_if.publish_pub('reinit_navpose_solution',Empty())

    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_ros.get_time()
            while self.status_connected == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        return self.img_status_dict

    def unregister(self):
        self._unsubscribeTopic()

    def set_pub_rate(self,pub_rate):
        pub_name = 'set_pub_rate'
        msg = pub_rate
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_3d_frame(self,set_3d_frame):
        pub_name = 'set_3d_frame'
        msg = set_3d_frame
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_alt_frame(self,alt_frame):
        pub_name = 'set_alt_frame'
        msg = alt_frame
        self.con_node_if.publish_pub(pub_name,msg)  

    def save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())
        
    #################
    ## Save Data Functions

    def get_data_products(self):
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_status_dict(self):
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def save_data_pub(self,enable):
        self.con_save_data_if.save_data_pub(enable)

    def save_data_prefix_pub(self,prefix):
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def snapshot_pub(self):
        self.con_save_data_if.publish_pub()

    def reset_pub(self):
        self.con_save_data_if.publish_pub(pub_name,msg)

    def factory_reset_pub(self):
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_save_data_if.publish_pub(pub_name,msg)

    ###############################
    # Class Private Methods
    ###############################
   

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering", log_name_list = self.log_name_list)
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.status_connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e), log_name_list = self.log_name_list)
        return success


    def _navposeDataCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg

