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

from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion


from nepi_ros_interfaces.msg import NavPose, NavPoseData
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, NavPoseQueryResponse
from nepi_ros_interfaces.srv import NavPoseStatusQuery, NavPoseStatusQueryRequest, NavPoseStatusQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils


from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.messages_if import MsgIF


class ConnectMgrNavPoseIF:
 
    MGR_NODE_NAME = 'nav_pose_mgr'

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
            }
        }



        # Subscribers Config Dict ####################
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
            self.msg_if.pub_warn("Failed to create service request: " + " " + str(e))
        try:
            response = self.node_if.call_service(service_name, request, verbose = verbose)
            #self.msg_if.pub_info("Got time status response: " + str(response))
        except Exception as e:
            if verbose == True:
                self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))
            else:
                pass
        try:
            navpose_dict = nepi_nav.convert_navpose_resp2data_dict(response)
        except Exception as e:
            if verbose == True:
                self.msg_if.pub_warn("Failed to convdert navpose query response to dict: " + service_name + " " + str(e))
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
            self.msg_if.pub_warn("Failed to create service request: " + " " + str(e))
        try:
            response = self.node_if.call_service(service_name, request, verbose = verbose)
            #self.msg_if.pub_info("Got time status response: " + str(response))
            navpose_status = response.status
            status_dict = nepi_ros.convert_msg2dict(time_status)
            #self.msg_if.pub_info("Got time status dict: " + str(navpose_status))
        except Exception as e:
            if verbose == True:
                self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e))
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


    #######################
    # Class Private Methods
    #######################

