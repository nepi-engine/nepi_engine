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
import numpy as np
import math
import time
import sys
import tf
import yaml
import threading
import copy



from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry

from nepi_interfaces.msg import MgrNavPoseStatus,MgrNavPoseCompInfo

from nepi_interfaces.msg import UpdateTopic, UpdateNavPoseTopic, UpdateFrame3DTransform

from nepi_interfaces.msg import NavPose
from nepi_interfaces.msg import NavPoseLocation, NavPoseHeading
from nepi_interfaces.msg import NavPoseOrientation, NavPosePosition
from nepi_interfaces.msg import NavPoseAltitude, NavPoseDepth

from nepi_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, NavPoseQueryResponse

from nepi_interfaces.msg import Frame3DTransform, Frame3DTransforms
from nepi_interfaces.srv import Frame3DTransformsQuery, Frame3DTransformsQueryRequest, Frame3DTransformsQueryResponse

from nepi_interfaces.msg import SaveDataRate

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.connect_system_if import ConnectSaveDataIF


class ConnectMgrNavPoseIF:
 
    MGR_NODE_NAME = 'navpose_mgr'

    ready = False

    services_connected = False

    status_msg = None
    status_dict = None
    status_connected = False

    navpose_dict = None

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
        self.mgr_namespace = os.path.join(self.base_namespace,self.MGR_NODE_NAME)

        ##############################
        ### Wait for status to publish
        status_topic = nepi_sdk.create_namespace(self.mgr_namespace,'status')
        self.msg_if.pub_info("Waiting for status topic: " + status_topic, log_name_list = self.log_name_list)
        nepi_sdk.wait_for_topic(status_topic)
        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'namespace': self.mgr_namespace
        }

        # Services Config Dict ####################     
        # Services Config Dict ####################
        self.SRVS_DICT = {
            'navpose_query': {
                'namespace': self.mgr_namespace,
                'topic': 'navpose_query',
                'srv': NavPoseQuery,
                'req': NavPoseQueryRequest(),
                'resp': NavPoseQueryResponse()
            }
        }

    

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'pub_rate': {
                'namespace': self.mgr_namespace,
                'topic': 'set_pub_rate',
                'msg': Float32,
                'qsize': 1,
                'latch': False,
            },
            'set_topic': {
                'namespace': self.mgr_namespace,
                'topic': 'set_topic',
                'msg': UpdateNavPoseTopic,
                'qsize': 1,
                'latch': False,
            },
            'clear_topic': {
                'namespace': self.mgr_namespace,
                'topic': 'clear_topic',
                'msg': String,
                'qsize': 1,
                'latch': False,
            },
            'set_transform': {
                'namespace': self.mgr_namespace,
                'topic': 'set_transform',
                'msg': UpdateFrame3DTransform,
                'qsize': 1,
                'latch': False,
            },
            'clear_transform': {
                'namespace': self.mgr_namespace,
                'topic': 'clear_transform',
                'msg': String,
                'qsize': 1,
                'latch': False,
            },
            'set_navpose': {
                'namespace': self.mgr_namespace,
                'topic': 'set_navpose',
                'msg': NavPose,
                'qsize': 1,
                'latch': False,
            },
            'reset_navpose': {
                'namespace': self.mgr_namespace,
                'topic': 'reset_navpose',
                'msg': Empty,
                'qsize': 1,
                'latch': False
            },
            'set_init_navpose': {
                'namespace': self.mgr_namespace,
                'topic': 'set_init_navpose',
                'msg': NavPose,
                'qsize': 1,
                'latch': False
            },
            'reset_init_navpose': {
                'namespace': self.mgr_namespace,
                'topic': 'reset_init_navpose',
                'msg': Empty,
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
            'status_sub': {
                'namespace': self.mgr_namespace,
                'topic': 'status',
                'msg': MgrNavPoseStatus,
                'qsize': 1,
                'callback': self._navposeDataStatusCb
            },
            'navpose_sub': {
                'namespace': self.base_namespace,
                'topic': 'navpose',
                'msg': NavPose,
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
        nepi_sdk.wait()

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
        self.msg_if.pub_info("Waiting for ready", log_name_list = self.log_name_list)
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
            exists = nepi_sdk.check_for_service('nav_pose_status_query')
            nepi_sdk.wait()
            if exists == True:
                ret = self.get_navpose_solution_msg(verbose = True)
                connected = (ret is not None)
            timer = nepi_sdk.get_time() - time_start
        if connected == False:
            self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return connected

    def call_navpose_dict_service(self, verbose = True):
        service_name = 'navpose_query'
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
                self.msg_if.pub_warn("Failed to call service request: " + service_name + " " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
            else:
                pass
        if response is not None:
            navpose_msg = response.navpose_dict
            navpose_dict = nepi_nav.convert_navpose_msg2dict(navpose_msg)
        return navpose_dict


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
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

    def get_pub_rate(self):
        rate = 10.0
        if self.status_dict is not None:
            rate = self.status_dict['pub_rate']


    def set_pub_rate(self,pub_rate):
        pub_name = 'set_pub_rate'
        msg = pub_rate
        self.con_node_if.publish_pub(pub_name,msg)  
   

    def get_comp_names(self):
        return nepi_nav.NAVPOSE_COMPONENTS

    def set_navpose_source(self,name,topic,transform_list = None):
        if name in nepi_nav.NAVPOSE_COMPONENTS:
            msg = UpdateNavPoseTopic()
            msg.name = name
            msg.topic = topic
            msg.apply_transform = False
            if transform_list is not None:
                tr_msg = nepi_nav.convert_transform_list2msg(transform_list)
                if tr_msg is not None:
                    msg.apply_transform = True
                    msg.transform = tr_msg
            self.node_if.publish_pub('set_topic',msg)


    def clear_comp_topic(self,name):
        if name in nepi_nav.NAVPOSE_COMPONENTS:
            msg = name
            self.node_if.publish_pub('clear_topic',msg)


    def reinit_navpose_solution(self):
        self.node_if.publish_pub('reinit_navpose_solution',Empty())


    def get_navpose_dict(self, verbose = True):
        return self.navpose_dict    

    def get_navpose_status_dict(self):
        return self.status_dict


    def unregister(self):
        self._unsubscribeTopic()

    #################
    ## Save Config Functions

    def call_save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def call_reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def call_factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())
        
    #################
    ## Save Data Functions

    def get_save_data_products(self):
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_save_data_status(self):
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def set_save_data_prefix(self,prefix):
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def set_save_data_rate(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def set_save_data_enable(self,enable):
        self.con_save_data_if.save_data_pub(enable)

    def call_save_data_snapshot(self):
        self.con_save_data_if.publish_pub()

    def call_save_data_reset(self):
        self.con_save_data_if.publish_pub(pub_name,msg)

    def call_save_data_factory_reset(self):
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


    def _navposeDataStatusCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg
        self.status_dict = nepi_sdk.convert_msg2dict(status_msg)

    def _navposeDataCb(self,data_msg):      
        self.navpose_dict = nepi_nav.convert_navpose_msg2dict(data_msg)

