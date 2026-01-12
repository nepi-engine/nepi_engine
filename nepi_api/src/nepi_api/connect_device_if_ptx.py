#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import os
import time 
import copy 
import math
import numpy as np


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_interfaces.msg import RangeWindow, NavPose, SaveDataRate
from nepi_interfaces.msg import DevicePTXStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove
from nepi_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryRequest, PTXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.device_if_npx import NPXDeviceIF


from nepi_api.connect_node_if import ConnectNodeClassIF

#########################################
# Node Class
#########################################

class ConnectPTXDeviceIF:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False
    navpose_msg = None

    statusCb = None
    navposeCb = None
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                statusCb = None,
                navposeCb = None,
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################    
        # Initialize Class Variables

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace,APP_NODE_NAME)
        else:
            namespace = namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.statusCb = statusCb
        self.navposeCb = navposeCb

        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'namespace': self.namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'speed_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'stop_moving': {
                'namespace': self.node_namespace,
                'topic': 'ptx/stop_moving',
                'msg': Empty,
                'qsize': 1,
            },
            'goto_to_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_position',
                'msg': PanTiltPosition,
                'qsize': 1,
            },
            'goto_to_pan_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_pan_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_to_tilt_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_tilt_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_pan_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_pan_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_tilt_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'jog_timed_pan': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_timed_pan',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
            },
            'jog_timed_tilt': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_timed_tilt',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
            },
            'reverse_pan_enabled': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_reverse_pan_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'reverse_tilt_enabled': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_reverse_tilt_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_soft_limits': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_soft_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
            },
            'go_home': {
                'namespace': self.node_namespace,
                'topic': 'ptx/go_home',
                'msg': Empty,
                'qsize': 1,
            },
            'set_home_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_home_position',
                'msg': PanTiltPosition,
                'qsize': 1,
            },
            'set_home_position_here': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_home_position_here',
                'msg': Empty,
                'qsize': 1,
            },
            'set_auto_pan': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_auto_pan_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_auto_pan_window': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_auto_pan_window',
                'msg': RangeWindow,
                'qsize': 1,
            },
            'set_auto_tilt': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_auto_tilt_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_auto_tilt_window': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_auto_tilt_window',
                'msg': RangeWindow,
                'qsize': 1,
            }


        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/status',
                'msg': DevicePTXStatus,
                'qsize': 10,
                'callback': self._statusCb
            },
            'navpose_sub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/navpose',
                'msg': NavPose,
                'qsize': 10,
                'callback': self._navposeCb
            },
        }


        # Create Node Class ####################
        
        self.con_node_if = ConnectNodeClassIF(
                        namespace = self.namespace,
                        configs_dict = self.CONFIGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True,
                        msg_if = self.msg_if
        )

        

        self.con_node_if.wait_for_ready()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.status_connected

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_navpose_dict(self):
        navpose_dict = None
        if self.navpose_msg is not None:
            navpose_dict = nepi_nav.convert_navpose_msg2dict(self.navpose_msg)
        return navpose_dict

    def unregister(self):
        self._unsubscribeTopic()

    def set_speed_ratio(self,speed_ratio):
        pub_name = 'speed_ratio'
        msg = Float32
        self.con_node_if.publish_pub(pub_name,msg)

    def stop_moving(self):
        pub_name = 'stop_moving'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def goto_to_position(self,speed_ratio):
        pub_name = 'speed_ratio'
        msg = Float32
        self.con_node_if.publish_pub(pub_name,msg)

    def goto_to_position(self,position):
        pub_name = 'goto_to_position'
        msg = position
        self.con_node_if.publish_pub(pub_name,msg)

    def goto_to_pan_position(self,pan_position):
        pub_name = 'goto_to_pan_position'
        msg = pan_position
        self.con_node_if.publish_pub(pub_name,msg)
        
    def goto_to_tilt_position(self,tilt_position):
        pub_name = 'goto_to_tilt_position'
        msg = tilt_position
        self.con_node_if.publish_pub(pub_name,msg)

    def goto_pan_ratio(self,pan_ratio):
        pub_name = 'goto_pan_ratio'
        msg = pan_ratio
        self.con_node_if.publish_pub(pub_name,msg)        

    def goto_tilt_ratio(self, tilt_ratio):
        pub_name = 'goto_tilt_ratio'
        msg = tilt_ratio
        self.con_node_if.publish_pub(pub_name,msg)

    def jog_timed_pan(self, timed_pan):
        pub_name = 'jog_timed_pan'
        msg = timed_pan
        self.con_node_if.publish_pub(pub_name,msg)
        
    def jog_timed_tilt(self,timed_tilt):
        pub_name = 'jog_timed_tilt'
        msg = timed_tilt
        self.con_node_if.publish_pub(pub_name,msg)   

    def reverse_pan_enabled(self, reverse_pan):
        pub_name = 'reverse_pan_enabled'
        msg = reverse_pan
        self.con_node_if.publish_pub(pub_name,msg)

    def reverse_tilt_enabled(self, reverse_tilt):
        pub_name = 'reverse_tilt_enabled'
        msg = reverse_pan
        self.con_node_if.publish_pub(pub_name,msg)

    def go_home(self):
        pub_name = 'go_home'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def set_home_position(self,home_position):
        pub_name = 'set_home_position'
        msg = home_position
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_home_position_here(self):
        pub_name = 'set_home_position_here'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_auto_pan(self,auto_pan):
        pub_name = 'set_auto_pan'
        msg = auto_pan
        self.con_node_if.publish_pub(pub_name,msg) 

    def set_auto_pan_window(self,pan_window):
        pub_name = 'set_auto_pan_window'
        msg = pan_window
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_auto_tilt(self,auto_tilt):
        pub_name = 'set_auto_tilt'
        msg = auto_tilt
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_auto_tilt_window(self,tilt_window):
        pub_name = 'set_auto_pan_window'
        msg = tilt_window
        self.con_node_if.publish_pub(pub_name,msg)  

    def set_pt_offsets(self,pt_offsets):
        pub_name = 'set_auto_pan_window'
        msg = pt_offsets
        self.con_node_if.publish_pub(pub_name,msg)  

    def save_config(self):
        self.con_node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config',Empty())

    #################
    ## Save Data Functions

    def get_save_data_products(self):
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_save_data_status_dict(self):
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def save_data_enable_pub(self,enable):
        self.con_save_data_if.save_data_pub(enable)

    def save_data_prefix_pub(self,prefix):
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def save_data_snapshot_pub(self):
        self.con_save_data_if.publish_pub()

    def save_data_reset_pub(self):
        self.con_save_data_if.publish_pub(pub_name,msg)

    def save_data_factory_reset_pub(self):
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
            self.msg_if.pub_warn("Unregistering topic: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.status_connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg
        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)

    def _navposeCb(self,navpose_msg):      
        self.navpose_msg = navpose_msg
        if self.navposeCb is not None:
            navpose_dict = self.get_navpose_dict()
            self.navposeCb(navpose_dict)
