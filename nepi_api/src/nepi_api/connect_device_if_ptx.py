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
import math
import numpy as np


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_sdk_interfaces.msg import RangeWindow
from nepi_sdk_interfaces.msg import PTXStatus, PanTiltLimits, PanTiltOffsets, PanTiltPosition, SingleAxisTimedMove
from nepi_sdk_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryRequest, PTXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.device_if_npx import NPXDeviceIF

from nepi_api.connect_node_if import ConnectNodeClassIF

#########################################
# Node Class
#########################################

APP_NODE_NAME = 'app_file_pub_img'

class ConnectAppFilePubImgIF:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

 
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
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


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CFGS_DICT = {
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
            },
            'set_pt_offsets': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_pt_offsets',
                'msg': PanTiltOffsets,
                'qsize': 1,
            }


        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'joint_pub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/joint_states',
                'msg': JointState,
                'qsize': 10,
                'callback': self._statusCb
            }
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/joint_states',
                'msg': PTXStatus,
                'qsize': 10,
                'callback': self._statusCb
            }
        }


        # Create Node Class ####################
        
        self.con_node_if = ConnectNodeClassIF(
                        namespace = self.namespace,
                        configs_dict = self.CFGS_DICT,
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
            if self.connected == False:folder_name
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
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return self.img_status_dict

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
