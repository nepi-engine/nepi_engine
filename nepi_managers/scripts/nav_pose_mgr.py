#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

import os
import numpy as np
import math
import time
import sys
import tf
import yaml

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_ros_interfaces.msg import NavPoseData, NavPoseDataStatus

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.connect_mgr_if_nav_pose import ConnectMgrNavPoseIF
from nepi_api.system_if import SaveDataIF

#########################################
# Node Class
#########################################


class NavPoseMgr(object):
    MGR_NODE_NAME = 'nav_pose_mgr'

    NAVPOSE_PUB_RATE_OPTIONS = [1.0,20.0] 
    NAVPOSE_3D_FRAME_OPTIONS = ['ENU','NED']
    NAVPOSE_ALT_FRAME_OPTIONS = ['AMSL','WGS84']

    FACTORY_PUB_RATE_HZ = 5.0
    FACTORY_ID_FRAME = 'nepi_base_frame' 
    FACTORY_3D_FRAME = 'ENU'
    FACTORY_ALT_FRAME = 'WGS84'

    data_products_list = ['navpose']
    last_npdata_dict = None

    mgr_namespace = ""

    set_pub_rate = FACTORY_PUB_RATE_HZ
    set_frame_id = FACTORY_ID_FRAME
    set_frame_3d = FACTORY_3D_FRAME
    set_frame_alt = FACTORY_ALT_FRAME

    status_msg = NavPoseDataStatus()
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "nav_pose_mgr2" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        ## Connect NEPI NavPose Manager
        self.nav_mgr_if = ConnectMgrNavPoseIF()
        connected = self.nav_mgr_if.wait_for_services()

        ##############################
        # Initialize Class Variables
        self.mgr_namespace = nepi_ros.create_namespace(self.base_namespace, self.MGR_NODE_NAME)

        self.status_msg.publishing = False
        self.status_msg.pub_rate = self.set_pub_rate
        self.status_msg.frame_id = self.set_frame_id
        self.status_msg.frame_3d = self.set_frame_3d
        self.status_msg.frame_altitude = self.set_frame_alt

        self.status_msg.has_heading = True
        self.status_msg.has_position = True
        self.status_msg.has_orientation = True
        self.status_msg.has_location = True
        self.status_msg.has_altitude = True
        self.status_msg.has_depth = False


        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace': self.mgr_namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'pub_rate': {
                'namespace': self.mgr_namespace,
                'factory_val': self.FACTORY_PUB_RATE_HZ
            },
            'frame_id': {
                'namespace': self.mgr_namespace,
                'factory_val': self. FACTORY_ID_FRAME
            },
            'frame_3d': {
                'namespace': self.mgr_namespace,
                'factory_val': self.FACTORY_3D_FRAME
            },
            'frame_alt': {
                'namespace': self.mgr_namespace,
                'factory_val': self.FACTORY_ALT_FRAME
            },
        }

        # Services Config Dict ####################     
        self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'navpose_data_status': {
                'namespace': self.mgr_namespace,
                'topic': 'navpose/status',
                'msg': NavPoseDataStatus,
                'qsize': 1,
                'latch': True
            },
            'navpose_data': {
                'namespace': self.mgr_namespace,
                'topic': 'navpose',
                'msg': NavPoseData,
                'qsize': 1,
                'latch': True
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'pub_rate': {
                'namespace': self.mgr_namespace,
                'topic': 'set_pub_rate',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setPublishRateCb, 
                'callback_args': ()
            },
            'frame_3d': {
                'namespace': self.mgr_namespace,
                'topic': 'set_frame_3d',
                'msg': String,
                'qsize': 1,
                'callback': self.set3dFrameCb, 
                'callback_args': ()
            },
            'frame_alt': {
                'namespace': self.mgr_namespace,
                'topic': 'set_frame_alt',
                'msg': String,
                'qsize': 1,
                'callback': self.setAltFrameCb, 
                'callback_args': ()
            },
        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT
        )

        #ready = self.node_if.wait_for_ready()
        nepi_ros.wait()




        ##############################
        # Set up save data services ########################################################
        factory_data_rates = {}
        for d in self.data_products_list:
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
            if d == 'navpose':
                factory_data_rates[d][0] = float(1.0) / self.FACTORY_PUB_RATE_HZ
        self.save_data_if = SaveDataIF(data_products = self.data_products_list, factory_rate_dict = factory_data_rates,namespace = self.node_namespace)


        ######################
        # initialize variables from param server
        self.set_pub_rate = self.node_if.get_param('pub_rate')
        self.set_frame_id = self.node_if.get_param('frame_id')
        self.set_frame_3d = self.node_if.get_param('frame_3d')
        self.set_frame_alt = self.node_if.get_param('frame_alt')

        self.initCb(do_updates = True)
        ##############################
        # Start Node Processes
        nepi_ros.start_timer_process(1.0, self._getPublishSaveDataCb, oneshot = True)
        nepi_ros.start_timer_process(1.0, self._publishStatusCb)

        ##############################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        nepi_ros.spin()


    #######################
    ### Node Methods

    def setPublishRateCb(self,msg):
        rate = msg.data
        min = self.NAVPOSE_PUB_RATE_OPTIONS[0]
        max = self.NAVPOSE_PUB_RATE_OPTIONS[1]
        if rate < min:
            rate = min
        if rate > max:
            rate = max
        self.set_pub_rate = rate
        self.publish_status()
        self.node_if.set_param('pub_rate',rate)

    def set3dFrameCb(self,msg):
        frame = msg.data
        if frame in self.NAVPOSE_3D_FRAME_OPTIONS:
            self.set_frame_3d = frame
            self.publish_status()
            self.node_if.set_param('frame_3d',frame)

    def setAltFrameCb(self,msg):
        frame = msg.data
        if frame in self.NAVPOSE_ALT_FRAME_OPTIONS:
            self.set_frame_alt = frame
            self.publish_status()
            self.node_if.set_param('frame_alt',frame)



    def initCb(self,do_updates = False):
        if do_updates == True:
            self.resetCb(do_updates)

    def resetCb(self,do_updates = False):
        pass
        
    def factoryResetCb(self,do_updates = False):
        pass


    def get_navpose_solution_status(self, verbose = True):
        navpose_status = self.nav_mgr_if.get_navpose_solution_status()
        return navpose_status  

    def publish_status(self, do_updates = False):
        self.status_msg.pub_rate = self.set_pub_rate
        self.status_msg.frame_id = self.set_frame_id
        self.status_msg.frame_3d = self.set_frame_3d
        self.status_msg.frame_altitude = self.set_frame_alt
        #self.msg_if.pub_warn("will publish status msg: " + str(self.status_msg))
        self.node_if.publish_pub('navpose_data_status',self.status_msg)

    #######################
    # Private Members
    #######################

    ### Setup a regular background navpose get and publish timer callback
    def _getPublishSaveDataCb(self,timer):
        timestamp = nepi_utils.get_time()
        # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
        npdata_dict = self.nav_mgr_if.get_navpose_data_dict()
        if npdata_dict is not None:
            #self.msg_if.pub_warn("Got navpose data dict: " + str(npdata_dict))
            npdata_msg = nepi_nav.convert_navposedata_dict2msg(npdata_dict)
            #self.msg_if.pub_warn("Got navpose data msg: " + str(npdata_msg))
            if npdata_msg is not None:
                self.status_msg.publishing = True
                self.node_if.publish_pub('navpose_data',npdata_msg)  
            if self.last_npdata_dict != npdata_dict:
                self.save_data_if.save('navpose',npdata_dict,timestamp)
            # Setup nex update check
            self.last_npdata_dict = npdata_dict

        delay = float(1.0)/self.set_pub_rate
        nepi_ros.start_timer_process(delay, self._getPublishSaveDataCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.publish_status()


    def _cleanupActions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPoseMgr()


