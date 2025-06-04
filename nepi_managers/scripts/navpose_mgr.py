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


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_ros_interfaces.msg import NavPoseMgrStatus

from nepi_ros_interfaces.msg import UpdateTopic, UpdateNavPoseTopic, UpdateFrame3DTransform

from nepi_ros_interfaces.msg import NavPoseData, NavPoseDataStatus
from nepi_ros_interfaces.msg import NavPoseLocation, NavPoseHeading
from nepi_ros_interfaces.msg import NavPoseOrienation, NavPoseLocation
from nepi_ros_interfaces.msg import NavPoseAltitude, NavPoseDepth

from nepi_ros_interfaces.srv import NavPoseDataQuery, NavPoseDataQueryRequest, NavPoseDataQueryResponse

from nepi_ros_interfaces.msg import Frame3DTransform, Frame3DTransforms
from nepi_ros_interfaces.srv import Frame3DTransformsQuery, Frame3DTransformsQueryRequest, Frame3DTransformsQueryResponse
from nepi_ros_interfaces.srv import Frame3DTransformsRegister, Frame3DTransformsRegisterRequest, Frame3DTransformsRegisterResponse
from nepi_ros_interfaces.srv import Frame3DTransformsDelete, Frame3DTransformsDeleteRequest, Frame3DTransformsDeleteResponse

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
    FACTORY_3D_FRAME = 'nepi_frame' 
    FACTORY_NAV_FRAME = 'ENU'
    FACTORY_ALT_FRAME = 'WGS84'

    ZERO_TRANSFORM = [0,0,0,0,0,0,0]

    data_products_list = ['navpose']
    last_npdata_dict = None

    mgr_namespace = ""
    status_msg = NavPoseMgrStatus()

    set_pub_rate = FACTORY_PUB_RATE_HZ

    time_list = [0,0,0,0,0,0,0]

    connect_dict {
        'location': {
            'set_topic': "",
            'sub_topic': "",
            'sub': None,
            'transform': ZERO_TRANSFORM,
            'times': time_list,
            'last_time': 0.0
        },
        'heading': {
            'set_topic': "",
            'sub_topic': "",
            'sub': None,
            'transform': ZERO_TRANSFORM,
            'times': time_list,
            'last_time': 0.0
        },
        'orientation': {
            'set_topic': "",
            'sub_topic': "",
            'sub': None,
            'transform': ZERO_TRANSFORM,
            'times': time_list,
            'last_time': 0.0
        },
        'position': {
            'set_topic': "",
            'sub_topic': "",
            'sub': None,
            'transform': ZERO_TRANSFORM,
            'times': time_list,
            'last_time': 0.0
        },
        'altitude': {
            'set_topic': "",
            'sub_topic': "",
            'sub': None,
            'transform': ZERO_TRANSFORM,
            'times': time_list,
            'last_time': 0.0
        },
        'depth': {
            'set_topic': "",
            'sub_topic': "",
            'sub': None,
            'transform': ZERO_TRANSFORM,
            'times': time_list,
            'last_time': 0.0
        }
    }

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "nav_pose_mgr" # Can be overwitten by luanch command
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
        # Initialize Class Variables
        self.mgr_namespace = nepi_ros.create_namespace(self.base_namespace, self.MGR_NODE_NAME)


        self.status_msg.publishing = False
        self.status_msg.pub_rate = self.set_pub_rate

        self.status_msg.has_heading = False
        self.status_msg.has_position = False
        self.status_msg.has_orientation = False
        self.status_msg.has_location = False
        self.status_msg.has_altitude = False
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
            'connect_dict': {
                'namespace': self.mgr_namespace,
                'factory_val': self.connect_dict
            },

        }

        # Services Config Dict ####################
        self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.mgr_namespace,
                'topic': 'status',
                'msg': NavPoseMgrStatus,
                'qsize': 1,
                'latch': True
            },
            'navpose_pub': {
                'namespace': self.base_namespace
                'topic': 'navpose',
                'msg': NavPoseData,
                'qsize': 1,
                'latch': True
            },

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
            'set_topic': {
                'namespace': self.mgr_namespace,
                'topic': 'set_topic',
                'msg': UpdateNavPoseTopic,
                'qsize': 1,
                'callback': self.setTopicCb, 
                'callback_args': ()
            },
            'set_transform': {
                'namespace': self.mgr_namespace,
                'topic': 'set_transform',
                'msg': UpdateFrame3DTransformTopic,
                'qsize': 1,
                'callback': self.setTransformCb, 
                'callback_args': ()
            },
            'set_fixed': {
                'namespace': self.mgr_namespace,
                'topic': 'set_pub_rate',
                'msg': NavPoseData,
                'qsize': 1,
                'callback': self.setFixedCb, 
                'callback_args': ()
            },
            UpdateTopic, UpdateFrame3DTransform
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
        self.connect_dict = self.node_if.get_param('connect_dict')

        self.initCb(do_updates = True)
        ##############################
        # Start Node Processes
        nepi_ros.start_timer_process(1.0, self._getPublishSaveDataCb, oneshot = True)
        nepi_ros.start_timer_process(5.0, self._getAvailTopicsCb, oneshot = True)
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

    def setTopic(self,msg):
        frame = msg.data
        if frame in self.NAVPOSE_3D_FRAME_OPTIONS:
            self.set_frame_3d = frame
            self.publish_status()
            self.node_if.set_param('frame_3d',frame)

    def setTransformCb(self,msg):
        frame = msg.data
        if frame in self.NAVPOSE_3D_FRAME_OPTIONS:
            self.set_frame_nav = frame
            self.publish_status()
            self.node_if.set_param('frame_nav',frame)

    def setFixedCb(self,msg):
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


    def publish_navpose(self):
        navpose_msg = nepi_nav.convert_navpose_dict2msg(self.navpose_dict)
        self.node_if.publish_pub('navpose_pub',navpose_msg)

    def publish_status(self, do_updates = False):
        self.status_msg.pub_rate = self.set_pub_rate
        #self.msg_if.pub_warn("will publish status msg: " + str(self.status_msg))
        self.node_if.publish_pub('navpose_data_status',self.status_msg)

    #######################
    # Private Members
    #######################

    def _getAvailTopicsCb(self,timer):

        self.status_msg.available_location_topics = nepi_nav.get_location_publisher_namespaces()
        self.status_msg.available_heading_topics = nepi_nav.get_heading_publisher_namespaces()
        self.status_msg.available_orientation_topics = nepi_nav.get_orientation_publisher_namespaces()
        self.status_msg.available_position_topics = nepi_nav.get_position_publisher_namespaces()
        self.status_msg.available_altitude_topics = nepi_nav.get_altitude_publisher_namespaces()
        self.status_msg.available_depth_topics = nepi_nav.get_depth_publisher_namespaces()

        nepi_ros.start_timer_process(5.0, self._getAvailTopicsCb, oneshot = True)


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

    def _publishNavPoseCb(self,timer):
        self.publish_status()

    def _publishStatusCb(self,timer):
        self.publish_status()


    def _cleanupActions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPoseMgr()


