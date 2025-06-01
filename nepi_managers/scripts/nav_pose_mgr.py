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


from nepi_ros_interfaces.msg import NavPoseData

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


class NavPosePublisher(object):
  MGR_NODE_NAME = 'nav_pose_mgr'

  NAVPOSE_PUB_RATE_OPTIONS = [1.0,20.0] 
  NAVPOSE_3D_FRAME_OPTIONS = ['ENU','NED']
  NAVPOSE_ALT_FRAME_OPTIONS = ['AMSL','WGS84']

  FACTORY_PUB_RATE_HZ = 1
  FACTORY_3D_FRAME = 'ENU'
  FACTORY_ALT_FRAME = 'WGS84'

  data_products_list = ['navpose']
  last_navpose = None

  mgr_namespace = ""

  set_pub_rate = FACTORY_PUB_RATE_HZ
  set_3d_frame = FACTORY_3D_FRAME
  set_alt_frame = FACTORY_ALT_FRAME
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
    # Initialize Class Variables
    self.mgr_namespace = nepi_ros.create_namespace(self.base_namespace, self.MGR_NODE_NAME)


    ##############################
    ## Connect NEPI NavPose Manager
    nav_mgr_if = ConnectMgrNavPoseIF()
    connected = nav_mgr_if.wait_for_services()
  
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
        'navpose_pub': {
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
        '3d_frame': {
            'namespace': self.mgr_namespace,
            'topic': 'set_3d_frame',
            'msg': String,
            'qsize': 1,
            'callback': self.set3dFrameCb, 
            'callback_args': ()
        },
        'alt_frame': {
            'namespace': self.mgr_namespace,
            'topic': 'set_alt_frame',
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

    ready = self.node_if.wait_for_ready()

    ##############################
    ## Initialize From Param Server
    self.initCb(do_updates = True)


    ##############################
    # Set up save data services ########################################################
    factory_data_rates = {}
    for d in self.data_products_list:
        factory_data_rates[d] = [1.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
    self.save_data_if = SaveDataIF(data_products = self.data_products_list, factory_rate_dict = factory_data_rates,namespace = self.node_namespace)


    ######################
    # initialize variables from param server
    self.set_pub_rate = self.node_if.get_param('pub_rate')
    self.set_3d_frame = self.node_if.get_param('frame_3d')
    self.set_alt_frame = self.node_if.get_param('frame_alt')

    ##############################
    # Start Node Processes
    nepi_ros.timer(nepi_ros.ros_duration(1), self.navpose_get_publish_callback, oneshot = True)

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

    self.node_if.set_param('pub_rate',rate)

  def set3dFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_3D_FRAME_OPTIONS:
      self.set_3d_frame = frame
      self.node_if.set_param('frame_3d',frame)

  def setAltFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_ALT_FRAME_OPTIONS:
      self.set_frame_alt = frame
      self.node_if.set_param('frame_alt',frame)



  def initCb(self,do_updates = False):
      if do_updates == True:
        self.resetCb(do_updates)

  def resetCb(self,do_updates = False):
      pass
      
  def factoryResetCb(self,do_updates = False):
      pass


  ### Setup a regular background navpose get and publish timer callback
  def navpose_get_publish_callback(self,timer):
    ros_timestamp = nepi_ros.ros_time_now()
    # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call

    npdata_dict = self.get_navpose_solution()
    if npdata_dict is None:
      self.msg_if.pub_warn("Failed to convert navpose response: " + str(navpose_response))
      return
      
    self.save_data_if.save_dict2file('navpose',npdata_dict,ros_timestamp)

    # Setup nex update check
    self.last_navpose = navpose_response

    delay = float(1.0)/self.set_pub_rate
    nepi_ros.timer(delay, self.navpose_get_publish_callback, oneshot = True)


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

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPosePublisher()


