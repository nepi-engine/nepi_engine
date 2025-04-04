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
import rospy
import numpy as np
import math
import time
import sys
import tf
import yaml

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_ros_interfaces.msg import NPXStatus
from nepi_ros_interfaces.msg import NavPose, NavPoseData
from nepi_ros_interfaces.srv import  NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_save
from nepi_sdk import nepi_nav

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF
from nepi_api.data_if_navpose import NavPoseIF
from nepi_api.sys_if_settings import SettingsIF
from nepi_api.sys_if_save_data import SaveDataIF
from nepi_api.sys_if_save_cfg import SaveCfgIF

#########################################
# Node Class
#########################################


class NPXDeviceIF:

  NAVPOSE_PUB_RATE_OPTIONS = [1.0,40.0] 
  NAVPOSE_3D_FRAME_OPTIONS = ['ENU','NED']
  NAVPOSE_ALT_FRAME_OPTIONS = ['AMSL','WGS84']

  FACTORY_3D_FRAME = 'ENU'
  FACTORY_ALT_FRAME = 'WGS84'

  data_products = ['navpose']

  init_pub_rate = 0
  init_frame_3d = FACTORY_3D_FRAME
  init_frame_alt = FACTORY_ALT_FRAME

  has_loc = False
  has_pose = False
  has_head = False

  last_nav_dict = None

  navpose_if = None  
  #######################
  ### IF Initialization
  def __init__(self, 
                device_info,
                get_navpose_function,
                pub_rate = 10):
    ####  IF INIT SETUP ####
    self.class_name = type(self).__name__
    self.base_namespace = nepi_ros.get_base_namespace()
    self.node_name = nepi_ros.get_node_name()
    self.node_namespace = os.path.join(self.base_namespace,self.node_name)

    ##############################  
    # Create Msg Class
    log_name = self.class_name
    self.msg_if = MsgIF(log_name = log_name)
    self.msg_if.pub_info("Starting IF Initialization Processes")  

    ##############################
    self.initCb(do_updates = False)

    if pub_rate < .1:
      pub_rate = .1
    if pub_rate > 50:
      pub_rate = 50
    self.factory_pub_rate_hz = pub_rate


    # Test get dict funciton
    test_msg = None
    if get_navpose_function is not None:
      try:
          test_dict = get_navpose_function()
          test_msg = nepi_nav.convert_navposedata_dict2msg(test_dict)
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get data function: " + str(e))
    if test_msg is None:
      return

    # All Good
    self.namespace = os.path.join(self.base_namespace,self,node_name)
    self.get_navpose_function = get_navpose_function

    self.navpose_if = NavPoseIF(data_product = 'navpose', pub_namespace = '~npx')

    # Create a navpose capabilities service
    self.navpose_capabilities_report = NavPoseCapabilitiesQueryResponse()
    self.navpose_capabilities_report.has_gps = self.has_location
    self.navpose_capabilities_report.has_orientation = self.has_pose
    self.navpose_capabilities_report.has_heading = self.has_heading

    rospy.Service('~npx/navpose_capabilities_query', self._navposeCapabilitiesHandler)

    # Set Up Publishers
    self.status_msg = NPXStatus()
    self.status_pub = rospy.Publisher('~npx/status', NPXStatus, queue_size=1, latch=True)
    time.sleep(1)

    ## Start Class Subscribers
    rospy.Subscriber('~npx/set_pub_rate', Float32, self.setPublishRateCb, queue_size=1) # start local callback
    rospy.Subscriber('~npx/publish_once', Empty, self.publishOnceCb, queue_size=1) # start local callback
    rospy.Subscriber('~npx/set_3d_frame', String, self.set3dFrameCb, queue_size=1) # start local callback
    rospy.Subscriber('~npx/set_alt_frame', String, self.setAltFrameCb, queue_size=1) # start local callback
    ## Set up save and config interfaces

    factory_data_rates = {}
    for d in self.data_products:
        factory_data_rates[d] = [1.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
    self.save_data_if = SaveDataIF(data_product_names = self.data_products, factory_data_rate_dict = factory_data_rates)
    # Temp Fix until added as NEPI ROS Node
    self.save_cfg_if = SaveCfgIF(initCb=self.initCb ,resetCb=self.resetCb, factoryResetCb=self.factoryresetCb)


    ## Initialize From Param Server
    self.initCb(do_updates = True)
    ## Initiation Complete

    self.publish_once()

    self.ready = True

    self.msg_if.pub_info("Initialization Complete")
    nepi_ros.spin()


  ###############################
  # Class Public Methods
  ###############################

  def check_ready(self):
      return self.ready  

  def publish_once(self):
    nepi_ros.timer(nepi_ros.ros_duration(0.1), self._publishCb, oneshot = True)
    return True


  def publishStatus(self):
      self.status_msg.pub_rate = rospy.get_param("~pub_rate",self.init_pub_rate)
      self.status_msg.frame_3d = rospy.get_param("~frame_3d",self.init_frame_3d)
      self.status_msg.frame_alt = rospy.get_param("~frame_alt",self.init_frame_alt)
      self.status_pub.publish(self.status_msg)

    ###############################
    # Class Private Methods
    ###############################

  def _navposeCapabilitiesHandler(self, _):
    return self.navpose_capabilities_report    



  def _publishOnceCb(self,msg):
    self.publish_once()

  def _publishCb(self,timer):
    nav_dict = None
    try:
        nav_dict = self.get_navpose_function()
    except Exception as e:
        self.msg_if.pub_warn("Failed to call get navpose dict function: " + str(e))
    
    if nav_dict is not None:
      if self.last_nav_dict != nav_dict:
        self.last_nav_dict = nav_dict
        ros_timestamp = nepi_ros.ros_time_now()
        set_pub_rate = rospy.get_param("~pub_rate",self.init_pub_rate)
        set_3d_frame = rospy.get_param("~frame_3d",self.init_frame_3d)
        set_alt_frame = rospy.get_param("~frame_alt",self.init_frame_alt)
        # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call

        # Adjust alt Frame if Needed
        got_f = nav_dict['frame_alt']
        if got_f not in self.NAVPOSE_ALT_FRAME_OPTIONS:
          nav_dict['frame_alt'] = 'WGS84'

        f = nav_dict['frame_alt']
        if f != set_alt_frame:
          if set_alt_frame == 'WGS84':
            nav_dict = nepi_nav.convert_navposedata_amsl2wgs84(nav_dict)
          else:
            nav_dict = nepi_nav.convert_navposedata_wgs842amsl(nav_dict)


        # Adjust 3d Frame if Needed
        got_f = nav_dict['frame_3d']
        if got_f not in self.NAVPOSE_3D_FRAME_OPTIONS:
          nav_dict['frame_3d'] = 'ENU'
        
        f = nav_dict['frame_3d']
        if f != set_3d_frame:
          if set_3d_frame == 'ENU':
            nav_dict = nav_dict = nepi_nav.convert_navposedata_ned2edu(nav_dict)
          else:
            nav_dict = nav_dict = nepi_nav.convert_navposedata_enu2ned(nav_dict)
                        

        # Pub NavPoseData
        try:
          self.navpose_if.publish_navpose_dict(nav_dict)
        except Exception as e:
          self.msg_if.pub_warn("Failed to publish navpose data msg: " + str(e))

        nepi_save.save_dict2file('navpose',nav_dict,ros_timestamp)

    if self.pub_rate != 0:
        delay = float(1) / self.pub_rate
        rospy.timer(nepi_ros.ros_duration(delay), self._publishCb, oneshot = True)
            

  def setPublishRateCb(self,msg):
    rate = msg.data
    min = self.NAVPOSE_PUB_RATE_OPTIONS[0]
    max = self.NAVPOSE_PUB_RATE_OPTIONS[1]
    if rate < min:
      rate = min
    if rate > max:
      rate = max
    rospy.set_param("~pub_rate",rate)

  def set3dFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_3D_FRAME_OPTIONS:
      rospy.set_param("~frame_3d",frame)

  def setAltFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_ALT_FRAME_OPTIONS:
      rospy.set_param("~frame_alt",frame)

  def initConfig(self):
      self.initCb(do_updates = True)

  def initCb(self,do_updates = False):
      self.msg_if.pub_info("Setting init values to param values")
      self.init_pub_rate = rospy.get_param("~pub_rate",self.factory_pub_rate_hz)
      self.init_frame_3d = rospy.get_param("~frame_3d",self.FACTORY_3D_FRAME)
      self.init_frame_alt = rospy.get_param("~frame_alt",self.FACTORY_ALT_FRAME)
      if do_updates == True:
        self.resetCb(do_updates)

  def resetCb(self,do_updates = True):
      rospy.set_param("~pub_rate",self.init_pub_rate)
      rospy.set_param("~frame_3d",self.init_frame_3d)
      rospy.set_param("~frame_alt",self.init_frame_alt)

  def factoryresetCb(self,do_updates = True):
      rospy.set_param("~pub_rate",self.init_pub_rate)
      rospy.set_param("~frame_3d",self.init_frame_3d)
      rospy.set_param("~frame_alt",self.init_frame_alt)


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPosePublisher()


