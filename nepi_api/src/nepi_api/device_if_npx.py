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

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.data_if import NavPoseIF


#########################################
# Node Class
#########################################


class NPXDeviceIF:

  NAVPOSE_PUB_RATE_OPTIONS = [1.0,40.0] 
  NAVPOSE_3D_FRAME_OPTIONS = ['ENU','NED']
  NAVPOSE_ALT_FRAME_OPTIONS = ['AMSL','WGS84']

  FACTORY_3D_FRAME = 'ENU'
  FACTORY_ALT_FRAME = 'WGS84'

  data_products_list = ['navpose']


  has_location = False
  has_position = False
  has_orientation = False
  has_heading = False

  last_nav_dict = None

  navpose_if = None  

  settings_if = None
  #######################
  ### IF Initialization
  def __init__(self, 
                device_info,
                capSettings=None, factorySettings=None, 
                settingUpdateFunction=None, getSettingsFunction=None,
                has_location = False, has_position = False, has_orientation = False, has_heading = False,
                getNavPoseDictFunction = None,
                pub_rate = 10):
    ####  IF INIT SETUP ####
    self.class_name = type(self).__name__
    self.base_namespace = nepi_ros.get_base_namespace()
    self.node_name = nepi_ros.get_node_name()
    self.node_namespace = os.path.join(self.base_namespace,self.node_name)

    ##############################  
    # Create Msg Class
    log_name = self.class_name
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting IF Initialization Processes")  

    ##############################
    # Initialize Class Variables
    self.has_location = has_location
    self.has_position = has_position
    self.has_orientation = has_orientation
    self.has_heading = has_heading


    self.initCb(do_updates = False)

    if pub_rate < .1:
      pub_rate = .1
    if pub_rate > 50:
      pub_rate = 50
    self.factory_pub_rate_hz = pub_rate


    # Test get dict funciton
    test_dict = None
    test_msg = None
    if getNavPoseDictFunction is not None:
      try:
          test_dict = getNavPoseDictFunction()
          test_msg = nepi_nav.convert_navposedata_dict2msg(test_dict)
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get data function: " + str(e))
    if test_msg is None:
      return

    # All Good

    npx_namespace = nepi_ros.create_namespace(self.node_namespace,'npx','navpose')
    self.navpose_if = NavPoseIF(namespace = npx_namespace,
                                has_location = has_location,  
                                has_orientation = has_orientation,
                                has_position = has_position, 
                                has_heading = has_heading )

    # Create a navpose capabilities service
    self.navpose_capabilities_report = NavPoseCapabilitiesQueryResponse()
    self.navpose_capabilities_report.has_location = has_location
    self.navpose_capabilities_report.has_position = has_position
    self.navpose_capabilities_report.has_orientation = has_orientation
    self.navpose_capabilities_report.has_heading = has_heading


    self.status_msg = NPXStatus()

    ##################################################
    ### Node Class Setup

    self.save_cfg_if = SaveCfgIF(initCb=self.initCb, resetCb=self.resetCb,  factoryResetCb=self.factoryResetCb)


    # Configs Config Dict ####################
    self.CFGS_DICT = {
            'init_callback': None,
            'reset_callback': None,
            'factory_reset_callback': None,
            'init_configs': True,
            'namespace': self.node_namespace
    }



    # Params Config Dict ####################
    self.init_pub_rate = rospy.get_param("~pub_rate",self.factory_pub_rate_hz)
    self.init_frame_3d = rospy.get_param("~frame_3d",self.FACTORY_3D_FRAME)
    self.init_frame_alt = rospy.get_param("~frame_alt",self.FACTORY_ALT_FRAME)

    self.PARAMS_DICT = {
        'param1_name': {
            'namespace': self.node_namespace,
            'factory_val': 100
        },
        'param2_name': {
            'namespace': self.node_namespace,
            'factory_val': "Something"
        }
    }

    # Services Config Dict ####################
    rospy.Service('~npx/navpose_capabilities_query', self.navposeCapabilitiesHandler)

    self.SRVS_DICT = {
        'service_name': {
            'namespace': self.node_namespace,
            'topic': 'empty_query',
            'svr': EmptySrv,
            'req': EmptySrvRequest(),
            'resp': EmptySrvResponse(),
            'callback': self.CALLBACK_FUNCTION
        }
    }


    # Publishers Config Dict ####################
    self.status_pub = rospy.Publisher('~npx/status', NPXStatus, queue_size=1, latch=True)

    self.PUBS_DICT = {
        'pub_name': {
            'namespace': self.node_namespace,
            'topic': 'set_empty',
            'msg': EmptyMsg,
            'qsize': 1,
            'latch': False
        }
    }


    rospy.Subscriber('~npx/set_pub_rate', Float32, self.setPublishRateCb, queue_size=1) # start local callback
    rospy.Subscriber('~npx/publish_once', Empty, self.publishOnceCb, queue_size=1) # start local callback
    rospy.Subscriber('~npx/set_3d_frame', String, self.set3dFrameCb, queue_size=1) # start local callback
    rospy.Subscriber('~npx/set_alt_frame', String, self.setAltFrameCb, queue_size=1) # start local callback


    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'sub_name': {
            'namespace': self.node_namespace,
            'topic': 'set_empty',
            'msg': EmptyMsg,
            'qsize': 1,
            'callback': self.SUB_CALLBACK, 
            'callback_args': ()
        }
    }


    # Create Node Class ####################
    self.NODE_IF = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    services_dict = self.SRVS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT,
                    log_class_name = True
    )

    ready = self.NODE_IF.wait_for_ready()


    # Setup Settings IF Class ####################
    if capSettings is not None:
      self.SETTINGS_DICT = {
                  'capSettings': capSettings, 
                  'factorySettings': factorySettings,
                  'setSettingFunction': settingUpdateFunction, 
                  'getSettingsFunction': getSettingsFunction, 
                  namespace='~'
      }
    else:
      self.SETTINGS_DICT = {
                  'capSettings': nepi_settings.NONE_CAP_SETTINGS, 
                  'factorySettings': nepi_settings.NONE_SETTINGS,
                  'setSettingFunction': nepi_settings.UPDATE_NONE_SETTINGS_FUNCTION, 
                  'getSettingsFunction': nepi_settings.GET_NONE_SETTINGS_FUNCTION, 
                  namespace='~'
      }
    self.settings_if = SettingsIF(self.SETTINGS_DICT)


    # Setup Save Data IF Class ####################
    factory_data_rates = {}
    for d in self.data_products_list:
        factory_data_rates[d] = [1.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz

    factory_filename_dict = {
        'prefix': "", 
        'add_timestamp': True, 
        'add_ms': True,
        'add_ns': False,
        'suffix': "",
        'add_node_name': True
        }

    self.save_data_if = SaveDataIF(data_product_names = self.data_products_list,
                                  factory_data_rate_dict = factory_data_rates,
                                  factory_filename_dict = factory_filename_dict)


    time.sleep(1)

    ###############################
    # Finish Initialization
    self.initCb(do_updates = True)
    self.publish_once()
    self.ready = True
    self.msg_if.pub_info("IF Initialization Complete")



  ###############################
  # Class Methods

  def check_ready(self):
      return self.ready  

  def wait_for_ready(self, timout = float('inf') ):
      success = False
      if self.ready is not None:
          self.msg_if.pub_info("Waiting for connection")
          timer = 0
          time_start = nepi_ros.get_time()
          while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
              nepi_ros.sleep(.1)
              timer = nepi_ros.get_time() - time_start
          if self.ready == False:
              self.msg_if.pub_info("Failed to Connect")
          else:
              self.msg_if.pub_info("Connected")
      return self.ready   


  def publish_once(self):
    nepi_ros.timer(nepi_ros.ros_duration(0.1), self.publishNavPoseCb, oneshot = True)
    return True


  def publish_status(self):
      self.status_msg.pub_rate = rospy.get_param("~pub_rate",self.init_pub_rate)
      self.status_msg.frame_3d = rospy.get_param("~frame_3d",self.init_frame_3d)
      self.status_msg.frame_alt = rospy.get_param("~frame_alt",self.init_frame_alt)
      self.status_pub.publish(self.status_msg)

    ###############################
    # Class Private Methods
    ###############################

  def navposeCapabilitiesHandler(self, _):
    return self.navpose_capabilities_report    


  def publishOnceCb(self,msg):
    self.publish_once()

  def publishNavPoseCb(self,timer):
    nav_dict = None
    try:
        nav_dict = self.getNavPoseDictFunction()
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
        rospy.timer(nepi_ros.ros_duration(delay), self.publishNavPoseCb, oneshot = True)
            

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


