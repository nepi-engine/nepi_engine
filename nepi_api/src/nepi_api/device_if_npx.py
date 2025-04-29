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

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_ros_interfaces.msg import NPXStatus
from nepi_ros_interfaces.msg import NavPose, NavPoseData
from nepi_ros_interfaces.srv import  NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryRequest, NavPoseCapabilitiesQueryResponse


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.data_if import NavPoseIF



EXAMPLE_NAVPOSE_DATA_DICT = {

                          'frame_3d': 'ENU',
                          'frame_alt': 'WGS84',

                          'geoid_height_meters': 0,

                          'has_heading': True,
                          'time_heading': nepi_utils.get_time(),
                          'heading_deg': 120.50,

                          'has_oreientation': True,
                          'time_oreientation': nepi_utils.get_time(),
                          # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
                          'roll_deg': 30.51,
                          'pitch_deg': 30.51,
                          'yaw_deg': 30.51,

                          'has_position': True,
                          'time_position': nepi_utils.get_time(),
                          # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
                          'x_m': 1.234,
                          'y_m': 1.234,
                          'z_m': 1.234,

                          'has_location': True,
                          # Global Location in set altitude frame (lat,long,alt) with alt in meters
                          'lat': 47.080909,
                          'long': -120.8787889,

                          'has_altitude': True,
                          'alt_m': 12.321,
                          'has_depth': False,
                          'alt_m': 0
}

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
                has_heading = False, has_position = False, has_orientation = False, 
                has_location = False, has_altitude = False, has_depth = False,
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

    self.msg_if.pub_info("Starting Node IF Initialization")
    # Configs Config Dict ####################
    self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace':  self.node_namespace
    }



    # Params Config Dict ####################

    self.PARAMS_DICT = {
        'pub_rate': {
            'namespace': self.node_namespace,
            'factory_val': self.factory_pub_rate_hz
        },
        'frame_3d': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_3D_FRAME
        },
        'frame_alt': {
            'namespace': self.node_namespace,
            'factory_val': self.FACTORY_ALT_FRAME
        }        
    }

    # Services Config Dict ####################

    self.SRVS_DICT = {
        'navpose_capabilities_query': {
            'namespace': self.node_namespace,
            'topic': 'npx/navpose_capabilities_query',
            'srv': NavPoseCapabilitiesQuery,
            'req': NavPoseCapabilitiesQueryRequest(),
            'resp': NavPoseCapabilitiesQueryResponse(),
            'callback': self.navposeCapabilitiesHandler
        }
    }

    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'npx/status',
            'msg': NPXStatus,
            'qsize': 1,
            'latch': True
        }
    }


    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'set_pub_rate': {
            'namespace': self.node_namespace,
            'topic': 'npx/set_pub_rate',
            'msg': Float32,
            'qsize': 1,
            'callback': self.setPublishRateCb, 
            'callback_args': ()
        },
        'publish_once': {
            'namespace': self.node_namespace,
            'topic': 'npx/publish_once',
            'msg': Empty,
            'qsize': 1,
            'callback': self.publishOnceCb, 
            'callback_args': ()
        },
        'set_3d_frame': {
            'namespace': self.node_namespace,
            'topic': 'npx/set_3d_frame',
            'msg': String,
            'qsize': 1,
            'callback': self.set3dFrameCb, 
            'callback_args': ()
        },
        'set_alt_frame': {
            'namespace': self.node_namespace,
            'topic': 'npx/set_alt_frame',
            'msg': String,
            'qsize': 1,
            'callback': self.setAltFrameCb, 
            'callback_args': ()
        }


    }


    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    services_dict = self.SRVS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT,
                    log_class_name = True
    )

    ready = self.node_if.wait_for_ready()


    # Setup Settings IF Class ####################
    self.msg_if.pub_info("Starting Settings IF Initialization")
    if capSettings is not None:
      self.SETTINGS_DICT = {
                  'capSettings': capSettings, 
                  'factorySettings': factorySettings,
                  'setSettingFunction': settingUpdateFunction, 
                  'getSettingsFunction': getSettingsFunction, 
                  'namespace': self.node_namespace
      }
    else:
      self.SETTINGS_DICT = {
                  'capSettings': nepi_settings.NONE_CAP_SETTINGS, 
                  'factorySettings': nepi_settings.NONE_SETTINGS,
                  'setSettingFunction': nepi_settings.UPDATE_NONE_SETTINGS_FUNCTION, 
                  'getSettingsFunction': nepi_settings.GET_NONE_SETTINGS_FUNCTION, 
                  'namespace': self.node_namespace
      }
    self.settings_if = SettingsIF(self.SETTINGS_DICT)


    # Setup Save Data IF Class ####################
    self.msg_if.pub_info("Starting Save Data IF Initialization")
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

    self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                  factory_rate_dict = factory_data_rates,
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
      self.status_msg.pub_rate = self.nepi_ros.get_param('pub_rate')
      self.status_msg.frame_3d = self.nepi_ros.get_param('frame_3d')
      self.status_msg.frame_alt = self.nepi_ros.get_param('frame_alt')
      self.node_if.publish_pub('status_pub',self.status_msg)

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
        set_pub_rate = self.nepi_ros.get_param('pub_rate')
        set_3d_frame = self.nepi_ros.get_param('frame_3d')
        set_alt_frame = self.nepi_ros.get_param('frame_alt')
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

        self.save_data_if.save_dict2file('navpose',nav_dict,ros_timestamp)

    if self.pub_rate != 0:
        delay = float(1) / self.pub_rate
        self.nepi_ros.start_timer_process(nepi_ros.ros_duration(delay), self.publishNavPoseCb, oneshot = True)
            

  def setPublishRateCb(self,msg):
    rate = msg.data
    min = self.NAVPOSE_PUB_RATE_OPTIONS[0]
    max = self.NAVPOSE_PUB_RATE_OPTIONS[1]
    if rate < min:
      rate = min
    if rate > max:
      rate = max
    self.nepi_ros.set_param('pub_rate',rate)

  def set3dFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_3D_FRAME_OPTIONS:
      self.nepi_ros.set_param('frame_3d',frame)

  def setAltFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_ALT_FRAME_OPTIONS:
      self.nepi_ros.set_param('frame_alt',frame)

  def initConfig(self):
      self.initCb(do_updates = True)

  def initCb(self,do_updates = False):
      self.msg_if.pub_info("Setting init values to param values")

      if do_updates == True:
        self.resetCb(do_updates)

  def resetCb(self,do_updates = True):
      self.nepi_ros.set_param('pub_rate',self.init_pub_rate)
      self.nepi_ros.set_param('frame_3d',self.init_frame_3d)
      self.nepi_ros.set_param('frame_alt',self.init_frame_alt)

  def factoryresetCb(self,do_updates = True):
      self.nepi_ros.set_param('pub_rate',self.init_pub_rate)
      self.nepi_ros.set_param('frame_3d',self.init_frame_3d)
      self.nepi_ros.set_param('frame_alt',self.init_frame_alt)


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPosePublisher()


