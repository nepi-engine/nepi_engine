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
import copy
import sys
import tf
import yaml

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from geographic_msgs.msg import GeoPoint

from nepi_ros_interfaces.msg import Frame3DTransform

from geometry_msgs.msg import Vector3

from nepi_ros_interfaces.msg import NPXStatus
from nepi_ros_interfaces.msg import NavPose, NavPoseData
from nepi_ros_interfaces.srv import  NPXCapabilitiesQuery, NPXCapabilitiesQueryRequest, NPXCapabilitiesQueryResponse


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_settings

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.data_if import NavPoseIF
from nepi_api.connect_mgr_if_nav_pose import ConnectMgrNavPoseIF

EXAMPLE_HEADING_DATA_DICT = {
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 120.50,
}

EXAMPLE_POSITION_DATA_DICT = {
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters ENU (x,y,z) with x forward, y left, and z up
    'x_m': 1.234,
    'y_m': 1.234,
    'z_m': 1.234,
}

EXAMPLE_ORIENTATION_DATA_DICT = {
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees ENU
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,
}

EXAMPLE_LOCATION_DATA_DICT = {
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'lat': 47.080909,
    'long': -120.8787889,
}

EXAMPLE_ALTITUDE_DATA_DICT = {
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters WGS84
    'altitude_m': 12.321,
}

EXAMPLE_DEPTH_DATA_DICT = {
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive distance from surface in meters
    'depth_m': 0
}

EXAMPLE_NAVPOSE_DATA_DICT = {
    'frame_id': 'nepi_base_frame',
    'frame_3d': 'ENU',
    'frame_3d': 'ENU',
    'frame_altitude': 'WGS84',

    'geoid_height_meters': 0,

    'has_heading': True,
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 120.50,

    'has_position': True,
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters in specified 3d frame (x,y,z) with x forward, y right/left, and z up/down
    'x_m': 1.234,
    'y_m': 1.234,
    'z_m': 1.234,

    'has_orientation': True,
    'time_oreientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,

    'has_location': True,
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'lat': 47.080909,
    'long': -120.8787889,

    'has_altitude': True,
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters in specified alt frame
    'altitude_m': 12.321,

    'has_depth': False,
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive meters
    'depth_m': 0.0
}

#########################################
# Node Class
#########################################


class NPXDeviceIF:

  MIN_PUB_RATE = 1.0
  NAVPOSE_FRAME_ID_OPTIONS = ['nepi_base_frame','sensor_frame']
  NAVPOSE_FRAME_3D_OPTIONS = ['ENU','NED']
  NAVPOSE_FRAME_ALT_OPTIONS = ['AMSL','WGS84','DEPTH']

  DEFAULT_UPDATE_RATE = 10
  DEFAULT_ID_FRAME = 'sensor_frame'
  DEFAULT_3D_FRAME = 'ENU'
  DEFAULT_ALT_FRAME = 'WGS84'

  ZERO_TRANSFORM = [0,0,0,0,0,0,0]

  data_products_list = ['navpose']

  node_if = None

  update_rate = DEFAULT_UPDATE_RATE
  frame_id = DEFAULT_ID_FRAME
  frame_3d = DEFAULT_3D_FRAME
  frame_altitude = DEFAULT_ALT_FRAME

  frame_transform = ZERO_TRANSFORM

  has_heading = False
  has_position = False
  has_orientation = False
  has_location = False  
  has_altitude = False
  has_depth = False

  navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT

  navpose_if = None  

  settings_if = None

  has_updated = False

  has_gps_pub = False
  has_elevation_pub = False
  has_pose_pub = False
  has_heading_pub = False

  set_gps_source = False
  set_elevation_source = False
  set_pose_source = False
  set_heading_source = False

  navpose_mgr_if = None
  #######################
  ### IF Initialization
  def __init__(self, 
                device_info,
                capSettings=None, factorySettings=None, 
                settingUpdateFunction=None, getSettingsFunction=None,
                frame_3d = None, frame_altitude = None,
                getHeadingCb = None, getPositionCb = None, getOrientationCb = None,
                getLocationCb = None, getAltitudeCb = None, getDepthCb = None,
                max_navpose_update_rate = DEFAULT_UPDATE_RATE,
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
        self.msg_if.pub_info("Starting NPX Device IF Initialization Processes", log_name_list = self.log_name_list)  





        ##############################
        # Initialize Class Variables
        self.gps_pub_ns = nepi_ros.create_namespace(self.node_namespace,'npx/gps')
        self.elevation_pub_ns = nepi_ros.create_namespace(self.node_namespace,'npx/elevation')
        self.pose_pub_ns = nepi_ros.create_namespace(self.node_namespace,'npx/pose')
        self.heading_pub_ns = nepi_ros.create_namespace(self.node_namespace,'npx/heading')

    
        self.update_rate = max_navpose_update_rate

        self.initCb(do_updates = False)



        if frame_3d is not None:
          self.frame_3d = frame_3d
        if frame_altitude is not None:
          self.frame_altitude = frame_altitude


        ###
        self.getHeadingCb = getHeadingCb
        if self.getHeadingCb is not None:
          self.has_heading = True

        self.getPositionCb = getPositionCb
        if self.getPositionCb is not None:
          self.has_position = True

        self.getOrientationCb = getOrientationCb
        if self.getOrientationCb is not None:
          self.has_orientation = True

        self.getLocationCb = getLocationCb
        if self.getLocationCb is not None:
          self.has_location = True  

        self.getAltitudeCb = getAltitudeCb
        if self.getAltitudeCb is not None:
          self.has_altitude = True

        self.getDepthCb = getDepthCb
        if self.getDepthCb is not None:
          self.has_depth = True

        if self.has_location == True:
            self.has_gps_pub = True
            
        if self.has_altitude == True or self.has_depth == True:
            self.has_elevation_pub = True

        if self.has_position == True or self.has_orientation == True:
            self.has_pose_pub = True

        if self.has_heading == True:
            self.has_heading_pub = True


        # Create capabilities report
        self.navpose_capabilities_report = NPXCapabilitiesQueryResponse()
        self.navpose_capabilities_report.has_heading = self.has_heading
        self.navpose_capabilities_report.has_position = self.has_position
        self.navpose_capabilities_report.has_orientation = self.has_orientation
        self.navpose_capabilities_report.has_location = self.has_location      
        self.navpose_capabilities_report.has_altitude = self.has_altitude
        self.navpose_capabilities_report.has_depth =  self.has_depth

        self.navpose_capabilities_report.has_gps_pub =  self.has_gps_pub
        self.navpose_capabilities_report.has_elevation_pub =  self.has_elevation_pub
        self.navpose_capabilities_report.has_pose_pub =  self.has_pose_pub
        self.navpose_capabilities_report.has_heading_pub =  self.has_heading_pub


        self.navpose_capabilities_report.pub_rate_min_max = [self.MIN_PUB_RATE,max_navpose_update_rate]
        self.navpose_capabilities_report.frame_id_options = self.NAVPOSE_FRAME_ID_OPTIONS
        self.navpose_capabilities_report.frame_3d_options = self.NAVPOSE_FRAME_3D_OPTIONS
        self.navpose_capabilities_report.frame_altitude_options = self.NAVPOSE_FRAME_ALT_OPTIONS


        #  Connect to navpose manager

        self.navpose_mgr_if = ConnectMgrNavPoseIF()
      

        # All Good

        npx_namespace = nepi_ros.create_namespace(self.node_namespace,'npx')
        self.navpose_if = NavPoseIF(namespace = npx_namespace,
                            enable_gps_pub = self.has_gps_pub,
                            enable_elevation_pub = self.has_elevation_pub,
                            enable_pose_pub =  self.has_pose_pub,
                            enable_heading_pub = self.has_heading_pub,
                            log_name = 'navpose',
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )

        self.status_msg = NPXStatus()

        ##################################################
        ### Node Class Setup

        self.msg_if.pub_info("Starting Node IF Initialization", log_name_list = self.log_name_list)
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
            'update_rate': {
                'namespace': self.node_namespace,
                'factory_val': self.update_rate
            },
            'set_gps_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_gps_source
            },
            'set_elevation_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_elevation_source
            },
            'set_pose_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_pose_source
            },
            'set_heading_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_heading_source
            },
            'frame_id': {
                'namespace': self.node_namespace,
                'factory_val': self.frame_id
            },
            'frame_3d': {
                'namespace': self.node_namespace,
                'factory_val': self.frame_3d
            },
            'frame_altitude': {
                'namespace': self.node_namespace,
                'factory_val': self.frame_altitude
            },
            'frame_transform': {
                'namespace': self.node_namespace,
                'factory_val': self.ZERO_TRANSFORM
            },
        }

        # Services Config Dict ####################

        self.SRVS_DICT = {
            'navpose_capabilities_query': {
                'namespace': self.node_namespace,
                'topic': 'npx/capabilities_query',
                'srv': NPXCapabilitiesQuery,
                'req': NPXCapabilitiesQueryRequest(),
                'resp': NPXCapabilitiesQueryResponse(),
                'callback': self._navposeCapabilitiesHandler
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
            'set_update_rate': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_update_rate',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setUpdateRateCb, 
                'callback_args': ()
            },
            'set_frame_id': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_frame_id',
                'msg': String,
                'qsize': 1,
                'callback': self._setIdFrameCb, 
                'callback_args': ()
            },
            'set_frame_3d': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_frame_3d',
                'msg': String,
                'qsize': 1,
                'callback': self._set3dFrameCb, 
                'callback_args': ()
            },
            'set_frame_altitude': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_frame_altitude',
                'msg': String,
                'qsize': 1,
                'callback': self._setAltFrameCb, 
                'callback_args': ()
            },
            'clear_frame_3d_transform': {
                'namespace': self.node_namespace,
                'topic': 'npx/clear_frame_transform',
                'msg': Empty,
                'qsize': 1,
                'callback': self._clearFrame3dTransformCb, 
                'callback_args': ()
            },
            'set_frame_transform': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_frame_transform',
                'msg': Frame3DTransform,
                'qsize': 1,
                'callback': self._setFrame3dTransformCb, 
                'callback_args': ()
            },
            'set_as_gps_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_gps_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setGpsSourceCb, 
                'callback_args': ()
            },
            'set_as_elevation_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_elevation_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setElevationSourceCb, 
                'callback_args': ()
            },
            'set_as_pose_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_pose_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setPoseSourceCb, 
                'callback_args': ()
            },
            'set_as_heading_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_heading_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setHeadingSourceCb, 
                'callback_args': ()
            },
            


        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )
      

        ready = self.node_if.wait_for_ready()


        # Setup Settings IF Class ####################
        self.msg_if.pub_info("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = nepi_ros.create_namespace(self.node_namespace,'npx')

        self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction, 
                    'namespace': settings_ns
                    
        }

        self.settings_if = SettingsIF(self.SETTINGS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )


        # Setup Save Data IF Class ####################
        self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
        factory_data_rates = {}
        for d in self.data_products_list:
            factory_data_rates[d] = [1.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "",
            'add_node_name': True
            }

        sd_namespace = nepi_ros.create_namespace(self.node_namespace,'ptx')
        self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                factory_rate_dict = factory_data_rates,
                                factory_filename_dict = factory_filename_dict,
                                namespace = sd_namespace,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )


        time.sleep(1)


        self.set_gps_source = self.node_if.get_param('set_gps_source')
        self.set_elevation_source = self.node_if.get_param('set_elevation_source')
        self.set_pose_source = self.node_if.get_param('set_pose_source')
        self.set_heading_source = self.node_if.get_param('set_heading_source')
        self.frame_transform = self.node_if.get_param('frame_transform')

        self.initCb(do_updates = True)

        nepi_ros.start_timer_process(0.1, self._updateNavPoseDictCb, oneshot = True)
        nepi_ros.start_timer_process(1.0, self._publishStatusCb, oneshot = False)
        ###############################
        # Finish Initialization

        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)



  ###############################
  # Class Methods

  def check_ready(self):
      return self.ready  

  def wait_for_ready(self, timeout = float('inf') ):
      success = False
      if self.ready is not None:
          self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
          timer = 0
          time_start = nepi_ros.get_time()
          while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
              nepi_ros.sleep(.1)
              timer = nepi_ros.get_time() - time_start
          if self.ready == False:
              self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
          else:
              self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
      return self.ready   


  def get_navpose_dict(self):
    return self.navpose_dict




    ###############################
    # Class Private Methods
    ###############################

  def _setUpdateRateCb(self,msg):
    rate = msg.data
    min = self.NAVPOSE_UPDATE_RATE_OPTIONS[0]
    max = self.NAVPOSE_UPDATE_RATE_OPTIONS[1]
    if rate < min:
      rate = min
    if rate > max:
      rate = max
    self.update_rate = rate
    self.node_if.set_param('update_rate',rate)

  def _set3dFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_FRAME_3D_OPTIONS:
      self.frame_3d = frame
      self.node_if.set_param('frame_3d',frame)

  def _setIdFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_FRAME_ID_OPTIONS:
      self.frame_3d = frame
      self.node_if.set_param('frame_id',frame)

  def _setAltFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_FRAME_ALT_OPTIONS:
      self.frame_altitude = frame
      self.node_if.set_param('frame_altitude',frame)


  def _setFrame3dTransformCb(self, msg):
      self.msg_if.pub_info("Recived 3D frame_transform update message: " + str(msg))
      new_transform_msg = msg
      self.setFrame3dTransform(new_transform_msg)

  def setFrame3dTransform(self, transform_msg):
      x = transform_msg.translate_vector.x
      y = transform_msg.translate_vector.y
      z = transform_msg.translate_vector.z
      roll = transform_msg.rotate_vector.x
      pitch = transform_msg.rotate_vector.y
      yaw = transform_msg.rotate_vector.z
      heading = transform_msg.heading_offset
      self.frame_transform = [x,y,z,roll,pitch,yaw,heading]
      self.status_msg.frame_3d_transform = transform_msg
      self.publishStatus(do_updates=False) # Updated inline here 

      self.node_if.set_param('frame_3d_transform',  self.frame_transform)


    def clearFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Clear 3D Transform update message: ")
        self.clearFrame3dTransform()

    def clearFrame3dTransform(self):
        self.frame_transform = self.ZERO_TRANSFORM
        self.status_msg.frame_3d_transform = Frame3DTransform()
        self.publishStatus(do_updates=False) # Updated inline here 
        self.node_if.set_param('frame_3d_transform',  self.frame_transform)

  def _setGpsSourceCb(self, msg):
      self.msg_if.pub_info("Recived set gps source update message: " + str(msg))
      enabled = msg.data
      self.set_gps_source = enabled
      self.status_msg.set_as_gps_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_gps_source',  enabled)

  def _setElevationSourceCb(self, msg):
      self.msg_if.pub_info("Recived set elevation source update message: " + str(msg))
      enabled = msg.data
      self.set_elevation_source = enabled
      self.status_msg.set_as_elevation_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_elevation_source',  enabled)

  def _setPoseSourceCb(self, msg):
      self.msg_if.pub_info("Recived set pose source update message: " + str(msg))
      enabled = msg.data
      self.set_pose_source = enabled
      self.status_msg.set_as_pose_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_pose_source',  enabled)

  def _setHeadingSourceCb(self, msg):
      self.msg_if.pub_info("Recived set heading source update message: " + str(msg))
      enabled = msg.data
      self.set_heading_source = enabled
      self.status_msg.set_as_heading_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_heading_source',  enabled)



  def initCb(self,do_updates = False):
      self.msg_if.pub_info("Setting init values to param values", log_name_list = self.log_name_list)

      if do_updates == True:
        self.resetCb(do_updates)

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
        self.update_rate = self.node_if.get_param('update_rate')
        self.frame_3d = self.node_if.get_param('frame_3d')
        self.frame_altitude = self.node_if.get_param('frame_altitude')

  def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        self.update_rate = self.node_if.get_param('update_rate')
        self.frame_3d = self.node_if.get_param('frame_3d')
        self.frame_altitude = self.node_if.get_param('frame_altitude')



  def _navposeCapabilitiesHandler(self, _):
    return self.navpose_capabilities_report    


  def _updateNavPoseDictCb(self,timer):

    navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    navpose_dict['frame_id'] = self.frame_id
    navpose_dict['frame_3d'] = self.frame_3d
    navpose_dict['altitude_frame'] = self.frame_altitude
       

    navpose_dict['has_heading'] = self.has_heading 
    if self.has_heading == True:
      try:
          data_dict = self.getHeadingCb()
          if self.has_updated == False:
            self.msg_if.pub_warn("Get Heading function returned: " + str(data_dict))
          for key in data_dict.keys():
            navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Heading Cb: " + str(e))  


    navpose_dict['has_position'] = self.has_position 
    if self.has_position == True:
      try:
          data_dict = self.getPositionCb()
          if self.has_updated == False:
            self.msg_if.pub_warn("Get Position function returned: " + str(data_dict))
          for key in data_dict.keys():
            navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Position Cb: " + str(e)) 


    navpose_dict['has_orientation'] = self.has_orientation 
    if self.has_orientation == True:
      try:
          data_dict = self.getOrientationCb()
          if self.has_updated == False:
            self.msg_if.pub_warn("Get Orientation function returned: " + str(data_dict))
          for key in data_dict.keys():
            navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Orientation Cb: " + str(e)) 


    navpose_dict['has_location'] = self.has_location 
    if self.has_location == True:
      try:
          data_dict = self.getLocationCb()
          if self.has_updated == False:
            self.msg_if.pub_warn("Get Location function returned: " + str(data_dict))
          for key in data_dict.keys():
            navpose_dict[key] = data_dict[key]

          geopoint = GeoPoint()
          geopoint.latitude = data_dict['latitude']
          geopoint.longitude = data_dict['longitude']
          geiod_height = nepi_nav.get_navpose_geoid_height_at_geopoint(geopoint)
          navpose_dict['geoid_height_meters'] = geiod_height
          #self.msg_if.pub_warn("Got Geoid Height: " + str(geiod_height))
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Location Cb: " + str(e))

    navpose_dict['has_altitude'] = self.has_altitude 
    if self.has_altitude == True:
      try:
          data_dict = self.getAltitudeCb()
          if self.has_updated == False:
            self.msg_if.pub_warn("Get Altitude function returned: " + str(data_dict))
          for key in data_dict.keys():
            navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Altitude Cb: " + str(e))

    navpose_dict['has_depth'] = self.has_depth 
    if self.has_depth == True:
      try:
          data_dict = self.getDepthCb()
          if self.has_updated == False:
            self.msg_if.pub_warn("Get Depth function returned: " + str(data_dict))
          for key in data_dict.keys():
            navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Depth Cb: " + str(e))            

    # Do Conversions if Needed
    if self.frame_3d == 'NED':
        navpose_dict = nepi_nav.convert_navposedata_edu2ned(navpose_dict)
    if self.frame_altitude == 'AMSL':
        navpose_dict = nepi_nav.convert_navposedata_wgs842amsl(navpose_dict)

    if self.has_updated == False:
        self.msg_if.pub_warn("Returning navpose dict: " + str(navpose_dict))

    if self.frame_id == 'nepi_base_frame':
      navpose_dict = nepi_nav.transform_navpose_dict(navpose_dict,self.frame_transform)

    self.has_updated = True
    self.navpose_dict =  navpose_dict
    try:
      self.navpose_if.publish_navpose(self.navpose_dict)
    except Exception as e:
      self.msg_if.pub_warn("Failed to publish navpose data msg: " + str(e))
    timestamp = nepi_utils.get_time()
    self.save_data_if.save('navpose',self.navpose_dict,timestamp)

    delay = float(1) / self.update_rate


    # Set navpose manager source topics if required
    if self.set_gps_source == True and self.navpose_mgr_if is not None:
      self.navpose_mgr_if.set_gps_source_topic(self.gps_pub_ns)

    if self.set_elevation_source == True and self.navpose_mgr_if is not None:
      self.navpose_mgr_if.set_elevation_source_topic(self.elevation_pub_ns)

    if self.set_pose_source == True and self.navpose_mgr_if is not None:
      self.navpose_mgr_if.set_pose_source_topic(self.pose_pub_ns)

    if self.set_heading_source == True and self.navpose_mgr_if is not None:
      self.navpose_mgr_if.set_heading_source_topic(self.heading_pub_ns)

    nepi_ros.start_timer_process(delay, self._updateNavPoseDictCb, oneshot = True)



  def _publishStatusCb(self,timer):
      self.status_msg.update_rate = self.update_rate
      self.status_msg.frame_id = self.frame_id
      self.status_msg.frame_3d = self.frame_3d
      self.status_msg.frame_altitude = self.frame_altitude

      self.status_msg.has_heading = self.has_heading
      self.status_msg.has_position = self.has_position
      self.status_msg.has_orientation = self.has_orientation
      self.status_msg.has_location = self.has_location      
      self.status_msg.has_altitude = self.has_altitude
      self.status_msg.has_depth =  self.has_depth

      self.status_msg.has_gps_pub =  self.has_gps_pub
      self.status_msg.has_elevation_pub =  self.has_elevation_pub
      self.status_msg.has_pose_pub =  self.has_pose_pub
      self.status_msg.has_heading_pub =  self.has_heading_pub

      self.status_msg.set_as_gps_source = self.set_gps_source
      self.status_msg.set_as_elevation_source = self.set_elevation_source
      self.status_msg.set_as_pose_source = self.set_pose_source
      self.status_msg.set_as_heading_source = self.set_heading_source

      transform = self.frame_transform
      transform_msg = Frame3DTransform()
      transform_msg.translate_vector.x = transform[0]
      transform_msg.translate_vector.y = transform[1]
      transform_msg.translate_vector.z = transform[2]
      transform_msg.rotate_vector.x = transform[3]
      transform_msg.rotate_vector.y = transform[4]
      transform_msg.rotate_vector.z = transform[5]
      transform_msg.heading_offset = transform[6]
      self.status_msg.frame_transform = transform_msg

      self.node_if.publish_pub('status_pub',self.status_msg)



  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions", log_name_list = self.log_name_list)


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPosePublisher()


