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

from nepi_sdk_interfaces.msg import Frame3DTransform, Frame3DTransforms
from nepi_sdk_interfaces.msg import UpdateNavPoseTopic

from geometry_msgs.msg import Vector3

from nepi_sdk_interfaces.msg import NPXStatus
from nepi_sdk_interfaces.srv import  NPXCapabilitiesQuery, NPXCapabilitiesQueryRequest, NPXCapabilitiesQueryResponse


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_settings

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.data_if import NavPoseIF
from nepi_api.connect_mgr_if_navpose import ConnectMgrNavPoseIF


EXAMPLE_LOCATION_DATA_DICT = {
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long, Altitude
    'latitude': 47.080909,
    'longitude': -120.8787889
}

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


EXAMPLE_ALTITUDE_DATA_DICT = {
    'time_altitude': nepi_utils.get_time(),
    'altitude_m': 12.321,
}

EXAMPLE_DEPTH_DATA_DICT = {
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive distance from surface in meters
    'depth_m': 0
}

EXAMPLE_NAVPOSE_DATA_DICT = {
    'frame_3d': 'nepi_frame',
    'frame_nav': 'ENU',
    'frame_altitude': 'WGS84',
    'frame_depth': 'MSL',

    'geoid_height_meters': 0,

    'has_location': True,
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'latitude': 47.080909,
    'longitude': -120.8787889,
    
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



    'has_altitude': True,
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters in specified altitude_m frame
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
  NAVPOSE_FRAME_ID_OPTIONS = ['sensor_frame','custom_frame']
  NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
  NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
  NAVPOSE_DEPTH_FRAME_OPTIONS = ['MSL','TOC','DF','KB','DEPTH','UKNOWN']

  DEFAULT_UPDATE_RATE = 10
  DEFAULT_3D_FRAME = 'sensor_frame'
  DEFAULT_NAV_FRAME = 'ENU'
  DEFAULT_ALT_FRAME = 'WGS84'
  DEFAULT_DEPTH_FRAME = 'DEPTH'

  ZERO_TRANSFORM = [0,0,0,0,0,0,0]

  data_products_list = ['navpose']

  node_if = None

  update_rate = DEFAULT_UPDATE_RATE
  frame_3d = DEFAULT_3D_FRAME
  frame_nav = DEFAULT_NAV_FRAME
  frame_altitude = DEFAULT_ALT_FRAME
  frame_depth = DEFAULT_DEPTH_FRAME

  nepi_frame_3d_transform = ZERO_TRANSFORM
  apply_transform = False

  has_location = False  
  has_heading = False
  has_orientation = False
  has_position = False
  has_altitude = False
  has_depth = False

  set_location_source = False
  set_heading_source = False
  set_orientation_source = False  
  set_position_source = False
  set_altitude_source = False
  set_depth_source = False

  was_location_source = False
  was_heading_source = False
  was_orientation_source = False  
  was_position_source = False
  was_altitude_source = False
  was_depth_source = False

  navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT

  navpose_if = None  

  settings_if = None

  status_msg = NPXStatus()

  has_updated = False


  navpose_mgr_if = None
  #######################
  ### IF Initialization
  def __init__(self, 
                device_info,
                capSettings=None, factorySettings=None, 
                settingUpdateFunction=None, getSettingsFunction=None,
                frame_3d = 'sensor_frame', frame_nav = 'ENU',
                frame_altitude = 'WGS84', frame_depth = 'DEPTH',
                getHeadingCb = None, getPositionCb = None, getOrientationCb = None,
                getLocationCb = None, getAltitudeCb = None, getDepthCb = None,
                max_navpose_update_rate = DEFAULT_UPDATE_RATE,
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
        self.msg_if.pub_info("Starting NPX Device IF Initialization Processes", log_name_list = self.log_name_list)  


        ##############################    
        #  Connect to navpose manager
        self.navpose_mgr_if = ConnectMgrNavPoseIF()

        ##############################
        # Initialize Class 
   
        self.update_rate = max_navpose_update_rate

        self.initCb(do_updates = False)

        if frame_3d is not None:
          self.frame_3d = frame_3d

        if frame_nav is not None:
          self.frame_nav = frame_nav

        if frame_altitude is not None:
          self.frame_altitude = frame_altitude

        if frame_depth is not None:
          self.frame_depth = frame_depth


        ###

        self.getLocationCb = getLocationCb
        if self.getLocationCb is not None:
          self.has_location = True  

        self.getHeadingCb = getHeadingCb
        if self.getHeadingCb is not None:
          self.has_heading = True

        self.getPositionCb = getPositionCb
        if self.getPositionCb is not None:
          self.has_position = True

        self.getOrientationCb = getOrientationCb
        if self.getOrientationCb is not None:
          self.has_orientation = True

        self.getAltitudeCb = getAltitudeCb
        if self.getAltitudeCb is not None:
          self.has_altitude = True

        self.getDepthCb = getDepthCb
        if self.getDepthCb is not None:
          self.has_depth = True


        # Create capabilities report
        self.caps_report = NPXCapabilitiesQueryResponse()
        self.caps_report.has_heading = self.has_heading
        self.caps_report.has_position = self.has_position
        self.caps_report.has_orientation = self.has_orientation
        self.caps_report.has_location = self.has_location      
        self.caps_report.has_altitude = self.has_altitude
        self.caps_report.has_depth =  self.has_depth

        self.caps_report.pub_rate_min_max = [self.MIN_PUB_RATE,max_navpose_update_rate]

        self.caps_report.data_products =  self.data_products_list


        # Initialize navpose_dict
        self.navpose_dict['frame_3d'] = self.frame_3d
        self.navpose_dict['frame_nav'] = self.frame_nav
        self.navpose_dict['frame_altitude'] = self.frame_altitude
        self.navpose_dict['frame_depth'] = self.frame_depth

        # Initialize status message
        self.status_msg.has_location = self.has_location
        self.status_msg.has_heading = self.has_heading
        self.status_msg.has_orientation = self.has_orientation
        self.status_msg.has_position = self.has_position
        self.status_msg.has_altitude = self.has_altitude
        self.status_msg.has_depth = self.has_depth

        self.status_msg.frame_3d = self.frame_3d
        self.status_msg.nepi_frame_3d_transform = Frame3DTransform()

        self.status_msg.source_frame_nav = self.frame_nav
        self.status_msg.source_frame_altitude = self.frame_altitude
        self.status_msg.source_frame_depth = self.frame_depth


     

        # Create a navpose data IF
        npx_namespace = nepi_sdk.create_namespace(self.node_namespace,'npx')
        self.navpose_if = NavPoseIF(namespace = npx_namespace,
                            has_location = self.has_location,
                            has_heading = self.has_heading,
                            has_orientation =  self.has_orientation,
                            has_position = self.has_position,
                            has_altitude = self.has_altitude,
                            has_depth = self.has_depth,
                            log_name = 'navpose',
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )


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
            'set_location_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_location_source
            },
            'set_heading_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_heading_source
            },
            'set_orientation_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_orientation_source
            },
            'set_position_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_position_source
            },
            'set_altitude_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_altitude_source
            },
            'set_depth_source': {
                'namespace': self.node_namespace,
                'factory_val': self.set_depth_source
            },
            'nepi_frame_3d_transform': {
                'namespace': self.node_namespace,
                'factory_val': self.ZERO_TRANSFORM
            },
            'include_transform_enabled': {
                'namespace': self.node_namespace,
                'factory_val': False
            }

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
            'set_as_heading_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_heading_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setHeadSourceCb, 
                'callback_args': ()
            },
            'set_as_location_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_location_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setLocSourceCb, 
                'callback_args': ()
            },
            'set_as_position_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_position_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setPosSourceCb, 
                'callback_args': ()
            },
            'set_as_altitude_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_altitude_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setAltSourceCb, 
                'callback_args': ()
            },            
            'set_as_orientation_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_orientation_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setOrienSourceCb, 
                'callback_args': ()
            },
            'set_as_depth_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_depth_source',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setDepthSourceCb, 
                'callback_args': ()
            },
            'clear_frame_3d_transform': {
                'namespace': self.node_namespace,
                'topic': 'npx/clear_3d_transform',
                'msg': Empty,
                'qsize': 1,
                'callback': self._clearFrame3dTransformCb, 
                'callback_args': ()
            },
            'set_frame_transform': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_3d_transform',
                'msg': Frame3DTransform,
                'qsize': 1,
                'callback': self._setFrame3dTransformCb, 
                'callback_args': ()
            },
            'include_transform_enabled': {
                'namespace': self.node_namespace,
                'topic': 'npx/include_3d_transform_enabled',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setApplyTransformCb, 
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
        settings_ns = nepi_sdk.create_namespace(self.node_namespace,'npx')
        
        self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction
                    
        }

        self.settings_if = SettingsIF(namespace = settings_ns,
                            settings_dict = self.SETTINGS_DICT,
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

        sd_namespace = nepi_sdk.create_namespace(self.node_namespace,'ptx')
        self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                factory_rate_dict = factory_data_rates,
                                factory_filename_dict = factory_filename_dict,
                                namespace = sd_namespace,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )


        time.sleep(1)

        self.include_transform_enabled = self.node_if.get_param('include_transform_enabled')
        self.set_location_source = self.node_if.get_param('set_location_source')
        self.set_heading_source = self.node_if.get_param('set_heading_source')
        self.set_orientation_source = self.node_if.get_param('set_orientation_source')
        self.set_position_source = self.node_if.get_param('set_position_source')
        self.set_altitude_source = self.node_if.get_param('set_altitude_source')
        self.set_depth_source = self.node_if.get_param('set_depth_source')

        self.nepi_frame_3d_transform = self.node_if.get_param('nepi_frame_3d_transform')

        self.apply_transform = self.node_if.get_param('set_apply_transform')
      

        self.initCb(do_updates = True)

        nepi_sdk.start_timer_process(0.1, self._updateNavPoseDictCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._updateNavPoseConnectCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)
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
          time_start = nepi_sdk.get_time()
          while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
              nepi_sdk.sleep(.1)
              timer = nepi_sdk.get_time() - time_start
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
      self.frame_nav = frame
      self.node_if.set_param('frame_nav',frame)

  def _setIdFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_FRAME_ID_OPTIONS:
      self.frame_3d = frame
      self.node_if.set_param('frame_3d',frame)

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
      self.nepi_frame_3d_transform = [x,y,z,roll,pitch,yaw,heading]
      self.status_msg.nepi_frame_3d_transform = transform_msg
      self.publishStatus(do_updates=False) # Updated inline here 

      self.node_if.set_param('nepi_frame_3d_transform',  self.nepi_frame_3d_transform)


  def _clearFrame3dTransformCb(self, msg):
      self.msg_if.pub_info("Recived Clear 3D Transform update message: ")
      self.clearFrame3dTransform()

  def clearFrame3dTransform(self):
        self.nepi_frame_3d_transform = self.ZERO_TRANSFORM
        self.status_msg.nepi_frame_3d_transform = Frame3DTransform()
        self.publishStatus(do_updates=False) # Updated inline here 
        self.node_if.set_param('nepi_frame_3d_transform',  self.nepi_frame_3d_transform)


  def _setApplyTransformCb(self, msg):
      self.msg_if.pub_info("Recived set apply transform update message: " + str(msg))
      enabled = msg.data
      self.setLocSource(enabled)

  def setApplyTransforme(self, enabled):
      self.apply_transform = enabled
      self.status_msg.include_transform_enabled = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_apply_transform',  enabled)

  def _setLocSourceCb(self, msg):
      self.msg_if.pub_info("Recived set location source update message: " + str(msg))
      enabled = msg.data
      self.setLocSource(enabled)

  def setLocSource(self, enabled):
      self.set_location_source = enabled
      self.status_msg.set_as_location_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_location_source',  enabled)

  def _setHeadSourceCb(self, msg):
      self.msg_if.pub_info("Recived set heading source update message: " + str(msg))
      enabled = msg.data
      self.setHeadSource(enabled)

  def setHeadSource(self, enabled):
      self.set_heading_source = enabled
      self.status_msg.set_as_heading_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_heading_source',  enabled)

  def _setOrienSourceCb(self, msg):
      self.msg_if.pub_info("Recived set orientation source update message: " + str(msg))
      enabled = msg.data
      self.setOrienSource(enabled)

  def setOrienSource(self, enabled):
      self.set_orientation_source = enabled
      self.status_msg.set_as_orientation_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_orientation_source',  enabled)

  def _setPosSourceCb(self, msg):
      self.msg_if.pub_info("Recived set position source update message: " + str(msg))
      enabled = msg.data
      self.setPosSource(enabled)

  def setPosSource(self, enabled):
      self.set_position_source = enabled
      self.status_msg.set_as_position_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_position_source',  enabled)

  def _setAltSourceCb(self, msg):
      self.msg_if.pub_info("Recived set altitude source update message: " + str(msg))
      enabled = msg.data
      self.setAltSource(enabled)

  def setAltSource(self, enabled):
      self.set_altitude_source = enabled
      self.status_msg.set_as_altitude_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_altitude_source',  enabled)

  def _setDepthSourceCb(self, msg):
      self.msg_if.pub_info("Recived set depth source update message: " + str(msg))
      enabled = msg.data
      self.setDepthSource(enabled)

  def setDepthSource(self, enabled):
      self.set_depth_source = enabled
      self.status_msg.set_as_depth_source = enabled
      self.publishStatus(do_updates=False) # Updated inline here 
      self.node_if.set_param('set_depth_source',  enabled)

  def publishStatus(self, do_updates=True):
      self._publishStatusCb(None)

  def initCb(self,do_updates = False):
      self.msg_if.pub_info("Setting init values to param values", log_name_list = self.log_name_list)

      if do_updates == True:
        self.resetCb(do_updates)

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
        self.update_rate = self.node_if.get_param('update_rate')
        self.frame_nav = self.node_if.get_param('frame_nav')
        self.frame_altitude = self.node_if.get_param('frame_altitude')

  def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        self.update_rate = self.node_if.get_param('update_rate')
        self.frame_nav = self.node_if.get_param('frame_nav')
        self.frame_altitude = self.node_if.get_param('frame_altitude')



  def _navposeCapabilitiesHandler(self, _):
    return self.caps_report    


  def _updateNavPoseDictCb(self,timer):     
    
    # Update Location
    if self.has_location == True:
      try:
          data_dict = self.getLocationCb()
          for key in data_dict.keys():
            self.navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Location Cb: " + str(e), throttle_s = 5.0)

    # Update Heading
    if self.has_heading == True:
      try:
          data_dict = self.getHeadingCb()
          for key in data_dict.keys():
            self.navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Heading Cb: " + str(e), throttle_s = 5.0)

    # Update Orientation
    if self.has_orientation == True:
      try:
          data_dict = self.getOrientationCb()
          for key in data_dict.keys():
            self.navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Orientation Cb: " + str(e), throttle_s = 5.0)

    # Update Position
    if self.has_position == True:
      try:
          data_dict = self.getPositionCb()
          for key in data_dict.keys():
            self.navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Position Cb: " + str(e), throttle_s = 5.0)

    # Update Altitude
    if self.has_altitude == True:
      try:
          data_dict = self.getAltitudeCb()
          for key in data_dict.keys():
            self.navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Altitude Cb: " + str(e), throttle_s = 5.0)

    # Update Depth
    if self.has_depth == True:
      try:
          data_dict = self.getDepthCb()
          for key in data_dict.keys():
            self.navpose_dict[key] = data_dict[key]
      except Exception as e:
          self.msg_if.pub_warn("Failed to call get Depth Cb: " + str(e), throttle_s = 5.0)      

    # publish navpose data
    self.navpose_if.publish_navpose(self.navpose_dict)

    # save data if needed
    timestamp = nepi_utils.get_time()
    self.save_data_if.save('navpose',self.navpose_dict,timestamp)

    # set up next loop
    delay = float(1) / self.update_rate
    nepi_sdk.start_timer_process(delay, self._updateNavPoseDictCb, oneshot = True)


  def _updateNavPoseConnectCb(self,timer):     
    if self.navpose_mgr_if is not None:
    
      if self.include_transform_enabled == False:
        transform = None
      else:
        transform = self.nepi_frame_3d_transform

      # Update Location
      name = 'location'
      topic = self.node_name + '/navpose/' + name
      was_set = copy.deepcopy(self.was_location_source)
      is_set = copy.deepcopy(self.set_location_source)
      self.was_location_source = is_set
      if is_set == True:
        self.navpose_mgr_if.set_comp_topic(name,topic,transform_list = transform)
      elif is_set == False and was_set == True:
        self.navpose_mgr_if.clear_comp_topic(name)

      # Update Heading
      name = 'heading'
      topic = self.node_name + '/navpose/' + name
      was_set = copy.deepcopy(self.was_heading_source)
      is_set = copy.deepcopy(self.set_heading_source)
      self.was_heading_source = is_set
      if is_set == True:
        self.navpose_mgr_if.set_comp_topic(name,topic,transform_list = transform)
      elif is_set == False and was_set == True:
        self.navpose_mgr_if.clear_comp_topic(name)

      # Update Orientation
      name = 'orientation'
      topic = self.node_name + '/navpose/' + name
      was_set = copy.deepcopy(self.was_orientation_source)
      is_set = copy.deepcopy(self.set_orientation_source)
      self.was_orientation_source = is_set
      if is_set == True and was_set == False:
        self.navpose_mgr_if.set_comp_topic(name,topic,transform_list = transform)
      elif was_set == True:
        self.navpose_mgr_if.clear_comp_topic(name)


      # Update position
      name = 'position'
      topic = self.node_name + '/navpose/' + name
      was_set = copy.deepcopy(self.was_position_source)
      is_set = copy.deepcopy(self.set_position_source)
      self.was_position_source = is_set
      if is_set == True and was_set == False:
        self.navpose_mgr_if.set_comp_topic(name,topic,transform_list = transform)
      elif is_set == False and was_set == True:
        self.navpose_mgr_if.clear_comp_topic(name)


      # Update altitude
      name = 'altitude'
      topic = self.node_name + '/navpose/' + name
      was_set = copy.deepcopy(self.was_altitude_source)
      is_set = copy.deepcopy(self.set_altitude_source)
      self.was_altitude_source = is_set
      if is_set == True and was_set == False:
        self.navpose_mgr_if.set_comp_topic(name,topic,transform_list = transform)
      elif is_set == False and was_set == True:
        self.navpose_mgr_if.clear_comp_topic(name)


      # Update depth
      name = 'depth'
      topic = self.node_name + '/navpose/' + name
      was_set = copy.deepcopy(self.was_depth_source)
      is_set = copy.deepcopy(self.set_depth_source)
      self.was_depth_source = is_set
      if is_set == True and was_set == False:
        self.navpose_mgr_if.set_comp_topic(name,topic,transform_list = transform)
      elif is_set == False and was_set == True:
        self.navpose_mgr_if.clear_comp_topic(name)
  
    nepi_sdk.start_timer_process(1.0, self._updateNavPoseConnectCb, oneshot = True)



  def _publishStatusCb(self,timer):

      self.status_msg.set_as_location_source = self.set_location_source
      self.status_msg.set_as_heading_source = self.set_heading_source
      self.status_msg.set_as_orientation_source = self.set_orientation_source
      self.status_msg.set_as_position_source = self.set_position_source
      self.status_msg.set_as_location_source = self.set_location_source
      self.status_msg.set_as_altitude_source = self.set_altitude_source
      self.status_msg.set_as_depth_source = self.set_depth_source
      transform_msg = nepi_nav.convert_transform_list2msg(self.nepi_frame_3d_transform)
      self.status_msg.nepi_frame_3d_transform = transform_msg
      self.status_msg.include_transform_enabled = self.include_transform_enabled

      self.status_msg.update_rate = self.update_rate

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


