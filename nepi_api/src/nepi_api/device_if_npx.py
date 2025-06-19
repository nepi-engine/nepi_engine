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

from nepi_interfaces.msg import Frame3DTransform
from nepi_interfaces.msg import UpdateNavPoseTopic

from geometry_msgs.msg import Vector3

from nepi_interfaces.msg import DeviceNPXStatus
from nepi_interfaces.srv import  NPXCapabilitiesQuery, NPXCapabilitiesQueryRequest, NPXCapabilitiesQueryResponse


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_settings

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF, Transform3DIF

from nepi_api.data_if import NavPoseIF
from nepi_api.connect_mgr_if_navpose import ConnectMgrNavPoseIF


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
  settings_if = None
  save_data_if = None

  status_msg = DeviceNPXStatus()
  
  update_rate = DEFAULT_UPDATE_RATE
  frame_3d = DEFAULT_3D_FRAME
  frame_nav = DEFAULT_NAV_FRAME
  frame_altitude = DEFAULT_ALT_FRAME
  frame_depth = DEFAULT_DEPTH_FRAME

  frame_transform = ZERO_TRANSFORM

  has_location = False  
  has_heading = False
  has_orientation = False
  has_position = False
  has_altitude = False
  has_depth = False
  supports_transform_updates = True

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

  has_updated = False

  pub_subs = False

  navpose_mgr_if = None



  frame_3d = 'nepi_frame'
  tr_source_ref_description = 'data_reference'
  tr_end_ref_description = 'nepi_frame'

  data_source_description = 'navpose_sensor'
  data_ref_description = 'data_reference'
  device_mount_description = 'fixed'
  mount_desc = 'None'

  getNavPoseCb = None
  get3DTransformCb = None
  supports_transform_updates = True

  #######################
  ### IF Initialization
  def __init__(self, 
                device_info,
                capSettings=None, factorySettings=None, 
                settingUpdateFunction=None, getSettingsFunction=None,
                data_source_description = 'navpose_sensor',
                data_ref_description = 'data_reference',
                frame_3d = 'sensor_frame', frame_nav = 'ENU',
                frame_altitude = 'WGS84', frame_depth = 'DEPTH',
                getNavPoseCb = None,
                get3DTransformCb = None,
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
        ## Connect NEPI NavPose Manager
        self.nav_mgr_if = ConnectMgrNavPoseIF()
        ready = self.nav_mgr_if.wait_for_ready()

        ##############################
        # Initialize Class 
   
        self.device_id = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.device_name = device_info["device_id"] + "_" + device_info["identifier"]

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description
        self.tr_source_ref_description = data_ref_description

        self.get3DTransformCb = get3DTransformCb

        self.supports_transform_updates = (get3DTransformCb is None)
        self.has_transform = True

        if frame_3d is not None:
          self.frame_3d = frame_3d

        if frame_nav is not None:
          self.frame_nav = frame_nav

        if frame_altitude is not None:
          self.frame_altitude = frame_altitude

        if frame_depth is not None:
          self.frame_depth = frame_depth

        self.get3DTranformCb = get3DTranformCb
        ###
        self.getNavPoseCb = getNavPoseCb
        if self.getNavPose is not None:
            navpose_dict = None
            try:
                navpose_dict = self.getNavPoseCb()
            except:
                pass
            if navpose_dict is None:
                self.getNavPoseCb = None
            else:
                self.has_location = navpose_dict['has_location'] 
                self.has_heading = navpose_dict['has_heading'] 
                self.has_orientation = navpose_dict['has_orientation'] 
                self.has_position = navpose_dict['has_position'] 
                self.has_altitude = navpose_dict['has_altitude'] 
                self.has_depth = navpose_dict['has_depth'] 

        # Create capabilities report
        self.caps_report = NPXCapabilitiesQueryResponse()
        self.caps_report.has_heading = self.has_heading
        self.caps_report.has_position = self.has_position
        self.caps_report.has_orientation = self.has_orientation
        self.caps_report.has_location = self.has_location      
        self.caps_report.has_altitude = self.has_altitude
        self.caps_report.has_depth =  self.has_depth

        self.caps_report.has_transform = self.has_transform
        self.caps_report.supports_transform_updates = self.supports_transform_updates

        self.caps_report.pub_rate_min_max = [self.MIN_PUB_RATE,max_navpose_update_rate]

        self.caps_report.data_products =  self.data_products_list


        # Initialize navpose_dict
        self.navpose_dict['frame_3d'] = self.frame_3d
        self.navpose_dict['frame_nav'] = self.frame_nav
        self.navpose_dict['frame_altitude'] = self.frame_altitude
        self.navpose_dict['frame_depth'] = self.frame_depth

        # Initialize status message
        self.status_msg.device_id = self.device_id
        self.status_msg.identifier = self.identifier
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description
        self.status_msg.device_mount_description = self.get_mount_description() 


        self.status_msg.has_location = self.has_location
        self.status_msg.has_heading = self.has_heading
        self.status_msg.has_orientation = self.has_orientation
        self.status_msg.has_position = self.has_position
        self.status_msg.has_altitude = self.has_altitude
        self.status_msg.has_depth = self.has_depth

        self.status_msg.frame_3d = self.frame_3d

        self.status_msg.supports_transform_updates = self.supports_transform_updates

        self.status_msg.source_frame_nav = self.frame_nav
        self.status_msg.source_frame_altitude = self.frame_altitude
        self.status_msg.source_frame_depth = self.frame_depth

   

        # Create a navpose data IF
        np_namespace = nepi_sdk.create_namespace(self.node_namespace,'npx')
        self.navpose_if = NavPoseIF(namespace = np_namespace,
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            pub_location = self.has_location,
                            pub_heading = self.has_heading,
                            pub_orientation =  self.has_orientation,
                            pub_position = self.has_position,
                            pub_altitude = self.has_altitude,
                            pub_depth = self.has_depth,
                            log_name = 'navpose',
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )


        ##################################################
        ### Node Class Setup

        self.msg_if.pub_info("Starting Node IF Initialization", log_name_list = self.log_name_list)
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.node_namespace
        }



        # Params Config Dict ####################

        self.PARAMS_DICT = {
            'device_name': {
                'namespace': self.node_namespace,
                'factory_val': self.device_name
            },
            'update_rate': {
                'namespace': self.node_namespace,
                'factory_val': self.update_rate
            },
            'mount_desc': {
                'namespace': self.node_namespace,
                'factory_val': 'None'
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
                'msg': DeviceNPXStatus,
                'qsize': 1,
                'latch': True
            }
        }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_device_name': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_device_name',
                'msg': String,
                'qsize': 1,
                'callback': self.updateDeviceNameCb, 
                'callback_args': ()
            },
            'reset_device_name': {
                'namespace': self.node_namespace,
                'topic': 'rbx/reset_device_name',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetDeviceNameCb, 
                'callback_args': ()
            },
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
                'msg': Empty,
                'qsize': 1,
                'callback': self._setHeadSourceCb, 
                'callback_args': ()
            },
            'set_as_location_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_location_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setLocSourceCb, 
                'callback_args': ()
            },
            'set_as_position_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_position_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setPosSourceCb, 
                'callback_args': ()
            },
            'set_as_altitude_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_altitude_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setAltSourceCb, 
                'callback_args': ()
            },            
            'set_as_orientation_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_orientation_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setOrienSourceCb, 
                'callback_args': ()
            },
            'set_as_depth_source': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_as_depth_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setDepthSourceCb, 
                'callback_args': ()
            },
            'set_mount_desc': {
                'namespace': self.node_namespace,
                'topic': 'npx/set_mount_description',
                'msg': String,
                'qsize': 1,
                'callback': self.setMountDescCb, 
                'callback_args': ()
            },
            'reset_mount_desc': {
                'namespace': self.node_namespace,
                'topic': 'npx/reset_mount_description',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetMountDescCb, 
                'callback_args': ()
            }
        }
        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )
      

        ready = self.node_if.wait_for_ready()


        # Setup 3D Transform IF Class ####################
        self.msg_if.pub_debug("Starting 3D Transform IF Initialization", log_name_list = self.log_name_list)
        tranform_ns = nepi_sdk.create_namespace(self.node_namespace,'npx')
        self.transform_if = Frame3DTransformIF(namespace = tranform_ns,
                        source_ref_description = self.tr_source_ref_description,
                        end_ref_description = self.tr_end_ref_description,
                        get_3d_tranform_function = self.get3DTransformCb,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

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
            factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz
        if 'navpose' in self.data_products_list:
            factory_data_rates['navpse'] = [1.0, 0.0, 100.0] 

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "npx",
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

      

        self.initCb(do_updates = True)

        nepi_sdk.start_timer_process(0.1, self._updateNavPoseDictCb, oneshot = True)
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


  def get_mount_description(self):
      desc = self.device_mount_description
      if self.mount_desc != 'None':
          desc = self.mount_desc
      return desc


  def get_3d_transform(self):
    return self.transform_if.get_3d_transform()


  def get_navpose_dict(self):
    return self.navpose_dict

    ###############################
    # Class Private Methods
    ###############################



  def updateDeviceNameCb(self, msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        new_device_name = msg.data
        self.updateDeviceName(new_device_name)

  def updateDeviceName(self, new_device_name):
        valid_name = True
        for char in self.BAD_NAME_CHAR_LIST:
            if new_device_name.find(char) != -1:
                valid_name = False
        if valid_name is False:
            self.msg_if.pub_info("Received invalid device name update: " + new_device_name)
        else:
            self.status_msg.device_name = new_device_name
            self.publish_status(do_updates=False) # Updated inline here 
            self.node_if.set_param('device_name', new_device_name)

  def resetDeviceNameCb(self,msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        self.resetDeviceName()

  def resetDeviceName(self):
        self.status_msg.device_name = self.device_name
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('device_name', self.device_name)



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


  def setMountDescCb(self,msg):
      self.msg_if.pub_info("Recived set mount description message: " + str(msg))
      self.mount_desc = msg.data
      self.publish_status(do_updates=False) # Updated inline here 
      self.node_if.set_param('mount_desc', self.mount_desc)

  def resetMountDescCb(self,msg):
      self.msg_if.pub_info("Recived reset mount description message: " + str(msg))
      self.mount_desc = 'None'
      self.publish_status(do_updates=False) # Updated inline here 
      self.node_if.set_param('mount_desc', self.mount_desc)


  def _setLocSourceCb(self, msg):
      self.msg_if.pub_info("Recived set location source update message: " + str(msg))
      self.setLocSource()

  def setLocSource(self):
      self._setNavPoseSource('location')

  def _setHeadSourceCb(self, msg):
      self.msg_if.pub_info("Recived set heading source update message: " + str(msg))
      self.setHeadSource()

  def setHeadSource(self):
      self._setNavPoseSource('heading')

  def _setOrienSourceCb(self, msg):
      self.msg_if.pub_info("Recived set orientation source update message: " + str(msg))
      self.setOrienSource()

  def setOrienSource(self):
      self._setNavPoseSource('orientation')

  def _setPosSourceCb(self, msg):
      self.msg_if.pub_info("Recived set position source update message: " + str(msg))
      self.setPosSource()

  def setPosSource(self):
       self._setNavPoseSource('position')


  def _setAltSourceCb(self, msg):
      self.msg_if.pub_info("Recived set altitude source update message: " + str(msg))
      self.setAltSource()

  def setAltSource(self):
      self._setNavPoseSource('altitude')

  def _setDepthSourceCb(self, msg):
      self.msg_if.pub_info("Recived set depth source update message: " + str(msg))
      self.setDepthSource()

  def setDepthSource(self):
      self._setNavPoseSource('depth')

  def publishStatus(self, do_updates=True):
      self._publishStatusCb(None)

  def initCb(self,do_updates = False):
      #self.msg_if.pub_info("Setting init values to param values", log_name_list = self.log_name_list)
      if self.node_if is not None:
        self.device_name = self.node_if.get_param('device_name')
        self.update_rate = self.node_if.get_param('update_rate')
        self.mount_desc = self.node_if.get_param('mount_desc')
      if do_updates == True:
        pass
      self.publish_status()

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.reset_params()
      if self.save_data_if is not None:
          self.save_data_if.reset()
      if self.settings_if is not None:
          self.settings_if.reset_settings(update_status = False, update_params = True)
      self.initCb()

  def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.factory_reset_params()
      if self.save_data_if is not None:
          self.save_data_if.factory_reset()
      if self.settings_if is not None:
          self.settings_if.factory_reset(update_status = False, update_params = True)
      self.initCb()



  def _navposeCapabilitiesHandler(self, _):
    return self.caps_report    


  def _updateNavPoseDictCb(self,timer):     
    if self.getNavPoseCb is not None:
        try:
            navpose_dict = self.getNavPoseCb()
        except:
            pass

    if navpose_dict is None:
        navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    # publish navpose data
    transform = self.get_3d_transform()
    self.navpose_dict = self.navpose_if.publish_navpose(self.navpose_dict, 
                                            frame_3d_transform = transform,
                                            device_mount_description = self.get_mount_description())

    # save data if needed
    timestamp = nepi_utils.get_time()
    self.save_data_if.save('navpose',self.navpose_dict,timestamp)

    # set up next loop
    delay = float(1) / self.update_rate
    nepi_sdk.start_timer_process(delay, self._updateNavPoseDictCb, oneshot = True)


  def _setNavPoseSource(self,comp_name):     
    if self.navpose_mgr_if is not None:
        name = comp_name
        topic = self.node_name + '/navpose/' + name
        transform = self.get_3d_transform()
        self.navpose_mgr_if.set_navpose_source(name,topic,transform_list = transform)



  def _publishStatusCb(self,timer):
      self.status_msg.device_name = self.device_name
      self.status_msg.device_mount_description = self.get_mount_description()

      transform = self.get_3d_transform()
      transform_msg = nepi_nav.convert_transform_list2msg(transform)
      transform_msg.source_ref_description = self.data_ref_description
      transform_msg.end_ref_description = 'nepi_frame'
      self.status_msg.frame_3d_transform = transform_msg
      self.status_msg.update_rate = self.update_rate

      self.node_if.publish_pub('status_pub', self.status_msg)

  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions", log_name_list = self.log_name_list)


#########################################
# Main
#########################################


if __name__ == '__main__':
    NavPosePublisher()


