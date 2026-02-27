#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


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


from geometry_msgs.msg import Vector3

from nepi_interfaces.msg import DeviceNPXStatus
from nepi_interfaces.srv import  NPXCapabilitiesQuery, NPXCapabilitiesQueryRequest, NPXCapabilitiesQueryResponse


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_settings

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF

from nepi_api.data_if import NavPoseIF


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


  data_products_list = ['navpose']

  node_if = None
  settings_if = None
  save_data_if = None
  navpose_if = None

  status_msg = DeviceNPXStatus()
  
  update_rate = DEFAULT_UPDATE_RATE
  navpose_frame = DEFAULT_3D_FRAME
  frame_nav = DEFAULT_NAV_FRAME
  frame_altitude = DEFAULT_ALT_FRAME
  frame_depth = DEFAULT_DEPTH_FRAME


  has_location = False  
  has_heading = False
  has_orientation = False
  has_position = False
  has_altitude = False
  has_depth = False
  has_pan_tilt = False
  supports_updates = True

  set_location_source = False
  set_heading_source = False
  set_orientation_source = False  
  set_position_source = False
  set_altitude_source = False
  set_depth_source = False
  set_pan_tilt_source = False

  was_location_source = False
  was_heading_source = False
  was_orientation_source = False  
  was_position_source = False
  was_altitude_source = False
  was_depth_source = False
  was_pan_tilt_source = False

  navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

  navpose_if = None  

  settings_if = None

  has_updated = False

  pub_subs = False

  navpose_mgr_if = None

  data_source_description = 'navpose_sensor'
  data_ref_description = 'data_reference'
  navpose_frame = 'None'

  getNavPoseCb = None
  supports_updates = True

  #######################
  ### IF Initialization
  def __init__(self, 
                device_info,
                node_namespace = None,
                capSettings=None, factorySettings=None, 
                settingUpdateFunction=None, getSettingsFunction=None,
                data_source_description = 'navpose_sensor',
                data_ref_description = 'data_reference',
                navpose_frame = 'sensor_frame', frame_nav = 'ENU',
                frame_altitude = 'WGS84', frame_depth = 'DEPTH',
                getNavPoseCb = None,
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
        if node_namespace is not None:
            self.namespace = nepi_sdk.create_namespace(node_namespace,'npx')
        else:
            self.namespace = nepi_sdk.create_namespace(self.node_namespace,'npx')
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
        # Initialize Class 
   
        self.device_id = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.device_name = self.device_id + "_" + self.identifier

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description
        self.tr_source_ref_description = data_ref_description
        

        if navpose_frame is not None:
          self.navpose_frame = navpose_frame

        if frame_nav is not None:
          self.frame_nav = frame_nav

        if frame_altitude is not None:
          self.frame_altitude = frame_altitude

        if frame_depth is not None:
          self.frame_depth = frame_depth
       

        if getNavPoseCb is not None:
            navpose_dict = None
            try:
                navpose_dict = getNavPoseCb()

            except:
                print("ERROR in getNavPoseCb():", e)

            if navpose_dict is None:
                self.getNavPoseCb = None
            else:
                self.getNavPoseCb = getNavPoseCb
                self.has_location = navpose_dict['has_location'] 
                self.has_heading = navpose_dict['has_heading'] 
                self.has_orientation = navpose_dict['has_orientation'] 
                self.has_position = navpose_dict['has_position'] 
                self.has_altitude = navpose_dict['has_altitude'] 
                self.has_depth = navpose_dict['has_depth'] 
                self.has_pan_tilt = navpose_dict['has_pan_tilt']
        

        # Create capabilities report
        self.caps_report = NPXCapabilitiesQueryResponse()
        self.caps_report.has_heading = self.has_heading
        self.caps_report.has_position = self.has_position
        self.caps_report.has_orientation = self.has_orientation
        self.caps_report.has_location = self.has_location      
        self.caps_report.has_altitude = self.has_altitude
        self.caps_report.has_depth =  self.has_depth
        self.caps_report.has_pan_tilt = self.has_pan_tilt


        self.caps_report.supports_updates = self.supports_updates

        self.caps_report.pub_rate_min_max = [self.MIN_PUB_RATE,max_navpose_update_rate]

        self.caps_report.data_products =  self.data_products_list


        # Initialize navpose_dict
        self.navpose_dict['navpose_frame'] = self.navpose_frame
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


        self.status_msg.has_location = self.has_location
        self.status_msg.has_heading = self.has_heading
        self.status_msg.has_orientation = self.has_orientation
        self.status_msg.has_position = self.has_position
        self.status_msg.has_altitude = self.has_altitude
        self.status_msg.has_depth = self.has_depth
        self.status_msg.has_pan_tilt = self.has_pan_tilt

        self.status_msg.navpose_frame = self.navpose_frame

        self.status_msg.source_frame_nav = self.frame_nav
        self.status_msg.source_frame_altitude = self.frame_altitude
        self.status_msg.source_frame_depth = self.frame_depth

   

        ##################################################
        ### Node Class Setup

        self.msg_if.pub_info("Starting Node IF Initialization", log_name_list = self.log_name_list)
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.namespace
        }



        # Params Config Dict ####################

        self.PARAMS_DICT = {
            'device_name': {
                'namespace': self.namespace,
                'factory_val': self.device_name
            },
            'update_rate': {
                'namespace': self.namespace,
                'factory_val': self.update_rate
            },
            'navpose_frame': {
                'namespace': self.namespace,
                'factory_val': 'None'
            }

        }

        # Services Config Dict ####################

        self.SRVS_DICT = {
            'navpose_capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': NPXCapabilitiesQuery,
                'req': NPXCapabilitiesQueryRequest(),
                'resp': NPXCapabilitiesQueryResponse(),
                'callback': self._navposeCapabilitiesHandler
            }
        }

        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DeviceNPXStatus,
                'qsize': 1,
                'latch': True
            }
        }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_device_name': {
                'namespace': self.namespace,
                'topic': 'rbx/set_device_name',
                'msg': String,
                'qsize': 1,
                'callback': self.updateDeviceNameCb, 
                'callback_args': ()
            },
            'reset_device_name': {
                'namespace': self.namespace,
                'topic': 'rbx/reset_device_name',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetDeviceNameCb, 
                'callback_args': ()
            },
            'set_update_rate': {
                'namespace': self.namespace,
                'topic': 'set_update_rate',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setUpdateRateCb, 
                'callback_args': ()
            },
            'set_as_heading_source': {
                'namespace': self.namespace,
                'topic': 'set_as_heading_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setHeadSourceCb, 
                'callback_args': ()
            },
            'set_as_location_source': {
                'namespace': self.namespace,
                'topic': 'set_as_location_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setLocSourceCb, 
                'callback_args': ()
            },
            'set_as_position_source': {
                'namespace': self.namespace,
                'topic': 'set_as_position_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setPosSourceCb, 
                'callback_args': ()
            },
            'set_as_altitude_source': {
                'namespace': self.namespace,
                'topic': 'set_as_altitude_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setAltSourceCb, 
                'callback_args': ()
            },            
            'set_as_orientation_source': {
                'namespace': self.namespace,
                'topic': 'set_as_orientation_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setOrienSourceCb, 
                'callback_args': ()
            },
            'set_as_depth_source': {
                'namespace': self.namespace,
                'topic': 'set_as_depth_source',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setDepthSourceCb, 
                'callback_args': ()
            },
            'set_navpose_frame': {
                'namespace': self.namespace,
                'topic': 'set_navpose_frame',
                'msg': String,
                'qsize': 1,
                'callback': self.setMountDescCb, 
                'callback_args': ()
            },
            'reset_navpose_frame': {
                'namespace': self.namespace,
                'topic': 'reset_navpose_frame',
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
      
        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.initCb(do_updates = True)
        self.publish_status()

        
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        # Setup Settings IF Class ####################
        self.msg_if.pub_info("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = self.namespace
        
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
            factory_data_rates[d] = [0.0, 0.0, 100] # Default to 0Hz save rate, set last save = 0.0, max rate = 100Hz
        if 'navpose' in self.data_products_list:
            factory_data_rates['navpse'] = [1.0, 0.0, 100] 

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_us': False,
            'suffix': "npx",
            'add_node_name': True
            }

        sd_namespace = self.namespace
        self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                            factory_rate_dict = factory_data_rates,
                            factory_filename_dict = factory_filename_dict,
                            namespace = sd_namespace,
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )


        # Create a NavPose IF
        np_namespace = self.namespace
        self.navpose_if = NavPoseIF(namespace = np_namespace,
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            pub_navpose = True,
                            pub_location = self.has_location,
                            pub_heading = self.has_heading,
                            pub_orientation =  self.has_orientation,
                            pub_position = self.has_position,
                            pub_altitude = self.has_altitude,
                            pub_depth = self.has_depth,
                            pub_pan_tilt = self.has_pan_tilt,
                            log_name = 'navpose',
                            log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                            )


        #####################
        # Update Status Message
        nepi_sdk.sleep(1)
        if self.settings_if is not None:
            self.status_msg.settings_topic = self.settings_if.get_namespace()
            self.msg_if.pub_info("Using settings namespace: " + str(self.status_msg.settings_topic))
        if self.save_data_if is not None:
            self.status_msg.save_data_topic = self.save_data_if.get_namespace()
            self.msg_if.pub_info("Using save_data namespace: " + str(self.status_msg.save_data_topic))
        if self.navpose_if is not None:            
            self.status_msg.navpose_topic = self.navpose_if.get_namespace()
            self.msg_if.pub_info("Using navpose namespace: " + str(self.status_msg.navpose_topic))
      

        self.initCb(do_updates = True)

        nepi_sdk.start_timer_process(0.1, self._updateNavPoseDictCb, oneshot = True)
        
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
      self.navpose_frame = frame
      self.node_if.set_param('navpose_frame',frame)

  def _setAltFrameCb(self,msg):
    frame = msg.data
    if frame in self.NAVPOSE_FRAME_ALT_OPTIONS:
      self.frame_altitude = frame
      self.node_if.set_param('frame_altitude',frame)


  def setMountDescCb(self,msg):
      self.msg_if.pub_info("Recived set navpose_frame message: " + str(msg))
      self.navpose_frame = msg.data
      self.publish_status(do_updates=False) # Updated inline here 
      self.node_if.set_param('navpose_frame', self.navpose_frame)

  def resetMountDescCb(self,msg):
      self.msg_if.pub_info("Recived reset navpose_frame message: " + str(msg))
      self.navpose_frame = 'None'
      self.publish_status(do_updates=False) # Updated inline here 
      self.node_if.set_param('navpose_frame', self.navpose_frame)


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

  def initConfig(self):
      self.initCb()

  def initCb(self,do_updates = False):
      #self.msg_if.pub_info("Setting init values to param values", log_name_list = self.log_name_list)
      if self.node_if is not None:
        self.device_name = self.node_if.get_param('device_name')
        self.update_rate = self.node_if.get_param('update_rate')
        self.navpose_frame = self.node_if.get_param('navpose_frame')
      if do_updates == True:
        pass
      self.publish_status()

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.reset_params()
      if self.save_data_if is not None:
          self.save_data_if.reset()
      if self.settings_if is not None:
          self.settings_if.reset()
      if self.navpose_if is not None:
          self.navpose_if.reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)

  def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.factory_reset_params()
      if self.save_data_if is not None:
          self.save_data_if.factory_reset()
      if self.settings_if is not None:
          self.settings_if.factory_reset()
      if self.navpose_if is not None:
          self.navpose_if.factory_reset()
      if do_updates == True:
        pass
      self.initCb(do_updates = True)



  def _navposeCapabilitiesHandler(self, _):
    return self.caps_report    


  def _updateNavPoseDictCb(self,timer):    
    navpose_dict = None 
    if self.getNavPoseCb is not None:
        try:
            navpose_dict = self.getNavPoseCb()
        except:
            pass

    if navpose_dict is None:
        navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    # publish navpose data

    if self.navpose_if is not None:
        self.navpose_dict = self.navpose_if.publish_navpose(navpose_dict)

    # save data if needed
    timestamp = nepi_utils.get_time()
    self.save_data_if.save('navpose',self.navpose_dict,timestamp)

    # set up next loop
    delay = float(1) / self.update_rate
    nepi_sdk.start_timer_process(delay, self._updateNavPoseDictCb, oneshot = True)



  def _publishStatusCb(self,timer):
    self.publish_status()

  def publish_status(self): 
      self.status_msg.device_name = self.device_name
      self.status_msg.update_rate = self.update_rate

      if self.node_if is not None:
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


