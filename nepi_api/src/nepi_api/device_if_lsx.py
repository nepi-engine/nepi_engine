#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import time


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_ros_interfaces.msg import LSXStatus
from nepi_ros_interfaces.srv import LSXCapabilitiesQuery, LSXCapabilitiesQueryRequest, LSXCapabilitiesQueryResponse


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, StatesIF, TriggersIF


class LSXDeviceIF:
    STATUS_UPDATE_RATE_HZ = 1


    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict( standby_enabled = False,
    on_off_state = False,
    intensity_ratio = 0.0,
    color_selection = "None",
    kelvin_val = 4000,
    strobe_enbled = False,
    blink_interval_sec = 0,
    )

    # Define class variables

    ready = False

    getStatusFunction = None
    factory_device_name = "None"
    init_device_name = "None"
  
    has_standby_mode = False
    has_on_off_control = False
    has_intensity_control = False
    has_color_control = False
    color_options_list = ["None"]
    has_kelvin_control = False
    kelvin_limits_list = [1000,10000]
    has_blink_control = False
    has_hw_strobe = False
    reports_temp = False
    reports_power = False



    settings_if = None
    save_cfg_if = None
    
    rbx_status_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)

    #######################
    ### IF Initialization    
    def __init__(self, device_info, getStatusFunction, capSettings, 
                 factorySettings, settingUpdateFunction, getSettingsFunction,
                 factoryControls = None, 
                 standbyEnableFunction = None, turnOnOffFunction = None,
                 setIntensityRatioFunction = None, 
                 color_options_list =  None, setColorFunction = None,
                 kelvin_limits_list = None, setKelvinFunction = None,
                 enableStrobeFunction = None,
                 blinkOnOffFunction = None,
                 reports_temp = False, reports_power = False
                 ):
        
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################
        # Initialize Class Variables

        self.device_name = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]

        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]


        ##############################
        self.initCb(do_updates = False)


        
        # Start LSX node control subscribers
        self.standbyEnableFunction = standbyEnableFunction
        if self.standbyEnableFunction is not None:
            self.has_standby_mode = True
            
        self.turnOnOffFunction = turnOnOffFunction
        if self.turnOnOffFunction is not None:
            self.has_on_off_control = True
            
        self.setIntensityRatioFunction = setIntensityRatioFunction
        if self.setIntensityRatioFunction is not None:
            self.has_intensity_control = True
            
        self.setColorFunction = setColorFunction
        if self.setColorFunction is not None:
            self.has_color_control = True
            if color_options_list != None:
              self.color_options_list = color_options_list
            

        self.setKelvinFunction = setKelvinFunction
        if self.setKelvinFunction is not None:
            self.has_kelvin_control = True
            if kelvin_limits_list != None:
              self.kelvin_limits_list = kelvin_limits_list
            

        self.blinkOnOffFunction = blinkOnOffFunction
        if self.blinkOnOffFunction is not None:
            self.has_blink_control = True
            

        self.enableStrobeFunction = enableStrobeFunction
        if self.enableStrobeFunction is not None:
            self.has_hw_strobe = True
           

        if reports_temp:
          self.reports_temp = True
    
        if reports_power:
          self.reports_power = True


        # Populate and advertise LSX node capabilities report
        self.capabilities_report = LSXCapabilitiesQueryResponse()
        self.capabilities_report.has_standby_mode = self.has_standby_mode
        self.capabilities_report.has_on_off_control = self.has_on_off_control
        self.capabilities_report.has_intensity_control = self.has_intensity_control
        self.capabilities_report.has_color_control = self.has_color_control
        self.capabilities_report.color_options_list = self.color_options_list
        self.capabilities_report.has_kelvin_control = self.has_kelvin_control
        self.capabilities_report.kelvin_min = self.kelvin_limits_list[0]
        self.capabilities_report.kelvin_max = self.kelvin_limits_list[1]
        self.capabilities_report.has_blink_control = self.has_blink_control
        self.capabilities_report.has_hw_strobe = self.has_hw_strobe
        self.capabilities_report.reports_temperature = self.reports_temp
        self.capabilities_report.reports_power = self.reports_power


        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]

        self.getStatusFunction = getStatusFunction


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
            'device_name': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_device_name
            },
            'standby_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict.get("standby_enabled")
            },
            'on_off_state': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict.get("on_off_state")
            },
            'intensity_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict.get("intensity_ratio")
            },
            'color_selection': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict.get("color_selection")
            },
            'kelvin_val': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict.get("kelvin_val")
            }, 
            'strobe_enbled': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict.get("strobe_enbled")
            },
            'blink_interval_sec': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict.get("blink_interval_sec")
            }
        }

        # Services Config Dict ####################

        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.node_namespace,
                'topic': 'lsx/capabilities_query',
                'srv': LSXCapabilitiesQuery,
                'req': LSXCapabilitiesQueryRequest(),
                'resp': LSXCapabilitiesQueryResponse(),
                'callback': self.capabilities_query_callback
            }
        }



        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'lsx/status',
                'msg': LSXStatus,
                'qsize': 1,
                'latch': True
            }
        }



        self.SUBS_DICT = {
            'set_standby': {
                'namespace': self.node_namespace,
                'topic': 'lsx/set_empty',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setStandbyCb, 
                'callback_args': ()
            },
            'turn_on_off': {
                'namespace': self.node_namespace,
                'topic': 'lsx/turn_on_off',
                'msg': Bool,
                'qsize': 1,
                'callback': self.turnOnOffCb, 
                'callback_args': ()
            },
            'set_intensity_ratio': {
                'namespace': self.node_namespace,
                'topic': 'lsx/set_intensity_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setIntensityRatioCb, 
                'callback_args': ()
            },
            'set_color': {
                'namespace': self.node_namespace,
                'topic': 'lsx/set_color',
                'msg': String,
                'qsize': 1,
                'callback': self.setColorCb, 
                'callback_args': ()
            },
            'set_kelvin': {
                'namespace': self.node_namespace,
                'topic': 'lsx/set_kelvin',
                'msg': Int32,
                'qsize': 1,
                'callback': self.setKelvinCb, 
                'callback_args': ()
            },
            'set_blink_interval': {
                'namespace': self.node_namespace,
                'topic': 'lsx/set_blink_interval',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setBlinkIntervalCb, 
                'callback_args': ()
            },
            'set_strobe_enable': {
                'namespace': self.node_namespace,
                'topic': 'lsx/set_strobe_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setStrobeEnableCb, 
                'callback_args': ()
            },
            'update_device_name': {
                'namespace': self.node_namespace,
                'topic': 'lsx/update_device_name',
                'msg': String,
                'qsize': 1,
                'callback': self.updateDeviceNameCb, 
                'callback_args': ()
            },
            'reset_device_name': {
                'namespace': self.node_namespace,
                'topic': 'lsx/reset_device_name',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetDeviceNameCb, 
                'callback_args': ()
            },
            'reset_controls': {
                'namespace': self.node_namespace,
                'topic': 'lsx/reset_controls',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetControlsCb, 
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

        '''
        self.msg_if.pub_info("Starting Save Data IF Initialization")
        # Setup Save Data IF Class ####################
        factory_data_rates = {}
        for d in self.data_products:
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

        '''
 
        time.sleep(1)

        ###############################
        # Finish Initialization
        self.initCb(do_updates = True)
        status_update_time = float(1)/self.STATUS_UPDATE_RATE_HZ
        nepi_ros.start_timer_process(status_update_time, self.statusTimerCb)      
        self.publish_status()
        self.ready = True
        self.msg_if.pub_info("Initialization Complete")


        nepi_ros.sleep

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



        def UpdateDevice(self):
            if self.standbyEnableFunction is not None:
              val = self.node_if.get_param('standby_enabled')
              self.standbyEnableFunction(val)
            if self.turnOnOffFunction is not None:
              val = self.node_if.get_param('on_off_state')
              self.turnOnOffFunction(val)
            if self.setIntensityRatioFunction is not None:
              val = self.node_if.get_param('intensity_ratio')
              self.setIntensityRatioFunction(val)
            if self.setColorFunction is not None:
              val = self.node_if.get_param('color_selection')
              self.setColorFunction(val)
            if self.setKelvinFunction is not None:
              val = self.node_if.get_param('kelvin_val')
              self.setKelvinFunction(val)
            if self.enableStrobeFunction is not None:
              val = self.node_if.get_param('strobe_enbled')
              self.enableStrobeFunction(val)

        nepi_ros.sleep
        def resetControlsCb(self, msg):
            self.msg_if.pub_info("Resetting LSX Device Controls")
            self.node_if.set_param('standby_enabled', self.init_standby_enabled)
            self.node_if.set_param('on_off_state', self.init_on_off_state)
            self.node_if.set_param('intensity_ratio', self.init_intensity_ratio)
            self.node_if.set_param('color_selection', self.init_color_selection)
            self.node_if.set_param('kelvin_val', self.init_kelvin_val)
            self.node_if.set_param('strobe_enbled', self.init_strobe_enbled)
            self.node_if.set_param('blink_interval_sec', self.init_blink_interval_sec)
            self.UpdateDevice()
            self.publish_status()


        def initConfig(self):
            self.initCb(do_updates = True)

        def initCb(self,do_updates = False):
            if self.settings_if is not None:
                self.settings_if.initialize_settings(do_updates)
            if do_updates == True:
              self.resetCb(do_updates)



        def resetCb(self,do_updates = True):
            if self.settings_if is not None:
              self.settings_if.reset_settings()
            if do_updates:
                self.UpdateDevice()
            self.publishStatus()

        def factoryResetCb(self, do_updates = True):
            if self.settings_if is not None:
              self.settings_if.factory_reset_settings()
            if do_updates:
                self.UpdateDevice()
            self.publish_status()


            


        def setCurrentAsDefault(self):
            if self.settings_if is not None:
              self.settings_if.initialize_settings(do_updates = False)
            pass # We only use the param server, no member variables to apply to param server
        
        ## Callback to regulary check device comms, track failures, and kill unresponsive device connections
        def statusTimerCb(self,timer):
          #Update the status message
          self.publish_status()

        ### Status callback
        def publish_status(self):
          # update status values from device
          blink_interval = self.node_if.get_param('blink_interval_sec')
          if self.getStatusFunction is not None:
            status_msg=self.getStatusFunction()
            status_msg.user_name = self.node_if.get_param('device_name')
            status_msg.on_off_state = self.node_if.get_param('on_off_state')
            try:
              self.node_if.publish_pub('status_pub',self.status_msg)
            except Exception as e:
              self.msg_if.pub_info("Failed to publish status msg with exception: " + str(e))


        ### Capabilities callback
        def capabilities_query_callback(self, _):
            return self.capabilities_report

        ### Device Name callbacks
        def updateDeviceNameCb(self, msg):
            self.msg_if.pub_info("Received Device Name update msg")
            self.msg_if.pub_info(msg)
            new_device_name = msg.data
            self.updateDeviceName(new_device_name)

        def updateDeviceName(self, new_device_name):
            valid_name = True
            for char in self.BAD_NAME_CHAR_LIST:
                if new_device_name.find(char) != -1:
                    valid_name = False
            if valid_name is False:
                self.update_error_msg("Received invalid device name update: " + new_device_name)
            else:
                self.node_if.set_param('device_name', new_device_name)
            self.publish_status()


        def resetDeviceNameCb(self,msg):
            self.msg_if.pub_info("Received Device Name reset msg")
            self.msg_if.pub_info(msg)
            self.resetDeviceName()

        def resetDeviceName(self):
            self.node_if.set_param('device_name', self.factory_device_name)
            self.publish_status()

        ### LSX callbacks
        def setStandbyCb(self, standby_msg):
          self.msg_if.pub_info("Recieved standby message: (" + str(standby_msg) + ")")
          standby=standby_msg.data
          if self.standbyEnableFunction is not None:
            self.standbyEnableFunction(standby)
          self.node_if.set_param('standby_enabled', standby)
          self.publish_status()


        def turnOnOffCb(self, on_off_msg):
          self.msg_if.pub_info("Recieved on off message: (" + str(on_off_msg) + ")")
          on_off=on_off_msg.data
          if self.turnOnOffFunction is not None:
            self.turnOnOffFunction(on_off)
          self.node_if.set_param('on_off_state', on_off)
          self.publish_status()

        ### Set intensity callback
        def setIntensityRatioCb(self, intensity_msg):
          self.msg_if.pub_info("Recieved intensity message (" + str(intensity_msg) + ")")
          intensity=intensity_msg.data
          if self.setIntensityRatioFunction is not None:
            self.setIntensityRatioFunction(intensity)
          self.node_if.set_param('intensity_ratio', intensity)
          self.publish_status()

        ### Set color selection callback
        def setColorCb(self, color_msg):
          self.msg_if.pub_info("Recieved color selection message (" + str(color_msg) + ")")
          color = color_msg.data
          if color in self.color_options_list:
            if self.setColorFunction is not None:
              self.setColorFunction(color)
            self.node_if.set_param('color_selection', color)
          self.publish_status()

        ### Set kelvin value callback
        def setKelvinCb(self, kelvin_msg):
          self.msg_if.pub_info("Recieved set kelvin message (" + str(kelvin_msg) + ")")
          kelvin = kelvin_msg.data
          if kelvin >= self.kelvin_limits_list[0] and kelvin <= self.kelvin_limits_list[1]:
            if self.setKelvinFunction is not None:
              self.setKelvinFunction(kelvin)
            self.node_if.set_param('kelvin_val', kelvin)
          self.publish_status()

        def setStrobeEnableCb(self, strobe_enable_msg):
          self.msg_if.pub_info("Recieved strobe enable message (" + str(strobe_enable_msg) + ")")
          strobe_enable=strobe_enable_msg.data
          if self.enableStrobeFunction is not None:
            self.enableStrobeFunction(strobe_enable)
          self.node_if.set_param('strobe_enbled', strobe_enable)
          self.publish_status()


        ### Set blink interval callback
        def setBlinkIntervalCb(self, blink_int_msg):
          self.msg_if.pub_info("Recieved blink interval message (" + str(blink_int_msg) + ")")
          blink_interval = blink_int_msg.data
          if self.blinkOnOffFunction is not None:
            self.blinkOnOffFunction(blink_interval)
          self.node_if.set_param('blink_interval_sec', blink_int)
          self.publish_status()




          
