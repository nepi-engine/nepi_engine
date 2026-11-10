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
import time 
import copy


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header


from nepi_interfaces.srv import DeviceLSXStatusQuery, DeviceLSXStatusQueryRequest, DeviceLSXStatusQueryResponse

from nepi_interfaces.msg import DeviceLSXStatus




from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF




from nepi_api.device_if import DeviceIF

# NavPose ConnectNavPosesIF retired (class removed from connect_data_if); LSX has no navpose source.
#from nepi_api.connect_data_if import ConnectNavPosesIF


class LSXDeviceIF(DeviceIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = DeviceLSXStatus()
    DATA_MSG = None

    ID = 'lsx'
    DESCRIPTION = 'Light and Strobe Device'

    DATA_SOURCE_DESCRIPTION = 'Device'
    DATA_REF_DESCRIPTION = 'Device Light Source'

    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict( 
    standby_enabled = False,
    on_off_state = False,
    intensity_ratio = 0.0,
    color_selection = "None",
    kelvin_val = 4000,
    strobe_enbled = False,
    blink_interval_sec = 0,
    )

    #########################
    ## Managed by NodeIF
    #########################

    # id = ''
    # name = ''
    # namespace = '~'

    # status_msg = None
    # data_msg = None

    # msg_if = None
    # node_if = None
    # settings_if = None
    # save_data_if = None
    # transform_if = None
    # navpose_if = None    
    
    # callback_dict = dict()

    # active_topics = []
    # active_topic_types = []
    # active_services = []

    # caps_dict = dict()

    #########################
    ## Managed by DataProductIF
    #########################
    # SYSTEM_IF_DICT = dict()

    # data_source_description = 'sensor'
    # data_ref_description = 'sensor'

    # device_name = ''
    # path = ''
    # serial_num = ''
    # hw_version = ''
    # sw_version = ''

    # factory_controls_dict = dict()


    # msg_if = None
    # node_if = None
    # settings_if = None
    # save_data_if = None
    # transform_if = None
    # navpose_if = None

    # color_image_if = None
    # depthmap_if = None
    # pointcloud_if = None

    #########################
    ## CUSTOM
    #########################
    ready = False

    STATUS_UPDATE_RATE_HZ = 1
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]

    #Factory Control Values 


    # Define class variables

    data_products_list = []

    ready = False

    last_status_msg = None
   

    getStatusFunction = None
   
  
    has_standby_mode = False
    has_on_off_control = False
    has_intensity_control = False
    has_color_control = False
    color_options_list = ["None"]
    has_kelvin_control = False
    kelvin_limits_list = [1000,5000]
    has_blink_control = False
    has_hw_strobe = False
    reports_temp = False
    reports_power = False


    caps_dict = dict()

    #######################
    ### IF Initialization    
    def __init__(self, device_info, 
                 getSettingsFunction=None, setSettingFunction=None, 
                 factoryControls = None, 
                 data_source_description = None,
                 data_ref_description = None,
                 getStatusFunction = None,
                 standbyEnableFunction = None, turnOnOffFunction = None,
                 setIntensityRatioFunction = None, 
                 color_options_list =  None, setColorFunction = None,
                 kelvin_limits_list = None, setKelvinFunction = None,
                 enableStrobeFunction = None,
                 blinkOnOffFunction = None,
                 reports_temp = False, reports_power = False,
                msg_if = None,
                node_if = None,
                settings_if = None,
                save_data_if = None,
                connect_navpose_if = None,
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.caps_dict = copy.deepcopy(self.caps_dict)


        ##############################
        # Update Class Variables
        ##############################
        ##############################
        # Initialize Class Variables
        self.start_time = nepi_utils.get_time()
       
        
        # Start LSX Capability Checks
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
            else:
              self.color_options_list = ["None"]
        else:
            self.color_options_list = ["None"]
            
        self.setKelvinFunction = setKelvinFunction
        
        if self.setKelvinFunction is not None:
            self.has_kelvin_control = True
            if kelvin_limits_list != None:
              self.kelvin_limits_list = kelvin_limits_list
            else:
              self.kelvin_limits_list = [1000, 5000]
        else:
            self.kelvin_limits_list = [1000, 5000]
            

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


        self.getStatusFunction = getStatusFunction


        self.caps_dict['has_standby_mode'] = self.has_standby_mode
        self.caps_dict['has_on_off_control'] = self.has_on_off_control
        self.caps_dict['has_intensity_control'] = self.has_intensity_control
        self.caps_dict['color_options_list'] = self.color_options_list
        self.caps_dict['has_kelvin_control'] = self.has_kelvin_control
        self.caps_dict['has_blink_control'] = self.has_blink_control
        self.caps_dict['has_hw_strobe'] = self.has_hw_strobe
        self.caps_dict['reports_temp'] = self.reports_temp   
        self.caps_dict['reports_power'] = self.reports_power  

        ##############################  
        # Setup NodeIF Updates
        ##############################  

        ##################
        update_info_dict = dict()
        ##################        
        update_caps_dict = self.caps_dict
        ##################        
        update_config_control_dict = dict()
        ##################       
        update_node_controls_dict = dict()
        ##################
        update_stats_dict = dict()
        ##################
        update_rui_dict = dict()
        ##################
        update_callback_dict = dict()

        ###########
        # NODE IF SETUP

        # Call the parent class constructor
        super().__init__(                
                device_info = device_info, 
                getSettingsFunction=getSettingsFunction,
                setSettingFunction=setSettingFunction, 
                factoryControls = factoryControls, 
                data_source_description = data_source_description,
                data_ref_description = data_ref_description,
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict,              
                msg_if = msg_if,
                node_if = node_if,
                settings_if = settings_if,
                save_data_if = save_data_if,
                connect_navpose_if = connect_navpose_if
                )

        ready = self.wait_for_ready()
        ######################################################

        #######################
        # Update NodeIF NodeClass
        #######################

        alt_namespace = None
        if self.device_name != self.node_name:
            alt_namespace = self.node_namespace.replace(self.node_name,self.device_name)

        # Params Config Dict ####################
        params_dict = {
            'standby_enabled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("standby_enabled")
            },
            'on_off_state': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("on_off_state")
            },
            'intensity_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("intensity_ratio")
            },
            'color_selection': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("color_selection")
            },
            'kelvin_val': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("kelvin_val")
            }, 
            'strobe_enbled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("strobe_enbled")
            },
            'blink_interval_sec': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict.get("blink_interval_sec")
            }
        }
    
        # Services Config Dict ####################
        services_dict = {
            'status_query': {
                'namespace': self.namespace,
                'topic': 'status_query',
                'srv': DeviceLSXStatusQuery,
                'req': DeviceLSXStatusQueryRequest(),
                'resp': DeviceLSXStatusQueryResponse(),
                'callback': self.status_query_callback
            }
        }

        # Publishers Config Dict ####################
        pubs_dict = {}
                            

        # Subscribers Config Dict ####################
        subs_dict =  {
            'set_standby': {
                'namespace': self.namespace,
                'topic': 'set_empty',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setStandbyCb, 
                'callback_args': ()
            },
            'turn_on_off': {
                'namespace': self.namespace,
                'topic': 'turn_on_off',
                'msg': Bool,
                'qsize': 1,
                'callback': self.turnOnOffCb, 
                'callback_args': ()
            },
            'set_intensity_ratio': {
                'namespace': self.namespace,
                'topic': 'set_intensity_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setIntensityRatioCb, 
                'callback_args': ()
            },
            'set_color': {
                'namespace': self.namespace,
                'topic': 'set_color',
                'msg': String,
                'qsize': 1,
                'callback': self.setColorCb, 
                'callback_args': ()
            },
            'set_kelvin': {
                'namespace': self.namespace,
                'topic': 'set_kelvin',
                'msg': Int32,
                'qsize': 1,
                'callback': self.setKelvinCb, 
                'callback_args': ()
            },
            'set_blink_interval': {
                'namespace': self.namespace,
                'topic': 'set_blink_interval',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setBlinkIntervalCb, 
                'callback_args': ()
            },
            'set_strobe_enable': {
                'namespace': self.namespace,
                'topic': 'set_strobe_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setStrobeEnableCb, 
                'callback_args': ()
            },
            'reset_controls': {
                'namespace': self.namespace,
                'topic': 'reset_controls',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetControlsCb, 
                'callback_args': ()
            },
            'set_navpose_frame': {
                'namespace': self.namespace,
                'topic': 'set_navpose_frame',
                'msg': String,
                'qsize': 1,
                'callback': self.setMountDescCb, 
                'callback_args': ()
            }

        }



        #######################
        self.add_params(params_dict)
        self.add_services(services_dict)
        self.add_pubs(pubs_dict)
        self.add_subs(subs_dict)
        nepi_sdk.sleep(0.1)

        ######################################################


        #########################
        # Update Status Msg
        ##############################
        # Populate LSX node capabilities, which status_query returns with the status
        self.status_msg.device_name = self.device_name
        self.status_msg.device_path = self.path
        self.status_msg.device_node_name = self.node_name
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        self.status_msg.source_description = self.data_source_description
        self.status_msg.source_ref_description = self.data_ref_description

        self.status_msg.has_standby_mode = self.has_standby_mode
        self.status_msg.has_on_off_control = self.has_on_off_control
        self.status_msg.has_intensity_control = self.has_intensity_control
        self.status_msg.has_color_control = self.has_color_control
        self.status_msg.color_options_list = self.color_options_list
        self.status_msg.has_kelvin_control = self.has_kelvin_control
        self.status_msg.kelvin_min = self.kelvin_limits_list[0]
        self.status_msg.kelvin_max = self.kelvin_limits_list[1]
        self.status_msg.has_blink_control = self.has_blink_control
        self.status_msg.has_hw_strobe = self.has_hw_strobe
        self.status_msg.reports_temperature = self.reports_temp
        self.status_msg.reports_power = self.reports_power




        ##############################
        # Start Node Processes
        ##############################



        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", color = 'GREEN')
        ####################################

        ##############################
        # Start NodeIF Processes
        if type(self) is LSXDeviceIF:
            self.start()



    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        """Builds and publishes the DeviceIDXStatus message on the status topic.

        Populates all fields of the status message from current instance state,
        including device identity, FOV angles, resolution, framerate, image
        control ratios, range-window settings, and per-data-product FPS. When
        ``do_updates`` is True, also queries the RTSP URL callback (if
        registered) and includes the result in the message.

        Args:
            do_updates (bool, optional): When True, perform live queries (e.g.
                RTSP URL) before publishing. Set to False when the caller has
                already updated state inline and a lightweight publish is
                sufficient. Defaults to True.
        """

        """Builds and publishes the DeviceNPXStatus message to the status topic.

        Updates the device name and current update rate on the status message, then
        publishes it on the latched status publisher if the node interface is available.
        """
        blink_interval = self.node_if.get_param(self.node_if_prefix + 'blink_interval_sec')
        if self.getStatusFunction is not None:
            status_msg=self.getStatusFunction()
            status_msg.on_off_state = self.node_if.get_param(self.node_if_prefix + 'on_off_state')
            # The driver builds this msg, so stamp the IF-owned identity and
            # capabilities onto it, then keep it for status_query to return
            status_msg.device_name = self.device_name
            status_msg.device_path = self.path
            status_msg.device_node_name = self.node_name
            status_msg.has_standby_mode = self.has_standby_mode
            status_msg.has_on_off_control = self.has_on_off_control
            status_msg.has_intensity_control = self.has_intensity_control
            status_msg.has_color_control = self.has_color_control
            status_msg.color_options_list = self.color_options_list
            status_msg.has_kelvin_control = self.has_kelvin_control
            status_msg.kelvin_min = self.kelvin_limits_list[0]
            status_msg.kelvin_max = self.kelvin_limits_list[1]
            status_msg.has_blink_control = self.has_blink_control
            status_msg.has_hw_strobe = self.has_hw_strobe
            status_msg.reports_temperature = self.reports_temp
            status_msg.reports_power = self.reports_power
            status_msg.data_products = self.data_products_list

            self.last_status_msg = status_msg
        ###################
        # TOPIC UPDATES
        
        ########
        # NAVPOSE DATA
        try:
           navpose_topic = self.navpose_if.get_namespace()
        except:
           navpose_topic = ''
        status_msg.navpose_topic = navpose_topic


        try:
           navpose_msg = self.navpose_if.get_navpose_msg()
           status_msg.navpose_msg = navpose_msg
        except:
           pass

        try:
           connect_navpose_topic = self.connect_navpose_if.get_namespace()
        except:
           connect_navpose_topic = ''
        status_msg.connect_navpose_topic = connect_navpose_topic
        return status_msg


    def initCb(self,do_updates = False):
      if self.node_if is not None:
        self.init_standby_enabled = self.node_if.get_param(self.node_if_prefix + 'standby_enabled')
        self.init_on_off_state = self.node_if.get_param(self.node_if_prefix + 'on_off_state')
        self.init_intensity_ratio = self.node_if.get_param(self.node_if_prefix + 'intensity_ratio')
        self.init_color_selection = self.node_if.get_param(self.node_if_prefix + 'color_selection')
        self.init_kelvin_val = self.node_if.get_param(self.node_if_prefix + 'kelvin_val')
        self.init_strobe_enbled = self.node_if.get_param(self.node_if_prefix + 'strobe_enbled')
        self.init_blink_interval_sec = self.node_if.get_param(self.node_if_prefix + 'blink_interval_sec')

      if do_updates == True:
        self.updateDevice()

    ###############################
    # Class Methods
    ###############################

    def resetControlsCb(self, msg):
        self.msg_if.pub_info("Resetting LSX Device Controls" )
        self.node_if.set_param(self.node_if_prefix + 'standby_enabled', self.init_standby_enabled)
        self.node_if.set_param(self.node_if_prefix + 'on_off_state', self.init_on_off_state)
        self.node_if.set_param(self.node_if_prefix + 'intensity_ratio', self.init_intensity_ratio)
        self.node_if.set_param(self.node_if_prefix + 'color_selection', self.init_color_selection)
        self.node_if.set_param(self.node_if_prefix + 'kelvin_val', self.init_kelvin_val)
        self.node_if.set_param(self.node_if_prefix + 'strobe_enbled', self.init_strobe_enbled)
        self.node_if.set_param(self.node_if_prefix + 'blink_interval_sec', self.init_blink_interval_sec)
        self.updateDevice()
        self.publish_status()


    def setMountDescCb(self,msg):
        self.msg_if.pub_info("Recived set navpose_frame message: " + str(msg))
        self.navpose_frame = msg.data
        self.publish_status() # Updated inline here 
        self.node_if.set_param(self.node_if_prefix + 'navpose_frame', self.navpose_frame)

    def resetMountDescCb(self,msg):
        self.msg_if.pub_info("Recived reset navpose_frame message: " + str(msg))
        self.navpose_frame = 'None'
        self.publish_status() # Updated inline here 
        self.node_if.set_param(self.node_if_prefix + 'navpose_frame', self.navpose_frame)





    def updateDevice(self):
        # Reachable before self.node_if is bound: NodeClassIF.__init__ builds a
        # NodeConfigsIF, whose __init__ calls reset_config() -> this class's
        # resetCb -> initCb(do_updates=True) -> here, all while the
        # 'self.node_if = NodeClassIF(...)' assignment is still evaluating.
        # initCb and resetCb each guard on node_if for that reason; this did not,
        # so every param read below raised AttributeError on None. The premature
        # call is redundant anyway -- initCb(do_updates=True) runs again right
        # after node_if is assigned, which is when the device actually gets pushed.
        if self.node_if is None:
            return
        if self.standbyEnableFunction is not None:
          val = self.node_if.get_param(self.node_if_prefix + 'standby_enabled')
          self.standbyEnableFunction(val)
        if self.turnOnOffFunction is not None:
          val = self.node_if.get_param(self.node_if_prefix + 'on_off_state')
          self.turnOnOffFunction(val)
        if self.setIntensityRatioFunction is not None:
          val = self.node_if.get_param(self.node_if_prefix + 'intensity_ratio')
          self.setIntensityRatioFunction(val)
        if self.setColorFunction is not None:
          val = self.node_if.get_param(self.node_if_prefix + 'color_selection')
          self.setColorFunction(val)
        if self.setKelvinFunction is not None:
          val = self.node_if.get_param(self.node_if_prefix + 'kelvin_val')
          self.setKelvinFunction(val)
        if self.enableStrobeFunction is not None:
          val = self.node_if.get_param(self.node_if_prefix + 'strobe_enbled')
          self.enableStrobeFunction(val)
        


    def setCurrentAsDefault(self):
        if self.settings_if is not None:
          # initialize_settings() has never existed on SettingsIF -- this raised
          # AttributeError on every setCurrentAsDefault(). init() is the method.
          self.settings_if.init(do_updates = False)
        pass # We only use the param server, no member variables to apply to param server
    
  
        
        


    ### Capabilities callback
    def status_query_callback(self, _):
        """Service handler that returns the LSX capabilities report.

        Returns:
            DeviceLSXtatusQueryResponse: The last published DeviceLSXStatus in
                device_status (the IF's own status before the first publish),
                describing which LSX capabilities are supported, including standby mode,
                on/off control, intensity control, color control, kelvin control, blink
                control, hardware strobe, temperature reporting, and power reporting.
        """
        status_msg = self.status_msg
        if self.last_status_msg is not None:
            status_msg = self.last_status_msg
        return DeviceLSXStatusQueryResponse(device_status = status_msg)

    ### LSX callbacks
    def setStandbyCb(self, standby_msg):
      self.msg_if.pub_info("Recieved standby message: " + str(standby_msg) )
      standby=standby_msg.data
      self.set_value('standby_enabled',standby)
      if self.standbyEnableFunction is not None:
        self.standbyEnableFunction(standby)



    def turnOnOffCb(self, on_off_msg):
      self.msg_if.pub_info("Recieved on off message: " + str(on_off_msg) )
      on_off=on_off_msg.data
      self.set_value('on_off_state',on_off)
      if self.turnOnOffFunction is not None:
        self.turnOnOffFunction(on_off)


    ### Set intensity callback
    def setIntensityRatioCb(self, intensity_msg):
      self.msg_if.pub_info("Recieved intensity message: " + str(intensity_msg) )
      intensity=intensity_msg.data
      self.set_value('intensity_ratio',intensity)
      if self.setIntensityRatioFunction is not None:
        self.setIntensityRatioFunction(intensity)


    ### Set color selection callback
    def setColorCb(self, color_msg):
      self.msg_if.pub_info("Recieved color selection message: " + str(color_msg) )
      color = color_msg.data
      if color in self.color_options_list:
        self.set_value('color_selection',color)
        if self.setColorFunction is not None:
          self.setColorFunction(color)
  

    ### Set kelvin value callback
    def setKelvinCb(self, kelvin_msg):
      self.msg_if.pub_info("Recieved set kelvin message: " + str(kelvin_msg) )
      kelvin = kelvin_msg.data
      if kelvin >= self.kelvin_limits_list[0] and kelvin <= self.kelvin_limits_list[1]:
        self.set_value('kelvin_val',kelvin)
        if self.setKelvinFunction is not None:
          self.setKelvinFunction(kelvin)


    def setStrobeEnableCb(self, strobe_enable_msg):
      self.msg_if.pub_info("Recieved strobe enable message: " + str(strobe_enable_msg) )
      strobe_enable=strobe_enable_msg.data
      self.set_value('strobe_enbled',strobe_enable)
      if self.enableStrobeFunction is not None:
        self.enableStrobeFunction(strobe_enable)
      


    ### Set blink interval callback
    def setBlinkIntervalCb(self, blink_int_msg):
      self.msg_if.pub_info("Recieved blink interval message: " + str(blink_int_msg) )
      blink_interval = blink_int_msg.data
      self.set_value('blink_interval_sec',blink_interval)
      if self.blinkOnOffFunction is not None:
        self.blinkOnOffFunction(blink_interval)
   



      
