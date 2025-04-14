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
import copy

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_settings

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.msg import Setting, Settings, SettingCap, SettingCaps
from nepi_ros_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryResponse

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF


SETTING_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]
EXAMPLE_CAP_SETTINGS = {"None":{"name":"None","type":"None","optons":[]}}
EXAMPLE_SETTINGS = {"None":{"name":"None","type":"None","value":"None"}}

class SettingsIF(object):


    # Class Vars ####################

    msg_if = None
    ready = False
    namespace = '~'

    node_if = None


    caps_settings = nepi_settings.NONE_CAP_SETTINGS
    factorySettings = nepi_settings.NONE_SETTINGS
    setSettingFunction = None

    capabilities_response = SettingsCapabilitiesQueryResponse()

    init_settings = dict()
    #######################
    ### IF Initialization
    def __init__(self, 
                    capSettings, 
                    factorySettings,
                    setSettingFunction, 
                    getSettingsFunction, 
                    namespace='~'
                    ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        self.namespace = nepi_ros.get_full_namespace(namespace)

        #  Initialize capabilities info
        if capSettings is None:
            self.cap_settings = nepi_settings.NONE_CAP_SETTINGS
            cap_setting_msgs_list = nepi_settings.get_cap_setting_msgs_list(self.cap_settings)
            self.capabilities_response.settings_count = len(cap_setting_msgs_list)
            self.capabilities_response.setting_caps_list = cap_setting_msgs_list
            self.factory_settings = nepi_settings.NONE_SETTINGS
        else:
            self.msg_if.pub_info("Got Node settings capabilitis dict : " + str(capSettings))
            self.cap_settings = capSettings   
            cap_setting_msgs_list = nepi_settings.get_cap_setting_msgs_list(self.cap_settings)
            self.capabilities_response.settings_count = len(cap_setting_msgs_list)
            self.capabilities_response.setting_caps_list = cap_setting_msgs_list

            if factorySettings is None:
                self.factory_settings = nepi_settings.NONE_SETTINGS
            else:
                self.factory_settings = factorySettings
            #self.msg_if.pub_warn(str(self.factory_settings))

            if setSettingFunction is None:
                self.setSettingFunction = nepi_settings.UPDATE_NONE_SETTINGS_FUNCTION
            else:
                self.setSettingFunction = setSettingFunction
            
            if getSettingsFunction is None:
                self.getSettingsFunction = nepi_settings.GET_NONE_SETTINGS_FUNCTION
            else:
                self.getSettingsFunction = getSettingsFunction
            #Reset Settings and Update Param Server


        ##############################  
        # Create NodeClassIF Class  


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'settings': {
                'namespace': self.namespace,
                'factory_val': dict()
            }
        }

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'cap_settings': {
                'namespace': self.namespace,
                'topic': 'settings_capabilities_query',
                'svr': SettingsCapabilitiesQuery,
                'req': SettingsCapabilitiesQueryRequest(),
                'resp': SettingsCapabilitiesQueryResponse(),
                'callback': self._provideCapabilitiesHandler
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'msg': Settings,
                'topic': 'settings_status',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'update_setting': {
                'msg': Setting,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 1,
                'callback': self._updateSettingCb,
                'callback_args': None
            },
            'reset_setting': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 1,
                'callback': self._resetSettingsCb,
                'callback_args': None
            },
            'factory_reset_setting': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'factory_reset_settings',
                'qsize': 1,
                'callback': self._resetFactorySettingsCb,
                'callback_args': None
            },
        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(params_dict = self.PARAMS_DICT,
                                    services_dict = self.SRVS_DICT,
                                    pubs_dict = self.PUBS_DICT,
                                    subs_dict = self.SUBS_DICT,
                                    log_name = self.class_name,
                                    log_class_name = True
        )
   

        self.node_if.wait_for_ready()


        ##############################
        # Run Initialization Processes
        self.initialize_settings(do_updates = False)     
        #self.msg_if.pub_info("Cap Settings Message: " + str(self.capabilities_response))      
        nepi_ros.start_timer_process(1.0, self._publishSettingsCb)

  
        ##############################
        # Complete Initialization
        self.publish_status()
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
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

    def publish_status(self):
        current_settings = self.getSettingsFunction()
        self.node_if.set_param('settings', current_settings)
        settings_msg = nepi_settings.create_msg_data_from_settings(current_settings)
        if not nepi_ros.is_shutdown():
            #self.msg_if.pub_warn("Publishing settings status msg: " + str(settings_msg))
            self.node_if.publish_pub('status_pub', settings_msg)


    def update_setting(self,new_setting,update_status = True, update_param = True):
        success = False
        current_settings = self.getSettingsFunction()
        updated_settings = copy.deepcopy(current_settings)
        #self.msg_if.pub_warn("New Setting:" + str(new_setting))
        s_name = new_setting['name']
        if self.setSettingFunction != None:
            [name_match,type_match,value_match] = nepi_settings.compare_setting_in_settings(new_setting,current_settings)
            if value_match == False: # name_match would be true for value_match to be true
                self.msg_if.pub_info("Will try to update setting " + str(new_setting))
                [success,msg] = nepi_settings.try_to_update_setting(new_setting,current_settings,self.cap_settings,self.setSettingFunction)
                self.msg_if.pub_warn(msg)
                if success:
                    if update_param:
                        updated_settings[s_name] = new_setting
                        self.node_if.set_param('settings', updated_settings)
                    if update_status:
                        self.publish_status() 
        else:
            self.msg_if.pub_info("Settings updates ignored. No settings update function defined ")
        return success


    def initialize_settings(self, do_updates = True):
        self.msg_if.pub_info("Setting init values to param server values")
        current_settings = self.getSettingsFunction()
        self.init_settings = self.node_if.get_param('settings')
        if do_updates:
            self.reset_settings()


    def reset_settings(self, update_status = True):
        self.msg_if.pub_info("Applying Init Settings")
        #self.msg_if.pub_warn(self.init_settings)
        for setting_name in self.init_settings:
            setting = self.init_settings[setting_name]
            self.update_setting(setting, update_status = False, update_param = False)
        if update_status:
            self.publish_status()

    def factory_reset_settings(self, update_params = True, update_status = True):
        self.msg_if.pub_info("Applying Factory Settings")
        #self.msg_if.pub_warn(self.init_settings)
        for setting_name in self.factory_settings.keys():
            setting = self.factory_settings[setting_name]
            self.update_setting(setting,update_status = False, update_param = update_params)
        if update_status:
            self.publish_status()

    ###############################
    # Class Private Methods
    ###############################

    def _provideCapabilitiesHandler(self, req):
        return self.capabilities_response

    def _publishSettingsCb(self, timer):
        self.publish_status()


    def _updateSettingCb(self,msg):
        self.msg_if.pub_info("Received settings update msg ")
        #self.msg_if.pub_warn(msg)
        setting = nepi_settings.parse_setting_update_msg_data(msg)
        self.update_setting(setting, update_status = True, update_param = True)

    def _resetSettingsCb(self,msg):
        self.reset_settings()

    def _resetFactorySettingsCb(self,msg):
        self.factory_reset_settings()


