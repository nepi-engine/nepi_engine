#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy
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

    namespace = '~'

    class_if = None
    msg_if = None

    caps_settings = nepi_settings.NONE_CAP_SETTINGS
    factorySettings = nepi_settings.NONE_SETTINGS
    setSettingFunction = None

    capabilities_report = SettingsCapabilitiesQueryResponse()

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

        if namespace is not None:
            self.namespace = namespace
        else:
            self.namespace = self.node_namespace

        #  Initialize capabilities info
        if capSettings is None:
            self.cap_settings = nepi_settings.NONE_CAP_SETTINGS
            cap_setting_msgs_list = nepi_settings.get_cap_setting_msgs_list(self.cap_settings)
            self.capabilities_report.settings_count = len(cap_setting_msgs_list)
            self.capabilities_report.setting_caps_list = cap_setting_msgs_list
            self.factory_settings = nepi_settings.NONE_SETTINGS
        else:
            self.msg_if.pub_info("Got Node settings capabilitis dict : " + str(capSettings))
            self.cap_settings = capSettings   
            cap_setting_msgs_list = nepi_settings.get_cap_setting_msgs_list(self.cap_settings)
            self.capabilities_report.settings_count = len(cap_setting_msgs_list)
            self.capabilities_report.setting_caps_list = cap_setting_msgs_list

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
                'factory_val': dict()
            }
        }

        self.PARAMS_CONFIG_DICT = {
                'init_callback': None,
                'reset_callback': None,
                'factory_reset_callback': None,
                'namespace': self.namespace,
                'params_dict': self.PARAMS_DICT
        }

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'cap_settings': {
                'topic': 'settings_capabilities_query',
                'msg': SettingsCapabilitiesQuery,
                'callback': self._provideCapabilitiesHandler
            }
        }

        self.SRVS_CONFIG_DICT = {
            'namespace': self.namespace,
            'srvs_dict': self.SRVS_DICT
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

        self.PUBS_CONFIG_DICT = {
            'namespace': self.namespace,
            'pubs_dict': self.PUBS_DICT
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'update_setting': {
                'msg': Setting,
                'topic': 'update_setting',
                'qsize': 1,
                'callback': self._updateSettingCb,
                'callback_args': None
            },
            'reset_setting': {
                'msg': Empty,
                'topic': 'reset_settings',
                'qsize': 1,
                'callback': self._resetSettingsCb,
                'callback_args': None
            },
            'factory_reset_setting': {
                'msg': Empty,
                'topic': 'factory_reset_settings',
                'qsize': 1,
                'callback': self._resetFactorySettingsCb,
                'callback_args': None
            },
        }

        self.SUBS_CONFIG_DICT = {
            'namespace': '~',
            'subs_dict': self.SUBS_DICT
        }



        self.class_if = NodeClassIF(params_config_dict = self.PARAMS_CONFIG_DICT,
                        srvs_config_dict = self.SRVS_CONFIG_DICT,
                        pubs_config_dict = self.PUBS_CONFIG_DICT,
                        subs_config_dict = self.SUBS_CONFIG_DICT,
                        log_name = self.class_name,
                        log_class_name = True
        )
   

        self.initialize_settings(do_updates = False)     
        #self.msg_if.pub_info("Cap Settings Message: " + str(self.capabilities_report)   )      

        nepi_ros.sleep(1)
        nepi_ros.start_timer_process(nepi_ros.ros_duration(1), self._publishSettingsCb)
  
        ##############################   
        self.msg_if.pub_info("IF Initialization Complete")
        ##############################  

    ###############################
    # Class Public Methods
    ###############################

    def publish_status(self):
        current_settings = self.getSettingsFunction()
        self.class_if.set_param('settings', current_settings)
        settings_msg = nepi_settings.create_msg_data_from_settings(current_settings)
        if not nepi_ros.is_shutdown():
            #self.msg_if.pub_warn("Publishing settings status msg: " + str(settings_msg))
            self.class_if.publish_pub('status_pub', settings_msg)


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
                        self.class_if.set_param('settings', updated_settings)
                    if update_status:
                        self.publish_status() 
        else:
            self.msg_if.pub_info("Settings updates ignored. No settings update function defined ")
        return success


    def initialize_settings(self, do_updates = True):
        self.msg_if.pub_info("Setting init values to param server values")
        current_settings = self.getSettingsFunction()
        self.init_settings = self.class_if.get_param('settings')
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
        return self.capabilities_report

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


