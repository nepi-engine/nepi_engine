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
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.msg import Setting, Settings, SettingCap, SettingCaps
from nepi_ros_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryRequest, SettingsCapabilitiesQueryResponse

from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.sys_if_msg import MsgIF

SETTING_TYPES = ["Menu","Discsettings_namespacee","String","Bool","Int","Float"]
EXAMPLE_CAP_SETTINGS_DICT = {"None":{"name":"None","type":"None","optons":[]}}
EXAMPLE_SETTINGS_DICT = {"None":{"name":"None","type":"None","value":"None"}}

class ConnectSettingsIF(object):

    connected = False

    settings_namespace = None

    cap_settings_dict = None

    settings_msg = None

    #######################
    ### IF Initialization
    def __init__(self, namespace , timeout = float('inf')):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        log_name = self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        

        #############################
        if namespace is None:
            namespace = self.base_namespace
        self.settings_namespace = namespace


        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'setting_query': {
                'topic': 'settings_capabilities_query',
                'msg': SettingsCapabilitiesQuery
            }
        }

        self.SRVS_CONFIG_DICT = {
                'namespace': self.settings_namespace,
                'srvs_dict': self.SRVS_DICT
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'update_pub': {
                'msg': Setting,
                'topic': 'update_setting',
                'qsize': 1,
                'latch': False
            },
            'reset_pub': {
                'msg': Empty,
                'topic': 'reset_settings',
                'qsize': 1,
                'latch': False
            }
        }

        self.PUBS_CONFIG_DICT = {
            'namespace': self.settings_namespace,
            'pubs_dict': self.PUBS_DICT
        }



        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'settings_sub': {
                'msg': Settings,
                'topic': 'system_settings',
                'qsize': 1,
                'callback': self._settingsCb,
                'callback_args': ()
            }
        }

        self.SUBS_CONFIG_DICT = {
            'namespace': self.settings_namespace,
            'subs_dict': self.SUBS_DICT
        }


        self.class_if = ConnectNodeClassIF(srvs_config_dict = self.SRVS_CONFIG_DICT,
                        pubs_config_dict = self.PUBS_CONFIG_DICT,
                        subs_config_dict = self.SUBS_CONFIG_DICT
                        )

        ### Get Settings Capabilities
        self.cap_settings_dict = self._getCapSettings()
        if self.cap_settings_dict is not None:
            self.connected = True

        ##############################   
        self.msg_if.publish_info("IF Initialization Complete")


    ###############################
    # Class Public Methods
    ###############################

    def check_connection(self):
        return self.connected

    def get_namespace(self):
        return self.settings_namespace

    def get_settings_capabilities_dict(self):
        return self.cap_settings_dict

    def get_settings_dict(self):
        settings_dict = None
        if self.settings_msg is not None:
            settings_dict = nepi_settings.parse_settings_msg(self.settings_msg)
        return settings_dict

    def update_setting(self, setting_dict):
        msg = nepi_settings.create_msg_from_setting(setting_dict)
        success = self.class_if.publish_pub('update_pub',msg)
        return success


    def reset_settings(self):
        msg = Empty()
        success = self.class_if.publish_pub('reset_pub',msg)
        return success

 

    ###############################
    # Class Private Methods
    ###############################

    def _getCapSettings(self):
        req = SettingsCapabilitiesQueryRequest()
        resp = self.class_if.call_service('setting_query',req)
        cap_dict = nepi_settings.parse_cap_settings_msg_data(resp)
        return cap_dict

    def _settingsCb(self,msg):
        self.settings_msg = msg

