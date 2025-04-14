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

SETTING_TYPES = ["Menu","Discnamespacee","String","Bool","Int","Float"]
EXAMPLE_CAP_SETTINGS_DICT = {"None":{"name":"None","type":"None","optons":[]}}
EXAMPLE_SETTINGS_DICT = {"None":{"name":"None","type":"None","value":"None"}}

class ConnectSettingsIF(object):

    msg_if = None
    ready = False
    namespace = '~'

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
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        self.namespace = nepi_ros.get_full_namespace(namespace)


        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'setting_query': {
                'namespace': self.namespace,
                'topic': 'settings_capabilities_query',
                'msg': SettingsCapabilitiesQuery
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'update_pub': {
                'msg': Setting,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 1,
                'latch': False
            },
            'reset_pub': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 1,
                'latch': False
            }
        }


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'settings_sub': {
                'msg': Settings,
                'namespace': self.namespace,
                'topic': 'system_settings',
                'qsize': 1,
                'callback': self._settingsCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT
                        )

        self.con_node_if.wait_for_ready()

        ##############################
        # Run Initialization Processes
        self.cap_settings_dict = self._getCapSettings()   
        #self.msg_if.pub_info("Cap Settings Message: " + str(self.cap_settings_dict))      

        ##############################
        # Complete Initialization
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

    def get_namespace(self):
        return self.namespace

    def get_settings_capabilities_dict(self):
        return self.cap_settings_dict

    def get_settings_dict(self):
        settings_dict = None
        if self.settings_msg is not None:
            settings_dict = nepi_settings.parse_settings_msg(self.settings_msg)
        return settings_dict

    def update_setting(self, setting_dict):
        msg = nepi_settings.create_msg_from_setting(setting_dict)
        success = self.con_node_if.publish_pub('update_pub',msg)
        return success


    def reset_settings(self):
        msg = Empty()
        success = self.con_node_if.publish_pub('reset_pub',msg)
        return success

 

    ###############################
    # Class Private Methods
    ###############################

    def _getCapSettings(self):
        req = SettingsCapabilitiesQueryRequest()
        resp = self.con_node_if.call_service('setting_query',req)
        cap_dict = nepi_settings.parse_cap_settings_msg_data(resp)
        self.ready = True
        return cap_dict

    def _settingsCb(self,msg):
        self.settings_msg = msg


