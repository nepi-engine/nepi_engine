#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy
import time
import copy

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_settings

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.msg import Setting, Settings, SettingCap, SettingCaps
from nepi_ros_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryResponse

'''
Basic interface for the global and private settings topics.
'''
class SettingsIF(object):

    caps_settings = nepi_settings.NONE_CAP_SETTINGS
    factorySettings = nepi_settings.NONE_SETTINGS
    initSettings = nepi_settings.NONE_SETTINGS
    SettingFunction = None

    capabilities_report = SettingsCapabilitiesQueryResponse()
    def __init__(self, capSettings=None, factorySettings=None,SettingFunction=None, getSettingsFunction=None, namespace=None ):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting IF Initialization Processes")
        ##############################        
        if namespace is not None:
            if namespace[-1] != '/':
                namespace = namespace + '/'
                nepi_msg.publishMsgInfo(self,"Using namespace: " + namespace)
            else:
                namespace = namespace + ''
        # Initialize Sensor Settings from Node
        
        self.settings_status_pub = rospy.Publisher(namespace + 'settings_status', Settings, queue_size=1, latch=True)
        time.sleep(1)

        if capSettings is None:
            self.cap_settings = nepi_settings.NONE_CAP_SETTINGS
            cap_setting_msgs_list = nepi_settings.get_cap_setting_msgs_list(self.cap_settings)
            self.capabilities_report.settings_count = len(cap_setting_msgs_list)
            self.capabilities_report.setting_caps_list = cap_setting_msgs_list
            self.factory_settings = nepi_settings.NONE_SETTINGS
        else:
            self.cap_settings = capSettings   
            cap_setting_msgs_list = nepi_settings.get_cap_setting_msgs_list(self.cap_settings)
            self.capabilities_report.settings_count = len(cap_setting_msgs_list)
            self.capabilities_report.setting_caps_list = cap_setting_msgs_list
            if factorySettings is None:
                self.factory_settings = nepi_settings.NONE_SETTINGS
            else:
                self.factory_settings = factorySettings
            #nepi_msg.publishMsgWarn(self,"" + str(self.factory_settings))

            if SettingFunction is None:
                self.SettingFunction = nepi_settings.UPDATE_NONE_SETTINGS_FUNCTION
            else:
                self.SettingFunction = SettingFunction
            
            if getSettingsFunction is None:
                self.getSettingsFunction = nepi_settings.GET_NONE_SETTINGS_FUNCTION
            else:
                self.getSettingsFunction = getSettingsFunction
            #Reset Settings and Update Param Server
        self.init_settings = nepi_ros.get_param(self,namespace + 'settings',self.factory_settings)
        nepi_ros.set_param(self,namespace + 'settings', self.init_settings )
        nepi_msg.publishMsgInfo(self,"Initialize settings: " + str(self.init_settings))
        self.resetFactorySettings(update_params = False, update_status = False)
        self.initializeParamServer(do_updates = False)     
        #nepi_msg.publishMsgInfo(self,"Cap Settings Message: " + str(self.capabilities_report)   )      
  
       # Update settings  and publish current values
        self.updateFromParamServer()

        rospy.Subscriber(namespace + 'update_setting', Setting, self.updateSettingCb, queue_size=1) # start local callbac
        rospy.Subscriber(namespace + 'publish_settings', Empty, self.publishSettingsCb, queue_size=1) # start local callback
        rospy.Subscriber(namespace + 'reset_settings', Empty, self.resetInitSettingsCb, queue_size=1) # start local callback

        # Subscribe to global resets as well
        rospy.Subscriber('reset_settings', Empty, self.resetInitSettingsCb, queue_size=1) # start local callback


        # Start capabilities services
        rospy.Service(namespace + 'settings_capabilities_query', SettingsCapabilitiesQuery, self.provide_capabilities)

 

    def provide_capabilities(self, _):
        return self.capabilities_report

    def publishSettingsCb(self, msg):
        self.publishSettingsStatus()

    def publishSettingsStatus(self):
        current_settings = self.getSettingsFunction()
        nepi_ros.set_param(self,namespace + 'settings', current_settings)
        settings_msg = nepi_settings.create_msg_data_from_settings(current_settings)
        self.settings_status_pub.publish(settings_msg)

    def updateSettingCb(self,msg):
        nepi_msg.publishMsgInfo(self,"Received settings update msg ")
        #nepi_msg.publishMsgInfo(self,msg)
        setting = nepi_settings.parse_setting_update_msg_data(msg)
        self.updateSetting(setting, update_status = True, update_param = True)

    def updateSetting(self,new_setting,update_status = True, update_param = True):
        success = False
        current_settings = self.getSettingsFunction()
        updated_settings = copy.deepcopy(current_settings)
        #nepi_msg.publishMsgWarn(self,"New Setting:" + str(new_setting))
        s_name = new_setting['name']
        if self.SettingFunction != None:
            [name_match,type_match,value_match] = nepi_settings.compare_setting_in_settings(new_setting,current_settings)
            if value_match == False: # name_match would be true for value_match to be true
                nepi_msg.publishMsgInfo(self,"Will try to update setting " + str(new_setting))
                [success,msg] = nepi_settings.try_to_update_setting(new_setting,current_settings,self.cap_settings,self.SettingFunction)
                nepi_msg.publishMsgInfo(self,msg)
                if success:
                    if update_param:
                        updated_settings[s_name] = new_setting
                        nepi_ros.set_param(self,namespace + 'settings', updated_settings)
                    if update_status:
                        self.publishSettingsStatus() 
        else:
            nepi_msg.publishMsgInfo(self,"Settings updates ignored. No settings update function defined ")
        return success

    def initializeParamServer(self, do_updates = True):
        nepi_msg.publishMsgInfo(self,"Setting init values to param server values")
        current_settings = self.getSettingsFunction()
        self.init_settings = nepi_ros.get_param(self,namespace + 'settings', current_settings)
        if do_updates:
            for setting_name in self.init_settings:
                setting = self.init_settings[setting_name]
                self.updateSetting(setting, update_status = False, update_param = False)
            self.publishSettingsStatus()


    def resetInitSettingsCb(self, msg):
        nepi_msg.publishMsgInfo(self,"Recieved settings reset msg")
        self.resetParamServer(do_updates = True)


    def resetParamServer(self, do_updates = True):
        nepi_msg.publishMsgInfo(self,"Resettings to inititial values: " + str(self.init_settings))
        nepi_ros.set_param(self,namespace + 'settings', self.init_settings)
        if do_updates:
            self.updateFromParamServer()
            self.publishSettingsStatus()

    def updateFromParamServer(self):
        nepi_msg.publishMsgInfo(self,"Updating Settings From Param Server")
        settings = nepi_ros.get_param(self,namespace + 'settings', self.init_settings )
        current_settings = self.getSettingsFunction()
        for setting_name in settings:
            setting = settings[setting_name]
            self.updateSetting(setting,update_status = False, update_param = True)
        self.publishSettingsStatus()

    def resetFactorySettings(self, update_params = True, update_status = True):
        nepi_msg.publishMsgInfo(self,"Applying Factory Settings")
        #nepi_msg.publishMsgInfo(self,self.init_settings)
        for setting_name in self.factory_settings.keys():
            setting = self.factory_settings[setting_name]
            self.updateSetting(setting,update_status = False, update_param = update_params)
        if update_status:
            self.publishSettingsStatus()

