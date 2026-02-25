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


# NEPI ros utility functions include
# 1) ROS Node utility functions
# 2) ROS Topic utility functions
# 3) NEPI ROS Script utility functions
# 4) NEPI Settings utility functions
# 5) Misc helper functions

import rospy

import os
import copy

from nepi_interfaces.msg import Setting, Settings, SettingCap, SettingCaps, SettingsStatus
from nepi_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryRequest, SettingsCapabilitiesQueryResponse

from nepi_sdk import nepi_sdk

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_settings"
logger = Logger(log_name = log_name)


#########################
### Settings Helper Functions

SETTING_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]
NONE_CAP_SETTINGS = {"None":{"name":"None","type":"None","optons":[]}}
NONE_SETTINGS = {"None":{"name":"None","type":"None","value":"None"}}


def get_settings_publisher_namespaces():
    topics_list = nepi_sdk.find_topics_by_msg('nepi_interfaces.msg/Settings')
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic))
    return namespaces_list


def TEST_UPDATE_FUNCTION_SUCCESS(setting):
  s_str = get_setting_as_str(setting)
  logger.log_info("Setting update success: " + s_str)
  return True, "Success"

def TEST_UPDATE_FUNCTION_FAIL(setting):
  s_str = get_setting_as_str(setting)
  logger.log_info("Setting update failed: " + s_str)
  str(2+['value'])
  return False, "Failed just because"

def TEST_UPDATE_FUNCTION_EXCEPTION(setting):
  s_str = get_setting_as_str(setting)
  logger.log_info("Setting update will cauase exception: " + s_str)
  str(2+['value'])
  return True, "Failed with exception"


def UPDATE_NONE_SETTINGS_FUNCTION():
  return False, "No settings update function available"

def GET_NONE_SETTINGS_FUNCTION():
  return NONE_SETTINGS


def parse_cap_setting_msg(cap_setting_msg):
  cap_setting = dict()
  cap_setting['name'] = cap_setting_msg.name_str
  cap_setting['type'] = cap_setting_msg.type_str
  cap_setting['options'] = cap_setting_msg.options_list
  return cap_setting 

def parse_cap_setting_msgs_list(cap_settings_msg_list):
  cap_settings = dict()
  for entry in setting_caps_list:
    cap_settings[entry.name_str] = parse_cap_setting_msg(entry)
  return cap_settings


def create_msg_from_cap_setting(cap_setting):
    cap_setting_msg = SettingCap()
    cap_setting_msg.type_str = cap_setting['type']
    cap_setting_msg.name_str = cap_setting['name']
    if 'options' in cap_setting.keys():
      cap_setting_msg.options_list = cap_setting['options']
    else:
      cap_setting_msg.options_list = []
    return cap_setting_msg


def create_msgs_list_from_cap_settings(cap_settings):
  cap_setting_msgs_list = []
  for cap_setting_name in cap_settings.keys():
    cap_setting = cap_settings[cap_setting_name]
    cap_setting_msg = create_msg_from_cap_setting(cap_setting)
    cap_setting_msgs_list.append(cap_setting_msg)
  return cap_setting_msgs_list

def parse_setting_msg(setting_msg):
  setting = dict()
  setting['name'] = setting_msg.name_str
  setting['type'] = setting_msg.type_str
  setting['value'] = setting_msg.value_str
  return setting

def parse_setting_msgs_list(settings_msg):
  settings = dict()
  for entry in settings_msg.settings_list:
    setting = dict()
    setting['name'] = entry.name_str
    setting['type'] = entry.type_str
    setting['value'] = entry.value_str
    settings[entry.name_str] = setting
  return settings

def create_msg_from_setting(setting):
  setting_msg = Setting()
  setting_msg.type_str = setting['type']
  setting_msg.name_str = setting['name']
  setting_msg.value_str = setting['value']
  return setting_msg

def create_msgs_list_from_settings(settings):
  settings_list = []
  for setting_name in settings.keys():
    setting = settings[setting_name]
    setting_msg = create_msg_from_setting(setting)
    settings_list.append(setting_msg)
  return settings_list


def create_capabilities_response(cap_settings, has_cap_updates = False):
  response = SettingsCapabilitiesQueryResponse()
  response.settings_count = len(cap_settings)
  response.setting_caps_list = create_msgs_list_from_cap_settings(cap_settings)
  response.has_cap_updates = has_cap_updates
  return response

def parse_capabilities_response(response):
  cap_settings = parse_cap_setting_msgs_list(response.setting_caps_list)
  has_cap_updates = response.has_cap_updates
  return(cap_settings,has_cap_updates )


def create_status_msg(settings,cap_settings, has_cap_updates = False):
  status_msg = SettingsStatus()
  if len(settings) == len(cap_settings):
    status_msg.settings_count = len(settings)
    status_msg.settings_list = create_msgs_list_from_settings(settings)
    status_msg.setting_caps_list = create_msgs_list_from_cap_settings(cap_settings)
    status_msg.has_cap_updates = has_cap_updates
  return status_msg

def parse_status_msg(status_msg):
  settings = parse_setting_msgs_list(msg.settings_list)
  cap_settings = parse_cap_setting_msgs_list(msg.setting_caps_list)
  has_cap_updates = status_msg.has_cap_updates
  return(settings, cap_settings,has_cap_updates )





def get_setting_from_settings(setting_name,settings):
  setting = None
  if setting_name in settings.keys():
    setting = settings[setting_name]
  return setting


def get_data_from_setting(setting):

  s_name = 'Missing'
  s_type = 'Missing'
  s_value = 'Missing'
  data = None

  setting_str = str(setting)
  try:
    s_name = setting['name']
    s_type = setting['type']
    s_value = setting['value']
  except Exception as e:
    logger.log_warn("Failed to check setting: " + setting_str + " with exception: " + str(e) )
    return s_name, s_type, data

  if len(setting) == 3:
    if setting['type'] != None and setting['value'] != None:

      s_value = setting['value']
      try:
        if s_type == "Bool":
          data = (s_value == "True")
        elif s_type == "Int":
          data = int(s_value)
        elif s_type == "Float":
          data = float(s_value)
        elif s_type == "String":
          data = s_value
        elif s_type == "Discrete":
          data = s_value
        elif s_type == "Menu":
          data = int(s_value.split(":")[1])
      except Exception as e:
        logger.log_info("Setting conversion failed for setting " + s_str + "with exception" + str(e) )
  return s_name, s_type, data


def compare_setting_in_settings(setting,settings):

  name_match = False
  type_match = False
  value_match = False

  setting_str = str(setting)
  try:
    s_name = setting['name']
    s_type = setting['type']
    s_value = setting['value']
  except Exception as e:
    logger.log_warn("Failed to check setting: " + setting_str + " with exception: " + str(e) )
    return name_match, type_match, value_match
  for setting_name in settings.keys():
    check_setting = settings[setting_name]
    if check_setting['name'] == s_name:
      name_match = True
      if check_setting['type'] == s_type:
        type_match = True
      if check_setting['value'] == s_value:
        value_match = True
      return name_match, type_match, value_match
      break
  return name_match, type_match, value_match

def check_valid_setting(setting,cap_settings):
  valid= False
  setting_str = str(setting)
  try:
    s_name = setting['name']
    s_type = setting['type']
    s_value = setting['value']
  except Exception as e:
    logger.log_warn("Failed to check setting: " + setting_str + " with exception: " + str(e) )
    return False
  if s_name in cap_settings.keys():
      cap_setting = cap_settings[s_name]
      c_type = cap_setting['type']
      if 'options' in cap_setting.keys():
        c_options = cap_setting['options'] 
      else:
        c_options = []
      if s_type == c_type:
        if c_type == "Bool" and (s_value == 'True' or s_value == 'False'):
          valid = True
        elif c_type == "String" and isinstance(s_value,str):
          valid = True      
        elif c_type == "Menu" and isinstance(s_value,str):  
          if s_value in c_options:
            valid = True
        elif c_type == "Discrete" and isinstance(s_value,str):  
          if s_value in c_options:
            valid = True
        elif c_type == "Int":
          try:
            val = int(s_value)
            valid = True
          except Exception as e:
            print(str(e))
          if valid == True:
            if len(c_options) == 2:
              try:
                min = int(c_options[0])
                max = int(c_options[1])
                if val < min or val > max:
                  valid = False
              except:
                pass
        elif c_type == "Float":
          try:
            val = float(s_value)
            valid = True
          except Exception as e:
            print(str(e))
          if valid == True:
            if len(c_options) == 2:
              try:
                min = float(c_options[0])
                max = float(c_options[1])
                if val < min or val > max:
                  valid = False
              except Exception as e:
                print(str(e))
  return valid



def try_to_update_setting(setting_to_update,settings,cap_settings,update_settings_function):
  settings_str = str(setting_to_update)
  s_name = setting_to_update['name']
  updated_settings = copy.deepcopy(settings)
  success = False
  msg = ""
  if s_name != "None":
    if update_settings_function is not None:
      if check_valid_setting(setting_to_update,cap_settings):
        try:
          [success, msg] = update_settings_function(setting_to_update)
          if success:
            msg = ("Updated setting: " + settings_str + " : " + str(msg))
          else:
            msg = ("Failed to update setting: " + settings_str + " : " + str(msg))
        except Exception as e:
          msg = ("Failed to update setting: " + settings_str + " with exception: " + str(e) )
      else:
        msg = ("Failed to update setting: " + settings_str + " Invalid setting name or value with cap settings: " + str(cap_settings))
    else:
      msg = ("Failed to update setting: " + settings_str + " No update function provided")
  else:
    success = True
  return success, msg


def get_settings_by_type(settings,type_str):
  settings_of_type = dict()
  for setting_name in settings.keys():
    setting = settings[setting_name]
    s_name = setting['name']
    s_type = setting['type']
    if s_type == type_str :
        settings_of_type[s_name] = setting
  return settings_of_type


  
