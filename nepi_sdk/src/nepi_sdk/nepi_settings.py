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
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from std_msgs.msg import Empty, Int8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import Control, SettingsStatus




from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_settings"
logger = Logger(log_name = log_name)


#########################
### Controls Helper Functions




def get_publisher_namespaces(topics_list = None, types_list = None):
    topics_list = nepi_sdk.find_topics_by_msg('SettingsStatus', topics_list = topics_list, types_list = types_list)
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic.replace('/status','')))
    return namespaces_list


############################################################
# Status Msg Functions

def create_status_msg( name = '', display_name = '', description = ''):
  status_msg = SettingsStatus()
  name = nepi_utils.get_clean_name(str(name))
  status_msg.name= name
  if display_name == '':
    display_name = name
  status_msg.display_name= str(display_name)
  if description == '':
    description = name
  status_msg.description= str(description)
  return status_msg


def update_status_msg( status_msg, settings_dict):
  if status_msg is None:
    status_msg = SettingsStatus()


  names_list = [] 
  types_list = [] 
  msgs_list = [] 

  try:
    names = list(settings_dict.keys())
  except Exception as e:
    logger.log_warn("update_status_msg: could not read control names from controls dict: " +
                    type(e).__name__ + ": " + str(e))
    names = []
  for name in names:
    try:
      setting_dict = settings_dict[name]
      setting_type = setting_dict['type']
      if setting_type == 'Discrete':
        setting_type = 'Selection'
      setting_dict['type'] = setting_type
      if setting_type in nepi_controls.CONTROL_TYPES:

        # Convert default and value to string lists for Controls Msg
        value = setting_dict['value']
        default = setting_dict['default']

        if setting_type == 'Trigger':
          if value <= 0:
            value = -999
          else:
            value = nepi_utils.get_time() - value
 
        if setting_type in nepi_controls.LIST_TYPES:
          if isinstance(value, list):
              msg_value = [str(item) for item in value]
              msg_default = [str(item) for item in default]
          else:
            msg_value = [str(value)]
            msg_default = [str(default)]
        else:
          msg_value = [str(value)]
          msg_default = [str(default)]
        setting_dict['value'] = msg_value
        setting_dict['default'] = msg_default

        msg_dict = nepi_sdk.convert_msg2dict(Control())
        for key in msg_dict.keys():
          if key in setting_dict.keys():
            msg_dict[key] = setting_dict[key]

        msg_type = 'nepi_interfaces/Control'
        setting_msg = nepi_sdk.convert_dict2msg(msg_type,msg_dict)


        #logger.log_warn("got Settings Control msg from dict:  " + str([setting_msg,msg_dict,setting_dict]), throttle_s = 10)
        if setting_msg is not None:
          names_list.append(name)
          types_list.append(setting_type)
          msgs_list.append(setting_msg)
      else:
        # Same silent fall-through as create_controls_dict: a control that made
        # it into the dict but carries a type this list does not know is simply
        # left out of the status message, so the RUI never sees it.
        logger.log_warn("update_status_msg: left control '" + str(name) +
                        "' of declared type '" + str(setting_type) +
                        "' out of the status message: type is not one of " + str(nepi_controls.CONTROL_TYPES),
                        throttle_s = 5)
    except Exception as e:
      # Dropped the control from the published status with no log. Throttled,
      # unlike create_controls_dict: this runs on every status publish, not once
      # at registration.
      logger.log_warn("update_status_msg: left control '" + str(name) +
                      "' out of the status message: " +
                      type(e).__name__ + ": " + str(e), throttle_s = 5)
    status_msg.controls_name_list = names_list
    status_msg.controls_msg_list = msgs_list
  return status_msg
