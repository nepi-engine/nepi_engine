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

from std_msgs.msg import Empty, Int8, UInt32, Int32, Bool, String, StringArray, Float32, Float64

from nepi_interfaces.msg import Control, ControlCap, ControlsStatus


from nepi_sdk import nepi_sdk

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_controls"
logger = Logger(log_name = log_name)


#########################
### Controls Helper Functions




CONTROL_TYPES = ["Menu","String","StringArray","Trigger","Bool","Int","Float","FloatSlider","FloatsSlider"]

BLANK_CONTROL_DICT = {"name":"None",
                      "type":'None',
                      'order':0,
                      'optons':[],
                      'default':'',
                      'value': '',
                      'display_name':'',
                      'description':'',
                      'hidden':False,}

NONE_CONTROLS_DICT = {"None":BLANK_CONTROL_DICT}

EXAMPLE_CONTROLS_DICT = dict(
      pub_rate = {"name":"rate","type":"Float","order":1,"options":[0.1,15],"default":2,'value':4,'display_name':'Pub Rate','description':'Set pub rate','hidden':False},
      index = {"name":"index","type":"Int","order":2,"options":[3,10],"default":3,'value':3,'display_name':'Select Index','description':'Set index','hidden':False},
    )


def get_controls_publisher_namespaces():
    topics_list = nepi_sdk.find_topics_by_msg('nepi_interfaces.msg/ControlsStatus')
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic.replace('/status','')))
    return namespaces_list


def parse_control_msg(control_msg):
  cap_control = dict()
  cap_control['name'] = control_msg.name_str
  cap_control['type'] = control_msg.type_str
  cap_control['options'] = control_msg.options_list
  return cap_control 

def parse_cap_control_msgs_list(constrols_cap_list_msg):
  cap_controls = dict()
  for entry in constrols_cap_list_msg:
    cap_controls[entry.name_str] = parse_cap_control_msg(entry)
  return cap_controls


def create_msg_from_cap_control(cap_control):
    cap_control_msg = ControlCap()
    cap_control_msg.type_str = cap_control['type']
    cap_control_msg.name_str = cap_control['name']
    if 'options' in cap_control.keys():
      cap_control_msg.options_list = cap_control['options']
    else:
      cap_control_msg.options_list = []
    return cap_control_msg


def create_msgs_list_from_cap_controls(cap_controls):
  cap_control_msgs_list = []
  for cap_control_name in cap_controls.keys():
    cap_control = cap_controls[cap_control_name]
    cap_control_msg = create_msg_from_cap_control(cap_control)
    cap_control_msgs_list.append(cap_control_msg)
  return cap_control_msgs_list

def parse_control_msg(control_msg):
  control = dict()
  control['name'] = control_msg.name_str
  control['type'] = control_msg.type_str
  control['value'] = control_msg.value_str
  return control

def parse_control_msgs_list(controls_msg):
  controls = dict()
  for entry in controls_msg.controls_list:
    control = dict()
    control['name'] = entry.name_str
    control['type'] = entry.type_str
    control['value'] = entry.value_str
    controls[entry.name_str] = control
  return controls

def create_msg_from_control(control):
  control_msg = Control()
  control_msg.type_str = control['type']
  control_msg.name_str = control['name']
  control_msg.value_str = control['value']
  return control_msg

def create_msgs_list_from_controls(controls):
  controls_list = []
  for control_name in controls.keys():
    control = controls[control_name]
    control_msg = create_msg_from_control(control)
    controls_list.append(control_msg)
  return controls_list


def create_capabilities_response(cap_controls, has_cap_updates = False):
  response = ControlsCapabilitiesQueryResponse()
  response.controls_count = len(cap_controls)
  response.constrols_cap_list = create_msgs_list_from_cap_controls(cap_controls)
  response.has_cap_updates = has_cap_updates
  return response

def parse_capabilities_response(response):
  cap_controls = parse_cap_control_msgs_list(response.constrols_cap_list)
  has_cap_updates = response.has_cap_updates
  return(cap_controls,has_cap_updates )


def create_status_msg(controls,cap_controls, has_cap_updates = False):
  status_msg = ControlsStatus()
  if len(controls) == len(cap_controls):
    status_msg.controls_count = len(controls)
    status_msg.controls_list = create_msgs_list_from_controls(controls)
    status_msg.constrols_cap_list = create_msgs_list_from_cap_controls(cap_controls)
    status_msg.has_cap_updates = has_cap_updates
  return status_msg

def parse_status_msg(status_msg):
  controls = parse_control_msgs_list(status_msg.controls_list)
  cap_controls = parse_cap_control_msgs_list(status_msg.constrols_cap_list)
  has_cap_updates = status_msg.has_cap_updates
  return(controls, cap_controls,has_cap_updates )





def get_control_from_controls(control_name,controls):
  control = None
  if control_name in controls.keys():
    control = controls[control_name]
  return control


def get_data_from_control(control):

  s_name = 'Missing'
  s_type = 'Missing'
  s_value = 'Missing'
  data = None

  control_str = str(control)
  try:
    s_name = control['name']
    s_type = control['type']
    s_value = control['value']
  except Exception as e:
    logger.log_warn("Failed to check control: " + control_str + " with exception: " + str(e) )
    return s_name, s_type, data

  if len(control) == 3:
    if control['type'] != None and control['value'] != None:

      s_value = control['value']
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
        logger.log_info("Control conversion failed for control " + s_str + "with exception" + str(e) )
  return s_name, s_type, data


def compare_control_in_controls(control,controls):

  name_match = False
  type_match = False
  value_match = False

  control_str = str(control)
  try:
    s_name = control['name']
    s_type = control['type']
    s_value = control['value']
  except Exception as e:
    logger.log_warn("Failed to check control: " + control_str + " with exception: " + str(e) )
    return name_match, type_match, value_match
  for control_name in controls.keys():
    check_control = controls[control_name]
    if check_control['name'] == s_name:
      name_match = True
      if check_control['type'] == s_type:
        type_match = True
      if check_control['value'] == s_value:
        value_match = True
      return name_match, type_match, value_match
      break
  return name_match, type_match, value_match

def check_valid_control(control,cap_controls):
  valid= False
  control_str = str(control)
  try:
    s_name = control['name']
    s_type = control['type']
    s_value = control['value']
  except Exception as e:
    logger.log_warn("Failed to check control: " + control_str + " with exception: " + str(e) )
    return False
  if s_name in cap_controls.keys():
      cap_control = cap_controls[s_name]
      c_type = cap_control['type']
      if 'options' in cap_control.keys():
        c_options = cap_control['options'] 
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



def try_to_update_control(control_to_update,controls,cap_controls,update_controls_function):
  controls_str = str(control_to_update)
  s_name = control_to_update['name']
  updated_controls = copy.deepcopy(controls)
  success = False
  msg = ""
  if s_name != "None":
    if update_controls_function is not None:
      if check_valid_control(control_to_update,cap_controls):
        try:
          [success, msg] = update_controls_function(control_to_update)
          if success:
            msg = ("Updated control: " + controls_str + " : " + str(msg))
          else:
            msg = ("Failed to update control: " + controls_str + " : " + str(msg))
        except Exception as e:
          msg = ("Failed to update control: " + controls_str + " with exception: " + str(e) )
      else:
        msg = ("Failed to update control: " + controls_str + " Invalid control name or value with cap controls: " + str(cap_controls))
    else:
      msg = ("Failed to update control: " + controls_str + " No update function provided")
  else:
    success = True
  return success, msg


def get_controls_by_type(controls,type_str):
  controls_of_type = dict()
  for control_name in controls.keys():
    control = controls[control_name]
    s_name = control['name']
    s_type = control['type']
    if s_type == type_str :
        controls_of_type[s_name] = control
  return controls_of_type


  
