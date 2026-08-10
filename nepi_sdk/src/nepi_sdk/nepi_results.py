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

from nepi_interfaces.msg import Result, ResultStatus




from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_result"
logger = Logger(log_name = log_name)


#########################
### Result Helper Functions

# A result is read-only from the RUI's point of view: it carries a current value
# and the time that value was last written. There are no bounds, options,
# factory values or default values here -- those are control concepts and live
# in nepi_controls.

RESULT_TYPES = ["Bool", "Bools", "String", "Strings", "Int", "Ints","Float","Floats"]

BLANK_RESULT_DICT = nepi_sdk.convert_msg2dict(Result())

BLANK_RESULT_DICT = dict()

EXAMPLE_INIT_DICT = dict(


      exp_bool_result = {"type":"Bool", "value": True,
                   # OPTIONAL
                   'display_name':'Example Bool Result', 'description':'Example bool result', 'hidden':False},

      exp_bools_result = {"type":"Bools", "value":[True,False],
                   # OPTIONAL
                   'display_name':'Example Bools Result', 'description':'Example bools result', 'hidden':False},


      exp_string_result = {"type":"String", "value":'string1',
                   # OPTIONAL
                   'display_name':'Example String Result', 'description':'Example string result', 'hidden':False},

      exp_strings_result = {"type":"Strings", "value":['string1','string2'],
                   # OPTIONAL
                   'display_name':'Example Strings Result', 'description':'Example strings result', 'hidden':False},


      exp_int_result = {"type":"Int", "value":2,
                  # OPTIONAL
                  'display_name':'Example Int Result', 'description':'Example int result', 'hidden':False},

      exp_ints_result = {"type":"Ints", "value":[2,2],
                  # OPTIONAL
                  'display_name':'Example Ints Result', 'description':'Example ints result', 'hidden':False},


      exp_float_result = {"type":"Float", "value":2.0, 'round_value': 2,
                  # OPTIONAL
                  'display_name':'Example Float Result', 'description':'Example float result', 'hidden':False, 'round_display': 2,},

      exp_floats_result = {"type":"Floats", "value":[2.0,2.0], 'round_value': 2,
                  # OPTIONAL
                  'display_name':'Example Floats Result', 'description':'Example floats result', 'hidden':False, 'round_display': 2,},
    )


def get_result_publisher_namespaces(topics_list = None, types_list = None):
    topics_list = nepi_sdk.find_topics_by_msg('ResultStatus', topics_list = topics_list, types_list = types_list)
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic.replace('/status','')))
    return namespaces_list


def create_result_dict(init_dict):
  result_dict = dict()

  try:
    names = list(init_dict.keys())
  except:
    names = []

  for name in names:
    try:
      input_dict = init_dict[name]
      input_type = input_dict['type']
      if input_type in RESULT_TYPES:
        result_dict = copy.deepcopy(BLANK_RESULT_DICT)
        result_dict['name'] = name
        result_dict['round_value'] = -1
        result_dict['round_display'] = 2
        for key in result_dict.keys():
          if key in input_dict.keys():
            result_dict[key] = input_dict[key]

        # round_value applies to Float and Floats only. -1 means no rounding.
        round_value = -1
        try:
          round_value = int(result_dict['round_value'])
        except:
          round_value = -1

        if input_type == "Bool": ###########################################################
            value = False
            try:
                value  = (input_dict['value'] == True)
            except:
              pass
            result_dict['value_bool'] = value

        elif input_type == "Bools": ###########################################################
            values = []
            input_values = input_dict['value']
            for value in input_values:
              try:
                value  = (value == True)
              except:
                value = False
              values.append(value)
            result_dict['value_bools'] = values

        elif input_type == "String": ###########################################################
            value = ''
            try:
                value  = str(input_dict['value'])
            except:
              pass
            result_dict['value_string'] = value

        elif input_type == "Strings": ###########################################################
            values = []
            input_values = input_dict['value']
            for value in input_values:
              try:
                value  = str(value)
              except:
                value = ''
              values.append(value)
            result_dict['value_strings'] = values

        elif input_type == "Int": ###########################################################
            try:
              value  = int(input_dict['value'])
            except:
              value = 0
            result_dict['value_int'] = value

        elif input_type == "Ints": ###########################################################
            values = []
            input_values = input_dict['value']
            for value in input_values:
              try:
                value  = int(value)
              except:
                value = 0
              values.append(value)
            result_dict['value_ints'] = values

        elif input_type == "Float": ###########################################################
            try:
              value  = float(input_dict['value'])
              if round_value >= 0:
                value = round(value,round_value)
            except:
              value = 0.0
            result_dict['value_float'] = value

        elif input_type == "Floats": ###########################################################
            values = []
            input_values = input_dict['value']
            for value in input_values:
              try:
                value  = float(value)
                if round_value >= 0:
                  value = round(value,round_value)
              except:
                value = 0.0
              values.append(value)

            result_dict['value_floats'] = values

        result_dict['timestamp'] = nepi_utils.get_time()

        # ADD TO RESULT if try has not failed
        result_dict[name] = result_dict
    except:
      pass

  return result_dict

##################
# Result Functions

def get_result_value(result_dict, result_name):
  value = None
  if result_name in result_dict.keys():
      result_dict = result_dict[result_name]
      result_type = result_dict['type']
      if result_type == "Bool": ###########################################################
          value = result_dict['value_bool']

      elif result_type == "Bools": ###########################################################
          value = result_dict['value_bools']

      elif result_type == "String": ###########################################################
          value = result_dict['value_string']

      elif result_type == "Strings": ###########################################################
          value = result_dict['value_strings']

      elif result_type == "Int": ###########################################################
          value = result_dict['value_int']

      elif result_type == "Ints": ###########################################################
          value = result_dict['value_ints']

      elif result_type == "Float": ###########################################################
          value = result_dict['value_float']

      elif result_type == "Floats": ###########################################################
          value = result_dict['value_floats']
  return value


def set_result_value(result_dict, result_name, update_value, timestamp = None):
  if result_name in result_dict.keys():
      result_dict = result_dict[result_name]
      result_type = result_dict['type']
      if timestamp is None:
         timestamp = nepi_utils.get_time()

      round_value = -1
      try:
        round_value = int(result_dict['round_value'])
      except:
        round_value = -1

      if result_type == "Bool": ###########################################################
          try:
              value  = (update_value == True)
              result_dict['value_bool'] = value
              result_dict['timestamp'] = timestamp
          except Exception as e:
            logger.log_warn('Failed to update ' + str(result_name) + " to " + str(update_value) + " : " + str(e))

      elif result_type == "Bools": ###########################################################
          values = []
          valid = True
          try:
            for value in update_value:
              values.append(value == True)
          except Exception as e:
            valid = False
            logger.log_warn('Failed to update ' + str(result_name) + " to " + str(update_value) + " : " + str(e))
          if valid == True:
            result_dict['value_bools'] = values
            result_dict['timestamp'] = timestamp

      elif result_type == "String": ###########################################################
          try:
              value  = str(update_value)
              result_dict['value_string'] = value
              result_dict['timestamp'] = timestamp
          except Exception as e:
            logger.log_warn('Failed to update ' + str(result_name) + " to " + str(update_value) + " : " + str(e))

      elif result_type == "Strings": ###########################################################
          values = []
          valid = True
          try:
            for value in update_value:
              values.append(str(value))
          except Exception as e:
            valid = False
            logger.log_warn('Failed to update ' + str(result_name) + " to " + str(update_value) + " : " + str(e))
          if valid == True:
            result_dict['value_strings'] = values
            result_dict['timestamp'] = timestamp

      elif result_type == "Int": ###########################################################
          try:
              value  = int(update_value)
              result_dict['value_int'] = value
              result_dict['timestamp'] = timestamp
          except Exception as e:
            logger.log_warn('Failed to update ' + str(result_name) + " to " + str(update_value) + " : " + str(e))

      elif result_type == "Ints": ###########################################################
          values = []
          valid = True
          try:
            for value in update_value:
              values.append(int(value))
          except Exception as e:
            valid = False
            logger.log_warn('Failed to update ' + str(result_name) + " to " + str(update_value) + " : " + str(e))
          if valid == True:
            result_dict['value_ints'] = values
            result_dict['timestamp'] = timestamp

      elif result_type == "Float": ###########################################################
          try:
              value  = float(update_value)
              if round_value >= 0:
                value = round(value,round_value)
              result_dict['value_float'] = value
              result_dict['timestamp'] = timestamp
          except Exception as e:
            logger.log_warn('Failed to update ' + str(result_name) + " to " + str(update_value) + " : " + str(e))


      elif result_type == "Floats": ###########################################################
          values = []
          valid = True
          try:
            for value in update_value:
              value = float(value)
              if round_value >= 0:
                value = round(value,round_value)
              values.append(value)
          except Exception as e:
            valid = False
            logger.log_warn('Failed to update ' + str(result_name) + " to " + str(update_value) + " : " + str(e))
          if valid == True:
            result_dict['value_floats'] = values
            result_dict['timestamp'] = timestamp

      ###########################################################
      result_dict[result_name] = result_dict
  return result_dict


def get_result_timestamp(result_dict, result_name):
  timestamp = 0.0
  if result_name in result_dict.keys():
      timestamp = result_dict[result_name]['timestamp']
  return timestamp


##################
# Display Functions

def get_result_display_name(result_dict, result_name):
  display_name = ''
  if result_name in result_dict.keys():
      display_name = result_dict[result_name]['display_name']
  return display_name

def set_result_display_name(result_dict, result_name, display_name):
  display_name = str(display_name)
  if result_name in result_dict.keys():
      result_dict[result_name]['display_name'] = display_name
  return result_dict


def get_result_description(result_dict, result_name):
  description = ''
  if result_name in result_dict.keys():
      description = result_dict[result_name]['description']
  return description

def set_result_description(result_dict, result_name, description):
  description = str(description)
  if result_name in result_dict.keys():
      result_dict[result_name]['description'] = description
  return result_dict

def get_result_hidden(result_dict, result_name):
  hidden = False
  if result_name in result_dict.keys():
      hidden = (result_dict[result_name]['hidden'] == True)
  return hidden

def set_result_hidden(result_dict, result_name, hidden):
  hidden = (hidden == True)
  if result_name in result_dict.keys():
      result_dict[result_name]['hidden'] = hidden
  return result_dict

def get_result_display_order(result_dict, result_name):
  order = -1
  if result_name in result_dict.keys():
      ordered_list = list(result_dict.keys())
      order = ordered_list.index(result_name)
  return order

def set_result_display_order(result_dict, result_name, update_order = 0):
  update_result_dict = copy.deepcopy(result_dict)
  cur_ordered_list = list(result_dict.keys())
  num_result = len(cur_ordered_list)
  cur_order = -1
  if result_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(result_name)
    if cur_order != -1 and update_order >= 0 and update_order < num_result:
      update_ordered_list = list(result_dict.keys())
      update_ordered_list.remove(result_name)
      num_result = len(update_ordered_list)
      if update_order == num_result:
        update_ordered_list.append(result_name)
      else:
          update_ordered_list.insert(update_order, result_name)
      update_result_dict = {key: result_dict[key] for key in update_ordered_list}
  return update_result_dict



def move_result_display_top(result_dict, result_name):
  update_result_dict = copy.deepcopy(result_dict)
  cur_ordered_list = list(result_dict.keys())
  num_result = len(cur_ordered_list)
  if result_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(result_name)
    update_order = 0
    if cur_order != update_order and update_order >= 0 and update_order < num_result:
      update_result_dict = set_result_display_order(result_dict, result_name, update_order)
  return update_result_dict

def move_result_display_bottom(result_dict, result_name):
  update_result_dict = copy.deepcopy(result_dict)
  cur_ordered_list = list(result_dict.keys())
  num_result = len(cur_ordered_list)
  if result_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(result_name)
    update_order = num_result - 1
    if cur_order != update_order and update_order >= 0 and update_order < num_result:
      update_result_dict = set_result_display_order(result_dict, result_name, update_order)
  return update_result_dict

def move_result_display_up(result_dict, result_name):
  update_result_dict = copy.deepcopy(result_dict)
  cur_ordered_list = list(result_dict.keys())
  num_result = len(cur_ordered_list)
  if result_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(result_name)
    update_order = cur_order + 1
    if cur_order != -1 and update_order >= 0 and update_order < num_result:
      update_result_dict = set_result_display_order(result_dict, result_name, update_order)
  return update_result_dict

def move_result_display_down(result_dict, result_name):
  update_result_dict = copy.deepcopy(result_dict)
  cur_ordered_list = list(result_dict.keys())
  num_result = len(cur_ordered_list)
  if result_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(result_name)
    update_order = cur_order - 1
    if cur_order != -1 and update_order >= 0 and update_order < num_result:
      update_result_dict = set_result_display_order(result_dict, result_name, update_order)
  return update_result_dict

############################################################
# Status Msg Functions

def create_status_msg( name = '', display_name = '', description = '', show_result = True,  has_show_control = False):
  status_msg = ResultStatus()
  name = nepi_utils.get_clean_name(str(name))
  status_msg.name= name
  if display_name == '':
    display_name = name
  status_msg.display_name= str(display_name)
  if description == '':
    description = name
  status_msg.description= str(description)
  status_msg.show_result = show_result
  status_msg.has_show_control = has_show_control and show_result == True
  return status_msg


def update_status_msg( status_msg, result_dict, hidden = False):
  if status_msg is None:
    status_msg = ResultStatus()
  status_msg.hidden= hidden


  names_list = []
  types_list = []
  msgs_list = []
  hidden_list = []

  try:
    names = list(result_dict.keys())
  except:
    names = []
  for name in names:
    try:
      result_dict = result_dict[name]
      result_type = result_dict['type']
      if result_type in RESULT_TYPES:
        msg_type = 'nepi_interfaces/Result'
        result_msg = nepi_sdk.convert_dict2msg(msg_type,result_dict)
        # convert_dict2msg() swallows its own exception and returns None. A None
        # in result_msg_list would fail the whole status publish, so skip it.
        if result_msg is not None:
          names_list.append(name)
          types_list.append(result_type)
          msgs_list.append(result_msg)
          hidden_list.append(result_dict['hidden'] == True)
    except:
      pass

  status_msg.result_name_list = names_list
  status_msg.result_type_list = types_list
  status_msg.result_msg_list = msgs_list
  status_msg.result_hidden_list = hidden_list
  return status_msg
