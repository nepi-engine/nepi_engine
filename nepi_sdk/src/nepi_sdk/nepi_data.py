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

from nepi_interfaces.msg import Datum, DataStatus




from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_data"
logger = Logger(log_name = log_name)


#########################
### Data Helper Functions

# A datum is read-only from the RUI's point of view: it carries a current value
# and the time that value was last written. There are no bounds, options,
# factory values or default values here -- those are control concepts and live
# in nepi_controls.

DATUM_TYPES = ["Bool", "Bools", "String", "Strings", "Int", "Ints","Float","Floats"]

BLANK_DATUM_DICT = nepi_sdk.convert_msg2dict(Datum())

BLANK_DATA_DICT = dict()

EXAMPLE_INIT_DICT = dict(


      exp_bool_data = {"type":"Bool", "value": True,
                   # OPTIONAL
                   'display_name':'Example Bool Data', 'description':'Example bool data', 'hidden':False},

      exp_bools_data = {"type":"Bools", "value":[True,False],
                   # OPTIONAL
                   'display_name':'Example Bools Data', 'description':'Example bools data', 'hidden':False},


      exp_string_data = {"type":"String", "value":'string1',
                   # OPTIONAL
                   'display_name':'Example String Data', 'description':'Example string data', 'hidden':False},

      exp_strings_data = {"type":"Strings", "value":['string1','string2'],
                   # OPTIONAL
                   'display_name':'Example Strings Data', 'description':'Example strings data', 'hidden':False},


      exp_int_data = {"type":"Int", "value":2,
                  # OPTIONAL
                  'display_name':'Example Int Data', 'description':'Example int data', 'hidden':False},

      exp_ints_data = {"type":"Ints", "value":[2,2],
                  # OPTIONAL
                  'display_name':'Example Ints Data', 'description':'Example ints data', 'hidden':False},


      exp_float_data = {"type":"Float", "value":2.0, 'round_value': 2,
                  # OPTIONAL
                  'display_name':'Example Float Data', 'description':'Example float data', 'hidden':False, 'round_display': 2,},

      exp_floats_data = {"type":"Floats", "value":[2.0,2.0], 'round_value': 2,
                  # OPTIONAL
                  'display_name':'Example Floats Data', 'description':'Example floats data', 'hidden':False, 'round_display': 2,},
    )


def get_data_publisher_namespaces(topics_list = None, types_list = None):
    topics_list = nepi_sdk.find_topics_by_msg('DataStatus', topics_list = topics_list, types_list = types_list)
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic.replace('/status','')))
    return namespaces_list


def create_data_dict(init_dict):
  data_dict = dict()

  try:
    names = list(init_dict.keys())
  except:
    names = []

  for name in names:
    try:
      input_dict = init_dict[name]
      input_type = input_dict['type']
      if input_type in DATUM_TYPES:
        datum_dict = copy.deepcopy(BLANK_DATUM_DICT)
        datum_dict['name'] = name
        datum_dict['round_value'] = -1
        datum_dict['round_display'] = 2
        for key in datum_dict.keys():
          if key in input_dict.keys():
            datum_dict[key] = input_dict[key]

        # round_value applies to Float and Floats only. -1 means no rounding.
        round_value = -1
        try:
          round_value = int(datum_dict['round_value'])
        except:
          round_value = -1

        if input_type == "Bool": ###########################################################
            value = False
            try:
                value  = (input_dict['value'] == True)
            except:
              pass
            datum_dict['value_bool'] = value

        elif input_type == "Bools": ###########################################################
            values = []
            input_values = input_dict['value']
            for value in input_values:
              try:
                value  = (value == True)
              except:
                value = False
              values.append(value)
            datum_dict['value_bools'] = values

        elif input_type == "String": ###########################################################
            value = ''
            try:
                value  = str(input_dict['value'])
            except:
              pass
            datum_dict['value_string'] = value

        elif input_type == "Strings": ###########################################################
            values = []
            input_values = input_dict['value']
            for value in input_values:
              try:
                value  = str(value)
              except:
                value = ''
              values.append(value)
            datum_dict['value_strings'] = values

        elif input_type == "Int": ###########################################################
            try:
              value  = int(input_dict['value'])
            except:
              value = 0
            datum_dict['value_int'] = value

        elif input_type == "Ints": ###########################################################
            values = []
            input_values = input_dict['value']
            for value in input_values:
              try:
                value  = int(value)
              except:
                value = 0
              values.append(value)
            datum_dict['value_ints'] = values

        elif input_type == "Float": ###########################################################
            try:
              value  = float(input_dict['value'])
              if round_value >= 0:
                value = round(value,round_value)
            except:
              value = 0.0
            datum_dict['value_float'] = value

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

            datum_dict['value_floats'] = values

        datum_dict['timestamp'] = nepi_utils.get_time()

        # ADD TO DATA if try has not failed
        data_dict[name] = datum_dict
    except:
      pass

  return data_dict

##################
# Data Functions

def get_datum_value(data_dict, datum_name):
  value = None
  if datum_name in data_dict.keys():
      datum_dict = data_dict[datum_name]
      datum_type = datum_dict['type']
      if datum_type == "Bool": ###########################################################
          value = datum_dict['value_bool']

      elif datum_type == "Bools": ###########################################################
          value = datum_dict['value_bools']

      elif datum_type == "String": ###########################################################
          value = datum_dict['value_string']

      elif datum_type == "Strings": ###########################################################
          value = datum_dict['value_strings']

      elif datum_type == "Int": ###########################################################
          value = datum_dict['value_int']

      elif datum_type == "Ints": ###########################################################
          value = datum_dict['value_ints']

      elif datum_type == "Float": ###########################################################
          value = datum_dict['value_float']

      elif datum_type == "Floats": ###########################################################
          value = datum_dict['value_floats']
  return value


def get_data_values(data_dict):
  values_dict = dict()
  if data_dict is not None:
      for datum_name in data_dict.keys():
         values_dict[datum_name] = get_datum_value(data_dict)
  return values_dict

def set_datum_value(data_dict, datum_name, update_value, timestamp = None):
  if datum_name in data_dict.keys():
      datum_dict = data_dict[datum_name]
      datum_type = datum_dict['type']
      if timestamp is None:
         timestamp = nepi_utils.get_time()

      round_value = -1
      try:
        round_value = int(datum_dict['round_value'])
      except:
        round_value = -1

      if datum_type == "Bool": ###########################################################
          try:
              value  = (update_value == True)
              datum_dict['value_bool'] = value
              datum_dict['timestamp'] = timestamp
          except Exception as e:
            logger.log_warn('Failed to update ' + str(datum_name) + " to " + str(update_value) + " : " + str(e))

      elif datum_type == "Bools": ###########################################################
          values = []
          valid = True
          try:
            for value in update_value:
              values.append(value == True)
          except Exception as e:
            valid = False
            logger.log_warn('Failed to update ' + str(datum_name) + " to " + str(update_value) + " : " + str(e))
          if valid == True:
            datum_dict['value_bools'] = values
            datum_dict['timestamp'] = timestamp

      elif datum_type == "String": ###########################################################
          try:
              value  = str(update_value)
              datum_dict['value_string'] = value
              datum_dict['timestamp'] = timestamp
          except Exception as e:
            logger.log_warn('Failed to update ' + str(datum_name) + " to " + str(update_value) + " : " + str(e))

      elif datum_type == "Strings": ###########################################################
          values = []
          valid = True
          try:
            for value in update_value:
              values.append(str(value))
          except Exception as e:
            valid = False
            logger.log_warn('Failed to update ' + str(datum_name) + " to " + str(update_value) + " : " + str(e))
          if valid == True:
            datum_dict['value_strings'] = values
            datum_dict['timestamp'] = timestamp

      elif datum_type == "Int": ###########################################################
          try:
              value  = int(update_value)
              datum_dict['value_int'] = value
              datum_dict['timestamp'] = timestamp
          except Exception as e:
            logger.log_warn('Failed to update ' + str(datum_name) + " to " + str(update_value) + " : " + str(e))

      elif datum_type == "Ints": ###########################################################
          values = []
          valid = True
          try:
            for value in update_value:
              values.append(int(value))
          except Exception as e:
            valid = False
            logger.log_warn('Failed to update ' + str(datum_name) + " to " + str(update_value) + " : " + str(e))
          if valid == True:
            datum_dict['value_ints'] = values
            datum_dict['timestamp'] = timestamp

      elif datum_type == "Float": ###########################################################
          try:
              value  = float(update_value)
              if round_value >= 0:
                value = round(value,round_value)
              datum_dict['value_float'] = value
              datum_dict['timestamp'] = timestamp
          except Exception as e:
            logger.log_warn('Failed to update ' + str(datum_name) + " to " + str(update_value) + " : " + str(e))


      elif datum_type == "Floats": ###########################################################
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
            logger.log_warn('Failed to update ' + str(datum_name) + " to " + str(update_value) + " : " + str(e))
          if valid == True:
            datum_dict['value_floats'] = values
            datum_dict['timestamp'] = timestamp

      ###########################################################
      data_dict[datum_name] = datum_dict
  return data_dict


def set_data_values(data_dict, datum_name, update_value, timestamp = None):
    for datum_name in data_dict.keys():
      update_value = data_dict[datum_name]
      data_dict = set_datum_value(data_dict, datum_name, update_value, timestamp = None)
    return data_dict

def get_datum_timestamp(data_dict, datum_name):
  timestamp = 0.0
  if datum_name in data_dict.keys():
      timestamp = data_dict[datum_name]['timestamp']
  return timestamp


##################
# Display Functions

def get_datum_display_name(data_dict, datum_name):
  display_name = ''
  if datum_name in data_dict.keys():
      display_name = data_dict[datum_name]['display_name']
  return display_name

def set_datum_display_name(data_dict, datum_name, display_name):
  display_name = str(display_name)
  if datum_name in data_dict.keys():
      data_dict[datum_name]['display_name'] = display_name
  return data_dict


def get_datum_description(data_dict, datum_name):
  description = ''
  if datum_name in data_dict.keys():
      description = data_dict[datum_name]['description']
  return description

def set_datum_description(data_dict, datum_name, description):
  description = str(description)
  if datum_name in data_dict.keys():
      data_dict[datum_name]['description'] = description
  return data_dict

def get_datum_hidden(data_dict, datum_name):
  hidden = False
  if datum_name in data_dict.keys():
      hidden = (data_dict[datum_name]['hidden'] == True)
  return hidden

def set_datum_hidden(data_dict, datum_name, hidden):
  hidden = (hidden == True)
  if datum_name in data_dict.keys():
      data_dict[datum_name]['hidden'] = hidden
  return data_dict

def get_datum_display_order(data_dict, datum_name):
  order = -1
  if datum_name in data_dict.keys():
      ordered_list = list(data_dict.keys())
      order = ordered_list.index(datum_name)
  return order

def set_datum_display_order(data_dict, datum_name, update_order = 0):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  cur_order = -1
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    if cur_order != -1 and update_order >= 0 and update_order < num_data:
      update_ordered_list = list(data_dict.keys())
      update_ordered_list.remove(datum_name)
      num_data = len(update_ordered_list)
      if update_order == num_data:
        update_ordered_list.append(datum_name)
      else:
          update_ordered_list.insert(update_order, datum_name)
      update_data_dict = {key: data_dict[key] for key in update_ordered_list}
  return update_data_dict



def move_datum_display_top(data_dict, datum_name):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    update_order = 0
    if cur_order != update_order and update_order >= 0 and update_order < num_data:
      update_data_dict = set_datum_display_order(data_dict, datum_name, update_order)
  return update_data_dict

def move_datum_display_bottom(data_dict, datum_name):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    update_order = num_data - 1
    if cur_order != update_order and update_order >= 0 and update_order < num_data:
      update_data_dict = set_datum_display_order(data_dict, datum_name, update_order)
  return update_data_dict

def move_datum_display_up(data_dict, datum_name):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    update_order = cur_order + 1
    if cur_order != -1 and update_order >= 0 and update_order < num_data:
      update_data_dict = set_datum_display_order(data_dict, datum_name, update_order)
  return update_data_dict

def move_datum_display_down(data_dict, datum_name):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    update_order = cur_order - 1
    if cur_order != -1 and update_order >= 0 and update_order < num_data:
      update_data_dict = set_datum_display_order(data_dict, datum_name, update_order)
  return update_data_dict

############################################################
# Status Msg Functions

def create_status_msg( name = '', display_name = '', description = '', show_data = True,  has_show_control = False):
  status_msg = DataStatus()
  name = nepi_utils.get_clean_name(str(name))
  status_msg.name= name
  if display_name == '':
    display_name = name
  status_msg.display_name= str(display_name)
  if description == '':
    description = name
  status_msg.description= str(description)
  status_msg.show_data = show_data
  status_msg.has_show_control = has_show_control and show_data == True
  return status_msg


def update_status_msg( status_msg, data_dict, hidden = False):
  if status_msg is None:
    status_msg = DataStatus()
  status_msg.hidden= hidden


  names_list = []
  types_list = []
  msgs_list = []
  hidden_list = []

  try:
    names = list(data_dict.keys())
  except:
    names = []
  for name in names:
    try:
      datum_dict = data_dict[name]
      datum_type = datum_dict['type']
      if datum_type in DATUM_TYPES:
        msg_type = 'nepi_interfaces/Datum'
        datum_msg = nepi_sdk.convert_dict2msg(msg_type,datum_dict)
        # convert_dict2msg() swallows its own exception and returns None. A None
        # in data_msg_list would fail the whole status publish, so skip it.
        if datum_msg is not None:
          names_list.append(name)
          types_list.append(datum_type)
          msgs_list.append(datum_msg)
          hidden_list.append(datum_dict['hidden'] == True)
    except:
      pass

  status_msg.data_name_list = names_list
  status_msg.data_type_list = types_list
  status_msg.data_msg_list = msgs_list
  status_msg.data_hidden_list = hidden_list
  return status_msg
