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

# A datum is read-only from the RUI's point of view: 

DATUM_TYPES = ["Trigger","Bool", "Bools", "String", "Strings",
               "Int", "Ints",
               "Float","Floats",
               "ColorRGB"]

LIST_TYPES = ["Bools", "Strings",
              "Ints", "Floats", "ColorRGB"]

LABELS_TYPES = ["Bools", "Strings",
               "Ints","Floats", "ColorRGB"]


STRING_TYPES = ["String", "Strings"]
BOOL_TYPES = ["Bool", "Bools"]
INT_TYPES = [ "Int","IntDouble","IntTriple", "Ints","ColorRGB"]
FLOAT_TYPES = ["Float","FloatDouble","FloatTriple","Floats"]
TRIGGER_TYPES = ['Trigger']



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


      exp_float_double_data = {"type":"FloatDouble", "value":[2.0,2.0],
                  # OPTIONAL
                  'display_name':'Example Float Double', 'labels': ['Width (Deg)', 'Height (Deg)'],
                 'description':'Example float double data', 'hidden':False, 'round_display': 2,},

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
  except Exception as e:
    # Dropped every datum at once, with no log line. Not throttled: this runs
    # once per data value at registration, so a throttle would hide it.
    logger.log_warn("create_data_dict: could not read datum names from init dict: " +
                    type(e).__name__ + ": " + str(e))
    names = []

  for name in names:
    try:
      init_datum_dict = init_dict[name]
      input_type = init_datum_dict['type']
      if input_type in DATUM_TYPES:
        datum_dict = copy.deepcopy(BLANK_DATUM_DICT)
        datum_dict['type'] = input_type
        datum_dict['type'] = input_type
        datum_dict['display_name'] = name
        datum_dict['description'] = name
        datum_dict['round_display'] = 2

        for key in datum_dict.keys():
          if key in init_datum_dict.keys():
            datum_dict[key] = init_datum_dict[key]

        #############
        # Clean Name
        datum_dict['name'] = name


        #############
        # Clean Rounds
        #############
        if datum_dict['round_value'] < 0 or datum_dict['round_value'] > 6:
          datum_dict['round_value'] = 6
        if datum_dict['round_display'] < 0 or datum_dict['round_display'] > 6:
          datum_dict['round_display'] = 6



        #############
        # Clean Bounds
        #############
        # Clean Bounds
        min_bound = -999
        max_bound = -999

        if input_type == 'ColorRGB':
              min_bound = 0
              max_bound = 255
        elif input_type in FLOAT_TYPES:
          try:
            min_bound = float(datum_dict['min_bound'])
          except:
            pass
          try:
            max_bound = float(datum_dict['max_bound'])
          except:
            pass
          try:
            min_bound = float(init_datum_dict['bounds'][0])
            max_bound = float(init_datum_dict['bounds'][1])
          except:
            pass
          try:
            min_bound = float(init_datum_dict['options'][0])
            max_bound = float(init_datum_dict['options'][1])
          except:
            pass
        elif input_type in INT_TYPES:
          try:
            min_bound = int(float(datum_dict['min_bound']))
          except:
            pass
          try:
            max_bound = int(float(datum_dict['max_bound']))
          except:
            pass
          try:
            min_bound = int(float(init_datum_dict['bounds'][0]))
            max_bound = int(float(init_datum_dict['bounds'][1]))
          except:
            pass
          try:
            min_bound = int(float(init_datum_dict['options'][0]))
            max_bound = int(float(init_datum_dict['options'][1]))
          except:
            pass

        datum_dict['min_bound'] = min_bound
        datum_dict['max_bound'] = max_bound



        #############
        # Clean Labels
        labels = [str(item) for item in datum_dict['labels']]
        datum_dict['labels'] = labels


        if input_type ==  'Bools' or input_type ==  'IntDouble' or input_type == 'IntTriple' or input_type == 'Ints' or \
            input_type == 'FloatDouble' or input_type == 'FloatTriple' or input_type == 'Floats':


            options = init_datum_dict.get('options',[])
            if len(labels) == 0 and len(options) > 0:
              labels = options

            if input_type !=  'Bools':
                   
              for i, entry in enumerate(value):
                if len(labels) <= i:
                  labels.append('value_' + str(i))
              datum_dict['labels'] = labels

            datum_dict['labels'] = labels 

        elif input_type == 'ColorRGB':
              datum_dict['labels'] = ['R','G','B']




        #############
        # Clean Value

        if input_type == 'Trigger':
          value = 0
        else:

          value  = datum_dict['value']

          check_dict = dict()
          check_dict[name] = datum_dict

          check_value = copy.deepcopy(value)
          value = get_clean_value(check_dict, name, value)
          default = value
          #logger.log_warn("Got clean value from check value: " + str(name) + ": " + str(value) + ": " + str(check_value))
          if value is None:
            continue
          

        datum_dict['value'] = value


        #####e() - value



        #############
        # Add to dict
        data_dict[name] = datum_dict
    except Exception as e:
      # A failing datum is still skipped and the loop still continues, exactly
      # as before -- the only change is that the failure is now audible. This
      # bare except:pass is why every other defect in this file went unnoticed:
      # a datum that raised here vanished from the dict with no error, no log
      # line, and no absence anyone could see except in the RUI.
      #
      # Not throttled. A data value registers all of its data in one pass,
      # so a throttle window would report the first failure and swallow the rest
      # -- which is the behavior being fixed.
      declared_type = '<unreadable>'
      try:
        declared_type = str(init_dict[name]['type'])
      except Exception:
        pass
      logger.log_warn("create_data_dict: dropped datum '" + str(name) +
                      "' of declared type '" + declared_type + "': " +
                      type(e).__name__ + ": " + str(e))
    
  return data_dict

##################
# Data Functions

def get_clean_value(data_dict, datum_name, datum_value = None):
  valid = False
  value = None
  if datum_name in data_dict.keys():
      datum_dict = data_dict[datum_name]
      datum_type = datum_dict['type']

      if datum_type == 'Discrete':
        datum_type = 'Selection'

      if datum_value is None:
        try:
          datum_value = datum_dict['value']
        except:
          pass

      if datum_type in LIST_TYPES:
        if isinstance(datum_value, list):
            pass
        else:
            datum_value = [datum_value]
     
      else:
        if isinstance(datum_value, list):
            try:
              datum_value = datum_value[0]
            except:
              pass
        else:
            pass
      # if datum_value is None or None in datum_value:
      #   return value

      
      if datum_type == "Menu": ###########################################################
        try:
          value  = int(float(datum_value))
        except Exception as e:
          pass
    

      elif datum_type == "Trigger": ###########################################################
          value = 0
          try: 
            value = float(datum_value)
          except:
            pass

      elif datum_type == "Bool": ###########################################################
          try:
              value  = (datum_value == True or datum_value == 'True' or datum_value == 'true')
          except Exception as e:
            pass

   
      elif datum_type == "Bools": ###########################################################
        labels = datum_dict['labels']
        try:
          values = []
          for item in [str(item) for item in datum_value]:
            if item in labels:
              values.append(item)
          # An empty list is a legitimate value here (nothing selected), which is
          # why this assigns unconditionally rather than guarding on len().
          value = values

        except Exception as e:
          pass

          
      elif datum_type == "String": ###########################################################
        value = str(datum_value)



      elif datum_type == "Int" :  ###########################################################
        try:
          value = int(float(datum_value))
          if int(float(datum_dict['min_bound'])) != -999 and value < datum_dict['min_bound']:
            value = datum_dict['min_bound']
          if int(float(datum_dict['max_bound'])) != -999 and value > datum_dict['max_bound']:
            value = datum_dict['max_bound']
        except Exception as e:
          pass

      elif datum_type == 'IntDouble' or datum_type == 'IntTriple' or datum_type == "Ints": ###########################################################

        valid = True
        if datum_type == 'IntDouble' and len(datum_value) != 2:
          valid = False
        if datum_type == 'IntTriple' and len(datum_value) != 3:
          valid = False

        if valid == True:
          try:  
              values = [0] * len(datum_value)
              for i, item in enumerate(datum_value):  
                values[i] = int(values[i])
              value = values
          except Exception as e:
            pass




      elif datum_type == "Float": ###########################################################


        try:
          value  = float(datum_value)
          round_value = data_dict['round_value']
          if round_value >= 0:
            value = round(value,round_value)
          # Reset valid = True here, discarding the low handle's verdict.
          if float(data_dict['min_bound']) != -999 and value < data_dict['min_bound']:
            value = data_dict['max_bound']
          if float(data_dict['max_bound']) != -999 and value > data_dict['max_bound']:
            value = data_dict['max_bound']
        except Exception as e:
          pass





      elif datum_type == 'FloatDouble' or datum_type == 'FloatTriple' or datum_type == "Floats": ###########################################################
        round_value = data_dict['round_value']
        valid = True
        if datum_type == 'FloatDouble' and len(datum_value) != 2:
          valid = False
        if datum_type == 'FloatTriple' and len(datum_value) != 3:
          valid = False

        if valid == True:

          try:  
              values = [0] * len(datum_value)
              for i, item in enumerate(datum_value):  
                values[i] = float(values[i])
                if round_value >= 0:
                  values[i] = round(values[i],round_value)
              value = values
          except Exception as e:
            pass


      elif datum_type == "ColorRGB": ###########################################################      
        try:
          value = [255,255,255]
          datum_value = list(datum_value)
          for i, val in enumerate(datum_value):
            try:
              val = int(val)
              if 0 <= val <= 255:
                value[i] = val
            except:
              pass
        except Exception as e:
          pass



  return value


def get_value(data_dict, datum_name, index = None):
  value = None
  datum_type = None
  if datum_name in data_dict.keys():
      try:
        datum_type = data_dict[datum_name]['type']
        datum_value = get_clean_value(data_dict, datum_name)
        if index is None:
          value = datum_value
        else:
          try:
            index = int(index)
            if index > 0:
              if isinstance(datum_value, list):
                if len(datum_value) > index:
                  value = datum_value[index]
          except:
            value = None
      except:
        value = None

  ###################
  # Special Types Support
  if value is not None and datum_type == 'ColorRGB':
    try:
      value = tuple(value)
    except:
      value = None

  if datum_type == 'Trigger':
    if value <= 0:
      value = -999
    else:
      value = nepi_utils.get_time() - value

  return value

def get_values_dict(data_dict):
  data_values_dict = dict()
  for datum_name in data_dict.keys():
     datum_value = get_value(data_dict, datum_name)
     data_values_dict[datum_name] = datum_value
  return data_values_dict


def set_value(data_dict, datum_name, update_value, index = None, check_valid = True):
  if datum_name in data_dict.keys():
      
      if index is not None:
        try:
          index = int(index)
          if index > 0:
            datum_value = get_value(data_dict,datum_name)
            if isinstance(datum_value, list):
              if len(datum_value) > index:
                datum_value[index] = update_value
                update_value = datum_value
        except:
          pass

      if check_valid == False:
        update_value = get_clean_value(data_dict, datum_name, update_value)
      if update_value is not None:
        data_dict[datum_name]['value'] = update_value
  return data_dict

def sets_values(data_dict, data_values_dict):
  data_values_dict = dict()
  for datum_name in data_values_dict.keys():
     datum_value = data_values_dict[datum_name]
     data_dict = set_value(data_dict, datum_name, datum_value)
  return data_dict


def get_labels(data_dict, datum_name):
  labels = []
  if datum_name in data_dict.keys():
      labels = data_dict[datum_name].get('labels',[])
  return labels


def set_labels(data_dict, datum_name, labels):
  labels = [str(item) for item in labels]
  if datum_name in data_dict.keys():
      data_dict[datum_name]['labels'] = labels
  return data_dict



def get_bounds(data_dict, datum_name):
  bounds = [-999,-999]
  if datum_name in data_dict.keys():
      min_bound = data_dict[datum_name]['min_bound']
      max_bound = data_dict[datum_name]['max_bound']
  return [min_bound, max_bound]



def set_min_bound(data_dict, datum_name, min_bound = None):
  if min_bound is None:
    min_bound = -999
  if datum_name in data_dict.keys():
      input_type = data_dict[datum_name]['type']
      max_bound = data_dict[datum_name]['max_bound']
      if input_type in FLOAT_TYPES:
        try:
          min_bound = float(min_bound)
        except:
          pass
     
      elif input_type in INT_TYPES:
        try:
          min_bound = int(float(min_bound))
        except:
          pass
      if int(float(min_bound)) == -999 or int(float(max_bound)) == -999 or min_bound <  max_bound:
        data_dict[datum_name]['min_bound'] = min_bound
  return data_dict

def clear_min_bound(data_dict, datum_name):
  data_dict = set_min_bound(data_dict, datum_name)
  return data_dict

def set_max_bound(data_dict, datum_name, max_bound = None):
  if max_bound is None:
    max_bound = -999
  if datum_name in data_dict.keys():
      input_type = data_dict[datum_name]['type']
      min_bound = data_dict[datum_name]['min_bound']
      if input_type in FLOAT_TYPES:
        try:
          max_bound = float(max_bound)
        except:
          pass
     
      elif input_type in INT_TYPES:
        try:
          max_bound = int(max_bound)
        except:
          pass
      if int(float(min_bound)) == -999 or int(float(max_bound)) == -999 or min_bound <  max_bound:
        data_dict[datum_name]['max_bound'] = max_bound
  return data_dict

def clear_max_bound(data_dict, datum_name):
  data_dict = set_max_bound(data_dict, datum_name)
  return data_dict

def set_bounds(data_dict, datum_name, bounds = [-999,-999]):
  if len(bounds) == 2:
    [min_bound,max_bound] = bounds
    if min_bound is None:
      min_bound = -999
    if max_bound is None:
      max_bound = -999
    try:
      if int(float(min_bound)) == -999 or int(float(max_bound)) == -999 or min_bound <  max_bound:

        if datum_name in data_dict.keys():
            data_dict = set_min_bound(data_dict, datum_name, min_bound)
            data_dict = set_max_bound(data_dict, datum_name, max_bound)
    except:
      pass

  return data_dict




##################
# Display Functions

def get_display_name(data_dict, datum_name):
  display_name = ''
  if datum_name in data_dict.keys():
      display_name = data_dict[datum_name]['display_name']
  return display_name

def set_display_name(data_dict, datum_name, display_name):
  display_name = str(display_name)
  if datum_name in data_dict.keys():
      data_dict[datum_name]['display_name'] = display_name
  return data_dict


def get_description(data_dict, datum_name):
  description = ''
  if datum_name in data_dict.keys():
      description = data_dict[datum_name]['description']
  return description

def set_description(data_dict, datum_name, description):
  description = str(description)
  if datum_name in data_dict.keys():
      data_dict[datum_name]['description'] = description
  return data_dict

def get_hidden(data_dict, datum_name):
  hidden = False
  if datum_name in data_dict.keys():
      hidden = (data_dict[datum_name]['hidden'] == True)
  return hidden

def set_hidden(data_dict, datum_name, hidden):
  # str() here wrote the strings 'True'/'False' into Datum.hidden, a toggle
  # field. convert_dict2msg then rejected the dict and the datum vanished
  # from the status message instead of being hidden in it.
  hidden = (hidden == True)
  if datum_name in data_dict.keys():
      data_dict[datum_name]['hidden'] = hidden
  return data_dict

def get_display_order(data_dict, datum_name):
  order = -1
  if datum_name in data_dict.keys():
      ordered_list = list(data_dict.keys())
      order = ordered_list.index(datum_name)
  return order

def set_display_order(data_dict, datum_name, update_order = 0):
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



def move_datum_top(data_dict, datum_name):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    update_order = 0
    if cur_order != update_order and update_order >= 0 and update_order < num_data:
      update_data_dict = set_display_order(data_dict, datum_name, update_order)
  return update_data_dict

def move_datum_bottom(data_dict, datum_name):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    update_order = num_data - 1
    if cur_order != update_order and update_order >= 0 and update_order < num_data:
      update_data_dict = set_display_order(data_dict, datum_name, update_order)
  return update_data_dict

def move_datum_up(data_dict, datum_name):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    update_order = cur_order + 1
    if cur_order != -1 and update_order >= 0 and update_order < num_data:
      update_data_dict = set_display_order(data_dict, datum_name, update_order)
  return update_data_dict

def move_datum_down(data_dict, datum_name):
  update_data_dict = copy.deepcopy(data_dict)
  cur_ordered_list = list(data_dict.keys())
  num_data = len(cur_ordered_list)
  if datum_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(datum_name)
    update_order = cur_order - 1
    if cur_order != -1 and update_order >= 0 and update_order < num_data:
      update_data_dict = set_display_order(data_dict, datum_name, update_order)
  return update_data_dict

############################################################
# Status Msg Functions

def create_status_msg( name = '', display_name = '', description = '', show_data = True, has_show_datum = False):
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
  status_msg.has_show_datum = has_show_datum and show_data == True
  return status_msg


def update_status_msg( status_msg, data_dict):
  if status_msg is None:
    status_msg = DataStatus()


  names_list = [] 
  types_list = [] 
  msgs_list = [] 

  try:
    names = list(data_dict.keys())
  except Exception as e:
    logger.log_warn("update_status_msg: could not read datum names from data dict: " +
                    type(e).__name__ + ": " + str(e))
    names = []
  for name in names:
    try:
      datum_dict = data_dict[name]
      datum_type = datum_dict['type']
      if datum_type in DATUM_TYPES:

        # Convert default and value to string lists for Data Msg
        value = datum_dict['value']

        if datum_type == 'Trigger':
          if value <= 0:
            value = -999
          else:
            value = nepi_utils.get_time() - value
 
        if datum_type in LIST_TYPES:
          if isinstance(value, list):
              msg_value = [str(item) for item in value]
          else:
              msg_value = [str(value)]
        else:
          msg_value = [str(value)]
        datum_dict['value'] = msg_value

        msg_dict = nepi_sdk.convert_msg2dict(Datum())
        for key in msg_dict.keys():
          if key in datum_dict.keys():
            msg_dict[key] = datum_dict[key]

        msg_type = 'nepi_interfaces/Datum'
        datum_msg = nepi_sdk.convert_dict2msg(msg_type,msg_dict)
        if datum_msg is not None:
          names_list.append(name)
          types_list.append(datum_type)
          msgs_list.append(datum_msg)
      else:
        # Same silent fall-through as create_data_dict: a datum that made
        # it into the dict but carries a type this list does not know is simply
        # left out of the status message, so the RUI never sees it.
        logger.log_warn("update_status_msg: left datum '" + str(name) +
                        "' of declared type '" + str(datum_type) +
                        "' out of the status message: type is not one of " + str(DATUM_TYPES),
                        throttle_s = 5)
    except Exception as e:
      # Dropped the datum from the published status with no log. Throttled,
      # unlike create_data_dict: this runs on every status publish, not once
      # at registration.
      logger.log_warn("update_status_msg: left datum '" + str(name) +
                      "' out of the status message: " +
                      type(e).__name__ + ": " + str(e), throttle_s = 5)
    status_msg.data_name_list = names_list
    status_msg.data_msg_list = msgs_list
  return status_msg
