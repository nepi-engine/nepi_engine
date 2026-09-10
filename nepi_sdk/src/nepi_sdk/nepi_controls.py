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

from std_msgs.msg import Empty, Int8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import Control, ControlsStatus, UpdateControl




from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_controls"
logger = Logger(log_name = log_name)


#########################
### Controls Helper Functions





CONTROL_TYPES = ["Menu","Button", "Buttons", "Toggle", "Toggles", 
                 "String", "Selection","Selections",
                 "Int","Ints","IntSlider",
                 "Float","Floats","FloatSlider",
                 "RangeSlider", "ColorRGB"]

OPTION_TYPES =  ["Menu","Selection","Selections"]

SINGLE_TYPES = ["Menu","Button","Toggle", 
                 "String","Selection",
                 "Int","IntSlider",
                 "Float","FloatSlider"]

DOUBLE_TYPES = ["RangeSlider"]

TRIPLE_TYPES = ["ColorRGB"]

LIST_TYPES = ["Buttons", "Toggles", 
                "Selection","Selections",
                "Ints", "Floats",
                "RangeSlider", "ColorRGB"]


BOUND_TYPES = ["Int","Ints","IntSlider",
                 "Float","Floats","FloatSlider",
                 "RangeSlider", "ColorRGB"]

STRING_TYPES = ["String","Selection","Selections"]
BOOL_TYPES = ["Toggle","Toggles"]
INT_TYPES = ["Menu","Int","Ints","IntSlider","ColorRGB"]
FLOAT_TYPES = ["Float","Floats","FloatSlider","RangeSlider"]
TRIGGER_TYPES = ['Button','Buttons']

BLANK_CONTROL_DICT = nepi_sdk.convert_msg2dict(Control())

BLANK_CNTROLS_DICT = dict()

EXAMPLE_INIT_DICT = dict(
      pub_rate = {"type":"Float", "default":2, 
                  # OPTIONAL
                  "min_bound": 0.1, "max_bound":15, 'value_round': 2,
                  'display_name':'Pub Rate', 'description':'Value pub rate', 'display_hidden':False, 'display_round': 2,}, 
      wh_degrees = {"type":"FloatDouble", "default":[100,70], 
                  # OPTIONAL
                  "min_bound":10, "max_bound":200, 'value_round': 2, 'display_labels': ['Width (Deg)', 'Height (Deg)'],
                  'display_name':'Pub Rate', 'description':'Value pub rate', 'display_hidden':False, 'disabled':True, 'display_round': 2,}, 

      index = {"type":"Int", "default":3,  
               # OPTIONAL
               "min_bound": 3, "max_bound": 10, 'value_round': 2,
               'display_name':'Select Index', 'description':'Value index', 'display_hidden':False}, 

      topic_sel = {"type":"Selection", "default":'Topic1', "options":['Topic1', 'Topic2'], 
                   # OPTIONAL
                   'display_name':'Select Topic', 'description':'Value selected topic', 'display_hidden':False}, 

      topics_sel = {"type":"Selection", "default":['Topic1', 'Topic2'], "options":['Topic1', 'Topic2'], 
                    # OPTIONAL
                    'display_name':'Select Topics', 'description':'Value selected topics', 'display_hidden':False}, 

      event_button = {"type":"Button", 
                       # OPTIONAL
                       'display_name':'Event Button', 'description':'Event button', 'display_hidden':False}
    )


def get_publisher_namespaces(topics_list = None, types_list = None):
    topics_list = nepi_sdk.find_topics_by_msg('ControlsStatus', topics_list = topics_list, types_list = types_list)
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic.replace('/status','')))
    return namespaces_list


def create_controls_dict(init_dict):
  controls_dict = dict()

  try:
    names = list(init_dict.keys())
  except Exception as e:
    # Dropped every control at once, with no log line. Not throttled: this runs
    # once per controls value at registration, so a throttle would hide it.
    logger.log_warn("create_controls_dict: could not read control names from init dict: " +
                    type(e).__name__ + ": " + str(e))
    names = []

  for i, name in enumerate(names):
    try:
      init_control_dict = init_dict[name]
      input_type = init_control_dict['type']
      if input_type == 'Discrete':
        input_type = 'Selection'
      if input_type in CONTROL_TYPES:
        control_dict = copy.deepcopy(BLANK_CONTROL_DICT)
        control_dict['type'] = input_type
        control_dict['description'] = name
        control_dict['param'] = True
        control_dict['round'] = 6
        control_dict['default'] = []
        control_dict['length'] = 0
        control_dict['min_bound'] = -999
        control_dict['max_bound'] = -999
        control_dict['display_name'] = name
        control_dict['display_round'] = 2
        for key in control_dict.keys():
          if key in init_control_dict.keys():
            control_dict[key] = init_control_dict[key]

        #############
        # Clean Name
        control_dict['name'] = nepi_utils.get_clean_name(name)
        if  control_dict['name'] == '':
           control_dict['name'] = 'control' + str(i)


        #############
        # Clean Name
        control_type = control_dict['name']
        if control_type == 'Discrete':
          control_type = 'Selection'
        control_type = control_type.replace('Trigger','Button')
        control_dict['name'] = control_type


        #############
        # Clean Rounds
        #############
        if control_dict['round'] < 0 or control_dict['round'] > 6:
          control_dict['round'] = 6
        if control_dict['display_round'] < 0:
          control_dict['display_round'] = 0
        if control_dict['display_round'] > 6:
          control_dict['display_round'] = 6

        #############
        # Clean Bounds
        #############
        # Clean Bounds
        min_bound = -999
        max_bound = -999


        if input_type == 'BOUND_TYPES':
          if input_type == 'ColorRGB':
                min_bound = 0
                max_bound = 255
          elif input_type in FLOAT_TYPES:
            try:
              min_bound = float(control_dict['min_bound'])
            except:
              pass
            try:
              max_bound = float(control_dict['max_bound'])
            except:
              pass
            try:
              min_bound = float(init_control_dict['bounds'][0])
              max_bound = float(init_control_dict['bounds'][1])
            except:
              pass
          elif input_type in INT_TYPES:
            try:
              min_bound = int(float(control_dict['min_bound']))
            except:
              pass
            try:
              max_bound = int(float(control_dict['max_bound']))
            except:
              pass
            try:
              min_bound = int(float(init_control_dict['bounds'][0]))
              max_bound = int(float(init_control_dict['bounds'][1]))
            except:
              pass

        control_dict['min_bound'] = min_bound
        control_dict['max_bound'] = max_bound

        #############
        # Clean Value
        value = None
        if input_type in "TRIGGER_TYPES":
          value = [0]
        else:
          value  = control_dict['default']
          if value == []:
            value = control_dict['value']

          if value is not None:
            if isinstance(value, list) == False:
                values = [str(value)]
            else:
              values = [str(item) for item in value]
            value = values
        if value is None or isinstance(value, list) == False:
          continue          

        control_dict['value'] = value
        control_dict['length'] = len(value)



        #############
        # Clean Display Name
        #############
        if control_dict['display_name'] is None or control_dict['display_name'] == 'None':
          control_dict['display_name'] = ''
        if control_type in LIST_TYPES and control_dict['display_name'] == '':
          if isinstance(value, list):
              pass
          else:
              control_dict['display_name'] = name


        #############
        # Clean Options and Labels
        options = control_dict['options']
        if isinstance(options, list) == False:
          options = []
        display_labels = control_dict['display_labels']
        if isinstance(display_labels, list) == False:
          display_labels = []

        if input_type == 'OPTION_TYPES':
            if len(options) == 0 and len(display_labels) > 0:
              options = display_labels
            options = [str(item) for item in control_dict['options']]
            display_labels = [] 
        else:
            if len(display_labels) == 0 and len(options) > 0:
              display_labels = options
            options = [] 

            if input_type == 'ColorRGB':
                  display_labels = ['R','G','B']

            elif input_type == 'RangeSlider':
                  default_labels = ['start','stop']
                  for i, label in enumerate(display_labels):
                    default_labels[i] = label
                  display_labels = default_labels

            elif input_type in SINGLE_TYPES:
                  display_labels = [control_dict['display_name']]
            else:
              for i, entry in enumerate(value):
                if len(display_labels) <= i:
                  display_labels.append('control_' + str(i))

            display_labels = [str(item) for item in display_labels]
            options = []

        control_dict['options'] = options
        control_dict['display_labels'] = display_labels


        #############
        # Check Valid Value
        #############

        check_dict = dict()
        check_dict[name] = copy.deepcopy(control_dict)

        check_value = copy.deepcopy(value)
        value = get_clean_value(check_dict, name, check_value)
        #logger.log_warn("Got clean value from check value: " + str(name) + ": " + str(value) + ": " + str(check_value))
        if value is None:
          continue
  
        control_dict['default'] = value
        control_dict['length'] = len(value)
        


        #############
        # Add to dict
        controls_dict[name] = control_dict
    except Exception as e:
      # A failing control is still skipped and the loop still continues, exactly
      # as before -- the only change is that the failure is now audible. This
      # bare except:pass is why every other defect in this file went unnoticed:
      # a control that raised here vanished from the dict with no error, no log
      # line, and no absence anyone could see except in the RUI.
      #
      # Not throttled. A controls value registers all of its controls in one pass,
      # so a throttle window would report the first failure and swallow the rest
      # -- which is the behavior being fixed.
      declared_type = '<unreadable>'
      try:
        declared_type = str(init_dict[name]['type'])
      except Exception:
        pass
      logger.log_warn("create_controls_dict: dropped control '" + str(name) +
                      "' of declared type '" + declared_type + "': " +
                      type(e).__name__ + ": " + str(e))
    
  return controls_dict

##################
# Controls Functions

def get_clean_value(controls_dict, control_name, control_value = None):
  # If control_name not in controls_dict keys, None is returned
  # If control_value is None or any control_value is invalid, current valid values are returned
  valid = False
  value = None
  control_name = nepi_utils.get_clean_name(control_name)
  if control_name != '' and control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      current_value = control_dict['value']
      control_type = control_dict['type']
      control_length = control_dict['length']
      options = control_dict['options']
      min_bound = control_dict['min_bound']
      max_bound = control_dict['max_bound']

      if control_value is None:
        try:
          control_value = copy.deepcopy(current_value)
        except:
          pass
      if control_value is None:
        return value

      control_value = [str(item) for item in control_value]

      if len(control_value) != control_length:
        return value



      if control_type in OPTION_TYPES: ###########################################################

            if control_type == "Menu": ###########################################################
              try:
                value  = int(control_value)
                if len(options) <= value:
                  value = current_value
              except Exception as e:
                value = current_value

            elif control_type == "Selection": ###########################################################
              try:
                value  = str(control_value)
                if value not in options:
                  value = None
              except Exception as e:
                 value = current_value
              if value not in options:
                value = options[0]
              
            elif control_type == "Selections": ###########################################################
                values = []
                for item in [str(item) for item in control_value]:
                  if item in options:
                    values.append(item)
                # An empty list is a legitimate value here (nothing selected), which is
                # why this assigns unconditionally rather than guarding on len().
                value = values



      elif control_type == "ColorRGB": ###########################################################      
        if len(current_value) != 3:
          current_value = [255,255,255]
        new_value = []
        try:
          control_value = list(control_value)
          for i, val in enumerate(control_value):
            try:
              val = int(val)
              if 0 <= val <= 255:
                new_value.append(val)
            except:
              pass
        except Exception as e:
            control_value = []

        if len(new_value) == 3:
          value = new_value
        else:
          value = current_value


      else:
        values = []
        for i in range(control_length):
          cur_value = current_value[i]
          add_value = copy.deepcopy(cur_value)
          if len(control_value) > i:
            add_value = control_value[i]

          if control_type in STRING_TYPES: ###########################################################
            add_value = str(add_value)


          elif control_type in BOOL_TYPES: ###########################################################
              try:
                  add_value  = (add_value == True or add_value == 'True' or add_value == 'true')
              except Exception as e:
                  pass

          elif control_type in INT_TYPES:  ###########################################################
            try:
              add_value = int(float(add_value))
              if int(float(min_bound)) != -999 and add_value < min_bound:
                add_value = min_bound
              if int(float(max_bound)) != -999 and add_value > max_bound:
                add_value = max_bound
            except Exception as e:
              add_value = cur_value


          elif control_type in FLOAT_TYPES: ###########################################################

            try:
              add_value  = float(add_value)
              round = control_dict['round']
              if round >= 0:
                add_value = round(add_value,round)
              # Reset valid = True here, discarding the low handle's verdict.
              if float(min_bound) != -999 and add_value < min_bound:
                add_value = max_bound
              if float(max_bound) != -999 and add_value > max_bound:
                add_value = max_bound
            except Exception as e:
               add_value = cur_value


          elif control_type in TRIGGER_TYPES: ###########################################################
              try: 
                add_value = float(control_value)
              except:
                add_value = 0

          values.append(add_value)
        value = values


  clean_value = None
  if value is not None:

      if control_type in SINGLE_TYPES and len(value) > 0:
          clean_value = value[0]
      elif control_type in DOUBLE_TYPES and len(value) > 1:
          clean_value = [value[0],value[1]]
      elif control_type in TRIPLE_TYPES and len(value) > 2:
          clean_value = [value[0],value[1],value[2]]
      elif control_type in LIST_TYPES:
          clean_value = value
        
  return clean_value


def get_value(controls_dict, control_name, index = None):
  value = None
  control_type = None
  if controls_dict is not None:
    if control_name in controls_dict.keys():
        try:
          control_type = controls_dict[control_name]['type']
          control_value = get_clean_value(controls_dict, control_name)
          if control_value is None:
              logger.log_warn("Got None Value for control: " + str([control_name, controls_dict[control_name]]))
              pass
          else:
            if index is None:
              value = control_value
            else:
              try:
                index = int(index)
                if index > 0:
                  if isinstance(control_value, list):
                    if len(control_value) > index:
                      value = control_value[index]
              except:
                value = None

        except:
          pass

    ###################
    # Special Types Support
    if value is not None and control_type == 'ColorRGB':
      try:
        value = tuple(value)
      except:
        value = None

    if control_type == 'Button':
      if value <= 0:
        value = -999
      else:
        value = nepi_utils.get_time() - value

  return value

def get_values_dict(controls_dict):
  controls_values_dict = dict()
  if controls_dict is not None:
    for control_name in controls_dict.keys():
      control_value = get_value(controls_dict, control_name)
      if control_value is not None:
        controls_values_dict[control_name] = control_value
      else:
        #logger.log_warn("Got None Value for control: " + str(control_name))
        pass
  return controls_values_dict


def get_params_dict(controls_dict):
  controls_values_dict = dict()
  if controls_dict is not None:
    for control_name in controls_dict.keys():
      control_value = get_value(controls_dict, control_name)
      param = controls_dict[control_name].get('param',True)
      if control_value is not None and param == True:
        controls_values_dict[control_name] = control_value
      else:
        #logger.log_warn("Got None Value for control: " + str(control_name))
        pass
  return controls_values_dict

def set_value(controls_dict, control_name, update_value, index = None,  check_valid = True):
  if control_name in controls_dict.keys():
      
      if index is not None:
        try:
          index = int(index)
          if index > 0:
            control_value = get_value(controls_dict,control_name)
            if isinstance(control_value, list):
              if len(control_value) > index:
                control_value[index] = update_value
                update_value = control_value
        except:
          pass

      if check_valid == False:
        update_value = get_clean_value(controls_dict, control_name, update_value)
      if update_value is not None:
        controls_dict[control_name]['value'] = update_value
  return controls_dict

def sets_values(controls_dict, controls_values_dict):
  controls_values_dict = dict()
  for control_name in controls_values_dict.keys():
     control_value = controls_values_dict[control_name]
     controls_dict = set_value(controls_dict, control_name, control_value)
  return controls_dict


def reset_value(controls_dict, control_name):
  controls_dict[control_name]['value'] = controls_dict[control_name]['default']
  return controls_dict

def reset_values(controls_dict):
    control_names = list(controls_dict.keys())
    for control_name in control_names:
      controls_dict = reset_value(controls_dict, control_name)
    return controls_dict


def get_display_labels(controls_dict, control_name):
  display_labels = []
  if control_name in controls_dict.keys():
      display_labels = controls_dict[control_name].get('display_labels',[])
  return display_labels


def set_display_labels(controls_dict, control_name, display_labels):
  display_labels = [str(item) for item in display_labels]
  if control_name in controls_dict.keys():
      controls_dict[control_name]['display_labels'] = display_labels
  return controls_dict


def get_options(controls_dict, control_name):
  # Read 'options' before, a key Control.msg does not define and
  # BLANK_CONTROL_DICT therefore never carries -- so this raised KeyError for
  # every control. The field is 'options'.
  options = []
  if control_name in controls_dict.keys():
      options = controls_dict[control_name].get('options',[])
  return options

def set_options(controls_dict, control_name, options):
  options = [str(item) for item in options]
  if control_name in controls_dict.keys():
      controls_dict[control_name]['options'] = options
  return controls_dict




def get_bounds(controls_dict, control_name):
  bounds = [-999,-999]
  if control_name in controls_dict.keys():
      min_bound = controls_dict[control_name]['min_bound']
      max_bound = controls_dict[control_name]['max_bound']
  return [min_bound, max_bound]



def set_min_bound(controls_dict, control_name, min_bound = None):
  if min_bound is None:
    min_bound = -999
  if control_name in controls_dict.keys():
      input_type = controls_dict[control_name]['type']
      max_bound = controls_dict[control_name]['max_bound']
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
        controls_dict[control_name]['min_bound'] = min_bound
  return controls_dict

def clear_min_bound(controls_dict, control_name):
  controls_dict = set_min_bound(controls_dict, control_name)
  return controls_dict

def set_max_bound(controls_dict, control_name, max_bound = None):
  if max_bound is None:
    max_bound = -999
  if control_name in controls_dict.keys():
      input_type = controls_dict[control_name]['type']
      min_bound = controls_dict[control_name]['min_bound']
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
        controls_dict[control_name]['max_bound'] = max_bound
  return controls_dict

def clear_max_bound(controls_dict, control_name):
  controls_dict = set_max_bound(controls_dict, control_name)
  return controls_dict

def set_bounds(controls_dict, control_name, bounds = [-999,-999]):
  if len(bounds) == 2:
    [min_bound,max_bound] = bounds
    if min_bound is None:
      min_bound = -999
    if max_bound is None:
      max_bound = -999
    try:
      if int(float(min_bound)) == -999 or int(float(max_bound)) == -999 or min_bound <  max_bound:

        if control_name in controls_dict.keys():
            controls_dict = set_min_bound(controls_dict, control_name, min_bound)
            controls_dict = set_max_bound(controls_dict, control_name, max_bound)
    except:
      pass

  return controls_dict




##################
# Display Functions

def get_display_name(controls_dict, control_name):
  display_name = ''
  if control_name in controls_dict.keys():
      display_name = controls_dict[control_name]['display_name']
  return display_name

def set_display_name(controls_dict, control_name, display_name):
  display_name = str(display_name)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['display_name'] = display_name
  return controls_dict


def get_description(controls_dict, control_name):
  description = ''
  if control_name in controls_dict.keys():
      description = controls_dict[control_name]['description']
  return description

def set_description(controls_dict, control_name, description):
  description = str(description)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['description'] = description
  return controls_dict

def get_hidden(controls_dict, control_name):
  display_hidden = False
  if control_name in controls_dict.keys():
      display_hidden = (controls_dict[control_name]['display_hidden'] == True)
  return display_hidden

def set_hidden(controls_dict, control_name, display_hidden):
  # str() here wrote the strings 'True'/'False' into Control.display_hidden, a toggle
  # field. convert_dict2msg then rejected the dict and the control vanished
  # from the status message instead of being display_hidden in it.
  display_hidden = (display_hidden == True)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['display_hidden'] = display_hidden
  return controls_dict

def get_disabled(controls_dict, control_name):
  disabled = False
  if control_name in controls_dict.keys():
      disabled = (controls_dict[control_name]['disabled'] == True)
  return disabled

def set_disabled(controls_dict, control_name, disabled):
  # str() here wrote the strings 'True'/'False' into Control.disabled, a toggle
  # field. convert_dict2msg then rejected the dict and the control vanished
  # from the status message instead of being disabled in it.
  disabled = (disabled == True)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['disabled'] = disabled
  return controls_dict


def get_display_order(controls_dict, control_name):
  order = -1
  if control_name in controls_dict.keys():
      ordered_list = list(controls_dict.keys())
      order = ordered_list.index(control_name)
  return order

def set_display_order(controls_dict, control_name, update_order = 0):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  cur_order = -1
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    if cur_order != -1 and update_order >= 0 and update_order < num_controls:
      update_ordered_list = list(controls_dict.keys())
      update_ordered_list.remove(control_name)
      num_controls = len(update_ordered_list)
      if update_order == num_controls:
        update_ordered_list.append(control_name)
      else:
          update_ordered_list.insert(update_order, control_name)
      update_controls_dict = {key: controls_dict[key] for key in update_ordered_list}
  return update_controls_dict



def move_control_top(controls_dict, control_name):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    update_order = 0
    if cur_order != update_order and update_order >= 0 and update_order < num_controls:
      update_controls_dict = set_display_order(controls_dict, control_name, update_order)
  return update_controls_dict

def move_control_bottom(controls_dict, control_name):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    update_order = num_controls - 1
    if cur_order != update_order and update_order >= 0 and update_order < num_controls:
      update_controls_dict = set_display_order(controls_dict, control_name, update_order)
  return update_controls_dict

def move_control_up(controls_dict, control_name):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    update_order = cur_order + 1
    if cur_order != -1 and update_order >= 0 and update_order < num_controls:
      update_controls_dict = set_display_order(controls_dict, control_name, update_order)
  return update_controls_dict

def move_control_down(controls_dict, control_name):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    update_order = cur_order - 1
    if cur_order != -1 and update_order >= 0 and update_order < num_controls:
      update_controls_dict = set_display_order(controls_dict, control_name, update_order)
  return update_controls_dict

############################################################
# Status Msg Functions

def create_status_msg( name = '', display_name = '', description = ''):
  status_msg = ControlsStatus()
  name = nepi_utils.get_clean_name(str(name))
  status_msg.name= name
  if display_name == '':
    display_name = name
  status_msg.display_name= str(display_name)
  if description == '':
    description = name
  status_msg.description= str(description)
  return status_msg


def update_status_msg( status_msg, controls_dict):
  if status_msg is None:
    status_msg = ControlsStatus()


  names_list = [] 
  types_list = [] 
  msgs_list = [] 

  try:
    names = list(controls_dict.keys())
  except Exception as e:
    logger.log_warn("update_status_msg: could not read control names from controls dict: " +
                    type(e).__name__ + ": " + str(e))
    names = []
  for name in names:
    try:
      control_dict = controls_dict[name]
      control_type = control_dict['type']
      if control_type in CONTROL_TYPES:

        # Convert default and value to string lists for Controls Msg
        value = control_dict['value']
        default = control_dict['default']

        if control_type == 'Button':
          if value <= 0:
            value = -999
          else:
            value = nepi_utils.get_time() - value
 
        if control_type in LIST_TYPES:
          if isinstance(value, list):
              msg_value = [str(item) for item in value]
              msg_default = [str(item) for item in default]
          else:
            msg_value = [str(value)]
            msg_default = [str(default)]
        else:
          msg_value = [str(value)]
          msg_default = [str(default)]
        control_dict['value'] = msg_value
        control_dict['default'] = msg_default

        msg_dict = nepi_sdk.convert_msg2dict(Control())
        for key in msg_dict.keys():
          if key in control_dict.keys():
            msg_dict[key] = control_dict[key]


        msg_type = 'nepi_interfaces/Control'
        control_msg = nepi_sdk.convert_dict2msg(msg_type,msg_dict)
        if control_msg is not None:
          names_list.append(name)
          types_list.append(control_type)
          msgs_list.append(control_msg)
      else:
        # Same silent fall-through as create_controls_dict: a control that made
        # it into the dict but carries a type this list does not know is simply
        # left out of the status message, so the RUI never sees it.
        logger.log_warn("update_status_msg: left control '" + str(name) +
                        "' of declared type '" + str(control_type) +
                        "' out of the status message: type is not one of " + str(CONTROL_TYPES),
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

def apply_update_msg( controls_dict, msg):
  name = msg.name

  if name not in controls_dict.keys():
    return controls_dict

  control_dict = controls_dict[name]
  control_type = control_dict['type']

  display_name = msg.display_name
  if display_name != '':
    controls_dict[name]['display_name'] = display_name

  description = msg.description
  if description != '':
    controls_dict[name]['description'] = description

  index = msg.index
  if index == '':
    index = None

  value = msg.value
  if value != ['']:
    value = get_clean_value(controls_dict, name, value)
    if value is not None:
      controls_dict = set_value(controls_dict, name, value, index = index)

  min_bound = msg.min_bound
  if min_bound != '':
    controls_dict = set_min_bound(controls_dict, name, min_bound = min_bound)

  max_bound = msg.max_bound
  if max_bound != '':
    controls_dict = set_max_bound(controls_dict, name, max_bound = max_bound)

  options = msg.options
  if options != ['']:
    controls_dict = set_options(controls_dict, name, options)

  return controls_dict