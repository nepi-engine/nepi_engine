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




# "Discrete" is an alias of "Selection": a named list of options, one of which is
# current. Driver params yaml files and several driver nodes have always spelled
# it that way, so every dispatch on "Selection" below matches "Discrete" too.
# Note it aliases "Selection" singular, never "Selections" plural.
CONTROL_TYPES = ["Menu","Selection","Discrete","Selections","Trigger","Bool", "String", "Int","Float","FloatSlider","FloatSliders"]

BLANK_CONTROL_DICT = nepi_sdk.convert_msg2dict(Control())

BLANK_CNTROLS_DICT = dict()

EXAMPLE_INIT_DICT = dict(
      pub_rate = {"type":"Float", "default":2, "bounds":[0.1, 15], 'value_round': 2,
                  # OPTIONAL
                  'display_name':'Pub Rate', 'description':'Set pub rate', 'hidden':False, 'display_round': 2,}, 

      index = {"type":"Int", "default":3, "bounds":[3, 10], 
               # OPTIONAL
               'display_name':'Select Index', 'description':'Set index', 'hidden':False}, 

      topic_sel = {"type":"Selection", "default":'Topic1', "options":['Topic1', 'Topic2'], 
                   # OPTIONAL
                   'display_name':'Select Topic', 'description':'Set selected topic', 'hidden':False}, 

      topics_sel = {"type":"Selection", "default":['Topic1', 'Topic2'], "options":['Topic1', 'Topic2'], 
                    # OPTIONAL
                    'display_name':'Select Topics', 'description':'Set selected topics', 'hidden':False}, 

      event_trigger = {"type":"Trigger", "default":0, 
                       # OPTIONAL
                       'display_name':'Event Trigger', 'description':'Event trigger', 'hidden':False}
    )


def get_controls_publisher_namespaces(topics_list = None, types_list = None):
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
    # once per controls set at registration, so a throttle would hide it.
    logger.log_warn("create_controls_dict: could not read control names from init dict: " +
                    type(e).__name__ + ": " + str(e))
    names = []

  for name in names:
    try:
      input_dict = init_dict[name]
      input_type = input_dict['type']
      if input_type in CONTROL_TYPES:
        control_dict = copy.deepcopy(BLANK_CONTROL_DICT)
        control_dict['type'] = input_type
        control_dict['round_value'] = -1
        control_dict['display_name'] = name
        control_dict['description'] = name
        control_dict['round_display'] = 2
        for key in control_dict.keys():
          if key in input_dict.keys():
            control_dict[key] = input_dict[key]
          
        # Control.name was left blank unless the caller happened to repeat the
        # dict key inside its own entry. The key IS the control name.
        control_dict['name'] = name

        if input_type == "Menu": ###########################################################
            string_options = [str(item) for item in input_dict['options']]
            control_dict['string_options'] = string_options

            value  = int(input_dict['default'])
            # Out-of-range fell back to index 1 when any option existed, which
            # silently selected the *second* option. Index 0 is the first.
            if value < 0 or value >= len(string_options):
              value = 0
            control_dict['factory_index'] = value
            control_dict['default_index'] = value
            control_dict['set_index'] = value

        elif input_type == "Selection" or input_type == "Discrete": ###########################################################
            string_options = [str(item) for item in input_dict['options']]
            control_dict['string_options'] = string_options

            value  = str(input_dict['default'])
            if value not in string_options:
              # Indexed [0] unguarded before: IndexError on an empty options
              # list, swallowed by the outer except, dropping the control.
              value = string_options[0] if len(string_options) > 0 else ''
            control_dict['factory_string'] = value
            control_dict['default_string'] = value
            control_dict['set_string'] = value

        elif input_type == "Selections": ###########################################################
            string_options = [str(item) for item in input_dict['options']]
            control_dict['string_options'] = string_options

            check_values  = [str(item) for item in input_dict['default']]
            values = []
            for value in check_values:
              if value in string_options:
                values.append(value)
            control_dict['factory_strings'] = values
            control_dict['default_strings'] = values
            control_dict['set_strings'] = values

        elif input_type == "Int": ###########################################################
            int_bounds = [-999,-999]
            try:
              int_bounds[0] = int(input_dict['bounds'][0])
            except:
              pass
            try:
              int_bounds[1] = int(input_dict['bounds'][1])
            except:
              pass
            control_dict['int_bounds'] = int_bounds

            value  = int(input_dict['default'])
            if int(int_bounds[0]) != -999 and value < int_bounds[0]:
              value = int(int_bounds[0])
            if int(int_bounds[1]) != -999 and value > int_bounds[1]:
              value = int(int_bounds[1])
            control_dict['factory_int'] = value
            control_dict['default_int'] = value
            control_dict['set_int'] = value

        elif input_type == "Float" or input_type == "FloatSlider": ###########################################################
            float_bounds = [-999,-999]
            try:
              float_bounds[0] = float(input_dict['bounds'][0])
            except:
              pass
            try:
              float_bounds[1] = float(input_dict['bounds'][1])
            except:
              pass
            control_dict['float_bounds'] = float_bounds

            value  = float(input_dict['default'])
            round_value = control_dict['round_value']
            if round_value >= 0:
              value = round(value,round_value)
            if float(float_bounds[0]) != -999 and value < float_bounds[0]:
              value = float(float_bounds[0])
            if float(float_bounds[1]) != -999 and value > float_bounds[1]:
              # Clamped with int_bounds[1] here, a name this branch never binds:
              # NameError on any out-of-range Float default, swallowed by the
              # outer except, which dropped the control from the dict entirely.
              value = float(float_bounds[1])
            control_dict['factory_float'] = value
            control_dict['default_float'] = value
            control_dict['set_float'] = value

        elif input_type == "FloatSliders": ###########################################################
            float_bounds = [-999,-999]
            try:
              float_bounds[0] = float(input_dict['bounds'][0])
            except:
              pass
            try:
              float_bounds[1] = float(input_dict['bounds'][1])
            except:
              pass
            control_dict['float_bounds'] = float_bounds

            # Both handles read their own default: the second read
            # input_dict['default'][0] too, so the high handle silently took the
            # low handle's value. Every clamp below read a bare `value`, a name
            # this branch never binds, and two of them clamped to `int_bounds`,
            # a name it never binds either -- so this branch could not complete
            # for any input and no FloatSliders control ever registered.
            values  = [float(input_dict['default'][0]),float(input_dict['default'][1])]
            round_value = control_dict['round_value']
            if round_value >= 0:
              values[0] = round(values[0],round_value)
              values[1] = round(values[1],round_value)

            if float(float_bounds[0]) != -999 and values[0] < float_bounds[0]:
              values[0] = float(float_bounds[0])
            if float(float_bounds[1]) != -999 and values[0] > float_bounds[1]:
              values[0] = float(float_bounds[1])

            # Tested values[0] here as well, so the high handle never got a
            # floor check at all.
            if float(float_bounds[0]) != -999 and values[1] < float_bounds[0]:
              values[1] = float(float_bounds[0])
            if float(float_bounds[1]) != -999 and values[1] > float_bounds[1]:
              values[1] = float(float_bounds[1])

            if values[0] > values[1]:
              values[0] = values[1]
            control_dict['factory_floats'] = values
            control_dict['default_floats'] = values
            control_dict['set_floats'] = values

        elif input_type == "Trigger": ###########################################################
          control_dict['factory_bool'] = 0
          control_dict['default_bool'] = 0
          control_dict['set_bool'] = 0

        elif input_type == "Bool": ###########################################################
            value = False
            try:
                value  = (input_dict['default'] == True)
            except:
              pass
            control_dict['factory_bool'] = value
            control_dict['default_bool'] = value
            control_dict['set_bool'] = value

        elif input_type == "String": ###########################################################
            value = ''
            try:
                value  = str(input_dict['default'])
            except:
              pass
            control_dict['factory_string'] = value
            control_dict['default_string'] = value
            control_dict['set_string'] = value

        # ADD TO CONTROLS if try has not failed

        controls_dict[name] = control_dict

      else:
        # Not an exception path: an unrecognized type simply fails the `if` above
        # and the control is never added. Silent in exactly the same way, and the
        # way every 'Discrete' control was lost, so it gets the same log line.
        logger.log_warn("create_controls_dict: dropped control '" + str(name) +
                        "' of declared type '" + str(input_type) +
                        "': type is not one of " + str(CONTROL_TYPES))
    except Exception as e:
      # A failing control is still skipped and the loop still continues, exactly
      # as before -- the only change is that the failure is now audible. This
      # bare except:pass is why every other defect in this file went unnoticed:
      # a control that raised here vanished from the dict with no error, no log
      # line, and no absence anyone could see except in the RUI.
      #
      # Not throttled. A controls set registers all of its controls in one pass,
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


def get_control_value(controls_dict, control_name, type_key = 'set'):
  if type_key not in ['factory', 'default', 'set']:
    type_key = 'set'
  value = None
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']

      if control_type == "Menu": ###########################################################
        value_str = type_key + '_index'
        value = control_dict[value_str]
    
      elif control_type == "Selection" or control_type == "Discrete": ###########################################################
        value_str = type_key + '_string'
        value = control_dict[value_str] 

        
      elif control_type == "Selections": ###########################################################
        value_str = type_key + '_strings'
        value = control_dict[value_str]

      elif control_type == "Int":  ###########################################################
        value_str = type_key + '_int'
        value = control_dict[value_str] 

      elif control_type == "Float" or control_type == "FloatSlider": ###########################################################
        value_str = type_key + '_float'   
        value = control_dict[value_str] 

      elif control_type == "FloatSliders": ###########################################################      
        value_str = type_key + '_floats'
        value = control_dict[value_str]

      elif control_type == "Trigger": ###########################################################
          value_str = type_key + '_float'
          value = control_dict[value_str]

      elif control_type == "Bool": ###########################################################
          value_str = type_key + '_bool'
          value = control_dict[value_str]
          
      elif control_type == "String": ###########################################################
        value_str = type_key + '_string'
        value = control_dict[value_str]
    
  return value

def get_controls_values_dict(controls_dict, type_key = 'set'):
  controls_values_dict = dict()
  for control_name in controls_dict.keys():
     control_value = get_control_value(controls_dict, control_name, type_key = type_key)
     controls_values_dict[control_name] = control_value
  return controls_values_dict


def set_control_value(controls_dict, control_name, update_value, type_key = 'set' , check_valid = True):
  if type_key not in ('factory', 'default', 'set'):
    type_key = 'set'
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']
      
      if control_type == "Menu": ###########################################################
        value_str = type_key + '_index'  
        string_options = control_dict['string_options']
        try:
          value  = int(update_value)
          if 0 <= value < len(string_options):
            control_dict[value_str] = value
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Index out of range" + str(len(string_options)), throttle_s = 5)
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e), throttle_s = 5)  
    
      elif control_type == "Selection" or control_type == "Discrete": ###########################################################
        value_str = type_key + '_string'
        string_options = control_dict['string_options']
        try:
          value  = str(update_value)
          if value in string_options or check_valid == False:
            control_dict[value_str] = value
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Not in options" + str(string_options), throttle_s = 5)
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e), throttle_s = 5)

        
      elif control_type == "Selections": ###########################################################
        value_str = type_key + '_strings'
        string_options = control_dict['string_options']
        try:
          # Declarative full-selection update: the message carries the complete
          # desired list of selected options. Keep only valid options.
          values = []
          for value in [str(item) for item in update_value]:
            if value in string_options or check_valid == False:
              values.append(value)
            else:
              logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Not in options" + str(string_options), throttle_s = 5)
          control_dict[value_str] = values
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e), throttle_s = 5)

      if control_type == "Int":  ###########################################################
        value_str = type_key + '_int'
        int_bounds = control_dict['int_bounds']
        if len(int_bounds) < 2:
          int_bounds = [-999,-999]
        try:
          value = int(update_value)
          valid = True
          if int(int_bounds[0]) != -999 and value < int_bounds[0]:
            valid = False
          if int(int_bounds[1]) != -999 and value > int_bounds[1]:
            valid = False
          if valid == True:
            control_dict[value_str] = value
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Value out of range: " + str(int_bounds), throttle_s = 5)
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e), throttle_s = 5)

      elif control_type == "Float" or control_type == "FloatSlider": ###########################################################
        value_str = type_key + '_float'   
        float_bounds = control_dict['float_bounds']
        if len(float_bounds) < 2:
          float_bounds = [-999,-999]

        try:
          value = float(update_value)
          round_value = control_dict['round_value']
          if round_value >= 0:
            value = round(value,round_value)
          valid = True
          if float(float_bounds[0]) != -999 and value < float_bounds[0]:
            valid = False
          if float(float_bounds[1]) != -999 and value > float_bounds[1]:
            valid = False
          if valid == True:
            control_dict[value_str] = value
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Value out of range: " + str(float_bounds), throttle_s = 5)
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e), throttle_s = 5)


      elif control_type == "FloatSliders": ###########################################################      
        value_str = type_key + '_floats'
        float_bounds = control_dict['float_bounds']
        if len(float_bounds) < 2:
          float_bounds = [-999,-999]

        try:
          # Bound `value` and then read `value0`, a name never assigned, so
          # every FloatSliders update raised UnboundLocalError.
          value0  = float(update_value[0])
          round_value = control_dict['round_value']
          if round_value >= 0:
            value0 = round(value0,round_value)
          valid = True
          if float(float_bounds[0]) != -999 and value0 < float_bounds[0]:
            valid = False
          if float(float_bounds[1]) != -999 and value0 > float_bounds[1]:
            valid = False


          value1  = float(update_value[1])
          round_value = control_dict['round_value']
          if round_value >= 0:
            value1 = round(value1,round_value)
          # Reset valid = True here, discarding the low handle's verdict. Both
          # handles have to pass for the pair to be accepted.
          if float(float_bounds[0]) != -999 and value1 < float_bounds[0]:
            valid = False
          if float(float_bounds[1]) != -999 and value1 > float_bounds[1]:
            valid = False
          if value0 > value1:
            valid = False
          if valid == True:
            control_dict[value_str][0] = value0
            control_dict[value_str][1] = value1
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Value out of range: " + str(float_bounds), throttle_s = 5)
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e), throttle_s = 5)

      elif control_type == "Trigger": ###########################################################
          value_str = type_key + '_float'
          control_dict[value_str] = nepi_utils.get_time()

      elif control_type == "Bool": ###########################################################
          value_str = type_key + '_bool'
          try:
              value  = (update_value == True or update_value == 'True' or update_value == 'true')
              control_dict[value_str] = value
          except Exception as e:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e), throttle_s = 5)
          
      elif control_type == "String": ###########################################################
        value_str = type_key + '_string'
        value = str(update_value)
        control_dict[value_str] = value

      ###########################################################
      controls_dict[control_name] = control_dict
  return controls_dict

def set_controls_values(controls_dict, controls_values_dict, type_key = 'set'):
  controls_values_dict = dict()
  for control_name in controls_values_dict.keys():
     control_value = controls_values_dict[control_name]
     controls_dict = set_control_value(controls_dict, control_name, control_value, type_key = type_key)
  return controls_dict


def check_valid_value(controls_dict, control_name, update_value):
  valid = False
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']
      
      if control_type == "Menu": ###########################################################
        string_options = control_dict['string_options']
        try:
          value  = int(update_value)
          valid = True
        except Exception as e:
          pass
    
      elif control_type == "Selection" or control_type == "Discrete": ###########################################################
        string_options = control_dict['string_options']
        try:
          value  = str(update_value)
          if value in string_options:
            valid = True
        except Exception as e:
          pass

        
      elif control_type == "Selections": ###########################################################
        string_options = control_dict['string_options']
        try:
          # Declarative full-selection update: the message carries the complete
          # desired list of selected options. Keep only valid options.
          values = []
          for value in [str(item) for item in update_value]:
            if value in string_options:
              values.append(value)
          
          valid = values == update_value
        except Exception as e:
          pass

      if control_type == "Int":  ###########################################################
        int_bounds = control_dict['int_bounds']
        if len(int_bounds) < 2:
          int_bounds = [-999,-999]
        try:
          value = int(update_value)
          valid = True
          if int(int_bounds[0]) != -999 and value < int_bounds[0]:
            valid = False
          if int(int_bounds[1]) != -999 and value > int_bounds[1]:
            valid = False
        except Exception as e:
          pass

      elif control_type == "Float" or control_type == "FloatSlider": ###########################################################
        float_bounds = control_dict['float_bounds']
        if len(float_bounds) < 2:
          float_bounds = [-999,-999]

        try:
          value = float(update_value)
          round_value = control_dict['round_value']
          if round_value >= 0:
            value = round(value,round_value)
          valid = True
          if float(float_bounds[0]) != -999 and value < float_bounds[0]:
            valid = False
          if float(float_bounds[1]) != -999 and value > float_bounds[1]:
            valid = False
        except Exception as e:
          pass


      elif control_type == "FloatSliders": ###########################################################      
        float_bounds = control_dict['float_bounds']
        if len(float_bounds) < 2:
          float_bounds = [-999,-999]

        try:
          # Same value/value0 defect as set_control_value. This one gates the
          # settings update path (system_if.py), so it rejected every
          # FloatSliders update before set_control_value was ever reached.
          value0  = float(update_value[0])
          round_value = control_dict['round_value']
          if round_value >= 0:
            value0 = round(value0,round_value)
          valid = True
          if float(float_bounds[0]) != -999 and value0 < float_bounds[0]:
            valid = False
          if float(float_bounds[1]) != -999 and value0 > float_bounds[1]:
            valid = False


          value1  = float(update_value[1])
          round_value = control_dict['round_value']
          if round_value >= 0:
            value1 = round(value1,round_value)
          # Reset valid = True here, discarding the low handle's verdict.
          if float(float_bounds[0]) != -999 and value1 < float_bounds[0]:
            valid = False
          if float(float_bounds[1]) != -999 and value1 > float_bounds[1]:
            valid = False
          if value0 > value1:
            valid = False

        except Exception as e:
          pass

      elif control_type == "Trigger": ###########################################################
          valid = True

      elif control_type == "Bool": ###########################################################
          try:
              value  = (update_value == True)
              valid = True
          except Exception as e:
            pass
          
      elif control_type == "String": ###########################################################
        valid = True

    
  return valid



def get_control_default_value(controls_dict, control_name):
  value = None
  if control_name in controls_dict.keys(): 
    value = get_control_value(controls_dict, control_name, type_key = 'default')
  return value

def set_control_default_value(controls_dict, control_name, update_value):
  controls_dict = set_control_value(controls_dict, control_name, update_value, type_key = 'default' )
  return controls_dict

def get_control_factory_value(controls_dict, control_name):
  value = None
  if control_name in controls_dict.keys(): 
    value = get_control_value(controls_dict, control_name, type_key = 'factory')
  return value

def set_control_factory_value(controls_dict, control_name, update_value):
  controls_dict = set_control_value(controls_dict, control_name, update_value, type_key = 'factory' )
  return controls_dict


def reset_control_value(controls_dict, control_name, type_key = 'default'):
  if type_key not in ('factory', 'default'):
    type_key = 'default'
  if type_key == 'default':
    update_str = 'set'
    reset_str = 'default'
  else:
    update_str = 'default'
    reset_str = 'factory'
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']
      
      if control_type == "Menu": ###########################################################
        update_str = update_str + '_index'  
        reset_str = reset_str + '_index'  
        control_dict[update_str] = control_dict[reset_str]        
        
      elif control_type == "Selection" or control_type == "Discrete": ###########################################################
        update_str = update_str + '_string'
        reset_str = reset_str + '_string'  
        control_dict[update_str] = control_dict[reset_str] 
        
      elif control_type == "Selections": ###########################################################
        update_str = update_str + '_strings'
        reset_str = reset_str + '_strings'  
        control_dict[update_str] = control_dict[reset_str] 

      if control_type == "Int":  ###########################################################
        update_str = update_str + '_int'
        reset_str = reset_str + '_int'  
        control_dict[update_str] = control_dict[reset_str] 

      elif control_type == "Float" or control_type == "FloatSlider": ###########################################################
        update_str = update_str + '_float'   
        reset_str = reset_str + '_float'  
        control_dict[update_str] = control_dict[reset_str] 


      elif control_type == "FloatSliders": ###########################################################      
        update_str = update_str + '_floats'
        reset_str = reset_str + '_floats'  
        control_dict[update_str] = control_dict[reset_str] 

      elif control_type == "Trigger": ###########################################################
        update_str = update_str + '_float'
        control_dict[update_str] = 0

      elif control_type == "Bool": ###########################################################
        update_str = update_str + '_bool'
        reset_str = reset_str + '_bool'  
        control_dict[update_str] = control_dict[reset_str] 

      elif control_type == "String": ###########################################################
        update_str = update_str + '_string'
        reset_str = reset_str + '_string'
        control_dict[update_str] = control_dict[reset_str]

      ###########################################################
      controls_dict[control_name] = control_dict
  return controls_dict

def reset_control_values(controls_dict):
    control_names = list(controls_dict.keys())
    for control_name in control_names:
      controls_dict = reset_control_value(controls_dict, control_name)
    return controls_dict


def factory_reset_control_value(controls_dict, control_name):
  controls_dict = reset_control_value(controls_dict, control_name, type_key = 'factory')
  controls_dict = reset_control_value(controls_dict, control_name, type_key = 'default')
  return controls_dict

def factory_reset_control_values(controls_dict):
    control_names = list(controls_dict.keys())
    for control_name in control_names:
      controls_dict = factory_reset_control_value(controls_dict, control_name)
    return controls_dict



def get_control_options(controls_dict, control_name):
  # Read 'options' before, a key Control.msg does not define and
  # BLANK_CONTROL_DICT therefore never carries -- so this raised KeyError for
  # every control. The field is 'string_options'.
  options = []
  if control_name in controls_dict.keys():
      options = controls_dict[control_name].get('string_options',[])
  return options

def set_control_options(controls_dict, control_name, options):
  string_options = [str(item) for item in options]
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']
      if control_type == "Menu": ###########################################################
            # Wrote 'options', which is not a Control.msg field. convert_dict2msg
            # rejects a dict with an unknown key and returns None, so updating a
            # control's options used to delete it from the status message.
            control_dict['string_options'] = string_options

            # value  = int(controls_dict['default_index'])
            # if value >= len(string_options):
            #   value = 0
            #   if len(string_options) > 0:
            #     value = 1
            # control_dict['default_index'] = value

            # value  = int(controls_dict['set_index'])
            # if value >= len(string_options):
            #   value = 0
            #   if len(string_options) > 0:
            #     value = 1
            # control_dict['set_index'] = value        
        
      elif control_type == "Selection" or control_type == "Discrete": ###########################################################
        control_dict['string_options'] = string_options

        # value  = str(control_dict['default_string'])
        # if value not in string_options:
        #   value = string_options[0]
        # control_dict['default_string'] = value

        # value  = str(control_dict['set_string'])
        # if value not in string_options:
        #   value = string_options[0]
        # control_dict['set_string'] = value
        
      elif control_type == "Selections": ###########################################################
        control_dict['string_options'] = string_options

        # check_values  = [str(item) for item in control_dict['default_strings']]
        # values = []
        # for value in check_values:
        #   if value in string_options:
        #     values.append(value)
        # control_dict['default_strings'] = values

        # check_values  = [str(item) for item in control_dict['set_strings']]
        # values = []
        # for value in check_values:
        #   if value in string_options:
        #     values.append(value)
        # control_dict['set_strings'] = values

      ###########################################################
      controls_dict[control_name] = control_dict
  return controls_dict




def get_control_bounds(controls_dict, control_name):
  bounds = []
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']
      if control_type == "Int":
        bounds = controls_dict[control_name]['int_bounds']

      elif control_type == "Float" or control_type == "FloatSlider":      
        bounds = controls_dict[control_name]['float_bounds']

      elif control_type == "FloatSliders":      
        bounds = controls_dict[control_name]['float_bounds']

  return bounds

def set_control_bounds(controls_dict, control_name, min_bound = None, max_bound = None):


  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']

      if control_type == "Int": ###########################################################
        int_bounds = [-999,-999]
        cur_bounds = control_dict['int_bounds']
        if len(cur_bounds) == 2:
          int_bounds = cur_bounds

        try:
          int_bounds[0] = int(min_bound)
        except:
          pass
        try:
          int_bounds[1] = int(max_bound)
        except:
          pass
        control_dict['int_bounds'] = int_bounds

        value  = int(control_dict['default_int'])
        if int(int_bounds[0]) != -999 and value < int_bounds[0]:
          value = int(int_bounds[0])
        if int(int_bounds[1]) != -999 and value > int_bounds[1]:
          value = int(int_bounds[1])
        control_dict['default_int'] = value

        value  = int(control_dict['set_int'])
        if int(int_bounds[0]) != -999 and value < int_bounds[0]:
          value = int(int_bounds[0])
        if int(int_bounds[1]) != -999 and value > int_bounds[1]:
          value = int(int_bounds[1])
        control_dict['set_int'] = value

      elif control_type == "Float" or control_type == "FloatSlider": ###########################################################      
        float_bounds = [-999,-999]

        cur_bounds = control_dict['float_bounds']
        if len(cur_bounds) == 2:
          int_bounds = cur_bounds

        try:
          float_bounds[0] = float(min_bound)
        except:
          pass
        try:
          float_bounds[1] = float(max_bound)
        except:
          pass
        control_dict['float_bounds'] = float_bounds

        value  = float(control_dict['default_float'])
        if float(float_bounds[0]) != -999 and value < float_bounds[0]:
          value = float(float_bounds[0])
        if float(float_bounds[1]) != -999 and value > float_bounds[1]:
          value = float(float_bounds[1])
        control_dict['default_float'] = value

        value  = float(control_dict['set_float'])
        if float(float_bounds[0]) != -999 and value < float_bounds[0]:
          value = float(float_bounds[0])
        if float(float_bounds[1]) != -999 and value > float_bounds[1]:
          value = float(float_bounds[1])
        control_dict['set_float'] = value

      elif control_type == "FloatSliders": ###########################################################      
        float_bounds = [-999,-999]
        cur_bounds = control_dict['float_bounds']
        if len(cur_bounds) == 2:
          int_bounds = cur_bounds

        try:
          float_bounds[0] = float(min_bound)
        except:
          pass
        try:
          float_bounds[1] = float(max_bound)
        except:
          pass
        control_dict['float_bounds'] = float_bounds

        value  = float(control_dict['default_floats'][0])
        if float(float_bounds[0]) != -999 and value < float_bounds[0]:
          value = float(float_bounds[0])
        if float(float_bounds[1]) != -999 and value > float_bounds[1]:
          value = float(float_bounds[1])
        control_dict['default_floats'][0] = value

        value  = float(control_dict['default_floats'][1])
        if float(float_bounds[0]) != -999 and value < float_bounds[0]:
          value = float(float_bounds[0])
        if float(float_bounds[1]) != -999 and value > float_bounds[1]:
          value = float(float_bounds[1])

        control_dict['default_floats'][1] = value

        if control_dict['default_floats'][0] > control_dict['default_floats'][1]:
          control_dict['default_floats'][0] = control_dict['default_floats'][1]

        value  = float(control_dict['set_floats'][0])
        if float(float_bounds[0]) != -999 and value < float_bounds[0]:
          value = float(float_bounds[0])
        if float(float_bounds[1]) != -999 and value > float_bounds[1]:
          value = float(float_bounds[1])
        control_dict['set_floats'][0] = value

        value  = float(control_dict['set_floats'][1])
        if float(float_bounds[0]) != -999 and value < float_bounds[0]:
          value = float(float_bounds[0])
        if float(float_bounds[1]) != -999 and value > float_bounds[1]:
          value = float(float_bounds[1])

        control_dict['set_floats'][1] = value

        if control_dict['set_floats'][0] > control_dict['set_floats'][1]:
          control_dict['set_floats'][0] = control_dict['set_floats'][1]

      ###########################################################
      controls_dict[control_name] = control_dict
  return controls_dict

##################
# Display Functions

def get_control_display_name(controls_dict, control_name):
  display_name = ''
  if control_name in controls_dict.keys():
      display_name = controls_dict[control_name]['display_name']
  return display_name

def set_control_display_name(controls_dict, control_name, display_name):
  display_name = str(display_name)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['display_name'] = display_name
  return controls_dict


def get_control_description(controls_dict, control_name):
  description = ''
  if control_name in controls_dict.keys():
      description = controls_dict[control_name]['description']
  return description

def set_control_description(controls_dict, control_name, description):
  description = str(description)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['description'] = description
  return controls_dict

def get_control_hidden(controls_dict, control_name):
  hidden = False
  if control_name in controls_dict.keys():
      hidden = (controls_dict[control_name]['hidden'] == True)
  return hidden

def set_control_hidden(controls_dict, control_name, hidden):
  # str() here wrote the strings 'True'/'False' into Control.hidden, a bool
  # field. convert_dict2msg then rejected the dict and the control vanished
  # from the status message instead of being hidden in it.
  hidden = (hidden == True)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['hidden'] = hidden
  return controls_dict

def get_control_display_order(controls_dict, control_name):
  order = -1
  if control_name in controls_dict.keys():
      ordered_list = list(controls_dict.keys())
      order = ordered_list.index(control_name)
  return order

def set_control_display_order(controls_dict, control_name, update_order = 0):
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



def move_control_display_top(controls_dict, control_name):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    update_order = 0
    if cur_order != update_order and update_order >= 0 and update_order < num_controls:
      update_controls_dict = set_control_display_order(controls_dict, control_name, update_order)
  return update_controls_dict

def move_control_display_bottom(controls_dict, control_name):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    update_order = num_controls - 1
    if cur_order != update_order and update_order >= 0 and update_order < num_controls:
      update_controls_dict = set_control_display_order(controls_dict, control_name, update_order)
  return update_controls_dict

def move_control_display_up(controls_dict, control_name):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    update_order = cur_order + 1
    if cur_order != -1 and update_order >= 0 and update_order < num_controls:
      update_controls_dict = set_control_display_order(controls_dict, control_name, update_order)
  return update_controls_dict

def move_control_display_down(controls_dict, control_name):
  update_controls_dict = copy.deepcopy(controls_dict)
  cur_ordered_list = list(controls_dict.keys())
  num_controls = len(cur_ordered_list)
  if control_name in cur_ordered_list:
    cur_order = cur_ordered_list.index(control_name)
    update_order = cur_order - 1
    if cur_order != -1 and update_order >= 0 and update_order < num_controls:
      update_controls_dict = set_control_display_order(controls_dict, control_name, update_order)
  return update_controls_dict

############################################################
# Status Msg Functions

def create_status_msg( name = '', display_name = '', description = '', show_controls = True, has_show_control = False):
  status_msg = ControlsStatus()
  name = nepi_utils.get_clean_name(str(name))
  status_msg.name= name
  if display_name == '':
    display_name = name
  status_msg.display_name= str(display_name)
  if description == '':
    description = name
  status_msg.description= str(description)
  status_msg.show_controls = show_controls
  status_msg.has_show_control = has_show_control and show_controls == True
  return status_msg


def update_status_msg( status_msg, controls_dict, hidden = False):
  if status_msg is None:
    status_msg = ControlsStatus()
  status_msg.hidden= hidden


  names_list = [] 
  types_list = [] 
  msgs_list = [] 
  hidden_list = [] 

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
        msg_type = 'nepi_interfaces/Control'
        control_msg = nepi_sdk.convert_dict2msg(msg_type,control_dict)
        names_list.append(name)
        types_list.append(control_type)
        msgs_list.append(control_msg)
        hidden_list.append(control_msg.hidden)
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
    status_msg.controls_type_list = types_list
    status_msg.controls_msg_list = msgs_list
    status_msg.controls_hidden_list = hidden_list
  return status_msg

def apply_update_control_msg( controls_dict, msg):
  name = msg.name

  if name not in controls_dict.keys():
    return controls_dict

  control_dict = controls_dict[name]

  display_name = msg.display_name
  if display_name != '':
    controls_dict[name]['display_name'] = display_name

  description = msg.description
  if description != '':
    controls_dict[name]['description'] = description

  value = msg.value
  if value != '':
    controls_dict = set_control_value(controls_dict, name, value)
    
  values = msg.values
  if values != ['']:
    controls_dict = set_control_value(controls_dict, name, values)

  min_bound = msg.min_bound
  if min_bound != '':
    controls_dict = set_control_bounds(controls_dict, name, min_bound = min_bound)

  min_bound = msg.min_bound
  if min_bound != '':
    controls_dict = set_control_bounds(controls_dict, name, min_bound = min_bound)

  options = msg.options
  if options != ['']:
    controls_dict = set_control_options(controls_dict, name, options)

  return controls_dict