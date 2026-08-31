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




CONTROL_TYPES = ["Menu","Selection","Selections","Trigger","Bool", "String", "Int","Float","FloatSlider","FloatSliders"]

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
  except:
    names = []

  for name in names:
    try:
      input_dict = init_dict[name]
      input_type = input_dict['type']
      if input_type in CONTROL_TYPES:
        control_dict = copy.deepcopy(BLANK_CONTROL_DICT)
        control_dict['round_value'] = -1
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

        elif input_type == "Selection": ###########################################################
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

            values  = [float(input_dict['default'][0]),float(input_dict['default'][0])]
            round_value = control_dict['round_value']
            if round_value >= 0:
              values[0] = round(values[0],round_value)
              values[1] = round(values[1],round_value)

            if float(float_bounds[0]) != -999 and values[0] < float_bounds[0]:
              value[0] = float(float_bounds[0])
            if float(float_bounds[1]) != -999 and value[0] > float_bounds[1]:
              value[0] = float(int_bounds[1])

            if float(float_bounds[0]) != -999 and values[0] < float_bounds[0]:
              value[1] = float(float_bounds[0])
            if float(float_bounds[1]) != -999 and value[1] > float_bounds[1]:
              value[1] = float(int_bounds[1])

            if value[0] > value[1]:
              value[0] = value[1]
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
    except:
      pass
    
  return controls_dict

##################
# Controls Functions

def get_control_value(controls_dict, control_name, value_key = 'set'):
  # Was written as a chain of 'or' comparisons, which is true for every possible
  # value_key -- so every read and write landed on the 'set' slot and the factory
  # and default tiers were unreachable.
  if value_key not in ('factory', 'default', 'set'):
    value_key = 'set'
  value = None
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']

      if control_type == "Menu": ###########################################################
        value_str = value_key + '_index'
        value = control_dict[value_str]
    
      elif control_type == "Selection": ###########################################################
        value_str = value_key + '_string'
        value = control_dict[value_str] 

        
      elif control_type == "Selections": ###########################################################
        value_str = value_key + '_strings'
        value = control_dict[value_str]

      if control_type == "Int":  ###########################################################
        value_str = value_key + '_int'
        value = control_dict[value_str] 

      elif control_type == "Float" or control_type == "FloatSlider": ###########################################################
        value_str = value_key + '_float'   
        value = control_dict[value_str] 

      elif control_type == "FloatSliders": ###########################################################      
        value_str = value_key + '_floats'
        value = control_dict[value_str]

      elif control_type == "Trigger": ###########################################################
          value_str = value_key + '_float'
          value = control_dict[value_str]

      elif control_type == "Bool": ###########################################################
          value_str = value_key + '_bool'
          value = control_dict[value_str]
          
      elif control_type == "String": ###########################################################
        value_str = value_key + '_string'
        value = control_dict[value_str]
    
  return value


def set_control_value(controls_dict, control_name, update_value, value_key = 'set'):
  # Was written as a chain of 'or' comparisons, which is true for every possible
  # value_key -- so every read and write landed on the 'set' slot and the factory
  # and default tiers were unreachable.
  if value_key not in ('factory', 'default', 'set'):
    value_key = 'set'
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']
      
      if control_type == "Menu": ###########################################################
        value_str = value_key + '_index'  
        string_options = control_dict['string_options']
        try:
          value  = int(update_value)
          if 0 <= value < len(string_options):
            control_dict[value_str] = value
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Index out of range" + str(len(string_options)))
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e))     
    
      elif control_type == "Selection": ###########################################################
        value_str = value_key + '_string'
        string_options = control_dict['string_options']
        try:
          value  = str(update_value)
          if value in string_options:
            control_dict[value_str] = value
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Not in options" + str(string_options))
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e)) 

        
      elif control_type == "Selections": ###########################################################
        value_str = value_key + '_strings'
        string_options = control_dict['string_options']
        try:
          # Declarative full-selection update: the message carries the complete
          # desired list of selected options. Keep only valid options.
          values = []
          for value in [str(item) for item in update_value]:
            if value in string_options:
              values.append(value)
            else:
              logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Not in options" + str(string_options))
          control_dict[value_str] = values
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e)) 

      if control_type == "Int":  ###########################################################
        value_str = value_key + '_int'
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
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Value out of range: " + str(int_bounds))
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e)) 

      elif control_type == "Float" or control_type == "FloatSlider": ###########################################################
        value_str = value_key + '_float'   
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
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Value out of range: " + str(float_bounds))
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e)) 


      elif control_type == "FloatSliders": ###########################################################      
        value_str = value_key + '_floats'
        float_bounds = control_dict['float_bounds']
        if len(float_bounds) < 2:
          float_bounds = [-999,-999]

        try:
          value  = float(update_value[0])
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
          valid = True
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
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Value out of range: " + str(float_bounds))
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e)) 

      elif control_type == "Trigger": ###########################################################
          value_str = value_key + '_float'
          control_dict[value_str] = nepi_utils.get_time()

      elif control_type == "Bool": ###########################################################
          value_str = value_key + '_bool'
          try:
              value  = (update_value == True)
              control_dict[value_str] = value
          except Exception as e:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e))
          
      elif control_type == "String": ###########################################################
        value_str = value_key + '_string'
        value = str(update_value)
        control_dict[value_str] = value

      ###########################################################
      controls_dict[control_name] = control_dict
  return controls_dict

def get_control_default_value(controls_dict, control_name):
  value = None
  if control_name in controls_dict.keys():
    value = get_control_value(controls_dict, control_name, value_key = 'default')
  return value

def set_control_default_value(controls_dict, control_name, update_value):
  controls_dict = set_control_value(controls_dict, control_name, update_value, value_key = 'default' )
  return controls_dict

def get_control_factory_value(controls_dict, control_name):
  value = None
  if control_name in controls_dict.keys():
    value = get_control_value(controls_dict, control_name, value_key = 'factory')
  return value

def set_control_factory_value(controls_dict, control_name, update_value):
  controls_dict = set_control_value(controls_dict, control_name, update_value, value_key = 'factory' )
  return controls_dict



def reset_control_value(controls_dict, control_name, value_key = 'default'):
  # Same always-true 'or' chain as get/set_control_value -- it forced every
  # reset onto the 'default' tier, making factory_reset identical to reset.
  if value_key not in ('factory', 'default'):
    value_key = 'default'
  if value_key == 'default':
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
        
      elif control_type == "Selection": ###########################################################
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
  controls_dict = reset_control_value(controls_dict, control_name, value_key = 'factory')
  controls_dict = reset_control_value(controls_dict, control_name, value_key = 'default')
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
        
      elif control_type == "Selection": ###########################################################
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

def set_control_bounds(controls_dict, control_name, bounds = []):

  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']

      if control_type == "Int": ###########################################################
        int_bounds = [-999,-999]
        try:
          int_bounds[0] = int(bounds[0])
        except:
          pass
        try:
          int_bounds[1] = int(bounds[1])
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
        try:
          float_bounds[0] = float(bounds[0])
        except:
          pass
        try:
          float_bounds[1] = float(bounds[1])
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
        try:
          float_bounds[0] = float(bounds[0])
        except:
          pass
        try:
          float_bounds[1] = float(bounds[1])
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
  except:
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
    except:
      pass
    status_msg.controls_name_list = names_list
    status_msg.controls_type_list = types_list
    status_msg.controls_msg_list = msgs_list
    status_msg.controls_hidden_list = hidden_list
  return status_msg


############################################################
# Settings Compatibility Functions
#
# NEPI device drivers, and system_mgr for the system config, describe their
# settings as string-valued dicts:
#
#   cap setting : {'name': str, 'type': str, 'options': [str, ...]}
#                 (optionally 'default_value': str)
#   setting     : {'name': str, 'type': str, 'value': str}
#
# with setting types "Menu", "Discrete", "String", "Bool", "Int", "Float".
# SettingsIF adapts that form to and from a controls dict, so a device's
# settings ride the same Control/ControlsStatus messages every other control
# in the platform uses and the capability report travels in the status message
# instead of a separate query service. The driver-facing dict contract is
# unchanged -- these helpers exist so no driver has to know about controls.

SETTING_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

# Returned by a device that exposes no settings. The 'None' type maps to no
# control type, so such a device reports an empty controls list.
NONE_CAP_SETTINGS = {"None":{"name":"None","type":"None","options":[]}}
NONE_SETTINGS = {"None":{"name":"None","type":"None","value":"None"}}

SETTING_TYPE_2_CONTROL_TYPE = {
  'Menu': 'Menu',
  'Discrete': 'Selection',
  'String': 'String',
  'Bool': 'Bool',
  'Int': 'Int',
  'Float': 'Float'
}

CONTROL_TYPE_2_SETTING_TYPE = {
  'Menu': 'Menu',
  'Selection': 'Discrete',
  'String': 'String',
  'Bool': 'Bool',
  'Int': 'Int',
  'Float': 'Float'
}


def get_data_from_setting(setting):
  s_name = 'Missing'
  s_type = 'Missing'
  data = None

  setting_str = str(setting)
  try:
    s_name = setting['name']
    s_type = setting['type']
    s_value = setting['value']
  except Exception as e:
    logger.log_warn("Failed to check setting: " + setting_str + " with exception: " + str(e))
    return s_name, s_type, data

  if s_type is not None and s_value is not None:
    try:
      if s_type == "Bool":
        data = (s_value == "True")
      elif s_type == "Int":
        data = int(float(s_value))
      elif s_type == "Float":
        data = float(s_value)
      elif s_type == "String":
        data = s_value
      elif s_type == "Discrete":
        data = s_value
      elif s_type == "Menu":
        data = int(s_value.split(":")[1])
    except Exception as e:
      logger.log_info("Setting conversion failed for setting " + setting_str + " with exception " + str(e))
  return s_name, s_type, data


def check_valid_setting(setting, cap_settings):
  valid = False
  setting_str = str(setting)
  try:
    s_name = setting['name']
    s_type = setting['type']
    s_value = setting['value']
  except Exception as e:
    logger.log_warn("Failed to check setting: " + setting_str + " with exception: " + str(e))
    return False
  if s_name in cap_settings.keys():
      cap_setting = cap_settings[s_name]
      c_type = cap_setting['type']
      c_options = cap_setting['options'] if 'options' in cap_setting.keys() else []
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
            val = int(float(s_value))
            valid = True
          except Exception as e:
            logger.log_debug("Invalid Int setting value: " + setting_str + " : " + str(e))
          if valid == True and len(c_options) == 2:
            try:
              if val < int(c_options[0]) or val > int(c_options[1]):
                valid = False
            except:
              pass
        elif c_type == "Float":
          try:
            val = float(s_value)
            valid = True
          except Exception as e:
            logger.log_debug("Invalid Float setting value: " + setting_str + " : " + str(e))
          if valid == True and len(c_options) == 2:
            try:
              if val < float(c_options[0]) or val > float(c_options[1]):
                valid = False
            except Exception as e:
              logger.log_debug("Invalid Float setting bounds: " + str(c_options) + " : " + str(e))
  return valid


def get_setting_from_settings(setting_name, settings):
  setting = None
  if setting_name in settings.keys():
    setting = settings[setting_name]
  return setting


def get_settings_by_type(settings, type_str):
  settings_of_type = dict()
  for setting_name in settings.keys():
    setting = settings[setting_name]
    if setting['type'] == type_str:
        settings_of_type[setting['name']] = setting
  return settings_of_type


def get_menu_option_index(string_options, value_str):
  # A settings Menu option string is "name:device_index", and the setting value
  # is one whole option string. The control carries the *list position*, so the
  # value string round-trips as string_options[set_index] with no information
  # lost. Fall back to matching on the name half, for a device that reports a
  # bare name, then to the first option.
  value_str = str(value_str)
  if value_str in string_options:
    return string_options.index(value_str)
  name_only = value_str.split(":")[0]
  for i, option in enumerate(string_options):
    if option.split(":")[0] == name_only:
      return i
  return 0


def create_controls_init_dict_from_settings(cap_settings, factory_settings = None, settings = None):
  init_dict = dict()
  if cap_settings is None:
    return init_dict
  for name in cap_settings.keys():
    try:
      cap_setting = cap_settings[name]
      setting_type = cap_setting['type']
      if setting_type not in SETTING_TYPE_2_CONTROL_TYPE.keys():
        continue
      control_type = SETTING_TYPE_2_CONTROL_TYPE[setting_type]
      options = [str(item) for item in cap_setting['options']] if 'options' in cap_setting.keys() else []

      # Factory value, most specific source first: the cap setting's own
      # declared default, then the device's factory settings, then its live
      # settings.
      default_str = None
      if 'default_value' in cap_setting.keys():
        default_str = str(cap_setting['default_value'])
      elif factory_settings is not None and name in factory_settings.keys() and 'value' in factory_settings[name].keys():
        default_str = str(factory_settings[name]['value'])
      elif settings is not None and name in settings.keys() and 'value' in settings[name].keys():
        default_str = str(settings[name]['value'])

      entry = {'type': control_type, 'description': str(name), 'hidden': False}

      if control_type == "Menu":
        entry['options'] = options
        entry['default'] = get_menu_option_index(options, default_str) if default_str is not None else 0
      elif control_type == "Selection":
        entry['options'] = options
        entry['default'] = default_str if default_str is not None else (options[0] if len(options) > 0 else '')
      elif control_type == "Bool":
        entry['default'] = (default_str == 'True')
      elif control_type == "String":
        entry['default'] = default_str if default_str is not None else ''
      elif control_type == "Int":
        entry['bounds'] = options if len(options) == 2 else []
        try:
          entry['default'] = int(float(default_str))
        except:
          entry['default'] = 0
      elif control_type == "Float":
        entry['bounds'] = options if len(options) == 2 else []
        try:
          entry['default'] = float(default_str)
        except:
          entry['default'] = 0.0

      init_dict[name] = entry
    except Exception as e:
      logger.log_warn("Failed to convert cap setting: " + str(name) + " : " + str(e))
  return init_dict


def create_controls_dict_from_settings(cap_settings, settings = None, factory_settings = None):
  init_dict = create_controls_init_dict_from_settings(cap_settings, factory_settings, settings)
  controls_dict = create_controls_dict(init_dict)
  if settings is not None:
    controls_dict = update_controls_dict_from_settings(controls_dict, settings)
  return controls_dict


def set_control_value_from_setting_str(controls_dict, control_name, value_str, value_key = 'set'):
  if control_name not in controls_dict.keys():
    return controls_dict
  control_type = controls_dict[control_name]['type']
  try:
    if control_type == "Menu":
      string_options = controls_dict[control_name]['string_options']
      update_value = get_menu_option_index(string_options, value_str)
    elif control_type == "Bool":
      update_value = (str(value_str) == 'True')
    elif control_type == "Int":
      update_value = int(float(value_str))
    elif control_type == "Float":
      update_value = float(value_str)
    else:
      update_value = str(value_str)
  except Exception as e:
    logger.log_warn("Failed to convert setting value " + str(value_str) + " for " + str(control_name) + " : " + str(e))
    return controls_dict
  return set_control_value(controls_dict, control_name, update_value, value_key = value_key)


def update_controls_dict_from_settings(controls_dict, settings, value_key = 'set'):
  if settings is None:
    return controls_dict
  for name in settings.keys():
    if name not in controls_dict.keys():
      continue
    setting = settings[name]
    if 'value' not in setting.keys():
      continue
    controls_dict = set_control_value_from_setting_str(controls_dict, name, setting['value'], value_key = value_key)
  return controls_dict


def update_controls_dict_caps_from_settings(controls_dict, cap_settings):
  # Re-apply a device's live capability report (a getCapSettingsFunction that
  # re-reads, e.g. the V4L2 resolution and framerate option lists, which change
  # while the node runs) onto an existing controls dict, keeping the set,
  # default and factory values that are already in it.
  if cap_settings is None:
    return controls_dict
  for name in cap_settings.keys():
    if name not in controls_dict.keys():
      continue
    cap_setting = cap_settings[name]
    options = [str(item) for item in cap_setting['options']] if 'options' in cap_setting.keys() else []
    control_type = controls_dict[name]['type']
    if control_type in ("Menu","Selection","Selections"):
      controls_dict = set_control_options(controls_dict, name, options)
    elif control_type in ("Int","Float","FloatSlider") and len(options) == 2:
      controls_dict = set_control_bounds(controls_dict, name, options)
  return controls_dict


def get_setting_value_str(controls_dict, control_name, value_key = 'set'):
  if control_name not in controls_dict.keys():
    return None
  control_dict = controls_dict[control_name]
  control_type = control_dict['type']
  value = get_control_value(controls_dict, control_name, value_key = value_key)
  if value is None:
    return None
  if control_type == "Menu":
    string_options = control_dict['string_options']
    try:
      return str(string_options[int(value)])
    except:
      return ''
  if control_type == "Bool":
    return "True" if value == True else "False"
  return str(value)


def get_settings_from_controls_dict(controls_dict, value_key = 'set'):
  settings = dict()
  for name in controls_dict.keys():
    control_type = controls_dict[name]['type']
    if control_type not in CONTROL_TYPE_2_SETTING_TYPE.keys():
      continue
    value_str = get_setting_value_str(controls_dict, name, value_key = value_key)
    if value_str is None:
      continue
    settings[name] = {'name': name,
                      'type': CONTROL_TYPE_2_SETTING_TYPE[control_type],
                      'value': value_str}
  return settings


def get_cap_settings_from_controls_dict(controls_dict):
  cap_settings = dict()
  for name in controls_dict.keys():
    control_dict = controls_dict[name]
    control_type = control_dict['type']
    if control_type not in CONTROL_TYPE_2_SETTING_TYPE.keys():
      continue
    if control_type in ("Menu","Selection"):
      options = [str(item) for item in control_dict['string_options']]
    elif control_type == "Int":
      options = [str(item) for item in control_dict['int_bounds']] if len(control_dict['int_bounds']) == 2 else []
    elif control_type == "Float":
      options = [str(item) for item in control_dict['float_bounds']] if len(control_dict['float_bounds']) == 2 else []
    else:
      options = []
    cap_settings[name] = {'name': name,
                          'type': CONTROL_TYPE_2_SETTING_TYPE[control_type],
                          'options': options,
                          'default_value': get_setting_value_str(controls_dict, name, value_key = 'default')}
  return cap_settings


def parse_controls_status_msg(status_msg):
  controls_dict = dict()
  if status_msg is None:
    return controls_dict
  try:
    names_list = status_msg.controls_name_list
    msgs_list = status_msg.controls_msg_list
  except Exception as e:
    logger.log_warn("Failed to parse controls status msg: " + str(e))
    return controls_dict
  for i, name in enumerate(names_list):
    try:
      controls_dict[name] = nepi_sdk.convert_msg2dict(msgs_list[i])
    except Exception as e:
      logger.log_warn("Failed to parse control msg for " + str(name) + " : " + str(e))
  return controls_dict


def create_update_control_msg(control_name, control_type, value):
  # One typed update message for one control. The set_* field that matters is
  # chosen by control_type; the rest stay at their message defaults.
  msg = UpdateControl()
  msg.name = str(control_name)
  msg.type = str(control_type)
  try:
    if control_type == "Menu":
      msg.set_index = int(value)
    elif control_type == "Selection":
      msg.set_string = str(value)
    elif control_type == "Selections":
      msg.set_strings = [str(item) for item in value]
    elif control_type == "Bool":
      msg.set_bool = (value == True)
    elif control_type == "String":
      msg.set_string = str(value)
    elif control_type == "Int":
      msg.set_int = int(float(value))
    elif control_type in ("Float","FloatSlider","Trigger"):
      msg.set_float = float(value)
    elif control_type == "FloatSliders":
      msg.set_floats = [float(value[0]), float(value[1])]
  except Exception as e:
    logger.log_warn("Failed to build update control msg for " + str(control_name) + " : " + str(e))
  return msg


def parse_update_control_msg(msg):
  # Returns (name, type, value) with value read out of the set_* field the
  # declared type selects. Value is None for an unknown type.
  control_name = msg.name
  control_type = msg.type
  value = None
  if control_type == "Menu":
    value = msg.set_index
  elif control_type in ("Selection","String"):
    value = msg.set_string
  elif control_type == "Selections":
    value = list(msg.set_strings)
  elif control_type == "Bool":
    value = msg.set_bool
  elif control_type == "Int":
    value = msg.set_int
  elif control_type in ("Float","FloatSlider"):
    value = msg.set_float
  elif control_type == "FloatSliders":
    value = list(msg.set_floats)
  elif control_type == "Trigger":
    value = msg.set_float
  return control_name, control_type, value


def create_update_control_msg_from_setting(setting, string_options = []):
  # Bridge for a caller that still holds a settings-form dict
  # ({'name','type','value'} with a string value). The string is converted to
  # the control type's own value form before it goes on the wire -- a Menu
  # setting value is one whole "name:index" option string, so it needs the
  # option list to become a list position.
  setting_type = setting['type']
  control_type = SETTING_TYPE_2_CONTROL_TYPE.get(setting_type, setting_type)
  value_str = str(setting['value'])
  if control_type == "Menu":
    value = get_menu_option_index(string_options, value_str)
  elif control_type == "Bool":
    value = (value_str == 'True')
  elif control_type == "Int":
    value = value_str
  elif control_type == "Float":
    value = value_str
  else:
    value = value_str
  return create_update_control_msg(setting['name'], control_type, value)
