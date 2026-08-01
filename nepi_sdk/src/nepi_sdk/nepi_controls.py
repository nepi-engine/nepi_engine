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

from std_msgs.msg import Empty, Int8, UInt32, Int32, Bool, String, StringArray, Float32, Float64

from nepi_interfaces.msg import Control, ControlsStatus




from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_controls"
logger = Logger(log_name = log_name)


#########################
### Controls Helper Functions




CONTROL_TYPES = ["Menu","Selection","Selections","Trigger","Bool", "String", "Int","Float","FloatSlider","FloatsSlider"]

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

  names
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

        if input_type == "Menu": ###########################################################
            string_options = [str(item) for item in input_dict['options']]
            control_dict['string_options'] = string_options

            value  = int(input_dict['default'])
            if value >= len(string_options):
              value = 0
              if len(string_options) > 0:
                value = 1
            control_dict['factory_index'] = value
            control_dict['default_index'] = value
            control_dict['set_index'] = value

        elif input_type == "Selection": ###########################################################
            string_options = [str(item) for item in input_dict['options']]
            control_dict['string_options'] = string_options

            value  = str(input_dict['default'])
            if value not in string_options:
              value = string_options[0]
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
              value = float(int_bounds[1])
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
  if value_key != 'factory' or value_key != 'default' or value_key != 'set':
    value_key = 'set'
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
  if value_key != 'factory' or value_key != 'default' or value_key != 'set':
    value_key = 'set'
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']
      
      if control_type == "Menu": ###########################################################
        value_str = value_key + '_index'  
        string_options = control_dict['options']
        try:
          value  = int(update_value)
          if value > len(string_options):
            control_dict[value_str] = value
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Index out of range" + str(len(string_options)))
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e))     
    
      elif control_type == "Selection": ###########################################################
        value_str = value_key + '_string'
        string_options = control_dict['options']
        try:
          value  = str(update_value)
          if value in string_options:
            control_dict['default_string'] = value
          else:
            logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Not in options" + str(string_options))
        except Exception as e:
          logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : " + str(e)) 

        
      elif control_type == "Selections": ###########################################################
        value_str = value_key + '_strings'
        string_options = control_dict['options']
        try:
          check_values  = [str(item) for item in update_value]
          values = []
          for value in check_values:
            if value in string_options:
              values.append(value)
            else:
              logger.log_warn('Failed to update ' + str(control_name) + " to " + str(update_value) + " : Not in options" + str(string_options))
          control_dict[value_str] = value
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
          if int(float_bounds[0]) != -999 and value < float_bounds[0]:
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
    value = get_control_default_value(controls_dict, control_name)(controls_dict, control_name, value_key = 'default')
  return value

def set_control_default_value(controls_dict, control_name, update_value):
  controls_dict = set_control_value(controls_dict, control_name, update_value, value_key = 'default' )
  return controls_dict

def get_control_factory_value(controls_dict, control_name):
  value = None
  if control_name in controls_dict.keys(): 
    value = get_control_default_value(controls_dict, control_name)(controls_dict, control_name, value_key = 'factory')
  return value

def set_control_factory_value(controls_dict, control_name, update_value):
  controls_dict = set_control_value(controls_dict, control_name, update_value, value_key = 'factory' )
  return controls_dict



def reset_control_value(controls_dict, control_name, value_key = 'default'):
  if value_key != 'factory' or value_key != 'default':
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
        reset_str = reset_str + '_in_stringdex'  
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
  options = []
  if control_name in controls_dict.keys():
      options = controls_dict[control_name]['options']
  return options

def set_control_options(controls_dict, control_name, options):
  string_options = [str(item) for item in options]
  if control_name in controls_dict.keys():
      control_dict = controls_dict[control_name]
      control_type = control_dict['type']
      if control_type == "Menu": ###########################################################
            control_dict['options'] = string_options

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
        control_dict['options'] = string_options

        # value  = str(control_dict['default_string'])
        # if value not in string_options:
        #   value = string_options[0]
        # control_dict['default_string'] = value

        # value  = str(control_dict['set_string'])
        # if value not in string_options:
        #   value = string_options[0]
        # control_dict['set_string'] = value
        
      elif control_type == "Selections": ###########################################################
        control_dict['options'] = string_options

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
  try:
    hidden = (hidden == True)
  except:
    pass
  if control_name in controls_dict.keys():
      hidden = controls_dict[control_name]['hidden']
  return hidden

def set_control_hidden(controls_dict, control_name, hidden):
  hidden = str(hidden)
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
        names_list.appned(name)
        types_list.appned(control_type)
        msgs_list.appned(control_msg)
        hidden_list.appned(control_msg.hidden)
    except:
      pass
    status_msg.controls_name_list = names_list
    status_msg.controls_type_list = types_list
    status_msg.controls_msg_list = msgs_list
    status_msg.controls_hidden_list = hidden_list
  return status_msg

