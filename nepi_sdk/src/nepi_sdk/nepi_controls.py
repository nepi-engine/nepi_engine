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

EXAMPLE_INIT_DICT = {
            'menu': {
                'type': 'Menu', 'default': 1, 'options': ['Off', 'Low', 'High'],
                'display_name': 'Demo Menu', 'description': 'Pick one menu option (index based).', 'display_hidden': False},

            'string': {
                'type': 'String', 'default': 'hello nepi',
                'display_name': 'Demo String', 'description': 'Free-form text value.', 'display_hidden': False},


            'selection': {
                'type': 'Selection', 'default': 'Bravo', 'options': ['Alpha', 'Bravo', 'Charlie'],
                'display_name': 'Demo Selection', 'description': 'Select a single option by name.', 'display_hidden': False},

            'selections': {
                'type': 'Selections', 'default': ['Red', 'Blue'], 'options': ['Red', 'Green', 'Blue'],
                'display_name': 'Demo Selections', 'description': 'Select any number of options.', 'display_hidden': False},

            'color_rgb': {
                'type': 'ColorRBB', 'value': [0,255,0],
                'display_name': 'Demo Color RGB', 'description': 'A rbg color.', 'display_hidden': False},


            'trigger': {
                'type': 'Button', 'value': True,
                'display_name': 'Demo Button', 'description': 'A trigger age secs.', 'display_hidden': False},


            'trigger_column': {
                'type': 'Button', 'value': [0,2.1],
                'display_name': 'Demo Button', 'description': 'Two  triggers age secs.', 'display_hidden': False},

            'triggers_row': {
                'type': 'Buttons', 'value': [0,2.1],
                'display_name': 'Demo Buttons', 'description': 'Two triggers age secs.', 'display_hidden': False, 'display_row': True},

            'bool_data': {
                'type': 'Toggle', 'value': True,
                'display_name': 'Demo Toggle', 'description': 'Two booleans.', 'display_hidden': False},

            'bools_column': {
                'type': 'Toggle', 'value': [True, False, False],
                'display_name': 'Demo Toggle', 'description': 'Two booleans.', 'display_hidden': False},

            'bools_row': {
                'type': 'Toggle', 'value': [True, False, False],
                'display_name': 'Demo Toggle', 'description': 'Two booleans.', 'display_hidden': False, 'display_row': True},


            'int': {
                'type': 'Int', 'default': 5, 'bounds': [0, 10],
                'display_name': 'Demo Int', 'description': 'Integer value within [0, 10].', 'display_hidden': False},

            'ints_column': {
                'type': 'Ints', 'default': [1,2,3], 'bounds': [0, 10],
                'display_name': 'Demo Ints', 'description': 'Ints value within [0, 10].', 'display_hidden': False},

            'ints_row': {
                'type': 'Ints', 'default': [1,2,3], 'bounds': [0, 10],
                'display_name': 'Demo Ints', 'description': 'Ints value within [0, 10].', 'display_hidden': False, 'display_row': True},

            'float': {
                'type': 'Float', 'default': 2.5, 'bounds': [0.0, 10.0], 'round_value': 2,
                'display_name': 'Demo Float', 'description': 'Float value within [0.0, 10.0].', 'display_hidden': False},

            'floats_column': {
                'type': 'Floats', 'default': [1,5,2.5,3.5], 'bounds': [0.0, 10.0], 'round_value': 2,
                'display_name': 'Demo Floats', 'description': 'Floats value within [0.0, 10.0].', 'display_hidden': False},

            'floats_row': {
                'type': 'Floats', 'default': [1,5,2.5,3.5], 'bounds': [0.0, 10.0], 'round_value': 2,
                'display_name': 'Demo Floats', 'description': 'Floats value within [0.0, 10.0].', 'display_hidden': False, 'display_row': True},

            'float_slider': {
                'type': 'FloatSlider', 'default': 50.0, 'bounds': [0.0, 100.0], 'round_value': 1,
                'display_name': 'Demo Float Slider', 'description': 'Single-value slider over [0, 100].', 'display_hidden': False},

            'range_slider': {
                'type': 'RangeSlider', 'default': [0.25, 0.75], 'bounds': [0.0, 1.0], 'round_value': 2,
                'display_name': 'Demo Floats Slider', 'description': 'Dual-value range slider (0.0-1.0 ratio).', 'display_hidden': False},
          }


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
        control_dict['display_row'] = False
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
        # Clean Display Row
        #############
        # The overlay loop above copies the caller's value verbatim, so a hand
        # written init dict -- or a params yaml, which spells booleans 'True' --
        # can put a string or an int in what Control.msg declares a bool.
        # convert_dict2msg rejects the whole dict on a type mismatch, which
        # drops the control from the published status entirely rather than just
        # mis-rendering it. Same failure mode set_hidden and set_disabled coerce
        # against.
        control_dict['display_row'] = cleanDisplayRow(control_dict['display_row'])

        #############
        # Clean Bounds
        #############
        # Clean Bounds
        min_bound = -999
        max_bound = -999


        # Membership test, not equality against the name of the list. As an
        # equality test this was never true, so min_bound/max_bound stayed at
        # the -999 sentinel for every Int and Float control and a device's
        # reported bounds (v4l2 hands them over as init_control_dict['bounds'])
        # were discarded.
        if input_type in BOUND_TYPES:
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
        # Membership in the list, not a substring test against its name. As a
        # substring test this was never true, so a Button never got its [0]
        # seed: it fell through to the default branch, came out length 0, and
        # was dropped as invalid below.
        if input_type in TRIGGER_TYPES:
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
          # Third drop path, and it was the last silent one: no exception to
          # catch, so nothing was logged.
          logger.log_warn("create_controls_dict: dropped control '" + str(name) +
                          "' of declared type '" + str(input_type) +
                          "': no default or value to seed it with")
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

        # Membership test, not equality against the name of the list. As an
        # equality test this branch never ran, so Menu/Selection/Selections
        # took the else path -- which CLEARS options, at both of its ends. An
        # option type then reached get_clean_value with an empty option list,
        # where the Selection branch indexes options[0] and raised IndexError,
        # so the control was dropped at registration. This is what took the
        # camera's resolution/framerate/exposure_auto/power_line_frequency and
        # drivers_mgr's protocol/baud_rate out of their settings dicts.
        if input_type in OPTION_TYPES:
            if len(options) == 0 and len(display_labels) > 0:
              options = display_labels
            # From the local `options`, which carries the display_labels
            # fallback above; re-reading control_dict['options'] discarded it.
            options = [str(item) for item in options]
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
        # Keyed by the cleaned name, which is what get_clean_value looks up --
        # it cleans the name before indexing, so a raw key it could not find
        # came back None and the control was dropped for no stated reason.
        check_name = control_dict['name']
        check_dict[check_name] = copy.deepcopy(control_dict)

        check_value = copy.deepcopy(value)
        if len(check_value) == 0:
          continue
        if isinstance(check_value, list) == False:
          check_value = [check_value]
        if input_type in SINGLE_TYPES:
          check_value = check_value[0]
        elif input_type in DOUBLE_TYPES:
          check_value = [check_value[0],check_value[1]]
        elif input_type in TRIPLE_TYPES:
          check_value = [check_value[0],check_value[1],check_value[2]]
        clean_value = get_clean_value(check_dict, check_name, check_value)
        #logger.log_warn("Got clean value from check value: " + str(name) + ": " + str(clean_value) + ": " + str(check_value))
        if clean_value is None:
          logger.log_warn("create_controls_dict: dropped control '" + str(name) +
                          "' of declared type '" + str(input_type) +
                          "': value " + str(check_value) + " is not valid for the control")
          continue

        # Store the LIST form of the cleaned value. This took len() of
        # get_clean_value's return, which is the NATIVE form: len(3) and
        # len(True) raise TypeError, so every Int and Toggle was dropped, and a
        # String's length became its character count -- len('/dev/ttyUSB0') is
        # 12 against a one-entry value list -- which is what walked
        # get_clean_value's range(control_length) off the end of current_value
        # and killed drivers_mgr from its set_value call.
        value = get_value_list(clean_value)
        control_dict['value'] = value
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

def get_value_list(value):
  # The controls dict stores every value as a list of strings: 'length' counts
  # list entries, and get_clean_value re-lists whatever it is handed, so a
  # scalar stored in 'value' gets iterated one character at a time. Anything
  # coming back from get_clean_value -- which returns the NATIVE form, a scalar
  # for the single-value types -- has to come back through here before it is
  # stored.
  if value is None:
    return None
  if isinstance(value, list) == False:
    return [str(value)]
  return [str(item) for item in value]


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

      # Callers hand this the NATIVE value -- set_value from a driver, the
      # ControlsIF/SettingsIF wrappers, apply_update_msg. Iterating that
      # directly raised TypeError on an int (SettingsIF.init died here on
      # drivers_mgr's stored settings) and, worse, silently split a bare string
      # into one entry per CHARACTER, which is where the mismatched lengths and
      # the walk off the end of current_value came from.
      control_value = get_value_list(control_value)

      if isinstance(options, list) == False:
        options = []

      # 'Selections' is a multi-select: how many options are chosen is the value,
      # so its length legitimately differs from the registered one. Every other
      # type has a fixed arity and a mismatch means a malformed update.
      if control_type != 'Selections' and len(control_value) != control_length:
        return value



      if control_type in OPTION_TYPES: ###########################################################

        if control_type == "Menu": ###########################################################
          # The value of a Menu is an INDEX into options. int() of the whole
          # list raised every time, so this only ever reached its own except
          # branch and returned the current value -- a Menu could not be set.
          index = None
          try:
            index = int(float(control_value[0]))
          except Exception as e:
            index = None
          if index is not None and index >= 0 and index < len(options):
            value = [str(index)]
          else:
            value = copy.deepcopy(current_value)

        elif control_type == "Selection": ###########################################################
          # The value of a Selection is one option. str() of the whole list
          # produced "['1920:1080']", which is never in options, so this fell
          # through to options[0] -- meaning a Selection could only ever hold
          # its first option, and IndexError'd outright when the option list
          # was empty, dropping the control at registration.
          selection = None
          try:
            selection = str(control_value[0])
          except Exception as e:
            selection = None
          if selection is not None and selection in options:
            value = [selection]
          elif len(current_value) > 0 and str(current_value[0]) in options:
            value = [str(current_value[0])]
          elif len(options) > 0:
            value = [str(options[0])]
          else:
            # No options to choose from: there is no valid value, and
            # returning None is how the caller is told so.
            value = None

        elif control_type == "Selections": ###########################################################
            values = []
            for item in control_value:
              item = str(item)
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
              # Named round_to, not round: binding the name `round` shadowed the
              # builtin, so round(add_value, round) raised "'int' object is not
              # callable" on the very next line. The except below swallowed it and
              # handed back cur_value, so EVERY Float update silently reverted to
              # the value already held.
              round_to = control_dict['round']
              if round_to >= 0:
                add_value = round(add_value,round_to)
              # Clamps to min_bound. This assigned max_bound, so a value below the
              # minimum came back as the MAXIMUM.
              if float(min_bound) != -999 and add_value < min_bound:
                add_value = min_bound
              if float(max_bound) != -999 and add_value > max_bound:
                add_value = max_bound
            except Exception as e:
              add_value = cur_value


          elif control_type in TRIGGER_TYPES: ###########################################################
              # The stored value is the time the control last fired; get_value
              # reports seconds since, and reads <= 0 as "never fired". A press
              # arrives from the RUI as the non-numeric 'TRIGGER' sentinel, and
              # that is what the current time gets stamped onto. float() of the
              # whole control_value list raised TypeError on every path into
              # here -- including the [0] seed create_controls_dict checks at
              # registration -- so a Button could only ever hold 0.
              try:
                add_value = float(add_value)
              except:
                add_value = nepi_utils.get_time()

          values.append(add_value)
        value = values


  clean_value = None
  if value is not None:
    if len(value) > 0:
      if control_type in SINGLE_TYPES and len(value) > 0:
          clean_value = value[0]
      elif control_type in DOUBLE_TYPES and len(value) > 1:
          clean_value = [value[0],value[1]]
      elif control_type in TRIPLE_TYPES and len(value) > 2:
          clean_value = [value[0],value[1],value[2]]
      else:
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

    # No Button transform here. This accessor returns the value as it is
    # STORED, so that set_value(name, get_value(name)) is a no-op for every
    # type -- which is what _updateControlCb, _updateSettingCb and
    # save_params_dict/init all assume. A Button stores the time it last
    # fired, and reporting seconds-since here made it the one type where a
    # read written straight back replaced the trigger time with an age a few
    # milliseconds from zero. The seconds-since view belongs to the reporting
    # path and already lives there, computed from the raw value in
    # update_status_msg -- which is where the RUI reads it from.

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

      # Validate when asked to validate. The test was inverted, so the default
      # path (check_valid = True) wrote the raw wire value straight into the dict
      # -- handing a driver's setSettingFunction ['False'] for a Toggle and ['5']
      # for an Int -- while a caller passing check_valid = False to SKIP the check
      # got it run. drivers_mgr's discovery pass is that caller.
      if check_valid == True:
        update_value = get_clean_value(controls_dict, control_name, update_value)
      if update_value is not None:
        # Stored as a list of strings, the one shape the dict holds: 'length'
        # counts list entries and get_clean_value re-lists whatever it reads, so
        # a scalar written here comes back out one character per entry.
        controls_dict[control_name]['value'] = get_value_list(update_value)
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
  # Control.msg spells the field display_disabled, so that is the dict key: the
  # dict is built from convert_msg2dict(Control()) and update_status_msg copies
  # only keys the msg carries. Read under 'disabled' this was a KeyError on
  # every control, and written under 'disabled' it never reached the RUI.
  disabled = False
  if control_name in controls_dict.keys():
      disabled = (controls_dict[control_name].get('display_disabled',False) == True)
  return disabled

def set_disabled(controls_dict, control_name, disabled):
  # str() here wrote the strings 'True'/'False' into Control.display_disabled, a
  # bool field. convert_dict2msg then rejected the dict and the control vanished
  # from the status message instead of being disabled in it.
  disabled = (disabled == True)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['display_disabled'] = disabled
  return controls_dict


def cleanDisplayRow(display_row):
  # Control.msg declares display_row a bool, so anything reaching the message
  # has to be one. The string spellings are accepted because params yaml files
  # and hand written init dicts write booleans as 'True'/'true' -- the same
  # test get_clean_value applies to the BOOL_TYPES values.
  return (display_row == True or display_row == 'True' or display_row == 'true')

def get_display_row(controls_dict, control_name):
  """Return True if the control's value widgets should render side by side in one row."""
  # .get rather than [], as in get_disabled: a controls dict built before this
  # field existed does not carry the key, and a missing key means the stacked
  # column layout, not an error.
  display_row = False
  if control_name in controls_dict.keys():
      display_row = cleanDisplayRow(controls_dict[control_name].get('display_row',False))
  return display_row

def set_display_row(controls_dict, control_name, display_row):
  """Set whether the control's value widgets render side by side in one row."""
  display_row = cleanDisplayRow(display_row)
  if control_name in controls_dict.keys():
      controls_dict[control_name]['display_row'] = display_row
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

        # Convert value to a string list for the Control msg. This used to write
        # msg_value/msg_default BACK into the live controls dict, and for the
        # single-value types [str(value)] wrapped a value that was already a
        # one-entry list -- so every status publish re-wrapped it and the dict
        # ended up holding the string "['0']" in place of '0'. The dict the
        # device reads from is not this function's to edit.
        msg_value = get_value_list(control_dict['value'])

        if control_type in TRIGGER_TYPES:
          # A Button holds the time it was last fired; the status reports seconds
          # since, or -999 for never. Comparing the list itself to 0 raised
          # TypeError and left every Button out of the published status.
          fired_at = 0
          try:
            fired_at = float(msg_value[0])
          except Exception as e:
            fired_at = 0
          if fired_at <= 0:
            msg_value = [str(-999)]
          else:
            msg_value = [str(nepi_utils.get_time() - fired_at)]

        msg_dict = nepi_sdk.convert_msg2dict(Control())
        for key in msg_dict.keys():
          if key in control_dict.keys():
            msg_dict[key] = control_dict[key]
        msg_dict['value'] = msg_value
        # Carried by the key loop above like every other display field. The
        # coercion is repeated here because a controls dict assembled by hand
        # never passed through create_controls_dict's normalization, and a
        # string in this bool field makes convert_dict2msg return None -- which
        # takes the whole control out of the status message, not just its
        # layout.
        msg_dict['display_row'] = cleanDisplayRow(control_dict.get('display_row',False))


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

  # An UpdateControl carries only the fields the sender is actually changing;
  # every other field arrives at its ROS default. For the two string[] fields
  # that default is the EMPTY list, not [''], so testing only against ['']
  # treated "field omitted" as "set this field to nothing".
  value = list(msg.value)
  if len(value) > 0 and value != ['']:
    value = get_clean_value(controls_dict, name, value)
    if value is not None:
      controls_dict = set_value(controls_dict, name, value, index = index)

  min_bound = msg.min_bound
  if min_bound != '':
    controls_dict = set_min_bound(controls_dict, name, min_bound = min_bound)

  max_bound = msg.max_bound
  if max_bound != '':
    controls_dict = set_max_bound(controls_dict, name, max_bound = max_bound)

  # Same guard, and this one was doing real damage: the RUI never sends options
  # on a value change, so msg.options arrived as [], which is != [''] -- every
  # update from the RUI wiped the control's option list. The value was applied
  # first and the options cleared right after, so a Selection worked exactly
  # once and then had nothing left to be valid against: get_clean_value returns
  # None for an option type with no options, so the dropdown stopped taking
  # changes and the caller read back None.
  options = list(msg.options)
  if len(options) > 0 and options != ['']:
    controls_dict = set_options(controls_dict, name, options)

  return controls_dict