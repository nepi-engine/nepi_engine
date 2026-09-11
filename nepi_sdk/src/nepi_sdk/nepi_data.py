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

from nepi_interfaces.msg import Datum, DataStatus



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_data"
logger = Logger(log_name = log_name)


#########################
### Data Helper Functions





DATUM_TYPES = ["Trigger", "Triggers", "Bool", "Bools", 
                 "String", 
                 "Int","Ints",
                 "Float","Floats",
                  "ColorRGB"]


SINGLE_TYPES = ["Trigger","Bool", 
                 "String",
                 "Int",
                 "Float"]

DOUBLE_TYPES = []

TRIPLE_TYPES = ["ColorRGB"]

LIST_TYPES = ["Triggers", "Bools", 
                "Ints", "Floats",
                 "ColorRGB"]


BOUND_TYPES = ["Int","Ints",
                 "Float","Floats",
                  "ColorRGB"]

STRING_TYPES = ["String"]
BOOL_TYPES = ["Bool","Bools"]
INT_TYPES = ["Int","Ints","ColorRGB"]
FLOAT_TYPES = ["Float","Floats"]
TRIGGER_TYPES = ['Trigger','Triggers']

BLANK_DATUM_DICT = nepi_sdk.convert_msg2dict(Datum())

BLANK_CNTROLS_DICT = dict()




EXAMPLE_INIT_DICT = {
        'demo_bool_data': {
            'type': 'Bool', 'value': True,
            'display_name': 'Demo Bool', 'description': 'A boolean that toggles every update.', 'display_hidden': False},

        'demo_bools_data': {
            'type': 'Bools', 'value': [True, False],
            'display_name': 'Demo Bools', 'description': 'Two booleans, always opposite.', 'display_hidden': False},

        'demo_string_data': {
            'type': 'String', 'value': 'starting',
            'display_name': 'Demo String', 'description': 'A wall-clock timestamp string.', 'display_hidden': False},


        'demo_int_data': {
            'type': 'Int', 'value': 0,
            'display_name': 'Demo Int', 'description': 'A monotonic update counter.', 'display_hidden': False},

        'demo_ints_data': {
            'type': 'Ints', 'value': [0, 0],
            'display_name': 'Demo Ints', 'description': 'The counter and its negation.', 'display_hidden': False},

        'demo_float_data': {
            'type': 'Float', 'value': 0.0, 'round_value': 3, 'round_display': 3,
            'display_name': 'Demo Float', 'description': 'A sine wave over the update counter.', 'display_hidden': False},

        'demo_floats_data': {
            'type': 'Floats', 'value': [0.0, 0.0], 'round_value': 3, 'round_display': 3,
            'display_name': 'Demo Floats', 'description': 'The sine wave and its negation.', 'display_hidden': False},
    }



def get_publisher_namespaces(topics_list = None, types_list = None):
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

  for i, name in enumerate(names):
    try:
      init_datum_dict = init_dict[name]
      input_type = init_datum_dict['type']
      if input_type == 'Discrete':
        input_type = 'Selection'
      if input_type in DATUM_TYPES:
        datum_dict = copy.deepcopy(BLANK_DATUM_DICT)
        datum_dict['type'] = input_type
        datum_dict['description'] = name
        datum_dict['round'] = 6
        datum_dict['default'] = []
        datum_dict['length'] = 0
        datum_dict['min_bound'] = -999
        datum_dict['max_bound'] = -999
        datum_dict['display_name'] = name
        datum_dict['display_round'] = 2
        datum_dict['display_row'] = False
        for key in datum_dict.keys():
          if key in init_datum_dict.keys():
            datum_dict[key] = init_datum_dict[key]

        #############
        # Clean Name
        datum_dict['name'] = nepi_utils.get_clean_name(name)
        if  datum_dict['name'] == '':
           datum_dict['name'] = 'datum' + str(i)


        #############
        # Clean Name
        datum_type = datum_dict['name']
        if datum_type == 'Discrete':
          datum_type = 'Selection'
        datum_type = datum_type.replace('Trigger','Trigger')
        datum_dict['name'] = datum_type


        #############
        # Clean Rounds
        #############
        if datum_dict['round'] < 0 or datum_dict['round'] > 6:
          datum_dict['round'] = 6
        if datum_dict['display_round'] < 0:
          datum_dict['display_round'] = 0
        if datum_dict['display_round'] > 6:
          datum_dict['display_round'] = 6

        #############
        # Clean Display Row
        #############
        # The overlay loop above copies the caller's value verbatim, so a hand
        # written init dict -- or a params yaml, which spells booleans 'True' --
        # can put a string or an int in what Datum.msg declares a bool.
        # convert_dict2msg rejects the whole dict on a type mismatch, which
        # drops the datum from the published status entirely rather than just
        # mis-rendering it. Same failure mode set_hidden and set_disabled coerce
        # against.
        datum_dict['display_row'] = cleanDisplayRow(datum_dict['display_row'])

        #############
        # Clean Bounds
        #############
        # Clean Bounds
        min_bound = -999
        max_bound = -999


        # Membership test, not equality against the name of the list. As an
        # equality test this was never true, so min_bound/max_bound stayed at
        # the -999 sentinel for every Int and Float datum and a device's
        # reported bounds (v4l2 hands them over as init_datum_dict['bounds'])
        # were discarded.
        if input_type in BOUND_TYPES:
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

        datum_dict['min_bound'] = min_bound
        datum_dict['max_bound'] = max_bound

        #############
        # Clean Value
        value = None
        # Membership in the list, not a substring test against its name. As a
        # substring test this was never true, so a Trigger never got its [0]
        # seed: it fell through to the default branch, came out length 0, and
        # was dropped as invalid below.
        if input_type in TRIGGER_TYPES:
          value = [0]
        else:
          value  = datum_dict['default']
          if value == []:
            value = datum_dict['value']

          if value is not None:
            if isinstance(value, list) == False:
                values = [str(value)]
            else:
              values = [str(item) for item in value]
            value = values
        if value is None or isinstance(value, list) == False:
          # Third drop path, and it was the last silent one: no exception to
          # catch, so nothing was logged.
          logger.log_warn("create_data_dict: dropped datum '" + str(name) +
                          "' of declared type '" + str(input_type) +
                          "': no default or value to seed it with")
          continue

        datum_dict['value'] = value
        datum_dict['length'] = len(value)



        #############
        # Clean Display Name
        #############
        if datum_dict['display_name'] is None or datum_dict['display_name'] == 'None':
          datum_dict['display_name'] = ''
        if datum_type in LIST_TYPES and datum_dict['display_name'] == '':
          if isinstance(value, list):
              pass
          else:
              datum_dict['display_name'] = name


        #############
        # Clean Labels

    

        if input_type == 'ColorRGB':
              display_labels = ['R','G','B']

        elif input_type in SINGLE_TYPES:
              display_labels = [datum_dict['display_name']]
        else:
          for i, entry in enumerate(value):
            if len(display_labels) <= i:
              display_labels.append('datum_' + str(i))

        display_labels = [str(item) for item in display_labels]


        datum_dict['display_labels'] = display_labels


        #############
        # Check Valid Value
        #############

        check_dict = dict()
        # Keyed by the cleaned name, which is what get_clean_value looks up --
        # it cleans the name before indexing, so a raw key it could not find
        # came back None and the datum was dropped for no stated reason.
        check_name = datum_dict['name']
        check_dict[check_name] = copy.deepcopy(datum_dict)

        check_value = copy.deepcopy(value)
        clean_value = get_clean_value(check_dict, check_name, check_value)
        #logger.log_warn("Got clean value from check value: " + str(name) + ": " + str(clean_value) + ": " + str(check_value))
        if clean_value is None:
          logger.log_warn("create_data_dict: dropped datum '" + str(name) +
                          "' of declared type '" + str(input_type) +
                          "': value " + str(check_value) + " is not valid for the datum")
          continue

        # Store the LIST form of the cleaned value. This took len() of
        # get_clean_value's return, which is the NATIVE form: len(3) and
        # len(True) raise TypeError, so every Int and Bool was dropped, and a
        # String's length became its character count -- len('/dev/ttyUSB0') is
        # 12 against a one-entry value list -- which is what walked
        # get_clean_value's range(datum_length) off the end of current_value
        # and killed drivers_mgr from its set_value call.
        value = get_value_list(clean_value)
        datum_dict['value'] = value
        datum_dict['default'] = value
        datum_dict['length'] = len(value)




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

def get_value_list(value):
  # The data dict stores every value as a list of strings: 'length' counts
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


def get_clean_value(data_dict, datum_name, datum_value = None):
  # If datum_name not in data_dict keys, None is returned
  # If datum_value is None or any datum_value is invalid, current valid values are returned
  valid = False
  value = None
  datum_name = nepi_utils.get_clean_name(datum_name)
  if datum_name != '' and datum_name in data_dict.keys():
      datum_dict = data_dict[datum_name]
      current_value = datum_dict['value']
      datum_type = datum_dict['type']
      datum_length = datum_dict['length']
      min_bound = datum_dict['min_bound']
      max_bound = datum_dict['max_bound']

      if datum_value is None:
        try:
          datum_value = copy.deepcopy(current_value)
        except:
          pass
      if datum_value is None:
        return value

      # Callers hand this the NATIVE value -- set_value from a driver, the
      # DataIF/SettingsIF wrappers, apply_update_msg. Iterating that
      # directly raised TypeError on an int (SettingsIF.init died here on
      # drivers_mgr's stored settings) and, worse, silently split a bare string
      # into one entry per CHARACTER, which is where the mismatched lengths and
      # the walk off the end of current_value came from.
      datum_value = get_value_list(datum_value)

  

      if datum_type == "ColorRGB": ###########################################################      
        if len(current_value) != 3:
          current_value = [255,255,255]
        new_value = []
        try:
          datum_value = list(datum_value)
          for i, val in enumerate(datum_value):
            try:
              val = int(val)
              if 0 <= val <= 255:
                new_value.append(val)
            except:
              pass
        except Exception as e:
            datum_value = []

        if len(new_value) == 3:
          value = new_value
        else:
          value = current_value


      else:
        values = []
        for i in range(datum_length):
          cur_value = current_value[i]
          add_value = copy.deepcopy(cur_value)
          if len(datum_value) > i:
            add_value = datum_value[i]

          if datum_type in STRING_TYPES: ###########################################################
            add_value = str(add_value)


          elif datum_type in BOOL_TYPES: ###########################################################
              try:
                  add_value  = (add_value == True or add_value == 'True' or add_value == 'true')
              except Exception as e:
                  pass

          elif datum_type in INT_TYPES:  ###########################################################
            try:
              add_value = int(float(add_value))
              if int(float(min_bound)) != -999 and add_value < min_bound:
                add_value = min_bound
              if int(float(max_bound)) != -999 and add_value > max_bound:
                add_value = max_bound
            except Exception as e:
              add_value = cur_value


          elif datum_type in FLOAT_TYPES: ###########################################################

            try:
              add_value  = float(add_value)
              # Named round_to, not round: binding the name `round` shadowed the
              # builtin, so round(add_value, round) raised "'int' object is not
              # callable" on the very next line. The except below swallowed it and
              # handed back cur_value, so EVERY Float update silently reverted to
              # the value already held.
              round_to = datum_dict['round']
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


          elif datum_type in TRIGGER_TYPES: ###########################################################
              # The stored value is the time the datum last fired; get_value
              # reports seconds since, and reads <= 0 as "never fired". float()
              # of the whole datum_value list raised TypeError on every path
              # into here, so a Trigger could only ever hold 0.
              try:
                add_value = float(add_value)
              except:
                add_value = nepi_utils.get_time()

          values.append(add_value)
        value = values


  clean_value = None
  if value is not None:

      if datum_type in SINGLE_TYPES and len(value) > 0:
          clean_value = value[0]
      elif datum_type in DOUBLE_TYPES and len(value) > 1:
          clean_value = [value[0],value[1]]
      elif datum_type in TRIPLE_TYPES and len(value) > 2:
          clean_value = [value[0],value[1],value[2]]
      elif datum_type in LIST_TYPES:
          clean_value = value
        
  return clean_value


def get_value(data_dict, datum_name, index = None):
  value = None
  datum_type = None
  if data_dict is not None:
    if datum_name in data_dict.keys():
        try:
          datum_type = data_dict[datum_name]['type']
          datum_value = get_clean_value(data_dict, datum_name)
          if datum_value is None:
              logger.log_warn("Got None Value for datum: " + str([datum_name, data_dict[datum_name]]))
              pass
          else:
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
          pass

    ###################
    # Special Types Support
    if value is not None and datum_type == 'ColorRGB':
      try:
        value = tuple(value)
      except:
        value = None

    # No Trigger transform here, matching nepi_controls.get_value: this
    # accessor returns the value as it is STORED, so a read written straight
    # back is a no-op for every type. The seconds-since view belongs to the
    # reporting path and already lives there, computed from the raw value in
    # update_status_msg.

  return value

def get_values_dict(data_dict):
  data_values_dict = dict()
  if data_dict is not None:
    for datum_name in data_dict.keys():
      datum_value = get_value(data_dict, datum_name)
      if datum_value is not None:
        data_values_dict[datum_name] = datum_value
      else:
        #logger.log_warn("Got None Value for datum: " + str(datum_name))
        pass
  return data_values_dict


def get_params_dict(data_dict):
  data_values_dict = dict()
  if data_dict is not None:
    for datum_name in data_dict.keys():
      datum_value = get_value(data_dict, datum_name)
      param = data_dict[datum_name].get('param',True)
      if datum_value is not None and param == True:
        data_values_dict[datum_name] = datum_value
      else:
        #logger.log_warn("Got None Value for datum: " + str(datum_name))
        pass
  return data_values_dict

def set_value(data_dict, datum_name, update_value, index = None,  check_valid = True):
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

      # Validate when asked to validate. The test was inverted, so the default
      # path (check_valid = True) wrote the raw wire value straight into the dict
      # -- handing a driver's setSettingFunction ['False'] for a Bool and ['5']
      # for an Int -- while a caller passing check_valid = False to SKIP the check
      # got it run. drivers_mgr's discovery pass is that caller.
      if check_valid == True:
        update_value = get_clean_value(data_dict, datum_name, update_value)
      if update_value is not None:
        # Stored as a list of strings, the one shape the dict holds: 'length'
        # counts list entries and get_clean_value re-lists whatever it reads, so
        # a scalar written here comes back out one character per entry.
        data_dict[datum_name]['value'] = get_value_list(update_value)
  return data_dict

def sets_values(data_dict, data_values_dict):
  data_values_dict = dict()
  for datum_name in data_values_dict.keys():
     datum_value = data_values_dict[datum_name]
     data_dict = set_value(data_dict, datum_name, datum_value)
  return data_dict


def reset_value(data_dict, datum_name):
  data_dict[datum_name]['value'] = data_dict[datum_name]['default']
  return data_dict

def reset_values(data_dict):
    datum_names = list(data_dict.keys())
    for datum_name in datum_names:
      data_dict = reset_value(data_dict, datum_name)
    return data_dict


def get_display_labels(data_dict, datum_name):
  display_labels = []
  if datum_name in data_dict.keys():
      display_labels = data_dict[datum_name].get('display_labels',[])
  return display_labels


def set_display_labels(data_dict, datum_name, display_labels):
  display_labels = [str(item) for item in display_labels]
  if datum_name in data_dict.keys():
      data_dict[datum_name]['display_labels'] = display_labels
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
  display_hidden = False
  if datum_name in data_dict.keys():
      display_hidden = (data_dict[datum_name]['display_hidden'] == True)
  return display_hidden

def set_hidden(data_dict, datum_name, display_hidden):
  # str() here wrote the strings 'True'/'False' into Datum.display_hidden, a bool
  # field. convert_dict2msg then rejected the dict and the datum vanished
  # from the status message instead of being display_hidden in it.
  display_hidden = (display_hidden == True)
  if datum_name in data_dict.keys():
      data_dict[datum_name]['display_hidden'] = display_hidden
  return data_dict


def cleanDisplayRow(display_row):
  # Datum.msg declares display_row a bool, so anything reaching the message
  # has to be one. The string spellings are accepted because params yaml files
  # and hand written init dicts write booleans as 'True'/'true' -- the same
  # test get_clean_value applies to the BOOL_TYPES values.
  return (display_row == True or display_row == 'True' or display_row == 'true')

def get_display_row(data_dict, datum_name):
  """Return True if the datum's value widgets should render side by side in one row."""
  # .get rather than [], as in get_disabled: a data dict built before this
  # field existed does not carry the key, and a missing key means the stacked
  # column layout, not an error.
  display_row = False
  if datum_name in data_dict.keys():
      display_row = cleanDisplayRow(data_dict[datum_name].get('display_row',False))
  return display_row

def set_display_row(data_dict, datum_name, display_row):
  """Set whether the datum's value widgets render side by side in one row."""
  display_row = cleanDisplayRow(display_row)
  if datum_name in data_dict.keys():
      data_dict[datum_name]['display_row'] = display_row
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

def create_status_msg( name = '', display_name = '', description = ''):
  status_msg = DataStatus()
  name = nepi_utils.get_clean_name(str(name))
  status_msg.name= name
  if display_name == '':
    display_name = name
  status_msg.display_name= str(display_name)
  if description == '':
    description = name
  status_msg.description= str(description)
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

        # Convert value to a string list for the Datum msg. This used to write
        # msg_value/msg_default BACK into the live data dict, and for the
        # single-value types [str(value)] wrapped a value that was already a
        # one-entry list -- so every status publish re-wrapped it and the dict
        # ended up holding the string "['0']" in place of '0'. The dict the
        # device reads from is not this function's to edit.
        msg_value = get_value_list(datum_dict['value'])

        if datum_type in TRIGGER_TYPES:
          # A Trigger holds the time it was last fired; the status reports seconds
          # since, or -999 for never. Comparing the list itself to 0 raised
          # TypeError and left every Trigger out of the published status.
          fired_at = 0
          try:
            fired_at = float(msg_value[0])
          except Exception as e:
            fired_at = 0
          if fired_at <= 0:
            msg_value = [str(-999)]
          else:
            msg_value = [str(nepi_utils.get_time() - fired_at)]

        msg_dict = nepi_sdk.convert_msg2dict(Datum())
        for key in msg_dict.keys():
          if key in datum_dict.keys():
            msg_dict[key] = datum_dict[key]
        msg_dict['value'] = msg_value
        # Carried by the key loop above like every other display field. The
        # coercion is repeated here because a data dict assembled by hand
        # never passed through create_data_dict's normalization, and a
        # string in this bool field makes convert_dict2msg return None -- which
        # takes the whole datum out of the status message, not just its
        # layout.
        msg_dict['display_row'] = cleanDisplayRow(datum_dict.get('display_row',False))


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
