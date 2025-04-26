#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI System States utility functions 

  
import os
import rospy
import time
import copy

from nepi_ros_interfaces.msg import SystemState, SystemStatesStatus
from nepi_ros_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_sdk.nepi_ros import find_topics_by_msg

from nepi_sdk.nepi_ros import logger as Logger
log_name = "nepi_states"
logger = Logger(log_name = log_name)

#########################
### System States Helper Functions



STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]
NONE_STATES_DICT = {"state_name":{"name":"state_name","type":"Int","optons":[],"value":"20"}}


def get_states_publisher_namespaces():
    topics_list = find_topics_by_msg(SystemState)
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic))
    return namespaces_list


def create_state_msg(node_name, state_dict):
    state_msg = SystemState()
    state_msg.name = state_dict['name']
    state_msg.node_name = state_dict['node_name']
    state_msg.description = state_dict['description']
    state_msg.type_str = state_dict['type']
    state_msg.value_str = state_dict['value']
    if 'options' in state_dict.keys():
      state_msg.options_list = state_dict['options']
    else:
      state_msg.options_list = []
    state_msgs_list.append(state_msg)
    return state_msg

def parse_state_msg(msg):
    state_dict = nepi_ros.convert_msg2dict(msg)
    return state_dict

def get_data_from_state_dict(state_dict):
  s_str = str(state_dict)
  s_name = state_dict['name']
  s_type = state_dict['type']
  s_value = state_dict['value']
  data = None
  if s_type != None and s_value != None:

    s_value = state_dict['value']
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
        data = int(s_value.split(":")['name'])
    except Exception as e:
      logger.log_info("Data conversion failed for state_dict " + s_str + "with exception" + str(e) )
  return s_name, s_type, data

def create_states_query_resp(states_dict):
  states_query_resp = SystemStatesQueryResponse()
  states_list = []
  for state_name in states_dict.keys():
    state_dict = states_dict[state_name]
    state_msg = create_state_msg(state_dict)
    states_list.append(state_msg)
  states_query_resp.states_list = states_list
  return states_query_resp       

def parse_states_query_resp(states_query_resp):
  states_dict = dict()
  states = states_query_resp.states_list
  for state in states:
    states_dict = nepi_ros.convert_msg2dict(state)
    states_dict[state.name] = state_dict
  return states_dict


def create_states_status_msg(states_list):
  states_status_msg = SystemStatesStatus()
  states_status_msg.states_list = states_list
  return states_status_msg


def parse_states_status_msg(msg):
  states = msg.states_list
  states_list = []
  for state in states:
    state_dict = nepi_ros.convert_msg2dict(state)
    states_list.append(state_dict)
  return states_list
  





