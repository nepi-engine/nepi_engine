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


# NEPI System States utility functions 

  
import os
import time
import copy

from nepi_sdk import nepi_sdk

from nepi_interfaces.msg import SystemState, SystemStates, SystemStatesStatus
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_sdk.nepi_sdk import find_topics_by_msg

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_states"
logger = Logger(log_name = log_name)

#########################
### System States Helper Functions



STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]
NONE_STATES_DICT = {"state_name":{"name":"state_name","type":"Int","optons":[],"value":"20"}}


def get_states_publisher_namespaces(topics_list = None, types_list = None):
    namespace = []
    namespaces = nepi_sdk.find_topics_by_msg('SystemStatesStatus', topics_list = topics_list, types_list = types_list)
    for i, namespace in enumerate(namespaces):
        namespaces[i] = os.path.dirname(namespaces[i])
    return namespaces 



def create_state_msg(name, state):
    state_msg = SystemState()
    state_msg.name = name
    state_msg.state = state
    return state_msg


def create_states_msg(states_dict):
  states_status_msg = SystemStates()
  state_names = []
  states_msg_list = []
  for state_name in states_dict.keys():
      state_msg = SystemState()
      state_msg.name = state_name
      state_msg.state = states_dict[state_name]
      state_names.append(state_name)
      states_msg_list.append(state_msg)
  states_status_msg.state_names = state_names
  states_status_msg.states_msg_list = states_msg_list
  return states_status_msg


  states_status_msg.states_list = states_list
  return states_status_msg

def parse_states_msg(msg):
    state_dict = nepi_sdk.convert_msg2dict(msg)
    return state_dict


  





