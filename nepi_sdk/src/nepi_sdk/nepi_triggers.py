#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI System Trigger utility functions


  
import os
import time

from nepi_ros_interfaces.msg import SystemTrigger, SystemTriggersStatus
from nepi_ros_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryResponse

from nepi_sdk import nepi_utils

from nepi_sdk.nepi_ros import sec_from_ros_time, ros_time_now
from nepi_sdk.nepi_ros import find_topics_by_msg

from nepi_sdk.nepi_ros import logger as Logger
log_name = "nepi_triggers"
logger = Logger(log_name = log_name)

EXAMPLE_TRIGGER_DICT = {"name":"None",
                        "description": "None",
                        "data_str_list":["None"],
                        "time": nepi_utils.get_time(),
                        "node": "None"
}

EXAMPLE_TRIGGERS_DICT = {"None":EXAMPLE_TRIGGER_DICT}

#########################
### System Triggers Helper Functions



def get_triggers_publisher_namespaces(self):
    topics_list = find_topics_by_msg(SystemTrigger)
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic))
        return namespaces_list

def parse_trigger_msg(trigger_msg):
    trigger_dict = dict()
    trigger_dict['name'] = trigger_msg.name
    trigger_dict['time'] = sec_from_ros_time(trigger_msg.header.stamp)
    trigger_dict['description'] = trigger_msg.description
    trigger_dict['node'] = trigger_msg.node
    trigger_dict['data_str_list'] = trigger_msg.data_str_list
    return trigger_dict



def create_trigger_msg_from_trigger_dict(node_name, trigger_dict):
    trigger_msg = SystemTrigger()
    trigger_msg.header = Header()
    trigger_msg.header.stamp = ros_time_now()
    trigger_msg.name = trigger_dict['name']
    trigger_msg.description = trigger_dict['description']
    trigger_msg.data_str_list = trigger_dict['data_str_list']
    trigger_msg.node = trigger_dict['node']
    return trigger_msg




       
def parse_triggers_query_resp(triggers_query_resp):
  node_name = triggers_query_resp.node_name
  triggers = triggers_query_resp.triggers_list
  triggers_dict = dict()
  names_list = []
  for entry in triggers:
    name = entry.name
    num = 0
    while name in names_list:
      num += 1
      name = name + "_" + str(num)
    names_list.append(name)
    trigger_dict = dict()
    trigger_dict['name'] = name
    trigger_dict['description'] = entry.description
    trigger_dict['node'] = node_name
    trigger_dict['data_str_list'] = trigger_msg.data_str_list
    triggers_dict[entry.name] = trigger_dict
  return triggers_dict


def create_query_resp_from_triggers_dict(node_name, triggers_dict):
  triggers_query_resp = SystemTriggersQueryResponse()
  triggers_list = []
  for trigger_name in triggers_dict.keys():
    trigger_dict = triggers_dict[trigger_name]
    trigger_msg = create_trigger_msg_from_trigger_dict(trigger_dict)
    triggers_list.append(trigger_msg)
  triggers_query_resp.node_name = node_name
  triggers_query_resp.triggers_list = triggers_list
  return triggers_query_resp

def parse_triggers_status_msg(triggers_status_msg):
  trigger_names = triggers_status_msg.triggers_names_list
  trigger_nodes = triggers_status_msg.triggers_nodes_list
  triggers = triggers_status_msg.triggers_list
  triggers_dict = dict()
  names_list = []
  for i, name in enumerate(trigger_names):
    num = 0
    while name in  names_list:
      num += 1
      name = name + "_" + str(num)
    names_list.append(name)
    trigger_dict = dict()
    trigger_dict['node'] = triggers_nodes[i]
    entry = triggers[i]
    trigger_dict['name'] = entry.name
    trigger_dict['description'] = entry.description
    trigger_dict['data_str_list'] = trigger_msg.data_str_list
    triggers_dict[entry.name] = trigger_dict
  return triggers_dict
  



def create_status_msg_from_triggers_dict(node_name, triggers_dict):
  triggers_status_msg = SystemTriggersStatus()
  names_list = []
  nodes_list = []
  triggers_list = []
  for trigger_name in triggers_dict.keys():
    trigger_dict = triggers_dict[trigger_name]
    name = trigger_dict['name']
    names_list.append(name)
    node = node_name
    nodes_list.append(node)

    trigger_msg = create_trigger_msg_from_trigger_dict(trigger_dict)
    triggers_list.append(trigger_msg)

  triggers_status_msg.triggers_names_list = names_list
  triggers_status_msg.triggers_nodes_list = nodes_list
  triggers_status_msg.triggers_list = triggers_list
  return triggers_status_msg









