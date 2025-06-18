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

from nepi_interfaces.msg import SystemTrigger, SystemTriggersStatus
from nepi_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryResponse

from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import sec_from_msg_stamp, get_msg_stamp
from nepi_sdk.nepi_sdk import find_topics_by_msg

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_triggers"
logger = Logger(log_name = log_name)

EXAMPLE_TRIGGER_DICT = {"name":"None",
                        "description": "None",
                        "data_str_list":["None"],
                        "time": nepi_utils.get_time(),
                        "node_name": "None"
}

EXAMPLE_TRIGGERS_DICT = {"None":EXAMPLE_TRIGGER_DICT}

#########################
### System Triggers Helper Functions



def get_triggers_publisher_namespaces():
    topics_list = find_topics_by_msg(SystemTrigger)
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic))
        return namespaces_list

def parse_trigger_msg(trigger_msg):
    trigger_dict = dict()
    trigger_dict['name'] = trigger_msg.name

    trigger_dict['time'] = sec_from_msg_stamp(trigger_msg.header.stamp)
    trigger_dict['description'] = trigger_msg.description
    trigger_dict['node_name'] = trigger_msg.node_name
    trigger_dict['data_str_list'] = trigger_msg.data_str_list
    return trigger_dict



def create_trigger_msg(node_name, trigger_dict):
    trigger_msg = SystemTrigger()
    trigger_msg.header.stamp = get_msg_stamp()
    trigger_msg.name = trigger_dict['name']
    trigger_msg.description = trigger_dict['description']
    trigger_msg.data_str_list = trigger_dict['data_str_list']
    trigger_msg.node_name = trigger_dict['node_name']
    return trigger_msg


def parse_triggers_query_resp(triggers_query_resp):
  triggers = triggers_query_resp.triggers_list
  triggers_dict = dict()
  for trigger in triggers:
    triggers_dict[trigger.name] = nepi_sdk.convert_msg2dict(trigger)
  return triggers_dict


def create_query_resp(triggers_dict):
  triggers_query_resp = SystemTriggersQueryResponse()
  triggers_list = []
  for trigger_name in triggers_dict.keys():
    trigger_dict = triggers_dict[trigger_name]
    trigger_msg = create_trigger_msg_from_trigger_dict(trigger_dict)
    triggers_list.append(trigger_msg)
  triggers_query_resp.triggers_list = triggers_list
  return triggers_query_resp

def parse_triggers_status_msg(msg):
  return msg.triggers_name_list, msg.has_triggered_list
  

def create_triggers_status_msg(node_name, triggers_name_list, has_triggered_list):
  triggers_status_msg = SystemTriggersStatus()
  if len(triggers_name_list) == len(has_triggered_list):
    triggers_status_msg.triggers_names_list = triggers_names_list
    triggers_status_msg.has_triggered_list = has_triggered_list
  else:
    self.logger.log_error("triggers_name_list and has_triggered_list different lengths")
  return triggers_status_msg









