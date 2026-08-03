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


# NEPI System Trigger utility functions


  
import os
import time

from nepi_interfaces.msg import SystemTrigger

from nepi_sdk import nepi_sdk
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



def get_triggers_publisher_namespaces(topics_list = None, types_list = None):
    namespace = []
    namespaces = nepi_sdk.find_topics_by_msg('SystemTrigger', topics_list = topics_list, types_list = types_list)
    for i, namespace in enumerate(namespaces):
        namespaces[i] = os.path.dirname(namespaces[i])
    return namespaces 

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









