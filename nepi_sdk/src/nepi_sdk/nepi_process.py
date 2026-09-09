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
import math

import numpy as np


from nepi_sdk import nepi_utils
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_data
from nepi_sdk import nepi_controls


from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_obstacles"
logger = Logger(log_name = log_name)


########################
## Process IF Functions  


def update_processes_dict(processes_dict, process_name, process_dict):
    try:
        update_dict = dict()
        update_dict['if_dict'] = process_dict.get('if_dict', dict())
        update_dict['data_dict'] = process_dict.get('data_dict', dict())
        update_dict['controls_dict'] = nepi_controls.create_controls_dict(process_dict.get('controls_dict', dict()))
        update_dict['results_dict'] = nepi_data.create_data_dict(process_dict.get('results_dict', dict()))
        processes_dict[process_name] = update_dict
    except:
        pass
    return processes_dict


def get_process_dicts(processes_dict, process_name):
    if_dict = dict()
    data_dict = dict()
    controls_dict = dict()
    results_dict = dict()
    states_dict = dict()

    if process_name in processes_dict.keys():
        try:
            if_dict = processes_dict[process_name].get('if_dict', dict())
        except Exception as e:
            logger.log_warn("Failed to get dict from proccesses dict " + str(e)) 

        try:
            data_dict = processes_dict[process_name].get('data_dict', dict())
        except Exception as e:
            logger.log_warn("Failed to get dict from proccesses dict " + str(e)) 

        try:
           controls_dict = processes_dict[process_name].get('controls_dict', dict())
        except Exception as e:
            logger.log_warn("Failed to get dict from proccesses dict " + str(e)) 

        try:
            results_dict = processes_dict[process_name].get('results_dict', dict())
        except Exception as e:
            logger.log_warn("Failed to get dict from proccesses dict " + str(e)) 

        try:
             states_dict = processes_dict[process_name].get('results_dict', dict())
        except Exception as e:
            logger.log_warn("Failed to get dict from proccesses dict " + str(e)) 

        
    return if_dict,data_dict,controls_dict,results_dict,states_dict


def get_available_source_topics(msg_type, name_filters = [], topics_list = None, types_list = None):
    topics = []

    if msg_type is None:
        return topics

    if name_filters is None:
        name_filters = []

    topics = nepi_sdk.find_topics_by_msg('TargetingStatus', topics_list = topics_list, types_list = types_list)
    for i, topic in enumerate(topics):
        valid = True
        if len(name_filters) > 0:
            valid = False
            for filter in name_filters:
                if topics[i].index(filter) != -1:
                    valid = True
                    break
        if valid == True:
            topics[i] = os.path.dirname(topics[i])
    return topics 



def convert_results_pub_dict2msg( msg, msg_type, results_pub_dict):

    results_msg = None
    if msg is not None and msg_type is not None and results_pub_dict is not None:            
            results_dict = nepi_sdk.convert_msg2dict(msg())

            for result_name in results_dict.keys():
                if result_name in results_pub_dict.keys():
                    results_dict[result_name] = results_pub_dict[result_name]
            results_msg = nepi_sdk.convert_dict2msg(msg_type, results_dict)
    return results_msg


def convert_results_pub_msg2dict(self, results_msg):
    results_dict = nepi_sdk.convert_msg2dict(results_msg)
    return results_dict
