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

import copy
import math

import numpy as np
import cv2

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
        update_dict['data_dict'] = process_dict['data_dict']
        update_dict['controls_dict'] = nepi_controls.create_controls_dict(process_dict['controls_dict'])
        update_dict['results_dict'] = nepi_data.create_data_dict(process_dict['results_dict'])
        processes_dict[process_name] = update_dict
    except:
        pass
    return processes_dict


def get_process_dicts(processes_dict, process_name):
    [data_dict,controls_dict,results_dict] = [None,None,None]
    if process_name in processes_dict.keys():
        data_dict = processes_dict[process_name]['data_dict']
        controls_dict = processes_dict[process_name]['controls_dict']
        results_dict = processes_dict[process_name]['results_dict']
    return data_dict,controls_dict,results_dict



def convert_pub_results_dict2msg(results_pub_msg, results_pub_dict):
    results_msg = None
    if results_pub_msg is not None and results_pub_dict is not None:
            results_dict = nepi_sdk.convert_msg2dict(results_pub_msg())

            for result_name in results_dict.keys():
                if result_name in results_pub_dict.keys():
                    results_dict[result_name] = results_pub_dict[result_name]
            results_msg = nepi_sdk.convert_dict2msg(results_pub_msg(), results_dict)
 
    return results_msg


