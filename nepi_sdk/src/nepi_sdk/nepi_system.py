#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#





from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_drvs"
logger = Logger(log_name = log_name)



#######################
## Get System Data Functions
def check_debug_enabled(timeout = float('inf'), log_name_list = []):
    param_namespace = 'debug_enabled'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_user_folders(timeout = float('inf'), log_name_list = []):
    param_namespace = 'user_folders'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_system_folders(timeout = float('inf'), log_name_list = []):
    param_namespace = 'system_folders'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_config_folders(timeout = float('inf'), log_name_list = []):
    param_namespace = 'config_folders'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def check_container(timeout = float('inf'), log_name_list = []):
    param_namespace = 'in_container'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_navpose_frames(timeout = float('inf'), log_name_list = []):
    param_namespace = 'navpose_frames'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_timezone(timeout = float('inf'), log_name_list = []):
    param_namespace = 'timezone'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_drivers(timeout = float('inf'), log_name_list = []):
    param_namespace = 'active_drivers'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

''' Need to add
def get_active_ai_frameworks(timeout = float('inf'), log_name_list = []):
    param_namespace = 'active_ai_frameworks'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_ai_models(timeout = float('inf'), log_name_list = []):
    param_namespace = 'active_ai_models'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_apps(timeout = float('inf'), log_name_list = []):
    param_namespace = 'active_apps'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_auto_scrips(timeout = float('inf'), log_name_list = []):
    param_namespace = 'active_auto_scrips'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data
'''
