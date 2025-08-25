#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#



import os

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_system"
logger = Logger(log_name = log_name)


# Setup System Variables
BASE_NAMESPACE = nepi_sdk.get_base_namespace()

# Try and read bash set variables
'''
NEPI_CONFIG=None
NEPI_CONFIG_PATH='/home/nepi/.nepi_config'
try:
    NEPI_CONFIG = nepi_utils.read_sh_variables(NEPI_CONFIG_PATH)
except:
    pass
logger.log_warn("Got NEPI CONFIG: " +  str(NEPI_CONFIG))
'''




#######################
## Get System Data Functions
def get_base_namespace():
    return BASE_NAMESPACE

##########################

def get_hw_type(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'hw_type')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_hw_type(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'hw_type')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_hw_model(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'hw_model')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_hw_model(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'hw_model')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success


def get_in_container(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'in_container')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_in_container(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'in_container')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_has_cuda(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'has_cuda')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_has_cuda(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'has_cuda')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success


##########################

def get_manages_ssh(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'manages_ssh')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_manages_ssh(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'manages_ssh')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success


def get_manages_share(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'manages_share')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_manages_share(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'manages_share')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_manages_time(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'manages_time')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_manages_time(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'manages_time')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_manages_network(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'manages_network')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_manages_network(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'manages_network')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_debug_mode(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'debug_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_debug_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'debug_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_admin_mode(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'admin_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_admin_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'admin_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_admin_restricted(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'admin_restricted')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_admin_restricted(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'admin_restricted')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################



def get_user_folders(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'user_folders')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_user_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'user_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_system_folders(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'system_folders')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_system_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'system_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_config_folders(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'config_folders')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_config_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'config_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_navpose_frames(timeout = float('inf'), log_name_list = []):
    param_namespace = 'navpose_frames'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_navpose_frames(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'navpose_frames')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_timezone(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'timezone')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_timezone(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'timezone')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_active_drivers(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'active_drivers')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_active_drivers(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'active_drivers')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

''' Need to add
def get_active_ai_frameworks(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'active_ai_frameworks')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_ai_models(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'active_ai_models')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_apps(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'active_apps')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_auto_scrips(timeout = float('inf'), log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(BASE_NAMESPACE,'active_auto_scrips')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data
'''
