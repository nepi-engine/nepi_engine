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




#######################
## Get System Data Functions

def get_nepi_config(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'nepi_config')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_nepi_config(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'nepi_config')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_debug_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'debug_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_debug_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'debug_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_admin_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'admin_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_admin_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'admin_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_admin_restricted(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'admin_restricted')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_admin_restricted(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'admin_restricted')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################



def get_user_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_folders')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_user_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_system_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'system_folders')
    logger.log_warn("Got system folders namespace: " +  str(param_namespace))
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_system_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'system_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_config_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'config_folders')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_config_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'config_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_navpose_frames(timeout = 1000, log_name_list = []):
    param_namespace = 'navpose_frames'
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_navpose_frames(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'navpose_frames')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_timezone(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'timezone')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_timezone(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'timezone')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_active_drivers(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_drivers')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_active_drivers(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_drivers')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

''' Need to add
def get_active_ai_frameworks(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_ai_frameworks')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_ai_models(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_ai_models')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_apps(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_apps')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def get_active_auto_scrips(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'active_auto_scrips')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data
'''
