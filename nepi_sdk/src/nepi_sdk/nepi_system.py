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
import shutil

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_system"
logger = Logger(log_name = log_name)

NEPI_SYSTEM_CONFIG_FILE = '/opt/nepi/etc/nepi_system_config.yaml'
SYSTEM_CONFIG_FILE = '/mnt/nepi_config/system_cfg/etc/nepi_system_config.yaml'


NEPI_DOCKER_CONFIG_FILE = '/opt/nepi/docker_cfg/nepi_docker_config.yaml'
DOCKER_CONFIG_FILE = '/mnt/nepi_config/docker_cfg/nepi_docker_config.yaml'

NEPI_ALL_CONFIG_IDS = ['idx','ptx','lsx','npx','rbx']

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

def get_admin_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'admin_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_admin_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'admin_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_develop_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'develop_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_develop_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'develop_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_debug_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'debug_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_debug_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'debug_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success


def get_managers_enabled(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'managers_enabled')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_managers_enabled(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'managers_enabled')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_user_restrictions(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_restrictions')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_user_restrictions(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_restrictions')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_run_mode(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'run_mode')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_run_mode(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'run_mode')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################



def get_user_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_folders')
    #print("Waiting for User Folders in param: " + str(param_namespace))
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_user_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'user_folders')
    #print("Storing User Folders in param: " + str(param_namespace))
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_system_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'system_folders')
    #logger.log_warn("Got system folders namespace: " +  str(param_namespace))
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_system_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'system_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

##########################

def get_config_folders(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'config_folders')
    data = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    return data

def set_config_folders(value, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'config_folders')
    success = nepi_sdk.set_param(param_namespace, value, log_name_list = log_name_list)
    return success

def get_node_names_dict(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'node_names_dict')
    node_names_dict = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    if node_names_dict is None:
        node_names_dict = dict()
    return node_names_dict

def set_node_names_dict(node_names_dict, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'node_names_dict')
    success = nepi_sdk.set_param(param_namespace, node_names_dict, log_name_list = log_name_list)
    return success

def get_node_name(node_name):
    node_name_dict = get_node_names_dict()
    if node_name in node_name_dict.keys():
        node_name = node_name_dict[node_name]
    return node_name

def supports_all_config(namespace):
    supports_all = False
    for id in NEPI_ALL_CONFIG_IDS:
        check_str = '/' + id
        if check_str in namespace:
            supports_all = True
            break
    return supports_all
##########################

def get_navposes_dict(timeout = 1000, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'navposes_dict')
    navposes_dict = nepi_sdk.wait_for_param(param_namespace, timeout = timeout, log_name_list = log_name_list)
    if navposes_dict is None:
        navposes_dict = dict()
    return navposes_dict

def set_navposes_dict(navposes_dict, log_name_list = []):
    param_namespace = nepi_sdk.create_namespace(nepi_sdk.get_base_namespace(),'navposes_dict')
    success = nepi_sdk.set_param(param_namespace, navposes_dict, log_name_list = log_name_list)
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

########################
def load_nepi_system_config():
    config_dict=dict()

    target_file=SYSTEM_CONFIG_FILE
    backup_file=NEPI_SYSTEM_CONFIG_FILE
    if os.path.exists(target_file) == False:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))

    config_dict = nepi_utils.read_dict_from_file(SYSTEM_CONFIG_FILE)
    if config_dict is None:
        config_dict = dict()
    for key in config_dict.keys(): # Fixe empty arrays
        if config_dict[key] is None:
            config_dict[key]=[]
    # print("Printing System Config Dict " + str(config_dict))
    return config_dict

def save_nepi_system_config(config_dict):
    target_file=SYSTEM_CONFIG_FILE
    backup_file=NEPI_SYSTEM_CONFIG_FILE
    if os.path.exists(target_file) == False:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))
    success = nepi_utils.write_dict_to_file(config_dict, SYSTEM_CONFIG_FILE)
    return success

def update_nepi_system_config(config_key, config_value, config_dict = None):
    success=False
    if config_dict is None:
        config_dict=load_nepi_system_config()
    if len(config_dict.keys()) > 0:
        config_dict[config_key] = config_value
        success=save_nepi_system_config(config_dict)
    return config_dict


########################
def load_nepi_docker_config():
    config_dict=dict()

    target_file=DOCKER_CONFIG_FILE
    backup_file=NEPI_DOCKER_CONFIG_FILE
    if os.path.exists(target_file) == False:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))


    config_dict = nepi_utils.read_dict_from_file(target_file)
    if config_dict is None:
        config_dict = dict()
    for key in config_dict.keys(): # Fixe empty arrays
        if config_dict[key] is None:
            config_dict[key]=[]
    return config_dict

def save_nepi_docker_config(config_dict):

    target_file=DOCKER_CONFIG_FILE
    backup_file=NEPI_DOCKER_CONFIG_FILE
    if os.path.exists(target_file) == False:
        folder=os.path.dirname(target_file)
        if os.path.exists(folder) == False:       
            try:
                os.makedirs(folder)
            except Exception as e:
                print("Unable to create folder: " + str(e))
        try:
            shutil.copyfile(backup_file, target_file)
        except Exception as e:
            print("Unable to copy file: " + str(e))

    success = nepi_utils.write_dict_to_file(config_dict, DOCKER_CONFIG_FILE)
    return success

def update_nepi_docker_config(config_key, config_value):
    success=False
    config_dict=load_nepi_docker_config()
    if len(config_dict.keys()) > 0:
        config_dict[config_key] = config_value
        success=save_nepi_docker_config(config_dict)
    return success