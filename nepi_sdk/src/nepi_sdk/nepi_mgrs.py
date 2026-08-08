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
import sys
import zipfile
import getpass
import importlib
import subprocess
import shutil


import numpy as np
import time
import copy
from serial.tools import list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_mgrs"
logger = Logger(log_name = log_name)

#######################
### Manager Utility Functions
MANAGERS_NODE_FOLDER = '/opt/nepi/nepi_engine/lib/nepi_managers'
MANAGERS_ETC_FOLDER = '/opt/nepi/nepi_engine/etc'
BASE_MANAGERS = ['MANAGER-SYSTEM','MANAGER-CONFIG']

def getManagersDict(search_path):
    mgrs_dict = dict()
    # Find manager files
    ind = 0
    base_namespace = nepi_sdk.get_base_namespace()
    if os.path.exists(search_path):
        if search_path[-1] == "/":
            search_path = search_path[:-1]
        sys.path.append(search_path)
        #logger.log_info("Searching for managers in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".yaml") and f.find("params") != -1: 
                  try:
                    file_path = os.path.join(search_path,f)
                    read_dict = nepi_utils.read_yaml_2_dict(file_path)
                  except Exception as e:
                    logger.log_warn("Failed to import param file: " + f + " " + str(e))
                    continue
                  if 'manager' in read_dict.keys():
                    new_dict = read_dict['manager']
                    try:
                      mgr_name = new_dict['display_name']
                      new_dict['pkg_name'] = new_dict['pkg_name']
                      new_dict['display_name'] = new_dict['display_name']
                      new_dict['node_file'] = new_dict['node_file']
                      new_dict['etc_file'] = new_dict['etc_file']
                      new_dict['node_name'] = new_dict['node_name']
                      new_dict['node_namespace'] = os.path.join(base_namespace,new_dict['node_name'])
                      new_dict['param_file_name'] = os.path.basename(f)
                      new_dict['order'] = -1
                      new_dict['active'] = False
                      new_dict['msg'] = ""
                      #logger.log_warn("Got mgrs dict: " + str(new_dict))

                      #logger.log_warn("Got manager dict from file: " + str(new_dict))

                      mgrs_dict[mgr_name] = new_dict
                    except Exception as e:
                      logger.log_warn("Failed to get manager params from file: " + f + " " + str(e))
                  else:
                    logger.log_warn("Manager dict key not found in file: " + f )
    else:
        logger.log_warn("Manager path does not exist: " +  search_path)
    # Now assign factory orders
    mgrs_dict = setFactoryManagerOrder(mgrs_dict)
    return mgrs_dict

def printDict(mgrs_dict):
  logger.log_warn('')
  logger.log_warn('*******************')
  logger.log_warn('Printing Drv Manager Dictionary')
  for manager_name in mgrs_dict.keys():
    mgr_dict = mgrs_dict[manager_name]
    logger.log_warn('')
    logger.log_warn(manager_name)
    logger.log_warn(str(mgr_dict))


def refreshManagersDict(search_path,mgrs_dict):
  success = True
  if search_path[-1] == "/":
      search_path = search_path[:-1]
  get_mgrs_dict = getManagersDict(search_path)
  #logger.log_warn('Updating Drvs Dict: ' + str(mgrs_dict))
  #logger.log_warn('From Get Dict: ' + str(get_mgrs_dict))
  for manager_name in get_mgrs_dict.keys():
    if manager_name not in mgrs_dict.keys():
      get_mgrs_dict[manager_name]['active'] = False
    else:
      #.log_warn('')
      #logger.log_warn('Updating mgr: ' + manager_name)
      #logger.log_warn('Updating mgr: ' + str(mgrs_dict[manager_name]))
      active = mgrs_dict[manager_name]['active']
      #logger.log_warn("Refresh updating mgr : " + str(manager_name) + " to active state: " + str(active))
      get_mgrs_dict[manager_name]['active'] = active
      get_mgrs_dict[manager_name]['order'] = mgrs_dict[manager_name]['order']
  #logger.log_warn('Updated to: ' + str(get_mgrs_dict))
  return get_mgrs_dict




def setFactoryManagerOrder(mgrs_dict):
  indexes = []
  factory_indexes = []
  man_ind = 0
  call_ind = 1000
  run_ind = call_ind * 1000
  launch_ind = run_ind * 1000
  catch_ind = launch_ind * 1000
  order = catch_ind
  for manager_name in mgrs_dict.keys():
    mgr_dict = mgrs_dict[manager_name]
    if 'DISCOVERY_DICT' in mgr_dict.keys():
      if mgrs_dict[manager_name]['DISCOVERY_DICT']["file_name"] != "None":
        discovery_proc = mgr_dict['DISCOVERY_DICT']['process']
        if discovery_proc == 'CALL':
          order = call_ind
        elif discovery_proc == 'RUN':
          order = run_ind 
        elif discovery_proc == 'LAUNCH':
          order = launch_ind
    while order in indexes:
      order += 1
    indexes.append(order)
  for val in indexes:
      factory_indexes.append(0)
  sorted_indexes = list(np.sort(indexes))
  for i in range(len(indexes)):
     factory_indexes[i] = sorted_indexes.index(indexes[i])
  for i, manager_name in enumerate(mgrs_dict.keys()):
    mgrs_dict[manager_name]['order'] = factory_indexes[i]
  return mgrs_dict


def moveManagerTop(manager_name,mgrs_dict):
  if manager_name in mgrs_dict.keys():
    current_ordered_list = getManagersOrderedList(mgrs_dict)
    current_order = current_ordered_list.index(manager_name)
    if current_order > 0:
      new_order = 0
      mgrs_dict = setManagerOrder(manager_name,new_order,mgrs_dict)
  return mgrs_dict

def moveManagerBottom(manager_name,mgrs_dict):
  if manager_name in mgrs_dict.keys():
    current_ordered_list = getManagersOrderedList(mgrs_dict)
    current_order = current_ordered_list.index(manager_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = len(current_ordered_list) - 1
      mgrs_dict = setManagerOrder(manager_name,new_order,mgrs_dict)
  return mgrs_dict

def moveManagerUp(manager_name,mgrs_dict):
  if manager_name in mgrs_dict.keys():
    current_ordered_list = getManagersOrderedList(mgrs_dict)
    current_order = current_ordered_list.index(manager_name)
    if current_order > 0:
      new_order = current_order -1
      mgrs_dict = setManagerOrder(manager_name,new_order,mgrs_dict)
  return mgrs_dict

def moveManagerDown(manager_name,mgrs_dict):
  if manager_name in mgrs_dict.keys():
    current_ordered_list = getManagersOrderedList(mgrs_dict)
    current_order = current_ordered_list.index(manager_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = current_order + 1
      mgrs_dict = setManagerOrder(manager_name,new_order,mgrs_dict)
  return mgrs_dict

def setManagerOrder(manager_name,new_order,mgrs_dict):
  if manager_name in mgrs_dict.keys():
    ordered_list = getManagersOrderedList(mgrs_dict)
    current_order = ordered_list.index(manager_name)
    manager_entry = ordered_list.pop(current_order)
    ordered_list.insert(new_order,manager_entry)
    for manager_name in mgrs_dict.keys():
      mgrs_dict[manager_name]['order'] = ordered_list.index(manager_name)
  return mgrs_dict

def setManagerMsg(manager_name,msg,mgrs_dict):
  if manager_name in mgrs_dict.keys():
    mgrs_dict[manager_name]['msg'] = str(msg)
  return mgrs_dict

    

def getManagersOrderedList(mgrs_dict):
  name_list = []
  order_list = []
  ordered_name_list = []
  for manager_name in mgrs_dict.keys():
    name_list.append(manager_name)
    mgr_dict = mgrs_dict[manager_name]
    order = mgr_dict['order']
    if order == -1:
      order = 1000
    while(order in order_list):
      order += 0.1
    order_list.append(order)
  s = list(sorted(order_list))
  indexes = [s.index(x) for x in order_list]
  for val in order_list:
    ordered_name_list.append(0)
  for i,index in enumerate(indexes):
    ordered_name_list[index] = name_list[i]
  return ordered_name_list



# def getManagersActiveList(mgrs_dict):
#   active_list = []
#   for manager_name in mgrs_dict.keys():
#     if mgrs_dict[manager_name]['active'] == True:
#       active_list.append(manager_name)
#   return active_list
 



def installManagerPkg(pkg_name,mgrs_dict,install_from_path,managers_path):
    success = True
    if install_from_path[-1] == "/":
      install_from_path = install_from_path[:-1]
    if managers_path[-1] == "/":
      search_path = managers_path[:-1]

    if os.path.exists(install_from_path) == False:
      logger.log_warn("Install package source folder does not exist: " + install_from_path)
      return False, mgrs_dict
    if os.path.exists(managers_path) == False:
      logger.log_warn("Install managers destination folder does not exist: " + managers_path)
      return False, mgrs_dict
    pkg_list = getManagerPackagesList(install_from_path)
    if pkg_name not in pkg_list:
      logger.log_warn("Install package for not found in install folder: " + pkg_name + " " + install_from_path)
      return False, mgrs_dict
    os_user = getpass.getuser()
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_from_path)
    os.system('chown -R ' + 'nepi:nepi' + ' ' + managers_path)
    os.system('chown -R ' + 'nepi:nepi' + ' ' + params_path)
    pkg_path = install_from_path + "/" + pkg_name
    manager_path = managers_path
    try:
      pkg = zipfile.ZipFile(pkg_path)
      pkg_files = pkg.namelist()

    except Exception as e:
      logger.log_warn("" + str(e))
      success = False
    if success:
      # Create a list of files
      manager_files = []
      for pkg_file in pkg_files:
        if pkg_file.endswith(".yaml"):
          manager_file = manager_path + "/" + pkg_file
        manager_files.append(manager_file)
      for file in manager_files:
        if os.path.exists(file):
          try:
            os.remove(file)
          except Exception as e:
            success = False
            logger.log_warn(str(e))
      if success:
        # Unzip the package to the Manager path
        with zipfile.ZipFile(pkg_path,"r") as zip_ref:
          zip_ref.extractall(manager_path)
        # Check for success
        for f in manager_files:
          if os.path.exists(f) == False:
            success = False
          else:
            os.system('chown -R ' + 'nepi:nepi' + ' ' + f)
            new_f = os.path.join(managers_path,os.path.basename(f))
            try:
              os.rename(f,new_f)
              os.system('chown -R ' + 'nepi:nepi' + ' ' + new_f)
            except Exception as e:
              ospy.logwarn("NEPI_DRV: Failed to move param file to new location: " + new_f + " " + str(e))
              success = False
    mgrs_dict = updateManagersDict(manager_path,mgrs_dict)
    return success, mgrs_dict 



def removeManager(manager_name,mgrs_dict,managers_path,backup_path = None):
    success = True   
    if manager_name not in mgrs_dict.keys():
      logger.log_warn("Manager for removal request does not exist: " + manager_name)
      return False, mgrs_dict
    mgr_dict = mgrs_dict[manager_name]

    param_file = os.path.join(managers_path,mgr_dict['param_file_name'])
    node_file = os.path.join(managers_path,mgr_dict['NODE_DICT']['file_name'])
    discovery_file = os.path.join(managers_path,mgr_dict['DISCOVERY_DICT']['file_name'])
    manager_file = os.path.join(managers_path,mgr_dict['MANAGER_DICT']['file_name'])

    file_list = []
    file_list.append(param_file)
    file_list.append(node_file)
    file_list.append(discovery_file)
    file_list.append(manager_file)

    os_user = getpass.getuser()
    for i,file in enumerate(file_list):
      if file == 'None' or os.path.exists(file) == False:
        file_list.remove(file)
      else:
        os.system('chown -R ' + 'nepi:nepi' + ' ' + filepath)
    # Create an backup package from manager files
    logger.log_info("Removing manager files: " + str(file_list))      
    if backup_path != None:
      if backup_path[-1] == "/":
        backup_path = backup_path[:-1]
      if os.path.exists(backup_path) == False:
        backup_path = None
      else:
        os.system('chown -R ' + 'nepi:nepi' + ' ' + backup_path)
        zip_file = backup_path + "/" + manager_name + ".zip"
        logger.log_info("Backing up removed file to: " + zip_file)
        try:
          zip = zipfile.ZipFile(zip_file, "w", zipfile.ZIP_DEFLATED)
          for file in file_list:
            zip.write(file, os.path.basename(file), compress_type=zipfile.ZIP_DEFLATED)
          zip.close()
          zip = None
        except Exception as e:
          logger.log_warn("Failed to backup manager: " + str(e))
          if os.path.exists(zip_file) == True:
            try:
              zip.close()
            except Exception as e:
              logger.log_warn(str(e))
        for file in file_list:
          if os.path.exists(file) == True:
            try:
              os.remove(file)
            except Exception as e:
              success = False
              logger.log_warn("Failed to remove manager file: " + file + " " + str(e))

    if success:
      del mgrs_dict[manager_name]
    return success, mgrs_dict



def launchManagerNode(file_name, manager_node_name, etc_file = None):
  sub_process = None
  msg = 'Success'
  success = False
  logger.log_warn("Launching Manager Node: " + str(manager_node_name) )
  if etc_file is not None:
    etc_file_path = os.path.join(MANAGERS_ETC_FOLDER, etc_file)
    if os.path.exists(etc_file_path):
      base_namespace = nepi_sdk.get_base_namespace()
      namespace = nepi_sdk.create_namespace(base_namespace,manager_node_name)
      params_dict = nepi_sdk.load_params_from_file(etc_file_path, namespace = namespace)
      if len(list(params_dict.keys())) == 0:
        logger.log_warn("Failed to load etc params for Manager Node: " + str(manager_node_name) + " from file: " + str(etc_file_path) )
  manager_node_run_cmd = ['rosrun', 'nepi_managers', file_name, '__name:=' + manager_node_name]
  try:
    sub_process = subprocess.Popen(manager_node_run_cmd)
    success = True
  except Exception as e:
    msg = str("Failed to launch node with exception: " + manager_node_name + " " + str(e))
    logger.log_warn(msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("Failed to start " + manager_node_name + " via " + " ".join(x for x in manager_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
      logger.log_error(msg)
      sub_process = None
      success = False
  return success, msg, sub_process
 

def killManagerNode(node_namespace,sub_process):
    success = False
    if sub_process.poll() is None:
      sub_process.terminate()
      terminate_timeout = 3
      node_dead = False
      while (terminate_timeout > 0):
        time.sleep(1)
        if sub_process.poll() is None:
          terminate_timeout -= 1
        else:
          node_dead = True
          break
      if not node_dead:
        # Escalate it
        sub_process.kill()
        time.sleep(1)
    if sub_process.poll() is not None:
      success = True
    return success
        

