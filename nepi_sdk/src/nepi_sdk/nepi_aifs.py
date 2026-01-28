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


# NEPI ros utility functions include
# 1) NEPI IDX AI utility functions

import os
import sys
import importlib
import subprocess
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
  
from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_aifs"
logger = Logger(log_name = log_name)
 
#***************************
# NEPI AIs utility functions

def getAIFsDict(params_path, api_path):
    aifs_dict = dict()
    # Find AIF files
    ind = 0
    if os.path.exists(params_path):
        params_path = nepi_utils.clear_end_slash(params_path)
        if os.path.exists(api_path):
          api_path = nepi_utils.clear_end_slash(api_path)
          sys.path.append(params_path)
          logger.log_debug("Searching for AIFs in path: " + params_path, throttle_s = 5.0)
          for f in os.listdir(params_path):
            if f.endswith(".yaml") and f.find("params") != -1: 
              try:
                file_path = os.path.join(params_path,f)
                new_dict = nepi_utils.read_yaml_2_dict(file_path)
                framework_name = new_dict['framework_name']
                #logger.log_warn("Got ais dict: " + str(new_dict))
                new_dict['api_path'] = api_path
                new_dict['active'] = False
                aifs_dict[framework_name] = new_dict
              except Exception as e:
                logger.log_warn("Failed to import param file: " + f + " " + str(e))
        else:
              logger.log_warn("AIF api path does not exist: " +  api_path)
          # Check for node file   
    else:
        logger.log_warn("AIF params search path does not exist: " +  params_path)
    # Check for node file   

    purge_list = []
    for aif_name in aifs_dict.keys():
      purge = False
      pkg_name = aifs_dict[aif_name]['pkg_name']
      if_file = aifs_dict[aif_name]['if_file_name']
      if_file_path = os.path.join(api_path,if_file)
      if os.path.exists(if_file_path) == False:
        logger.log_warn("Could not find ai file: " + if_file_path)
        purge = True
      if purge == True:
        purge_list.append(aif_name)
    for aif_name in purge_list:
      del aifs_dict[aif_name]
    return aifs_dict

# ln = sys._getframe().f_lineno ; self.printND(aifs_dict,ln)
def printDict(aifs_dict):
  logger.log_warn('')
  logger.log_warn('*******************')
  if line_num is not None:
    logger.log_warn(str(line_num))
  logger.log_warn('Printing Nex AIF Dictionary')

  for aif_name in aifs_dict.keys():
    aifs_dict = aifs_dict[aif_name]
    logger.log_warn('')
    logger.log_warn(aif_name)
    logger.log_warn(str(aifs_dict))


# def refreshAIFsDict(params_path, api_path, aifs_dict):
#   success = True
#   params_path = nepi_utils.clear_end_slash(params_path)
#   api_path = nepi_utils.clear_end_slash(api_path)
#   get_aifs_dict = getAIFsDict(params_path, api_path)
#   for aif_name in get_aifs_dict.keys():
#     if aif_name in aifs_dict.keys():
#       get_aifs_dict[aif_name]['active'] = aifs_dict[aif_name]['active']
#   return get_aifs_dict

  

def getAIFsByActive(aifs_dict):
  active_dict = dict()
  for aif_name in aifs_dict.keys():
    aif_dict = aifs_dict[aif_name]
    aif_active = aif_dict['active']
    if aif_active == True:
      active_dict[aif_name] = aif_dict
  return active_dict

def getAIFsSortedList(aifs_dict):
  aifs_names = list(aifs_dict.keys())
  sorted_names = sorted(aifs_names)
  sorted_list = []
  for name in sorted_names:
    sorted_list.append(str(name))
  return sorted_names


def getAIFsActiveSortedList(aifs_dict):
  sorted_name_list = getAIFsSortedList(aifs_dict)
  #logger.log_warn("AIFS_MGR: sorted list: " + str(sorted_name_list))
  sorted_active_list =[]
  for aif_name in sorted_name_list:
    active = aifs_dict[aif_name]['active']
    if active:
      sorted_active_list.append(aif_name)
  return sorted_active_list



def getAIFInfoFilesList(aifs_path):
  aifs_list = []
  if aifs_path != '':
    if os.path.exists(aifs_path):
      [file_list, num_files] = nepi_utils.get_file_list(aifs_path,"py")
      for f in file_list:
        aifs_list.append(f.split(".")[0])
  return aifs_list

  
def getAIFPackagesList(install_path):
  pkg_list = []
  if install_path != '':
    if os.path.exists(install_path):
      [file_list, num_files] = nepi_utils.get_file_list(install_path,"zip")
      for pkg in file_list:
        pkg_list.append(os.path.basename(pkg))
  return pkg_list


 
def activateAllFws(aifs_dict):
  success = True
  for aif_name in aifs_dict.keys():
    aifs_dict = activateAIF(aif_name,aifs_dict)
  return aifs_dict

def disableAllFws(aifs_dict):
  success = True
  for aif_name in aifs_dict.keys():
    aifs_dict = disableAIF(aif_name,aifs_dict)
  return aifs_dict

def activateFw(aif_name,aifs_dict):
    if aif_name not in aifs_dict.keys():
      logger.log_warn("AIF for activate request does not exist: " + aif_name)
      return aifs_dict
    aifs_dict[aif_name]['active'] = True
    return aifs_dict

def disableFw(aif_name,aifs_dict):
    if aif_name not in aifs_dict.keys():
      logger.log_warn("AIF for removal request does not exist: " + aif_name)
      return aifs_dict
    aifs_dict[aif_name]['active'] = False
    return aifs_dict


def getModelsByActive(models_dict):
  active_dict = dict()
  for model_name in models_dict.keys():
    model_dict = models_dict[model_name]
    model_active = models_dict['active']
    if model_active == True:
      active_dict[model_name] = model_dict
  return active_dict

def getModelsSortedList(models_dict):
  sorted_models = sorted(list(models_dict.keys()))
  sorted_names = []
  for model in sorted_models:
    if 'name' in models_dict[model].keys():
      sorted_names.append(models_dict[model]['name'])
    else:
      sorted_names.append(model)
  return sorted_models,sorted_names


def getModelsActiveSortedList(models_dict):
  [sorted_models,sorted_names] = getModelsSortedList(models_dict)
  #logger.log_warn("AIFS_MGR: sorted list: " + str(sorted_name_list))
  sorted_active_models = []
  sorted_active_names =[]
  for model_name in sorted_models:
    for key in models_dict.keys():
      name = models_dict[key]['name']
      if name == model_name:
        active = models_dict[model_name]['active']
        if active:
          sorted_active_models.append(key)
          sorted_active_names.append(model_name)
  return sorted_active_models, sorted_active_names




def activateAllModels(models_dict):
    for model_name in models_dict.keys():
      models_dict[model_name]['active'] = True
    return models_dict

def disableAllModels(models_dict):
    for model_name in models_dict.keys():
      models_dict[model_name]['active'] = False
    return models_dict


def importAIFClass(file_name,file_path,module_name,class_name):
      module_class = None
      success = False
      msg = "failed"
      file_path = os.path.join(file_path,file_name)
      if os.path.exists(file_path):
        sys.path.append(file_path)
        try:
          module = importlib.import_module(module_name)
          try:
            module_class = getattr(module, class_name)
            success = True
            msg = 'success'
          except Exception as e:
            logger.log_warn("Failed to import class with exception: " + class_name + " " + module_name + " " + str(e))
        except Exception as e:
            logger.log_warn("Failed to import module with exception: "  + module_name + " " + str(e))
      else:
        logger.log_warn("Failed to find file in path for module: " + file_name + " " + file_path + " " + module_name)
      return success, msg, module_class



def unimportAIFClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            logger.log_info("Failed to clordered_unimport module: " + module_name)
        if module_name in sys.modules:
          success = False
    return success

def launchClassifierNode(pkg_name, file_name, ros_node_name, device_path = None):
  sub_process = None
  msg = 'Success'
  success = False
  if device_path is None:
    device_node_run_cmd = ['rosrun', pkg_name, file_name, '__name:=' + ros_node_name]
  else:
    device_node_run_cmd = ['rosrun', pkg_name, file_name, '__name:=' + ros_node_name, '_device_path:=' + device_path]
  try:
    sub_process = subprocess.Popen(device_node_run_cmd)
    success = True
  except Exception as e:
    msg = str("Failed to launch node with exception: " + ros_node_name + " " + str(e))
    logger.log_warn(msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("Failed to start " + device_node_name + " via " + " ".join(x for x in device_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
      logger.log_err(msg)
      sub_process = None
      success = False
  return success, msg, sub_process
  

def killClassifierNode(node_namespace,sub_process):
    success = True
    node_name = node_namespace.split("/")[-1]
    node_namespace_list = nepi_sdk.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    logger.log_warn( str(node_list))
    logger.log_warn( node_name)
    if node_name in node_list:
      logger.log_warn("Killing node: " + node_name)
      [kill_list,fail_list] = nepi_sdk.kill_node(node_name)
      time.sleep(2)    
      # Next check running processes
      if sub_process.poll() is not None: 
        sub_process.terminate()
        terminate_timeout = 3
        while (terminate_timeout > 0):
          time.sleep(1)
          if sub_process.poll() is not None:
            terminate_timeout -= 1
            success = False
          else:
            success = True
            break
        if success == False:
          # Escalate it
          sub_process.kill()
          time.sleep(1)
        if sub_process.poll() is not None:
          success = False
        else:
          success = True
    if success:
      cleanup_proc = subprocess.Popen(['rosnode', 'cleanup'], stdin=subprocess.PIPE)
      try:
        cleanup_proc.communicate(input=bytes("y\r\n", 'utf-8'), timeout=10)
        cleanup_proc.wait(timeout=10) 
      except Exception as e:
        logger.log_warn("rosnode cleanup failed: " + str(e))
      logger.log_warn("Killed node: " + node_name)
    else:
       logger.log_warn("Failed to kill node: " + node_name)
    return success
        


