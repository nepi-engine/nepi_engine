#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI ros utility functions include
# 1) NEPI IDX AI utility functions
import os
import sys
import zipfile
import getpass
import importlib
import subprocess
import rospy
import rosnode
import warnings
import numpy as np
import time
import usb
import copy
import yaml
from serial.tools import list_ports

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_save
  
 
#***************************
# NEPI AIs utility functions

def getAIFsDict(search_path):
    aifs_dict = dict()
    # Find AIF files
    ind = 0
    if os.path.exists(search_path):
        if search_path[-1] == "/":
            search_path = search_path[:-1]
        sys.path.append(search_path)
        rospy.loginfo("NEPI_AIFS: Searching for AIFs in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".yaml") and f.find("params") != -1: 
            try:
              file_path = os.path.join(search_path,f)
              new_dict = nepi_save.read_yaml2dict(file_path)
              #rospy.logwarn("NEPI_AIFS: Got ais dict: " + str(new_dict))
              new_dict['if_path'] = search_path
              new_dict['active'] = True
              aifs_dict[new_dict['pkg_name']] = new_dict
            except Exception as e:
              rospy.logwarn("NEPI_AIFS: Failed to import param file: " + f + " " + str(e))
    else:
        rospy.logwarn("NEPI_AIFS: AIF search path %s does not exist",  search_path)
    # Check for node file   

    purge_list = []
    for aif_name in aifs_dict.keys():
      purge = False
      pkg_name = aifs_dict[aif_name]['pkg_name']
      if_file = aifs_dict[aif_name]['if_file_name']
      if_file_path = os.path.join(search_path,if_file)
      if os.path.exists(if_file_path) == False:
        rospy.logwarn("NEPI_AIFS: Could not find ai file: " + if_file_path)
        purge = True
      if purge == True:
        purge_list.append(aif_name)
    for aif_name in purge_list:
      del aifs_dict[aif_name]
    return aifs_dict

# ln = sys._getframe().f_lineno ; self.printND(aifs_dict,ln)
def printDict(aifs_dict):
  rospy.logwarn('NEPI_AIFS: ')
  rospy.logwarn('NEPI_AIFS:*******************')
  if line_num is not None:
    rospy.logwarn('NEPI_AIFS: ' + str(line_num))
  rospy.logwarn('NEPI_AIFS: Printing Nex AIF Dictionary')

  for aif_name in aifs_dict.keys():
    aifs_dict = aifs_dict[aif_name]
    rospy.logwarn('NEPI_AIFS: ')
    rospy.logwarn('NEPI_AIFS: ' + aif_name)
    rospy.logwarn(str(aifs_dict))


def refreshAIFsDict(aifs_path,aifs_dict):
  success = True
  if aifs_path[-1] == "/":
      aifs_path = aifs_path[:-1]
  get_aifs_dict = getAIFsDict(aifs_path)
  for aif_name in get_aifs_dict.keys():
    if aif_name in aifs_dict.keys():
      get_aifs_dict[aif_name]['active'] = aifs_dict[aif_name]['active']
  return get_aifs_dict

  

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
  return sorted_list


def getAIFsActiveSortedList(aifs_dict):
  sorted_name_list = getAIFsSortedList(aifs_dict)
  #rospy.logwarn("AIFS_MGR: sorted list: " + str(sorted_name_list))
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
      [file_list, num_files] = nepi_ros.get_file_list(aifs_path,"py")
      for f in file_list:
        aifs_list.append(f.split(".")[0])
  return aifs_list

  
def getAIFPackagesList(install_path):
  pkg_list = []
  if install_path != '':
    if os.path.exists(install_path):
      [file_list, num_files] = nepi_ros.get_file_list(install_path,"zip")
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
      rospy.logwarn("NEPI_AIFS: AIF %s for activate request does not exist", aif_name)
      return aifs_dict
    aifs_dict[aif_name]['active'] = True
    return aifs_dict

def disableFw(aif_name,aifs_dict):
    if aif_name not in aifs_dict.keys():
      rospy.logwarn("NEPI_AIFS: AIF %s for removal request does not exist", aif_name)
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
  models_names = list(models_dict.keys())
  sorted_names = sorted(models_names)
  sorted_list = []
  for name in sorted_names:
    sorted_list.append(str(name))
  return sorted_list


def getModelsActiveSortedList(models_dict):
  sorted_name_list = getModelsSortedList(models_dict)
  #rospy.logwarn("AIFS_MGR: sorted list: " + str(sorted_name_list))
  sorted_active_list =[]
  for model_name in sorted_name_list:
    active = models_dict[model_name]['active']
    if active:
      sorted_active_list.append(model_name)
  return sorted_active_list




def activateAllModels(models_dict):
    for model_name in models_dict.keys():
      models_dict[model_name]['active'] = True
    return models_dict

def disableAllModels(models_dict):
    for model_name in models_dict.keys():
      models_dict[model_name]['active'] = False
    return models_dict

def activateModel(model_name,models_dict):
  models_dict[model_name]['active'] = True
  return models_dict

def disableModel(model_name,models_dict):
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
            rospy.logwarn("NEPI_AIFS: Failed to import class %s from module %s with exception: %s", class_name, module_name, str(e))
        except Exception as e:
            rospy.logwarn("NEPI_AIFS: Failed to import module %s with exception: %s", module_name, str(e))
      else:
        rospy.logwarn("NEPI_AIFS: Failed to find file %s in path %s for module %s", file_name, file_path, module_name)
      return success, msg, module_class



def unimportAIFClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            rospy.loginfo("NEPI_AIFS: Failed to clordered_unimport module: " + module_name)
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
    msg = str("Failed to launch node %s with exception: %s", ros_node_name, str(e))
    rospy.logwarn("NEPI_NEX: " + msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("Failed to start " + device_node_name + " via " + " ".join(x for x in device_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
      rospy.logerr(msg)
      sub_process = None
      success = False
  return success, msg, sub_process
  

def killClassifierNode(node_namespace,sub_process):
    success = True
    node_name = node_namespace.split("/")[-1]
    node_namespace_list = nepi_ros.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    rospy.logwarn("NEPI_AIS: " + str(node_list))
    rospy.logwarn("NEPI_AIS: " + node_name)
    if node_name in node_list:
      rospy.logwarn("NEPI_AIS: Killing node: " + node_name)
      [kill_list,fail_list] = rosnode.kill_nodes([node_name])
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
        rospy.logwarn(self.log_name + ": " + "rosnode cleanup failed (%s)", str(e))
      rospy.logwarn("NEPI_AIS: Killed node: " + node_name)
    else:
       rospy.logwarn("NEPI_AIS: Failed to kill node: " + node_name)
    return success
        


