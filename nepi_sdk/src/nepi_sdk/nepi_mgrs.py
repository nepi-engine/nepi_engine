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
# 1) NEPI IDX App utility functions
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
from serial.tools import list_ports

from nepi_sdk import nepi_ros
  
#***************************
# NEPI Apps utility functions

def getAppsDict(search_path):
    mgrs_dict = dict()
    # Find App files
    ind = 0
    if os.path.exists(search_path):
        if search_path[-1] == "/":
            search_path = search_path[:-1]
        sys.path.mgrend(search_path)
        rospy.loginfo("NEPI_MGR: Searching for Apps in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".py"): 
            module_name = f.split(".")[0]
            #rospy.loginfo("NEPI_MGR: Will try to import module: " + module_name)
            open_success = True
            read_success = True
            warnings.filterwarnings('ignore', '.*unclosed.*', )
            try:
              module = __import__(module_name)
            except Exception as e:
              #rospy.logwarn("NEPI_MGR: failed to import module %s with exception %s", f, str(e))
              open_success = False
            if open_success:
                try:
                  mgr_name = module.MGR_NAME                
                except:
                  #rospy.logwarn("NEPI_MGR: No MGR_NAME in module: " + f)
                  read_success = False
                if read_success:
                  #rospy.logwarn("NEPI_MGR: " + mgr_name)
                  try:
                    mgrs_dict[mgr_name] = {
                      'MGR_DICT': module.MGR_DICT,
                      'RUI_DICT': module.RUI_DICT,
                      'order': -1,
                      'subprocess': "",
                      'active': False,
                      'msg': ""
                      }
                  except Exception as e:
                    try:
                      del mgrs_dict[mgr_name]
                    except:
                      pass
                    rospy.logwarn("NEPI_MGR: Failed to get info from module: " + f +" with exception: " + str(e))
                else:
                    rospy.logwarn("NEPI_MGR: Failed to get valid MGR_NAME from: " + f )
                if open_success:
                  try:
                    #sys.modules.pop(module)
                    if module_name in sys.modules:
                      del sys.modules[module_name]
                    del module
                  except:
                    rospy.loginfo("NEPI_MGR: Failed to remove module: " + f)
    else:
        rospy.logwarn("NEPI_MGR: App path %s does not exist",  search_path)
    # Check for node file
    purge_list = []
    for mgr_name in mgrs_dict.keys():
      pkg_name = mgrs_dict[mgr_name]['MGR_DICT']['pkg_name']
      mgr_file = mgrs_dict[mgr_name]['MGR_DICT']['mgr_file']
      mgr_file_path = search_path + pkg_name + "/" + mgr_file
      if os.path.exists(mgr_file_path) == False:
        rospy.logwarn("NEPI_MGR: Could not find mgr file: " + mgr_file_path)
        purge_list.mgrend(mgr_name)
      else:
        mgrs_dict[mgr_name]['MGR_DICT']['mgr_path'] = search_path + pkg_name
    for mgr_name in purge_list:
      del mgrs_dict[mgr_name]
    mgrs_dict = setFactoryAppOrder(mgrs_dict)
    return mgrs_dict


# ln = sys._getframe().f_lineno ; self.printND(mgrs_dict,ln)
def printDict(mgrs_dict):
  rospy.logwarn('NEPI_MGR: ')
  rospy.logwarn('NEPI_MGR:*******************')
  if line_num is not None:
    rospy.logwarn('NEPI_MGR: ' + str(line_num))
  rospy.logwarn('NEPI_MGR: Printing Nex App Dictionary')

  for mgr_name in mgrs_dict.keys():
    mgrs_dict = mgrs_dict[mgr_name]
    rospy.logwarn('NEPI_MGR: ')
    rospy.logwarn('NEPI_MGR: ' + mgr_name)
    rospy.logwarn(str(mgrs_dict))


def updateAppsDict(mgrs_path,mgrs_dict):
  success = True
  if mgrs_path[-1] == "/":
      mgrs_path = mgrs_path[:-1]
  get_mgrs_dict = getAppsDict(mgrs_path)
  purge_list = []
  for mgr_name in mgrs_dict.keys():
    if mgr_name not in get_mgrs_dict.keys():
      purge_list.mgrend(mgr_name)
  for mgr_name in purge_list:
    del mgrs_dict[mgr_name]

  for mgr_name in get_mgrs_dict.keys():
    if mgr_name not in mgrs_dict.keys():
      mgrs_dict[mgr_name] = get_mgrs_dict[mgr_name]
      mgrs_dict[mgr_name]['active'] = True
      mgrs_dict = moveAppBottom(mgr_name,mgrs_dict)
  return mgrs_dict

def initAppsActiveOrder(active_list,mgrs_dict):
  rvs_list = list(reversed(active_list))
  #rospy.logwarn("NEPI_MGR: got rvs_list: " + str(rvs_list))
  # First set all to inactive and no order (-1)
  for name in mgrs_dict.keys():
    mgrs_dict[name]['active'] = False
    mgrs_dict[name]['order'] = -1
  for name in rvs_list:
    mgrs_dict = activateApp(name,mgrs_dict)
    mgrs_dict = moveAppTop(name,mgrs_dict)
  return mgrs_dict


def refreshAppsDict(mgrs_path,mgrs_dict):
  success = True
  if mgrs_path[-1] == "/":
      mgrs_path = mgrs_path[:-1]
  get_mgrs_dict = getAppsDict(mgrs_path)
  for mgr_name in get_mgrs_dict.keys():
    if mgr_name in mgrs_dict.keys():
      get_mgrs_dict[mgr_name]['order'] = mgrs_dict[mgr_name]['order']
      get_mgrs_dict[mgr_name]['active'] = mgrs_dict[mgr_name]['active']
  return get_mgrs_dict


def getAppsByActive(mgrs_dict):
  active_dict = dict()
  for mgr_name in mgrs_dict.keys():
    mgrs_dict = mgrs_dict[mgr_name]
    App_active = mgrs_dict['active']
    if App_active == True:
      active_dict[mgr_name] = mgrs_dict
  return active_dict

def setFactoryAppOrder(mgrs_dict):
  mgr_names = list(mgrs_dict.keys())
  mgr_names_sorted = sorted(mgr_names)
  for mgr_name in mgrs_dict.keys():
    mgrs_dict[mgr_name]['order']=mgr_names_sorted.index(mgr_name)
  return mgrs_dict


def moveAppTop(mgr_name,mgrs_dict):
  if mgr_name in mgrs_dict.keys():
    current_ordered_list = getAppsOrderedList(mgrs_dict)
    current_order = current_ordered_list.index(mgr_name)
    if current_order > 0:
      new_order = 0
      mgrs_dict = setAppOrder(mgr_name,new_order,mgrs_dict)
  return mgrs_dict

def moveAppBottom(mgr_name,mgrs_dict):
  if mgr_name in mgrs_dict.keys():
    current_ordered_list = getAppsOrderedList(mgrs_dict)
    current_order = current_ordered_list.index(mgr_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = len(current_ordered_list) - 1
      mgrs_dict = setAppOrder(mgr_name,new_order,mgrs_dict)
  return mgrs_dict

def moveAppUp(mgr_name,mgrs_dict):
  if mgr_name in mgrs_dict.keys():
    current_ordered_list = getAppsOrderedList(mgrs_dict)
    current_order = current_ordered_list.index(mgr_name)
    if current_order > 0:
      new_order = current_order -1
      mgrs_dict = setAppOrder(mgr_name,new_order,mgrs_dict)
  return mgrs_dict

def moveAppDown(mgr_name,mgrs_dict):
  if mgr_name in mgrs_dict.keys():
    current_ordered_list = getAppsOrderedList(mgrs_dict)
    current_order = current_ordered_list.index(mgr_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = current_order + 1
      mgrs_dict = setAppOrder(mgr_name,new_order,mgrs_dict)
  return mgrs_dict

def setAppOrder(mgr_name,new_order,mgrs_dict):
  if mgr_name in mgrs_dict.keys():
    ordered_list = getAppsOrderedList(mgrs_dict)
    current_order = ordered_list.index(mgr_name)
    App_entry = ordered_list.pop(current_order)
    ordered_list.insert(new_order,App_entry)
    for mgr_name in mgrs_dict.keys():
      mgrs_dict[mgr_name]['order'] = ordered_list.index(mgr_name)
  return mgrs_dict

def setAppMsg(mgr_name,msg,mgrs_dict):
  if mgr_name in mgrs_dict.keys():
    mgrs_dict[mgr_name]['msg'] = str(msg)
  return mgrs_dict

    

def getAppsOrderedList(mgrs_dict):
  name_list = []
  order_list = []
  ordered_name_list = []
  indexes = []
  for mgr_name in mgrs_dict.keys():
    name_list.mgrend(mgr_name)
    mgr_dict = mgrs_dict[mgr_name]
    order = mgr_dict['order']
    if order == -1:
      order = 100000
    while(order in order_list):
      order += 0.1
    order_list.mgrend(order)
    s = list(sorted(order_list))
    indexes = [s.index(x) for x in order_list]
  for val in order_list:
    ordered_name_list.mgrend(0)
  for i,index in enumerate(indexes):
    ordered_name_list[index] = name_list[i]
  return ordered_name_list
  
def getAppsGroupList(mgrs_dict):
  group_list = []
  for mgr_name in mgrs_dict.keys():
    try:
      group_list.mgrend(mgrs_dict[mgr_name]['MGR_DICT']['group_name'])
    except Exception as e:
      nepi_msg.printMsgInfo(self,"Failed to get group name from mgr dict: " + mgr_name + " " + str(e))
  return group_list

def getAppsActiveOrderedList(mgrs_dict):
  ordered_name_list = getAppsOrderedList(mgrs_dict)
  #rospy.logwarn("MGR_MGR: ordered list: " + str(ordered_name_list))
  ordered_active_list =[]
  for mgr_name in ordered_name_list:
    active = mgrs_dict[mgr_name]['active']
    if active:
      ordered_active_list.mgrend(mgr_name)
  return ordered_active_list

def getAppsRuiActiveList(mgrs_dict):
  ordered_name_list = getAppsOrderedList(mgrs_dict)
  rui_active_list =[]
  for mgr_name in ordered_name_list:
    active = mgrs_dict[mgr_name]['active']
    if active and mgrs_dict[mgr_name]['RUI_DICT']['rui_menu_name'] != "None":
      rui_active_list.mgrend(mgr_name)
  return rui_active_list



def getAppInfoFilesList(mgrs_path):
  mgrs_list = []
  if mgrs_path != '':
    if os.path.exists(mgrs_path):
      [file_list, num_files] = nepi_ros.get_file_list(mgrs_path,"py")
      for f in file_list:
        mgrs_list.mgrend(f.split(".")[0])
  return mgrs_list

  
def getAppPackagesList(install_path):
  pkg_list = []
  if install_path != '':
    if os.path.exists(install_path):
      [file_list, num_files] = nepi_ros.get_file_list(install_path,"zip")
      for pkg in file_list:
        pkg_list.mgrend(os.path.basename(pkg))
  return pkg_list


 
def activateAllApps(mgrs_dict):
  success = True
  for mgr_name in mgrs_dict.keys():
    mgrs_dict = activateApp(mgr_name,mgrs_dict)
  return mgrs_dict

def disableAllApps(mgrs_dict):
  success = True
  for mgr_name in mgrs_dict.keys():
    mgrs_dict = disableApp(mgr_name,mgrs_dict)
  return mgrs_dict

def activateApp(mgr_name,mgrs_dict):
    if mgr_name not in mgrs_dict.keys():
      rospy.logwarn("NEPI_MGR: App %s for activate request does not exist", mgr_name)
      return mgrs_dict
    mgrs_dict[mgr_name]['active'] = True
    return mgrs_dict

def disableApp(mgr_name,mgrs_dict):
    if mgr_name not in mgrs_dict.keys():
      rospy.logwarn("NEPI_MGR: App %s for removal request does not exist", mgr_name)
      return mgrs_dict
    mgrs_dict[mgr_name]['active'] = False
    return mgrs_dict

'''
def installAppPkg(pkg_name,mgrs_dict,install_from_path,install_to_path):
    success = True
    if os.path.exists(install_from_path) == False:
      rospy.logwarn("NEPI_MGR: Install package source folder does not exist %s", install_from_path)
      return False, mgrs_dict
    if os.path.exists(install_to_path) == False:
      rospy.logwarn("NEPI_MGR: Install package destination folder does not exist %s", install_to_path)
      return False, mgrs_dict
    pkg_list = getAppPackagesList(install_from_path)
    if pkg_name not in pkg_list:
      rospy.logwarn("NEPI_MGR: Install package for %s not found in install folder %s", pkg_name, install_from_path)
      return False, mgrs_dict
    os_user = getpass.getuser()
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_from_path)
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_to_path)
    pkg_path = install_from_path + "/" + pkg_name
    mgr_path = install_to_path
    try:
      pkg = zipfile.ZipFile(pMGR_NAME = 'AI_TARGETING' # Use in display menus
DESCRIPTION = 'Application for advanced targeting of AI detected objects'
LAUNCH_FILE = 'nepi_mgr_ai_targeting.launch'
PKG_NAME = 'nepi_mgr_ai_targeting'
NODE_NAME = 'mgr_ai_targeting'
RUI_FILES = ['NepiAppAiTargeting.js','NepiAppAiTargetingControls.js']
RUI_MAIN_FILE = "NepiAppAiTargeting.js"
RUI_MAIN_CLASS = "NepiAppAiTargeting"
RUI_MENU_NAME = "AI Targeting"
      mgr_files = []
      for pkg_file in pkg_files:
        mgr_file = mgr_path + "/" + pkg_file
        mgr_files.mgrend(mgr_file)
      for file in mgr_files:
        if os.path.exists(file):
          try:
            os.remove(file)
          except Exception as e:
            success = False
            rospy.logwarn(str(e))
      if success:
        # Unzip the package to the App path
        with zipfile.ZipFile(pkg_path,"r") as zip_ref:
          zip_ref.extractall(mgr_path)
        # Check for success
        for f in mgr_files:
          if os.path.exists(f) == False:
            os.system('chown -R ' + 'nepi:nepi' + ' ' + f)
            success = False
    mgrs_dict = updateAppsDict(mgr_path,mgrs_dict)
    return success, mgrs_dict 



def removeApp(mgr_name,mgrs_dict,backup_path = None):
    success = True
    if mgr_name not in mgrs_dict.keys():
      rospy.logwarn("NEPI_MGR: App %s for removal request does not exist", mgr_name)
      return False, mgrs_dict
    mgrs_dict = mgrs_dict[mgr_name]

    launch_file = mgrs_dict['launch_file_name']
    info_file = launch_file.replace(".launch",".py")
    mgr_files[launch_file,info_file]

    launch_path = mgrs_dict['launch_path']

    App_file_list = []
    for i,mgr_file in enumerate(mgr_files):
      if mgr_file != 'None' and App_names[i] == mgr_name:
        path = launch_path
        file = mgr_files[i]
        filepath = path + '/' + file
        if os.path.exists(filepath) == False:
          success = False
        if success:
          os.system('chown -R ' + 'nepi:nepi' + ' ' + path)
          App_file_list.mgrend(filepath)
          # Create an install package from App files
    rospy.loginfo("NEPI_MGR: Removing App files: " + str(App_file_list))      
    if backup_path != None:
      if os.path.exists(backup_path) == False:
        backup_path = None
      else:
        os.system('chown -R ' + 'nepi:nepi' + ' ' + backup_path)
        zip_file = backup_path + "/" + mgr_name + ".zip"
        rospy.loginfo("NEPI_MGR: Backing up removed file to: " + zip_file)
        try:
          zip = zipfile.ZipFile(zip_file, "w", zipfile.ZIP_DEFLATED)
          for file_path in App_file_list:
            zip.write(file_path, os.path.basename(file_path), compress_type=zipfile.ZIP_DEFLATED)
          zip.close()
          zip = None
        except Exception as e:
          rospy.logwarn("NEPI_MGR: Failed to backup App: " + str(e))
          if os.path.exists(zip_file) == True:
            try:
              zip.close()
            except Exception as e:
              rospy.logwarn(str(e))
            try:
              os.remove(file_path)
            except Exception as e:
              rospy.logwarn(str(e))
        for file_path in App_file_list:
          if os.path.exists(file_path) == True:
            try:
              os.remove(file_path)
            except Exception as e:
              success = False
              rospy.logwarn("NEPI_MGR: Failed to remove App file: " + file_path + " " + str(e))

    if success:
      del mgrs_dict[mgr_name]
    return success, mgrs_dict
'''


def launchAppNode(pkg_name, file_name, ros_node_name, device_path = None):
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
  
def checkAppNode(node_namespace,sub_process):
    running = True
    if sub_process.poll() is None:
      running = False
    return running

def killAppNode(node_namespace,sub_process):
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
        

def importAppClass(file_name,file_path,module_name,class_name):
      module_class = None
      success = False
      msg = "failed"
      file_list = os.listdir(file_path)
      if file_name in file_list:
        sys.path.mgrend(file_path)
        try:
          module = importlib.import_module(module_name)
          try:
            module_class = getattr(module, class_name)
            success = True
            msg = 'success'
          except Exception as e:
            rospy.logwarn("NEPI_MGR: Failed to import class %s from module %s with exception: %s", class_name, module_name, str(e))
        except Exception as e:
            rospy.logwarn("NEPI_MGR: Failed to import module %s with exception: %s", module_name, str(e))
      else:
        rospy.logwarn("NEPI_MGR: Failed to find file %s in path %s for module %s", file_name, file_path, module_name)
      return success, msg, module_class


def unimportAppClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            rospy.loginfo("NEPI_MGR: Failed to clordered_unimport module: " + module_name)
        if module_name in sys.modules:
          success = False
    return success
