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

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
  
from nepi_sdk.nepi_ros import logger as Logger
log_name = "nepi_apps"
logger = Logger(log_name = log_name)


#***************************
# NEPI Apps utility functions
APPS_SHARE_PATH = '/opt/nepi/ros/share/nepi_apps'
NEPI_PKG_FOLDER = '/opt/nepi/ros/lib/'

def getAppsDict(search_path):
    apps_dict = dict()
    # Find App files
    ind = 0
    if os.path.exists(search_path):
        if search_path[-1] == "/":
            search_path = search_path[:-1]
        sys.path.append(search_path)
        logger.log_info("Searching for Apps in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".yaml") and f.find("params") != -1: 
              file_path = os.path.join(search_path,f)
              #logger.log_warn("Loading app dict from file: " + str(file_path))
              try:
                new_dict = nepi_utils.read_yaml2dict(file_path)
                #logger.log_warn("Got app dict: " + str(new_dict))
                new_dict['order'] = -1
                new_dict['subprocess'] = ""
                new_dict['active'] = False
                new_dict['msg'] = ""
                if 'license_type' not in new_dict['APP_DICT'].keys():
                  new_dict['APP_DICT']['license_type'] = "Not Provided"
                if 'license_link' not in new_dict['APP_DICT'].keys():
                  new_dict['APP_DICT']['license_link'] = ""
                app_name = new_dict['APP_DICT']['pkg_name']
                #logger.log_warn("Adding dict: " + str(new_dict) + " with app_name: " + str(app_name))
                apps_dict[app_name] = new_dict   
              except Exception as e:
                logger.log_warn("Failed to import param file: " + file_path + " " + str(e))
    else:
        logger.log_warn("App path does not exist: " + search_path)
    # Check for node file

    purge_list = []
    for app_name in apps_dict.keys():
      pkg_name = apps_dict[app_name]['APP_DICT']['pkg_name']
      app_file = apps_dict[app_name]['APP_DICT']['app_file']
      app_file_path = os.path.join(NEPI_PKG_FOLDER ,pkg_name, app_file)
      if os.path.exists(app_file_path) == False:
        logger.log_warn("Could not find app file: " + app_file_path)
        purge_list.append(app_name)
      else:
        apps_dict[app_name]['APP_DICT']['app_path'] = NEPI_PKG_FOLDER + pkg_name
    for app_name in purge_list:
      del apps_dict[app_name]
    apps_dict = setFactoryAppOrder(apps_dict)
    return apps_dict


# ln = sys._getframe().f_lineno ; self.printND(apps_dict,ln)
def printDict(apps_dict):
  logger.log_warn('')
  logger.log_warn('*******************')
  if line_num is not None:
    logger.log_warn('' + str(line_num))
  logger.log_warn('Printing Nex App Dictionary')

  for app_name in apps_dict.keys():
    apps_dict = apps_dict[app_name]
    logger.log_warn('')
    logger.log_warn('' + app_name)
    logger.log_warn(str(apps_dict))


def refreshAppsDict(apps_path,apps_dict):
  success = True
  if apps_path[-1] == "/":
      apps_path = apps_path[:-1]
  get_apps_dict = getAppsDict(apps_path)
  for app_name in get_apps_dict.keys():
    if app_name in apps_dict.keys():
      get_apps_dict[app_name]['order'] = apps_dict[app_name]['order']
      get_apps_dict[app_name]['active'] = apps_dict[app_name]['active']
  return get_apps_dict

def initAppsActiveOrder(active_list,apps_dict):
  rvs_list = list(reversed(active_list))
  #logger.log_warn("got rvs_list: " + str(rvs_list))
  # First set all to inactive and no order (-1)
  for name in apps_dict.keys():
    apps_dict[name]['active'] = False
    apps_dict[name]['order'] = -1
  for name in rvs_list:
    apps_dict = activateApp(name,apps_dict)
    apps_dict = moveAppTop(name,apps_dict)
  return apps_dict


def getAppsByActive(apps_dict):
  active_dict = dict()
  for app_name in apps_dict.keys():
    apps_dict = apps_dict[app_name]
    App_active = apps_dict['active']
    if App_active == True:
      active_dict[app_name] = apps_dict
  return active_dict

def setFactoryAppOrder(apps_dict):
  app_names = list(apps_dict.keys())
  app_names_sorted = sorted(app_names)
  for app_name in apps_dict.keys():
    apps_dict[app_name]['order']=app_names_sorted.index(app_name)
  return apps_dict


def moveAppTop(app_name,apps_dict):
  if app_name in apps_dict.keys():
    current_ordered_list = getAppsOrderedList(apps_dict)
    current_order = current_ordered_list.index(app_name)
    if current_order > 0:
      new_order = 0
      apps_dict = setAppOrder(app_name,new_order,apps_dict)
  return apps_dict

def moveAppBottom(app_name,apps_dict):
  if app_name in apps_dict.keys():
    current_ordered_list = getAppsOrderedList(apps_dict)
    current_order = current_ordered_list.index(app_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = len(current_ordered_list) - 1
      apps_dict = setAppOrder(app_name,new_order,apps_dict)
  return apps_dict

def moveAppUp(app_name,apps_dict):
  if app_name in apps_dict.keys():
    current_ordered_list = getAppsOrderedList(apps_dict)
    current_order = current_ordered_list.index(app_name)
    if current_order > 0:
      new_order = current_order -1
      apps_dict = setAppOrder(app_name,new_order,apps_dict)
  return apps_dict

def moveAppDown(app_name,apps_dict):
  if app_name in apps_dict.keys():
    current_ordered_list = getAppsOrderedList(apps_dict)
    current_order = current_ordered_list.index(app_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = current_order + 1
      apps_dict = setAppOrder(app_name,new_order,apps_dict)
  return apps_dict

def setAppOrder(app_name,new_order,apps_dict):
  if app_name in apps_dict.keys():
    ordered_list = getAppsOrderedList(apps_dict)
    current_order = ordered_list.index(app_name)
    App_entry = ordered_list.pop(current_order)
    ordered_list.insert(new_order,App_entry)
    for app_name in apps_dict.keys():
      apps_dict[app_name]['order'] = ordered_list.index(app_name)
  return apps_dict

def setAppMsg(app_name,msg,apps_dict):
  if app_name in apps_dict.keys():
    apps_dict[app_name]['msg'] = str(msg)
  return apps_dict

    

def getAppsOrderedList(apps_dict):
  name_list = []
  order_list = []
  ordered_name_list = []
  indexes = []
  for app_name in apps_dict.keys():
    name_list.append(app_name)
    app_dict = apps_dict[app_name]
    order = app_dict['order']
    if order == -1:
      order = 100000
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
  
def getAppsGroupList(apps_dict):
  group_list = []
  for app_name in apps_dict.keys():
    try:
      group_list.append(apps_dict[app_name]['APP_DICT']['group_name'])
    except Exception as e:
      logger.log_warn("Failed to get group name from app dict: " + app_name + " " + str(e))
  return group_list

def getAppsActiveOrderedList(apps_dict):
  ordered_name_list = getAppsOrderedList(apps_dict)
  #logger.log_warn("ordered list: " + str(ordered_name_list))
  ordered_active_list =[]
  for app_name in ordered_name_list:
    active = apps_dict[app_name]['active']
    if active:
      ordered_active_list.append(app_name)
  return ordered_active_list

def getAppsRuiActiveList(apps_dict):
  ordered_name_list = getAppsOrderedList(apps_dict)
  rui_active_list =[]
  for app_name in ordered_name_list:
    active = apps_dict[app_name]['active']
    if 'rui_menu_name' in apps_dict[app_name]['RUI_DICT'].keys():
      rui_name = apps_dict[app_name]['RUI_DICT']['rui_menu_name']
    else:
      rui_name = "None"
    rui_active_list.append(rui_name)
  return rui_active_list



def getAppInfoFilesList(apps_path):
  apps_list = []
  file_list = []
  if apps_path != '':
    if os.path.exists(apps_path):
      [file_list, num_files] = nepi_utils.get_file_list(apps_path,"py")
      for f in file_list:
        apps_list.append(f.split(".")[0])
  return apps_list

  
def getAppPackagesList(install_path):
  pkg_list = []
  file_list = []
  if install_path != '':
    if os.path.exists(install_path):
      [file_list, num_files] = nepi_utils.get_file_list(install_path,"zip")
      for pkg in file_list:
        pkg_list.append(os.path.basename(pkg))
  return pkg_list


 
def activateAllApps(apps_dict):
  success = True
  for app_name in apps_dict.keys():
    apps_dict = activateApp(app_name,apps_dict)
  return apps_dict

def disableAllApps(apps_dict):
  success = True
  for app_name in apps_dict.keys():
    apps_dict = disableApp(app_name,apps_dict)
  return apps_dict

def activateApp(app_name,apps_dict):
    if app_name not in apps_dict.keys():
      logger.log_warn("App for activate request does not exist: " + app_name)
      return apps_dict
    apps_dict[app_name]['active'] = True
    return apps_dict

def disableApp(app_name,apps_dict):
    if app_name not in apps_dict.keys():
      logger.log_warn("App for removal request does not exist: " + app_name)
      return apps_dict
    apps_dict[app_name]['active'] = False
    return apps_dict

'''
def installAppPkg(pkg_name,apps_dict,install_from_path,install_to_path):
    success = True
    if os.path.exists(install_from_path) == False:
      logger.log_warn("Install package source folder does not exist: " + install_from_path)
      return False, apps_dict
    if os.path.exists(install_to_path) == False:
      logger.log_warn("Install package destination folder does not exist: " + install_to_path)
      return False, apps_dict
    pkg_list = getAppPackagesList(install_from_path)
    if pkg_name not in pkg_list:
      logger.log_warn("Install package not found in install folder: " + pkg_name + " " + install_from_path)
      return False, apps_dict
    os_user = getpass.getuser()
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_from_path)
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_to_path)
    pkg_path = install_from_path + "/" + pkg_name
    app_path = install_to_path
    try:
      pkg = zipfile.ZipFile(pAPP_NAME = 'AI_TARGETING' # Use in display menus
DESCRIPTION = 'Application for advanced targeting of AI detected objects'
LAUNCH_FILE = 'nepi_app_ai_targeting.launch'
PKG_NAME = 'nepi_app_ai_targeting'
NODE_NAME = 'app_ai_targeting'
RUI_FILES = ['NepiAppAiTargeting.js','NepiAppAiTargetingControls.js']
RUI_MAIN_FILE = "NepiAppAiTargeting.js"
RUI_MAIN_CLASS = "NepiAppAiTargeting"
RUI_MENU_NAME = "AI Targeting"
      app_files = []
      for pkg_file in pkg_files:
        app_file = app_path + "/" + pkg_file
        app_files.append(app_file)
      for file in app_files:
        if os.path.exists(file):
          try:
            os.remove(file)
          except Exception as e:
            success = False
            logger.log_warn(str(e))
      if success:
        # Unzip the package to the App path
        with zipfile.ZipFile(pkg_path,"r") as zip_ref:
          zip_ref.extractall(app_path)
        # Check for success
        for f in app_files:
          if os.path.exists(f) == False:
            os.system('chown -R ' + 'nepi:nepi' + ' ' + f)
            success = False
    apps_dict = updateAppsDict(app_path,apps_dict)
    return success, apps_dict 



def removeApp(app_name,apps_dict,backup_path = None):
    success = True
    if app_name not in apps_dict.keys():
      logger.log_warn("App removal request does not exist: " +  app_name)
      return False, apps_dict
    apps_dict = apps_dict[app_name]

    launch_file = apps_dict['launch_file_name']
    info_file = launch_file.replace(".launch",".py")
    app_files[launch_file,info_file]

    launch_path = apps_dict['launch_path']

    App_file_list = []
    for i,app_file in enumerate(app_files):
      if app_file != 'None' and App_names[i] == app_name:
        path = launch_path
        file = app_files[i]
        filepath = path + '/' + file
        if os.path.exists(filepath) == False:
          success = False
        if success:
          os.system('chown -R ' + 'nepi:nepi' + ' ' + path)
          App_file_list.append(filepath)
          # Create an install package from App files
    logger.log_info("Removing App files: " + str(App_file_list))      
    if backup_path != None:
      if os.path.exists(backup_path) == False:
        backup_path = None
      else:
        os.system('chown -R ' + 'nepi:nepi' + ' ' + backup_path)
        zip_file = backup_path + "/" + app_name + ".zip"
        logger.log_info("Backing up removed file to: " + zip_file)
        try:
          zip = zipfile.ZipFile(zip_file, "w", zipfile.ZIP_DEFLATED)
          for file_path in App_file_list:
            zip.write(file_path, os.path.basename(file_path), compress_type=zipfile.ZIP_DEFLATED)
          zip.close()
          zip = None
        except Exception as e:
          logger.log_warn("Failed to backup App: " + str(e))
          if os.path.exists(zip_file) == True:
            try:
              zip.close()
            except Exception as e:
              logger.log_warn(str(e))
            try:
              os.remove(file_path)
            except Exception as e:
              logger.log_warn(str(e))
        for file_path in App_file_list:
          if os.path.exists(file_path) == True:
            try:
              os.remove(file_path)
            except Exception as e:
              success = False
              logger.log_warn("Failed to remove App file: " + file_path + " " + str(e))

    if success:
      del apps_dict[app_name]
    return success, apps_dict
'''


