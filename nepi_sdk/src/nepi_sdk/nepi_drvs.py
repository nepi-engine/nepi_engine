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
import sys
import zipfile
import getpass
import importlib
import subprocess


import numpy as np
import time
import usb
import copy
from serial.tools import list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_drvs"
logger = Logger(log_name = log_name)

#######################
### Driver Utility Functions

DRIVERS_FOLDER = '/opt/nepi/nepi_engine/lib/nepi_drivers'
DRIVERS_CFG_FOLDER = '/mnt/nepi_storage/user_cfg'
DRIVER_FILE_TYPES = ['Node','Driver', 'Discovery']
DRIVER_KEYS = ['node','driver','discovery']


def getDriversDict(search_path):
    drvs_dict = dict()
    # Find driver files
    ind = 0
    if os.path.exists(search_path):
        if search_path[-1] == "/":
            search_path = search_path[:-1]
        sys.path.append(search_path)
        #logger.log_info("Searching for drivers in path: " + search_path)
        for f in os.listdir(search_path):
          if f.endswith(".yaml") and f.find("params") != -1: 
                  try:
                    file_path = os.path.join(search_path,f)
                    read_dict = nepi_utils.read_yaml_2_dict(file_path)
                  except Exception as e:
                    logger.log_warn("Failed to import param file: " + f + " " + str(e))
                    continue
                  if 'driver' in read_dict.keys():
                    new_dict = read_dict['driver']
                    try:
                      pkg_name = new_dict['pkg_name']
                      #logger.log_warn("Got drvs dict: " + str(new_dict))
                      new_dict['param_file_name'] = os.path.basename(f)
                      new_dict['path'] = DRIVERS_FOLDER
                      new_dict['order'] = -1
                      new_dict['active'] = False
                      new_dict['msg'] = ""
                      if 'group_id' not in new_dict.keys():
                        new_dict['group_id'] = 'None'
                      if 'display_name' not in new_dict.keys():
                        new_dict['display_name'] = pkg_name      
                      if 'DRIVER_DICT' not in new_dict.keys():
                        new_dict['DRIVER_DICT'] = dict()
                        new_dict['DRIVER_DICT']['file_name'] != 'None'
                      if 'DISCOVERY_DICT' not in new_dict.keys():
                        new_dict['DISCOVERY_DICT'] = dict()
                        new_dict['DISCOVERY_DICT']['file_name'] != 'None'
                        new_dict['DISCOVERY_DICT']['node_name'] = 'None'
                      else:
                        node_name = new_dict['DISCOVERY_DICT']['file_name'].split('.')[0]
                        new_dict['DISCOVERY_DICT']['node_name'] = node_name
                      if 'OPTIONS' not in new_dict['DISCOVERY_DICT'].keys():
                        new_dict['DISCOVERY_DICT']['OPTIONS'] = 'None'
                      if new_dict['DISCOVERY_DICT']['OPTIONS'] == 'None':
                        new_dict['DISCOVERY_DICT']['OPTIONS'] = dict()
                        new_dict['DISCOVERY_DICT']['OPTIONS']['None'] = dict()
                        new_dict['DISCOVERY_DICT']['OPTIONS']['None']['type'] = 'None'
                        new_dict['DISCOVERY_DICT']['OPTIONS']['None']['options'] = []
                        new_dict['DISCOVERY_DICT']['OPTIONS']['None']['default'] = 'None'
                      options_dict = new_dict['DISCOVERY_DICT']['OPTIONS']
                      # Clean up options
                      for option_name in options_dict.keys():
                        option_dict = new_dict['DISCOVERY_DICT']['OPTIONS'][option_name]
                        if 'options' not in option_dict.keys():
                          new_dict['DISCOVERY_DICT']['OPTIONS'][option_name]['options'] = []
                        default_val = options_dict[option_name]['default']
                        new_dict['DISCOVERY_DICT']['OPTIONS'][option_name]['value'] = default_val

                      #logger.log_warn("Got driver dict from file: " + str(new_dict))

                      drvs_dict[pkg_name] = new_dict
                    except Exception as e:
                      logger.log_warn("Failed to get driver params from file: " + f + " " + str(e))
                  else:
                    logger.log_warn("Driver dict key not found in file: " + f )
    else:
        logger.log_warn("Driver path does not exist: " +  search_path)
    # Now assign factory orders
    drvs_dict = setFactoryDriverOrder(drvs_dict)
    return drvs_dict

def printDict(drvs_dict):
  logger.log_warn('')
  logger.log_warn('*******************')
  logger.log_warn('Printing Drv Driver Dictionary')
  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    logger.log_warn('')
    logger.log_warn(drv_name)
    logger.log_warn(str(drv_dict))


def refreshDriversDict(drivers_path,drvs_dict):
  success = True
  if drivers_path[-1] == "/":
      drivers_path = drivers_path[:-1]
  get_drvs_dict = getDriversDict(drivers_path)
  #logger.log_warn('Updating Drvs Dict: ' + str(drvs_dict))
  #logger.log_warn('From Get Dict: ' + str(get_drvs_dict))
  for drv_name in get_drvs_dict.keys():
    if drv_name not in drvs_dict.keys():
      get_drvs_dict[drv_name]['active'] = False
    else:
      #logger.log_warn('')
      #logger.log_warn('Updating drv: ' + drv_name)
      #logger.log_warn('Updating drv: ' + str(drvs_dict[drv_name]))
      get_drvs_dict[drv_name]['active'] = drvs_dict[drv_name]['active']
      get_drvs_dict[drv_name]['order'] = drvs_dict[drv_name]['order']
      drv_dict = drvs_dict[drv_name]
      if 'OPTIONS' in drv_dict.keys():
        options_dict = drvs_dict[drv_name]['DISCOVERY_DICT']['OPTIONS']
        get_options_dict = get_drvs_dict[drv_name]['DISCOVERY_DICT']['OPTIONS']
        if get_options_dict != "None":
          if options_dict != "None":
            for get_option_name in get_options_dict.keys():
              if get_option_name in options_dict.keys():
                cur_value = options_dict[get_option_name]['value']
                if cur_value in options_dict[get_option_name]['options']:
                  get_drvs_dict[drv_name]['DISCOVERY_DICT']['OPTIONS'][get_option_name] = cur_value



  #logger.log_warn('Updated to: ' + str(get_drvs_dict))
  return get_drvs_dict

def initDriversActive(active_list,drvs_dict):
  rvs_list = list(reversed(active_list))  
  # First set all to inactive
  for name in drvs_dict.keys():
    drvs_dict[name]['active'] = False
  for name in active_list:
    if name in drvs_dict.keys():
      drvs_dict[name]['active'] = True
  return drvs_dict

def getDriversByActive(drvs_dict):
  active_dict = dict()
  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    driver_active = drv_dict['active']
    if driver_active == True:
      active_dict[drv_name] = drv_dict
  return active_dict


def getDriversByType(type,drvs_dict):
  type_dict = dict()
  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    driver_type = drv_dict['type']
    if driver_type == type:
      type_dict[drv_name] = drv_dict
  return type_dict


def setFactoryDriverOrder(drvs_dict):
  indexes = []
  factory_indexes = []
  man_ind = 0
  call_ind = 1000
  run_ind = call_ind * 1000
  launch_ind = run_ind * 1000
  catch_ind = launch_ind * 1000
  order = catch_ind
  for drv_name in drvs_dict.keys():
    drv_dict = drvs_dict[drv_name]
    if 'DISCOVERY_DICT' in drv_dict.keys():
      if drvs_dict[drv_name]['DISCOVERY_DICT']["file_name"] != "None":
        discovery_proc = drv_dict['DISCOVERY_DICT']['process']
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
  for i, drv_name in enumerate(drvs_dict.keys()):
    drvs_dict[drv_name]['order'] = factory_indexes[i]
  return drvs_dict


def moveDriverTop(drv_name,drvs_dict):
  if drv_name in drvs_dict.keys():
    current_ordered_list = getDriversOrderedList(drvs_dict)
    current_order = current_ordered_list.index(drv_name)
    if current_order > 0:
      new_order = 0
      drvs_dict = setDriverOrder(drv_name,new_order,drvs_dict)
  return drvs_dict

def moveDriverBottom(drv_name,drvs_dict):
  if drv_name in drvs_dict.keys():
    current_ordered_list = getDriversOrderedList(drvs_dict)
    current_order = current_ordered_list.index(drv_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = len(current_ordered_list) - 1
      drvs_dict = setDriverOrder(drv_name,new_order,drvs_dict)
  return drvs_dict

def moveDriverUp(drv_name,drvs_dict):
  if drv_name in drvs_dict.keys():
    current_ordered_list = getDriversOrderedList(drvs_dict)
    current_order = current_ordered_list.index(drv_name)
    if current_order > 0:
      new_order = current_order -1
      drvs_dict = setDriverOrder(drv_name,new_order,drvs_dict)
  return drvs_dict

def moveDriverDown(drv_name,drvs_dict):
  if drv_name in drvs_dict.keys():
    current_ordered_list = getDriversOrderedList(drvs_dict)
    current_order = current_ordered_list.index(drv_name)
    if current_order < (len(current_ordered_list)-1):
      new_order = current_order + 1
      drvs_dict = setDriverOrder(drv_name,new_order,drvs_dict)
  return drvs_dict

def setDriverOrder(drv_name,new_order,drvs_dict):
  if drv_name in drvs_dict.keys():
    ordered_list = getDriversOrderedList(drvs_dict)
    current_order = ordered_list.index(drv_name)
    driver_entry = ordered_list.pop(current_order)
    ordered_list.insert(new_order,driver_entry)
    for drv_name in drvs_dict.keys():
      drvs_dict[drv_name]['order'] = ordered_list.index(drv_name)
  return drvs_dict

def setDriverMsg(drv_name,msg,drvs_dict):
  if drv_name in drvs_dict.keys():
    drvs_dict[drv_name]['msg'] = str(msg)
  return drvs_dict

    

def getDriversOrderedList(drvs_dict):
  name_list = []
  order_list = []
  ordered_name_list = []
  for drv_name in drvs_dict.keys():
    name_list.append(drv_name)
    drv_dict = drvs_dict[drv_name]
    order = drv_dict['order']
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

def getDriversActiveOrderedList(drvs_dict):
  ordered_name_list = getDriversOrderedList(drvs_dict)
  #logger.log_warn("Drivers Ordered List: " + str(ordered_name_list))
  ordered_active_list =[]
  for drv_name in ordered_name_list:
    #logger.log_warn("Drivers Drv Entry: " + str(drvs_dict[drv_name]))
    active = drvs_dict[drv_name]['active']
    if active:
      ordered_active_list.append(drv_name)
  return ordered_active_list






def getDriverFilesList(drivers_path):
  drivers_list = []
  file_list = []
  if drivers_path != '':
    if os.path.exists(drivers_path):
      [file_list, num_files] = nepi_utils.get_file_list(drivers_path,"py")
  for f in file_list:
    drivers_list.append(f.split(".")[0])
  return drivers_list
  
def getDriverPackagesList(install_path):
  pkg_list = []
  file_list = []
  if install_path != '':
    if os.path.exists(install_path):
      [file_list, num_files] = nepi_utils.get_file_list(install_path,"zip")
  for pkg in file_list:
    pkg_list.append(os.path.basename(pkg))
  return pkg_list


 
def activateAllDrivers(drvs_dict):
  success = True
  for drv_name in drvs_dict.keys():
    drvs_dict = activateDriver(drv_name,drvs_dict)
  return drvs_dict

def activateDriver(drv_name,drvs_dict):
    if drv_name not in drvs_dict.keys():
      logger.log_warn("Driver for removal request does not exist: " + drv_name)
      return drvs_dict
    drvs_dict[drv_name]['active'] = True
    return drvs_dict

def disableAllDrivers(drvs_dict):
  success = True
  for drv_name in drvs_dict.keys():
    drvs_dict = disableDriver(drv_name,drvs_dict)
  return drvs_dict

def disableDriver(drv_name,drvs_dict):
    if drv_name not in drvs_dict.keys():
      logger.log_warn("Driver for removal request does not exist: " + drv_name)
      return drvs_dict
    drvs_dict[drv_name]['active'] = False
    return drvs_dict

def installDriverPkg(pkg_name,drvs_dict,install_from_path,drivers_path):
    success = True
    if install_from_path[-1] == "/":
      install_from_path = install_from_path[:-1]
    if drivers_path[-1] == "/":
      search_path = drivers_path[:-1]

    if os.path.exists(install_from_path) == False:
      logger.log_warn("Install package source folder does not exist: " + install_from_path)
      return False, drvs_dict
    if os.path.exists(drivers_path) == False:
      logger.log_warn("Install drivers destination folder does not exist: " + drivers_path)
      return False, drvs_dict
    pkg_list = getDriverPackagesList(install_from_path)
    if pkg_name not in pkg_list:
      logger.log_warn("Install package for not found in install folder: " + pkg_name + " " + install_from_path)
      return False, drvs_dict
    os_user = getpass.getuser()
    os.system('chown -R ' + 'nepi:nepi' + ' ' + install_from_path)
    os.system('chown -R ' + 'nepi:nepi' + ' ' + drivers_path)
    os.system('chown -R ' + 'nepi:nepi' + ' ' + params_path)
    pkg_path = install_from_path + "/" + pkg_name
    driver_path = drivers_path
    try:
      pkg = zipfile.ZipFile(pkg_path)
      pkg_files = pkg.namelist()

    except Exception as e:
      logger.log_warn("" + str(e))
      success = False
    if success:
      # Create a list of files
      driver_files = []
      for pkg_file in pkg_files:
        if pkg_file.endswith(".yaml"):
          driver_file = driver_path + "/" + pkg_file
        driver_files.append(driver_file)
      for file in driver_files:
        if os.path.exists(file):
          try:
            os.remove(file)
          except Exception as e:
            success = False
            logger.log_warn(str(e))
      if success:
        # Unzip the package to the Driver path
        with zipfile.ZipFile(pkg_path,"r") as zip_ref:
          zip_ref.extractall(driver_path)
        # Check for success
        for f in driver_files:
          if os.path.exists(f) == False:
            success = False
          else:
            os.system('chown -R ' + 'nepi:nepi' + ' ' + f)
            new_f = os.path.join(drivers_path,os.path.basename(f))
            try:
              os.rename(f,new_f)
              os.system('chown -R ' + 'nepi:nepi' + ' ' + new_f)
            except Exception as e:
              ospy.logwarn("NEPI_DRV: Failed to move param file to new location: " + new_f + " " + str(e))
              success = False
    drvs_dict = updateDriversDict(driver_path,drvs_dict)
    return success, drvs_dict 



def removeDriver(drv_name,drvs_dict,drivers_path,backup_path = None):
    success = True   
    if drv_name not in drvs_dict.keys():
      logger.log_warn("Driver for removal request does not exist: " + drv_name)
      return False, drvs_dict
    drv_dict = drvs_dict[drv_name]

    param_file = os.path.join(drivers_path,drv_dict['param_file_name'])
    node_file = os.path.join(drivers_path,drv_dict['NODE_DICT']['file_name'])
    discovery_file = os.path.join(drivers_path,drv_dict['DISCOVERY_DICT']['file_name'])
    driver_file = os.path.join(drivers_path,drv_dict['DRIVER_DICT']['file_name'])

    file_list = []
    file_list.append(param_file)
    file_list.append(node_file)
    file_list.append(discovery_file)
    file_list.append(driver_file)

    os_user = getpass.getuser()
    for i,file in enumerate(file_list):
      if file == 'None' or os.path.exists(file) == False:
        file_list.remove(file)
      else:
        os.system('chown -R ' + 'nepi:nepi' + ' ' + filepath)
    # Create an backup package from driver files
    logger.log_info("Removing driver files: " + str(file_list))      
    if backup_path != None:
      if backup_path[-1] == "/":
        backup_path = backup_path[:-1]
      if os.path.exists(backup_path) == False:
        backup_path = None
      else:
        os.system('chown -R ' + 'nepi:nepi' + ' ' + backup_path)
        zip_file = backup_path + "/" + drv_name + ".zip"
        logger.log_info("Backing up removed file to: " + zip_file)
        try:
          zip = zipfile.ZipFile(zip_file, "w", zipfile.ZIP_DEFLATED)
          for file in file_list:
            zip.write(file, os.path.basename(file), compress_type=zipfile.ZIP_DEFLATED)
          zip.close()
          zip = None
        except Exception as e:
          logger.log_warn("Failed to backup driver: " + str(e))
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
              logger.log_warn("Failed to remove driver file: " + file + " " + str(e))

    if success:
      del drvs_dict[drv_name]
    return success, drvs_dict



def launchDriverNode(file_name, ros_node_name, device_path = None):
  sub_process = None
  msg = 'Success'
  success = False
  if device_path is None:
    device_node_run_cmd = ['rosrun', 'nepi_drivers', file_name, '__name:=' + ros_node_name]
  else:
    device_node_run_cmd = ['rosrun', 'nepi_drivers', file_name, '__name:=' + ros_node_name, '_device_path:=' + device_path]
  try:
    sub_process = subprocess.Popen(device_node_run_cmd)
    success = True
  except Exception as e:
    msg = str("Failed to launch node with exception: " + ros_node_name + " " + str(e))
    logger.log_warn(msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("Failed to start " + device_node_name + " via " + " ".join(x for x in device_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
      logger.log_error(msg)
      sub_process = None
      success = False
  return success, msg, sub_process
  
def checkDriverNode(node_namespace,sub_process):
    running = True
    if sub_process.poll() is None:
      running = False
    return running


def killDriverNode(node_namespace,sub_process):
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
        

def importDriverClass(file_name,file_path,module_name,class_name):
      module_class = None
      success = False
      msg = "failed"
      file_list = os.listdir(file_path)
      logger.log_warn("Looking for driver file: " + str(file_name) + " : " + str(file_list))
      if file_name in file_list:
        sys.path.append(file_path)
        try:
          module = importlib.import_module(module_name)
          try:
            module_class = getattr(module, class_name)
            success = True
            msg = 'success'
          except Exception as e:
            logger.log_warn("Failed to import class from module with exception: " + class_name +" "+ module_name +" "+ str(e))
        except Exception as e:
            logger.log_warn("Failed to import module with exception: " + module_name +" "+ str(e))
      else:
        logger.log_warn("Failed to find file in path: " + file_name +" "+ file_path)
      return success, msg, module_class


def unimportDriverClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            logger.log_info("Failed to clordered_unimport module: " + module_name)
        if module_name in sys.modules:
          success = False
    return success


def checkLoadConfigFile(node_name):
  config_folder = DRIVERS_CFG_FOLDER
  config_file = os.path.join(config_folder, node_name + ".yaml")
  node_namespace = os.path.join(nepi_sdk.get_base_namespace(), node_name)
  if os.path.exists(config_file):
    print("Loading parameters from " + config_file + " to " + node_namespace)
    rosparam_load_cmd = ['rosparam', 'load', config_file, node_namespace]
    subprocess.run(rosparam_load_cmd)
  else:
    print("No config file found for " + node_name + " in " + DRIVERS_CFG_FOLDER)

#######################
### Serial Port Utility Functions

STANDARD_BUAD_RATES = [110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

SERIAL_PORT_DICT_ENTRY = dict(    vendor_id = 0,
                        product_id = 0,
                        manf_str = "None",
                        buad_rates = []
                   )
                  

def getSerialPortDict():
  port_dict = dict()

  devs = usb.core.find(find_all=True)
  port = list_ports.comports()
  product_id = 0
  for port in sorted(port):
    entry = copy.deepcopy(SERIAL_PORT_DICT_ENTRY)
    entry["vender_id"] = port.vid
    for dev in devs:
      if dev.idVendor == port.vid:
        product_id = dev.idProduct
        break
    entry["product_id"] = product_id
    entry["manf_str"] = port.manufacturer
    port_dict[port.device] = entry
  return port_dict

### Function for checking if port is available
def checkSerialPorts(port_str):
    success = False
    port = list_ports.comports()
    for loc, desc, hwid in sorted(port):
      if loc == port_str:
        success = True
    return success





  
