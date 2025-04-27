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

import glob
import sys
import subprocess
import time
import warnings
import copy

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_settings

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_ros_interfaces.msg import SystemStatus
from nepi_ros_interfaces.msg import DriversStatus, DriverStatus, UpdateState, UpdateOrder 
from nepi_ros_interfaces.srv import DriverStatusQuery, DriverStatusQueryRequest, DriverStatusQueryResponse

from nepi_ros_interfaces.msg import Setting, Settings, SettingCap, SettingCaps
from nepi_ros_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryRequest, SettingsCapabilitiesQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF
from nepi_api.connect_mgr_if_system import ConnectMgrSystemIF
from nepi_api.connect_mgr_if_config import ConnectMgrConfigIF



DRIVERS_SHARE_FOLDER = '/opt/nepi/ros/lib/nepi_drivers'
DRIVERS_INSTALL_FOLDER = '/mnt/nepi_storage/install/drivers'
USER_CFG_FOLDER = '/mnt/nepi_storage/user_cfg/ros'
#########################################

#########################################
# Node Class
#########################################

class NepiDriversMgr(object):

  NODE_LAUNCH_TIME_SEC = 10  # Will not check node status before

  UPDATE_CHECK_INTERVAL = 5
  PUBLISH_STATUS_INTERVAL = 1
  
  save_cfg_if = None
  drivers_share_folder = ''

  drivers_files = []
  drivers_ordered_list = []
  drivers_active_list = []
  drivers_install_folder = ''
  drivers_install_files = []
  drivers_install_list = []

  selected_driver = "NONE"
  active_paths_list = [] 


  discovery_node_dict = dict()
  discovery_classes_dict = dict()
  failed_class_import_list = []
  discovery_options_dict = dict()

  base_namespace = ""


  status_drivers_msg = DriversStatus()
  last_status_drivers_msg = DriversStatus()
  status_driver_msg = DriverStatus()

  active_node_dict = dict()

  drivers_status_pub = None
  driver_status_pub = None


  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "drivers_mgr" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.base_namespace = nepi_ros.get_base_namespace()
    self.node_name = nepi_ros.get_node_name()
    self.node_namespace = os.path.join(self.base_namespace,self.node_name)

    ##############################  
    # Create Msg Class
    self.msg_if = MsgIF(log_name = None)
    self.msg_if.pub_info("Starting IF Initialization Processes")

    ##############################
    # Initialize Params
    self.initCb(do_updates = False)

    ###########################
    ## Wait for NEPI core managers to start
    # Wait for System Manager
    self.msg_if.pub_info("Starting ConnectSystemIF processes")
    mgr_sys_if = ConnectMgrSystemIF()
    self.msg_if.pub_info("Waiting for system if initialiation to complete")
    success = mgr_sys_if.wait_for_connection()
    self.msg_if.pub_info("Got system connection: " + str(success))
    if success == False:
      nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Status Msg")
    self.drivers_share_folder = mgr_sys_if.get_sys_folder_path('drivers',DRIVERS_SHARE_FOLDER)
    self.msg_if.pub_info("Using Drivers Share Folder: " + str(self.drivers_share_folder))
    self.drivers_install_folder = mgr_sys_if.get_sys_folder_path('install/drivers',DRIVERS_INSTALL_FOLDER)
    self.msg_if.pub_info("Using Drivers Install Folder: " + str(self.drivers_install_folder))
    self.user_cfg_folder = mgr_sys_if.get_sys_folder_path('user_cfg/ros',USER_CFG_FOLDER)
    self.msg_if.pub_info("Using User Config Folder: " + str(self.user_cfg_folder))
    
    # Wait for Config Manager
    mgr_cfg_if = ConnectMgrConfigIF()
    success = mgr_cfg_if.wait_for_status()
    #if success == False:
      #nepi_ros.signal_shutdown(self.node_name + ": Failed to get Config Status Msg")
    


     ##############################
    ### Setup Node

    # Configs Config Dict ####################
    self.CFGS_DICT = {
        'init_callback': self.initCb,
        'reset_callback': self.resetCb,
        'factory_reset_callback': self.factoryResetCb,
        'init_configs': True,
        'namespace': self.node_namespace
    }



    # Params Config Dict ####################
    self.PARAMS_DICT = {
        'retry_enabled': {
            'namespace': self.node_namespace,
            'factory_val':  False
        },
        'backup_enabled': {
            'namespace': self.node_namespace,
            'factory_val': True
        },
        'drvs_dict': {
            'namespace': self.node_namespace,
            'factory_val': dict()
        },
    }


    # Services Config Dict ####################
    self.SRVS_DICT = {
        'driver_status_query': {
            'namespace': self.node_namespace,
            'topic': 'driver_status_query',
            'svr': DriverStatusQuery,
            'req': DriverStatusQueryRequest(),
            'resp': DriverStatusQueryResponse(),
            'callback': self.driverStatusService
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': DriversStatus,
            'qsize': 1,
            'latch': True
        },
        'status_driver': {
            'namespace': self.node_namespace,
            'topic': 'status_driver', 
            'msg': DriverStatus,
            'qsize': 1,
            'latch': True
        },
        'settings_status': {
            'namespace': self.node_namespace,
            'topic': '/settings_status', #self.all_namespace + '/all_detectors/settings_status
            'msg': Settings,
            'qsize': 1,
            'latch': True
        }
    }  

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'refresh_drivers': {
            'namespace': self.node_namespace,
            'topic': 'refresh_drivers',
            'msg': empty,
            'qsize': 10,
            'callback': self.refreshCb, 
            'callback_args': ()
        },
        'enable_all_drivers': {
            'namespace': self.node_namespace,
            'topic': 'enable_all_drivers',
            'msg': String,
            'qsize': 10,
            'callback': self.enableAllCb, 
            'callback_args': ()
        },
        'enable_all_drivers': {
            'namespace': self.node_namespace,
            'topic': '/update_setting', #options_namespace + '/update_setting'
            'msg': Setting,
            'qsize': 1,
            'callback': self.updateSettingCb, 
            'callback_args': (options_namespace)
        }
    }




    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT,
                    log_class_name = True
    )

    ready = self.node_if.wait_for_ready()




    ###########################
    # Initialize Params
    drvs_dict = self.node_if.reset_param("drvs_dict")
    self.msg_if.pub_warn("Got start drvs keys: " + str(drvs_dict.keys()))
    adrvs = nepi_drvs.getDriversActiveOrderedList(drvs_dict)
    self.msg_if.pub_warn("Got start active drvs: " + str(adrvs))
    #self.msg_if.pub_warn("Got start drvs dict: " + str(drvs_dict))

    drvs_dict = self.node_if.reset_param("drvs_dict")
    self.msg_if.pub_warn("Got cfg_if drvs keys: " + str(drvs_dict.keys()))
    adrvs = nepi_drvs.getDriversActiveOrderedList(drvs_dict)
    self.msg_if.pub_warn("Got cfg_if active drvs: " + str(adrvs))
    #self.msg_if.pub_warn("Got cfg_if drvs dict: " + str(drvs_dict))

    time.sleep(1)
    drvs_dict = self.node_if.reset_param("drvs_dict")
    drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_share_folder,drvs_dict)
    self.msg_if.pub_warn("Got refreshed drvs keys: " + str(drvs_dict.keys()))
    adrvs = nepi_drvs.getDriversActiveOrderedList(drvs_dict)
    self.msg_if.pub_warn("Got refreshed active drvs: " + str(adrvs))
    #self.msg_if.pub_warn("Got init drvs dict: " + str(drvs_dict))
    node_name = 'None'
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.initCb(do_updates = True)

    ###########################
    nepi_ros.start_timer_process(0.5, self.statusPublishCb)

    # Setup a driver folder timed check
    nepi_ros.timer(nepi_ros.ros_duration(1), self.checkAndUpdateCb, oneshot=True)
    nepi_ros.timer(nepi_ros.ros_duration(self.PUBLISH_STATUS_INTERVAL), self.publishStatusCb, oneshot=True)
    time.sleep(1)
    ## Publish Status
    self.publish_status()

    #########################################################
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    #Set up node shutdown
    nepi_ros.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_ros.spin()
    #########################################################

  #######################
  # Wait for System and Config Statuses Callbacks
  def systemStatusCb(self,msg):
    self.sys_status = msg

  def configStatusCb(self,msg):
    self.cfg_status = True
  

  #######################
  ### Mgr Config Functions


  def initCb(self,do_updates = False):
      if do_updates == True:
        self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_share_folder)
        self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
        drvs_dict = self.node_if.reset_param("drvs_dict")
        drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_share_folder,self.rvs_dict)
        nepi_ros.set_param("drivers_dict",drvs_dict)
        self.resetCb(do_updates)
        #nepi_drvs.printDict(drvs_dict)

  def resetCb(self,do_updates = True):
      self.publish_status()

  def refreshCb(self,msg):
    self.refresh()

  def factoryResetCb(self):
      self.selected_driver = 'NONE'
      # reset drivers dict
      self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_share_folder)
      self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
      drvs_dict = nepi_drvs.getDriversgetDriversDict(self.drivers_share_folder)
      drvs_dict = nepi_drvs.setFactoryDriverOrder(drvs_dict)
      drvs_dict = activateAllDrivers(drvs_dict)
      nepi_ros.set_param("~drvs_dict",drvs_dict)
      self.publish_status()

  def refresh(self):
    # refresh drivers dict
    self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_share_folder)
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    drvs_dict = self.node_if.reset_param("drvs_dict")
    drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_share_folder,drvs_dict)
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.publish_status()


  def publishStatusCb(self,timer):
    self.publish_status()

  def checkAndUpdateCb(self,_):
    ###############################
    ## First update Database
    #self.msg_if.pub_warn("Driver update check checking drivers folder: " + str(self.drivers_share_folder))
    drivers_files = nepi_drvs.getDriverFilesList(self.drivers_share_folder)
    #self.msg_if.pub_warn("Driver update check got driver files: " + str(drivers_files))
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    need_update = self.drivers_files != drivers_files
    if need_update:
      self.msg_if.pub_warn("Need to Update Drv Database")
      self.initCb(do_updates = True)
    drvs_dict = self.node_if.reset_param("drvs_dict")
    drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
    drvs_active_list = nepi_drvs.getDriversActiveOrderedList(drvs_dict)

    ## Next process active driver processes

    ################################    
    ## Check and purge disabled driver proccess that might be running
    # Get list of available device paths
    available_paths_list = self.getAvailableDevPaths()
    # Get list of active nodes
    warnings.filterwarnings('ignore', '.*unclosed.*', ) 
    node_namespace_list = nepi_ros.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    # First check on running nodes that should not be running
    purge_list = []
    for driver_name in drvs_ordered_list:
      if driver_name not in drvs_active_list and driver_name in self.discovery_node_dict.keys():
          node_name = self.discovery_node_dict[driver_name]['node_name']
          if node_name in node_list:
            process = self.discovery_node_dict[driver_name]['process']
            if process == "LAUNCH":
              sub_process = self.discovery_node_dict[driver_name]['subprocess']
              success = nepi_drvs.killDriverNode(node_name,sub_process)
              if success:
                purge_list.append(driver_name)
    # purge from active discovery dict
    for driver_name in purge_list:
      if driver_name in self.discovery_node_dict.keys():
        del self.discovery_node_dict[driver_name]
  

    ################################    
    ## Do Discovery
    #self.msg_if.pub_warn( "Checking on driver discovery for list: " + str(drvs_ordered_list))
    
    retry = self.node_if.reset_param("retry_enabled")
    for driver_name in drvs_ordered_list:
      if driver_name in drvs_active_list:
        drv_dict = drvs_dict[driver_name]
        drv_dict['user_cfg_path'] = self.user_cfg_folder
        #self.msg_if.pub_warn( "Checking on driver discovery for: " + driver_name)
        if drv_dict['DISCOVERY_DICT']['file_name'] != "None":
          discovery_path = drv_dict['path']
          discovery_file = drv_dict['DISCOVERY_DICT']['file_name']
          discovery_class_name = drv_dict['DISCOVERY_DICT']['class_name']
          discovery_process = drv_dict['DISCOVERY_DICT']['process']
          discovery_node_name = drv_dict['DISCOVERY_DICT']['node_name']        



          ############################
          # Check Auto-Node processes
          remove_from_dict = False
          if discovery_process == "LAUNCH":
            if driver_name in self.discovery_node_dict.keys():
              # Check if still running

              launch_time = self.discovery_node_dict[driver_name]['launch_time']
              cur_time = nepi_ros.get_time()
              do_check = (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC
              if do_check == True:
                node_name = self.discovery_node_dict[driver_name]['node_name']
                running = nepi_ros.check_node_by_name(node_name)  
                if running == True:
                  drvs_dict[driver_name]['msg'] = "Discovery process running"
                else:
                  remove_from_dict = True
                  if retry == False:
                    self.msg_if.pub_warn("Node not running: " + node_name  + " - Will not restart")
                    self.failed_class_import_list.append(driver_name)
                  else:
                    self.msg_if.pub_warn("Node not running: " + node_name  + " - Will attempt restart")
            else: 
              if driver_name not in self.failed_class_import_list: # Check for not retry on non-running nodes
                #Setup required param server drv_dict for discovery node
                dict_param_name = os.path.join(discovery_node_name, "drv_dict")
                #self.msg_if.pub_warn("Passing param name: " + dict_param_name + " drv_dict: " + str(drv_dict))
                nepi_ros.set_param(dict_param_name,drv_dict)
                #Try and launch node
                self.msg_if.pub_info("")
                self.msg_if.pub_info("Launching discovery process: " + discovery_node_name + " with drv_dict " + str(drv_dict))
                [success, msg, sub_process] = nepi_drvs.launchDriverNode(discovery_file, discovery_node_name)
                if success:
                  self.msg_if.pub_info("Discovery node: " + discovery_node_name  + " launched with msg: " + msg)
                  drvs_dict[driver_name]['msg'] = "Discovery process lanched"
                  self.discovery_node_dict[driver_name]=dict()
                  self.discovery_node_dict[driver_name]['process'] = "LAUNCH"
                  self.discovery_node_dict[driver_name]['node_name'] = discovery_node_name
                  self.discovery_node_dict[driver_name]['subprocess'] = sub_process
                  self.discovery_node_dict[driver_name]['launch_time'] =  nepi_ros.get_time()  
                else:
                  self.msg_if.pub_warn("Failed to Launch discovery node: " + discovery_node_name  + " with msg: " + msg)
                  drvs_dict[driver_name]['msg'] = msg
                  if retry == False:
                    self.failed_class_import_list.append(discovery_node_name)
                    self.msg_if.pub_warn("Will not retry discovery node launch: " + discovery_node_name )

              

            if remove_from_dict == True:
              if driver_name in self.discovery_node_dict.keys():
                del self.discovery_node_dict[driver_name]
            



          ############################
          # Call Auto-Call processes 
          if discovery_process == "CALL":
            #self.msg_if.pub_warn( "Checking on driver discovery class for: " + driver_name + " with drv_dict " + str(drv_dict))
            if driver_name not in self.discovery_classes_dict.keys() and driver_name not in self.failed_class_import_list:
              self.msg_if.pub_info("")
              self.msg_if.pub_info("Importing discovery class " + discovery_class_name + " for driver " + driver_name)
              discovery_module = discovery_file.split('.')[0]
              [success, msg,imported_class] = nepi_drvs.importDriverClass(discovery_file,discovery_path,discovery_module,discovery_class_name)
              if success:
                self.msg_if.pub_info("Instantiating discovery class " + discovery_class_name + " with drv_dict " + str(drv_dict))
                discovery_class = imported_class()
                self.discovery_classes_dict[driver_name] = discovery_class
                self.msg_if.pub_info("Instantiated discovery class " + discovery_class_name + " for driver " + driver_name)
              else: 
                if retry == False:
                  self.failed_class_import_list.append(driver_name)
                self.msg_if.pub_info("Failed to import discovery class " + discovery_class_name + " for driver " + driver_name)
            elif driver_name not in self.failed_class_import_list:
              #self.msg_if.pub_info("")
              #self.msg_if.pub_warn("Calling discovery function for class: " + discovery_class_name + " for driver " + driver_name)
              discovery_class = self.discovery_classes_dict[driver_name]
              active_paths_list = discovery_class.discoveryFunction(available_paths_list, self.active_paths_list, self.base_namespace, drv_dict)
              if active_paths_list is None:
                self.failed_class_import_list.append(driver_name)
              else:
                self.active_paths_list = active_paths_list

          ################################
          # update options in param server
          if driver_name not in self.discovery_options_dict.keys() and driver_name not in self.failed_class_import_list:
              success = self.createDriverOptionsIf(driver_name,drvs_dict)
              if self.save_cfg_if is not None:
                self.save_cfg_if.save() # Save config on options change
              self.msg_if.pub_info("Instantiated discovery settings IF class " + discovery_node_name + " for driver " + driver_name)
          if driver_name in self.discovery_options_dict.keys():
            self.publishOptionsStatus(driver_name) 

    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the drvt runDiscovery()
    nepi_ros.sleep(self.UPDATE_CHECK_INTERVAL,100)
    nepi_ros.timer(nepi_ros.ros_duration(1), self.checkAndUpdateCb, oneshot=True)

  def createDriverOptionsIf(self,driver_name, drvs_dict):
    self.msg_if.pub_info("Creating driver options dict: " + driver_name)
    self.discovery_options_dict[driver_name] = dict()
    drv_dict = drvs_dict[driver_name]
    options_node_name = drv_dict['DISCOVERY_DICT']['node_name']
    options_namespace = os.path.join(self.base_namespace,options_node_name)
    self.discovery_options_dict[driver_name]['namespace'] = options_namespace

    self.msg_if.pub_info("Starting discovery options processes for namespace: " + options_namespace)

    self.discovery_options_dict[driver_name]['status_pub'] = options_status_pub

    self.discovery_options_dict[driver_name]['update_sub'] = options_update_sub

    options_cap_service = nepi_ros.create_service(options_namespace + '/settings_capabilities_query', SettingsCapabilitiesQuery, self.provide_capabilities)
    self.discovery_options_dict[driver_name]['caps_service'] = options_cap_service

    time.sleep(1)
    return True
          

  def provide_capabilities(self, req):
    drvs_dict = self.node_if.reset_param("drvs_dict")
    self.msg_if.pub_info("Got capabilities req: " + str(req))
    namespace = req.namespace
    capabilities_report = SettingsCapabilitiesQueryResponse()
    for driver_name in self.discovery_options_dict.keys():
      options_dict = self.discovery_options_dict[driver_name]
      dict_namespace = options_dict['namespace']
      #self.msg_if.pub_info("Looking for namespace: " + namespace)
      #self.msg_if.pub_info("Have namespace: " + dict_namespace)
      if namespace == dict_namespace:
        drv_dict = drvs_dict[driver_name]
        drv_options = drv_dict['DISCOVERY_DICT']['OPTIONS']
        cap_list = []
        for drv_option_name in drv_options.keys():
          drv_option = drv_options[drv_option_name]
          option_cap = SettingCap()
          option_cap.name_str = drv_option_name
          option_cap.type_str = drv_option['type']
          option_cap.options_list = drv_option['options']
          cap_list.append(option_cap)
        capabilities_report = SettingsCapabilitiesQueryResponse()
        capabilities_report.settings_count = len(cap_list)
        capabilities_report.setting_caps_list = cap_list
        break
    return capabilities_report

   
  def updateSettingCb(self,msg, args):
      namespace = args
      self.msg_if.pub_info("Received settings update msg " + str(msg))
      found_driver_name = None
      for driver_name in self.discovery_options_dict.keys():
        options_dict = self.discovery_options_dict[driver_name]
        dict_namespace = options_dict['namespace']
        #self.msg_if.pub_info("Looking for namespace: " + namespace)
        #self.msg_if.pub_info("Have namespace: " + dict_namespace)
        if namespace == dict_namespace:
          found_driver_name = driver_name
          break
      if found_driver_name is not None:
        setting = nepi_settings.parse_setting_update_msg_data(msg)
        self.msg_if.pub_info("Updating option: " + str(setting) + " for driver " + found_driver_name)
        self.updateSetting(found_driver_name,setting)

  def updateSetting(self,driver_name, setting):
      drvs_dict = self.node_if.reset_param("drvs_dict")
      drv_dict = drvs_dict[driver_name]
      drv_options = drv_dict['DISCOVERY_DICT']['OPTIONS']
      # Create cap options dict
      options_cap_dict = dict()
      for drv_option_name in drv_options.keys():
        drv_option = drv_options[drv_option_name]
        option_cap = dict()
        option_cap[drv_option_name] = dict()
        option_cap['name'] = drv_option_name
        option_cap['type'] = drv_option['type']
        option_cap['options'] =  drv_option['options']
      valid = nepi_settings.check_valid_setting(setting,options_cap_dict)
      if valid == True:
        #self.msg_if.pub_warn("Updating setting " + str(setting))
        option_name = setting['name']
        option_value = setting['value']
        try:
          drvs_dict[driver_name]['DISCOVERY_DICT']['OPTIONS'][option_name]['value'] = option_value
          nepi_ros.set_param("~drvs_dict",drvs_dict)
          self.msg_if.pub_info("Update option " + str(setting) + " for driver: " + driver_name)
        except Exception as e:
          self.msg_if.pub_warn("Failed to update option " + str(setting) + " for driver: " + driver_name + " with e " + str(e))
      else:
        self.msg_if.pub_warn("Failed to update option " + str(setting) + " for driver: " + driver_name + " NOT VALID")
      self.save_cfg_if.save()
      self.publishOptionsStatus(driver_name)   


  def publishOptionsStatus(self,driver_name):
        drvs_dict = self.node_if.reset_param("drvs_dict")
        drv_dict = drvs_dict[driver_name]
        drv_options = drv_dict['DISCOVERY_DICT']['OPTIONS']
        settings_list = []
        for drv_option_name in drv_options.keys():
          drv_option = drv_options[drv_option_name]
          setting = Setting()
          setting.name_str = drv_option_name
          setting.type_str = drv_option['type']
          setting.value_str = drv_option['value']
          settings_list.append(setting)
        settings_msg = Settings()
        settings_msg.settings_count = len(settings_list)
        settings_msg.settings_list = settings_list
        status_pub = self.discovery_options_dict[driver_name]['status_pub']
        self.node_if.publish_pub('status_pub', settings_msg)


  ###############################################

  def driverStatusService(self,request):
    driver_name = request.name
    response = self.getDriverStatusServiceMsg(driver_name)
    return response

  def getDriverStatusServiceMsg(self,driver_name):
    drvs_dict = self.node_if.reset_param("drvs_dict")
    driver_name = self.selected_driver
    service_driver_msg = DriverStatus()
    service_driver_msg.pkg_name = driver_name
    if driver_name in drvs_dict.keys() and driver_name != 'NONE':
      drv_dict = drvs_dict[driver_name]
      #self.msg_if.pub_info("Creating Drv Status Msg from Dict: " + str(drv_dict))
      #self.msg_if.pub_info("")
      try:
        #self.msg_if.pub_info("Driver Dict: " + str(drv_dict))
        service_driver_msg.type = drv_dict['type']
        service_driver_msg.group_id = drv_dict['group_id']
        service_driver_msg.display_name = drv_dict['display_name']
        service_driver_msg.description = drv_dict['description']
        service_driver_msg.active_state  = drv_dict['active']
        service_driver_msg.order  = drv_dict['order']
        service_driver_msg.msg_str = drv_dict['msg']
      except Exception as e:
        self.msg_if.pub_warn("Failed to create drv status msg for : " + str(drv_dict) + " " + str(e))
      #self.msg_if.pub_info("Returning Drv Status Msg: " + str(service_driver_msg))
      #self.msg_if.pub_info("")
    return service_driver_msg



        
  # ln = sys._getframe().f_lineno ; 
  def printND(self):
    drvs_dict = self.node_if.reset_param("drvs_dict")
    self.msg_if.pub_info('')
    self.msg_if.pub_info('*******************')
    self.msg_if.pub_info('Printing Drv Driver Dictionary')
    for driver_name in drvs_dict.keys():
      drv_dict = drvs_dict[driver_name]
      self.msg_if.pub_info('')
      self.msg_if.pub_info()
      self.msg_if.pub_info(str(drv_dict))

  def publish_status(self):
    self.publish_drivers_status()
    self.publish_driver_status()


  def statusPublishCb(self,timer):
      self.publish_drivers_status()

  def publish_drivers_status(self):
    self.last_status_drivers_msg = self.status_drivers_msg
    self.status_drivers_msg = self.getDriversStatusMsg()
    if self.drivers_status_pub is not None and not nepi_ros.is_shutdown():
      #self.msg_if.pub_warn("DriversStatus: " + str(self.status_drivers_msg))
      self.node_if.publish_pub('drivers_status_pub', self.status_drivers_msg)
      if self.save_cfg_if is not None:
        if self.last_status_drivers_msg != self.status_drivers_msg:
          self.save_cfg_if.save() # Save config after initialization for drvt time

  def getDriversStatusMsg(self):
    drvs_dict = self.node_if.reset_param("drvs_dict")
    drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
    #self.msg_if.pub_warn("DriversStatus Drv List: " + str(drvs_ordered_list))
    drvs_active_list = nepi_drvs.getDriversActiveOrderedList(drvs_dict)
    #self.msg_if.pub_warn("DriversStatus Active List: " + str(drvs_active_list))
    status_drivers_msg = DriversStatus()
    status_drivers_msg.pkg_list = drvs_ordered_list
    name_list = []
    type_list = []
    group_id_list = []
    for driver_name in drvs_ordered_list:
      name_list.append(drvs_dict[driver_name]['display_name'])
      type_list.append(drvs_dict[driver_name]['type'])
      group_id_list.append(drvs_dict[driver_name]['group_id'])
    status_drivers_msg.name_list = name_list
    status_drivers_msg.type_list = type_list
    status_drivers_msg.group_id_list = group_id_list

    status_drivers_msg.active_pkg_list = drvs_active_list
    name_list = []
    type_list = []
    namespace_list = []
    for driver_name in drvs_active_list:
      name_list.append(drvs_dict[driver_name]['display_name'])
      type_list.append(drvs_dict[driver_name]['type'])
      node_name = drvs_dict[driver_name]['DISCOVERY_DICT']['node_name']
      namespace_list.append(os.path.join(self.base_namespace,node_name))
    status_drivers_msg.active_name_list = name_list
    status_drivers_msg.active_type_list = type_list
    status_drivers_msg.active_namespace_list =  namespace_list

    status_drivers_msg.install_path = self.drivers_install_folder
    status_drivers_msg.install_list = self.drivers_install_files
    status_drivers_msg.backup_path = self.drivers_install_folder
    status_drivers_msg.backup_on_remove = self.node_if.reset_param("backup_enabled")
    status_drivers_msg.selected_driver = self.selected_driver

    status_drivers_msg.retry_enabled = self.node_if.reset_param("retry_enabled")
    return status_drivers_msg

  
  def publish_driver_status(self):
    self.last_status_driver_msg = self.status_driver_msg
    self.status_driver_msg = self.getDriverStatusMsg()
    if self.driver_status_pub is not None and not nepi_ros.is_shutdown():
      #self.msg_if.pub_warn("DriverStatus: " + str(self.status_driver_msg))
      self.node_if.publish_pub('driver_status_pub', self.status_driver_msg)
      if self.last_status_driver_msg != self.status_driver_msg:
        self.save_cfg_if.save() # Save config after initialization for drvt time


  def getDriverStatusMsg(self):
    drvs_dict = self.node_if.reset_param("drvs_dict")
    driver_name = self.selected_driver
    status_driver_msg = DriverStatus()
    status_driver_msg.pkg_name = driver_name
    if driver_name in drvs_dict.keys() and driver_name != 'NONE':
      drv_dict = drvs_dict[driver_name]
      #self.msg_if.pub_warn("Creating Drv Status Msg from Dict: " + str(drv_dict))
      #self.msg_if.pub_warn("")
      try:
        #self.msg_if.pub_warn("Driver Dict: " + str(drv_dict))
        status_driver_msg.type = drv_dict['type']
        status_driver_msg.group_id = drv_dict['group_id']
        status_driver_msg.display_name = drv_dict['display_name']
        status_driver_msg.description = drv_dict['description']
        status_driver_msg.active_state  = drv_dict['active']
        status_driver_msg.order  = drv_dict['order']
        status_driver_msg.msg_str = drv_dict['msg']
      except Exception as e:
        self.msg_if.pub_warn("Failed to create drv status msg for : " + str(drv_dict) + " " + str(e))
      #self.msg_if.pub_warn("Returning Drv Status Msg: " + str(status_driver_msg))
      #self.msg_if.pub_warn("")
    return status_driver_msg






  
  ###################
  ## Drivers Mgr Callbacks
  def enableAllCb(self,msg):
    drvs_dict = self.node_if.reset_param("drvs_dict")
    drvs_dict = nepi_drvs.activateAllDrivers(drvs_dict)
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.publish_status()

  def disableAllCb(self,msg):
    drvs_dict = self.node_if.reset_param("drvs_dict")
    drvs_dict = nepi_drvs.disableAllDrivers(drvs_dict)
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.publish_status()


  def selectDriverCb(self,msg):
    self.msg_if.pub_info(str(msg))
    driver_name = msg.data
    drvs_dict = self.node_if.reset_param("drvs_dict")
    if driver_name in drvs_dict.keys() or driver_name == "NONE":
      self.selected_driver = driver_name
    self.publish_status()

  def updateStateCb(self,msg):
    self.msg_if.pub_info(str(msg))
    driver_name = msg.name
    new_active_state = msg.active_state
    drvs_dict = self.node_if.reset_param("drvs_dict")
    active_state = False
    if driver_name in drvs_dict.keys():
      driver = drvs_dict[driver_name]
      active_state = driver['active']
    if new_active_state != active_state:
      if new_active_state == True:
        drvs_dict = nepi_drvs.activateDriver(driver_name,drvs_dict)
      else:
        drvs_dict = nepi_drvs.disableDriver(driver_name,drvs_dict)
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.publish_status()


  def updateOrderCb(self,msg):
    self.msg_if.pub_info(str(msg))
    driver_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    drvs_dict = self.node_if.reset_param("drvs_dict")
    if driver_name in drvs_dict.keys():
      drvs_dict = moveFunction(driver_name,drvs_dict)
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.publish_status()


  def getOrderUpdateFunction(self,move_cmd):
    if move_cmd == 'top':
      updateFunction = nepi_drvs.moveDriverTop
    elif move_cmd == 'bottom':
      updateFunction = nepi_drvs.moveDriverBottom
    elif move_cmd == 'up':
      updateFunction = nepi_drvs.moveDriverUp
    elif move_cmd == 'down':
      updateFunction = nepi_drvs.moveDriverDown
    else:
      updateFunction = self.moveDriverNone
    return updateFunction

  def moveDriverNone(self,driver_name,drvs_dict):
    return drvs_dict
    
  def updateMsgCb(self,msg):
    self.msg_if.pub_info(str(msg))
    driver_name = msg.name
    msg_data = msg.data
    drvs_dict = self.node_if.reset_param("drvs_dict")
    if driver_name in drvs_dict.keys():
      drvs_dict[driver_name]['msg'] = msg_data
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.publish_status()

 
  def enableRetryCb(self,msg):
    self.msg_if.pub_info(str(msg))
    retry_enabled = msg.data
    nepi_ros.set_param("~retry_enabled",retry_enabled)
    self.publish_status()

  def installDriverPkgCb(self,msg):
    self.msg_if.pub_info(str(msg))
    pkg_name = msg.data
    drvs_dict = self.node_if.reset_param("drvs_dict")
    if pkg_name in self.drivers_install_files:
     [success,drvs_dict]  = nepi_drvs.installDriverPkg(pkg_name,drvs_dict,self.drivers_install_folder,self.drivers_share_folder)
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.publish_status()

  def removeDriverCb(self,msg):
    self.msg_if.pub_info(str(msg))
    driver_name = msg.data
    drvs_dict = self.node_if.reset_param("drvs_dict")
    backup_folder = None
    backup_enabled = self.node_if.reset_param("backup_enabled")
    if backup_enabled:
      backup_folder = self.drivers_install_folder
    if driver_name in drvs_dict:
      [success,drvs_dict] = nepi_drvs.removeDriver(driver_name,drvs_dict,self.drivers_share_folder,backup_path = backup_folder)
    nepi_ros.set_param("~drvs_dict",drvs_dict)
    self.publish_status()


  def enableBackupCb(self,msg):
    self.msg_if.pub_info(str(msg))
    backup_enabled = msg.data
    nepi_ros.set_param("~backup_enabled",backup_enabled)
    self.publish_status()




  #######################
  # Misc Utility Function

  def getAvailableDevPaths(self):
    dev_path_list = glob.glob('/dev/*')
    return dev_path_list


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiDriversMgr()







