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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_settings

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import MgrDriversStatus, DriverStatus, UpdateState, UpdateOrder 
from nepi_interfaces.srv import DriverStatusQuery, DriverStatusQueryRequest, DriverStatusQueryResponse

from nepi_interfaces.msg import Setting, Settings, SettingCap, SettingCaps, SettingsStatus
from nepi_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryRequest, SettingsCapabilitiesQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SettingsIF




#########################################

#########################################
# Node Class
#########################################

class NepiDriversMgr(object):

  NODE_LAUNCH_TIME_SEC = 10  # Will not check node status before

  UPDATE_CHECK_INTERVAL = 5
  PUBLISH_STATUS_INTERVAL = 1
  
  drivers_param_folder = ''

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
  discovery_settings_dict = dict()

  node_if = None 
  drvs_dict = dict()
  backup_enabled = False
  retry_enabled = False

  status_drivers_msg = MgrDriversStatus()
  last_status_drivers_msg = MgrDriversStatus()
  status_driver_msg = DriverStatus()

  active_node_dict = dict()


  settings_dict = dict()

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "drivers_mgr" # Can be overwitten by luanch command
  def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        nepi_sdk.sleep(1)
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = None)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for system folders")
        system_folders = nepi_system.get_system_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got system folders: " + str(system_folders))

        self.drivers_param_folder = system_folders['drivers_param']
        self.msg_if.pub_info("Using Drivers Param Folder: " + str(self.drivers_param_folder))

        self.drivers_install_folder = system_folders['drivers_install']
        self.msg_if.pub_info("Using Drivers Install Folder: " + str(self.drivers_install_folder))

        self.user_cfg_folder = system_folders['user_cfg']
        self.msg_if.pub_info("Using User Config Folder: " + str(self.user_cfg_folder))
    
        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()    
        
        ##############################
        # Initialize Class Variables
        
        self.initCb(do_updates = False)

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
                'srv': DriverStatusQuery,
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
                'msg': MgrDriversStatus,
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
        }  

        '''
        nepi_sdk.create_subscriber('~install_driver_pkg', String, self.installDriverPkgCb)
        nepi_sdk.create_subscriber('~backup_on_remeove', Bool, self.enableBackupCb)
        nepi_sdk.create_subscriber('~remove_driver', String, self.removeDriverCb)
        '''


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'refresh_drivers': {
                'namespace': self.node_namespace,
                'topic': 'refresh_drivers',
                'msg': Empty,
                'qsize': 10,
                'callback': self.refreshCb, 
                'callback_args': ()
            },
            'enable_all_drivers': {
                'namespace': self.node_namespace,
                'topic': 'enable_all_drivers',
                'msg': Empty,
                'qsize': 10,
                'callback': self.enableAllCb, 
                'callback_args': ()
            },
            'disable_all_drivers': {
                'namespace': self.node_namespace,
                'topic': 'disable_all_drivers',
                'msg': Empty,
                'qsize': 10,
                'callback': self.disableAllCb, 
                'callback_args': ()
            },
            'select_driver': {
                'namespace': self.node_namespace,
                'topic': 'select_driver',
                'msg': String,
                'qsize': 10,
                'callback': self.selectDriverCb, 
                'callback_args': ()
            },
            'update_state': {
                'namespace': self.node_namespace,
                'topic': 'update_state',
                'msg': UpdateState,
                'qsize': 10,
                'callback': self.updateStateCb, 
                'callback_args': ()
            },
            'update_order': {
                'namespace': self.node_namespace,
                'topic': 'update_order',
                'msg': UpdateOrder,
                'qsize': 10,
                'callback': self.updateOrderCb, 
                'callback_args': ()
            },
            'enable_retry': {
                'namespace': self.node_namespace,
                'topic': 'enable_retry',
                'msg': Bool,
                'qsize': 10,
                'callback': self.enableRetryCb, 
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

        }




        # Create Node Class ####################
        self.node_if = NodeClassIF(
                            configs_dict = self.CFGS_DICT,
                            params_dict = self.PARAMS_DICT,
                            services_dict = self.SRVS_DICT,
                            pubs_dict = self.PUBS_DICT,
                            subs_dict = self.SUBS_DICT,
                          msg_if = self.msg_if
        )

        nepi_sdk.wait()

        ###########################
        # Initialize Params

        self.initCb(do_updates = True)

        ###########################
        # Start node processes
        nepi_sdk.start_timer_process(0.5, self.statusPublishCb)
        nepi_sdk.start_timer_process(1.0, self.checkAndUpdateCb, oneshot=True)
        nepi_sdk.start_timer_process(self.PUBLISH_STATUS_INTERVAL, self.publishStatusCb, oneshot=True)


        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever (until object is detected)
        nepi_sdk.spin()
        #########################################################


  #######################
  ### Mgr Config Functions


  def initCb(self,do_updates = False):
      self.msg_if.pub_warn("Init with do_updates: " + str(do_updates))
      drvs_active_list = []
      if self.node_if is not None:
        self.retry_enabled = self.node_if.get_param("retry_enabled")
        self.backup_enabled = self.node_if.get_param("backup_enabled")
        drvs_dict = self.node_if.get_param("drvs_dict")
        self.msg_if.pub_warn("Init drvs keys: " + str(drvs_dict.keys()))
        drvs_active_list = nepi_drvs.getDriversActiveOrderedList(drvs_dict)
        self.msg_if.pub_warn("Init active drvs: " + str(drvs_active_list))
        drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_param_folder,drvs_dict)
        self.drvs_dict = drvs_dict
      if do_updates == True:
        self.refresh()
        nepi_system.set_active_drivers(drvs_active_list, log_name_list = [self.node_name])
        nepi_sdk.set_param('active_drivers', drvs_active_list)
      self.publish_status(do_updates = do_updates)
        

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
          pass
      if do_updates == True:
          pass
      self.initCb(do_updates = do_updates)


  def factoryResetCb(self,do_updates = True):
      self.aifs_classes_dict = dict()
      self.aif_classes_dict = dict()
      if self.node_if is not None:
          pass
      if do_updates == True:
          pass
      self.initCb(do_updates = do_updates)



  #######################
  # Wait for System and Config Statuses Callbacks
  def systemStatusCb(self,msg):
    self.sys_status = msg

  def configStatusCb(self,msg):
    self.cfg_status = True
  

  def refreshCb(self,msg):
    self.msg_if.pub_warn("Got refresh drivers request")
    self.refresh()

  def refresh(self):
    # refresh drivers dict
    self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_param_folder)
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    drvs_dict = copy.deepcopy(self.drvs_dict)
    self.msg_if.pub_warn("Refresh start drvs keys: " + str(drvs_dict.keys()))
    drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_param_folder,drvs_dict)
    self.drvs_dict = drvs_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",drvs_dict)



  def publishStatusCb(self,timer):
    self.publish_status()

  def checkAndUpdateCb(self,_):
    ###############################
    ## First update Database
    #self.msg_if.pub_warn("Driver update check checking drivers folder: " + str(self.drivers_param_folder))
    drivers_files = nepi_drvs.getDriverFilesList(self.drivers_param_folder)
    #self.msg_if.pub_warn("Driver update check got driver files: " + str(drivers_files))
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    need_update = self.drivers_files != drivers_files
    if need_update:
      self.msg_if.pub_warn("Need to Update Drv Database")
      self.initCb(do_updates = True)
    drvs_dict = copy.deepcopy(self.drvs_dict)
    #self.msg_if.pub_warn("Update start drvs: " + str(drvs_dict))
    drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
    drvs_active_list = nepi_drvs.getDriversActiveOrderedList(drvs_dict)
    nepi_system.set_active_drivers(drvs_active_list, log_name_list = [self.node_name])
    nepi_sdk.set_param('active_drivers', drvs_active_list)
    
    ## Next process active driver processes

    ################################    
    ## Check and purge disabled driver proccess that might be running
    # Get list of available device paths
    available_paths_list = self.getAvailableDevPaths()
    # Get list of active nodes
    warnings.filterwarnings('ignore', '.*unclosed.*', ) 
    node_namespace_list = nepi_sdk.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    # First check on running nodes that should not be running
    purge_list=[]
    for driver_name in drvs_ordered_list:
      if driver_name not in drvs_active_list:
        if driver_name in self.discovery_node_dict.keys():
          purge_list.append(driver_name)
        elif driver_name in self.discovery_classes_dict.keys():
          purge_list.append(driver_name)

    if len(purge_list) > 0:
      self.msg_if.pub_warn("Processing driver purge list " + str(purge_list))

    # purge from active discovery dict
    for driver_name in purge_list:
      process = drvs_dict[driver_name]['DISCOVERY_DICT']['process']
      if process == "LAUNCH":
        if driver_name in self.discovery_node_dict.keys():
          self.msg_if.pub_warn("Disabling driver: " + str(driver_name))
          node_name = self.discovery_node_dict[driver_name]['node_name']
          if node_name in node_list:
              self.msg_if.pub_warn("Killing driver node " + str(node_name) + " for driver: " + str(driver_name))
              sub_process = self.discovery_node_dict[driver_name]['subprocess']
              success = nepi_drvs.killDriverNode(node_name,sub_process)
              if success:
                self.msg_if.pub_warn("Killed driver node " + str(node_name) + " for driver: " + str(driver_name))
                if driver_name in self.discovery_node_dict.keys():
                  del self.discovery_node_dict[driver_name]
                  self.msg_if.pub_warn("Deleted driver node " + str(node_name) + " dict entry for driver: " + str(driver_name))
              else:
                self.msg_if.pub_warn("Failed to kill driver node " + str(node_name) + " for driver: " + str(driver_name))
      if process == "CALL":
        if driver_name in self.discovery_classes_dict.keys():
          self.msg_if.pub_warn("Disabling driver: " + str(driver_name))
          self.msg_if.pub_warn("Killing all devices for driver: " + str(driver_name) + " active paths list " + str(self.active_paths_list))
          try:
            
            self.active_paths_list = self.discovery_classes_dict[driver_name].killAllDevices(self.active_paths_list)
            self.msg_if.pub_warn("Got updated active paths list " + str(self.active_paths_list))
          except Exception as e:
            self.msg_if.pub_warn("Failed to call killAllDevices function: " + str(driver_name) + " : " + str(e))
          self.msg_if.pub_warn("Deleting imported classes function for driver: " + str(driver_name))
          del self.discovery_classes_dict[driver_name]
          discovery_file = drvs_dict[driver_name]['DISCOVERY_DICT']['file_name']
          discovery_module = discovery_file.split('.')[0]
          self.msg_if.pub_warn("Removing Module : " + discovery_module)
          del sys.modules[discovery_module]
          if driver_name in self.failed_class_import_list:
            self.msg_if.pub_warn("Removing driver from failed imported list: " + str(driver_name))
            self.failed_class_import_list.remove(driver_name) 


    ################################    
    ## Do Discovery
    #self.msg_if.pub_warn( "Checking on driver discovery for list: " + str(drvs_ordered_list))
    
    retry = self.retry_enabled
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
              cur_time = nepi_sdk.get_time()
              do_check = (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC
              if do_check == True:
                node_name = self.discovery_node_dict[driver_name]['node_name']
                running = nepi_sdk.check_node_by_name(node_name)  
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
                nepi_sdk.set_param(dict_param_name,drv_dict)
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
                  self.discovery_node_dict[driver_name]['launch_time'] =  nepi_sdk.get_time()  
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
            active_paths_list = None
            #self.msg_if.pub_warn( "Checking on driver discovery class for: " + driver_name ) #+ " with drv_dict " + str(drv_dict))
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
              
              try:
                active_paths_list = discovery_class.discoveryFunction(available_paths_list, self.active_paths_list, self.base_namespace, drv_dict)
              except Exception as e:
                self.msg_if.pub_info("Failed to call discovery function for driver: " + driver_name + " : " + str(e))
                self.msg_if.pub_info("Disabling driver: " + driver_name)
                self.update_state(driver_name,False)
                self.msg_if.pub_info("Deleting driver discovery class for : " + driver_name)
                del self.discovery_classes_dict[driver_name]
                discovery_module = discovery_file.split('.')[0]
                self.msg_if.pub_info("Removing Module : " + discovery_module)
                del sys.modules[discovery_module]
                #if retry == False:
                  #self.failed_class_import_list.append(driver_name)
              else:
                self.active_paths_list = active_paths_list

          ################################
          # update options in param server
          if driver_name not in self.discovery_settings_dict.keys() and driver_name not in self.failed_class_import_list:
              success = self.createDriverOptionsIf(driver_name,drvs_dict)
              self.node_if.save_config() # Save config on options change
              self.msg_if.pub_info("Instantiated discovery settings IF class " + discovery_node_name + " for driver " + driver_name)
          if driver_name in self.discovery_settings_dict.keys():
            self.publishDiscoverySettingsStatus(driver_name) 

    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the drvt runDiscovery()
    nepi_sdk.sleep(self.UPDATE_CHECK_INTERVAL,100)
    nepi_sdk.start_timer_process(1.0, self.checkAndUpdateCb, oneshot=True)

  def createDriverOptionsIf(self,driver_name, drvs_dict):
    drvs_dict = copy.deepcopy(self.drvs_dict)
    drv_dict = drvs_dict[driver_name]
    settings = drv_dict['DISCOVERY_DICT']['OPTIONS']
    self.msg_if.pub_warn("Creating driver options dict: " + driver_name + " with settings: " + str(settings))
    self.discovery_settings_dict[driver_name] = dict()
    drv_dict = drvs_dict[driver_name]
    settings_node_name = drv_dict['DISCOVERY_DICT']['node_name']
    settings_namespace = os.path.join(self.base_namespace,settings_node_name)
    self.discovery_settings_dict[driver_name]['namespace'] = settings_namespace

    self.msg_if.pub_info("Starting discovery options processes for namespace: " + settings_namespace)

    settings_status_pub = nepi_sdk.create_publisher(settings_namespace + '/settings/status', SettingsStatus, queue_size=1, latch=True)
    self.discovery_settings_dict[driver_name]['settings_pub'] = settings_status_pub

    settings_update_sub = nepi_sdk.create_subscriber(settings_namespace + '/settings/update_setting', Setting, self.updateSettingCb, queue_size=1, callback_args=(settings_namespace))
    self.discovery_settings_dict[driver_name]['update_sub'] = settings_update_sub

    settings_cap_service = nepi_sdk.create_service(settings_namespace + '/settings/capabilities_query', SettingsCapabilitiesQuery, self.provide_capabilities)
    self.discovery_settings_dict[driver_name]['caps_service'] = settings_cap_service

    time.sleep(1)
    return True
          

  def provide_capabilities(self, req):
    drvs_dict = copy.deepcopy(self.drvs_dict)
    #self.msg_if.pub_info("Got capabilities req: " + str(req))
    namespace = req.namespace
    caps_report = SettingsCapabilitiesQueryResponse()
    for driver_name in self.discovery_settings_dict.keys():
      settings_dict = self.discovery_settings_dict[driver_name]
      dict_namespace = settings_dict['namespace']
      #self.msg_if.pub_info("Looking for namespace: " + namespace)
      #self.msg_if.pub_info("Have namespace: " + dict_namespace)
      if namespace == dict_namespace:
        drv_dict = drvs_dict[driver_name]
        settings = drv_dict['DISCOVERY_DICT']['OPTIONS']
        #self.msg_if.pub_info("Updating Cap Settings from settings: " + str(settings))
        cap_list = []
        for setting_name in settings.keys():
          setting = settings[setting_name]
          setting_cap = SettingCap()
          setting_cap.name_str = setting_name
          setting_cap.type_str = setting['type']
          setting_cap.options_list = setting['options']
          cap_list.append(setting_cap)
        #self.msg_if.pub_info("Updated Caps List: " + str(cap_list))
        caps_report = SettingsCapabilitiesQueryResponse()
        caps_report.settings_count = len(cap_list)
        caps_report.setting_caps_list = cap_list
        caps_report.has_cap_updates = False
        break
    return caps_report

   
  def updateSettingCb(self,msg, args):
      namespace = args
      self.msg_if.pub_info("Received settings update msg " + str(msg))
      driver_name = None
      for driver_name in self.discovery_settings_dict.keys():
        settings_dict = self.discovery_settings_dict[driver_name]
        dict_namespace = settings_dict['namespace']
        #self.msg_if.pub_info("Looking for namespace: " + namespace)
        #self.msg_if.pub_info("Have namespace: " + dict_namespace)
        if namespace == dict_namespace:
          driver_name = driver_name
          break
      if driver_name is not None:
        setting = nepi_settings.parse_setting_msg(msg)
        self.msg_if.pub_warn("Updating option: " + str(setting) + " for driver " + driver_name)
        self.updateSetting(driver_name,setting)

  def updateSetting(self,driver_name, setting):
      drvs_dict = copy.deepcopy(self.drvs_dict)
      drv_dict = drvs_dict[driver_name]
      settings = drv_dict['DISCOVERY_DICT']['OPTIONS']
      # Create cap options dict
      settings_cap_dict = dict()
      for setting_name in settings.keys():
        setting = settings[setting_name]
        setting_caps = dict()
        setting_caps[setting_name] = dict()
        setting_caps['name'] = setting_name
        setting_caps['type'] = setting['type']
        setting_caps['options'] =  setting['options']
      valid = nepi_settings.check_valid_setting(setting,setting_caps)
      if valid == True:
        #self.msg_if.pub_warn("Updating setting " + str(setting))
        setting_name = setting['name']
        setting_value = setting['value']
        try:
          drvs_dict[driver_name]['DISCOVERY_DICT']['OPTIONS'][setting_name]['value'] = setting_value
          self.drvs_dict = drvs_dict
          if self.node_if is not None:
            self.node_if.set_param("drvs_dict",drvs_dict)
          self.msg_if.pub_info("Update option " + str(setting) + " for driver: " + driver_name)
        except Exception as e:
          self.msg_if.pub_warn("Failed to update option " + str(setting) + " for driver: " + driver_name + " with e " + str(e))
      else:
        self.msg_if.pub_warn("Failed to update option " + str(setting) + " for driver: " + driver_name + " NOT VALID")
      self.node_if.save_config()
      self.publishDiscoverySettingsStatus(driver_name)   


  def publishDiscoverySettingsStatus(self,driver_name):
        drvs_dict = copy.deepcopy(self.drvs_dict)
        drv_dict = drvs_dict[driver_name]

        settings = drv_dict['DISCOVERY_DICT']['OPTIONS']
        #self.msg_if.pub_info("Updating Status Cap Settings from settings: " + str(settings))
        settings_status_msg = SettingsStatus()
        settings_status_msg.settings_count = len(settings)

        setting_msgs_list = []
        for setting_name in settings.keys():
          setting = settings[setting_name]
          setting_msg = Setting()
          setting_msg.name_str = setting_name
          setting_msg.type_str = setting['type']
          setting_msg.value_str = setting['value']
          setting_msgs_list.append(setting_msg)
        settings_status_msg.settings_list = setting_msgs_list

        caps_msgs_list = []
        for setting_name in settings.keys():
          cap_setting = settings[setting_name]
          #self.msg_if.pub_info("Updating Status Cap Setting from setting: " + str(cap_setting))
          cap_msg = SettingCap()
          cap_msg.name_str = setting_name
          cap_msg.type_str = cap_setting['type']
          cap_msg.options_list = cap_setting['options']
          #self.msg_if.pub_info("Updated Caps Msg: " + str(cap_msg))
          caps_msgs_list.append(cap_msg)
        #self.msg_if.pub_info("Updated Caps Msg List: " + str(caps_msgs_list))
        settings_status_msg.setting_caps_list = caps_msgs_list

        settings_status_msg.has_cap_updates = False
        settings_pub = self.discovery_settings_dict[driver_name]['settings_pub']
        settings_pub.publish(settings_status_msg)


  ###############################################

  def driverStatusService(self,request):
    driver_name = request.name
    response = self.getDriverStatusServiceMsg(driver_name)
    return response

  def getDriverStatusServiceMsg(self,driver_name):
    drvs_dict = copy.deepcopy(self.drvs_dict)
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
    drvs_dict = copy.deepcopy(self.drvs_dict)
    self.msg_if.pub_info('')
    self.msg_if.pub_info('*******************')
    self.msg_if.pub_info('Printing Drv Driver Dictionary')
    for driver_name in drvs_dict.keys():
      drv_dict = drvs_dict[driver_name]
      self.msg_if.pub_info('')
      self.msg_if.pub_info()
      self.msg_if.pub_info(str(drv_dict))

  def publish_status(self, do_updates = True):
    self.publish_drivers_status(do_updates = do_updates)
    self.publish_driver_status(do_updates = do_updates)


  def statusPublishCb(self,timer):
      self.publish_status()

  def publish_drivers_status(self, do_updates = True):
    self.last_status_drivers_msg = self.status_drivers_msg
    self.status_drivers_msg = self.getMgrDriversStatusMsg()
    if self.node_if is not None:
      self.node_if.publish_pub('status_pub', self.status_drivers_msg)
      if do_updates == True:
        if self.last_status_drivers_msg != self.status_drivers_msg:
          self.node_if.save_config() # Save config after initialization for drvt time
  

  def getMgrDriversStatusMsg(self):
    drvs_dict = copy.deepcopy(self.drvs_dict)
    #self.msg_if.pub_warn("Got drivers status start drvs keys: " + str(drvs_dict.keys()), throttle_s = 5.0)
    drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
    #self.msg_if.pub_warn("MgrDriversStatus Drv List: " + str(drvs_ordered_list), throttle_s = 5.0)
    drvs_active_list = nepi_drvs.getDriversActiveOrderedList(drvs_dict)
    #self.msg_if.pub_warn("MgrDriversStatus Active List: " + str(drvs_active_list), throttle_s = 5.0)
    status_drivers_msg = MgrDriversStatus()
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
    status_drivers_msg.backup_on_remove = self.backup_enabled
    status_drivers_msg.selected_driver = self.selected_driver

    status_drivers_msg.retry_enabled = self.retry_enabled
    return status_drivers_msg

  
  def publish_driver_status(self, do_updates = True):
    self.last_status_driver_msg = self.status_driver_msg
    self.status_driver_msg = self.getDriverStatusMsg()
    if self.node_if is not None:
      self.node_if.publish_pub('status_driver', self.status_driver_msg)
      if do_updates == True:
        if self.last_status_driver_msg != self.status_driver_msg:
          self.node_if.save_config() # Save config after initialization for drvt time



  def getDriverStatusMsg(self):
    drvs_dict = copy.deepcopy(self.drvs_dict)
    #self.msg_if.pub_warn("Got driver status drvs keys: " + str(drvs_dict.keys()))
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
        running_state = driver_name in self.discovery_node_dict.keys() or driver_name in self.discovery_classes_dict.keys()
        status_driver_msg.running_state = running_state
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
    self.msg_if.pub_info("Got Update enable all msg: " + str(msg))
    drvs_dict = copy.deepcopy(self.drvs_dict)
    self.drvs_dict = nepi_drvs.activateAllDrivers(drvs_dict)   
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)

  def disableAllCb(self,msg):
    self.msg_if.pub_info("Got disable all msg: " + str(msg))
    drvs_namses = self.drvs_dict.keys()
    for driver_name in drvs_namses:
      self.update_state(driver_name,False)
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)


  def selectDriverCb(self,msg):
    self.msg_if.pub_info("Got select driver msg: " + str(msg))
    driver_name = msg.data
    drvs_dict = copy.deepcopy(self.drvs_dict)
    if driver_name in drvs_dict.keys() or driver_name == "NONE":
      self.selected_driver = driver_name
    self.publish_status()

  def updateStateCb(self,msg):
    self.msg_if.pub_info("Got update driver state msg: " + str(msg))
    driver_name = msg.name
    new_active_state = msg.active_state
    self.update_state(driver_name,new_active_state)

  def update_state(self,driver_name,new_active_state):
    self.msg_if.pub_info("Updateing driver " + driver_name + " state: " + str(new_active_state))
    drvs_dict = copy.deepcopy(self.drvs_dict)
    active_state = False
    if driver_name in drvs_dict.keys() and driver_name != 'None':
      driver = drvs_dict[driver_name]
      active_state = driver['active']
      if new_active_state != active_state:
        if new_active_state == True:
          drvs_dict = nepi_drvs.activateDriver(driver_name,drvs_dict)
        else:
          drvs_dict = nepi_drvs.disableDriver(driver_name,drvs_dict)
      #self.msg_if.pub_warn("Update driver state dict: " + str(drvs_dict[driver_name]))
      self.drvs_dict = drvs_dict
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param("drvs_dict",self.drvs_dict)


  def updateOrderCb(self,msg):
    self.msg_if.pub_info("Got Update driver order msg: " + str(msg))
    driver_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    drvs_dict = copy.deepcopy(self.drvs_dict)
    if driver_name in drvs_dict.keys() and driver_name != 'None':
      drvs_dict = moveFunction(driver_name,drvs_dict)
      self.drvs_dict = drvs_dict
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param("drvs_dict",self.drvs_dict)


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
    self.msg_if.pub_info("Got Update driver message msg: " + str(msg))
    driver_name = msg.name
    msg_data = msg.data
    drvs_dict = copy.deepcopy(self.drvs_dict)
    if driver_name in drvs_dict.keys() and driver_name != 'None':
      drvs_dict[driver_name]['msg'] = msg_data
      self.drvs_dict = drvs_dict
      self.publish_status()
      if self.node_if is not None:
        self.node_if.set_param("drvs_dict",self.drvs_dict)

 
  def enableRetryCb(self,msg):
    self.msg_if.pub_info(str(msg))
    self.retry_enabled = msg.data
    if self.retry_enabled == True:
      self.failed_class_import_list = []
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("retry_enabled",self.retry_enabled)
    

  def installDriverPkgCb(self,msg):
    self.msg_if.pub_info(str(msg))
    pkg_name = msg.data
    drvs_dict = copy.deepcopy(self.drvs_dict)
    if pkg_name in self.drivers_install_files:
     [success,drvs_dict]  = nepi_drvs.installDriverPkg(pkg_name,drvs_dict,self.drivers_install_folder,self.drivers_param_folder)
    self.drvs_dict = drvs_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)

  def removeDriverCb(self,msg):
    self.msg_if.pub_info(str(msg))
    driver_name = msg.data
    drvs_dict = copy.deepcopy(self.drvs_dict)
    backup_folder = None
    backup_enabled = self.backup_enabled
    if backup_enabled:
      backup_folder = self.drivers_install_folder
    if driver_name in drvs_dict:
      [success,drvs_dict] = nepi_drvs.removeDriver(driver_name,drvs_dict,self.drivers_param_folder,backup_path = backup_folder)
    self.drvs_dict = drvs_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("drvs_dict",self.drvs_dict)


  def enableBackupCb(self,msg):
    self.msg_if.pub_info(str(msg))
    self.backup_enabled = msg.data
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("backup_enabled",self.backup_enabled)
    




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







