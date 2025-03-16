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
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy
import glob
import rosnode
import sys
import subprocess
import time
import warnings
import copy

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_settings

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_ros_interfaces.msg import SystemStatus
from nepi_ros_interfaces.msg import DriversStatus, DriverStatus, UpdateState, UpdateOrder 
from nepi_ros_interfaces.srv import DriverStatusQuery, DriverStatusQueryResponse

from nepi_ros_interfaces.msg import Setting, Settings, SettingCap, SettingCaps
from nepi_ros_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryResponse

from nepi_sdk.mgr_if_system import MgrSystemIF
from nepi_sdk.settings_if import SettingsIF
from nepi_sdk.save_cfg_if import SaveCfgIF


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

  init_backup_enabled = True

  status_drivers_msg = DriversStatus()
  last_status_drivers_msg = DriversStatus()
  status_driver_msg = DriverStatus()

  active_node_dict = dict()

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "drivers_mgr" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################

    ##############################
    ## Wait for NEPI core managers to start
    # Wait for System Manager
    mgr_sys_if = MgrSystemIF()
    success = mgr_sys_if.wait_for_status()
    if success == False:
      nepi_ros.signal_shutdown(self.node_name + ": Failed to get System Status Msg")
    self.drivers_share_folder = mgr_sys_if.get_sys_folder_path('drivers',DRIVERS_SHARE_FOLDER)
    nepi_msg.publishMsgInfo(self,"Using Drivers Share Folder: " + str(self.drivers_share_folder))
    self.drivers_install_folder = mgr_sys_if.get_sys_folder_path('install/drivers',DRIVERS_INSTALL_FOLDER)
    nepi_msg.publishMsgInfo(self,"Using Drivers Install Folder: " + str(self.drivers_install_folder))
    self.user_cfg_folder = mgr_sys_if.get_sys_folder_path('user_cfg/ros',USER_CFG_FOLDER)
    nepi_msg.publishMsgInfo(self,"Using User Config Folder: " + str(self.user_cfg_folder))
    
    config_status_topic = os.path.join(self.base_namespace,'config_mgr/status')
    nepi_msg.publishMsgInfo(self,"Waiting for Config Mgr Status")
    nepi_ros.wait_for_topic(config_status_topic)
    self.cfg_status = None
    cfg_status_sub = rospy.Subscriber(config_status_topic, Empty, self.configStatusCb, queue_size = 1)
    nepi_msg.publishMsgInfo(self,"Waiting for Config Mgr Status to publish")
    while(self.cfg_status is None):
      nepi_ros.sleep(1)
    cfg_status_sub.unregister()
    ##############################

   
    # Setup drvs_mgr params
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)
    self.save_cfg_if.userReset()
    time.sleep(1)
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",dict())
    drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_share_folder,drvs_dict)
    ###nepi_msg.publishMsgWarn(self,"Got init drvs dict: " + str(drvs_dict))
    node_name = 'None'
    self.init_drvs_dict = drvs_dict
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)

    self.initParamServerValues(do_updates = True)

    # Setup Node Status Publisher
    self.drivers_status_pub = rospy.Publisher("~status", DriversStatus, queue_size=1, latch=True)
    self.driver_status_pub = rospy.Publisher("~status_driver", DriverStatus, queue_size=1, latch=True)

    time.sleep(1)
    rospy.Timer(rospy.Duration(0.5), self.statusPublishCb)

    # Setup message publisher and init param server
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")

    ## Mgr ROS Setup 
    mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
    mgr_reset_sub = rospy.Subscriber('~refresh_drivers', Empty, self.refreshCb, queue_size = 10)


    # Drivers Management Scubscirbers
    rospy.Subscriber('~enable_all_drivers', Empty, self.enableAllCb, queue_size = 10)
    rospy.Subscriber('~disable_all_drivers', Empty, self.disableAllCb, queue_size = 10)
    rospy.Subscriber('~select_driver', String, self.selectDriverCb)
    rospy.Subscriber('~update_state', UpdateState, self.updateStateCb)
    rospy.Subscriber('~update_order', UpdateOrder, self.updateOrderCb)
    rospy.Subscriber('~enable_retry', Bool, self.enableRetryCb)


    #rospy.Subscriber('~select_driver_status', DriverUpdateMsg, self.updateMsgCb)

    rospy.Subscriber('~install_driver_pkg', String, self.installDriverPkgCb)
    rospy.Subscriber('~backup_on_remeove', Bool, self.enableBackupCb)
    rospy.Subscriber('~remove_driver', String, self.removeDriverCb)

    # Start capabilities services
    rospy.Service('~driver_status_query', DriverStatusQuery, self.driverStatusService)

    # Setup a driver folder timed check
    nepi_ros.timer(nepi_ros.ros_duration(1), self.checkAndUpdateCb, oneshot=True)
    nepi_ros.timer(nepi_ros.ros_duration(self.PUBLISH_STATUS_INTERVAL), self.publishStatusCb, oneshot=True)
    time.sleep(1)
    ## Publish Status
    self.publish_status()

    #########################################################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self,"Initialization Complete")
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

  def resetMgrCb(self,msg):
    self.resetMgr()

  def resetMgr(self):
    # reset drivers dict
    nepi_ros.set_param(self,"~retry_enabled",False)
    nepi_ros.set_param(self,"~backup_enabled",True)
    self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_share_folder)
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    drvs_dict = nepi_drvs.getDriversgetDriversDict(self.drivers_share_folder)
    drvs_dict = nepi_drvs.setFactoryDriverOrder(drvs_dict)
    drvs_dict = activateAllDrivers(drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.resetParamServer()
    self.publish_status()


  def refreshCb(self,msg):
    self.refresh()

  def refresh(self):
    # refresh drivers dict
    self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_share_folder)
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_share_folder,drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    self.retry_enabled = nepi_ros.get_param(self,"~retry_enabled",False)
    self.backup_enabled = nepi_ros.get_param(self,"~backup_enabled", True)

    
    self.initParamServerValues(do_updates = False)
    self.publish_status()

  def updateFromParamServer(self):
    #nepi_msg.publishMsgWarn(self,"Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functionsinstallDriverPkgCb
    pass
    
  def initParamServerValues(self,do_updates = True):
      self.selected_driver = 'NONE'
      nepi_msg.publishMsgInfo(self,"Setting init values to param values")
      self.init_retry_enabled = nepi_ros.get_param(self,"~retry_enabled", False)
      self.init_backup_enabled = nepi_ros.get_param(self,"~backup_enabled", True)
      self.drivers_files = nepi_drvs.getDriverFilesList(self.drivers_share_folder)
      self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
      drvs_dict = nepi_ros.get_param(self,"~drvs_dict",dict())
      drvs_dict = nepi_drvs.refreshDriversDict(self.drivers_share_folder,drvs_dict)
      self.init_drvs_dict = drvs_dict
      #nepi_drvs.printDict(drvs_dict)
      self.resetParamServer(do_updates)
      #nepi_drvs.printDict(drvs_dict)

  def resetParamServer(self,do_updates = True):
      nepi_ros.set_param(self,"~drvs_dict",self.init_drvs_dict)
      nepi_ros.set_param(self,"~retry_enabled",self.init_retry_enabled)
      nepi_ros.set_param(self,"~backup_enabled",self.init_backup_enabled)
      nepi_ros.set_param(self,"~drvs_dict",self.init_drvs_dict)
      if do_updates:
          self.updateFromParamServer()


  def publishStatusCb(self,timer):
    self.publish_status()

  def checkAndUpdateCb(self,_):
    ###############################
    ## First update Database
    drivers_files = nepi_drvs.getDriverFilesList(self.drivers_share_folder)
    self.drivers_install_files = nepi_drvs.getDriverPackagesList(self.drivers_install_folder)
    need_update = self.drivers_files != drivers_files
    if need_update:
      nepi_msg.publishMsgInfo(self,"Need to Update Drv Database")
      self.initParamServerValues()
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
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
    #nepi_msg.publishMsgWarn(self, "Checking on driver discovery for list: " + str(drvs_ordered_list))
    
    retry = nepi_ros.get_param(self,"~retry_enabled",self.init_retry_enabled)
    for driver_name in drvs_ordered_list:
      if driver_name in drvs_active_list:
        drv_dict = drvs_dict[driver_name]
        drv_dict['user_cfg_path'] = self.user_cfg_folder
        #nepi_msg.publishMsgWarn(self, "Checking on driver discovery for: " + driver_name)
        if drv_dict['DISCOVERY_DICT']['file_name'] != "None":
          discovery_path = drv_dict['path']
          discovery_file = drv_dict['DISCOVERY_DICT']['file_name']
          discovery_class_name = drv_dict['DISCOVERY_DICT']['class_name']
          discovery_process = drv_dict['DISCOVERY_DICT']['process']
          discovery_node_name = drv_dict['DISCOVERY_DICT']['node_name']        



          #nepi_msg.publishMsgWarn(self,"Processing discovery process for driver %s with method %s and process %s",driver_name,discovery_method,discovery_process)
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
                running = check_node_by_name(node_name)  
                if running == True:
                  drvs_dict[driver_name]['msg'] = "Discovery process running"
                else:
                  remove_from_dict = True
                  if retry == False:
                    nepi_msg.publishMsgWarn(self,"Node not running: " + node_name  + " - Will not restart")
                    self.failed_class_import_list.append(driver_name)
                  else:
                    nepi_msg.publishMsgWarn(self,"Node not running: " + node_name  + " - Will attempt restart")
            else: 
              if driver_name not in self.failed_class_import_list: # Check for not retry on non-running nodes
                #Setup required param server drv_dict for discovery node
                dict_param_name = os.path.join(discovery_node_name, "drv_dict")
                #nepi_msg.publishMsgWarn(self,"Passing param name: " + dict_param_name + " drv_dict: " + str(drv_dict))
                nepi_ros.set_param(self,dict_param_name,drv_dict)
                #Try and launch node
                nepi_msg.publishMsgInfo(self,"")
                nepi_msg.publishMsgInfo(self,"Launching discovery process: " + discovery_node_name + " with drv_dict " + str(drv_dict))
                [success, msg, sub_process] = nepi_drvs.launchDriverNode(discovery_file, discovery_node_name)
                if success:
                  nepi_msg.publishMsgInfo(self,"Discovery node: " + discovery_node_name  + " launched with msg: " + msg)
                  drvs_dict[driver_name]['msg'] = "Discovery process lanched"
                  self.discovery_node_dict[driver_name]=dict()
                  self.discovery_node_dict[driver_name]['process'] = "LAUNCH"
                  self.discovery_node_dict[driver_name]['node_name'] = discovery_node_name
                  self.discovery_node_dict[driver_name]['subprocess'] = sub_process
                  self.discovery_node_dict[driver_name]['launch_time'] =  nepi_ros.get_time()  
                else:
                  nepi_msg.publishMsgWarn(self,"Failed to Launch discovery node: " + discovery_node_name  + " with msg: " + msg)
                  drvs_dict[driver_name]['msg'] = msg
                  if retry == False:
                    self.failed_class_import_list.append(discovery_node_name)
                    nepi_msg.publishMsgWarn(self,"Will not retry discovery node launch: " + discovery_node_name )

              

            if remove_from_dict == True:
              if driver_name in self.discovery_node_dict.keys():
                del self.discovery_node_dict[driver_name]
            



          ############################
          # Call Auto-Call processes 
          if discovery_process == "CALL":
            #nepi_msg.publishMsgWarn(self, "Checking on driver discovery class for: " + driver_name + " with drv_dict " + str(drv_dict))
            if driver_name not in self.discovery_classes_dict.keys() and driver_name not in self.failed_class_import_list:
              nepi_msg.publishMsgInfo(self,"")
              nepi_msg.publishMsgInfo(self,"Importing discovery class " + discovery_class_name + " for driver " + driver_name)
              discovery_module = discovery_file.split('.')[0]
              [success, msg,imported_class] = nepi_drvs.importDriverClass(discovery_file,discovery_path,discovery_module,discovery_class_name)
              if success:
                nepi_msg.publishMsgInfo(self,"Instantiating discovery class " + discovery_class_name + " with drv_dict " + str(drv_dict))
                discovery_class = imported_class()
                self.discovery_classes_dict[driver_name] = discovery_class
                nepi_msg.publishMsgInfo(self,"Instantiated discovery class " + discovery_class_name + " for driver " + driver_name)
              else: 
                if retry == False:
                  self.failed_class_import_list.append(driver_name)
                nepi_msg.publishMsgInfo(self,"Failed to import discovery class " + discovery_class_name + " for driver " + driver_name)
            elif driver_name not in self.failed_class_import_list:
              #nepi_msg.publishMsgInfo(self,"")
              #nepi_msg.publishMsgWarn(self,"Calling discovery function for class: " + discovery_class_name + " for driver " + driver_name)
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
              self.save_cfg_if.saveConfig(do_param_updates = False) # Save config on options change
              nepi_msg.publishMsgInfo(self,"Instantiated discovery settings IF class " + discovery_node_name + " for driver " + driver_name)
          if driver_name in self.discovery_options_dict.keys():
            self.publishOptionsStatus(driver_name) 

    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the drvt runDiscovery()
    nepi_ros.sleep(self.UPDATE_CHECK_INTERVAL,100)
    nepi_ros.timer(nepi_ros.ros_duration(1), self.checkAndUpdateCb, oneshot=True)

  def createDriverOptionsIf(self,driver_name, drvs_dict):
    nepi_msg.publishMsgInfo(self,"Creating driver options dict: " + driver_name)
    self.discovery_options_dict[driver_name] = dict()
    drv_dict = drvs_dict[driver_name]
    options_node_name = drv_dict['DISCOVERY_DICT']['node_name']
    options_namespace = os.path.join(self.base_namespace,options_node_name)
    self.discovery_options_dict[driver_name]['namespace'] = options_namespace

    nepi_msg.publishMsgInfo(self,driver_name + ": Starting discovery options processes for namespace: " + options_namespace)

    options_status_pub = rospy.Publisher(options_namespace + '/settings_status', Settings, queue_size=1, latch=True)
    self.discovery_options_dict[driver_name]['status_pub'] = options_status_pub

    options_update_sub = rospy.Subscriber(options_namespace + '/update_setting', Setting, self.updateSettingCb, queue_size=1, callback_args=(options_namespace))
    self.discovery_options_dict[driver_name]['update_sub'] = options_update_sub

    options_cap_service = rospy.Service(options_namespace + '/settings_capabilities_query', SettingsCapabilitiesQuery, self.provide_capabilities)
    self.discovery_options_dict[driver_name]['caps_service'] = options_cap_service

    time.sleep(1)
    return True
          

  def provide_capabilities(self, req):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    nepi_msg.publishMsgInfo(self,"Got capabilities req: " + str(req))
    namespace = req.namespace
    capabilities_report = SettingsCapabilitiesQueryResponse()
    for driver_name in self.discovery_options_dict.keys():
      options_dict = self.discovery_options_dict[driver_name]
      dict_namespace = options_dict['namespace']
      #nepi_msg.publishMsgInfo(self,"Looking for namespace: " + namespace)
      #nepi_msg.publishMsgInfo(self,"Have namespace: " + dict_namespace)
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
      nepi_msg.publishMsgInfo(self,"Received settings update msg " + str(msg))
      found_driver_name = None
      for driver_name in self.discovery_options_dict.keys():
        options_dict = self.discovery_options_dict[driver_name]
        dict_namespace = options_dict['namespace']
        #nepi_msg.publishMsgInfo(self,"Looking for namespace: " + namespace)
        #nepi_msg.publishMsgInfo(self,"Have namespace: " + dict_namespace)
        if namespace == dict_namespace:
          found_driver_name = driver_name
          break
      if found_driver_name is not None:
        setting = nepi_settings.parse_setting_update_msg_data(msg)
        nepi_msg.publishMsgInfo(self,"Updating option: " + str(setting) + " for driver " + found_driver_name)
        self.updateSetting(found_driver_name,setting)

  def updateSetting(self,driver_name, setting):
      drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
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
        #nepi_msg.publishMsgWarn(self,"Updating setting " + str(setting))
        option_name = setting['name']
        option_value = setting['value']
        try:
          drvs_dict[driver_name]['DISCOVERY_DICT']['OPTIONS'][option_name]['value'] = option_value
          nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
          nepi_msg.publishMsgInfo(self,"Update option " + str(setting) + " for driver: " + driver_name)
        except Exception as e:
          nepi_msg.publishMsgWarn(self,"Failed to update option " + str(setting) + " for driver: " + driver_name + " with e " + str(e))
      else:
        nepi_msg.publishMsgWarn(self,"Failed to update option " + str(setting) + " for driver: " + driver_name + " NOT VALID")
      self.save_cfg_if.saveConfig(do_param_updates = False)
      self.publishOptionsStatus(driver_name)   


  def publishOptionsStatus(self,driver_name):
        drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
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
        status_pub.publish(settings_msg)


  ###############################################

  def driverStatusService(self,request):
    driver_name = request.name
    response = self.getDriverStatusServiceMsg(driver_name)
    return response

  def getDriverStatusServiceMsg(self,driver_name):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    driver_name = self.selected_driver
    service_driver_msg = DriverStatus()
    service_driver_msg.pkg_name = driver_name
    if driver_name in drvs_dict.keys() and driver_name != 'NONE':
      drv_dict = drvs_dict[driver_name]
      #nepi_msg.publishMsgInfo(self,"Creating Drv Status Msg from Dict: " + str(drv_dict))
      #nepi_msg.publishMsgInfo(self,"")
      try:
        #nepi_msg.publishMsgInfo(self,"Driver Dict: " + str(drv_dict))
        service_driver_msg.type = drv_dict['type']
        service_driver_msg.group_id = drv_dict['group_id']
        service_driver_msg.display_name = drv_dict['display_name']
        service_driver_msg.description = drv_dict['description']
        service_driver_msg.active_state  = drv_dict['active']
        service_driver_msg.order  = drv_dict['order']
        service_driver_msg.msg_str = drv_dict['msg']
      except Exception as e:
        nepi_msg.publishMsgWarn(self,"Failed to create drv status msg for : " + str(drv_dict) + " " + str(e))
      #nepi_msg.publishMsgInfo(self,"Returning Drv Status Msg: " + str(service_driver_msg))
      #nepi_msg.publishMsgInfo(self,"")
    return service_driver_msg



        
  # ln = sys._getframe().f_lineno ; 
  def printND(self):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    nepi_msg.publishMsgInfo(self,'')
    nepi_msg.publishMsgInfo(self,'*******************')
    nepi_msg.publishMsgInfo(self,'Printing Drv Driver Dictionary')
    for driver_name in drvs_dict.keys():
      drv_dict = drvs_dict[driver_name]
      nepi_msg.publishMsgInfo(self,'')
      nepi_msg.publishMsgInfo(self,driver_name)
      nepi_msg.publishMsgInfo(self,str(drv_dict))

  def publish_status(self):
    self.publish_drivers_status()
    self.publish_driver_status()


  def statusPublishCb(self,timer):
      self.publish_drivers_status()

  def publish_drivers_status(self):
    self.last_status_drivers_msg = self.status_drivers_msg
    self.status_drivers_msg = self.getDriversStatusMsg()
    if not nepi_ros.is_shutdown():
      #nepi_msg.publishMsgWarn(self,"DriversStatus: " + str(self.status_drivers_msg))
      self.drivers_status_pub.publish(self.status_drivers_msg)
      if self.last_status_drivers_msg != self.status_drivers_msg:
        self.save_cfg_if.saveConfig(do_param_updates = False) # Save config after initialization for drvt time

  def getDriversStatusMsg(self):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    drvs_ordered_list = nepi_drvs.getDriversOrderedList(drvs_dict)
    #nepi_msg.publishMsgWarn(self,"DriversStatus Drv List: " + str(drvs_ordered_list))
    drvs_active_list = nepi_drvs.getDriversActiveOrderedList(drvs_dict)
    #nepi_msg.publishMsgWarn(self,"DriversStatus Active List: " + str(drvs_active_list))
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
    status_drivers_msg.backup_on_remove = nepi_ros.get_param(self,"~backup_enabled",self.init_backup_enabled)
    status_drivers_msg.selected_driver = self.selected_driver

    status_drivers_msg.retry_enabled = nepi_ros.get_param(self,"~retry_enabled",self.init_retry_enabled)
    return status_drivers_msg

  
  def publish_driver_status(self):
    self.last_status_driver_msg = self.status_driver_msg
    self.status_driver_msg = self.getDriverStatusMsg()
    if not nepi_ros.is_shutdown():
      #nepi_msg.publishMsgWarn(self,"DriverStatus: " + str(self.status_driver_msg))
      self.driver_status_pub.publish(self.status_driver_msg)
      if self.last_status_driver_msg != self.status_driver_msg:
        self.save_cfg_if.saveConfig(do_param_updates = False) # Save config after initialization for drvt time


  def getDriverStatusMsg(self):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    driver_name = self.selected_driver
    status_driver_msg = DriverStatus()
    status_driver_msg.pkg_name = driver_name
    if driver_name in drvs_dict.keys() and driver_name != 'NONE':
      drv_dict = drvs_dict[driver_name]
      #nepi_msg.publishMsgWarn(self,"Creating Drv Status Msg from Dict: " + str(drv_dict))
      #nepi_msg.publishMsgWarn(self,"")
      try:
        #nepi_msg.publishMsgWarn(self,"Driver Dict: " + str(drv_dict))
        status_driver_msg.type = drv_dict['type']
        status_driver_msg.group_id = drv_dict['group_id']
        status_driver_msg.display_name = drv_dict['display_name']
        status_driver_msg.description = drv_dict['description']
        status_driver_msg.active_state  = drv_dict['active']
        status_driver_msg.order  = drv_dict['order']
        status_driver_msg.msg_str = drv_dict['msg']
      except Exception as e:
        nepi_msg.publishMsgWarn(self,"Failed to create drv status msg for : " + str(drv_dict) + " " + str(e))
      #nepi_msg.publishMsgWarn(self,"Returning Drv Status Msg: " + str(status_driver_msg))
      #nepi_msg.publishMsgWarn(self,"")
    return status_driver_msg






  
  ###################
  ## Drivers Mgr Callbacks
  def enableAllCb(self,msg):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    drvs_dict = nepi_drvs.activateAllDrivers(drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def disableAllCb(self,msg):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    drvs_dict = nepi_drvs.disableAllDrivers(drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()


  def selectDriverCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.data
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys() or driver_name == "NONE":
      self.selected_driver = driver_name
    self.publish_status()

  def updateStateCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.name
    new_active_state = msg.active_state
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    active_state = False
    if driver_name in drvs_dict.keys():
      driver = drvs_dict[driver_name]
      active_state = driver['active']
    if new_active_state != active_state:
      if new_active_state == True:
        drvs_dict = nepi_drvs.activateDriver(driver_name,drvs_dict)
      else:
        drvs_dict = nepi_drvs.disableDriver(driver_name,drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()


  def updateOrderCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys():
      drvs_dict = moveFunction(driver_name,drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
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
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.name
    msg_data = msg.data
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys():
      drvs_dict[driver_name]['msg'] = msg_data
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

 
  def enableRetryCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    retry_enabled = msg.data
    nepi_ros.set_param(self,"~retry_enabled",retry_enabled)
    self.publish_status()

  def installDriverPkgCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    pkg_name = msg.data
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if pkg_name in self.drivers_install_files:
     [success,drvs_dict]  = nepi_drvs.installDriverPkg(pkg_name,drvs_dict,self.drivers_install_folder,self.drivers_share_folder)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def removeDriverCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.data
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    backup_folder = None
    backup_enabled = nepi_ros.get_param(self,"~backup_enabled",self.init_backup_enabled)
    if backup_enabled:
      backup_folder = self.drivers_install_folder
    if driver_name in drvs_dict:
      [success,drvs_dict] = nepi_drvs.removeDriver(driver_name,drvs_dict,self.drivers_share_folder,backup_path = backup_folder)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()


  def enableBackupCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    backup_enabled = msg.data
    nepi_ros.set_param(self,"~backup_enabled",backup_enabled)
    self.publish_status()




  #######################
  # Misc Utility Function

  def getAvailableDevPaths(self):
    dev_path_list = glob.glob('/dev/*')
    return dev_path_list


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiDriversMgr()







