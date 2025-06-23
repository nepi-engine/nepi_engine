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
import copy
import glob
import sys
import subprocess
import time
import warnings

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_apps

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import MgrAppsStatus, AppStatus, UpdateState, UpdateOrder
from nepi_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryRequest, SystemStorageFolderQueryResponse
from nepi_interfaces.srv import AppStatusQuery, AppStatusQueryRequest, AppStatusQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import StatesIF
from nepi_api.connect_mgr_if_system import ConnectMgrSystemIF
from nepi_api.connect_mgr_if_config import ConnectMgrConfigIF


APPS_SHARE_FOLDER = '/opt/nepi/ros/share/nepi_apps'
APPS_CONFIG_FOLDER = '/opt/nepi/ros/etc'
APPS_INSTALL_FOLDER = '/mnt/nepi_storage/install/apps'

#########################################

#########################################
# Node Class
#########################################

class NepiAppsMgr(object):


  UPDATE_CHECK_INTERVAL = 5
  PUBLISH_STATUS_INTERVAL = 1
  apps_param_folder = ''
  apps_files = []
  apps_ordered_list = []
  apps_active_list = []
  apps_install_folder = ''
  apps_install_files = []
  apps_install_list = []
  apps_active_dict = dict()
  status_apps_msg = MgrAppsStatus()
  status_app_msg = AppStatus()

  node_if = None

  apps_dict = dict()
  backup_enabled = False
  restart_enabled = False

  failed_app_list = []


  selected_app = "None"


  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "apps_mgr" # Can be overwitten by luanch command
  def __init__(self):
      #### APP NODE INIT SETUP ####
      nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
      self.class_name = type(self).__name__
      self.base_namespace = nepi_sdk.get_base_namespace()
      self.node_name = nepi_sdk.get_node_name()
      self.node_namespace = os.path.join(self.base_namespace,self.node_name)

      ##############################  
      # Create Msg Class
      self.msg_if = MsgIF(log_name = None)
      self.msg_if.pub_info("Starting IF Initialization Processes")

      ##############################
      # Initialize Variables
      # ToDo:CHANGE TO SYSTEM SERVICE CALLS
      self.apps_param_folder = APPS_SHARE_FOLDER + '/params'
      self.apps_config_folder = APPS_CONFIG_FOLDER
      self.apps_install_folder = APPS_INSTALL_FOLDER


      self.initCb(do_updates = False)

      ##############################
      ## Wait for NEPI core managers to start
      # Wait for System Manager
      mgr_sys_if = ConnectMgrSystemIF()
      success = mgr_sys_if.wait_for_status()
      if success == False:
          nepi_sdk.signal_shutdown(self.node_name + ": Failed to get System Status Msg")


    

      self.msg_if.pub_info("App folder set to " + self.apps_install_folder)
      self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
      self.msg_if.pub_info("App install packages folder files " + str(self.apps_install_files))

      # Wait for Config Manager
      mgr_cfg_if = ConnectMgrConfigIF()
      success = mgr_cfg_if.wait_for_status()
      if success == False:
          nepi_sdk.signal_shutdown(self.node_name + ": Failed to get Config Ready")



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
          'backup_enabled': {
              'namespace': self.node_namespace,
              'factory_val': True
          },
          'apps_dict': {
              'namespace': self.node_namespace,
              'factory_val': dict()
          },
          'restart_enabled': {
              'namespace': self.node_namespace,
              'factory_val': False
          },
      }


      # Services Config Dict ####################
      self.SRVS_DICT = {
          'app_status_query': {
              'namespace': self.node_namespace,
              'topic': 'app_status_query',
              'srv': AppStatusQuery,
              'req': AppStatusQueryRequest(),
              'resp': AppStatusQueryResponse(),
              'callback': self.appStatusService
          }
      }


      # Publishers Config Dict ####################
      self.PUBS_DICT = {
          'status_pub': {
              'namespace': self.node_namespace,
              'topic': 'status', #self.all_namespace + '/all_detectors/detection_image
              'msg': MgrAppsStatus,
              'qsize': 1,
              'latch': True
          },
          'status_app': {
              'namespace': self.node_namespace,
              'topic': 'status_app', #self.all_namespace + '/all_detectors/detection_image
              'msg': AppStatus,
              'qsize': 1,
              'latch': True
          },
      }  

      # Subscribers Config Dict ####################
      self.SUBS_DICT = {
          'enable_all_apps': {
              'namespace': self.node_namespace,
              'topic': 'enable_all_apps',
              'msg': Empty,
              'qsize': 10,
              'callback': self.enableAllCb, 
              'callback_args': ()
          },
          'disable_all_apps': {
              'namespace': self.node_namespace,
              'topic': 'disable_all_apps',
              'msg': Empty,
              'qsize': 10,
              'callback': self.disableAllCb, 
              'callback_args': ()
          },
          'select_app': {
              'namespace': self.node_namespace,
              'topic': 'select_app',
              'msg': String,
              'qsize': 10,
              'callback': self.selectAppCb, 
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
          }
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

      #ready = self.node_if.wait_for_ready()
      nepi_sdk.wait()

      # Setup message publisher and init param server
      self.msg_if.pub_info("Starting Initialization Processes")
      ## Mgr ROS Setup 


      ##############################
      self.initCb(do_updates = True)

       
      app_dict = dict()
      #self.msg_if.pub_warn("Got init apps dict: " + str(apps_dict))
      for app_name in self.apps_dict:
        app_dict[app_name] = self.apps_dict[app_name]['active']
      self.msg_if.pub_info("Got init app dict active list: " + str(app_dict))


      ###########################
      nepi_sdk.start_timer_process(0.5, self.statusPublishCb)

      # Setup a app folder timed check
      nepi_sdk.start_timer_process(1.0, self.checkAndUpdateCb, oneshot=True)
      nepi_sdk.start_timer_process(self.PUBLISH_STATUS_INTERVAL, self.publishStatusCb, oneshot=True)
      time.sleep(1)
      ## Publish Status
      self.publish_status()

      #########################################################
      ## Initiation Complete
      self.msg_if.pub_info("Initialization Complete")
      #Set up node shutdown
      nepi_sdk.on_shutdown(self.cleanup_actions)
      # Spin forever (until object is detected)
      nepi_sdk.spin()
      #########################################################



  ####################
  # Wait for System and Config Statuses Callbacks
  def systemStatusCb(self,msg):
      self.sys_status = msg

  def configStatusCb(self,msg):
      self.cfg_status = True
    
  #######################
  ### Mgr Config Functions


  def refreshCb(self,msg):
    self.refresh()

  def refresh(self):
    # refresh apps dict
    self.apps_files = nepi_apps.getAppInfoFilesList(self.apps_param_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    apps_dict = copy.deepcopy(self.apps_dict)
    apps_dict = nepi_apps.refreshAppsDict(self.apps_param_folder,apps_dict)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)

    
  def initCb(self,do_updates = False):
      if self.node_if is not None:
        self.apps_files = nepi_apps.getAppInfoFilesList(self.apps_param_folder)
        self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
        apps_dict = self.node_if.get_param("apps_dict")
        apps_dict = nepi_apps.refreshAppsDict(self.apps_param_folder,apps_dict)
        self.apps_dict = apps_dict
        if self.node_if is not None:
          self.node_if.set_param("apps_dict",apps_dict)
          self.backup_enabled = self.node_if.get_param("backup_enabled")
          self.restart_enabled = self.node_if.get_param("restart_enabled")   
      if do_updates == True:
        pass
      self.refresh()
      self.publish_status()

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.reset_params()
      if do_updates == True:
        pass
      self.initCb()


  def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        self.node_if.factory_reset_params()
      if do_updates == True:
        pass
      self.initCb()

  def publishStatusCb(self,timer):
    self.publish_status()


  def checkAndUpdateCb(self,_):
    ###############################
    ## First update Database
    
    apps_files = nepi_apps.getAppInfoFilesList(self.apps_param_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    need_update = self.apps_files != apps_files
    if need_update:
      self.msg_if.pub_info("Need to Update App Database")
      self.initCb(do_updates = True)
    apps_dict = copy.deepcopy(self.apps_dict)
    apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    apps_active_list = nepi_apps.getAppsActiveOrderedList(apps_dict)
    ## process active app processes


    ###############################
    # Get list of active nodes
    warnings.filterwarnings('ignore', '.*unclosed.*', ) 
    node_namespace_list = nepi_sdk.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    ################################    
    ## Check and purge disabled app proccess that might be running
    # First check on running nodes
    purge_list = []
    for app_name in self.apps_ordered_list:
      if app_name not in apps_active_list and app_name in self.apps_active_dict.keys():
          node_name = self.apps_active_dict[app_name]['node_name']
          if node_name in node_list:
            sub_process = self.apps_active_dict[app_name]['subprocess']
            success = nepi_sdk.kill_node_process(node_name,sub_process)
            if success:
              purge_list.append(app_name)
    # purge from active discovery dict
    for app_name in purge_list:
      if app_name in self.apps_active_dict.keys():
        del self.apps_active_dict[app_name]

        # Clear from failed list if disabled and killed
        if app_name in self.failed_app_list:
          self.failed_app_list.remove(app_name)


    ################################    
    ## Process Apps
    restart = self.restart_enabled
    for app_name in self.apps_ordered_list:
      if app_name in apps_active_list and app_name in apps_dict.keys():
        app_dict = apps_dict[app_name]
        #self.msg_if.pub_warn(app_dict)
        app_pkg_name = app_dict['APP_DICT']['pkg_name']
        app_group_name = app_dict['APP_DICT']['group_name']
        app_file_name = app_dict['APP_DICT']['app_file']
        app_config_file_name = app_dict['APP_DICT']['config_file']
        app_path_name = app_dict['APP_DICT']['app_path']
        app_node_name = app_dict['APP_DICT']['node_name']
        app_file_path = app_path_name + '/' + app_file_name
        if app_name in self.apps_active_dict.keys():
          # Need to add running and restart check
          pass

        elif app_name not in self.failed_app_list:
          #Try and initialize app param values
          config_file_path = self.apps_config_folder + "/" + app_config_file_name.split(".")[0] + "/" + app_config_file_name
          params_namespace = os.path.join(self.base_namespace, app_node_name)
          apt_dict_pn = os.path.join(params_namespace, 'app_dict')
          nepi_sdk.set_param(apt_dict_pn,app_dict)
          if os.path.exists(config_file_path):
            try:
              nepi_sdk.load_params_from_file(config_file_path, params_namespace)
            except Exception as e:
              self.msg_if.pub_warn("Could not get params from config file: " + config_file_path + " " + str(e))
          else:
            self.msg_if.pub_warn("Could not find config file at: " + config_file_path + " starting with factory settings")
          #Try and launch node
          self.msg_if.pub_info("")
          self.msg_if.pub_info("Launching application node: " + app_node_name)
          [success, msg, sub_process] = nepi_sdk.launch_node(app_pkg_name, app_file_name, app_node_name)
          if success:
            apps_dict[app_name]['msg'] = "Discovery process lanched"
            self.apps_active_dict[app_name]=dict()
            self.apps_active_dict[app_name]['node_name'] = app_node_name
            self.apps_active_dict[app_name]['subprocess'] = sub_process
          else:
            if restart == False:
              self.msg_if.pub_warn("Node not started: " + node_name  + " - Will not restart")
              self.failed_app_list.append(app_name)
            else:
              self.msg_if.pub_warn("Node not running: " + node_name  + " - Will attempt restart")

              


    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the appt runDiscovery()
    nepi_sdk.sleep(self.UPDATE_CHECK_INTERVAL,100)
    nepi_sdk.start_timer_process(1.0, self.checkAndUpdateCb, oneshot=True)
   


  def appStatusService(self,request):
    app_name = request.app_name
    response = self.getAppStatusServiceMsg(app_name)
    return response


  def getAppStatusServiceMsg(self, app_name):
    apps_dict = copy.deepcopy(self.apps_dict)
    status_app_msg = AppStatusQueryResponse()
    status_app_msg.app_name = app_name
    if app_name in apps_dict.keys() and app_name != 'NONE':
      app = apps_dict[app_name]
      try:
        #self.msg_if.pub_warn(app)
        status_app_msg.pkg_name = app['APP_DICT']['pkg_name']
        status_app_msg.group_name = app['APP_DICT']['group_name']
        status_app_msg.description = app['APP_DICT']['description']
        status_app_msg.node_name = app['APP_DICT']['node_name']
        status_app_msg.app_file = app['APP_DICT']['app_file']
        status_app_msg.app_path = app['APP_DICT']['app_path']   
        status_app_msg.rui_files_list = app['RUI_DICT']['rui_files']
        status_app_msg.rui_main_file = app['RUI_DICT']['rui_main_file']
        status_app_msg.rui_main_class = app['RUI_DICT']['rui_main_class']  
        status_app_msg.rui_menu_name = app['RUI_DICT']['rui_menu_name']
        status_app_msg.active_state  = app['active']
        status_app_msg.order  = app['order']
        status_app_msg.msg_str = app['msg']
        status_app_msg.license_type = app['APP_DICT']['license_type']
        status_app_msg.license_link = app['APP_DICT']['license_link']
      except Exception as e:
        self.msg_if.pub_info("Failed to create app status message: " + str(e))
    return status_app_msg

        
  # ln = sys._getframe().f_lineno ; 
  def printND(self):
    apps_dict = copy.deepcopy(self.apps_dict)
    self.msg_if.pub_info('')
    self.msg_if.pub_info('*******************')
    self.msg_if.pub_info('Printing Apps Dictionary')
    for app_name in apps_dict.keys():
      app_dict = apps_dict[app_name]
      self.msg_if.pub_info('')
      self.msg_if.pub_info(app_name)
      self.msg_if.pub_info(str(app_dict))



  def statusPublishCb(self,timer):
      self.publish_status()


  def publish_status(self):
    self.publish_apps_status()
    self.publish_app_status()

  def publish_apps_status(self):
    self.last_status_apps_msg = self.status_apps_msg
    self.status_apps_msg = self.getMgrAppsStatusMsg()
    if self.node_if is not None:
      self.node_if.publish_pub('status_pub', self.status_apps_msg)
      if self.last_status_apps_msg != self.status_apps_msg:
        self.node_if.save_config() # Save config

  def getMgrAppsStatusMsg(self):
    apps_dict = copy.deepcopy(self.apps_dict)
    self.apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    self.apps_group_list = nepi_apps.getAppsGroupList(apps_dict)
    apps_active_list = nepi_apps.getAppsActiveOrderedList(apps_dict)
    status_apps_msg = MgrAppsStatus()
    status_apps_msg.apps_path = self.apps_param_folder
    status_apps_msg.apps_ordered_list = self.apps_ordered_list
    apps_group_list = []
    for app_name in self.apps_ordered_list:
      apps_group_list.append(apps_dict[app_name]['APP_DICT']['group_name'])
    status_apps_msg.apps_group_list = apps_group_list
    status_apps_msg.apps_active_list = apps_active_list
    status_apps_msg.apps_install_path = self.apps_install_folder
    status_apps_msg.apps_install_list = self.apps_install_files
    status_apps_msg.apps_rui_list = nepi_apps.getAppsRuiActiveList(apps_dict)
    status_apps_msg.app_backup_path = self.apps_install_folder
    status_apps_msg.backup_removed_apps = self.backup_enabled
    status_apps_msg.selected_app = self.selected_app
    status_apps_msg.restart_enabled = self.restart_enabled
    return status_apps_msg

  
  def publish_app_status(self):
    self.last_status_app_msg = self.status_app_msg
    self.status_app_msg = self.getAppStatusMsg()
    if self.node_if is not None:
      self.node_if.publish_pub('status_app', self.status_app_msg)
      if self.last_status_app_msg != self.status_app_msg:
        self.node_if.save_config() # Save config


  def getAppStatusMsg(self):
    apps_dict = copy.deepcopy(self.apps_dict)
    app_name = self.selected_app
    status_app_msg = AppStatus()
    status_app_msg.name = app_name
    if app_name in apps_dict.keys() and app_name != 'NONE':
      app = apps_dict[app_name]
      try:
        #self.msg_if.pub_warn(app)
        status_app_msg.pkg_name = app['APP_DICT']['pkg_name']
        status_app_msg.group_name = app['APP_DICT']['group_name']
        status_app_msg.description = app['APP_DICT']['description']
        status_app_msg.node_name = app['APP_DICT']['node_name']
        status_app_msg.app_file = app['APP_DICT']['app_file']
        status_app_msg.app_path = app['APP_DICT']['app_path']   
        status_app_msg.rui_files_list = app['RUI_DICT']['rui_files']
        status_app_msg.rui_main_file = app['RUI_DICT']['rui_main_file']
        status_app_msg.rui_main_class = app['RUI_DICT']['rui_main_class']  
        status_app_msg.rui_menu_name = app['RUI_DICT']['rui_menu_name']
        status_app_msg.active_state  = app['active']
        status_app_msg.order  = app['order']
        status_app_msg.msg_str = app['msg']
        status_app_msg.license_type = app['APP_DICT']['license_type']
        status_app_msg.license_link = app['APP_DICT']['license_link']
      except Exception as e:
        self.msg_if.pub_info("Failed to create app status message: " + str(e))

    return status_app_msg

  

  ###################
  ## Apps Mgr Callbacks

  def enableAllCb(self,msg):
    apps_dict = copy.deepcopy(self.apps_dict)
    apps_dict = nepi_apps.activateAllApps(apps_dict)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)

  def disableAllCb(self,msg):
    apps_dict = copy.deepcopy(self.apps_dict)
    apps_dict = nepi_apps.disableAllApps(apps_dict)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)


  def selectAppCb(self,msg):
    self.msg_if.pub_info(str(msg))
    app_name = msg.data
    apps_dict = copy.deepcopy(self.apps_dict)
    if app_name in apps_dict.keys() or app_name == "NONE":
      self.selected_app = app_name
    self.publish_status()

  def updateStateCb(self,msg):
    self.msg_if.pub_info(str(msg))
    app_name = msg.name
    new_active_state = msg.active_state
    apps_dict = copy.deepcopy(self.apps_dict)
    if app_name in apps_dict.keys():
      app = apps_dict[app_name]
      active_state = app['active']
    if new_active_state != active_state:
      if new_active_state == True:
        apps_dict = nepi_apps.activateApp(app_name,apps_dict)
      else:
        apps_dict = nepi_apps.disableApp(app_name,apps_dict)
   
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)


  def updateOrderCb(self,msg):
    self.msg_if.pub_info(str(msg))
    app_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    apps_dict = copy.deepcopy(self.apps_dict)
    if app_name in apps_dict.keys():
      apps_dict = moveFunction(app_name,apps_dict)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)

  def enableRestartCb(self,msg):
    self.msg_if.pub_info(str(msg))
    self.restart_enabled = msg.data
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("restart_enabled",self.restart_enabled)


  def getOrderUpdateFunction(self,move_cmd):
    if move_cmd == 'top':
      updateFunction = nepi_apps.moveAppTop
    elif move_cmd == 'bottom':
      updateFunction = nepi_apps.moveAppBottom
    elif move_cmd == 'up':
      updateFunction = nepi_apps.moveAppUp
    elif move_cmd == 'down':
      updateFunction = nepi_apps.moveAppDown
    else:
      updateFunction = self.moveAppNone
    return updateFunction

  def moveAppNone(self,app_name,apps_dict):
    return apps_dict



  def installAppPkgCb(self,msg):
    self.msg_if.pub_info(str(msg))
    pkg_name = msg.data
    apps_dict = copy.deepcopy(self.apps_dict)
    if pkg_name in self.apps_install_files:
     [success,apps_dict]  = nepi_apps.installAppPkg(pkg_name,apps_dict,self.apps_install_folder,self.apps_install_folder)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)


  def removeAppCb(self,msg):
    self.msg_if.pub_info(str(msg))
    app_name = msg.data
    apps_dict = copy.deepcopy(self.apps_dict)
    backup_folder = None
    backup_enabled = self.backup_enabled
    if backup_enabled:
      backup_folder = self.apps_install_folder
    if app_name in apps_dict:
      [success,apps_dict] = nepi_apps.removeApp(app_name,apps_dict,backup_folder)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)


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
  NepiAppsMgr()







