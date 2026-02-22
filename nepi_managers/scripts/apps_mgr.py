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
import copy
import glob
import sys
import subprocess
import time
import warnings

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_apps

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_interfaces.msg import MgrSystemStatus
from nepi_interfaces.msg import MgrAppsStatus, AppStatus, UpdateBool, UpdateOrder
from nepi_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryRequest, SystemStorageFolderQueryResponse
from nepi_interfaces.srv import AppStatusQuery, AppStatusQueryRequest, AppStatusQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import StatesIF




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
  status_msg = MgrAppsStatus()
  status_published = False

  

  node_if = None

  apps_dict = dict()
  backup_enabled = False
  restart_enabled = False

  failed_app_list = []



  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "apps_mgr" # Can be overwitten by luanch command
  def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        nepi_sdk.sleep(1)
        self.class_name = type(self).__name__
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
        self.msg_if.pub_warn("Got system folders: " + str(system_folders))

        self.apps_param_folder = system_folders['apps_param']
        self.msg_if.pub_info("Using APPS Params Folder: " + str(self.apps_param_folder))

        self.apps_config_folder = system_folders['etc']
        self.msg_if.pub_info("Using APPS Config Folder: " + str(self.apps_config_folder))

        self.apps_install_folder = system_folders['apps_install']
        self.msg_if.pub_info("Using APPS Install Folder: " + str(self.apps_install_folder))

        self.msg_if.pub_warn("Waiting for Config Mgr")
        config_folders = nepi_system.get_config_folders()

        # self.msg_if.pub_info("Waiting for driver manager to start")
        # active_drivers = nepi_system.get_active_drivers(log_name_list = [self.node_name])
        # nepi_sdk.sleep(5) # Some extra time for drivers to load
          
        ##############################
        # Initialize Variables

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
            'update_state': {
                'namespace': self.node_namespace,
                'topic': 'update_state',
                'msg': UpdateBool,
                'qsize': 10,
                'callback': self.updateBoolCb, 
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

        



        ##############################
        # Finish Initalization
        self.initCb(do_updates = True)

          
        ###########################
        # Start Node Processes

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

  def getActiveApps(self):
      apps_dict = copy.deepcopy(self.apps_dict)
      active_apps = []
      for app_name in apps_dict.keys():
        if apps_dict[app_name]['active'] == True:
          active_apps.append(app_name)
      return active_apps

  def initCb(self,do_updates = False):

      self.msg_if.pub_warn("Initing apps dict keys: " + str(self.apps_dict.keys()))
      self.msg_if.pub_info("Initing active apps: " + str(self.getActiveApps()))

      self.apps_files = nepi_apps.getAppInfoFilesList(self.apps_param_folder)
      self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
      if self.node_if is not None:
          self.apps_dict = self.node_if.get_param("apps_dict")
          self.backup_enabled = self.node_if.get_param("backup_enabled")
          self.restart_enabled = self.node_if.get_param("restart_enabled")   
      if do_updates == True:
        pass
      self.refresh()
      
      self.msg_if.pub_warn("Init apps dict keys: " + str(self.apps_dict.keys()))
      self.msg_if.pub_info("Init active apps: " + str(self.getActiveApps()))
      self.publish_status()

  def resetCb(self,do_updates = True):
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb()


  def factoryResetCb(self,do_updates = True):
      if self.node_if is not None:
        pass
      if do_updates == True:
        pass
      self.initCb()

  def publishStatusCb(self,timer):
    self.publish_status()


  def checkAndUpdateCb(self,timer):
    ###############################
    ## First update Database
    #self.msg_if.pub_warn("Entering Updater with apps dict keys: " + str(self.apps_dict.keys()))
    #self.msg_if.pub_info("Entering Updater with active apps: " + str(self.getActiveApps()))
    apps_files = nepi_apps.getAppInfoFilesList(self.apps_param_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    need_update = self.apps_files != apps_files
    if need_update:
      self.msg_if.pub_info("Need to Update App Database")
      self.initCb(do_updates = True)
    apps_dict = copy.deepcopy(self.apps_dict)
    apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    apps_active_list = self.getActiveApps()
    
    ## process active app processes


    ###############################
    # Get list of active nodes
    warnings.filterwarnings('ignore', '.*unclosed.*', ) 
    node_namespace_list = nepi_sdk.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    #self.msg_if.pub_warn("Found nodes: " + str(node_list))
    ################################    
    ## Check and purge disabled app proccess that might be running
    # First check on running nodes
    purge_list = []
    apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    for app_name in apps_ordered_list:
      if app_name not in apps_active_list and app_name in self.apps_active_dict.keys():
          namespace = self.apps_active_dict[app_name]['node_namespace']
          sub_process = self.apps_active_dict[app_name]['subprocess']
          self.msg_if.pub_warn("Killing app: " + str(app_name) + " : " + namespace)
          success = nepi_sdk.kill_node_process(namespace,sub_process)
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
    apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    for app_name in apps_ordered_list:
      app_dict = apps_dict[app_name]
      was_running = False
      if 'running' in app_dict.keys():
        was_running = app_dict['running']

      if app_name in apps_active_list and app_name not in self.apps_active_dict.keys():
        #self.msg_if.pub_warn("Launching app: " + str(app_name) )

        #self.msg_if.pub_warn(app_dict)
        app_pkg_name = app_dict['pkg_name']
        app_group_name = app_dict['group_name']
        app_file_name = app_dict['app_file']
        app_config_file_name = app_dict['config_file']
        app_path_name = app_dict['app_path']
        app_node_name = app_dict['node_name']
        app_file_path = app_path_name + '/' + app_file_name
        '''
        if app_name in self.apps_active_dict.keys():
          # Need to add running and restart check
          pass

        elif app_name not in self.failed_app_list:
        '''
        if True:  
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
            pass
            #self.msg_if.pub_warn("Could not find config file at: " + config_file_path + " starting with factory settings")
          
          #Try and launch node
          self.msg_if.pub_info("")
          self.msg_if.pub_info("Launching application node: " + app_node_name)
          #self.msg_if.pub_info("Launching application node: " + app_node_name + " with App Dict: " + str(app_dict))
          [success, msg, sub_process] = nepi_sdk.launch_node(app_pkg_name, app_file_name, app_node_name)
          if success == True:
            apps_dict[app_name]['running'] = False
            apps_dict[app_name]['msg'] = "Application started"
            if app_name not in self.apps_active_dict.keys():
              self.apps_active_dict[app_name]=dict()
            self.apps_active_dict[app_name]['node_name'] = app_node_name
            namespace = nepi_sdk.create_namespace(self.base_namespace,app_node_name)
            self.apps_active_dict[app_name]['node_namespace'] = namespace
            self.apps_active_dict[app_name]['subprocess'] = sub_process
          else:
            apps_dict[app_name]['running'] = False
            apps_dict[app_name]['msg'] = "Application failed to start"
            if restart == False:
              self.msg_if.pub_warn("Node not started: " + app_node_name  + " - Will not restart")
              self.failed_app_list.append(app_name)
            else:
              self.msg_if.pub_warn("Node not running: " + app_node_name  + " - Will attempt restart")

      # Check for running apps
      app_dict = apps_dict[app_name]
      app_node_name = app_dict['node_name']

      running = nepi_sdk.check_node_by_name(app_node_name)  
      if running == True:
        self.apps_dict[app_name]['running'] = running
        self.apps_dict[app_name]['msg'] = "Application running"
        if was_running == False:
          self.msg_if.pub_warn("App Running: " + app_node_name)
      elif running == False:
        self.apps_dict[app_name]['running'] = running
        if was_running == False:
          self.apps_dict[app_name]['msg'] = "Application not running"

        # do_check = (cur_time - launch_time) > self.NODE_LAUNCH_TIME_SEC
        # if was_running == True and do_check == True:
        #   self.apps_dict[app_name]['msg'] = "Application stopped running"
        #   self.apps_dict[app_name]['active'] = False
        #   self.msg_if.pub_warn("App Stopped Running: " + app_node_name)
          

    


    #self.msg_if.pub_warn("Ending Updater with apps dict keys: " + str(self.apps_dict.keys()))
    #self.msg_if.pub_info("Ending Updater with active apps: " + str(self.getActiveApps()))

    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the appt runDiscovery()
    #nepi_sdk.sleep(self.UPDATE_CHECK_INTERVAL)
    nepi_sdk.start_timer_process(self.UPDATE_CHECK_INTERVAL, self.checkAndUpdateCb, oneshot=True)
   

  ###################
  ## Apps Mgr Callbacks

  def enableAllCb(self,msg):
    self.msg_if.pub_info("Got enable all msg: " + str(msg))
    apps_dict = copy.deepcopy(self.apps_dict)
    apps_dict = nepi_apps.activateAllApps(apps_dict)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)

  def disableAllCb(self,msg):
    self.msg_if.pub_info("Got disable all msg: " + str(msg))
    apps_dict = copy.deepcopy(self.apps_dict)
    apps_dict = nepi_apps.disableAllApps(apps_dict)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)



  def updateBoolCb(self,msg):
    self.msg_if.pub_info("Got update app state msg: " + str(msg))
    app_name = msg.name
    state = msg.value
    if app_name in self.apps_dict.keys():
      self.apps_dict[app_name]['active'] = state
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",self.apps_dict)
   



  def updateOrderCb(self,msg):
    self.msg_if.pub_info("Got update app order msg: " + str(msg))
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
    self.msg_if.pub_info("Got enable restart msg: " + str(msg))
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
    self.msg_if.pub_info("Got install app package msg: " + str(msg))
    pkg_name = msg.data
    apps_dict = copy.deepcopy(self.apps_dict)
    if pkg_name in self.apps_install_files:
     [success,apps_dict]  = nepi_apps.installAppPkg(pkg_name,apps_dict,self.apps_install_folder,self.apps_install_folder)
    self.apps_dict = apps_dict
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("apps_dict",apps_dict)


  def removeAppCb(self,msg):
    self.msg_if.pub_info("Got remove app package msg: " + str(msg))
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
    self.msg_if.pub_info("Got enable backup msg: " + str(msg))
    self.backup_enabled = msg.data
    self.publish_status()
    if self.node_if is not None:
      self.node_if.set_param("backup_enabled",self.backup_enabled)



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



  def getAppStatusMsg(self, app_name):
    apps_dict = copy.deepcopy(self.apps_dict)
    status_app_msg = AppStatus()
    status_app_msg.app_name = app_name
    if app_name in apps_dict.keys() and app_name != 'NONE':
      app_dict = apps_dict[app_name]
      try:
        #self.msg_if.pub_warn(app_dict)
        status_app_msg.pkg_name = app_dict['pkg_name']
        status_app_msg.display_name = app_dict['display_name']
        status_app_msg.group_name = app_dict['group_name']
        status_app_msg.description = app_dict['description']
        node_name = app_dict['node_name']
        status_app_msg.node_name = node_name
        namespace = nepi_sdk.create_namespace(self.base_namespace,node_name)
        status_app_msg.namespace = namespace
        status_app_msg.app_file = app_dict['app_file']
        status_app_msg.app_path = app_dict['app_path']   
        if 'RUI_DICT' in app_dict.keys():
          status_app_msg.rui_files_list = app_dict['RUI_DICT']['rui_files']
          status_app_msg.rui_main_file = app_dict['RUI_DICT']['rui_main_file']
          status_app_msg.rui_main_class = app_dict['RUI_DICT']['rui_main_class']  

        status_app_msg.enabled  = app_dict['active']
        running = False
        if 'running' in app_dict.keys():
          running = app_dict['running']
        status_app_msg.running = running        
        status_app_msg.order  = app_dict['order']
        status_app_msg.msg_str = app_dict['msg']
        status_app_msg.license_type = app_dict['license_type']
        status_app_msg.license_link = app_dict['license_link']
      except Exception as e:
        self.msg_if.pub_info("Failed to create app status message: " + str(e))
    return status_app_msg



  def appStatusService(self,request):
    app_name = request.app_name
    response = self.getAppStatusMsg(app_name)
    return response



  def statusPublishCb(self,timer):
      self.publish_status()


  def publish_status(self):

    last_status_msg = copy.deepcopy(self.status_msg)
    
    apps_dict = copy.deepcopy(self.apps_dict)

    status_msg = MgrAppsStatus()
    status_msg.apps_path = self.apps_param_folder


    apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    status_msg.apps_ordered_list = apps_ordered_list
    name_list = []
    group_list = []
    status_list = []
    for app_name in apps_ordered_list:      
      name_list.append(apps_dict[app_name]['display_name'])
      group_list.append(apps_dict[app_name]['group_name'])
      status_list.append(self.getAppStatusMsg(app_name))
    status_msg.apps_ordered_name_list = name_list
    status_msg.apps_ordered_group_list =group_list
    status_msg.apps_ordered_status_list = status_list


    active_list = self.getActiveApps()
    status_msg.apps_active_list= active_list


  
    running_apps_list = nepi_apps.getAppsRunningList(apps_dict)  
    status_msg.apps_running_list = running_apps_list
    name_list = []
    group_list = []
    for app_name in running_apps_list:      
      name_list.append(apps_dict[app_name]['display_name'])
      group_list.append(apps_dict[app_name]['group_name'])
    status_msg.apps_running_name_list = name_list
    status_msg.apps_running_group_list =group_list


    status_msg.restart_enabled = self.restart_enabled

    self.status_msg = status_msg

    if self.node_if is not None:
      if self.status_published == False:
        self.status_published = True
        #self.msg_if.pub_info("Publishing Status Msg: " + str(self.status_msg))

      self.node_if.publish_pub('status_pub', self.status_msg)
      if last_status_msg != self.status_msg:
        self.node_if.save_config() # Save config

  



  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiAppsMgr()







