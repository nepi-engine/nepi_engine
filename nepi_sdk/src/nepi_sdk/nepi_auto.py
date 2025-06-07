#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI automation manager interface functions

  

import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_auto"
logger = Logger(log_name = log_name)

#######################
# Script Utility Functions

# Function to get list of installed scripts
def get_installed_scripts(get_installed_scripts_service):
  installed_scripts = None
  try:
    response = get_installed_scripts_service()
    installed_scripts = response.scripts
    #logger.log_info("Installed Scripts = " + str(installed_scripts))
  except Exception as e:
    logger.log_info("Get installed scripts service call failed: " + str(e))
  return installed_scripts

# Function to get list of running scripts
def get_running_scripts(get_running_scripts_service):
  running_scripts = None
  try:
    response = get_running_scripts_service()
    running_scripts = response.running_scripts
    #logger.log_info("Running Scripts = " + str(running_scripts))
  except Exception as e:
    logger.log_info("Get running scripts service call failed: " + str(e))
  return running_scripts


# Function to launch a script
def launch_script(script2launch,launch_script_service):
  launch_success=False
  try:
    success = launch_script_service(script=script2launch)
    logger.log_info("Launched script: " + str(success))
    launch_success=True
  except Exception as e:
    logger.log_info("Launch script service call failed: " + str(e))
  return launch_success

### Function to stop script
def stop_script(script2stop,stop_script_service):
  stop_success=False
  try:
    success = stop_script_service(script=script2stop)
    logger.log_info("Stopped script: " + str(success))
    stop_success=True
  except Exception as e:
    logger.log_info("Stop script service call failed: " + str(e))
  return stop_success

# Function to start scripts from list
def launch_scripts(script_list,launch_script_service,get_installed_scripts_service,get_running_scripts_service):
  installed_scripts = get_installed_scripts(get_installed_scripts_service)
  running_scripts = get_running_scripts(get_running_scripts_service)
  if installed_scripts is not None and running_scripts is not None:
    for script2launch in script_list:
      script_installed = nepi_utils.nepi_utils.val_in_list(script2launch,installed_scripts)
      if script_installed:
        script_running = nepi_utils.nepi_utils.val_in_list(script2launch,running_scripts)
        if script_running is False:
            logger.log_info("")
            logger.log_info("Launching script: " + script2launch)
            script_launch = launch_script(script2launch,launch_script_service)
            if script_launch:
              logger.log_info("Script launch call success")
              script_running = False
              while script_running is False and not rospy.is_shutdown():
                running_scripts = get_running_scripts(get_running_scripts_service)
                script_running = nepi_utils.nepi_utils.val_in_list(script2launch,running_scripts)
                logger.log_info("Waiting for script to launch")
                time.sleep(.5) # Sleep before checking again
              logger.log_info("Script started successfully")
            else:
               logger.log_info("Scipt launch call failed")
        else:
          logger.log_info("Script already running, skipping launch process")
      else:
        logger.log_info("Script not found, skipping launch process")
  else:
    logger.log_info("Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #logger.log_info(str(running_scripts))
          

# Function to stop scripts from list, a
def stop_scripts(script_list,stop_script_service,get_installed_scripts_service,get_running_scripts_service,optional_ignore_script_list=[]):
  installed_scripts = get_installed_scripts(get_installed_scripts_service)
  running_scripts = get_running_scripts(get_running_scripts_service)
  if installed_scripts is not None and running_scripts is not None:
    for script2stop in script_list:
      script_running = nepi_utils.nepi_utils.val_in_list(script2stop,running_scripts)
      script_ignore = nepi_utils.nepi_utils.val_in_list(script2stop,optional_ignore_script_list)
      if script_running is True and script_ignore is False:
        script_running = nepi_utils.nepi_utils.val_in_list(script2stop,running_scripts)
        if script_running is True:
            logger.log_info("")
            logger.log_info("Stopping script: " + script2stop)
            script_stop = stop_script(script2stop,stop_script_service)
            if script_stop:
              logger.log_info("Script stop call success")
            else:
               logger.log_info("Scipt stop call failed")
        else:
          logger.log_info("Scipt in ignore list, skipping launch process")
      else:
        logger.log_info("Script not found, skipping launch process")
  else:
    logger.log_info("Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #logger.log_info(str(running_scripts))



def startup_script_initialize(self,NEPI_BASE_NAMESPACE):
  ## Initialize Class Variables
  self.scripts_installed_at_start = None
  self.scripts_running_at_start = None
  ## Define Class Namespaces
  AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "get_scripts"
  AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME = NEPI_BASE_NAMESPACE + "get_running_scripts"
  AUTO_LAUNCH_SCRIPT_SERVICE_NAME = NEPI_BASE_NAMESPACE + "launch_script"
  AUTO_STOP_SCRIPT_SERVICE_NAME = NEPI_BASE_NAMESPACE + "stop_script"
  ## Create Class Service Calls
  self.get_installed_scripts_service = rospy.ServiceProxy(AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME, GetScriptsQuery )
  self.get_running_scripts_service = rospy.ServiceProxy(AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME, GetRunningScriptsQuery )
  self.launch_script_service = rospy.ServiceProxy(AUTO_LAUNCH_SCRIPT_SERVICE_NAME, LaunchScript)
  self.stop_script_service = rospy.ServiceProxy(AUTO_STOP_SCRIPT_SERVICE_NAME, StopScript)
  ## Create Class Publishers
  ## Start Class Subscribers
  ## Start Node Processes
  rospy.loginfo("NEPI_ROS: ")
  rospy.loginfo("NEPI_ROS: ***********************")
  rospy.loginfo("NEPI_ROS: Starting Initialization")
  ### Get list of installed scripts
  rospy.loginfo("NEPI_ROS: Getting list of installed scripts")
  rospy.loginfo(["Calling service name: " + AUTO_GET_INSTALLED_SCRIPTS_SERVICE_NAME])
  while self.scripts_installed_at_start == None and not rospy.is_shutdown():
      self.scripts_installed_at_start = get_installed_scripts(self.get_installed_scripts_service)
      if self.scripts_installed_at_start == None:
        rospy.loginfo("NEPI_ROS: Service call failed, waiting 1 second then retrying")
        time.sleep(1)
  #rospy.loginfo("NEPI_ROS: Scripts installed at start:")
  #rospy.loginfo(self.scripts_installed_at_start)
  ### Get list of running scripts
  rospy.loginfo("NEPI_ROS: ")
  rospy.loginfo("NEPI_ROS: Getting list of running scripts at start")
  rospy.loginfo(["Calling service name: " + AUTO_GET_RUNNING_SCRIPTS_SERVICE_NAME])
  while self.scripts_running_at_start == None and not rospy.is_shutdown():
      self.scripts_running_at_start = get_running_scripts(self.get_running_scripts_service)
      if self.scripts_running_at_start == None:
        rospy.loginfo("NEPI_ROS: Service call failed, waiting 1 second then retrying")
        time.sleep(1)
  #rospy.loginfo("NEPI_ROS: Scripts running at start:")
  #rospy.loginfo(self.scripts_running_at_start)