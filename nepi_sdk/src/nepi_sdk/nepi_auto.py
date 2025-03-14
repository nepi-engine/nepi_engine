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

  
import os
import sys
import importlib
import shutil
import rospy
import rosnode
import rostopic
import rosservice
import time


from datetime import datetime
from std_msgs.msg import Empty, Float32
from std_srvs.srv import Empty, EmptyRequest, Trigger
from nepi_ros_interfaces.srv import GetScriptsQuery,GetRunningScriptsQuery ,LaunchScript, StopScript
from nepi_ros_interfaces.msg import  Setting


#######################
# Script Utility Functions

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



# Function to get list of installed scripts
def get_installed_scripts(get_installed_scripts_service):
  installed_scripts = None
  try:
    response = get_installed_scripts_service()
    installed_scripts = response.scripts
    #rospy.loginfo("NEPI_ROS: Installed Scripts = " + str(installed_scripts))
  except Exception as e:
    rospy.loginfo("NEPI_ROS: Get installed scripts service call failed: " + str(e))
  return installed_scripts

# Function to get list of running scripts
def get_running_scripts(get_running_scripts_service):
  running_scripts = None
  try:
    response = get_running_scripts_service()
    running_scripts = response.running_scripts
    #rospy.loginfo("NEPI_ROS: Running Scripts = " + str(running_scripts))
  except Exception as e:
    rospy.loginfo("NEPI_ROS: Get running scripts service call failed: " + str(e))
  return running_scripts


# Function to launch a script
def launch_script(script2launch,launch_script_service):
  launch_success=False
  try:
    success = launch_script_service(script=script2launch)
    rospy.loginfo("NEPI_ROS: Launched script: " + str(success))
    launch_success=True
  except Exception as e:
    rospy.loginfo("NEPI_ROS: Launch script service call failed: " + str(e))
  return launch_success

### Function to stop script
def stop_script(script2stop,stop_script_service):
  stop_success=False
  try:
    success = stop_script_service(script=script2stop)
    rospy.loginfo("NEPI_ROS: Stopped script: " + str(success))
    stop_success=True
  except Exception as e:
    rospy.loginfo("NEPI_ROS: Stop script service call failed: " + str(e))
  return stop_success

# Function to start scripts from list
def launch_scripts(script_list,launch_script_service,get_installed_scripts_service,get_running_scripts_service):
  installed_scripts = get_installed_scripts(get_installed_scripts_service)
  running_scripts = get_running_scripts(get_running_scripts_service)
  if installed_scripts is not None and running_scripts is not None:
    for script2launch in script_list:
      script_installed = val_in_list(script2launch,installed_scripts)
      if script_installed:
        script_running = val_in_list(script2launch,running_scripts)
        if script_running is False:
            rospy.loginfo("NEPI_ROS: ")
            rospy.loginfo(["Launching script: " + script2launch])
            script_launch = launch_script(script2launch,launch_script_service)
            if script_launch:
              rospy.loginfo("NEPI_ROS: Script launch call success")
              script_running = False
              while script_running is False and not rospy.is_shutdown():
                running_scripts = get_running_scripts(get_running_scripts_service)
                script_running = val_in_list(script2launch,running_scripts)
                rospy.loginfo("NEPI_ROS: Waiting for script to launch")
                time.sleep(.5) # Sleep before checking again
              rospy.loginfo("NEPI_ROS: Script started successfully")
            else:
               rospy.loginfo("NEPI_ROS: Scipt launch call failed")
        else:
          rospy.loginfo("NEPI_ROS: Script already running, skipping launch process")
      else:
        rospy.loginfo("NEPI_ROS: Script not found, skipping launch process")
  else:
    rospy.loginfo("NEPI_ROS: Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #rospy.loginfo(running_scripts)
          

# Function to stop scripts from list, a
def stop_scripts(script_list,stop_script_service,get_installed_scripts_service,get_running_scripts_service,optional_ignore_script_list=[]):
  installed_scripts = get_installed_scripts(get_installed_scripts_service)
  running_scripts = get_running_scripts(get_running_scripts_service)
  if installed_scripts is not None and running_scripts is not None:
    for script2stop in script_list:
      script_running = val_in_list(script2stop,running_scripts)
      script_ignore = val_in_list(script2stop,optional_ignore_script_list)
      if script_running is True and script_ignore is False:
        script_running = val_in_list(script2stop,running_scripts)
        if script_running is True:
            rospy.loginfo("NEPI_ROS: ")
            rospy.loginfo(["Stopping script: " + script2stop])
            script_stop = stop_script(script2stop,stop_script_service)
            if script_stop:
              rospy.loginfo("NEPI_ROS: Script stop call success")
            else:
               rospy.loginfo("NEPI_ROS: Scipt stop call failed")
        else:
          rospy.loginfo("NEPI_ROS: Scipt in ignore list, skipping launch process")
      else:
        rospy.loginfo("NEPI_ROS: Script not found, skipping launch process")
  else:
    rospy.loginfo("NEPI_ROS: Failed to get installed and running script list")
  #running_scripts = get_running_scripts()
  #rospy.loginfo(running_scripts)



