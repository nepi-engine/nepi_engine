#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_auto

from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.messages_if import MsgIF

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
      self.scripts_installed_at_start = nepi_auto.get_installed_scripts(self.get_installed_scripts_service)
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
      self.scripts_running_at_start = nepi_auto.get_running_scripts(self.get_running_scripts_service)
      if self.scripts_running_at_start == None:
        rospy.loginfo("NEPI_ROS: Service call failed, waiting 1 second then retrying")
        time.sleep(1)
  #rospy.loginfo("NEPI_ROS: Scripts running at start:")
  #rospy.loginfo(self.scripts_running_at_start)

