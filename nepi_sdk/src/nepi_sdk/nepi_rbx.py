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


# NEPI utility script includes
# 1) RBX Initialization Process
# 2) RBX Settings Utility Functions
# 3) RBX Control Utility Functions


import rospy
import rosnode
import time
import sys
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_settings

from std_msgs.msg import Empty, Int8, UInt32, Int32, Bool, String, Float32, Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandHome
from nepi_interfaces.msg import RBXInfo, RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors, \
    RBXGotoPose, RBXGotoPosition, RBXGotoLocation, Setting, Settings, SettingCap, SettingCaps
from nepi_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryRequest, RBXCapabilitiesQueryResponse
from nepi_interfaces.srv import NPXCapabilitiesQuery, NPXCapabilitiesQueryRequest, NPXCapabilitiesQueryResponse


# ROS namespace setup

#######################
# RBX initialization process

def rbx_initialize(self, rbx_namespace):
  ## Initialize Class Variables
  rbx_caps = RBXCapabilitiesQueryResponse()
  self.rbx_cap_states = [""]
  self.rbx_cap_modes = [""]
  self.rbx_cap_setup_actions = [""]
  self.rbx_cap_go_actions = [""]
  rbx_caps_navpose = NPXCapabilitiesQueryResponse()
  self.rbx_settings = None
  self.rbx_info = None
  self.rbx_status= None


  ## Define Namespaces
  # NEPI RBX DEVICE NAMESPACE
  rbx_topic=nepi_sdk.wait_for_topic(rbx_namespace)
  NEPI_ROBOT_NAMESPACE = rbx_topic.rpartition("rbx")[0]
  NEPI_RBX_NAMESPACE = (NEPI_ROBOT_NAMESPACE + "rbx/")
  rospy.loginfo("NEPI_RBX: Found rbx namespace: " + NEPI_RBX_NAMESPACE)
  rospy.loginfo("NEPI_RBX: Found rbx status topic: " + rbx_topic)
  # NEPI RBX Driver Service Topics
  NEPI_RBX_CAPABILITIES_TOPIC = NEPI_RBX_NAMESPACE + "capabilities_query"
  NEPI_RBX_CAPABILITIES_NAVPOSE_TOPIC = NEPI_RBX_NAMESPACE + "navpose_capabilities_query"
  # Get RBX capabilities
  nepi_sdk.wait_for_service(NEPI_RBX_CAPABILITIES_TOPIC)
  rbx_caps_service = rospy.ServiceProxy(NEPI_RBX_CAPABILITIES_TOPIC, RBXCapabilitiesQuery)
  time.sleep(1)
  rbx_caps = rbx_caps_service()
  rospy.loginfo(rbx_caps)
  self.rbx_cap_states = (rbx_caps.state_options)
  self.rbx_cap_modes = (rbx_caps.mode_options)
  self.rbx_cap_setup_actions = (rbx_caps.setup_action_options)
  self.rbx_cap_go_actions = (rbx_caps.go_action_options)
  # rospy.loginfo some results
  rospy.loginfo("NEPI_RBX: RBX State Options: ")
  for state in self.rbx_cap_states:
     rospy.loginfo("NEPI_RBX:  - " + state)
  rospy.loginfo("NEPI_RBX: RBX Mode Options: ")
  for mode in self.rbx_cap_modes:
     rospy.loginfo("NEPI_RBX:  - " + mode)
  rospy.loginfo("NEPI_RBX: RBX Setup Action Options: ")
  for action in self.rbx_cap_setup_actions:
     rospy.loginfo("NEPI_RBX:  - " + action)
  rospy.loginfo("NEPI_RBX: RBX Go Action Options: ")
  for action in self.rbx_cap_go_actions:
     rospy.loginfo("NEPI_RBX:  - " + action)

  ## Setup Settings Callback
  self.NEPI_RBX_SETTINGS_TOPIC = NEPI_ROBOT_NAMESPACE + "settings_status"
  rospy.loginfo("DRONE_INSPECT: Waiting for topic: " + self.NEPI_RBX_SETTINGS_TOPIC)
  nepi_sdk.wait_for_topic(self.NEPI_RBX_SETTINGS_TOPIC)
  rbx_settings_pub = rospy.Publisher(NEPI_ROBOT_NAMESPACE + 'publish_settings', Empty, queue_size=1)
  rospy.loginfo("DRONE_INSPECT: Starting rbx settings scubscriber callback")
  rospy.Subscriber(self.NEPI_RBX_SETTINGS_TOPIC, Settings, self.rbx_settings_callback, queue_size=None)
  while self.rbx_settings is None and not rospy.is_shutdown():
    rospy.loginfo("DRONE_INSPECT: Waiting for current rbx settings to publish")
    time.sleep(1)
    rbx_settings_pub.publish(Empty())
  settings_str = str(self.rbx_settings)
  rospy.loginfo("DRONE_INSPECT: Initial settings:" + settings_str)


    ## Setup Info Update Callback
  self.NEPI_RBX_INFO_TOPIC = NEPI_RBX_NAMESPACE + "info" # RBX Info Message
  rospy.loginfo("DRONE_INSPECT: Waiting for topic: " + self.NEPI_RBX_INFO_TOPIC)
  nepi_sdk.wait_for_topic(self.NEPI_RBX_INFO_TOPIC)
  rbx_info_pub = rospy.Publisher(NEPI_RBX_NAMESPACE + 'publish_info', Empty, queue_size=1)
  rospy.loginfo("DRONE_INSPECT: Starting rbx info scubscriber callback")
  rospy.Subscriber(self.NEPI_RBX_INFO_TOPIC,RBXInfo, self.rbx_info_callback, queue_size=None)
  while self.rbx_info is None and not rospy.is_shutdown():
    rospy.loginfo("DRONE_INSPECT: Waiting for current rbx info to publish")
    time.sleep(1)
    rbx_info_pub.publish(Empty())
  info_str = str(self.rbx_info)
  rospy.loginfo("DRONE_INSPECT: " + info_str)

  ## Setup Status Update Callback
  self.NEPI_RBX_STATUS_TOPIC = NEPI_RBX_NAMESPACE + "status" # RBX Status Message
  rospy.loginfo("DRONE_INSPECT: Waiting for topic: " + self.NEPI_RBX_STATUS_TOPIC)
  nepi_sdk.wait_for_topic(self.NEPI_RBX_STATUS_TOPIC)
  rbx_status_pub = rospy.Publisher(NEPI_RBX_NAMESPACE + 'publish_status', Empty, queue_size=1)
  rospy.loginfo("DRONE_INSPECT: Starting rbx status scubscriber callback")
  rospy.Subscriber(self.NEPI_RBX_STATUS_TOPIC, RBXStatus, self.rbx_status_callback, queue_size=None)
  while self.rbx_status is None and not rospy.is_shutdown():
    rospy.loginfo("DRONE_INSPECT: Waiting for current rbx status to publish")
    time.sleep(0.1)
    rbx_status_pub.publish(Empty())
  status_str = str(self.rbx_status)
  rospy.loginfo("DRONE_INSPECT: " + status_str)

 # NEPI RBX Driver Control Topics
  NEPI_RBX_SETTINGS_UPDATE_TOPIC = NEPI_ROBOT_NAMESPACE + "update_setting" # Int to Defined Dictionary RBX_STATES
  rospy.loginfo("NEPI_RBX: Setting robot setting update topic to: " + NEPI_RBX_SETTINGS_UPDATE_TOPIC)
  self.rbx_setting_update_pub = rospy.Publisher(NEPI_RBX_SETTINGS_UPDATE_TOPIC, Setting, queue_size=1)

  # NEPI RBX Driver Control Topics
  NEPI_RBX_SET_STATE_TOPIC = NEPI_RBX_NAMESPACE + "set_state" # Int to Defined Dictionary RBX_STATES
  NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"  # Int to Defined Dictionary RBX_MODES
  NEPI_RBX_SETUP_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "setup_action"  # Int to Defined Dictionary RBX_MODES
  NEPI_RBX_SET_CMD_TIMEOUT_TOPIC = NEPI_RBX_NAMESPACE + "set_cmd_timeout" # Int Seconds  - Any command that changes ready state
  NEPI_RBX_SET_HOME_TOPIC = NEPI_RBX_NAMESPACE + "set_home" # GeoPoint
  NEPI_RBX_SET_STATUS_IMAGE_TOPIC = NEPI_RBX_NAMESPACE + "set_image_topic" # full or partial ROS namespace
  NEPI_RBX_SET_PROCESS_NAME_TOPIC = NEPI_RBX_NAMESPACE + "set_process_name"  # string name of current process

  self.rbx_set_state_pub = rospy.Publisher(NEPI_RBX_SET_STATE_TOPIC, Int32, queue_size=1)
  self.rbx_set_mode_pub = rospy.Publisher(NEPI_RBX_SET_MODE_TOPIC, Int32, queue_size=1)
  self.rbx_setup_action_pub = rospy.Publisher(NEPI_RBX_SETUP_ACTION_TOPIC, Int32, queue_size=1)
  self.rbx_set_cmd_timeout_pub = rospy.Publisher(NEPI_RBX_SET_CMD_TIMEOUT_TOPIC, UInt32, queue_size=1)
  self.rbx_set_home_pub = rospy.Publisher(NEPI_RBX_SET_HOME_TOPIC, GeoPoint, queue_size=1)
  self.rbx_set_image_topic_pub = rospy.Publisher(NEPI_RBX_SET_STATUS_IMAGE_TOPIC, String, queue_size=1)
  self.rbx_set_process_name_pub = rospy.Publisher(NEPI_RBX_SET_PROCESS_NAME_TOPIC, String, queue_size=1)

  NEPI_RBX_GO_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "go_action"  # Int to Defined Dictionary RBX_ACTIONS
  NEPI_RBX_GO_HOME_TOPIC = NEPI_RBX_NAMESPACE + "go_home" # Aborts any active goto processes
  NEPI_RBX_GO_STOP_TOPIC = NEPI_RBX_NAMESPACE + "go_stop" # Aborts any active goto processes
  NEPI_RBX_GOTO_POSE_TOPIC = NEPI_RBX_NAMESPACE + "goto_pose" # Ignored if any active goto processes
  NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position" # Ignored if any active goto processes
  NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location" # Ignored if any active goto processes

  self.rbx_go_action_pub = rospy.Publisher(NEPI_RBX_GO_ACTION_TOPIC, Int32, queue_size=1)
  self.rbx_go_home_pub = rospy.Publisher(NEPI_RBX_GO_HOME_TOPIC, Empty, queue_size=1)
  self.rbx_go_stop_pub = rospy.Publisher(NEPI_RBX_GO_STOP_TOPIC, Empty, queue_size=1)
  self.rbx_goto_pose_pub = rospy.Publisher(NEPI_RBX_GOTO_POSE_TOPIC, RBXGotoPose, queue_size=1)
  self.rbx_goto_position_pub = rospy.Publisher(NEPI_RBX_GOTO_POSITION_TOPIC,RBXGotoPosition, queue_size=1)
  self.rbx_goto_location_pub = rospy.Publisher(NEPI_RBX_GOTO_LOCATION_TOPIC, RBXGotoLocation, queue_size=1)

  # Fake GPS Controls
  NEPI_RBX_FAKE_GPS_ENABLE_TOPIC = NEPI_RBX_NAMESPACE + "enable_fake_gps" 
  self.rbx_enable_fake_gps_pub = rospy.Publisher(NEPI_RBX_FAKE_GPS_ENABLE_TOPIC, Bool, queue_size=1)

  rospy.loginfo("NEPI_RBX: RBX initialize process complete")

#######################
### RBX Settings, Info, and Status Callbacks
def rbx_settings_callback(self, msg):
  self.rbx_settings = nepi_settings.parse_settings_msg_data(msg)


def rbx_info_callback(self, msg):
  self.rbx_info = msg


def rbx_status_callback(self, msg):
  self.rbx_status = msg

#######################
### RBX Helper Functions

def get_capabilities(self,caps_topic):
  nepi_sdk.wait_for_service(caps_topic)
  rbx_caps_service = rospy.ServiceProxy(caps_topic, RBXCapabilitiesQuery)
  time.sleep(1)
  rbx_caps = rbx_caps_service()
  return rbx_caps

def get_navpose_capabilities(self,caps_navpose_topic):
  nepi_sdk.wait_for_service(caps_navpose_topic)
  rbx_cap_navpose_service = rospy.ServiceProxy(caps_navpose_topic, NPXCapabilitiesQuery)
  time.sleep(1)
  rbx_cap_navpose = rbx_cap_navpose_service()
  return rbx_caps_navpose



### Function to set rbx state
def set_rbx_state(self,state_str,timeout_sec = 5):
  rospy.loginfo("NEPI_RBX: *******************************")  
  rospy.loginfo("NEPI_RBX: Set State Request Recieved: " + state_str)
  success = False
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  new_state_ind = -1
  for ind, state in enumerate(self.rbx_cap_states):
    if state == state_str:
      new_state_ind = ind
  if new_state_ind == -1:
    rospy.loginfo("NEPI_RBX: No matching state found")
  else:
    rospy.loginfo("NEPI_RBX: Setting state to: " + state_str)
    self.rbx_set_state_pub.publish(new_state_ind)
    timeout_timer = 0
    sleep_time_sec = 1
    while self.rbx_info.state != new_state_ind and timeout_timer < timeout_sec and not rospy.is_shutdown():
      rospy.loginfo("NEPI_RBX: Waiting for rbx state " + self.rbx_cap_states[new_state_ind] + " to set")
      rospy.loginfo("NEPI_RBX: Current rbx state is " + self.rbx_cap_states[self.rbx_info.state])
      time.sleep(sleep_time_sec)
      timeout_timer = timeout_timer + sleep_time_sec
    if self.rbx_info.state == new_state_ind:
      success = True
  rospy.loginfo("NEPI_RBX: Current rbx state is " + self.rbx_cap_states[self.rbx_info.state])
  time.sleep(2)
  return success
  
### Function to set rbx mode
def set_rbx_mode(self,mode_str, timeout_sec = 5):
  rospy.loginfo("NEPI_RBX: *******************************")  
  rospy.loginfo("NEPI_RBX: Set Mode Request Recieved: " + mode_str)
  success = False
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  new_mode_ind = -1
  for ind, mode in enumerate(self.rbx_cap_modes):
    if mode == mode_str:
      new_mode_ind = ind
  if new_mode_ind == -1:
    rospy.loginfo("NEPI_RBX: No matching mode found")
  else:
    rospy.loginfo("NEPI_RBX: Setting mode to: " + mode_str)
    self.rbx_set_mode_pub.publish(new_mode_ind)
    timeout_timer = 0
    sleep_time_sec = 1
    while self.rbx_info.mode != new_mode_ind and timeout_timer < timeout_sec and not rospy.is_shutdown():
      rospy.loginfo("NEPI_RBX: Waiting for rbx mode " + self.rbx_cap_modes[new_mode_ind] + " to set")
      rospy.loginfo("NEPI_RBX: Current rbx mode is " + self.rbx_cap_modes[self.rbx_info.mode])
      time.sleep(sleep_time_sec)
      timeout_timer = timeout_timer + sleep_time_sec
    if self.rbx_info.mode == new_mode_ind:
      success = True
  rospy.loginfo("NEPI_RBX: Current rbx mode is " + self.rbx_cap_modes[self.rbx_info.mode])
  time.sleep(1)
  return success
  
 ### Function to send rbx action control
def setup_rbx_action(self,action_str,timeout_sec = 10):
  rospy.loginfo("NEPI_RBX: *******************************")  
  rospy.loginfo("NEPI_RBX: Setup Action Request Recieved: " + action_str)
  success = False
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  action_ind = -1
  for ind, action in enumerate(self.rbx_cap_setup_actions):
    if action == action_str:
      action_ind = ind
  if action_ind == -1:
    rospy.loginfo("NEPI_RBX: No matching action found")
  else:
    rospy.loginfo("NEPI_RBX: Waiting for ready state for takeoff")
    ready = wait_for_rbx_status_ready(self,timeout_sec)
    if ready:
      rospy.loginfo("NEPI_RBX: Sending takeoff command")
      self.rbx_setup_action_pub.publish(action_ind)
      busy = wait_for_rbx_status_busy(self,timeout_sec)
      if busy:
        ready = wait_for_rbx_status_ready(self,timeout_sec)
    time.sleep(1)
    success = self.rbx_status.cmd_success
  return success

### Function to set image topic name
def set_rbx_image_topic(self,image_topic):
  rospy.loginfo("NEPI_RBX: *******************************")  
  rospy.loginfo("NEPI_RBX: Set Image Topic Request Recieved: ")
  rospy.loginfo(image_topic)
  success = False
  self.rbx_set_image_topic_pub.publish(image_topic)
  success = True
  return success

### Function to set image topic name
def set_rbx_process_name(self,process_name):
  rospy.loginfo("NEPI_RBX: *******************************")  
  rospy.loginfo("NEPI_RBX: Set Process Name Request Recieved: ")
  rospy.loginfo(process_name)
  success = False
  self.rbx_set_process_name_pub.publish(process_name)
  success = True
  return success


#######################
# RBX Control Functions

### Function to send rbx action control
def go_rbx_action(self,action_str,timeout_sec = 10):
  rospy.loginfo("NEPI_RBX: *******************************")  
  rospy.loginfo("NEPI_RBX: Goto Action Request Recieved: " + action_str)
  success = False
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  action_ind = -1
  for ind, action in enumerate(self.rbx_cap_go_actions):
    if action == action_str:
      action_ind = ind
  if action_ind == -1:
    rospy.loginfo("NEPI_RBX: No matching action found")
  else:
    rospy.loginfo("NEPI_RBX: Waiting for ready state for takeoff")
    ready = wait_for_rbx_status_ready(self,timeout_sec)
    if ready:
      rospy.loginfo("NEPI_RBX: Sending takeoff command")
      self.rbx_go_action_pub.publish(action_ind)
      busy = wait_for_rbx_status_busy(self,timeout_sec)
      if busy:
        ready = wait_for_rbx_status_ready(self,timeout_sec)
    time.sleep(1)
    success = self.rbx_status.cmd_success
  return success

### Function to send rbx home control
def go_rbx_home(self,timeout_sec = 10):
  rospy.loginfo("NEPI_RBX: *******************************")  
  rospy.loginfo("NEPI_RBX: Go Home Request Recieved: ")
  success = False
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  ready = wait_for_rbx_status_ready(self,timeout_sec)
  if ready:
    self.rbx_go_home_pub.publish(Empty())
    busy = wait_for_rbx_status_busy(self,timeout_sec)
    if busy:
      ready = wait_for_rbx_status_ready(self,timeout_sec)
    time.sleep(1)
    success = self.rbx_status.cmd_success
  return success

### Function to send rbx home control
def go_rbx_stop(self,timeout_sec = 10):
  rospy.loginfo("NEPI_RBX: *******************************")  
  rospy.loginfo("NEPI_RBX: Go Stop Request Recieved: ")
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  self.rbx_go_stop_pub.publish(Empty())
  return success
  

### Function to call goto Attititude NED control
def goto_rbx_pose(self,goto_data,timeout_sec = 10):
  # Send goto Attitude Command
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  if len(goto_data) == 3:
    ready = wait_for_rbx_status_ready(self,timeout_sec)
    if ready:
      rospy.loginfo("NEPI_RBX: Starting goto Attitude NED Process")
      goto_msg = RBXGotoPose()
      goto_msg.roll_deg = goto_data[0]
      goto_msg.pitch_deg = goto_data[1]
      goto_msg.yaw_deg = goto_data[2]
      self.rbx_goto_pose_pub.publish(goto_msg)
      busy = wait_for_rbx_status_busy(self,timeout_sec)
      if busy:
        ready = wait_for_rbx_status_ready(self,timeout_sec)
    time.sleep(1)
    return self.rbx_status.cmd_success
  else:
    return False


### Function to call goto Location Global control
def goto_rbx_location(self,goto_data,timeout_sec = 10):
  # Send goto Location Command
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  if len(goto_data) == 4:
    ready = wait_for_rbx_status_ready(self,timeout_sec)
    if ready:
      rospy.loginfo("NEPI_RBX: Starting goto Location Global Process")
      goto_msg = RBXGotoLocation()
      goto_msg.lat = goto_data[0]
      goto_msg.long = goto_data[1]
      goto_msg.altitude_meters= goto_data[2]
      goto_msg.yaw_deg = goto_data[3]
      self.rbx_goto_location_pub.publish(goto_msg)
      busy = wait_for_rbx_status_busy(self,timeout_sec)
      if busy:
        ready = wait_for_rbx_status_ready(self,timeout_sec)
    time.sleep(1)
    return self.rbx_status.cmd_success
  else:
    return False

### Function to call goto Position Body control
def goto_rbx_position(self,goto_data,timeout_sec = 10):
  # Send goto Position Command
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  if len(goto_data) == 4:
    ready = wait_for_rbx_status_ready(self,timeout_sec)
    if ready:
      rospy.loginfo("NEPI_RBX: Starting goto Position Body Process")
      goto_msg = RBXGotoPosition()
      goto_msg.x_meters = goto_data[0]
      goto_msg.y_meters = goto_data[1]
      goto_msg.z_meters= goto_data[2]
      goto_msg.yaw_deg = goto_data[3]
      self.rbx_goto_position_pub.publish(goto_msg)
      busy = wait_for_rbx_status_busy(self,timeout_sec)
      if busy:
        ready = wait_for_rbx_status_ready(self,timeout_sec)
    time.sleep(1)
    return self.rbx_status.cmd_success
  else:
    return False


  
### Function to wait for goto control process to complete
def wait_for_rbx_status_ready(self,timeout_sec = 10):
  rospy.loginfo("NEPI_RBX: Waiting for status ready")
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  count_goal = 3 # fix for strange ready glitch
  counter = 0
  timeout_timer = 0
  sleep_time_sec = 0.1
  while (counter < count_goal) and timeout_timer < timeout_sec and (not rospy.is_shutdown()):
    if self.rbx_status.ready is True:
      counter = counter + 1
      #rospy.loginfo("NEPI_RBX: Status ready counter updated to: " + str(self.rbx_status.ready))
    else:
      counter = 0
      #rospy.loginfo("NEPI_RBX: Status ready counter reset")
    time.sleep(sleep_time_sec)
    timeout_timer += sleep_time_sec
  if timeout_timer > timeout_sec:
    rospy.loginfo("NEPI_RBX: Aborted Wait for Ready due to timeout" + str(timeout_timer))
  else:
    rospy.loginfo("NEPI_RBX: Got status ready True")
  return self.rbx_status.ready 

### Function to wait for goto control process to complete
def wait_for_rbx_status_busy(self,timeout_sec = 10):
  rospy.loginfo("NEPI_RBX: Waiting for status busy")
  self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
  time.sleep(0.1)
  count_goal = 3 # fix for strange ready glitch
  counter = 0
  timeout_timer = 0
  sleep_time_sec = 0.1
  while (counter < count_goal) and timeout_timer < timeout_sec and (not rospy.is_shutdown()):
    if self.rbx_status.ready is False:
      counter = counter + 1
      #rospy.loginfo("NEPI_RBX: Status busy counter updated to: " + str(self.rbx_status.ready))
    else:
      counter = 0
      #rospy.loginfo("NEPI_RBX: Status busy counter reset")
    time.sleep(sleep_time_sec)
    timeout_timer += sleep_time_sec
  if timeout_timer > timeout_sec:
    rospy.loginfo("NEPI_RBX: Aborted Wait for Busy due to timeout: " + str(timeout_timer))
  else:
    rospy.loginfo("NEPI_RBX: Got status busy True")
  return self.rbx_status.ready == False




