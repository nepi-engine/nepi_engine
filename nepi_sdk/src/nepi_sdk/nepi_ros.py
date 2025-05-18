#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI ros utility functions include
# 1) ROS Node utility functions
# 2) ROS Topic utility functions
# 3) ROS Service utility functions
# 4) ROS Param utility functions
# 5) ROS publisher, subscriber, and service
# 6) ROS Time utility functions
# 7) Misc helper functions

  
import os
import sys
import importlib
import shutil
import time
from datetime import datetime
import subprocess

import rospy
import rosnode
import rostopic
import rosservice
import rosparam
from rospy_message_converter import message_converter

# Import ROS msgs and srvs 
from std_msgs.msg import *
from std_srvs.srv import *

from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from geographic_msgs.msg import *

from nepi_ros_interfaces.srv import *
from nepi_ros_interfaces.msg import *


#######################
### Log Utility Functions


def log_msg(msg, level = "None", throttle_s = None, prefix = ""):
  #if msg is None:
      #msg = "nepi_ros log_msg got None msg"
  if msg is not None:
    msg_str = prefix + str(msg)
    level = level.lower()
    if level == 'debug':
      log_msg_debug(msg_str, throttle_s = throttle_s)
    elif level == 'warn':
      log_msg_warn(msg_str, throttle_s = throttle_s)
    elif level == 'error':
      log_msg_error(msg_str, throttle_s = throttle_s)
    elif level == 'fatal':
      log_msg_fatal(msg_str, throttle_s = throttle_s)
    else:
      log_msg_info(msg_str, throttle_s = throttle_s)

def log_msg_info(msg, throttle_s = None):
  msg_str = str(msg)
  if throttle_s is None:
    rospy.loginfo(msg_str)
  else:
    rospy.loginfo_throttle(throttle_s,msg_str)

def log_msg_warn(msg, throttle_s = None):
  msg_str = str(msg)
  if throttle_s is None:
    rospy.logwarn(msg_str)
  else:
    rospy.logwarn_throttle(throttle_s,msg_str)

def log_msg_debug(msg, throttle_s = None):
  msg_str = str(msg)
  if throttle_s is None:
    rospy.logdebug(msg_str)
  else:
    rospy.logdebug_throttle(throttle_s,msg_str)

def log_msg_error(msg, throttle_s = None):
  msg_str = str(msg)
  if throttle_s is None:
    rospy.logerr(msg_str)
  else:
    rospy.logerr_throttle(throttle_s,msg_str)

def log_msg_fatal(msg, throttle_s = None):
  msg_str = str(msg)
  if throttle_s is None:
    rospy.logfatal(msg_str)
  else:
    rospy.logfatal_throttle(throttle_s,msg_str)


class logger:

    ln_str = ""

    #######################
    ### IF Initialization
    def __init__(self, log_name = None):
        if log_name is not None:
            self.ln_str = log_name + ": "

    ###############################
    # Class Public Methods
    
    def log(self, msg, level = "None", throttle_s = None):
        if msg is None:
            msg = "nepi_ros logger got None msg"
        if msg is not None:
          msg_str = self._createMsgString(msg)
          log_msg(msg_str, level = level, throttle_s = throttle_s)

    
    def log_info(self, msg, throttle_s = None):
        self.log(msg, level = 'info', throttle_s = throttle_s)
    
    def log_warn(self, msg, throttle_s = None):
        self.log(msg,level = 'warn', throttle_s = throttle_s)
    
    def log_debug(self, msg, throttle_s = None):
        self.log(msg,level = 'debug', throttle_s = throttle_s)
    
    def log_error(self, msg, throttle_s = None):
        self.log(msg,level = 'error', throttle_s = throttle_s)
    
    def log_fatal(self, msg, throttle_s = None):
        self.log(msg,level = 'fatal', throttle_s = throttle_s)

    ###############################
    # Class Private Methods
    ###############################
  
    def _createMsgString(self,msg):
         return self.ln_str + str(msg)




#######################
### Namespace Utility Functions

def get_full_namespace(namespace):
  base_namespace = get_base_namespace()
  node_namespace = get_node_namespace()
  if namespace is None:
    namespace = node_namespace
  elif namespace == 'None':
    namespace = node_namespace
  elif namespace == '':
    namespace = base_namespace
  elif namespace[0] == '~':
    ext_namespace = namespace.replace('~','')
    namespace = os.path.join(node_namespace,ext_namespace)
  if namespace.find(base_namespace) == -1:
    if namespace[0] == '/':
      namespace = namespace[1:]
    namespace = os.path.join(base_namespace,namespace)
  return namespace

def create_namespace(base_namespace,topic):
  #base_namespace = get_full_namespace(base_namespace)

  if len(topic)>0:
    if topic[0] == '/':
      topic = topic[1:]

  namespace = os.path.join(base_namespace,topic)
  return namespace



#######################
### Node Utility Functions

def init_node(name,disable_signals=False):
  rospy.init_node(name,disable_signals=disable_signals)

  
def get_base_namespace():
  nepi_node=find_node('nepi')
  nepi_names = nepi_node.split('/')
  base_namespace = ('/' + nepi_names[1] + '/' + nepi_names[2] + '/')
  return base_namespace

def get_node_namespace():
  namespace = rospy.get_name()
  if namespace[-1] == "/":
    namespace = namespace[:-1] 
  return namespace
  
def get_node_name():
  return get_node_namespace().split('/')[-1]



# Function to get list of active topics
def get_node_list():
  node_list=rosnode.get_node_names()
  return node_list

### Function to find a node
def find_node(node_name):
  node = ""
  node_list=get_node_list()
  #rospy.loginfo(node_list)
  for node_entry in node_list:
    #rospy.loginfo(node_entry)
    if node_entry.find(node_name) != -1:
      node = node_entry
      break
  return node

### Function to check for a node 
def check_for_node(node_name):
  node_exists = True
  node=find_node(node_name)
  if node == "":
    node_exists = False
  return node_exists

### Function to wait for a node
def wait_for_node(node_name, timeout = float('inf')):
  start_time = get_time()
  timer = 0
  rospy.logwarn("nepi_ros: Waiting for node with name: " + node_name)
  node = ""
  while node == "" and timer < timeout and not rospy.is_shutdown():
    node=find_node(node_name)
    time.sleep(.1)
    timer = get_time() - start_time
  rospy.loginfo("nepi_ros: Found node: " + node)
  return node


def launch_node(pkg_name, file_name, ros_node_name, device_path = None):
  sub_process = None
  msg = 'Success'
  success = False
  if device_path is None:
    device_node_run_cmd = ['rosrun', pkg_name, file_name, '__name:=' + ros_node_name]
  else:
    device_node_run_cmd = ['rosrun', pkg_name, file_name, '__name:=' + ros_node_name, '_device_path:=' + device_path]
  try:
    sub_process = subprocess.Popen(device_node_run_cmd)
    success = True
  except Exception as e:
    msg = str("Failed to launch node %s with exception: %s", ros_node_name, str(e))
    rospy.logwarn("NEPI_NEX: " + msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("Failed to start " + device_node_name + " via " + " ".join(x for x in device_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
      rospy.logerr(msg)
      sub_process = None
      success = False
  return success, msg, sub_process
  
def check_node_by_name(node_name):
    running = check_for_node(node_name)
    return running


def check_node_by_process(sub_process):
    running = True
    if sub_process.poll() is None:
      running = False
    return running


def kill_node(node_name):
  kill_node = ""
  if check_for_node(node_name):
    nodes = os.popen("rosnode list").readlines()
    for i in range(len(nodes)):
      if nodes[i].find(node_name) != -1:
        kill_node = nodes[i].replace("\n","")
        break
  if kill_node != "":
    os.system("rosnode kill " + kill_node)

def kill_node_process(node_namespace,sub_process):
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
        

def kill_node_namespace(node_namespace):
  try:
    subprocess.call(["rosnode","kill", node_namespace])
  except Exception as e:
    rospy.logwarn("nepi_ros: Failed to kill node_namespace: " + node_namespace + " " + str(e))

def spin():
  rospy.spin()
  

#######################
### Topic Utility Functions


def create_subscriber(sub_namespace, msg, callback, queue_size = None, callback_args=[]):
  sub = None
  sub_namespace = get_full_namespace(sub_namespace)
  try:
    if len(callback_args) == 0:
        sub = rospy.Subscriber(sub_namespace, msg, callback, queue_size = queue_size)
    else:
        sub = rospy.Subscriber(sub_namespace, msg, callback, queue_size = queue_size, callback_args=callback_args)
  except Exception as e:
    rospy.logwarn("nepi_ros: Failed to create subscriber: " + str(e))
  return sub

def create_publisher(pub_namespace, msg, queue_size = None,  latch = False):
  pub = None
  pub_namespace = get_full_namespace(pub_namespace)
  try:
    pub = rospy.Publisher(pub_namespace, msg, queue_size = queue_size,  latch = latch)
  except Exception as e:
    rospy.logwarn("nepi_ros: Failed to create publisher: " + str(e))
  return pub


# Function to get list of active topics
def get_topic_list():
  topic_list = []
  try:
    pubs, subs =rostopic.get_topic_list()
    topic_list = pubs + subs
  except:
    pass
  return topic_list

def get_published_topics():
  return rospy.get_published_topics()

# Function to find a topic
def find_topic(topic_name):
  topic = ""
  topic_list = []
  try:
    topic_list=get_topic_list()
  except:
    pass
  for topic_entry in topic_list:
    topic_str = topic_entry[0]
    if isinstance(topic_str,str):
      if topic_str.find(topic_name) != -1 and topic_str.find(topic_name+"_") == -1:

        topic = topic_str
        break
  return topic

# Function to find a topic
def find_topics_by_msg(msg_type):
  topic_list = []
  try:
    topics=get_topic_list()
    for topic_entry in topics:
      topic_str = topic_entry[0]
      msg_str = topic_entry[1]
      if isinstance(topic_str,str) and isinstance(msg_str,str):
        if msg_str.find(msg_type) != -1:
          topic_list.append(topic_str)
  except:
    pass
  return topic_list

# Function to find a topic
def find_topics_by_name(topic_name):
  topic_list = []
  try:
    topics=get_topic_list()
    for topic_entry in topics:
      topic_str = topic_entry[0]
      msg_str = topic_entry[1]
      if topic_name == os.path.basename(topic_str):
        topic_list.append(topic_str)
  except:
    pass
  return topic_list

### Function to check for a topic 
def check_for_topic(topic_name):
  topic_exists = True
  topic=find_topic(topic_name)
  if topic == "":
    topic_exists = False
  return topic_exists

# Function to wait for a topic
def wait_for_topic(topic_name, timeout = float('inf')):
  start_time = get_time()
  timer = 0
  rospy.loginfo("nepi_ros: Waiting for topic with name: " + topic_name)
  topic = ""
  while topic == "" and timer < timeout and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.1)
    timer = get_time() - start_time
  rospy.loginfo("nepi_ros: Found topic: " + topic)
  return topic

#######################
### Service Utility Functions

# Function to get list of active topics
def get_service_list():
  services_list=rosservice.get_service_list()
  return services_list

# Function to get list of active services
def get_published_services_list(search_namespace='/'):
  services_list=rospy.get_published_services(namespace=search_namespace)
  return services_list

# Function to find a service
def find_service(service_name):
  found_service = ""
  service_list=get_service_list()
  #rospy.loginfo(service_list)
  for service_entry in service_list:
    #rospy.loginfo(service_entry[0])
    if service_entry.find(service_name) != -1 and service_entry.find(service_name+"_") == -1:
      found_service = service_entry
      break
  return found_service

# Function to find a service
def find_services_by_msg(msg_type):
  service_list = []
  try:
    services=get_service_list()
    for service_entry in services:
      service_str = service_entry[0]
      msg_str = service_entry[1]
      if isinstance(service_str,str) and isinstance(msg_str,str):
        if msg_str.find(msg_type) != -1:
          service_list.append(service_str)
  except:
    pass
  return service_list

# Function to find a service
def find_services_by_name(service_name):
  service_list = []
  try:
    services=get_service_list()
    for service_entry in services:
      service_str = service_entry[0]
      msg_str = service_entry[1]
      if service_name == os.path.basename(service_str):
        service_list.append(service_str)
  except:
    pass
  return service_list



### Function to check for a service 
def check_for_service(service_name):
  service_exists = True
  found_service=find_service(service_name)
  if found_service == "":
    service_exists = False
  return service_exists

# Function to wait for a service
def wait_for_service(service_name, timeout = float('inf')):
  start_time = get_time()
  timer = 0
  rospy.loginfo("nepi_ros: Waiting for service name: " + service_name)
  found_service = ""
  while found_service == "" and timer < timeout and not rospy.is_shutdown():
    found_service=find_service(service_name)
    time.sleep(.1)
    timer = get_time() - start_time
  rospy.loginfo("nepi_ros: Found service: " + found_service)
  return found_service

def create_service(service_namespace, srv_msg, srv_callback):
  service = None
  service_namespace = get_full_namespace(service_namespace)
  try:
    service = rospy.Service(service_namespace, srv_msg, srv_callback)
  except Exception as e:
    rospy.loginfo("nepi_ros: Failed to create service: " + str(e) )
  return service

def connect_service(service_namespace, service_msg):
  service = None
  service_namespace = get_full_namespace(service_namespace)
  try:
    service = rospy.ServiceProxy(service_namespace, service_msg)
  except Exception as e:
      rospy.loginfo("nepi_ros: Failed to connect to service: " + str(e) )
  return service

def call_service(service, request):
    response = None
    if service is not None and service != "None":
      try:
          response = service(request)
      except Exception as e:
          rospy.loginfo("nepi_ros: Failed to call service: " + str(e) )
    else:
      rospy.loginfo("nepi_ros: Cant call None service")
    return response

#########################
### Param Utility Functions

EXAMPLE_PARAMS_DICT = {'threshold': 0.3,'max_rate': 5}

def upload_params(namespace, params, verbose=False):
    try:
        rospy.upload_params(namespace, params, verbose=verbose)
    except rosparam.RosParamException as e:
        rospy.logerr("Error uploading parameters from param " + str(e))

def load_params_from_dict(params_dict, params_namespace):
    for key in params_dict:
      val = params_dict[key]
      namespace = create_namespace(params_namespace,key)
    try:
        rospy.set_param(namespace,val)
    except rosparam.RosParamException as e:
        rospy.logerr("Error creating parameters from param key: " + str(key) + " " + str(e))


def load_params_from_file(file_path, params_namespace = None):
    if params_namespace is not None:
      if params_namespace[-1] != "/":
        params_namespace += "/"
    else:
      params_namespace = "~/"
    rospy.logwarn("Will try loading parameters from file: " + file_path)
    try:
        params_input = rosparam.load_file(file_path)
    except rosparam.RosParamException as e:
        rospy.logwarn("Error loading parameters from file: " + file_path + " " + str(e))
    try:
        if params_input != []:
          #rospy.logwarn("nepi_ros: loaded params" + str(params_input) + " for " + params_namespace)
          params = params_input[0][0]
          for key in params.keys():
              value = params[key]
              param_namesapce = params_namespace + key
              #rospy.logwarn("nepi_ros: setting param " + key + " value: " + str(value)  + " for " + params_namespace)
              rospy.set_param(param_namesapce, value)
          rospy.loginfo("Parameters loaded successfully for " + params_namespace)
    except rosparam.RosParamException as e:
        rospy.logwarn("Error updating parameters: " + file_path + " " + str(e))


def save_params_to_file(file_path, namespace):
    #Try and initialize app param values
    try:
      rosparam.dump_params(file_path, namespace)
    except Exception as e:
      print("Could not create params file: " + str(e))
  

def has_param(param_namespace):
  #param_namespace = get_full_namespace(param_namespace)
  return rospy.has_param(param_namespace)

def get_param(param_namespace,fallback_param = None):
  param = None
  #param_namespace = get_full_namespace(param_namespace)
  try:
    if fallback_param is None:
      param = rospy.get_param(param_namespace)
    else:
      param = rospy.get_param(param_namespace,fallback_param)
  except Exception as e:
    rospy.logerr("Failed to get param for: " + param_namespace + " " + str(e))
  return param

def set_param(param_namespace,param_val):
  success = False
  #param_namespace = get_full_namespace(param_namespace)
  try:
    rospy.set_param(param_namespace,param_val)
    success = True
  except Exception as e:
    rospy.logerr("Failed to set param for: " + param_namespace + " "  + param_val + " " + str(e))
  return success


def print_node_params():
  node_name = rospy.get_name()
  all_params = rospy.get_param_names()
  node_params = [param for param in all_params if node_name in param]
  if node_params:
      print(f"Parameters for node '{node_name}':")
      for param in node_params:
          try:
              param_value = rospy.get_param(param)
              print(f"  {param}: {param_value}")
          except Exception as e:
              print(f"  {param}: Error retrieving value - {e}")
  else:
      print(f"No parameters found for node '{node_name}'.")


def load_params_from_file(file_path, params_namespace = None):
    if params_namespace is not None:
      if params_namespace[-1] != "/":
        params_namespace += "/"
    else:
      params_namespace = ""
    try:
        params_input = rosparam.load_file(file_path)
        if params_input != []:
          #rospy.logwarn("NEPI_ROS: loaded params" + str(params_input) + " for " + params_namespace)
          params = params_input[0][0]
          for key in params.keys():
              value = params[key]
              param_namesapce = params_namespace + key
              #rospy.logwarn("NEPI_ROS: setting param " + key + " value: " + str(value)  + " for " + params_namespace)
              rospy.set_param(param_namesapce, value)
          rospy.loginfo("Parameters loaded successfully for " + params_namespace)
    except rosparam.RosParamException as e:
        rospy.logerr("Error loading parameters from file: " + file_path + " " + str(e))

#########################
### Publisher, Subscriber, and Service Utility Functions

def start_timer_process(duration, callback_function, oneshot = False):
  success = False
  if isinstance(duration,int) or isinstance(duration,float):
    duration = ros_duration(duration)
  try:
    rospy.Timer(duration, callback_function, oneshot)
    success = True
  except Exception as e:
    rospy.logerr("Failed to start timer process: " + str(e))
  return success
  

#########################
### Time Helper Functions

def get_time():
  return time.time_ns() / 1000000000


def get_ros_time_type():
  return rospy.rostime.Time

def ros_time_now():
  return rospy.Time.now()

def get_rostime():
  return rospy.get_rostime()

def ros_time_from_sec(time_ns):
  return rospy.Time.from_sec(time_ns)

def ros_time(time_ns = None):
  if time_ns is None:
    return ros_time_now()
  else:
    return ros_time_from_sec(time_ns)


def sec_from_ros_time(ros_time):
  ros_time_str = str(ros_time)
  if len(ros_time_str) > 9:
    sec_str = ros_time_str[0:-9]
  else:
    sec_str = '0'
  dec_str = ros_time_str[-9:]
  time_str = sec_str + '.' + dec_str
  time_sec = float(time_str)
  return time_sec



def get_ros_stamp():
    ros_time = ros_time_now()
    return ros_stamp_from_ros_time(ros_time)

def ros_stamp_now():
    ros_time = ros_time_now()
    return ros_stamp_from_ros_time(ros_time)


def ros_stamp_from_ros_time(ros_time):
  ros_stamp = ros_time
  return ros_stamp


def ros_stamp_from_sec(time_sec):
  ros_time = ros_time_from_sec(time_sec)
  ros_stamp = ros_stamp_from_ros_time(ros_time)
  return ros_stamp
  
def sec_from_ros_stamp(stamp):
  sec_str = str(stamp.secs) + '.' + str(stamp.nsecs)
  sec = float(sec_str)
  return sec


def ros_stamp_from_timestamp(timestamp):
    #rospy.logwarn("Got timestamp to convert to stamp: " + str(timestamp))
    #rospy.logwarn("Got timestamp type to convert to stamp: " + str(type(timestamp)))
    if timestamp is None:
        stamp = get_ros_stamp()
    elif isinstance(timestamp,get_ros_time_type()) == False:
          if isinstance(timestamp,float) == True:
              stamp = ros_stamp_from_sec(timestamp)
          else:
              stamp = get_ros_stamp()
    else:
      try:
        stamp = ros_stamp_from_ros_time(timestamp)
      except:
        stamp = get_ros_stamp()
    return stamp



def sec_from_timestamp(timestamp = None):
    #rospy.logwarn("Got timestamp to convert to sec: " + str(timestamp))
    #rospy.logwarn("Got timestamp type to convert to sec: " + str(type(timestamp)))
    if timestamp is None:
        time_ns = get_time()
    else:
        if isinstance(timestamp,get_ros_time_type()) == True:
              try:
                time_ns = sec_from_ros_stamp(timestamp)
              except:
                try:
                  time_ns = sec_from_ros_time(timestamp)
                except:
                  time_ns = get_time()
        elif isinstance(timestamp,float) == True:
            time_ns = timestamp
        else:
            time_ns = get_time()
    return time_ns



def ros_duration(time_s):
  return duration(time_s)

def duration(time_s):
  return rospy.Duration(time_s)

def ros_rate(time_s):
  return rospy.Rate(time_s)


# Sleep process that breaks sleep into smaller times for better shutdown
def sleep(sleep_sec,sleep_steps = None):
  if sleep_sec <= 1:
    time.sleep(sleep_sec)
  else:
    if sleep_steps is None:
      if sleep_sec > 1:
        sleep_steps = int(sleep_sec * 10)
      else:
        sleep_steps = 1
    if sleep_sec > 0 and sleep_steps >= 0:
      if sleep_steps != 0:
        delay_timer = 0
        delay_sec = sleep_sec/sleep_steps
        while delay_timer < sleep_sec and not rospy.is_shutdown():
          time.sleep(delay_sec)
          delay_timer = delay_timer + delay_sec
      else:
        time.sleep(sleep_sec)
  return True



def get_datetime_str_from_stamp(ros_stamp_msg):
  tm=time.gmtime(ros_stamp_msg.secs)
  year_str = time_part_2_valid_str(tm.tm_year)
  mon_str = time_part_2_valid_str(tm.tm_mon)
  day_str = time_part_2_valid_str(tm.tm_mday)
  hr_str = time_part_2_valid_str(tm.tm_hour)
  min_str = time_part_2_valid_str(tm.tm_min)
  sec_str = time_part_2_valid_str(tm.tm_sec)
  date_str=(year_str + '-' + mon_str + '-' + day_str)
  time_str = (hr_str + min_str + sec_str)
  ms_str= str(ros_stamp_msg.nsecs)[0:3]
  dt_str = (date_str + "T" + time_str + "." + ms_str)
  return dt_str

def get_datetime_dict_from_stamp(ros_stamp_msg):
  tm=time.gmtime(ros_stamp_msg.secs)
  datatime = dict()
  datatime['year'] = (tm.tm_year)
  datatime['mon'] = (tm.tm_mon)
  datatime['day'] = (tm.tm_mday)
  datatime['hr'] = (tm.tm_hour)
  datatime['min'] = (tm.tm_min)
  datatime['sec'] = (tm.tm_sec)
  return datatime

def time_part_2_valid_str(tm_val):
  tm_str = str(tm_val)
  if len(tm_str) == 1:
    tm_str = ('0'+ tm_str)
  return tm_str


def get_time():
  return time.time()

#########################
### Msg Helper Functions

def convert_msg2dict(msg):
  msg_dict = None
  if msg is not None:
    msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
  return msg_dict


def parse_string_list_msg_data(msg_data):
  str_list = []
  if msg_data[0] == "[" and msg_data[-1] == "]" :
    str_list = eval(msg_data)
  return(str_list)

#########################
### Shutdown Helper Functions

def is_shutdown():
  return rospy.is_shutdown()

def signal_shutdown(msg):
  rospy.signal_shutdown(msg)

def on_shutdown(shutdown_fuction):
  rospy.on_shutdown(shutdown_fuction)

def Time(float):
  rospy.Time(float)

def wait_for_service(wait_topic, timeout_s):
  rospy.wait_for_service(wait_topic, timeout_s)