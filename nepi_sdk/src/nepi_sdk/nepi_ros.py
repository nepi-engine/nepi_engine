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
import yaml

import rospy
import rosnode
import rostopic
import rosservice
import rosparam
import rosgraph
from rospy_message_converter import message_converter

# Import ROS msgs and srvs 
from std_msgs.msg import *
from std_srvs.srv import *

from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from geographic_msgs.msg import *

from nepi_interfaces.srv import *
from nepi_interfaces.msg import *



#######################
### Setup some global variables
BASE_NAMESPACE = None
DEFUALT_CFG_FOLDERS = ['/mnt/nepi_storage/user_cfg','/mnt/nepi_config/system_cfg']

#######################
### Log Utility Functions

def set_debug_log(enabled):
  if enabled == True:
    rospy.logger.setLevel(logging.DEBUG)
  else:
    rospy.logger.setLevel(logging.INFO)

def log_msg(msg, level = "None", throttle_s = None, prefix = "", log_name_list = []):
  #if msg is None:
      #msg = "nepi_sdk log_msg got None msg"
  if msg is not None:
    msg_str = prefix + str(msg)
    level = level.lower()
    if level == 'debug':
      log_msg_debug(msg_str, throttle_s = throttle_s, log_name_list = log_name_list)
    elif level == 'warn':
      log_msg_warn(msg_str, throttle_s = throttle_s, log_name_list = log_name_list)
    elif level == 'error':
      log_msg_error(msg_str, throttle_s = throttle_s, log_name_list = log_name_list)
    elif level == 'fatal':
      log_msg_fatal(msg_str, throttle_s = throttle_s, log_name_list = log_name_list)
    else:
      log_msg_info(msg_str, throttle_s = throttle_s, log_name_list = log_name_list)

def log_msg_info(msg, throttle_s = None, log_name_list = []):
  msg_str = str(msg)
  if len(log_name_list) > 0:
      msg_str = str(' : '.join(log_name_list)) + ": " + msg_str
  if throttle_s is None:
    rospy.loginfo(msg_str)
  else:
    rospy.loginfo_throttle(throttle_s,msg_str)

def log_msg_warn(msg, throttle_s = None, log_name_list = []):
  msg_str = str(msg)
  if len(log_name_list) > 0:
      msg_str = str(' : '.join(log_name_list)) + ": " + msg_str
  if throttle_s is None:
    rospy.logwarn(msg_str)
  else:
    rospy.logwarn_throttle(throttle_s,msg_str)

def log_msg_debug(msg, throttle_s = None, log_name_list = []):
  msg_str = str(msg)
  if len(log_name_list) > 0:
      msg_str = str(' : '.join(log_name_list)) + ": " + msg_str
  if throttle_s is None:
    rospy.logdebug(msg_str)
  else:
    rospy.logdebug_throttle(throttle_s,msg_str)

def log_msg_error(msg, throttle_s = None, log_name_list = []):
  msg_str = str(msg)
  if len(log_name_list) > 0:
      msg_str = str(' : '.join(log_name_list)) + ": " + msg_str
  if throttle_s is None:
    log_msg_warn(msg_str)
  else:
    log_msg_warn_throttle(throttle_s,msg_str)

def log_msg_fatal(msg, throttle_s = None, log_name_list = []):
  msg_str = str(msg)
  if len(log_name_list) > 0:
      msg_str = str(' : '.join(log_name_list)) + ": " + msg_str
  if throttle_s is None:
    rospy.logfatal(msg_str)
  else:
    rospy.logfatal_throttle(throttle_s,msg_str)


class logger:

    ln_str = ""

    #######################
    ### IF Initialization
    def __init__(self, log_name = None, log_name_list = []):
        if log_name is not None:
            log_name_list = [log_name] + log_name_list
        if len(log_name_list) > 0:
          self.ln_str = str(' : '.join(log_name_list)) + ": " 

    ###############################
    # Class Public Methods
    
    def log(self, msg, level = "None", throttle_s = None):
        if msg is None:
            msg = "nepi_sdk logger got None msg"
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

def get_full_namespace(namespace, log_name_list = []):
  base_namespace = get_base_namespace()
  node_namespace = get_node_namespace()
  if namespace is None:
    namespace = get_node_namespace()
  elif namespace == 'None':
    namespace = get_node_namespace()
  elif namespace == '':
    namespace = get_base_namespace()
  elif namespace[0] == '~':
    ext_namespace = namespace.replace('~','')
    namespace = os.path.join(get_node_namespace(),ext_namespace)
  if namespace[-1] == '/':
    namespace = namespace[:-1]

  return namespace

def create_namespace(base_namespace,topic):
  #base_namespace = get_full_namespace(base_namespace)

  if len(topic)>0:
    if topic[0] == '/':
      topic = topic[1:]

  namespace = os.path.join(base_namespace,topic)
  namespace = get_full_namespace(namespace)
  return namespace

def get_unique_name_from_namespace(namespace,base_namespace, add_name = None):
    try:
        uid = '_'.join(namespace.split(base_namespace)[1].split('/'))
    except:
        uid = '_'.join(namespace.split('/'))
    if len(uid) > 1:
        if uid[0] == '_':
            uid = uid[1:]
    else:
        uid = os.path.join(base_namespace,'_' + str(time.time_ns())[-6:])
    if add_name is not None:
      uid = uid + '_' + add_name
    uid = uid.replace('.','')
    return uid

#######################
### Node Utility Functions

def init_node(name,disable_signals=False):
  rospy.init_node(name,disable_signals=disable_signals)

  
def get_base_namespace():
  global BASE_NAMESPACE
  if BASE_NAMESPACE is None:
    nepi_names = []
    nepi_node=find_node('nepi')
    nepi_names = nepi_node.split('/')
    if len(nepi_names) < 3:
      while(len(nepi_names) < 3):
        nepi_node=find_node('nepi')
        nepi_names = nepi_node.split('/')
        sleep(.01)
    base_namespace = ('/' + nepi_names[1] + '/' + nepi_names[2])
    BASE_NAMESPACE = base_namespace
  return BASE_NAMESPACE

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
def find_node(node_name, exact = False):
  node = ""
  node_list=get_node_list()
  #log_msg_debug(node_list)
  for node_entry in node_list:
    #log_msg_debug(node_entry)
    if exact == True:
      if node_entry == node_name:
        node = node_entry
        break
    elif node_entry.find(node_name) != -1:
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
def wait_for_node(node_name, timeout = 60, log_name_list = []):
  start_time = get_time()
  timer = 0
  log_msg_debug("nepi_sdk: Waiting for node with name: " + node_name, log_name_list = log_name_list, throttle_s = 5.0)
  node = ""
  while node == "" and timer < timeout and not rospy.is_shutdown():
    node=find_node(node_name)
    time.sleep(.1)
    timer = get_time() - start_time
  log_msg_debug("nepi_sdk: Found node: " + node, log_name_list = log_name_list, throttle_s = 5.0)
  return node


def launch_node(pkg_name, file_name, ros_node_name, namespace = None, device_path = None, log_name_list = []):
  sub_process = None
  msg = 'Success'
  success = False
  if namespace is None:
    namespace = get_base_namespace()

  if device_path is None:
    device_node_run_cmd = ['rosrun', pkg_name, file_name, '__name:=' + ros_node_name, '__ns:=' + namespace]
  else:
    device_node_run_cmd = ['rosrun', pkg_name, file_name, '__name:=' + ros_node_name, '__ns:=' + namespace, '_device_path:=' + device_path]
  try:
    msg = ("nepi_sdk: " + "Launching node " + ros_node_name + " via " + " ".join(x for x in device_node_run_cmd))
    sub_process = subprocess.Popen(device_node_run_cmd)
    success = True
  except Exception as e:
    msg = str("nepi_sdk: " + "Failed to launch node %s with Exception: %s", ros_node_name, str(e), log_name_list = log_name_list)
    log_msg_debug("nepi_sdk: " + msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("nepi_sdk: " + "Failed to start " + ros_node_name + " via " + " ".join(x for x in device_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
      log_msg_warn(msg)
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


def kill_node(node_name, sub_process = None, log_name_list = []):
  success = False
  kill_node = ""
  if check_for_node(node_name):
    nodes = os.popen("rosnode list").readlines()
    for i in range(len(nodes)):
      if nodes[i].find(node_name) != -1:
        kill_node = nodes[i].replace("\n","")
        break
  if kill_node != "":
    os.system("rosnode kill " + kill_node)
    if sub_process is not None:
      sleep(2)
      success = kill_node_process(node_name, sub_process, log_name_list)
  return success


def kill_node_process(node_name, sub_process, log_name_list = []):
    log_msg_warn("nepi_sdk: Killing node: " + node_name, log_name_list = log_name_list)
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
      log_msg_warn("nepi_sdk: Killed node: " + node_name, log_name_list = log_name_list)
    else:
      log_msg_warn("nepi_sdk: failed to node: " + node_name, log_name_list = log_name_list)
    return success
        

def kill_node_namespace(node_namespace, log_name_list = []):
  success = False
  try:
    subprocess.call(["rosnode","kill", node_namespace])
    success = True
  except Exception as e:
    log_msg_debug("nepi_sdk: Failed to kill node_namespace: " + node_namespace + " " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return success

def spin():
  rospy.spin()
  
#########################
### Param Utility Functions

EXAMPLE_PARAMS_DICT = {'threshold': 0.3,'max_rate': 5}

def set_params_from_dict(params_dict, namespace, log_name_list = []):
    for key in params_dict:
      val = params_dict[key]
      namespace = create_namespace(namespace,key)
      try:
          rospy.set_param(namespace,val)
      except Exception as e:
          log_msg_warn("Error creating parameters from param key: " + str(key) + " " + str(e), log_name_list = log_name_list, throttle_s = 5.0)


def load_params_from_file(file_path, namespace = None, prime_key = None, log_name_list = []):
    if namespace is None:
      namespace = "~/"
    namespace = get_full_namespace(namespace)
    log_msg_debug("Will try loading parameters from file: " + file_path, log_name_list = log_name_list)
    params_dict = dict()
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                get_params_dict = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            log_msg_warn("Failed to get dict from file: " + file_path + " " + str(e))
        if prime_key is not None:
          if prime_key in get_params_dict.keys():
            params_dict = get_params_dict[prime_key]
        else:
          params_dict = get_params_dict
        for key in params_dict.keys():
          ns = create_namespace(namespace,key)
          value = params_dict[key]
          set_param(ns,value)
        log_msg_debug("Parameters loaded successfully for " + namespace, log_name_list = log_name_list)
    else:
        log_msg_warn("Param file not found: " + file_path, log_name_list = log_name_list)
    return params_dict

def load_node_config(node_name, load_name = None):
  config_folders = DEFUALT_CFG_FOLDERS
  param_namespace = create_namespace(get_base_namespace(),'config_folders')
  sys_config_folders = get_param(param_namespace)
  if sys_config_folders is not None:
    config_folders = sys_config_folders
  success = False
  config_file = None
  for config_folder in config_folders:

    if load_name is not None:
      load_file = os.path.join(config_folder, load_name + ".yaml")
      if os.path.exists(load_file):
        config_file = load_file
      load_name_all = load_name[0].rsplit('_',1)[0] + '_ALL' # Split on last
      load_file_all = os.path.join(config_folder, load_name_all + ".yaml")
      if os.path.exists(load_file_all):
        config_file = load_file_all
        
    if config_file is None:
      node_file = os.path.join(config_folder, node_name + ".yaml")
      if os.path.exists(node_file):
        config_file = node_file
        if load_name is not None:
          shutil.copy2(config_file, load_file)

      node_name_all = node_name[0].rsplit('_',1)[0] + '_ALL' # Split on last
      node_file_all = os.path.join(config_folder, node_name_all + ".yaml")
      if os.path.exists(node_file_all):
        config_file = node_file_all
        if load_name is not None:
          shutil.copy2(node_file_all, load_file_all)
    if load_name is None:
      load_name = node_name

    if config_file is None:
      log_msg_info("No config file found for " + node_name + " in " + config_folder)
  
  if config_file is not None:
    node_namespace = os.path.join(nepi_sdk.get_base_namespace(), load_name)
    log_msg_warn("Loading parameters from " + config_file + " for " + node_namespace)
    rosparam_load_cmd = ['rosparam', 'load', config_file, node_namespace]
    try:
      subprocess.run(rosparam_load_cmd)
      success = False
    except:
      logger.log_warn("Failed to load config_file: " + config_file)
  return success


def save_params_to_file(file_path, namespace, save_all = False, log_name_list = []):
    namespace = get_full_namespace(namespace)
    params_dict = dict()
    params = get_params(namespace,dict())
    #log_msg_warn("Got params to save: " + str(params), log_name_list = log_name_list)
    if save_all == False: # Use only base ns level
      if params is not None:
        for key in params.keys():
          key = key.replace(namespace + '/','')
          if '/' not in key:
            params_dict[key] = params[key]
    else:
      params_dict = params
    #Try and initialize param values
    try:
      rosparam.dump_params(file_path, namespace)
    except Exception as e:
      log_msg_warn("Could not create params file: " + str(e), log_name_list = log_name_list)
  

def has_param(namespace,param_name = None, log_name_list = []):
  if param_name is not None:
    namespace = create_namespace(namespace,param_name)
  namespace = get_full_namespace(namespace)
  return rospy.has_param(namespace)



def get_params(param_namespace,fallback_param = dict(), log_name_list = []):
  params = None
  param_namespace = get_full_namespace(param_namespace)
  try:
    if fallback_param is None:
      params = rospy.get_param(param_namespace)
    else:
      params = rospy.get_param(param_namespace,fallback_param)
  except Exception as e:
    log_msg_warn("Failed to get param for: " + param_namespace + " " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return params


def get_param(param_namespace,fallback_param = None, log_name_list = []):
  param = None
  param_namespace = get_full_namespace(param_namespace)
  try:
    if fallback_param is None:
      param = rospy.get_param(param_namespace)
    else:
      param = rospy.get_param(param_namespace,fallback_param)
  except Exception as e:
    log_msg_warn("Failed to get param for: " + param_namespace + " " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return param

def set_param(param_namespace,param_val, log_name_list = []):
  success = False
  param_namespace = get_full_namespace(param_namespace)
  try:
    rospy.set_param(param_namespace,param_val)
    success = True
  except Exception as e:
    log_msg_warn("Failed to set param for: " + str(param_namespace) + " "  + str(param_val) + " " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return success

# Function to wait for a param
def wait_for_param(param_namespace, timeout = 60, log_name_list = []):
  start_time = get_time()
  timer = 0
  #log_msg_warn("nepi_sdk: Waiting for param name: " + param_namespace, log_name_list = log_name_list , throttle_s = 10.0)
  param = None
  #log_msg_warn("nepi_sdk: Waiting for param check: " + str([param,param_namespace,timer,rospy.is_shutdown()]), log_name_list = log_name_list ) # , throttle_s = 5.0)
  param_namespace = get_full_namespace(param_namespace)
  while param is None and timer < timeout and not rospy.is_shutdown():
    try:
      param = rospy.get_param(param_namespace)
    except:
      param_namespace = get_full_namespace(param_namespace)

    time.sleep(1)
    timer = get_time() - start_time
  if param == None:
    log_msg_warn("nepi_sdk: Failed to get param: " + str([param,param_namespace,timer,rospy.is_shutdown()]), log_name_list = log_name_list ) # , throttle_s = 5.0)
  return param


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
def find_service(service_name, exact = False):
  found_service = ""
  service_list=get_service_list()
  #log_msg_debug(service_list)
  for service_entry in service_list:
    #log_msg_debug(service_entry[0])
    if  exact == True and service_entry == service_name:
        found_service = service_entry
        break
    elif service_entry.find(service_name) != -1 and service_entry.find(service_name+"_") == -1:
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
def wait_for_service(service_name, timeout = 60, log_name_list = []):
  start_time = get_time()
  timer = 0
  log_msg_debug("nepi_sdk: Waiting for service name: " + service_name, log_name_list = log_name_list, throttle_s = 5.0)
  found_service = ""
  while found_service == "" and timer < timeout and not rospy.is_shutdown():
    found_service=find_service(service_name)
    time.sleep(.01)
    timer = get_time() - start_time
  log_msg_debug("nepi_sdk: Found service: " + found_service, log_name_list = log_name_list, throttle_s = 5.0)
  return found_service

def create_service(service_namespace, srv_msg, srv_callback, log_name_list = []):
  service = None
  #service_namespace = get_full_namespace(service_namespace)
  try:
    service = rospy.Service(service_namespace, srv_msg, srv_callback)
  except Exception as e:
    log_msg_debug("nepi_sdk: Failed to create service: " + str(e) , log_name_list = log_name_list, throttle_s = 5.0)
  return service

def connect_service(service_namespace, service_msg, log_name_list = []):
  service = None
  #service_namespace = get_full_namespace(service_namespace)
  try:
    service = rospy.ServiceProxy(service_namespace, service_msg)
  except Exception as e:
      log_msg_debug("nepi_sdk: Failed to connect to service: " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return service

def call_service(service, request, verbose = True, log_name_list = []):
    response = None
    if service is not None and service != "None":
      try:
          response = service(request)
      except Exception as e:
          if verbose == True:
            log_msg_debug("nepi_sdk: Failed to call service: " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
          else:
            pass
    else:
      log_msg_debug("nepi_sdk: Cant call None service", log_name_list = log_name_list, throttle_s = 5.0)
    return response



#######################
### Topic Utility Functions


def create_subscriber(sub_namespace, msg, callback, queue_size = 10, callback_args = (), log_name_list = []):
  if queue_size is None:
    queue_size = 1
  sub = None
  #sub_namespace = get_full_namespace(sub_namespace)
  try:
    if len(callback_args) == 0:
        sub = rospy.Subscriber(sub_namespace, msg, callback, queue_size = queue_size)
    else:
        sub = rospy.Subscriber(sub_namespace, msg, callback, queue_size = queue_size, callback_args = callback_args)
  except Exception as e:
    log_msg_debug("nepi_sdk: Failed to create subscriber: " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return sub

def create_publisher(pub_namespace, msg, queue_size = 10, latch = False, log_name_list = []):
  if queue_size is None:
    queue_size = 1
  pub = None
  #pub_namespace = get_full_namespace(pub_namespace)
  try:
    pub = rospy.Publisher(pub_namespace, msg, queue_size = queue_size, latch = latch)
  except Exception as e:
    log_msg_debug("nepi_sdk: Failed to create publisher: " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return pub

def publish_pub(publisher, msg, log_name_list = []):
  success = False
  if publisher is not None and msg is not None:
    try:
      publisher.publish(msg)
      success = True
    except Exception as e:
      log_msg_debug("nepi_sdk: Failed to publish message: " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return success


# Function to get list of active topics
def get_topics_data_list():
  topics_list = []
  types_list = []

  topics_data_list = []
  try:
    pubs, subs =rostopic.get_topic_list()
    topics_data_list = pubs
  except Exception as e:
    log_msg_warn("Nepi Sdk rostopic.get_topic_list failed: " + str(e))
    return topics_list,types_list
  

  try:
    for topic_entry in topics_data_list:
      topic_str = topic_entry[0]
      msg_str = topic_entry[1]
      #log_msg_warn("topics check: " + str(topic_str) + " in " + str(topic_names_list))
      if isinstance(topic_str,str):
            topics_list.append(topic_str)
            types_list.append(msg_str)
  except Exception as e:
    log_msg_warn("Nepi Sdk get_topics failed: " + str(e))
  return topics_list,types_list


def get_topics_list():
  topics_list = []
  try:
    [topics_list,types_list]=get_topics_data_list()
  except Exception as e:
    log_msg_warn("Nepi Sdk get_topics failed: " + str(e))
  return topics_list

def get_published_topics():
  return rospy.get_published_topics()

# Function to find a topic
def find_topic(topic_name, exact = False):
  find_topic = ""
  topics_list = []
  try:
    [topics_list,types_list]=get_topics_data_list()
  except:
    pass
  for topic in topics_list:
      if topic.find(topic_name) != -1 and topic.find(topic_name+"_") == -1:
        #log_msg_warn("Found potential topic : " + str(topic_str))
        if exact == True:
          if topic == topic_name:
            find_topic = topic
            break
          else:
            #log_msg_warn("Topic did not match : " + str(topic_name))
            pass
        else:
          find_topic = topic_name
          break
  return find_topic


# Function to find a topic
def find_topics(topic_names_list):
  #log_msg_warn("msg find: " + str(msg_type_list))

  find_topics = []
  topics_list = []

  try:
    [topics_list,types_list]=get_topics_data_list()
    for topic in topics_list:
        for topic_name in topic_names_list:
          if topic_name == topic and topic_name not in find_topics:
            find_topics.append(topic_name)
  except Exception as e:
    log_msg_warn("Nepi Sdk find_topics failed: " + str(e))
  return find_topics


# Function to find a topic
def find_topics_by_msg(msg_type):
  #log_msg_warn("msg find: " + str(msg_type))
  find_topics = []
  try:
    [topics_list,types_list]=get_topics_data_list()
    for i, topic in enumerate(topics_list):
      topic_str = topics_list[i]
      msg_str = types_list[i]
      #log_msg_warn("msg check: " + str([msg_type, topic_str, msg_str]))
      if msg_str.find(msg_type) != -1:
        find_topics.append(topic_str)
        #log_msg_warn("msg found: " + str([msg_type, topic_str]))

  except Exception as e:
    log_msg_warn("Nepi Sdk find_topics_by_msg failed: " + str(e))
  return find_topics

# Function to find a topic
def find_topics_by_msgs(msg_type_list):
  #log_msg_warn("msg find: " + str(msg_type_list))

  find_topics = []
  find_msgs = []
  topics_list = []
  types_list = []
  try:
    [topics_list,types_list]=get_topics_data_list()
    for i, topic in enumerate(topics_list):
      topic_str = topics_list[i]
      msg_str = types_list[i]
      #log_msg_warn("msg check: " + str([msg_type, topic_str, msg_str]))
      for msg_type in msg_type_list:
        if msg_str.find(msg_type) != -1:
          #log_msg_warn("msg check: " + str([topic_str, msg_str]))
          find_topics.append(topic_str)
          find_msgs.append(msg_str)
          #log_msg_warn("msgs found: " + str([msg_type, topic_str]))

  except Exception as e:
    log_msg_warn("Nepi Sdk find_topics_by_msgs failed: " + str(e))
  return find_topics,find_msgs

# Function to find a topic
def find_msg_by_topic(topic):
  find_msg = ''
  topics_list = []
  types_list = []
  try:
    [topics_list,types_list]=get_topics_data_list()
    for i, topic in enumerate(topics_list):
      topic_str = topics_list[i]
      msg_str = types_list[i]
      if topic_str == topic:
        find_msg = msg_str
  except Exception as e:
    log_msg_warn("Nepi Sdk find_msg_by_topic failed: " + str(e))
  return find_msg

# Function to find a topic
def find_topics_by_name(topic_name):

  find_topics = []
  topics_list = []
  types_list = []
  try:
    [topics_list,types_list]=get_topics_data_list()
    for topic in topics_list:
        if topic.index(topic_name) != -1:
          find_topics.append(topic)
  except Exception as e:
    log_msg_warn("Nepi Sdk find_topics failed: " + str(e))
  return find_topics


### Function to check for a topic 
def check_for_topic(topic_name):
  topic_exists = True
  topic=find_topic(topic_name)
  if topic == "":
    topic_exists = False
  return topic_exists

# Function to wait for a topic
def wait_for_topic(topic_name, timeout = 60, log_name_list = []):
  start_time = get_time()
  timer = 0
  log_msg_debug("nepi_sdk: Waiting for topic with name: " + topic_name, log_name_list = log_name_list, throttle_s = 5.0)
  topic = ""
  while topic == "" and timer < timeout and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.01)
    timer = get_time() - start_time
  log_msg_debug("nepi_sdk: Found topic: " + topic, log_name_list = log_name_list, throttle_s = 5.0)
  return topic

def check_for_subscribers(topic_names,filters=[], log_name_list = []):
  [has_subs,has_subs_dict] = find_subscribers(topic_names = topic_names,filters = filters, log_name_list = log_name_list)
  return has_subs


def find_subscribers(topic_names,filters=[], log_name_list = []):
    """
    Checks if topics have any active subscribers.
    """
    topic_names = list(topic_names)
    has_subs = False
    has_subs_dict = dict()
    if len(topic_names) > 0:

      try:
          #log_msg_warn("nepi_sdk: Checking subscribers for: " + str(topic_names), log_name_list = log_name_list)
          
          for topic in topic_names:
            has_subs_dict[topic] = []
          master = rosgraph.Master('/rospy_info')  # Create a Master proxy
          publishers, subscribers = master.getSystemState()[0], master.getSystemState()[1]

          # Check if the topic exists in the list of published topics
          for pub_topic, _ in publishers:
              for topic in topic_names:
                if pub_topic == topic:
                    # If the topic is published, check if it has any subscribers
                    for sub_topic, _ in subscribers:
                        #log_msg_warn("nepi_sdk: Found subscriber: " + sub_topic + " for topic " + pub_topic, log_name_list = log_name_list)
                        if sub_topic not in filters:
                            has_subs = True
                            has_subs_dict[topic].append(sub_topic)  # Found subscribers for this 

      except rospy.ROSException as e:
          rospy.logerr(f"Error connecting to ROS Master: {e}")
          return has_subs, has_subs_dict
    return has_subs, has_subs_dict


#########################
### Publisher, Subscriber, and Service Utility Functions

def start_timer_process(duration, callback_function, oneshot = False, log_name_list = []):
  success = False
  if isinstance(duration,int) or isinstance(duration,float):
    duration = ros_duration(duration)
  try:
    rospy.Timer(duration, callback_function, oneshot)
    success = True
  except Exception as e:
    log_msg_warn("Failed to start timer process: " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
  return success





#########################
### Time Helper Functions

def get_time():
  return time.time_ns() / 1000000000


def get_msg_time_type():
  return rospy.rostime.Time

def get_msg_stamp():
    return rospy.Time.now()


def msg_stamp_from_sec(time_sec = None):
  if time_sec is None:
    return rospy.Time.now()
  else:
    return rospy.Time(time_sec)


def sec_from_msg_stamp(msg_stamp):
  time_sec = 0.0
  if hasattr(msg_stamp,'sec'):
    sec_str = str(stamp.sec) + '.' + str(stamp.nsecs)
    time_sec = float(sec_str)
  elif isinstance(msg_stamp,int) == True:
    msg_stamp_str = str(msg_stamp)
    if len(msg_stamp_str) > 9:
      sec_str = msg_stamp_str[0:-9]
    else:
      sec_str = '0'
    dec_str = msg_stamp_str[-9:]
    time_str = sec_str + '.' + dec_str
    time_sec = float(time_str)
  return time_sec



def msg_stamp_from_timestamp(timestamp, log_name_list = []):
    log_msg_debug("Got timestamp to convert to stamp: " + str(timestamp), log_name_list = log_name_list, throttle_s = 5.0)
    log_msg_debug("Got timestamp type to convert to stamp: " + str(type(timestamp)), log_name_list = log_name_list, throttle_s = 5.0)
    stamp = get_msg_stamp()
    if timestamp is None:
        return stamp
    elif isinstance(timestamp,get_msg_time_type()) == False:
          if isinstance(timestamp,float) == True:
              stamp = msg_stamp_from_sec(timestamp)
          elif isinstance(timestamp,int) == True:
              stamp = msg_stamp_from_sec(timestamp)
    else:
      try:
        time_sec = sec_from_msg_stamp(timestamp)
        stamp = msg_stamp_from_sec(time_sec)
      except:
        pass
    return stamp


def sec_from_timestamp(timestamp = None, log_name_list = []):
    log_msg_debug("Got timestamp to convert to sec: " + str(timestamp), log_name_list = log_name_list, throttle_s = 5.0)
    log_msg_debug("Got timestamp type to convert to sec: " + str(type(timestamp)), log_name_list = log_name_list, throttle_s = 5.0)
    if timestamp is None:
        time_sec = get_time()
    else:
        if isinstance(timestamp,get_msg_time_type()) == True:
              try:
                time_sec = sec_from_msg_stamp(timestamp)
              except:
                time_sec = get_time()
        elif isinstance(timestamp,float) == True:
            time_sec = timestamp
        else:
            time_sec = get_time()
    return time_sec



def ros_duration(time_sec):
  return duration(time_sec)

def duration(time_sec):
  return rospy.Duration(time_sec)

def ros_rate(time_sec):
  return rospy.Rate(time_sec)


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

  # Sleep process that breaks sleep into smaller times for better shutdown
def wait(wait_time = 0.1):
  sleep(wait_time)
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

def create_header_msg(time_sec = None, frame_id = None):
  header = Header()
  header.stamp = msg_stamp_from_sec(time_sec)
  if frame_id is not None:
    header.frame_id = frame_id
  return header

def parse_header_msg(header_msg):
  time_sec = sec_from_msg_stamp(header_msg.stamp)
  frame_id = header_msg.frame_id
  return time_sec,frame_id
  
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
