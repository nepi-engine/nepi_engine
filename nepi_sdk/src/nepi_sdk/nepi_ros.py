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
import rospy
import rosnode
import rostopic
import rosservice
import rosparam
import time
import subprocess


from datetime import datetime
from std_msgs.msg import Empty, Float32, Header
from std_srvs.srv import Empty, EmptyRequest, Trigger
from nepi_ros_interfaces.srv import GetScriptsQuery,GetRunningScriptsQuery ,LaunchScript, StopScript
from nepi_ros_interfaces.msg import  Setting


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
    #rospy.loginfo(node_entry[0])
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
  rospy.loginfo("NEPI_ROS: Waiting for node with name: " + node_name)
  node = ""
  while node == "" and timer < timeout and not rospy.is_shutdown():
    node=find_node(node_name)
    time.sleep(.1)
    timer = get_time() - start_time
  rospy.loginfo("NEPI_ROS: Found node: " + node)
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
  
def check_node(node_namespace,sub_process):
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
    rospy.logwarn("NEPI_ROS: Failed to kill node_namespace: " + node_namespace + " " + str(e))

def spin():
  rospy.spin()
  

#######################
### Topic Utility Functions

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
  rospy.loginfo("NEPI_ROS: Waiting for topic with name: " + topic_name)
  topic = ""
  while topic == "" and timer < timeout and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.1)
    timer = get_time() - start_time
  rospy.loginfo("NEPI_ROS: Found topic: " + topic)
  return topic

#######################
### Service Utility Functions

# Function to get list of active topics
def get_service_list():
  service = ""
  service_list=rosservice.get_service_list()
  return service_list

# Function to get list of active services
def get_published_service_list(search_namespace='/'):
  service = ""
  service_list=rospy.get_published_services(namespace=search_namespace)
  return service_list

# Function to find a service
def find_service(service_name):
  service = ""
  service_list=get_service_list()
  #rospy.loginfo(service_list)
  for service_entry in service_list:
    #rospy.loginfo(service_entry[0])
    if service_entry.find(service_name) != -1 and service_entry.find(service_name+"_") == -1:
      service = service_entry
      break
  return service

### Function to check for a service 
def check_for_service(service_name):
  service_exists = True
  service=find_service(service_name)
  if service == "":
    service_exists = False
  return service_exists

# Function to wait for a service
def wait_for_service(service_name, timeout = float('inf')):
  start_time = get_time()
  timer = 0
  rospy.loginfo("NEPI_ROS: Waiting for servcie name: " + service_name)
  service = ""
  while service == "" and timer < timeout and not rospy.is_shutdown():
    service=find_service(service_name)
    time.sleep(.1)
    timer = get_time() - start_time
  rospy.loginfo("NEPI_ROS: Found service: " + service)
  return service


#########################
### Param Utility Functions

def has_param(self,param_namespace):
  return rospy.has_param(param_namespace)

def get_param(self,param_namespace,fallback_param = None):
  if fallback_param is None:
    param = rospy.get_param(param_namespace)
  else:
    param = rospy.get_param(param_namespace,fallback_param)
  return param

def set_param(self,param_namespace,param):
  rospy.set_param(param_namespace,param)

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
  rospy.Timer(duration, callback_function, oneshot)

def timer(duration, callback_function, oneshot = False):
  rospy.Timer(duration, callback_function, oneshot)

'''
def getPublisher(namespace, msg_type, queue_size=1):
  return rospy.Publisher(namespace, msg_type, queue_size)

def startSubscriber(namespace, msg_type, callback_function, queue_size=1):
  return rospy.Subscriber(namespace, msg_type, callback_function, queue_size)
'''

#########################
### Time Helper Functions

def ros_time_now():
  return rospy.Time.now()

'''
# Anothe way to get ros_time
def get_rostime():
  return rospy.get_rostime()
'''

def ros_time_from_ros_stamp(stamp):
  return rospy.Time.from_sec(stamp)

def ros_time_from_sec(time_sec):
  return rospy.Time.from_sec(time_sec)

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


def ros_stamp_from_ros_time(ros_time):
  ros_time_str = str(ros_time)
  if len(ros_time_str) > 9:
    sec_str = ros_time_str[0:-9]
  else:
    sec_str = '0'
  nsecs_str = ros_time_str[-9:]
  header = Header()
  ros_stamp = stamp = header.stamp
  ros_stamp.sec = int(sec_str)
  ros_stamp.nsecs =  int(nsecs_str)
  return ros_stamp

def ros_stamp_from_sec(time_sec):
  ros_time = ros_time_from_sec(time_sec)
  ros_stamp = ros_stamp_from_ros_time(ros_time)
  return ros_stamp

def sec_from_ros_stamp(stamp):
  sec_str = str(stamp.sec) + '.' + str(stamp.nsecs)
  sec = float(sec_str)
  return sec
  

def ros_duration(time_s):
  return rospy.Duration(time_s)

def ros_rate(time_s):
  return rospy.Rate(time_s)


# Sleep process that breaks sleep into smaller times for better shutdown
def sleep(sleep_sec,sleep_steps = None):
  if sleep_steps is None:
    if sleep_sec >= 1:
      sleep_steps = int(sleep_sec * 10)
    else:
      sleep_steps = 1
  if sleep_sec > 0 and sleep_steps >= 0:
    if sleep_steps != 0:
      delay_timer = 0
      delay_sec = sleep_sec/sleep_steps
      while delay_timer < sleep_sec and not rospy.is_shutdown():
        rospy.sleep(delay_sec)
        delay_timer = delay_timer + delay_sec
    else:
      rospy.sleep(sleep_sec)
  return True

def get_datetime_str_now():
  date_str=datetime.utcnow().strftime('%Y-%m-%d')
  time_str=datetime.utcnow().strftime('%H%M%S')
  ms_str =datetime.utcnow().strftime('%f')[:-3]
  dt_str = (date_str + "T" + time_str + "." + ms_str)
  return dt_str

def get_datetime_str_from_stamp(ros_stamp_msg):
  tm=time.gmtime(ros_stamp_msg.secs)
  year_str = tm_2_str(tm.tm_year)
  mon_str = tm_2_str(tm.tm_mon)
  day_str = tm_2_str(tm.tm_mday)
  hr_str = tm_2_str(tm.tm_hour)
  min_str = tm_2_str(tm.tm_min)
  sec_str = tm_2_str(tm.tm_sec)
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

def get_time():
  return time.time_ns() / 1000000000


def tm_2_str(tm_val):
  tm_str = str(tm_val)
  if len(tm_str) == 1:
    tm_str = ('0'+ tm_str)
  return tm_str


#########################
### Misc Helper Functions

def is_shutdown():
  return rospy.is_shutdown()

def signal_shutdown(msg):
  rospy.signal_shutdown(msg)

def on_shutdown(shutdown_fuction):
  rospy.on_shutdown(shutdown_fuction)

def parse_string_list_msg_data(msg_data):
  str_list = []
  if msg_data[0] == "[" and msg_data[-1] == "]" :
    str_list = eval(msg_data)
  return(str_list)




# Function for checking if val in list
def val_in_list(val2check,list2check):
  in_list = False
  if len(list2check) > 0:
    for list_val in list2check:
      #rospy.loginfo(str(val2check) + ' , ' + str(list_val))
      #rospy.loginfo(val2check == list_val)
      if val2check == list_val:
        in_list = True
  return in_list

def get_folder_list(search_path):
  filelist=os.listdir(search_path + '/')
  folder_list=[]
  #print('')
  #print('Files and Folders in Path:')
  #print(search_path)
  #print(filelist)
  for file in enumerate(filelist):
    foldername = (search_path + '/' + file[1])
    #print('Checking file: ')
    #print(foldername)
    if os.path.isdir(foldername): # file is a folder
       folder_list.append(foldername)
  return folder_list


def get_file_list(search_path,ext_str="png"):
  count = 0
  file_list = []
  for f in os.listdir(search_path):
    if f.endswith(ext_str):
      #rospy.loginfo('Found image file')
      count = count + 1
      file = (search_path + '/' + f)
      file_list.append(file)
  return file_list,count

def get_file_count(search_path,ext_str="png"):
  count = 0
  for f in os.listdir(search_path):
    if f.endswith(ext_str):
      #rospy.loginfo('Found image file')
      count = count + 1
  return count

def check_make_folder(pathname):
  if not os.path.exists(pathname):
    try:
      os.makedirs(pathname)
      rospy.loginfo("NEPI_ROS: Made folder: " + pathname)
    except rospy.ServiceException as e:
      rospy.loginfo("NEPI_ROS: Failed to make folder: " + pathname + " with exeption" + str(e))
  return os.path.exists(pathname)

def copy_files_from_folder(src_path,dest_path):
  success = True
  files_copied = []
  files_not_copied = []
  if os.path.exists(src_path):
    dest_folders = dest_path.split('/')
    dest_folders.remove('')
    check_folder = ""
    for folder in dest_folders:
      check_folder = check_folder + "/" + folder
      check_make_folder(check_folder)
    if os.path.exists(dest_path):
        files = os.listdir(src_path)
        for file in files:
          srcFile = src_path + "/" + file
          destFile = dest_path + "/" + file
          if not os.path.exists(destFile):
            try:
              shutil.copyfile(srcFile, destFile)
            # If source and destination are same
            except shutil.SameFileError:
                rospy.loginfo("NEPI_ROS: Source and destination represents the same file: " + destFile)
            
            # If destination is a directory.
            except IsADirectoryError:
                rospy.loginfo("NEPI_ROS: Destination is a directory: " + destFile)
            
            # If there is any permission issue
            except PermissionError:
                rospy.loginfo("NEPI_ROS: Permission denied for file copy: " + destFile)
            
            # For other errors
            except:
                rospy.loginfo("NEPI_ROS: Error occurred while copying file: " + destFile)
            if not os.path.exists(destFile):
              files_not_copied.append(file)
              success = False
            else: 
              files_copied.append(file)
              #rospy.loginfo("NEPI_ROS: Copied file: " + file)
    else:
      rospy.loginfo("NEPI_ROS: Did not find and can't make destination folder: " + dest_path)
      success = False
  else:
    rospy.loginfo("NEPI_ROS: Did not find source folder: " + src_path)
    success = False
  return success, files_copied, files_not_copied

def check_if_container():
  first_line = ""
  in_cn = False
  try:
      with open("/proc/mounts", 'r') as f:
          first_line = f.readline().strip()
  except Exception as e:
      print("Failed to open proc/mount file for container check")
  if first_line.find("overlay") != -1:
      in_cn = True
  return in_cn


def save_config_file(config_file_path, namespace):
    #Try and initialize app param values
    try:
      rosparam.dump_params(config_file_path, namespace)
    except Exception as e:
      print("Could not create config file: " + str(e))
  

def load_config_file(config_file_path, defualt_config_dict, namespace):
    #Try and initialize app param values
    if not os.path.exists(config_file_path):
        rospy.logwarn("Config file " + config_file_path + ".yaml not present... will create")
        try:
          rosparam.dump_params(config_file_path, namespace)
        except Exception as e:
          print("Could not create config file: " + str(e))
    if os.path.exists(config_file_path):
      try:
        load_params_from_file(config_file_path, namespace)
      except Exception as e:
        print("Could not get params from config file: " + config_file_path + " " + str(e))
    else:
      print("Could not find config file at: " + config_file_path + " starting with factory settings")



def print_node_params(self):
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

  
  

  
