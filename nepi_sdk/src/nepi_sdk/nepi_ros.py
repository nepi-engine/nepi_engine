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
import copy
import importlib
import shutil
import time
import traceback
from datetime import datetime
import subprocess
import yaml
import inspect
import string
import re
import logging
import threading



import logging

# Configure your logging settings
logging.basicConfig(
    level=logging.ERROR,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        #logging.FileHandler("crash_log.log"), # Saves the error to a file
        logging.StreamHandler(sys.stdout)     # Prints the error to the console
    ]
)


# import tracemalloc
# tracemalloc.start()

# import warnings
# warnings.filterwarnings("ignore")


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

from nepi_interfaces.msg import Message, MgrSystemStatus






#######################
### Setup some global variables

BASE_NAMESPACE = None
DEFUALT_CFG_FOLDERS = ['/mnt/nepi_storage/user_cfg','/mnt/nepi_config/system_cfg']

DEBUG_ENABLED = False
# ANSI color codes used to make console debug output stand out from info
DEBUG_COLOR = "\033[32m"   # green (ROS convention for debug)
COLOR_RESET = "\033[0m"


MSG_FILTER_FUNCTIONS = ['__init__','_logSelfMsg','<module>']
MSG_FILTER_FUNCTIONS = MSG_FILTER_FUNCTIONS + ['log','logger','log_msg','log_info','log_warn','log_error','log_debug']
MSG_FILTER_FILES = ['log_msg','node_if','connect_node_if','logger','Logger']
MSG_FILTER_CLASSES = ['logger','MsgIF']

MSG_COLORS = {
    "WHITE":  "\033[0m",
   
    # --- Standard Rainbow Colors ---
    "RED": "\033[31m",
    "ORANGE": "\033[38;5;208m",
    "YELLOW": "\033[33m",
    "GREEN": "\033[32m",
    "BLUE": "\033[34m",
    "INDIGO": "\033[38;5;93m",
    "VIOLET": "\033[35m",
    "CYAN": "\033[36m",
    "MAGENTA": "\033[95m",
    "PINK": "\033[91m",

    # --- Dark Rainbow Versions ---
    "DARK_RED": "\033[38;5;88m",
    "DARK_ORANGE": "\033[38;5;130m",
    "DARK_YELLOW": "\033[38;5;142m",
    "DARK_GREEN": "\033[38;5;22m",
    "DARK_BLUE": "\033[38;5;18m",
    "DARK_INDIGO": "\033[38;5;54m",
    "DARK_VIOLET": "\033[38;5;90m",
    "DARK_CYAN": "\033[38;5;30m",
    "DARK_MAGENTA": "\033[38;5;89m",
    "DARK_PINK": "\033[38;5;125m",

    "INFO": "\033[0m",
    "WARN": "\033[33m",
    "ERROR": "\033[31m",
    "FATAL": "\033[31m",
    "DEBUG": "\033[38;5;208m",

    # --- Utility ---
    "RESET": "\033[0m"
}



MSG_DEFAULT_COLORS = {
  'info': MSG_COLORS['INFO'],
  'warn': MSG_COLORS['WARN'],
  'error': MSG_COLORS['ERROR'],
  'fatal': MSG_COLORS['FATAL'],
  'debug': MSG_COLORS['DEBUG']
}

#########################
### Shutdown Helper Functions

def is_shutdown():
  return rospy.is_shutdown()

def signal_shutdown(msg):
  rospy.signal_shutdown(msg)

def on_shutdown(shutdown_fuction):
  rospy.on_shutdown(shutdown_fuction)



#######################
### Log Utility Functions

def get_message_class():
    """Returns a list of classes that called this function in the execution chain."""
    classes = []
    
    # Get the current call stack (excluding this function's frame)
    frames = inspect.stack()[1:]
    
    for frame_info in frames:
        frame = frame_info.frame
        frame_class = None
        # Check if 'self' is in local variables (indicates an instance method call)
        if 'self' in frame.f_locals:
            frame_class = frame.f_locals['self'].__class__
        # Check if 'cls' is in local variables (indicates a class method call)
        elif 'cls' in frame.f_locals and isinstance(frame.f_locals['cls'], type):
            frame_class = frame.f_locals['cls']
        if frame_class is not None:
            try:
              caller_class = str(frame_class.__name__).rsplit('.', 1)[0].strip()
              if caller_class not in MSG_FILTER_CLASSES or len(MSG_FILTER_CLASSES) == 0:
                if caller_class not in classes:
                  classes.append(caller_class)
            except:
              pass

    msg_class = 'UKNOWN'
    if len(classes) > 0:
       msg_class = classes[-1]
    return msg_class

def get_message_classes():
    """Returns a list of classes that called this function in the execution chain."""
    classes = []
    
    # Get the current call stack (excluding this function's frame)
    frames = inspect.stack()[1:]
    
    for frame_info in frames:
        frame = frame_info.frame
        frame_class = None
        # Check if 'self' is in local variables (indicates an instance method call)
        if 'self' in frame.f_locals:
            frame_class = frame.f_locals['self'].__class__
        # Check if 'cls' is in local variables (indicates a class method call)
        elif 'cls' in frame.f_locals and isinstance(frame.f_locals['cls'], type):
            frame_class = frame.f_locals['cls']
        if frame_class is not None:
            try:
              caller_class = str(frame_class.__name__).rsplit('.', 1)[0].strip()
              if caller_class not in MSG_FILTER_CLASSES or len(MSG_FILTER_CLASSES) == 0:
                if caller_class not in classes:
                  classes.append(caller_class)
            except:
              pass
            
    return classes



########################################################


def log_msg_info(msg, throttle_s = None, color = None, caller_str = None):
  global DEBUG_ENABLED

  msg_str = None
  if msg is not None and not is_shutdown():
    msg_str = str(msg)
    log_str = ''
    ##########
    # Add Classes
    classes_list = get_message_classes()
    if len(classes_list):
      msg_classes = classes_list
      if DEBUG_ENABLED == False and len(classes_list) > 1:
        msg_classes = [classes_list[0]]
        if len(classes_list) > 1:
            msg_classes.append(classes_list[-1].strip())
      try:
        log_str = log_str + str('/'.join(msg_classes)) + ": "
      except Exception as e:
        rospy.logwarn_throttle(1,'Failed to convert classes to log_name_msg: ' + str(msg_classes) + " : " + str(e))
    
    ##########
    # Add Caller Function
    if caller_str is None:
      try:
        caller_str = str(sys._getframe(1).f_code.co_name)
      except:
        pass
    if caller_str is not None:
      caller_str = caller_str.strip()
      if 'log_' not in caller_str and caller_str not in MSG_FILTER_FUNCTIONS:
          log_str = log_str + caller_str + ": "

    ##########
    # Add Log Str to Msg Str
    msg_str = log_str + msg_str


    ##########
    # Log Msg
    msg_color = MSG_DEFAULT_COLORS['info']
    if color is not None:
      if color in MSG_COLORS.keys():
          msg_color = MSG_COLORS[color]

    log_str = msg_color + msg_str + MSG_COLORS['RESET']

    if throttle_s is None:
      rospy.loginfo(log_str)
    else:
      rospy.loginfo_throttle(throttle_s,log_str)
  return msg_str

########################################################
def log_msg_warn(msg, throttle_s = None, color = None, caller_str = None):
  global DEBUG_ENABLED

  msg_str = None
  if msg is not None and not is_shutdown():
    msg_str = str(msg)
    log_str = ''
    ##########
    # Add Classes
    classes_list = get_message_classes()
    if len(classes_list):
      msg_classes = classes_list
      if DEBUG_ENABLED == False and len(classes_list) > 1:
        msg_classes = [classes_list[0]]
        if len(classes_list) > 1:
            msg_classes.append(classes_list[-1].strip())
      try:
        log_str = log_str + str('/'.join(msg_classes)) + ": "
      except Exception as e:
        rospy.logwarn_throttle(1,'Failed to convert classes to log_name_msg: ' + str(msg_classes) + " : " + str(e))
    
    ##########
    # Add Caller Function
    if caller_str is None:
      try:
        caller_str = str(sys._getframe(1).f_code.co_name)
      except:
        pass
    if caller_str is not None:
      caller_str = caller_str.strip()
      if 'log_' not in caller_str and caller_str not in MSG_FILTER_FUNCTIONS:
          log_str = log_str + caller_str + ": "

    ##########
    # Add Log Str to Msg Str
    msg_str = log_str + msg_str

    ##########
    # Log Msg
    msg_color = MSG_DEFAULT_COLORS['warn']
    if color is not None:
      if color in MSG_COLORS.keys():
          msg_color = MSG_COLORS[color]

    log_str = msg_color + msg_str + MSG_COLORS['RESET']


    if throttle_s is None:
      rospy.logwarn(log_str)
    else:
      rospy.logwarn_throttle(throttle_s,log_str)
  return msg_str


########################################################
def log_msg_error(msg, throttle_s = None, color = None, caller_str = None):
  global DEBUG_ENABLED

  msg_str = None
  if msg is not None and not is_shutdown():
    msg_str = str(msg)
    log_str = ''
    ##########
    # Add Classes
    classes_list = get_message_classes()
    if len(classes_list):
      msg_classes = classes_list
      if DEBUG_ENABLED == False and len(classes_list) > 1:
        msg_classes = [classes_list[0]]
        if len(classes_list) > 1:
            msg_classes.append(classes_list[-1].strip())
      try:
        log_str = log_str + str('/'.join(msg_classes)) + ": "
      except Exception as e:
        rospy.logwarn_throttle(1,'Failed to convert classes to log_name_msg: ' + str(msg_classes) + " : " + str(e))
    
    ##########
    # Add Caller Function
    if caller_str is None:
      try:
        caller_str = str(sys._getframe(1).f_code.co_name)
      except:
        pass
    if caller_str is not None:
      caller_str = caller_str.strip()
      if 'log_' not in caller_str and caller_str not in MSG_FILTER_FUNCTIONS:
          log_str = log_str + caller_str + ": "

    ##########
    # Add Log Str to Msg Str
    msg_str = log_str + msg_str

    ##########
    # Log Msg
    msg_color = MSG_DEFAULT_COLORS['error']
    if color is not None:
      if color in MSG_COLORS.keys():
          msg_color = MSG_COLORS[color]

    log_str = msg_color + msg_str + MSG_COLORS['RESET']


    if throttle_s is None:
      rospy.logerr(log_str)
    else:
      rospy.logerr_throttle(throttle_s,log_str)
  return msg_str


########################################################
def log_msg_fatal(msg, throttle_s = None, color = None, caller_str = None):
  global DEBUG_ENABLED

  msg_str = None
  if msg is not None and not is_shutdown():
    msg_str = str(msg)
    log_str = ''
    ##########
    # Add Classes
    classes_list = get_message_classes()
    if len(classes_list):
      msg_classes = classes_list
      if DEBUG_ENABLED == False and len(classes_list) > 1:
        msg_classes = [classes_list[0]]
        if len(classes_list) > 1:
            msg_classes.append(classes_list[-1].strip())
      try:
        log_str = log_str + str('/'.join(msg_classes)) + ": "
      except Exception as e:
        rospy.logwarn_throttle(1,'Failed to convert classes to log_name_msg: ' + str(msg_classes) + " : " + str(e))
    
    ##########
    # Add Caller Function
    if caller_str is None:
      try:
        caller_str = str(sys._getframe(1).f_code.co_name)
      except:
        pass
    if caller_str is not None:
      caller_str = caller_str.strip()
      if 'log_' not in caller_str and caller_str not in MSG_FILTER_FUNCTIONS:
          log_str = log_str + caller_str + ": "

    ##########
    # Add Log Str to Msg Str
    msg_str = log_str + msg_str

    ##########
    # Log Msg
    msg_color = MSG_DEFAULT_COLORS['fatal']
    if color is not None:
      if color in MSG_COLORS.keys():
          msg_color = MSG_COLORS[color]

    log_str = msg_color + msg_str + MSG_COLORS['RESET']


    if throttle_s is None:
      rospy.logfatal(log_str)
    else:
      rospy.logfatal_throttle(throttle_s,log_str)
  return msg_str


########################################################
def log_msg_debug(msg, throttle_s = None, color = None, caller_str = None):
  global DEBUG_ENABLED

  msg_str = None
  if msg is not None and DEBUG_ENABLED == True and not is_shutdown():
    msg_str = str(msg)
    log_str = ''
    ##########
    # Add Classes
    classes_list = get_message_classes()
    if len(classes_list):
      msg_classes = classes_list
      if DEBUG_ENABLED == False and len(classes_list) > 1:
        msg_classes = [classes_list[0]]
        if len(classes_list) > 1:
            msg_classes.append(classes_list[-1].strip())
      try:
        log_str = log_str + str('/'.join(msg_classes)) + ": "
      except Exception as e:
        rospy.logwarn_throttle(1,'Failed to convert classes to log_name_msg: ' + str(msg_classes) + " : " + str(e))
    
    ##########
    # Add Caller Function
    if caller_str is None:
      try:
        caller_str = str(sys._getframe(1).f_code.co_name)
      except:
        pass
    if caller_str is not None:
      caller_str = caller_str.strip()
      if 'log_' not in caller_str and caller_str not in MSG_FILTER_FUNCTIONS:
          log_str = log_str + caller_str + ": "

    ##########
    # Add Log Str to Msg Str
    msg_str = log_str + msg_str

    ##########
    # Log Msg
    msg_color = MSG_DEFAULT_COLORS['debug']
    if color is not None:
      if color in MSG_COLORS.keys():
          msg_color = MSG_COLORS[color]

    log_str = msg_color + msg_str + MSG_COLORS['RESET']


    if throttle_s is None:
      rospy.loginfo(log_str)
    else:
      rospy.loginfo_throttle(throttle_s,log_str)
  return msg_str


########################################################
def log_msg(msg, level = "None", throttle_s = None, prefix = "", color = None, caller_str = None):
  #if msg is None:
      #msg = "nepi_sdk log_msg got None msg"
  msg_str = None
  if msg is not None and not is_shutdown():
    msg_str = prefix + str(msg)
    level = level.lower()
    if level == 'debug':
      msg_str = log_msg_debug(msg_str, throttle_s = throttle_s, color = color, caller_str = caller_str)
    elif level == 'warn':
      msg_str = log_msg_warn(msg_str, throttle_s = throttle_s, color = color, caller_str = caller_str)
    elif level == 'error':
      msg_str = log_msg_error(msg_str, throttle_s = throttle_s, color = color, caller_str = caller_str)
    elif level == 'fatal':
      msg_str = log_msg_fatal(msg_str, throttle_s = throttle_s, color = color, caller_str = caller_str)
    else:
      msg_str = log_msg_info(msg_str, throttle_s = throttle_s, color = color, caller_str = caller_str)
  return msg_str


########################################################
# LOGGER CLASS
########################################################

def get_msg_uids(ignore_files = [], ignore_functions = []):
    # Get Msg ID

    frame_dict = dict()
    msg_uids = []

    function_name = inspect.currentframe().f_code.co_name
    ignore_functions.append(function_name)
    try:
      stack = inspect.stack()
      frame = stack[1]
      for i, frame in enumerate(stack):

          try:
              file_path = frame.filename
              if 'nepi' in file_path or 'nepi_system' in file_path:
                  file_name = os.path.basename(frame.filename).split('.')[0]
                  if file_name not in ignore_files:
                      if file_name not in frame_dict.keys():
                          frame_dict[file_name] = ''
                  
                      function_name = frame.function
                      if function_name not in ignore_functions:
                          if function_name not in frame_dict[file_name]:
                              if len(frame_dict[file_name]) == 0:
                                frame_dict[file_name] = frame_dict[file_name] + str(function_name)
                              else:
                                frame_dict[file_name] = frame_dict[file_name] + str("/" + function_name)
          except Exception as e:
              log_msg_debug('Failed to get function: ' + str(e))
              pass

      for i, key in enumerate(frame_dict.keys()):
          if len(frame_dict[key]) > 0:
            try:
              entry = str(key) + '/' + str(frame_dict[key])
              msg_uids.append(entry)
            except:
               pass

    except Exception as e:
        #rospy.logerr('Failed to get frame from stack: '  + str(stack) + " : " + str(e))
        pass
    if msg_uids is None:
       msg_uids = []
       #rospy.logerr('Got None msg_uids from stack: '  + str(stack))
    rmsg_uids = msg_uids[::-1]
    return rmsg_uids
      

def remove_msg_numbers(msg):
    # string.digits provides '0123456789'
    clean_msg = msg
    try:
      clean_msg = re.sub(r'\d+', '', msg)
    except Exception as e:
        rospy.logwarn_throttle(1,'Failed to remove numbers from msg: ' + str(e))
        pass
    return clean_msg

def round_msg_floats(msg , throttle_f = 3, round_f = 3):
    """
    Finds all floating-point numbers in a string and rounds them 
    to the specified number of decimal places.
    """
    clean_msg = msg
    # Regex to match numbers with decimal points (e.g., 12.3456)
    # \d+\.\d+ ensures it looks for digits before and after a dot
    pattern = r'\d+\.\d+'
    
    # Format specifier for the f-string (e.g., ".2f")
    fmt = f".{round_f}f"
    
    # re.sub can accept a lambda function to process each match object
    clean_msg = re.sub(pattern, lambda match: f"{float(match.group()):{fmt}}", msg)
    return clean_msg



def clean_throttle_dict(throttle_dict, purge_time = 60):
    if throttle_dict is None:
        throttle_dict = dict()
    cur_time = get_time()
    purge_list = []
    try:
        for msg_key in throttle_dict.keys():
            try:
                [throttle_s,last_time] = throttle_dict[msg_key]
                if cur_time - last_time > purge_time:
                    purge_list.append(msg_key)
            except:
                purge_list.append(msg_key)
    except:
        throttle_dict = dict()

    for msg_key in purge_list:
        try:
            del throttle_dict[msg_key]
        except:
            pass
    return throttle_dict



def get_debug_namespace(base_namespace = '/nepi/device1'):
    param_namespace = create_namespace(base_namespace,'debug_namespace')
    debug_enabled = get_param(param_namespace,fallback_param = 'NONE')
    return debug_enabled

def get_debug_enabled(base_namespace = '/nepi/device1'):
    param_namespace = create_namespace(base_namespace,'debug_enabled')
    debug_enabled = get_param(param_namespace,fallback_param = False)
    return debug_enabled

def set_debug_log(enabled):
  if enabled == True:
    rospy.logger.setLevel(logging.DEBUG)
  else:
    rospy.logger.setLevel(logging.INFO)


class logger:

    
    update_delay = 60

    ln_str = ""

    debug_enabled = False
    debug_namespace = 'NONE'
    throttle_dict = dict()
    throttle_busy = False

    status_sub = None

    msg_class = None
    msg_pub = None
    msg_pub_sys = None

    log_name = None
    namespace = None

    colors_dict = copy.deepcopy(MSG_DEFAULT_COLORS)

    #######################
    ### IF Initialization
    def __init__(self, namespace = None, ignore_functions = [], ignore_files = [], colors_dict = dict()):
        self.base_namespace = get_base_namespace()

        if namespace is not None:
          namespace = get_full_namespace(namespace)
        self.namespace = namespace

        for key in colors_dict.keys():
          color = str(colors_dict[key])
          if key in self.colors_dict.keys() and color in MSG_COLORS.keys():
             self.colors_dict = color
             
        self.ignore_functions = MSG_FILTER_FUNCTIONS + ignore_functions
        self.ignore_files = MSG_FILTER_FILES + ignore_functions

        sub_updater_thread = threading.Thread(target=self.subUpdaterCb)
        sub_updater_thread.start()

        pub_updater_thread = threading.Thread(target=self.pubUpdaterCb)
        pub_updater_thread.start()

        throttle_updater_thread = threading.Thread(target=self.throttleUpdaterCb)
        throttle_updater_thread.start()



  
        ###############################
        # Init Complete
        ###############################



    ###############################
    # Class Public Methods
    
    def log(self, msg, msg_list = [], level = "None", color = None, caller_str = None, throttle_s = None, throttle_n = False, throttle_f = -1, round_f = 3):
        
        if (self.debug_enabled == False and level == 'debug'):
            return
        if (level == 'debug' and (self.debug_namespace != 'ALL' and self.debug_namespace != self.namespace)):
            return
        elif level == 'None':
          level = 'info'

        if msg is None:
            return
        try:
            msg = str(msg)
        except:
            msg = None
        if msg is None:
            return

        #############
        # Process Msg Str
        #############
        str_msg = str(msg)
        if str_msg is None:
          #rospy.logwarn_throttle(1,'Got None Str from Msg: ' + str(msg))
          #rospy.logwarn_throttle(1,'Got None Str from Msg: ' + str(msg))
          return


        #################
        # Add msg_list entries
        try:  
            if str_msg != '':
                if msg_list is not None:
                  if isinstance(msg_list, list) == False:
                      msg_list = str(msg_list)
                  if isinstance(msg_list, list) == True:
                      for entry in msg_list:
                          try:
                              str_msg = str_msg + " : " + str(entry)
                          except Exception as e:
                              rospy.logwarn_throttle(1,'Failed to process msg list: ' + str(e))
                              pass
        except Exception as e:
            rospy.logwarn_throttle(1,'Failed to process msg: ' + str(e))
            pass


        if str_msg is None:
            str_msg = str(msg)
            rospy.logwarn_throttle(1,'Failed to add msg_list entries, using orig: ' + str(msg_list))
  

        #################
        # Round Floats if Needed
        if round_f > 0:
          try:
            str_msg  = round_msg_floats(str_msg, round_f)
          except Exception as e:
              rospy.logwarn_throttle(1,'Failed to round floats in msg: ' + str(e))
              pass

        if str_msg is None:
            str_msg = str(msg)
            rospy.logwarn_throttle(1,'Failed to round floats in msg, using orig:')


        #################
        # Recheck Message      
         

        
        #############
        msg_uids = get_msg_uids(self.ignore_files,self.ignore_functions)
        # if msg_uids is None:
        #   msg_uids = []
        #   #rospy.logerr('Got None msg_uids from get_msg_uids: '  + str(msg))
        msg_uid = ''
        if len(msg_uids) > 0:
            for i, entry in enumerate(msg_uids):
                if i != 0:
                    msg_uid = msg_uid + ":"
                msg_uid = msg_uid + entry


        key_msg = None
        if throttle_n == False:
          key_msg =  remove_msg_numbers(str_msg)
        elif throttle_f > 0:
          key_msg = round_msg_floats(str_msg, throttle_f)

        if key_msg is None:
           key_msg = copy.deepcopy(str_msg)

        key_msg = key_msg.replace(' ','').replace(',','').replace('[]','').replace(']]','')
        msg_key = msg_uid + ':' + key_msg


        #################################
        if throttle_s is None:
            # if level != 'debug':
            #     throttle_s = 1
            # else:
            #     throttle_s = -1
            throttle_s = 0
        if throttle_s > 0.1 or throttle_s == -1:
            try:
                throttle_dict = copy.deepcopy(self.throttle_dict)
                if throttle_dict is None:
                    throttle_dict = dict()
                
                # print('Got Msg Throttle Key: ' + str(msg_key))
                throttle_update = None
                if msg_key is not None:
                    # rospy.logwarn_throttle(1,'Got Msg Throttle Key: ' + str(msg_key))
                    cur_time = get_time()
                    if msg_key not in throttle_dict.keys():
                        # rospy.logwarn_throttle(1,'New Msg Throttle Key: ' + str(msg_key))
                        throttle_update = [throttle_s,cur_time]
                    else:

                        if throttle_s == -1:
                            str_msg = None
                        else:
                          try:
                              last_time = throttle_dict[msg_key][1]
                          except Exception as e:
                              #rospy.logwarn_throttle(1,'Failed to get throttle data for throttle key: ' + str(msg_key) + " : " + str(e))
                              last_time = 0
                          if (cur_time-last_time) > throttle_s:
                              throttle_update = [throttle_s,cur_time]
                          else:
                              str_msg = None
            
                if throttle_update is not None:
                    throttle_dict[msg_key] = throttle_update
                self.throttle_dict = throttle_dict
            except:
              pass
      
        if str_msg is None:
            return




        if caller_str is None:
          try:
            caller_str = str(sys._getframe(1).f_code.co_name).strip()
          except:
            pass

        if self.debug_enabled == True:
            if (self.debug_namespace == 'ALL' or self.debug_namespace == self.namespace):
              str_msg = str_msg +  ' : ' + msg_uid
              str_msg = str_msg +  ' : throttle_s = ' + str(throttle_s)   
              namespace = os.path.basename(str(self.namespace))
              str_msg = str_msg + " : " + str(namespace)  


        msg_color = None
        if color is not None:
          if color in MSG_COLORS.keys():
             msg_color = color
        
        if msg_color is None:
           msg_color = level.upper()

        pub_str = log_msg(str_msg, level = level, color = msg_color, caller_str = caller_str)      

        if pub_str is not None:
          pub_msg = Message()
          pub_msg.level = level
          pub_msg.color = msg_color
          pub_msg.message = pub_str

          if self.msg_pub_sys is not None:
            try:
              self.msg_pub_sys.publish(pub_msg)
            except:
              pass

          if self.msg_pub is not None:
            try:
              self.msg_pub.publish(pub_msg)
            except:
              pass
    
    def log_info(self, msg, msg_list = [], color = None, caller_str = None, throttle_s = None, throttle_n = True, throttle_f = 3, round_f = 3):
        if caller_str is None:
          try:
            caller_str = str(sys._getframe(1).f_code.co_name).strip()
          except:
            caller_str = None
        self.log(msg, msg_list = msg_list, level = 'info', color = color, caller_str = caller_str, throttle_s = throttle_s, throttle_n = throttle_n, throttle_f = throttle_f, round_f = round_f)
    
    def log_warn(self, msg, msg_list = [], color = None, caller_str = None, throttle_s = None, throttle_n = True, throttle_f = 3, round_f = 3):
        if caller_str is None:
          try:
            caller_str = str(sys._getframe(1).f_code.co_name).strip()
          except:
            caller_str = None
        self.log(msg, msg_list = msg_list, level = 'warn', color = color, caller_str = caller_str, throttle_s = throttle_s, throttle_n = throttle_n, throttle_f = throttle_f, round_f = round_f)
    
    def log_error(self, msg, msg_list = [], color = None, caller_str = None, throttle_s = None, throttle_n = True, throttle_f = 3, round_f = 3):
        if caller_str is None:
          try:
            caller_str = str(sys._getframe(1).f_code.co_name).strip()
          except:
            caller_str = None
        self.log(msg, msg_list = msg_list, level = 'error', color = color, caller_str = caller_str, throttle_s = throttle_s, throttle_n = throttle_n, throttle_f = throttle_f, round_f = round_f)
    
    def log_fatal(self, msg, msg_list = [], color = None, caller_str = None, throttle_s = None, throttle_n = True, throttle_f = 3, round_f = 3):
        if caller_str is None:
          try:
            caller_str = str(sys._getframe(1).f_code.co_name).strip()
          except:
            caller_str = None
        self.log(msg, msg_list = msg_list, level = 'fatal', color = color, caller_str = caller_str, throttle_s = throttle_s, throttle_n = throttle_n, throttle_f = throttle_f, round_f = round_f)

    def log_debug(self, msg, msg_list = [], color = None, caller_str = None, throttle_s = None, throttle_n = True, throttle_f = 3, round_f = 3):
        if caller_str is None:
          try:
            caller_str = str(sys._getframe(1).f_code.co_name).strip()
          except:
            caller_str = None
        self.log(msg, msg_list = msg_list, level = 'debug', color = color, caller_str = caller_str, throttle_s = throttle_s, throttle_n = throttle_n, throttle_f = throttle_f, round_f = round_f)
    
    ###############################
    # Class Private Methods
    ###############################

    def subUpdaterCb(self):
      while self.status_sub is None and not is_shutdown():

        try:
          self.status_sub = create_subscriber(self.base_namespace + "/status", MgrSystemStatus, self.statusCb, queue_size=1)
        except:
          pass

        sleep(5)


    def pubUpdaterCb(self):
      while (self.msg_pub is None or self.msg_pub_sys is None) and not is_shutdown():

        if self.msg_pub_sys is None:
          try:
            self.msg_pub_sys = create_publisher(self.base_namespace + "/messages", Message, queue_size=1)
          except:
            pass

        if self.namespace is None:
          try:
              namespace = str(get_node_namespace())
              if namespace != None and 'unnamed' not in namespace:
                 self.namespace = namespace
          except:
              pass
        


        if self.msg_pub is None and self.namespace is not None:
          try:
            self.msg_pub = create_publisher(self.namespace + "/messages", Message, queue_size=1)
          except:
            pass

        sleep(5)
        

    def throttleUpdaterCb(self):
        while not is_shutdown():

          try:
              delay = self.update_delay
              self.throttle_busy = True
              self.throttle_dict = clean_throttle_dict(self.throttle_dict)
              self.throttle_busy = False
          except:
             delay = 1

          sleep(delay)


    def statusCb(self, msg):
          self.debug_namespace = msg.sys_debug_namespace
          self.debug_enabled = msg.sys_debug_enabled
          global DEBUG_ENABLED
          DEBUG_ENABLED = self.debug_enabled


########################################################



#######################
### Namespace Utility Functions

def get_full_namespace(namespace):
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
  if base_namespace not in namespace:
     namespace = os.path.join(base_namespace,namespace)
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

def myargv():
  return rospy.myargv(sys.argv)

def init_node(name,disable_signals=False):
  rospy.init_node(name,disable_signals=disable_signals)

  
# Maximum seconds get_base_namespace() waits for a namespaced 'nepi' node to
# register before giving up. Long enough that a slow but healthy startup still
# resolves, short enough that a misnamespaced node fails visibly rather than
# looking hung.
BASE_NAMESPACE_TIMEOUT_S = 10.0

def get_base_namespace():
  """Returns the NEPI device base namespace, e.g. '/nepi/device'.

  Derives the namespace from the first registered ROS node whose name contains
  'nepi', taking its first two namespace segments. The result is cached in the
  BASE_NAMESPACE module global, so only the first call does any work.

  If no such node is registered yet, waits up to BASE_NAMESPACE_TIMEOUT_S
  seconds for one to appear, logging what it is waiting for. This call is made
  very early in node startup (MsgIF.__init__ reaches it before any logging is
  set up), so a silent wait here presents as a node that hangs with no output.

  Returns:
      str: The device base namespace, with no trailing slash.

  Raises:
      RuntimeError: If no namespaced 'nepi' node appears within
          BASE_NAMESPACE_TIMEOUT_S seconds, or if ROS shuts down while waiting.
          Raised rather than returning None because no caller in the tree checks
          the return value -- a None would surface later as a TypeError inside an
          unrelated os.path.join, or launch a node into the namespace 'None'.
  """
  global BASE_NAMESPACE
  if BASE_NAMESPACE is None:
    nepi_node=find_node('nepi')
    nepi_names = nepi_node.split('/')
    if len(nepi_names) < 3:
      # Slow path. Log on entry, not just on expiry: this runs before the node
      # has emitted anything, so silence here is indistinguishable from a hang.
      log_msg_debug("get_base_namespace: no ROS node matching 'nepi' with a" + \
        " /<rootname>/<device_id>/ namespace is registered yet. Waiting up to " + \
        str(BASE_NAMESPACE_TIMEOUT_S) + "s. Nodes must run under the device namespace" + \
        " set by the ROOTNAME and DEVICE_ID environment variables.")
      start_time = get_time()
      timer = 0
      while len(nepi_names) < 3 and timer < BASE_NAMESPACE_TIMEOUT_S and not rospy.is_shutdown():
        sleep(.01)
        nepi_node=find_node('nepi')
        nepi_names = nepi_node.split('/')
        timer = get_time() - start_time
        if len(nepi_names) < 3:
          log_msg_debug("get_base_namespace: still waiting for a namespaced" + \
            " 'nepi' node ({:.1f}s".format(timer) + " elapsed of " + \
            str(BASE_NAMESPACE_TIMEOUT_S) + "s). Closest match so far: '" + \
            str(nepi_node) + "'", throttle_s = 2.0)
      if len(nepi_names) < 3:
        if rospy.is_shutdown():
          msg = ("get_base_namespace: ROS shutdown requested after" + \
            " {:.1f}s".format(timer) + " while waiting for a namespaced 'nepi' node.")
        else:
          msg = ("get_base_namespace: gave up after {:.1f}s".format(timer) + \
            " waiting for a ROS node matching 'nepi' under a /<rootname>/<device_id>/" + \
            " namespace. Closest match found: '" + str(nepi_node) + "'. Run this node" + \
            " under a device namespace, for example" + \
            " 'rosrun <pkg> <node> __ns:=/nepi/device', or start it from nepi_base.launch.")
        log_msg_error(msg)
        raise RuntimeError(msg)
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
def wait_for_node(node_name, timeout = 60):
  start_time = get_time()
  timer = 0
  log_msg_debug("Waiting for node with name: " + node_name, throttle_s = 5.0)
  node = ""
  while node == "" and timer < timeout and not rospy.is_shutdown():
    node=find_node(node_name)
    time.sleep(.1)
    timer = get_time() - start_time
  log_msg_debug("Found node: " + node, throttle_s = 5.0)
  return node


def launch_node(pkg_name, file_name, ros_node_name, namespace = None, device_path = None):
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
    msg = ("" + "Launching node " + ros_node_name + " via " + " ".join(x for x in device_node_run_cmd))
    sub_process = subprocess.Popen(device_node_run_cmd)
    success = True
  except Exception as e:
    msg = str("" + "Failed to launch node %s with Exception: %s", ros_node_name, str(e))
    log_msg_debug("" + msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("" + "Failed to start " + ros_node_name + " via " + " ".join(x for x in device_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
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


def kill_node(node_name, sub_process = None):
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
      success = kill_node_process(node_name, sub_process)
  return success


def kill_node_process(node_name, sub_process):
    log_msg_debug("Killing node: " + node_name)
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
      log_msg_debug("Killed node: " + node_name)
    else:
      log_msg_debug("failed to node: " + node_name)
    return success
        

def kill_node_namespace(node_namespace):
  success = False
  try:
    subprocess.call(["rosnode","kill", node_namespace])
    success = True
  except Exception as e:
    log_msg_debug("Failed to kill node_namespace: " + node_namespace + " " + str(e), throttle_s = 5.0)
  return success

def spin():
  rospy.spin()
  
#########################
### Param Utility Functions

EXAMPLE_PARAMS_DICT = {'threshold': 0.3,'max_rate': 5}

def set_params_from_dict(params_dict, namespace):
    for key in params_dict:
      val = params_dict[key]
      namespace = create_namespace(namespace,key)
      try:
          rospy.set_param(namespace,val)
      except Exception as e:
          log_msg_debug("Error creating parameters from param key: " + str(key) + " " + str(e), throttle_s = 5.0)


def load_params_from_file(file_path, namespace = None, prime_key = None):
    if namespace is None:
      namespace = "~/"
    namespace = get_full_namespace(namespace)
    log_msg_debug("Will try loading parameters from file: " + file_path)
    params_dict = dict()
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                get_params_dict = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            log_msg_debug("Failed to get dict from file: " + file_path + " " + str(e))
        if prime_key is not None:
          if prime_key in get_params_dict.keys():
            params_dict = get_params_dict[prime_key]
        else:
          params_dict = get_params_dict
        if params_dict is None:
          params_dict = dict()
          log_msg_debug("Failed to load params for " + str(namespace) +  " from file: " + str(file_path))
        else:
          for key in params_dict.keys():
            ns = create_namespace(namespace,key)
            value = params_dict[key]
            set_param(ns,value)
          log_msg_debug("Parameters loaded successfully for " + namespace)
    else:
        log_msg_debug("Param file not found: " + file_path)
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
    node_namespace = os.path.join(get_base_namespace(), load_name)
    log_msg_debug("Loading parameters from " + config_file + " for " + node_namespace)
    rosparam_load_cmd = ['rosparam', 'load', config_file, node_namespace]
    try:
      subprocess.run(rosparam_load_cmd)
      success = False
    except:
      logger.log_warn("Failed to load config_file: " + config_file)
  return success


def save_params_to_file(file_path, namespace, save_all = False):
    namespace = get_full_namespace(namespace)
    params_dict = dict()
    params = get_params(namespace,dict())
    #log_msg_debug("Got params to save: " + str(params))
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
      log_msg_debug("Could not create params file: " + str(e))
  

def has_param(namespace,param_name = None):
  if param_name is not None:
    namespace = create_namespace(namespace,param_name)
  namespace = get_full_namespace(namespace)
  return rospy.has_param(namespace)



def get_params(param_namespace,fallback_param = dict()):
  params = None
  param_namespace = get_full_namespace(param_namespace)
  try:
    if fallback_param is None:
      params = rospy.get_param(param_namespace)
    else:
      params = rospy.get_param(param_namespace,fallback_param)
  except Exception as e:
    log_msg_debug("Failed to get param for: " + param_namespace + " " + str(e), throttle_s = 5.0)
  return params


def get_param(param_namespace,fallback_param = None):
  param = None
  param_namespace = get_full_namespace(param_namespace)
  try:
    if fallback_param is None:
      param = rospy.get_param(param_namespace)
    else:
      param = rospy.get_param(param_namespace,fallback_param)
  except Exception as e:
    pass #log_msg_debug("Failed to get param for: " + param_namespace + " " + str(e), throttle_s = 5.0)
    
  return param

def set_param(param_namespace,param_val):
  success = False
  param_namespace = get_full_namespace(param_namespace)
  try:
    rospy.set_param(param_namespace,param_val)
    success = True
  except Exception as e:
    log_msg_debug("Failed to set param for: " + str(param_namespace) + " "  + str(param_val) + " " + str(e), throttle_s = 5.0)
  return success


def delete_params(params_namespace):
  success = False
  if rospy.has_param(params_namespace):
      rospy.delete_param(params_namespace)
      rospy.loginfo("Deleted all parameters in namespace: " + str(params_namespace))  
      success = True
  return success

# Function to wait for a param
def wait_for_param(param_namespace, timeout = 60):
  start_time = get_time()
  timer = 0
  #log_msg_debug("Waiting for param name: " + param_namespace , throttle_s = 10.0)
  param = 'NOPARAM'
  #log_msg_debug("Waiting for param check: " + str([param,param_namespace,timer,rospy.is_shutdown()]) ) # , throttle_s = 5.0)
  param_namespace = get_full_namespace(param_namespace)
  while param == 'NOPARAM' and timer < timeout and not rospy.is_shutdown():
    try:
      param = rospy.get_param(param_namespace,'NOPARAM')
    except:
      param_namespace = get_full_namespace(param_namespace)

    time.sleep(1)
    timer = get_time() - start_time
  if param == 'NOPARAM':
    param = None
    log_msg_debug("Failed to get param: " + str([param_namespace,timer,rospy.is_shutdown()]) ) # , throttle_s = 5.0)
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
def find_services_by_name(service_name):
  service_list = []
  try:
    services=get_service_list()
    for service_entry in services:
      if service_name == os.path.basename(service_entry):
        service_list.append(service_entry)
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
def wait_for_service(service_name, timeout = 60):
  start_time = get_time()
  timer = 0
  log_msg_debug("Waiting for service name: " + service_name, throttle_s = 5.0)
  found_service = ""
  while found_service == "" and timer < timeout and not rospy.is_shutdown():
    found_service=find_service(service_name)
    time.sleep(.01)
    timer = get_time() - start_time
  log_msg_debug("Found service: " + found_service, throttle_s = 5.0)
  return found_service

def create_service(service_namespace, srv_msg, srv_callback):
  service = None
  #service_namespace = get_full_namespace(service_namespace)
  try:
    service = rospy.Service(service_namespace, srv_msg, srv_callback)
  except Exception as e:
    log_msg_debug("Failed to create service: " + str(e) , throttle_s = 5.0)
  return service

def connect_service(service_namespace, service_msg):
  service = None
  #service_namespace = get_full_namespace(service_namespace)
  try:
    service = rospy.ServiceProxy(service_namespace, service_msg)
  except Exception as e:
      log_msg_debug("Failed to connect to service: " + str(e), throttle_s = 5.0)
  return service

def call_service(service, request, verbose = True):
    response = None
    if service is not None and service != "None":
      try:
          response = service(request)
      except Exception as e:
          if verbose == True:
            log_msg_debug("Failed to call service: " + str(e), throttle_s = 5.0)
          else:
            pass
    else:
      log_msg_debug("Cant call None service", throttle_s = 5.0)
    return response



#######################
### Topic Utility Functions


def create_subscriber(sub_namespace, msg, callback, queue_size = 10, callback_args = (), buff_size = None):
  # buff_size is rospy's RECEIVE buffer, not the queue. It defaults to 65536,
  # which is smaller than a single color image frame, and a queue_size of 1
  # cannot drop a stale frame that has not been read out of the socket yet --
  # so frames back up and are read late, by a different amount per topic. Any
  # subscriber whose message exceeds 64 KB should pass one; see the stereo cam
  # app, where leaving it unset put the two cameras hundreds of ms apart while
  # rostopic delay showed both stamps ~11 ms old.
  #
  # Passed to rospy only when set, so the default path stays exactly as it was.
  if queue_size is None:
    queue_size = 1
  sub = None
  #sub_namespace = get_full_namespace(sub_namespace)
  kwargs = dict(queue_size = queue_size)
  if buff_size is not None:
    kwargs['buff_size'] = buff_size
  try:
    if len(callback_args) == 0:
        sub = rospy.Subscriber(sub_namespace, msg, callback, **kwargs)
    else:
        sub = rospy.Subscriber(sub_namespace, msg, callback, callback_args = callback_args, **kwargs)
  except Exception as e:
    log_msg_debug("Failed to create subscriber: " + str(e), throttle_s = 5.0)
  return sub

def create_publisher(pub_namespace, msg, queue_size = 10, latch = False):
  if queue_size is None:
    queue_size = 1
  pub = None
  #pub_namespace = get_full_namespace(pub_namespace)
  try:
    pub = rospy.Publisher(pub_namespace, msg, queue_size = queue_size, latch = latch)
  except Exception as e:
    log_msg_debug("Failed to create publisher: " + str(e), throttle_s = 5.0)
  return pub

def publish_pub(publisher, msg):
  success = False
  if publisher is not None and msg is not None:
    try:
      publisher.publish(msg)
      success = True
    except Exception as e:
      log_msg_debug("Failed to publish message: " + str(e), throttle_s = 5.0)
  return success


# Function to get list of active topics
def get_topics_data_list():
  topics_list = []
  types_list = []
  topics_data_list = []
  # stack = inspect.stack()
  # call_functions = []
  # for frame in stack:
  #     call_functions.append(str(f"Function: {frame.function}, File: {frame.filename}, Line: {frame.lineno}"))
  # log_msg_debug("get_topics_call functions: " + str(call_functions) )
  try:
    pubs,subs = rostopic.get_topic_list()
    topics_data_list = pubs
  except Exception as e:
    log_msg_debug("Nepi Sdk rostopic.get_topic_list failed: " + str(e))
    return topics_list,types_list
  

  try:
    for topic_entry in topics_data_list:
      topic_str = topic_entry[0]
      msg_str = topic_entry[1]
      #log_msg_debug("topics check: " + str(topic_str) + " in " + str(topic_names_list))
      if isinstance(topic_str,str):
            topics_list.append(topic_str)
            types_list.append(msg_str)
  except Exception as e:
    log_msg_debug("Nepi Sdk get_topics failed: " + str(e))
  return topics_list,types_list


def get_topics_list():
  topics_list = []
  try:
    [topics_list,types_list]=get_topics_data_list()
  except Exception as e:
    log_msg_debug("Nepi Sdk get_topics failed: " + str(e))
  return topics_list

def get_published_topics():
  return rospy.get_published_topics()

# Function to find a topic
def find_topic(topic_name, exact = False, topics_list = None, types_list = None):
  if types_list is None or topics_list is None:
    [topics_list,types_list]=get_topics_data_list()
  find_topic = ''
  for topic in topics_list:
      if topic.find(topic_name) != -1 and topic.find(topic_name+"_") == -1:
        #log_msg_debug("Found potential topic : " + str(topic_str))
        if exact == True:
          if topic == topic_name:
            find_topic = topic
            break
          else:
            #log_msg_debug("Topic did not match : " + str(topic_name))
            pass
        else:
          find_topic = topic_name
          break
  return find_topic


# Function to find a topic
def find_topics(topic_names_list, topics_list = None, types_list = None):
  if types_list is None or topics_list is None:
    [topics_list,types_list]=get_topics_data_list()
  find_topics = []
  try:
    for topic in topics_list:
        for topic_name in topic_names_list:
          if topic_name == topic and topic_name not in find_topics:
            find_topics.append(topic_name)
  except Exception as e:
    log_msg_debug("Nepi Sdk find_topics failed: " + str(e))
  return find_topics


# Function to find a topic
def find_topics_by_msg(msg_type, topics_list = None, types_list = None):
  if types_list is None or topics_list is None:
    [topics_list,types_list]=get_topics_data_list()
  find_topics = []
  try:
    for i, topic in enumerate(topics_list):
      topic_str = topics_list[i]
      msg_str = types_list[i]
      msg_str_base = os.path.basename(msg_str)
      #log_msg_debug("msg check: " + str([msg_type, topic_str, msg_str]))
      if msg_str == msg_type or msg_str_base == msg_type:
        find_topics.append(topic_str)
        #log_msg_debug("msg found: " + str([msg_type, topic_str]))

  except Exception as e:
    log_msg_debug("Nepi Sdk find_topics_by_msg failed: " + str(e))
  return find_topics

# Function to find a topic
def find_topics_by_msgs(msg_type_list, topics_list = None, types_list = None):
  if types_list is None or topics_list is None:
    [topics_list,types_list]=get_topics_data_list()

  find_topics = []
  find_msgs = []
  try:
    
    for i, topic in enumerate(topics_list):
      topic_str = topics_list[i]
      msg_str = types_list[i]
      #log_msg_debug("msg check: " + str([msg_type, topic_str, msg_str]))
      for msg_type in msg_type_list:
        if msg_str.find(msg_type) != -1:
          #log_msg_debug("msg check: " + str([topic_str, msg_str]))
          find_topics.append(topic_str)
          find_msgs.append(msg_str)
          #log_msg_debug("msgs found: " + str([msg_type, topic_str]))

  except Exception as e:
    log_msg_debug("Nepi Sdk find_topics_by_msgs failed: " + str(e))
  return find_topics,find_msgs

# Function to find a topic
def find_msg_by_topic(topic, topics_list = None, types_list = None):
  if types_list is None or topics_list is None:
    [topics_list,types_list]=get_topics_data_list()
  find_msg = ''
  try:
    for i, topic in enumerate(topics_list):
      topic_str = topics_list[i]
      msg_str = types_list[i]
      if topic_str == topic:
        find_msg = msg_str
  except Exception as e:
    log_msg_debug("Nepi Sdk find_msg_by_topic failed: " + str(e))
  return find_msg

# Function to find a topic
def find_topics_by_name(topic_name, topics_list = None, types_list = None):
  if types_list is None or topics_list is None:
    [topics_list,types_list]=get_topics_data_list()
  find_topics = []
  try:
    for topic in topics_list:
        if topic.index(topic_name) != -1:
          find_topics.append(topic)
  except Exception as e:
    log_msg_debug("Nepi Sdk find_topics failed: " + str(e))
  return find_topics


### Function to check for a topic 
def check_for_topic(topic_name, topics_list = None, types_list = None):

  topic=find_topic(topic_name)
  
  topic_exists = True
  if topic == "":
    topic_exists = False
  return topic_exists

# Function to wait for a topic
def wait_for_topic(topic_name, timeout = 60, topics_list = None, types_list = None):
  start_time = get_time()
  timer = 0
  log_msg_debug("Waiting for topic with name: " + topic_name, throttle_s = 5.0)
  topic = ""
  while topic == "" and timer < timeout and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.01)
    timer = get_time() - start_time
  log_msg_debug("Found topic: " + topic, throttle_s = 5.0)
  return topic

def check_for_subscribers(topic_names,filters=[]):
  [has_subs,has_subs_dict] = find_subscribers(topic_names = topic_names,filters = filters)
  return has_subs


def find_subscribers(topic_names,filters=[], topics_list = None, types_list = None):

    """
    Checks if topics have any active subscribers.
    """
    topic_names = list(topic_names)
    has_subs = False
    has_subs_dict = dict()
    if len(topic_names) > 0:

      try:
          #log_msg_debug("Checking subscribers for: " + str(topic_names))
          
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
                        #log_msg_debug("Found subscriber: " + sub_topic + " for topic " + pub_topic)
                        if sub_topic not in filters:
                            has_subs = True
                            has_subs_dict[topic].append(sub_topic)  # Found subscribers for this 

      except rospy.ROSException as e:
          rospy.logerr(f"Error connecting to ROS Master: {e}")
          return has_subs, has_subs_dict
    return has_subs, has_subs_dict


def find_publishers(topic_name):
    """
    Returns the names of the nodes currently publishing a topic.

    Args:
        topic_name (str): Fully qualified topic name.

    Returns:
        list: Publishing node names. Empty if the topic has no publishers
            or the ROS master could not be queried.
    """
    pub_names = []
    try:
        master = rosgraph.Master('/rospy_info')  # Create a Master proxy
        for pub_topic, node_names in master.getSystemState()[0]:
            if pub_topic == topic_name:
                pub_names = list(node_names)
    except Exception as e:
        log_msg_debug("Could not read publishers for " + str(topic_name) + ": " + str(e))
    return pub_names


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
    log_msg_debug("Failed to start timer process: " + str(e), throttle_s = 5.0)
  return success





#########################
### Time Helper Functions

def TimeMsgClasss():
  return rospy.Time

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
  if hasattr(msg_stamp,'secs'):
    sec_str = str(msg_stamp.secs) + '.' + str(msg_stamp.nsecs)
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



def msg_stamp_from_timestamp(timestamp):
    log_msg_debug("Got timestamp to convert to stamp: " + str(timestamp), throttle_s = 5.0)
    log_msg_debug("Got timestamp type to convert to stamp: " + str(type(timestamp)), throttle_s = 5.0)
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


def sec_from_timestamp(timestamp = None):
    log_msg_debug("Got timestamp to convert to sec: " + str(timestamp), throttle_s = 5.0)
    log_msg_debug("Got timestamp type to convert to sec: " + str(type(timestamp)), throttle_s = 5.0)
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

def convert_dict2msg(msg_instanance, data_dict, data_msg = None, control_name = None):
  if control_name is None:
     control_name = 'control'
  failed_list = []
  if data_msg is not None:
     clean_dict = convert_msg2dict(data_msg)
     for key in clean_dict.keys():
        if key in data_dict.keys():
          clean_val = clean_dict[key]
          clean_type = type(clean_val)
          data_val = data_dict[key]
          convert_msg = ''
          if type(clean_val).__name__ == 'int': 
            try:
              data_val = int(float(data_val))
              convert_msg = 'converted to type' + str(type(data_val))
            except Exception as e:
              convert_msg ='Failed to convert type' +  str(e)
          if type(clean_val).__name__ == 'float': 
            try:
              data_val = float(data_val)
              convert_msg = 'converted to type' + str(type(data_val))
            except Exception as e:
              convert_msg ='Failed to convert type' +  str(e)
          if type(clean_val).__name__ == 'str':  
            try:
              data_val = str(data_val)
              convert_msg = 'converted to type' + str(type(data_val))
            except Exception as e:
              convert_msg ='Failed to convert type' +  str(e)
          data_type = type(data_val)

          if data_type == clean_type:
             clean_dict[key] = data_dict[key]
             #log_msg_debug("Added Dict entry to Msg: " + str([control_name,key,clean_val,clean_type,data_val,data_type]), throttle_s = 10.0)
          else:
             failed_list.append([control_name, key,clean_val,clean_type,data_val,data_type,convert_msg])
  else:
     clean_dict = data_dict
  if len(failed_list) > 0:
     log_msg_debug("Failed to add Dict entrys to Msg: " + str(failed_list), throttle_s = 10.0)
  msg = None
  try:
    msg = message_converter.convert_dictionary_to_ros_message(msg_instanance,clean_dict)
  except Exception as e:
    log_msg_debug("Failed to Convert Dict to Msg: " + str(e), throttle_s = 10.0)
  return msg

def parse_string_list_msg_data(msg_data):
  str_list = []
  if msg_data[0] == "[" and msg_data[-1] == "]" :
    str_list = eval(msg_data)
  return(str_list)




########################################################
# SDK LOGGER INSTANCE
########################################################

sdk_logger = logger()

#######################
### Setup Crash Handler
def handle_unhandled_exception(exc_type, exc_value, exc_traceback):
    """
    Callback function to capture uncaught exceptions.
    """
    # Ignore KeyboardInterrupt (Ctrl+C) so the user can still exit normally
    if not is_shutdown():
      if issubclass(exc_type, KeyboardInterrupt):
          sys.__excepthook__(exc_type, exc_value, exc_traceback)
          return

      # Log the exception with the full traceback details
      logging.error("An uncaught exception occurred!", exc_info=(exc_type, exc_value, exc_traceback))
      
      # Optional: Perform cleanup actions here (e.g., closing database connections)
      error_message = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))

      try:
        sdk_logger.log_error(error_message)
      except Exception as e:
        log_msg_error("Failed to print Exception MSG: " + error_message + " " + str(e), throttle_s = 5.0)

# Register the crash handler
sys.excepthook = handle_unhandled_exception

