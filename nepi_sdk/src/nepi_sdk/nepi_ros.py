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
import json
import pytz



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



def systemLogWarn(msg, throttle_s = None):
    try:
      caller_msg = str(sys._getframe(1).f_code.co_name).strip() + ": "
    except:
      caller_msg = ''
    msg_str = caller_msg + str(msg)
    success = False
    if throttle_s is not None:
      try:
          rospy.logwarn_throttle(throttle_s, msg_str)
          success = True
      except:
         pass

    if success == False:
        try:
          rospy.logwarn(msg_str)
        except:
           print(msg_str)



#######################
### Setup some global variables

BASE_NAMESPACE = None
DEFUALT_CFG_FOLDERS = ['/mnt/nepi_storage/user_cfg','/mnt/nepi_config/system_cfg']


MSG_TYPES = ['info','warn','error','fatal','debug']
MSG_FILTER_FUNCTIONS = ['_logSelfMsg','<module>']
MSG_FILTER_FUNCTIONS = MSG_FILTER_FUNCTIONS + ['log','logger','log_msg','log_info','log_warn','log_error','log_debug']
MSG_FILTER_FILES = ['log_msg','node_if','node_if','logger','Logger']
MSG_FILTER_CLASSES = ['logger','MsgIF']

MSG_COLORS = {
    "WHITE":  "\033[0m",
    "GREY": "\033[37m",
   
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
    # --- Utility ---
    "RESET": "\033[0m"
}



MSG_DEFAULT_COLORS = {
  'info': 'GREY',
  'warn': 'YELLOW',
  'error': 'RED',
  'fatal': 'RED',
  'debug': 'ORANGE',
  'data_msg': 'MAGENTA',
  'debug_msg': 'GREY',
}


MSG_DEBUG_COLORS = {
  'info': "DARK_PINK",
  'warn': 'DARK_YELLOW',
  'error': 'DARK_RED',
  'fatal': 'DARK_RED',
  'debug': 'DARK_ORANGE',
  'data_msg': 'MAGENTA',
  'debug_msg': 'GREY',
}

DEBUG_ENABLED = False
DEBUG_LEVELS = ['LOW','MED','HIGH','ALL']
DEBUG_LEVEL = 'LOW'
DEBUG_AVAILABLE_OPTIONS = ['STACK','CLASSES','THROTTLE','TIME']
DEBUG_OPTIONS = ['STACK']
DEBUG_NAMESPACE = 'ALL'
#########################
### Shutdown Helper Functions

def is_shutdown():
  return rospy.is_shutdown()

def signal_shutdown(msg):
  rospy.signal_shutdown(msg)

def on_shutdown(shutdown_fuction):
  rospy.on_shutdown(shutdown_fuction)

#########################
### SDK SHARED Helper Functions
def get_time():
  return time.time_ns() / 1000000000


def get_datetime_str_now():
  now = datetime.now()
  # Format with milliseconds
  time_str = now.isoformat(sep=' ', timespec='milliseconds')
  return time_str


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


def get_debug_enabled():
    global DEBUG_ENABLED
    return DEBUG_ENABLED

def get_debug_level():
    global DEBUG_LEVEL
    return DEBUG_LEVEL

def get_debug_options():
    global DEBUG_OPTIONS
    return DEBUG_OPTIONS


def get_debug_namespace():
    global DEBUG_NAMESPACE
    return DEBUG_NAMESPACE


# Function to get list of active topics
def get_node_list():
  node_list=rosnode.get_node_names()
  return node_list

### Function to find a node
def find_node(node_name, exact = False):
  node = ""
  node_list=get_node_list()
  #sdk_logger.log_debug(node_list)
  for node_entry in node_list:
    #sdk_logger.log_debug(node_entry)
    if exact == True:
      if node_entry == node_name:
        node = node_entry
        break
    elif node_entry.find(node_name) != -1:
      node = node_entry
      break
  return node



def get_display_name_from_namespace(namespace = []):
    name = namespace
    namespace = namespace[1:]
    namespace_split = namespace.split('/')
    if len(namespace_split) > 2:
        name = namespace_split[2]
    name = name.replace('_',' ').capitalize()
    return name

def get_display_names_from_namespaces(namespaces = []):
    names = []
    for namespace in namespaces:
        name = get_display_name_from_namespace(namespace)
        names.append(name)
    return names

def clear_end_slash(str_2_check):
    if str_2_check[-1] == '/':
      str_2_check[0:-1]
    return str_2_check

def get_clean_name(name, invalid_chars = r'[<>:"/\\|?*\x00-\x1F]', replacement='_'):
    if name is None:
        name = ''
    elif name != '':
        name = re.sub(invalid_chars, replacement, name)
    return name
#########################
### Namespace Helper Functions

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
      systemLogWarn("get_base_namespace: no ROS node matching 'nepi' with a" + \
        " /<rootname>/<device_id>/ namespace is registered yet. Waiting up to " + \
        str(BASE_NAMESPACE_TIMEOUT_S) + "s. Nodes must run under the device namespace" + \
        " set by the ROOTNAME and DEVICE_ID environment variables.", throttle_s = 1)
      start_time = get_time()
      timer = 0
      while len(nepi_names) < 3 and timer < BASE_NAMESPACE_TIMEOUT_S and not rospy.is_shutdown():
        sleep(.01)
        nepi_node=find_node('nepi')
        nepi_names = nepi_node.split('/')
        timer = get_time() - start_time
        if len(nepi_names) < 3:
          systemLogWarn("get_base_namespace: still waiting for a namespaced" + \
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
        systemLogWarn(msg)
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


#######################
### Log Utility Functions

def get_msg_class():
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

def get_msg_classes():
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
    if len(classes) > 1:
       classes_list = classes[::-1]
    else:
       classes_list = classes
    return classes_list


def get_msg_frames(ignore_files = [], ignore_functions = []):
    # Get Msg ID

    ignore_files = MSG_FILTER_FILES + ignore_files
    self_function_name = inspect.currentframe().f_code.co_name
    ignore_functions = MSG_FILTER_FUNCTIONS + ignore_functions + [self_function_name]

    msg_frames = []
    #########
    frame_dict = dict()
    try:
      stack = inspect.stack()
      frame = stack[1]
      for frame in stack:

          try:
              ###############
              file_path = frame.filename
              #systemLogWarn('Processing file_path: ' + str(file_path))
              if 'nepi' not in file_path and 'nepi_system' not in file_path:
                #systemLogWarn('Ignoring file_path: ' + str(file_path))
                pass
              else:
                  #systemLogWarn('Adding file path: ' + str(file_path))
                  file_name = os.path.basename(frame.filename).split('.')[0]
                  #systemLogWarn('Processing file name: ' + str(file_name))

                  #########
                  if file_name in ignore_files:
                      #systemLogWarn('Ignoring file name: ' + str(file_name))
                      pass
                  else:
                      #systemLogWarn('Adding file name: ' + str(file_name))
                      if file_name not in frame_dict.keys():
                          frame_dict[file_name] = ''

                      ###################
                      function_name = frame.function
                      #systemLogWarn('Processing function: ' + str(function_name))
                      if function_name in ignore_functions:
                        #systemLogWarn('Ignoring function: ' + str(function_name))
                        pass
                      else:
                          #systemLogWarn('Adding function to frame_dict: ' + str(function_name))
                          if function_name not in frame_dict[file_name]:
                              if len(frame_dict[file_name]) == 0:
                                frame_dict[file_name] = frame_dict[file_name] + str(function_name)
                              else:
                                frame_dict[file_name] = frame_dict[file_name] + str("/" + function_name)
          except Exception as e:
              systemLogWarn('Failed to add frame to dict: ' + str(e), throttle_s = 2)
              pass
    
    except Exception as e:
        systemLogWarn('Failed to get frame from stack: ' + str(e), throttle_s = 2)
        return []


    #systemLogWarn('Got Frame Dict from stack:' + str(frame_dict)) #: '  + str(stack))
    if len(list(frame_dict.keys())) == 0:
      systemLogWarn('Got Empty Frame Dict from stack', throttle_s = 2)
      return []
    #########

    for key in frame_dict.keys():
        if len(frame_dict[key]) > 0:
          try:
            entry = str(key) + ':' + str(frame_dict[key])
            msg_frames.append(entry)
          except Exception as e:
            systemLogWarn('Failed to append frame data: ' + str(e), throttle_s = 2)
            pass

    if len(msg_frames) == 0:
        systemLogWarn('Got Zero msg_frames from stack', throttle_s = 2)
        return []

    #########
    rmsg_frames = msg_frames
    if len(msg_frames) >  1:
       rmsg_frames = msg_frames[::-1]
    if len(rmsg_frames) == 0:
        systemLogWarn('Got Zero rmsg_frames from stack', throttle_s = 2)
        return []
    
    return rmsg_frames
      

def remove_msg_numbers(msg):
    # string.digits provides '0123456789'
    clean_msg = msg
    try:
      clean_msg = re.sub(r'\d+', '', msg)
    except Exception as e:
        systemLogWarn('Failed to remove numbers from msg: ' + str(e), throttle_s = 1)
        pass
    return clean_msg

def round_msg_floats(msg , round_f = 3):
    """
    Keeps only a specified number of digits after any '.' 
    for numbers in a multiline string.
    """
    clean_msg = msg
    try:
      # Pattern explanation:
      # (\.\d{0,precision}) -> group 1 matches the dot and up to 'precision' digits
      # \d*                 -> matches and discards any remaining excess digits
      pattern = rf"(\.\d{{0,{round_f}}})\d*"
      # re.sub replaces the whole matched decimal sequence with just group 1
      clean_msg = re.sub(pattern, r"\1", msg)
    except:
       pass
    return clean_msg



def get_msg_list_from_data(data, round_f = -1):
    msg_list = []
    if isinstance(data, dict):
      msg_list = [f"{key}: {value}" for key, value in data.items()]
    elif isinstance(data, list):
      msg_list =  [str(item) for item in data]
    else:
      msg_list = [str(data)]

    if round_f >= 0:
      for i, msg in enumerate(msg_list):
        msg_list[i] = round_msg_floats(msg, round_f)
    return msg_list



########################################################
def log_msg(msg, level = "info", throttle_s = None, msg_prefix = None, data_msg_list = [], debug_dict = dict(), color = None):
    global DEBUG_ENABLED
  
    debug_enabled = DEBUG_ENABLED
    if level == 'debug' and  debug_enabled == False:
       return None

       
    msg_str = None
    if msg is not None and not is_shutdown():
      

      ####################################
      # Msg Str
      ###########
    
      if color is not None:
        if color not in MSG_COLORS.keys():
           color = None
      if color is None:
         color = MSG_DEFAULT_COLORS.get(level,'WHITE')
      log_color = MSG_COLORS[color]

      if msg_prefix is not None:
        msg_str = log_color + str(msg_prefix) + MSG_COLORS['RESET']  + str(msg)
      else:
        msg_str = log_color +  str(msg) + MSG_COLORS['RESET'] 

      if data_msg_list is not None:
        data_color = MSG_DEFAULT_COLORS['data_msg']
        if isinstance(data_msg_list, list) == False:
                data_msg_list = [data_msg_list]
        for data_msg in data_msg_list:
            data_msg = str(data_msg).strip()
            if data_msg != '':
              dmsg_str = MSG_COLORS[data_color] + data_msg  + MSG_COLORS['RESET']
              msg_str = msg_str + '\n' + dmsg_str


      if debug_dict is not None:
        debug_color = MSG_DEFAULT_COLORS['debug_msg']
        for key in debug_dict.keys():
            debug_msg = str(debug_dict[key]).strip()
            if debug_msg != '':
              dmsg_str = MSG_COLORS[debug_color] + '---' + str(key) + ": " + debug_msg  + MSG_COLORS['RESET']
              msg_str = msg_str + '\n' + dmsg_str


      if level == 'warn':
        if throttle_s is None:
          rospy.logwarn(msg_str)
        else:
          rospy.logwarn_throttle(throttle_s,msg_str)
      elif level == 'error':
        if throttle_s is None:
          rospy.logerr(msg_str)
        else:
          rospy.logerr_throttle(throttle_s,msg_str)
      elif level == 'fatal':
        if throttle_s is None:
          rospy.logfatal(msg_str)
        else:
          rospy.logfatal_throttle(throttle_s,msg_str)
      else:
        if throttle_s is None:
          rospy.loginfo(msg_str)
        else:
          rospy.loginfo_throttle(throttle_s,msg_str)
    return msg_str







########################################################
# LOGGER CLASS
########################################################


class logger:

    
    update_delay = 60

    ln_str = ""

    debug_enabled = copy.deepcopy(DEBUG_ENABLED)
    debug_level = copy.deepcopy(DEBUG_LEVEL)
    debug_options = copy.deepcopy(DEBUG_OPTIONS)
    debug_namespace = copy.deepcopy(DEBUG_NAMESPACE)
    is_debug = False
    throttle_dict = dict()
    throttle_busy = False

    status_sub = None

    msg_class = None
    msg_pub = None
    msg_pub_sys = None

    log_name = None
    namespace = None

    colors_dict = copy.deepcopy(MSG_DEFAULT_COLORS)

    debug_value_dict =  {
      "LOW":  1,
      "MED":  2,
      "HIGH":  3,
      "ALL": 4
    }

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

    def log(self, msg, data = [], debug_data = [],
                  level = "None", color = None, msg_prefix = None, 
                  throttle_s = None, throttle_n = False, throttle_r = 2, 
                  round_f = 3):

        #############
        # Run Msg Checks
        #############

        # Check Msg
        if msg is None:
            return
        try:
            str_msg = str(msg)
        except:
            str_msg = None
        if str_msg is None:
            return

        # Check Msg Level
        if str(level) not in MSG_TYPES:
            level = 'info'

        # Check Msg Debug
        debug_enabled = copy.deepcopy(self.debug_enabled)
        debug_level = copy.deepcopy(self.debug_level)
        debug_options = copy.deepcopy(self.debug_options)
        is_debug = copy.deepcopy(self.is_debug)
        if (debug_enabled == False and level == 'debug'):
            return

        debug_value = 0
        if debug_enabled == True:
          debug_value = self.debug_value_dict.get(debug_level,1)

        
          if level == 'debug' and debug_value < 2: # Only show debug for Med or Higher
            return
          elif level == 'info' and debug_value != 4: # Only show Standard Debug Messages for ALL
            debug_value = 0


        
        #############
        # Process Msg Str
        #############


        #################
        # Round Floats if Needed
        if round_f > 0:
          try:
            str_msg  = round_msg_floats(str_msg, round_f)
          except Exception as e:
              systemLogWarn('Failed to round floats in msg: ' + str(e), throttle_s = 1)
              pass

        if str_msg is None:
            str_msg = str(msg)
            systemLogWarn('Failed to round floats in msg, using org:', throttle_s = 1)


        #################
        # Get MSG Stack 
        #################


        #############
        msg_frames = get_msg_frames(self.ignore_files,self.ignore_functions)
        if msg_frames is None:
            msg_frames = ['nepi']
            systemLogWarn('Got None msg_frames from get_msg_frames', throttle_s = 1)
        if len(msg_frames) == 0:
            msg_frames = ['nepi']
            systemLogWarn('Got Zero msg_frames from get_msg_frames', throttle_s = 1)

        #############
        msg_stack = ''
        if len(msg_frames) > 0:
            for i, entry in enumerate(msg_frames):
                if i != 0:
                    msg_stack = msg_stack + " : "
                msg_stack = msg_stack + entry

        if msg_stack.strip() == '':
            systemLogWarn('Failed to get msg frame', throttle_s = 1)


        #################
        # Get MSG Throttle KEY    
        #################

        ############
        key_msg = None
        if throttle_n == False:
          key_msg =  remove_msg_numbers(str_msg)
        elif throttle_r >= 0:
          key_msg = round_msg_floats(str_msg, throttle_r)

        if key_msg is None:
            key_msg = copy.deepcopy(str_msg)
            systemLogWarn('Got None key Message', throttle_s = 1)

        if key_msg.strip() == '':
            key_msg = str_msg
            #systemLogWarn('Got Empty Key Message', throttle_s = 1)
            return
            
        key_msg = key_msg.replace(' ','').replace(',','').replace('[]','').replace('[[','').replace(']]','').replace(':','')

        #################
        # Process Throttle
        #################

        #################################
        if throttle_s is None:
            throttle_s = 0
        if throttle_s > 0.1:
            msg_key = msg_stack + ":" + key_msg
            try:
                throttle_dict = copy.deepcopy(self.throttle_dict)
                if throttle_dict is None:
                    throttle_dict = dict()
                
                # print('Got Msg Throttle Key: ' + str(msg_key))
                throttle_update = None
                if msg_key is not None:
                    # systemLogWarn('Got Msg Throttle Key: ' + str(msg_key), throttle_s = 1)
                    cur_time = get_time()
                    if msg_key not in throttle_dict.keys():
                        # systemLogWarn('New Msg Throttle Key: ' + str(msg_key), throttle_s = 1)
                        throttle_update = [throttle_s,cur_time]
                    else:

                        if throttle_s == -1:
                            str_msg = None
                        else:
                          try:
                              last_time = throttle_dict[msg_key][1]
                          except Exception as e:
                              #systemLogWarn('Failed to get throttle data for throttle key: ' + str(msg_key) + " : " + str(e), throttle_s = 1)
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



        #############
        # Process Msg Data
        #############

        #################
        # Add passed data entries
        data_msg_list = []
        if data is not None:
          if isinstance(data, list) == False:
                data = [data]
          for entry in data:
              try:
                  data_str_list = get_msg_list_from_data(entry, round_f)
                  data_msg_list = data_msg_list + data_str_list
              except Exception as e:
                  systemLogWarn('Failed to process data msg list entry: ' + str(entry) + ' : ' + str(e), throttle_s = 1)
                  pass
            

        #################
        # Process Debug
        #################

        debug_dict = dict()
        ##########
        # Add Debug Prefix
        if debug_enabled == True:

          #################
          # Add passed debug entries
          debug_msg_list = []
          if debug_data is not None:
            if isinstance(debug_data, list) == False:
                  debug_data = [debug_data]
            for entry in debug_data:
                try:
                    debug_str_list = get_msg_list_from_data(entry, round_f)
                    debug_msg_list = debug_msg_list + debug_str_list
                except Exception as e:
                    systemLogWarn('Failed to process debug msg list entry: ' + str(entry) + ' : ' + str(e), throttle_s = 1)
                    pass   
          if len(debug_msg_list) > 0:
              data_msg_list = data_msg_list + debug_msg_list        



          ### LOW #####################
          if debug_value == 1:                  

            #################
            # Add Caller    
            debug_dict['CALLER'] = str(msg_frames[-1])

          ### MED and HIGH #####################
          if debug_value >= 2:

            ############ 
            # Stack Msg
            if 'STACK' in debug_options:
              debug_dict['STACK'] = msg_stack

          if debug_value >= 3:
              ############ 
              # Classes Msg
              if 'CLASSES' in debug_options:
                classes_list = get_msg_classes()
                if len(classes_list):
                  msg_classes = classes_list
                  # if  debug_enabled == False and len(classes_list) > 1:
                  #   msg_classes = [classes_list[0]]
                  #   if len(classes_list) > 1:
                  #       msg_classes.append(str(classes_list[-1]).strip())
                  try:
                    debug_dict['CLASSES'] = str('/'.join(msg_classes)) + ": "
                  except Exception as e:
                    systemLogWarn('Failed to convert classes to log_name_msg: ' + str(msg_classes) + " : " + str(e), throttle_s = 1)

              ############ 
              # Throttle Msg
              if 'THROTTLE' in debug_options:
                  debug_dict['THROTTLE'] = str([throttle_s,throttle_n,throttle_r]) + ' : ' + str(key_msg)

              #################
              # Add Time              
              if 'TIME' in debug_options:
                  debug_dict['TIME'] = get_datetime_str_now()



        ############################
        # Process Msg Prefix
        ##########
        caller_prefix = msg_prefix
        msg_prefix = ''

        ##########
        # Node Name Msg  
        node_name_msg = ''
        try:
          node_name_msg = get_node_name()
          msg_prefix = node_name_msg + ": "
        except:
          msg_prefix = 'nepi: '    

        ##########
        # Caller Prefix Goes After Node Name
        if caller_prefix is not None:
          msg_prefix = msg_prefix + str(caller_prefix)





        ############################
        # Process Msg Color
        ##########


        #################
        if color is not None:
          if color not in MSG_COLORS.keys():
            color = None
        if color is None:
          if is_debug == True:
            color = MSG_DEBUG_COLORS.get(level,'DARK_PINK')
          else:
            color = self.colors_dict.get(level,'GREY')



        ############################
        # Send Msg
        ##########

        log_msg(str_msg, level = level, color = color, msg_prefix = msg_prefix, data_msg_list = data_msg_list, debug_dict = debug_dict)      


        ############################
        # Pub Msg
        ##########

        if str_msg is not None:
          pub_msg = Message()
          pub_msg.prefix = msg_prefix
          pub_msg.level = level
          pub_msg.color = color
          pub_msg.message = str_msg
          pub_msg.data_list = data_msg_list
          debug_list = []
          if debug_dict is not None:
              for key in debug_dict.keys():
                debug_str = str(key) + ': ' + str(debug_dict[key])
                debug_list.append(debug_str)
          pub_msg.debug_list = debug_list

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

    ############################################
    
    def log_info(self, *args, **kwargs):
       self.log(*args, **kwargs, level = 'info')
    
    def log_warn(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
       self.log(*args, **kwargs, level = 'warn')
    
    def log_error(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
       self.log(*args, **kwargs, level = 'error')
    
    def log_fatal(self, *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
       self.log(*args, **kwargs, level = 'fatal')
      

    def log_debug(self,  *args, **kwargs):
        # Dynamically forward all arguments to the superclass method
       self.log(*args, **kwargs, level = 'debug')
    
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


 
    def clean_throttle_dict(self, throttle_dict, purge_time = 60):
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

    def throttleUpdaterCb(self):
        while not is_shutdown():

          try:
              delay = self.update_delay
              self.throttle_busy = True
              self.throttle_dict = self.clean_throttle_dict(self.throttle_dict)
              self.throttle_busy = False
          except:
             delay = 1

          sleep(delay)


    def statusCb(self, msg):
          debug_enabled = msg.sys_debug_enabled
          self.debug_namespace = msg.sys_debug_namespace
          self.debug_level = msg.sys_debug_level
          self.debug_options = msg.sys_debug_options
          check_debug = False
          if (self.debug_namespace == 'ALL' or self.debug_namespace == self.namespace):
            check_debug = debug_enabled
          self.debug_enabled = check_debug

          is_debug = False
          if (self.debug_namespace == self.namespace):
            is_debug = debug_enabled
          self.is_debug = is_debug

          global DEBUG_ENABLED
          DEBUG_ENABLED = self.debug_enabled
          global DEBUG_LEVEL
          DEBUG_LEVEL = self.debug_level
          global DEBUG_OPTIONS
          DEBUG_OPTIONS = self.debug_options
          global DEBUG_NAMESPACE
          DEBUG_NAMESPACE = self.debug_namespace

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
        systemLogWarn("Failed to print Exception MSG: " + error_message + " " + str(e), throttle_s = 5.0)

# Register the crash handler
sys.excepthook = handle_unhandled_exception

########################################################



#######################
### Namespace Utility Functions

def get_full_namespace(namespace):
  if namespace is None:
     return ''
  if namespace.strip() == '':
     return ''
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
        frame = '_'.join(namespace.split(base_namespace)[1].split('/'))
    except:
        frame = '_'.join(namespace.split('/'))
    if len(frame) > 1:
        if frame[0] == '_':
            frame = frame[1:]
    else:
        frame = os.path.join(base_namespace,'_' + str(time.time_ns())[-6:])
    if add_name is not None:
      frame = frame + '_' + add_name
    frame = frame.replace('.','')
    return frame

#######################
### Node Utility Functions

def myargv():
  return rospy.myargv(sys.argv)

def init_node(name,disable_signals=False):
  rospy.init_node(name,disable_signals=disable_signals)

  
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
  sdk_logger.log_debug("Waiting for node with name: " + node_name, throttle_s = 5.0)
  node = ""
  while node == "" and timer < timeout and not rospy.is_shutdown():
    node=find_node(node_name)
    time.sleep(.1)
    timer = get_time() - start_time
  sdk_logger.log_debug("Found node: " + node, throttle_s = 5.0)
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
    sdk_logger.log_debug("" + msg)
  if success: 
    if sub_process.poll() is not None:
      msg = ("" + "Failed to start " + ros_node_name + " via " + " ".join(x for x in device_node_run_cmd) + " (rc =" + str(p.returncode) + ")")
      sdk_logger.log_warn(msg)
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
    sdk_logger.log_debug("Killing node: " + node_name)
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
      sdk_logger.log_debug("Killed node: " + node_name)
    else:
      sdk_logger.log_debug("Failed to node: " + node_name)
    return success
        

def kill_node_namespace(node_namespace):
  success = False
  try:
    subprocess.call(["rosnode","kill", node_namespace])
    success = True
  except Exception as e:
    sdk_logger.log_debug("Failed to kill node_namespace: " + node_namespace + " " + str(e), throttle_s = 5.0)
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
          sdk_logger.log_debug("Error creating parameters from param key: " + str(key) + " " + str(e), throttle_s = 5.0)


def load_params_from_file(file_path, namespace = None, prime_key = None):
    if namespace is None:
      namespace = "~/"
    namespace = get_full_namespace(namespace)
    sdk_logger.log_debug("Will try loading parameters from file: " + file_path)
    params_dict = dict()
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                get_params_dict = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            sdk_logger.log_debug("Failed to get dict from file: " + file_path + " " + str(e))
        if prime_key is not None:
          if prime_key in get_params_dict.keys():
            params_dict = get_params_dict[prime_key]
        else:
          params_dict = get_params_dict
        if params_dict is None:
          params_dict = dict()
          sdk_logger.log_debug("Failed to load params for " + str(namespace) +  " from file: " + str(file_path))
        else:
          for key in params_dict.keys():
            ns = create_namespace(namespace,key)
            value = params_dict[key]
            set_param(ns,value)
          sdk_logger.log_debug("Parameters loaded successfully for " + namespace)
    else:
        sdk_logger.log_debug("Param file not found: " + file_path)
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
      sdk_logger.log_debug("No config file found for " + node_name + " in " + config_folder)
  
  if config_file is not None:
    node_namespace = os.path.join(get_base_namespace(), load_name)
    sdk_logger.log_debug("Loading parameters from " + config_file + " for " + node_namespace)
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
    #sdk_logger.log_debug("Got params to save: " + str(params))
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
      sdk_logger.log_debug("Could not create params file: " + str(e))
  

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
    sdk_logger.log_debug("Failed to get param for: " + param_namespace + " " + str(e), throttle_s = 5.0)
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
    pass #sdk_logger.log_debug("Failed to get param for: " + param_namespace + " " + str(e), throttle_s = 5.0)
    
  return param

def set_param(param_namespace,param_val):
  success = False
  param_namespace = get_full_namespace(param_namespace)
  try:
    rospy.set_param(param_namespace,param_val)
    success = True
  except Exception as e:
    sdk_logger.log_debug("Failed to set param for: " + str(param_namespace) + " "  + str(param_val) + " " + str(e), throttle_s = 5.0)
  return success


def delete_params(params_namespace):
  success = False
  if rospy.has_param(params_namespace):
      rospy.delete_param(params_namespace)
      sdk_logger.log_debug("Deleted all parameters in namespace: " + str(params_namespace))
      success = True
  return success

# Function to wait for a param
def wait_for_param(param_namespace, timeout = 60):
  start_time = get_time()
  timer = 0
  #sdk_logger.log_debug("Waiting for param name: " + param_namespace , throttle_s = 10.0)
  param = 'NOPARAM'
  #sdk_logger.log_debug("Waiting for param check: " + str([param,param_namespace,timer,rospy.is_shutdown()]) ) # , throttle_s = 5.0)
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
    sdk_logger.log_debug("Failed to get param: " + str([param_namespace,timer,rospy.is_shutdown()]) ) # , throttle_s = 5.0)
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
  #sdk_logger.log_debug(service_list)
  for service_entry in service_list:
    #sdk_logger.log_debug(service_entry[0])
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
  sdk_logger.log_debug("Waiting for service name: " + service_name, throttle_s = 5.0)
  found_service = ""
  while found_service == "" and timer < timeout and not rospy.is_shutdown():
    found_service=find_service(service_name)
    time.sleep(.01)
    timer = get_time() - start_time
  sdk_logger.log_debug("Found service: " + found_service, throttle_s = 5.0)
  return found_service

def create_service(service_namespace, srv_msg, srv_callback):
  service = None
  #service_namespace = get_full_namespace(service_namespace)
  try:
    service = rospy.Service(service_namespace, srv_msg, srv_callback)
  except Exception as e:
    sdk_logger.log_debug("Failed to create service: " + str(e) , throttle_s = 5.0)
  return service

def connect_service(service_namespace, service_msg):
  service = None
  #service_namespace = get_full_namespace(service_namespace)
  try:
    service = rospy.ServiceProxy(service_namespace, service_msg)
  except Exception as e:
      sdk_logger.log_debug("Failed to connect to service: " + str(e), throttle_s = 5.0)
  return service

def call_service(service, request, verbose = True):
    response = None
    if service is not None and service != "None":
      try:
          response = service(request)
      except Exception as e:
          if verbose == True:
            sdk_logger.log_debug("Failed to call service: " + str(e), throttle_s = 5.0)
          else:
            pass
    else:
      sdk_logger.log_debug("Cant call None service", throttle_s = 5.0)
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
    sdk_logger.log_debug("Failed to create subscriber: " + str(e), throttle_s = 5.0)
  return sub

def create_publisher(pub_namespace, msg, queue_size = 10, latch = False):
  if queue_size is None:
    queue_size = 1
  pub = None
  #pub_namespace = get_full_namespace(pub_namespace)
  try:
    pub = rospy.Publisher(pub_namespace, msg, queue_size = queue_size, latch = latch)
  except Exception as e:
    sdk_logger.log_debug("Failed to create publisher: " + str(e), throttle_s = 5.0)
  return pub

def publish_pub(publisher, msg):
  success = False
  if publisher is not None and msg is not None:
    try:
      publisher.publish(msg)
      success = True
    except Exception as e:
      sdk_logger.log_debug("Failed to publish message: " + str(e), throttle_s = 5.0)
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
  # sdk_logger.log_debug("get_topics_call functions: " + str(call_functions) )
  try:
    pubs,subs = rostopic.get_topic_list()
    topics_data_list = pubs
  except Exception as e:
    sdk_logger.log_debug("Nepi Sdk rostopic.get_topic_list failed: " + str(e))
    return topics_list,types_list
  

  try:
    for topic_entry in topics_data_list:
      topic_str = topic_entry[0]
      msg_str = topic_entry[1]
      #sdk_logger.log_debug("topics check: " + str(topic_str) + " in " + str(topic_names_list))
      if isinstance(topic_str,str):
            topics_list.append(topic_str)
            types_list.append(msg_str)
  except Exception as e:
    sdk_logger.log_debug("Nepi Sdk get_topics failed: " + str(e))
  return topics_list,types_list


def get_topics_list():
  topics_list = []
  try:
    [topics_list,types_list]=get_topics_data_list()
  except Exception as e:
    sdk_logger.log_debug("Nepi Sdk get_topics failed: " + str(e))
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
        #sdk_logger.log_debug("Found potential topic : " + str(topic_str))
        if exact == True:
          if topic == topic_name:
            find_topic = topic
            break
          else:
            #sdk_logger.log_debug("Topic did not match : " + str(topic_name))
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
    sdk_logger.log_debug("Nepi Sdk find_topics failed: " + str(e))
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
      #sdk_logger.log_debug("msg check: " + str([msg_type, topic_str, msg_str]))
      if msg_str == msg_type or msg_str_base == msg_type:
        find_topics.append(topic_str)
        #sdk_logger.log_debug("msg found: " + str([msg_type, topic_str]))

  except Exception as e:
    sdk_logger.log_debug("Nepi Sdk find_topics_by_msg failed: " + str(e))
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
      #sdk_logger.log_debug("msg check: " + str([msg_type, topic_str, msg_str]))
      for msg_type in msg_type_list:
        if msg_str.find(msg_type) != -1:
          #sdk_logger.log_debug("msg check: " + str([topic_str, msg_str]))
          find_topics.append(topic_str)
          find_msgs.append(msg_str)
          #sdk_logger.log_debug("msgs found: " + str([msg_type, topic_str]))

  except Exception as e:
    sdk_logger.log_debug("Nepi Sdk find_topics_by_msgs failed: " + str(e))
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
    sdk_logger.log_debug("Nepi Sdk find_msg_by_topic failed: " + str(e))
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
    sdk_logger.log_debug("Nepi Sdk find_topics failed: " + str(e))
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
  sdk_logger.log_debug("Waiting for topic with name: " + topic_name, throttle_s = 5.0)
  topic = ""
  while topic == "" and timer < timeout and not rospy.is_shutdown():
    topic=find_topic(topic_name)
    time.sleep(.01)
    timer = get_time() - start_time
  sdk_logger.log_debug("Found topic: " + topic, throttle_s = 5.0)
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
          #sdk_logger.log_debug("Checking subscribers for: " + str(topic_names))
          
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
                        #sdk_logger.log_debug("Found subscriber: " + sub_topic + " for topic " + pub_topic)
                        if sub_topic not in filters:
                            has_subs = True
                            has_subs_dict[topic].append(sub_topic)  # Found subscribers for this 

      except rospy.ROSException as e:
          sdk_logger.log_debug("Error connecting to ROS Master: " + str(e))
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
        sdk_logger.log_debug("Could not read publishers for " + str(topic_name) + ": " + str(e))
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
    sdk_logger.log_debug("Failed to start timer process: " + str(e), throttle_s = 5.0)
  return success





#########################
### Time Helper Functions

def TimeMsgClasss():
  return rospy.Time


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
    #sdk_logger.log_debug("Got timestamp to convert to stamp: " + str(timestamp), throttle_s = 5.0)
    #sdk_logger.log_debug("Got timestamp type to convert to stamp: " + str(type(timestamp)), throttle_s = 5.0)
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
    #sdk_logger.log_debug("Got timestamp to convert to sec: " + str(timestamp), throttle_s = 5.0)
    #sdk_logger.log_debug("Got timestamp type to convert to sec: " + str(type(timestamp)), throttle_s = 5.0)
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
             #sdk_logger.log_debug("Added Dict entry to Msg: " + str([control_name,key,clean_val,clean_type,data_val,data_type]), throttle_s = 10.0)
          else:
             failed_list.append([control_name, key,clean_val,clean_type,data_val,data_type,convert_msg])
  else:
     clean_dict = data_dict
  if len(failed_list) > 0:
     sdk_logger.log_debug("Failed to add Dict entrys to Msg: " + str(failed_list), throttle_s = 10.0)
  msg = None
  try:
    msg = message_converter.convert_dictionary_to_ros_message(msg_instanance,clean_dict)
  except Exception as e:
    sdk_logger.log_debug("Failed to Convert Dict to Msg: " + str(e), throttle_s = 10.0)
  return msg

########################
