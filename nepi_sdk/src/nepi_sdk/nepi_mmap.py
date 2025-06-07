#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI memmory mapping utility functions 

import time


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils


from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_mmap"
logger = Logger(log_name = log_name)

####################
# General Functions

def get_mmap_id_from_topic(topic):
  mmap_id = topic.replace('/','')
  return mmap_id


def get_mmap_list():
  # Future work
  return[]

def check_for_mmap(mmap_id):
  mmap_exists = False
  # Future work
  return mmap_exists
  
def lock_mmap(mmap_id):
  success = False
  # Future work
  return success
   
def unlock_mmap(mmap_id):
  success = False
  # Future work
  return success

def check_lock_mmap(mmap_id):
  locked = None   
  # Future work
  return locked

def wait_for_unlock_mmap(mmap_id, timeout = 1):
  locked = True
  unlocked = False
  start_time = nepi_utils.get_time_now()
  timer = 0
  while locked == True and timer < timeout and not nepi_sdk.is_shutdown():
    locked = check_lock_mmap(mmap_id)
    if locked is not None:
      if locked == False:
        unlocked = True
        break
    time.sleep(.01)
  return unlocked

####################
# cv2 Image Functions

IMG_ENCODING_OPTIONS = ['mono8','rgb8','bgr8','32FC1']

NONE_CV2IMG_INFO_DICT = dict()
NONE_CV2IMG_INFO_DICT['mmap_id'] = "nepis2xsomething"
NONE_CV2IMG_INFO_DICT['timestamp'] = time.time_ns()
NONE_CV2IMG_INFO_DICT['img_width'] = 0
NONE_CV2IMG_INFO_DICT['img_height'] = 0
NONE_CV2IMG_INFO_DICT['img_encoding'] = "None"
  
def create_cv2img_mmap(mmap_id, cv2_img,img_encoding = 'bgr8'):
  success = False
  msg = "Failed"
  if img_encoding not in IMG_ENCODING_OPTIONS:
    msg = "Specified encoding not supported: " + img_encoding + " Options are: " + str(IMG_ENCODING_OPTIONS)
    return False, msg
  # Future work
  msg = "Success"
  return success, msg
  
  
def write_cv2img_mmap_data(mmap_id, cv2_img, encoding = 'bgr8', timestamp = time.time_ns()):
  success = False
  msg = ""
  # Future work
  return success, msg
 
  
def get_cv2img_mmap_info(mmap_id,cv2_img):
  success = False
  msg = "Failed"
  info_dict = None
  
  # Get info dict
  try:
    # Add Get Info Dict Code
    info_dict = NONE_CV2IMG_INFO_DICT
    success = True
  except Exception as e:
    success = False
    msg = "Failed to get info dict: " + mmap_id + " " + str(e)
    
  # Get data from info dict
  if success == True:
    try:
      info_dict['mmap_id'] = mmap_id
      info_dict['timestamp'] = time.time_ns()
      info_dict['img_width'] = 0
      info_dict['img_height'] = 0
      info_dict['img_encoding'] = "None"
      msg = Success
    except Exception as e:
      success = False
      msg = "Failed to get data from info dict: " + mmap_id + " " + str(e)
      info_dict = None

  return success, msg, info_dict
  
  
  
def read_cv2img_mmap_data(mmap_id):
  # Init return values
  success = False
  msg = "Failed"
  cv2_img = None
  img_encoding = "None"
  timestamp = 0
  latency_sec = 0 
  
  # Try and get info dict
  [success,msg,info_dict] = get_cv2img_mmap_info(mmap_id)
  if info_dict is not None:
    if 'timestamp' in info_dict:
      timestamp = info_dict['timestamp']
      if 'img_encoding' in info_dict:
        img_encoding = info_dict['img_encoding']
      if 'img_encoding' in info_dict:
        img_encoding = info_dict['img_encoding']      
  # Calc latency
  read_time = time.time_ns()
  latency_sec = (read_time - timestamp)

  return success, msg, cv2_img, img_encoding, timestamp, latency_sec
  
  

  




