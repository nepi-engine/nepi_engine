#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI misc python utility functions


  
import os

import shutil
import time
import subprocess
import yaml

from datetime import datetime, timezone

from nepi_sdk.nepi_ros import logger as Logger
log_name = "nepi_utils"
logger = Logger(log_name = log_name)

#########################
### Time Helper Functions

def get_time():
  return time.time_ns() / 1000000000

def get_datetime_str_now(add_ms = True, add_ns = False):
  date_str=datetime.utcnow().strftime('%Y-%m-%d')
  time_str=datetime.utcnow().strftime('%H%M%S')
  ms_str = ""
  if add_ms == True:
    ms_str ="." + datetime.utcnow().strftime('%f')[:-3]
  ns_str = ""
  if add_ns == True:
    ns_str ="." + datetime.utcnow().strftime('%f')[0:3]
  dt_str = (date_str + "T" + time_str + ms_str + ns_str)
  return dt_str


def convert_time_to_datetime(time_ns):
    """Converts nanoseconds since epoch to a datetime object.

    Args:
        time_ns: Integer representing nanoseconds since the epoch.

    Returns:
        A datetime object representing the given nanoseconds, or None if
        the input is invalid.
    """
    try:
        seconds = time_ns / 1_000_000_000  # Convert nanoseconds to seconds
        return datetime.fromtimestamp(seconds)
    except (ValueError, OSError, OverflowError) as e:
        print(f"Error converting: {e}")
        return None

def convert_date_to_time(year, month, day):
  """Converts a date (year, month, day) to seconds since the epoch."""
  dt_object = datetime.datetime(year, month, day)
  timestamp = time.mktime(dt_object.timetuple())
  return int(timestamp)


#########################
### Network Helper Functions

def ping_ip(ip_address):
      """
      Pings an IP address and returns True if successful, False otherwise.
      """
      try:
          # Use the ping command with a count of 1 to avoid continuous pings
          process = subprocess.run(['ping', '-c', '1', ip_address], capture_output=True, text=True, timeout=5)

          # Check the return code
          if process.returncode == 0:
              return True
          else:
              return False
      except subprocess.TimeoutExpired:
          return False
      except Exception as e:
          logger.log_info("An error occurred: " + str(e))
          return False



 
#########################
### File Helper Functions
  
def clear_end_slash(str_2_check):
    if str_2_check[-1] == '/':
      str_2_check[0:-1]
    return str_2_check
  

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


def get_file_list(search_path,ext_str=""):
  count = 0
  file_list = []
  for f in os.listdir(search_path):
    if f.endswith(ext_str) or ext_str == "":
      #print('Found image file')
      count = count + 1
      file = (search_path + '/' + f)
      file_list.append(file)
  return file_list,count

def get_file_count(search_path,ext_str=""):
  count = 0
  for f in os.listdir(search_path):
    if f.endswith(ext_str) or ext_str == "":
      #print('Found image file')
      count = count + 1
  return count

def check_make_folder(pathname):
  if not os.path.exists(pathname):
    try:
      os.makedirs(pathname)
      logger.log_info("Made folder: " + pathname)
    except rospy.ServiceException as e:
      logger.log_info("Failed to make folder: " + pathname + " with exeption" + str(e))
  return os.path.exists(pathname)

def get_symlink_target(symlink_path):
    try:
        target_path = os.readlink(symlink_path)
        return target_path
    except OSError as e:
        print(f"Error reading symlink: {e}")
        return None

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
                logger.log_info("Source and destination represents the same file: " + destFile)
            
            # If destination is a directory.
            except IsADirectoryError:
                logger.log_info("Destination is a directory: " + destFile)
            
            # If there is any permission issue
            except PermissionError:
                logger.log_info("Permission denied for file copy: " + destFile)
            
            # For other errors
            except:
                logger.log_info("Error occurred while copying file: " + destFile)
            if not os.path.exists(destFile):
              files_not_copied.append(file)
              success = False
            else: 
              files_copied.append(file)
              #logger.log_info("Copied file: " + file)
    else:
      logger.log_info("Did not find and can't make destination folder: " + dest_path)
      success = False
  else:
    logger.log_info("Did not find source folder: " + src_path)
    success = False
  return success, files_copied, files_not_copied

def check_if_container():
  first_line = ""
  in_cn = False
  try:
      with open("/proc/mounts", 'r') as f:
          first_line = f.readline().strip()
  except Exception as e:
      logger.log_warn("Failed to open proc mount file for container check")
  if first_line.find("overlay") != -1:
      in_cn = True
  return in_cn


def read_yaml_2_dict(file_path):
    dict_from_file = dict()
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            logger.log_info("Failed to get dict from file: " + file_path + " " + str(e))
    else:
        logger.log_info("Failed to find dict file: " + file_path)
    return dict_from_file


def write_dict_2_yaml(dict_2_save,file_path,defaultFlowStyle=False,sortKeys=False):
    success = False
    try:
        with open(file_path, "w") as f:
            yaml.dump(dict_2_save, stream=f, default_flow_style=defaultFlowStyle, sort_keys=sortKeys)
        success = True
    except:
        logger.log_info("Failed to write dict: " + str(dict_2_save) + " to file: " + file_path + " " + str(e))
    return success
  
#########################
### List Helper Functions

# Function for checking if val in list
def val_in_list(val2check,list2check):
  in_list = False
  if len(list2check) > 0:
    for list_val in list2check:
      #print(str(val2check) + ' , ' + str(list_val))
      #print(val2check == list_val)
      if val2check == list_val:
        in_list = True
  return in_list