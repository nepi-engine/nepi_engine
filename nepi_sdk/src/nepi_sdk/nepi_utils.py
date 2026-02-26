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


# NEPI misc python utility functions


  
import os
import re
import sys
import shutil
import time
import subprocess
import yaml
import csv
import inspect
import numpy as np
import string
import math

import pytz
import datetime

from nepi_sdk import nepi_sdk
log_name = "nepi_utils"
logger = nepi_sdk.logger(log_name = log_name)




#########################
### Time Helper Functions

def get_time():
  return time.time()

def get_datetime_str_now(add_ms = True, add_us = False, timezone = None):
  time_ns = get_time()
  return get_datetime_str_from_timestamp(timestamp = time_ns, add_ms = add_ms, add_us = add_us, timezone = timezone)


def get_datetime_str_from_timestamp(timestamp = None, add_ms = True, add_us = False, add_tz = False, timezone = None):
  if timestamp is None:
      timestamp = get_time()
  time_ns = nepi_sdk.sec_from_timestamp(timestamp)
  #logger.log_warn("  Creating dt string with timestamp: " + str(timestamp) + " time_sec: " + str(time_ns))
  dt = convert_time_to_datetime(time_ns, timezone = timezone)
  #logger.log_warn("  Got dt str: " + dt.strftime('%Y-%m-%d,%H-%M-%S'))
  date_str='D' + dt.strftime('%Y-%m-%d')
  time_str=dt.strftime('%H-%M-%S')
  ms_str = ''
  if add_ms or add_us:
    ms_str = 'p'
    ms_str += dt.strftime('%f')[:-3]
    if add_us == True:
      ms_str += dt.strftime('%f')[3:]
  tz_str = ""
  if add_tz == True:
    try:
        os.environ["TZ"] = timezone
        time.tzset()
        tz_str = 'Tz' + time.strftime('%Z')
    except Exception as e:
        os.environ["TZ"] = 'UTC'
        time.tzset()
        tz_str = 'Tz' + time.strftime('%Z')
        logger.log_warn("Not valid timezone: " + str(e))
  #logger.log_warn("  Creating dt string with timezone: " + str(timezone) + " id: " + str(tz_str))
  dt_str = (date_str + "T" + time_str + ms_str + tz_str)
  return dt_str


def convert_time_to_datetime(time_sec, timezone = None):
    tzo = pytz.timezone('UTC')
    if timezone is not None:
      try:
        tzo = pytz.timezone(timezone)
      except Exception as e:
        logger.log_warn("Timezone not valid timezone: " + str(e))

    try:
        dt =  datetime.datetime.fromtimestamp(time_sec,tzo)
    except Exception as e:
        logger.log_warn("Error converting: " + str(e))
        dt = datetime.datetime.now(tzo)
    return dt

def get_time_from_datetime_str(dt_str):
    if dt_str is not None:
        try:
            if 'Tz' in dt_str:
              [dt_str,tz] = dt_str.split('Tz', maxsplit=1)
              try:
                os.environ["TZ"] = tz
              except:
                os.environ["TZ"] = 'UTC'
            else:
              os.environ["TZ"] = 'UTC'
            time.tzset()
            tstr = time.strftime('%z')
            tsign = int(tstr[0] + '1')
            tho = tsign * int(tstr[1:3]) * 60 * 60
            tmo = tsign * int(tstr[3:5]) * 60
            sec_offset = tho + tmo


            dt_str = dt_str[1:]
            [year,dt_str] = dt_str.split('-', maxsplit=1)
            year = int(year)
            [month,dt_str] = dt_str.split('-', maxsplit=1)
            month = int(month)
            [day,dt_str] = dt_str.split('T', maxsplit=1)
            day = int(day)
            [hour,dt_str] = dt_str.split('-', maxsplit=1)
            hour = int(hour) 
            [minute,dt_str] = dt_str.split('-', maxsplit=1)
            minute = int(minute)

            if 'p' in dt_str:
              [sec,dt_str] = dt_str.split('p', maxsplit=1)
              if len(dt_str) > 0:
                  pad = 6 - len(dt_str)
                  for i in range(pad):
                      dt_str = dt_str + '0'
                  try:
                    msec = int(dt_str)
                  except:
                    msec = 0
              else:
                  msec = 0
            tzo = pytz.timezone('UTC')
            file_time = datetime.datetime(year, month, day, hour, minute, sec, msec, tzo) + sec_offset
        except Exception as e:
            logger.log_warn("Failed to convert date time string: " + str(dt_str_base) )




def convert_date_to_time(year, month, day, timezone = None):
  tzo = pytz.timezone('UTC')
  if timezone is not None:
    try:
      tzo = pytz.timezone(timezone)
    except Exception as e:
      logger.log_warn("Timezone not valid timezone: " + str(e))

  """Converts a date (year, month, day) to seconds since the epoch."""
  dto = datetime.datetime(year, month, day, tzo)
  timestamp = time.mktime(dto.timetuple())
  return int(timestamp)

def get_timezone_description(abbreviation):
    """
    Returns a standard time zone description for a given abbreviation.
    """
    timezone = ""
    for zone in pytz.all_timezones:
        if datetime.datetime.now(pytz.timezone(zone)).strftime("%Z") == abbreviation:
            if zone in standard_timezones_dict.keys():
              timezone = zone
              break
    return timezone

standard_timezones_dict = {'UTC': 0, 'America/New_York': -5, 'America/Chicago': -6, 'America/Denver': -7, 'America/Phoenix': -7, 'America/Los_Angeles': -8,
 'America/Anchorage': -9, 'Pacific/Honolulu': -10, 'Africa/Johannesburg': 2, 'America/Mexico City': -6, 'Africa/Monrousing': 0, 'Asia/Tokyo': 9,
  'America/Jamaica': -5, 'Europe/Rome': 1, 'Asia/Hong Kong': 8, 'Pacific/Guam': 10, 'Europe/Athens': 2, 'Europe/London': 0, 'Europe/Paris': 1,
   'Europe/Madrid': 1, 'Africa/Cairo': 2, 'Europe/Copenhagen': 1, 'Europe/Berlin': 1, 'Europe/Prague': 1, 'America/Vancouver': -8, 'America/Edmonton': -7,
    'America/Toronto': -5, 'America/Montreal': -5, 'America/Sao Paulo': -3, 'Europe/Brussels': 1, 'Australia/Perth': 8, 'Australia/Sydney': 10, 'Asia/Seoul': 9,
     'Africa/Lagos': 1, 'Europe/Warsaw': 1, 'America/Puerto Rico': -4, 'Europe/Moscow': 4, 'Asia/Manila': 8, 'Atlantic/Reykjavik': 0, 'Asia/Jerusalem': 2}
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
CURRENT_FOLDER = os.path.realpath(__file__)


def check_partition_busy_lsof(mount_point):
    try:
        # Run lsof command for the given mount point
        # -F p: output process ID
        # -F n: output file name
        # -F a: output access mode
        command = ["lsof", "-F", "pn", mount_point]
        process = subprocess.run(command, capture_output=True, text=True, check=True)
        output = process.stdout.strip()

        if output:
            #logger.log_info("Processes using mount_point: " + str(mount_point))
            # Parse the output to extract process information
            current_pid = None
            for line in output.splitlines():
                if line.startswith('p'):
                    current_pid = line[1:]
                elif line.startswith('n') and current_pid:
                    file_path = line[1:]
                    #print(f"  PID: {current_pid}, File: {file_path}")
            return True
        else:
            print(f"No processes found using {mount_point}.")
            return False
    except subprocess.CalledProcessError as e:
        #logger.log_warn("Error running lsof: " + str(e))
        # This might happen if lsof is not found or permission issues
        return False


def fix_folder_permissions(folder_path, user, group):
    success = True
    print("setting permissions for folder: " + folder_path + " to " + user + ":"  + group)
    if os.path.exists(folder_path) == True:
        try:
            os.system('chown -R ' + user + ':' + group + ' ' + folder_path) # Use os.system instead of os.chown to have a recursive option
            #os.chown(full_path_subdir, user, group)
            os.system('chmod -R 0775 ' + folder_path)
        except Exception as e:
            success = False
            print("Failed to update folder permissions: " + folder_path + " " + str(e))
    return success

def read_sh_variables(filepath):
    variables = {}
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#'):  # Basic check for variable assignment
                key, value = line.split('=', 1)
                variables[key.strip()] = value.strip().strip('"\'') # Remove quotes_l
    return variables


def clear_end_slash(str_2_check):
    if str_2_check[-1] == '/':
      str_2_check[0:-1]
    return str_2_check

def get_clean_name(name, invalid_chars = r'[<>:"/\\|?*\x00-\x1F]', replacement='_'):
    return re.sub(invalid_chars, replacement, name)
  

def get_folder_list(search_path):
  filelist=os.listdir(search_path + '/')
  folder_list=[]
  #logger.log_warn('')
  #logger.log_warn('Files and Folders in Path:')
  #logger.log_warn(search_path)
  #logger.log_warn(filelist)
  for file in enumerate(filelist):
    foldername = (search_path + '/' + file[1])
    #logger.log_warn('Checking file: ')
    #logger.log_warn(foldername)
    if os.path.isdir(foldername): # file is a folder
       folder_list.append(foldername)
  return folder_list


def get_file_list(search_path,ext_str="",ext_strs=[]):
  count = 0
  file_list = []
  for f in os.listdir(search_path):
    f_path = os.path.join(search_path,f)
    if os.path.isfile(f_path):
        split = os.path.splitext(f)
        if len(split) > 1:
            ext = split[1]
            if len(ext)>1:
                if ext[0] == '.':
                    ext = ext[1:]
                if (ext_str != "" and f.endswith(ext_str)) or (ext in ext_strs) or (ext_str == "" and len(ext_strs) == 0):
                    #logger.log_warn('Found image file')
                    count = count + 1
                    file = (search_path + '/' + f)
                    file_list.append(file)
  file_list = sorted(file_list)
  return file_list, count

def get_file_count(search_path,ext_str="",ext_strs=[]):
  count = 0
  if os.path.exists(search_path):
    for f in os.listdir(search_path):
        f_path = os.path.join(search_path,f)
        if os.path.isfile(f_path):
            split = os.path.splitext(f)
            if len(split) > 1:
                ext = split[1]
                if len(ext)>1:
                    if ext[0] == '.':
                        ext = ext[1:]
                    if (ext_str != "" and f.endswith(ext_str)) or (ext in ext_strs) or (ext_str == "" and len(ext_strs) == 0):
                        #logger.log_warn('Found image file')
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
        logger.log_warn("Error reading symlink: " + str(e))
        return None


def check_for_sudo():
    # Check if the effective user ID is not 0 (root)
    if os.geteuid() != 0:
        print("Error: This script must be run with sudo or as root.")
        sys.exit(1) # Exit with a non-zero status code to indicate an error
    else:
        return True



def get_user_id(folder = CURRENT_FOLDER):
    stat_info = os.stat(folder)
    uid = stat_info.st_uid
    gid = stat_info.st_gid

    user = pwd.getpwuid(uid)[0]
    group = grp.getgrgid(gid)[0]
    #print([self.user, self.group])
    return user,group

def make_folder(folder_path, user = None, group = None):
    success = False
    try:
        os.mkdir(folder_path)
        fix_folder_permissions(folder_path, user = user, group = user)
        success = True
    except Exception as e:
        print("Failed to make folder: " + folder_path + " " + str(e))
    return success

def fix_folder_permissions(folder_path, user = None, group = None):
    success = True
    [fuser,fgroup] = get_user_id(folder_path)
    if user is None:
        user = fuser
    if group is None:
        group = fgroup
    print("setting permissions for folder: " + folder_path + " to " + user + ":"  + group)
    if os.path.exists(folder_path) == True:
        try:
            os.system('chown -R ' + user + ':' + group + ' ' + folder_path) # Use os.system instead of os.chown to have a recursive option
            #os.chown(full_path_subdir, user, group)
            os.system('chmod -R 0775 ' + folder_path)
        except Exception as e:
            success = False
            print("Failed to update folder permissions: " + folder_path + " " + str(e))
    return success


def get_folder_list(folder_path):

  folder_list=[]
  if os.path.exists(folder_path):
    filelist=os.listdir(folder_path + '/')
    #print('')
    #print('Files and Folders in Path:')
    #print(folder_path)
    #print(filelist)
    for i, file in enumerate(filelist):
        #print(file)
        foldername = (folder_path + '/' + file)
        #print('Checking file: ')
        #print(foldername)
        if os.path.isdir(foldername): # file is a folder
            folder_list.append(foldername)
  return folder_list


def get_folder_files(folder_path):
    files_dict = dict()
    if os.path.exists(folder_path) == False:
        print('Get stats folder not found: ' + folder_path)
    else:
        path, dirs, files = next(os.walk(folder_path))
        for file in files:
            f_ext = os.path.splitext(file)[1]
            f_ext = f_ext.replace(".","")
            if f_ext not in files_dict.keys():
                files_dict[f_ext] = [file]
            else:
                files_dict[f_ext].append(file)        
    return files_dict


def open_new_file(file_path):
  print('')
  if os.path.isfile(file_path):
    print('Deleting existing file:')
    print(file_path)
    os.remove(file_path)
  print('Creating new file: ' + file_path)
  fnew = open(file_path, 'w')
  return fnew

def read_list_from_file(file_path):
    lines = []
    with open(file_path) as f:
        lines = [line.rstrip() for line in f] 
    return lines

def write_list_to_file(file_path,data_list):
    success = True
    try:
        with open(file_path, 'w') as file:
            for data in data_list:
                file.write(data + '\n')
    except Exception as e:
        print("Failed to write list to file " + file_path + " " + str(e))
        success = False
    return success



def read_dict_from_file(file_path):
    dict_from_file = None
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            print("Failed to get dict from file: " + file_path + " " + str(e))
    else:
        print("Failed to find dict file: " + file_path)
    return dict_from_file


def write_dict_to_file(dict_2_save,file_path,defaultFlowStyle=False,sortKeys=False):
    success = False
    try:
        with open(file_path, "w") as f:
            yaml.dump(dict_2_save, stream=f, default_flow_style=defaultFlowStyle, sort_keys=sortKeys)
        success = True
    except Exception as e:
        print("Failed to write dict: "  + " to file: " + file_path + " " + str(e))
    return success


def copy_file(file_path, destination_path):
    success = False
    output_path = destination_path.replace(" ","_")
    #print("Checking on file copy: " + file_path + " to: " + output_path)
    if os.path.exists(output_path) == False:
        try:
            shutil.copy(file_path, output_path)
            #print("File: " + file_path + " Copied to: " + output_path)
            success = True
        except FileNotFoundError:
            print("Error file " + file_path + "not found") 
        except Exception as e:
            print("Excepton: " + str(e))
    return success 


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


def write_dict_2_yaml(file_path,dict_2_save, defaultFlowStyle=False,sortKeys=False):
    success = False
    try:
        with open(file_path, "w") as f:
            yaml.dump(dict_2_save, stream=f, default_flow_style=defaultFlowStyle, sort_keys=sortKeys)
        success = True
    except:
        logger.log_info("Failed to write dict: " + str(dict_2_save) + " to file: " + file_path + " " + str(e))
    return success
  
def read_csv_file(file_path):
    """
    Reads a CSV file and returns its content as a list of rows.

    Args:
        file_path (str): The path to the CSV file.

    Returns:
        list: A list of rows, where each row is a list of strings.
              Returns an empty list if an error occurs.
    """
    data = []
    try:
        with open(file_path, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                data.append(row)
    except FileNotFoundError:
        logger.log_warn("File not found: " + file_path)
    except Exception as e:
        logger.log_warn("Failed to load file: " + file_path + " " + str(e))
    return data

def clear_folder(folder_path):
    for root, dirs, files in os.walk(folder_path):
        for f in files:
            os.unlink(os.path.join(root, f))
        for d in dirs:
            shutil.rmtree(os.path.join(root, d))

def delete_files_in_folder(folder_path, ext = None):
    """
    Deletes all files within a specified folder, leaving subdirectories intact.

    Args:
        folder_path (str): The path to the folder to clean.
    """
    if not os.path.isdir(folder_path):
        print(f"Error: '{folder_path}' is not a valid directory.")
        return

    for item_name in os.listdir(folder_path):
        item_path = os.path.join(folder_path, item_name)
        if os.path.isfile(item_path):
            try:
                delete = True
                if ext is not None:
                    root, extension = os.path.splitext(item_path)
                    if ext != extension and ext != extension.replace('.',''):
                       delete = False
                if delete == True:
                    os.remove(item_path)
                    print(f"Deleted file: {item_path}")
            except OSError as e:
                print(f"Error deleting file {item_path}: {e}")

def rsync_folders(source_folder,destitation_folder, options = "-avrh", folders = True, delete = False):    
    success = False

    foptions=""
    if folders == False:
       foptions="--exclude='*/'"

    doption=""
    if delete == True:
       doption="--delete"

    if source_folder == destitation_folder or destitation_folder.find(source_folder) != -1:
        logger.log_warn("Can't sync same folders: " + source_folder + " : " + destitation_folder)
        return success
    # if os.path.basename(source_folder) == os.path.basename(destitation_folder):
    #     destitation_folder = os.path.dirname(destitation_folder)
    
    # Basic rsync command with common options
    if folders == False:
        command = ["rsync", options, foptions, source_folder + "/", destitation_folder]
    else:
        command = ["rsync", options, source_folder + "/", destitation_folder]
    logger.log_warn("Sync folders with run command: " + str(command))    

    try: 

        # Execute the command
        subprocess.run(command, text=True, check=True)
        success = True
        #print(result.stdout)
        
    except subprocess.CalledProcessError as e:
        logger.log_warn("Can't sync folders: " + source_folder + " : " + destitation_folder + " : " + str(e.stderr) + " : " + str(e.returncode))

    return success


def remove_pycache_folders(directory):
    """
    Recursively removes all __pycache__ folders within the specified directory.

    Args:
        directory (str): The path to the root directory to start searching from.
    """
    for root, dirs, files in os.walk(directory):
        if '__pycache__' in dirs:
            pycache_path = os.path.join(root, '__pycache__')
            print(f"Removing: {pycache_path}")
            shutil.rmtree(pycache_path)


#########################
### List Helper Functions

# Function for checking if val in list
def val_in_list(val2check,list2check):
  in_list = False
  if len(list2check) > 0:
    for list_val in list2check:
      #logger.log_warn(str(val2check) + ' , ' + str(list_val))
      #logger.log_warn(val2check == list_val)
      if val2check == list_val:
        in_list = True
  return in_list


def find_all_indexes(input_string, char):
    """
    Finds all indexes of a character in a string.

    Args:
        input_string: The string to search within.
        char: The character to find.

    Returns:
        A list of integers representing the indexes of the character, 
        or an empty list if the character is not found.
    """
    indexes = [i for i, letter in enumerate(input_string) if letter == char]
    return indexes



########################
### Class and Method Helper Functions


def get_caller_class(self):
    caller_class = None
    frame = inspect.stack()[1][0]
    args, _, _, value_dict = inspect.getargvalues(frame)
    # we check the first parameter for the frame function is
    if len(args) and args[0] == 'self':
      # in that case, 'self' will be referenced in value_dict
      instance = value_dict.get('self', None)
      if instance:
        caller_class = getattr(instance, '__class__', None)
    return caller_class

def get_caller_class_name(self):
    caller_class = get_caller_class(self)
    if caller_class is None:
      return ""
    else:
        caller_class_name = str(caller_class.__name__)
    return caller_class_name


##############
## Misc Math Functions

def rotate_3d(vector, axis, angle_degrees):
    angle_radians = np.deg2rad(angle_degrees)
    if axis == 'x':
        R = np.array([[1, 0, 0],
                      [0, np.cos(angle_radians), -np.sin(angle_radians)],
                      [0, np.sin(angle_radians), np.cos(angle_radians)]])
    elif axis == 'y':
        R = np.array([[np.cos(angle_radians), 0, np.sin(angle_radians)],
                      [0, 1, 0],
                      [-np.sin(angle_radians), 0, np.cos(angle_radians)]])
    elif axis == 'z':
        R = np.array([[np.cos(angle_radians), -np.sin(angle_radians), 0],
                      [np.sin(angle_radians), np.cos(angle_radians), 0],
                      [0, 0, 1]])
    else:
        raise ValueError("Invalid axis. Choose 'x', 'y', or 'z'.")
    return np.dot(R, vector)

def check_ratio(ratio):
    if ratio < 0.0:
        ratio = 0.0
    if ratio > 1.0:
        ratio = 1.0
    return ratio

##################
## Misc String Functions

def get_uppercase_letters():
  return list(string.ascii_uppercase)

def get_lowercase_letters():
  return list(string.ascii_uppercase)

##################
## Misc Math Functions

def get_closest_odd_integer(f_num):
    """
    Finds the closest odd integer to a given float.

    Args:
        f_num (float): The input floating-point number.

    Returns:
        int: The closest odd integer.
    """
    rounded_int = round(f_num)

    if rounded_int % 2 != 0:  # If already odd
        return int(rounded_int)
    else:  # If even, find the closest odd neighbor
        lower_odd = rounded_int - 1
        upper_odd = rounded_int + 1

        # Compare distances to the original float
        if abs(f_num - lower_odd) <= abs(f_num - upper_odd):
            return int(lower_odd)
        else:
            return int(upper_odd)
        

##################
## Misc Check Functions


def check_ratio(ratio):
    if ratio < 0:
      ratio = 0
    if ratio > 1:
       ratio = 1
    return ratio


####################
## Class Util Funtions


def importClass(file_name,file_path,module_name,class_name):
      module_class = None
      success = False
      msg = "failed"
      file_list = os.listdir(file_path)
      #logger.log_warn("Looking for class file: " + str(file_name) + " : " + str(file_list))
      if file_name in file_list:
        sys.path.append(file_path)
        try:
          module = importlib.import_module(module_name)
          try:
            module_class = getattr(module, class_name)
            success = True
            msg = 'success'
          except Exception as e:
            logger.log_warn("Failed to import class from module with exception: " + class_name +" "+ module_name +" "+ str(e))
        except Exception as e:
            logger.log_warn("Failed to import module with exception: " + module_name +" "+ str(e))
      else:
        logger.log_warn("Failed to find file in path: " + file_name +" "+ file_path)
      return success, msg, module_class


def unimportClass(module_name):
    success = True
    if module_name in sys.modules:
        try:
           sys.modules.pop(module_name)
        except:
            logger.log_info("Failed to clordered_unimport module: " + module_name)
        if module_name in sys.modules:
          success = False
    return success