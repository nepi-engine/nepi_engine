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


import os
import time 
import copy
import numpy as np
import copy
import threading

os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import open3d as o3d

import cv2


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_states
from nepi_sdk import nepi_triggers
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus, FilenameConfig
from nepi_interfaces.srv import SaveDataCapabilitiesQuery, SaveDataCapabilitiesQueryRequest, SaveDataCapabilitiesQueryResponse
from nepi_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryRequest, SystemStorageFolderQueryResponse

from nepi_interfaces.msg import Frame3DTransform, Frame3DTransformStatus

from nepi_interfaces.msg import Setting, SettingsStatus, SettingCap, SettingCaps
from nepi_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryRequest, SettingsCapabilitiesQueryResponse

from nepi_interfaces.msg import SystemState
from nepi_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_interfaces.msg import SystemTrigger
from nepi_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import  NodeClassIF


################################################
## ReadWriteIF


class ReadWriteIF:

    ready = False
    
    # Save data variables
    filename_dict = {
        'prefix': "",
        'suffix': "",
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }

    node_if = None


    #######################
    ### IF Initialization
    def __init__(self,
                filename_dict = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)
        

        #############################
        # Initialize Class Variables
        if filename_dict is not None:
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]

        self.data_dict = {
            'dict': {
                'data_type': dict,
                'file_types': ['yaml'],
                'read_function': self.read_dict_file,
                'write_function': self.write_dict_file
            },
            'image': {
                'data_type': np.ndarray,
                'file_types': ['png','PNG','jpg','jpeg','JPG'],
                'read_function': self.read_image_file,
                'write_function': self.write_image_file 
            },
            'pointcloud': {
                'data_type': o3d.geometry.PointCloud,
                'file_types': ['pcd'],
                'read_function': self.read_pointcloud_file,
                'write_function': self.write_pointcloud_file 
            },
        }

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready   
        

    def get_namespace(self):
        return self.namespace

    def get_supported_data_dict(self):
        return self.data_dict


    def get_filename_prefix(self):
        return self.filename_dict['prefix']

    def set_filename_prefix(self, prefix = ''):
        self.filename_dict['prefix'] = prefix

    def get_use_utc_tz(self):
        return self.filename_dict['use_utc_tz']

    def set_use_utc_tz(self, use_utc_tz):
        self.filename_dict['use_utc_tz'] = use_utc_tz

    def get_add_timestamp(self):
        return self.filename_dict['add_timestamp']

    def set_add_timestamp(self, add_timestamp):
        self.filename_dict['add_timestamp'] = add_timestamp

    def get_add_ms(self):
        return self.filename_dict['add_ms']

    def set_add_ms(self, add_ms):
        self.filename_dict['add_ms'] = add_ms

    def get_add_us(self):
        return self.filename_dict['add_us']

    def set_add_us(self, add_us):
        self.filename_dict['add_us'] = add_us

    def get_add_tz(self):
        return self.filename_dict['add_tz']

    def set_add_tz(self, add_tz):
        self.filename_dict['add_tz'] = add_tz


    def get_filename_dict(self):
        return self.filename_dict
    
    def set_filename_dict(self,filename_dict):
        for key in self.filename_dict.keys():
            if key not in filename_dict.keys():
                filename_dict[key] = self.filename_dict[key]

        self.filename_dict = filename_dict
        return self.filename_dict


    def get_folder_files(self, path, ext_str = ""):
        file_list = nepi_utils.get_file_list(path,ext_str=ext_str)
        return file_list

    def get_time_from_filename(self,filename):
        file_time = None
        dt_str = self._getDtStr(filename)
        file_time = nepi_utils.get_time_from_datetime_str(dt_str)
        return file_time



    def write_data_file(self, filepath, data, data_name, timestamp = None, timezone = None):
        data_type = type(data)
        found_type = False
        for data_key in self.data_dict:
            if data_type == self.data_dict[data_key]['data_type']:
                save_function = self.data_dict[data_key]['write_function']
                self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone), log_name_list = self.log_name_list, throttle_s = 5.0)
                save_function(filepath, data, data_name, timestamp = timestamp, timezone = timezone)
                found_type = True
        if found_type == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type) , log_name_list = self.log_name_list)


    def read_dict_file(self, filepath, filename):
        data_key = 'dict'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
            else:
                data = nepi_utils.read_yaml_2_dict(file_path)
        return data

    def write_dict_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'yaml'):
        data_key = 'dict'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    success = nepi_utils.write_dict_2_yaml(data, file_path)
        return success


    def read_image_file(self, filepath, filename):
        data_key = 'image'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
            else:
                data = nepi_img.read_image_file(file_path)
        return data

    def write_image_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'png'):
        data_key = 'image'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(type(data)), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    success = nepi_img.write_image_file(data, file_path)
        return success


    def read_pointcloud_file(self, filepath, filename):
        data_key = 'pointcloud'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
            else:
                data = nepi_pc.read_pointcloud_file(file_path)
        return data

    def write_pointcloud_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'pcd'):
        data_key = 'pointcloud'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    success = nepi_pc.write_pointcloud_file(file_path)
        return success


    def get_example_filename(self, data_name = 'data_product', timestamp = None, timezone = None, ext_str = 'ext'):
        filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
        return filename
    ###############################
    # Class Private Methods
    ###############################

    def _createFileName(self, data_name_str, timestamp = None, timezone = None, ext_str = ""):
        if timestamp == None:
            timestamp = nepi_utils.get_time()
        prefix = self.filename_dict['prefix']
        if len(prefix) > 0:
            if prefix[-1] != '_':
                prefix = prefix + '_'
        suffix = self.filename_dict['suffix']
        if len(suffix) > 0:
            if suffix[0] != '_':
                suffix = '_' + suffix
        add_time = self.filename_dict['add_timestamp']
        data_time_str = ''
        if add_time == True:
            time_ns = nepi_sdk.sec_from_timestamp(timestamp)
            add_ms = self.filename_dict['add_ms']
            add_us = self.filename_dict['add_us']
            add_tz = self.filename_dict['add_tz']
            data_time_str = nepi_utils.get_datetime_str_from_timestamp(time_ns, add_ms = add_ms, add_us = add_us, add_tz = add_tz, timezone = timezone) + '_'
        node_name_str = ""
        if self.filename_dict['add_node_name'] == True:
            node_name_str = self.node_name
        if len(ext_str) > 0:
            ext_str  = '.' + ext_str
        if len(data_name_str) >  0:
            data_name_str = '-' + data_name_str
        filename = prefix + data_time_str + node_name_str + data_name_str + suffix + ext_str
        return filename

    
    def _getDtStr(self,filename):
        d_inds = nepi_utils.find_all_indexes(filename, 'D')
        dt_ind = None
        dt_str = filename
        for ind in d_inds:
            if len(filename) >= ind + 1:
                if filename[ind + 1].isdigit() == True:
                    dt_ind = ind
                    break
        dt_str = dt_str[dt_ind:]
        dt_str = dt_str.split('_')[0]
        return dt_str
                




################################################
## SaveDataIF

FALLBACK_DATA_FOLDER = '/mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']

'''
EXAMPLE_RATE_DICT = {
    'data_product_1' : [save_rate_hz, last_time, max_rate],
    'data_product_2' : [save_rate_hz, last_time, max_rate]
}
'''

EXAMPLE_FILENAME_DICT = {
    'prefix': "", 
    'subfolder': "",
    'add_timestamp': True, 
    'use_utc_tz': True,
    'add_ms': True,
    'add_us': False,
    'add_tz': True,
    'add_node_name': False
    }

class SaveDataIF:

    DEFAULT_TIMEZONE = 'UTC'

    ready = None
    namespace = "~"
    all_save_namespace = None
    navpose_save_namespace = None
    status_msg = SaveDataStatus
    node_if = None
    read_write_if = None
 
    snapshot_dict = dict()

    sys_mgr_if = None
    read_write_if = None

    save_data_root_directory = FALLBACK_DATA_FOLDER

    filename_dict = {
        'prefix': "", 
        'subfolder': "", 
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }

    save_rate_dict = dict()
    save_data = False
    use_utc_tz = False

    save_all_enabled = False
    save_all_rate = 0.0

    log_navposes_enabled = False
    log_navposes_rate = 0.0

    file_prefix = ""
    subfolder = ""

    was_saving = False

    ### IF Initialization
    def __init__(self, 
                namespace = None,
                data_products = [], 
                factory_rate_dict = None, 
                factory_filename_dict = None, 
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)

        self.msg_if.pub_info("Starting SaveData IF Initialization Processes", log_name_list = self.log_name_list)
        ############################## 
        # Initialize Class Variables
        if namespace is None:
            namespace = self.node_namespace
        namespace = nepi_sdk.create_namespace(namespace,'save_data')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_warn("Using save data namespace: " + self.namespace, log_name_list = self.log_name_list)
        
        tzd = nepi_utils.get_timezone_description(self.DEFAULT_TIMEZONE)
        self.timezone = tzd

        all_save_namespace = nepi_sdk.create_namespace(self.base_namespace,'/save_data')
        if all_save_namespace != self.namespace:
            self.all_save_namespace = all_save_namespace

        navpose_save_namespace = nepi_sdk.create_namespace(self.base_namespace,'navposes/save_data')
        if navpose_save_namespace != self.namespace:
            self.navpose_save_namespace = navpose_save_namespace
        ###############################
        # Connect Sys Mgr Services

        
        ##############################
        # Get for System Folders
        self.msg_if.pub_info("Waiting for user folders")
        user_folders = nepi_system.get_user_folders(log_name_list = [self.node_name])
        #self.msg_if.pub_warn("Got user folders: " + str(user_folders))
        if user_folders is not None and 'data' in user_folders.keys():
            self.save_data_root_directory = user_folders['data']
        self.msg_if.pub_info("Using SDK Share Folder: " + str(self.save_data_root_directory))

        # Ensure the data folder exists with proper ownership
        if not os.path.exists(self.save_data_root_directory):
            self.msg_if.pub_warn("Reported data folder does not exist... data saving is disabled", log_name_list = self.log_name_list)
            self.save_data_root_directory = None # Flag it as non-existent
            return # Don't enable any of the ROS interface stuff
        self.save_path = self.save_data_root_directory
        # And figure out user/group so that we know what ownership to create subfolders with
        stat_info = os.stat(self.save_data_root_directory)
        self.DATA_UID = stat_info.st_uid
        self.DATA_GID = stat_info.st_gid

 
 


        # Setup System IF Classes
        # Initialize with empty dict, then call update function
        self.read_write_if = ReadWriteIF(
                            filename_dict = dict()
                            )
        if factory_filename_dict is not None:
            self.update_filename_dict(factory_filename_dict)

        # Config initial data products dict
        self.msg_if.pub_debug("^^^^^^^^^^^^^^^^^^^^^^", log_name_list = self.log_name_list)
        self.msg_if.pub_debug("Starting Save_Data_IF with data products: " + str(data_products), log_name_list = self.log_name_list)
        self.msg_if.pub_debug("Starting Save_Data_IF with rate dict: " + str(factory_rate_dict), log_name_list = self.log_name_list)
        save_rate_dict = dict()
        save_rate = 0.0
        last_time = 0.0
        max_rate = 100
        for data_product in data_products:
            save_rate = 0.0
            last_time = 0.0
            max_rate = 100
            save_rate_entry = [save_rate, last_time, max_rate]
            if factory_rate_dict is not None:
                if data_product in factory_rate_dict.keys():
                    save_rate = factory_rate_dict[data_product][0]
            save_rate_entry[0] = save_rate
            save_rate_dict[data_product] = save_rate_entry
            self.snapshot_dict[data_product] = False
        self.save_rate_dict = save_rate_dict
        self.msg_if.pub_debug("Got defualt data rate dict: " + str(self.save_rate_dict), log_name_list = self.log_name_list)
            


        ##############################    
        # Node Setup
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'save_rate_dict': {
                'namespace': self.namespace,
                'factory_val': self.save_rate_dict
            },
            'save_data': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'filename_dict': {
                'namespace': self.namespace,
                'factory_val': self.filename_dict
            },
            'log_navposes_enabled': {
                'namespace': self.namespace,
                'factory_val': self.log_navposes_enabled
            },
            'log_navposes_rate': {
                'namespace': self.namespace,
                'factory_val': self.log_navposes_rate
            }
        }


        # Services Config Dict ####################
        if self.namespace == self.namespace:
            self.SRVS_DICT = {
                'capabilities_query': {
                    'namespace': self.namespace,
                    'topic': 'capabilities_query',
                    'srv': SaveDataCapabilitiesQuery,
                    'req': SaveDataCapabilitiesQueryRequest(),
                    'resp': SaveDataCapabilitiesQueryResponse(),
                    'callback': self._capabilitiesHandler
                }
            }
        else:
            self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'msg': SaveDataStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        if self.navpose_save_namespace is not None:
            self.PUBS_DICT['set_navpose_enable'] = {
                'namespace': self.navpose_save_namespace,
                'msg': Bool,
                'topic': 'set_navpose_enable',
                'qsize': 1,
                'latch': True
            }
            self.PUBS_DICT['set_navpose_rate'] = {
                'namespace': self.navpose_save_namespace,
                'msg': SaveDataRate,
                'topic': 'set_navpose_rate',
                'qsize': 1,
                'latch': True
            }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'save': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_enable',
                'qsize': 5,
                'callback': self._saveEnableCb, 
                'callback_args': ()
            },  
            'log_navposes_enable': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'log_navposes_enable',
                'qsize': 5,
                'callback': self._logNavPoseEnableCb, 
                'callback_args': ()
            }, 
            'log_navposes_rate': {
                'namespace': self.namespace,
                'msg': Float32,
                'topic': 'log_navposes_rate',
                'qsize': 5,
                'callback': self._logNavPoseRateCb, 
                'callback_args': ()
            }, 
            'prefix': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_prefix',
                'qsize': 5,
                'callback': self._setPrefixCb, 
                'callback_args': ()
            },
            'subfolder': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_subfolder',
                'qsize': 5,
                'callback': self._setSubfolderCb, 
                'callback_args': ()
            },
            'save_data_utc': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_utc',
                'qsize': 5,
                'callback': self._setLocalTzCb, 
                'callback_args': ()
            },
            'filename': {
                'namespace': self.namespace,
                'msg': FilenameConfig,
                'topic': 'filename_config',
                'qsize': 5,
                'callback': self._setFilenameCb, 
                'callback_args': ()
            },
            'rate': {
                'namespace': self.namespace,
                'msg': SaveDataRate,
                'topic': 'save_data_rate',
                'qsize': 5,
                'callback': self._saveRateCb, 
                'callback_args': ()
            },          
            'snapshot': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'snapshot_trigger',
                'qsize': 5,
                'callback': self._snapshotCb,  
                'callback_args': ()
            },
            'reset': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'reset_save_data',
                'qsize': 5,
                'callback': self._resetCb,  
                'callback_args': ()
            },

        }

        if self.all_save_namespace is not None:
            ALL_SUBS_DICT =  {
                'save_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'save_data_enable',
                    'qsize': 5,
                    'callback': self._saveEnableCb, 
                    'callback_args': ()
                },
                'prefix_all': {
                    'namespace': self.all_save_namespace,
                    'msg': String,
                    'topic': 'save_data_prefix',
                    'qsize': 5,
                    'callback': self._setPrefixCb, 
                    'callback_args': ()
                },
                'subfolder_all': {
                    'namespace': self.all_save_namespace,
                    'msg': String,
                    'topic': 'save_data_subfolder',
                    'qsize': 5,
                    'callback': self._setSubfolderCb, 
                    'callback_args': ()
                },
                'use_local_tz_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Bool,
                    'topic': 'save_data_utc',
                    'qsize': 5,
                    'callback': self._setLocalTzCb, 
                    'callback_args': ()
                },
                'filename_all': {
                    'namespace': self.all_save_namespace,
                    'msg': FilenameConfig,
                    'topic': 'filename_config',
                    'qsize': 5,
                    'callback': self._setFilenameCb,
                    'callback_args': ()
                },
                'rate_all': {
                    'namespace': self.all_save_namespace,
                    'msg': SaveDataRate,
                    'topic': 'save_data_rate',
                    'qsize': 5,
                    'callback': self._saveRateCb, 
                    'callback_args': ()
                },  
                'snapshot_all': {
                    'namespace': self.all_save_namespace,
                    'msg': Empty,
                    'topic': 'snapshot_trigger',
                    'qsize': 5,
                    'callback': self._snapshotCb, 
                    'callback_args': ()
                },
                'save_all_config': {
                    'namespace': self.all_save_namespace,
                    'msg': Empty,
                    'topic': 'save_config',
                    'qsize': 5,
                    'callback': self._saveConfigCb, 
                    'callback_args': ()
                },
                'all_save_sub': {
                    'namespace': self.all_save_namespace,
                    'msg': SaveDataStatus,
                    'topic': 'status',
                    'qsize': 5,
                    'callback': self._saveAllStatusCb, 
                    'callback_args': ()
                }
            }

            self.SUBS_DICT.update(ALL_SUBS_DICT)
        


        if self.navpose_save_namespace is not None:
            self.SUBS_DICT['navpose_save_sub'] = {
                'namespace': self.navpose_save_namespace,
                'msg': SaveDataStatus,
                'topic': 'status',
                'qsize': 5,
                'callback': self._logNavPoseStatusCb, 
                'callback_args': ()
            }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()
        
        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################



    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready    

            
    def get_namespace(self):
        return self.namespace

    def get_data_products(self):
        return list(self.save_rate_dict.keys())

    def register_data_product(self, data_product,factory_rate = 0):

        save_rate_dict = self.save_rate_dict
        if data_product not in save_rate_dict.keys():
            save_rate_dict[data_product] =  [factory_rate, 0.0, 100] # Default to 1Hz save rate, max rate = 100Hz
            self.save_rate_dict = save_rate_dict
            self.snapshot_dict[data_product] = False
            self.publish_status()
            if self.node_if is not None:
                self.node_if.set_param('save_rate_dict',save_rate_dict)


    def update_filename_dict(self,filename_dict):
        if self.filename_dict != filename_dict:
            if 'prefix' in filename_dict.keys():
               new_prefix = filename_dict['prefix']
               self.filename_dict['prefix'] = nepi_utils.get_clean_name(new_prefix)

            if 'subfolder' in filename_dict.keys():
                new_subfolder= nepi_utils.get_clean_name(filename_dict['subfolder'])
                 

                if new_subfolder != "" and self.save_data_root_directory != None:
                    full_path = os.path.join(self.save_data_root_directory, new_subfolder)
                elif self.save_data_root_directory != None:
                    full_path = self.save_data_root_directory
                else:
                    full_path = ""

                if not os.path.exists(full_path):
                    self.msg_if.pub_debug("Creating new data subdirectory " + full_path)
                    try:
                        os.makedirs(full_path)
                    except Exception as e:
                        self.msg_if.pub_warn("Could not create save folder " + new_subfolder + str(e) )
                if os.path.exists(full_path):
                    try:
                        os.chown(full_path, self.DATA_UID, self.DATA_GID)
                    except Exception as e:
                        self.msg_if.pub_warn("Could not chmod on save folder " + full_path + str(e) )
                    self.save_path = full_path
                    filename_dict['subfolder'] = new_subfolder

            # Apply Updates
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]  

            self.publish_status()
            if self.read_write_if is not None:
                self.read_write_if.set_filename_dict(filename_dict)
            if self.node_if is not None:
                self.node_if.set_param('filename_dict',self.filename_dict)
                self.node_if.save_config()

    def set_save_rate(self,data_product,save_rate_hz=0):
        save_all = SaveDataRate().ALL_DATA_PRODUCTS
        save_none = SaveDataRate().NONE_DATA_PRODUCTS
        save_active = SaveDataRate().ACTIVE_DATA_PRODUCTS
        save_rate_dict = self.save_rate_dict
        if (data_product == save_active):
            for d in save_rate_dict.keys():
                # Respect the max save rate
                if save_rate_dict[d][0] > 0:
                    save_rate_dict[d][0] = save_rate_hz if save_rate_hz <= save_rate_dict[d][2] else save_rate_dict[d][2]
        elif (data_product == save_all):
            for d in save_rate_dict.keys():
                save_rate_dict[d][0] = save_rate_hz
        elif (data_product == save_none):
            for d in save_rate_dict.keys():
                save_rate_dict[d][0] = 0.0
        elif (data_product in save_rate_dict.keys()):
            save_rate_dict[data_product][0] = save_rate_hz if save_rate_hz <= save_rate_dict[data_product][2] else save_rate_dict[data_product][2]
        else:
            self.msg_if.pub_warn("Requested unknown data product: " + data_product)    
        self.save_rate_dict = save_rate_dict     
        #self.msg_if.pub_warn("Updated save rate dict: " + str(self.save_rate_dict))   
        self.publish_status()
        if self.node_if is not None:
            self.node_if.set_param('save_rate_dict',save_rate_dict)
            self.node_if.save_config()
        
    def save_data_enable(self, enabled):
        if enabled == True and self.was_saving == False:
            for d in self.save_rate_dict.keys():
                self.save_rate_dict[d][1] = 0.0
            self.was_saving = True
        else:
            self.was_saving = False
        self.msg_if.pub_warn("Setting Save Enable to: " + str(enabled))  
        self.save_data = enabled
        self.publish_status()    
        if self.node_if is not None: 
            self.node_if.set_param('save_data', enabled)  
            #self.node_if.save_config()

    def get_saving_enabled(self):
        return self.save_data

    def data_product_save_enabled(self, data_product):
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.save_rate_dict
            if self.save_data == False:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.pub_warn("Unknown data product " + data_product)
                return False

            save_rate = save_rate_dict[data_product][0]
            return (save_rate > 0.0)
        except:
            return False
        
    def data_product_save_rate(self, data_product):
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.save_rate_dict
            if self.save_data == False:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.pub_warn("Unknown data product " + data_product)
                return False

            save_rate = save_rate_dict[data_product][0]
            return save_rate
        except:
            return 0.0



    def data_product_should_save(self, data_product):
        # If saving is disabled for this node, then it is not time to save this data product!
        save_rate_dict = self.save_rate_dict
        #self.msg_if.pub_debug("Checking should save for save rate dict: " + str(save_rate_dict), log_name_list = self.log_name_list, throttle_s = 5)
        
        if self.save_data == False:
            return False

        if data_product not in save_rate_dict.keys():
            self.msg_if.pub_warn("Unknown data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False

        save_rate = save_rate_dict[data_product][0]
        if save_rate == 0.0:
            return False

        save_period = float(1) / float(save_rate)
        now = nepi_utils.get_time()
        elapsed = now - save_rate_dict[data_product][1]
        #self.msg_if.pub_debug("Checking should save: " + str([save_period,elapsed]), log_name_list = self.log_name_list, throttle_s = 5)
        snapshot = self.snapshot_dict[data_product]
        if (elapsed >= save_period or snapshot):
            self.msg_if.pub_debug("Should save: " + data_product + " : " + str([save_period,elapsed]), log_name_list = self.log_name_list, throttle_s = 5)
            self.save_rate_dict = save_rate_dict
            return True
        return False



    def data_product_snapshot_enabled(self, data_product):
        try:
            enabled = self.snapshot_dict[data_product]
            return enabled
        except:
            self.msg_if.pub_warn("Unknown snapshot data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False
        return False
    
    def data_product_snapshot_reset(self, data_product):
        try:
            self.snapshot_dict[data_product] = False
            return True
        except:
            self.msg_if.pub_warn("Unknown snapshot data product " + data_product, log_name_list = self.log_name_list, throttle_s = 5)
            return False
        return False
        

    def data_products_should_save_dict(self):
        dps_dict=copy.deepcopy(self.snapshot_dict)
        for dp in dps_dict.keys():
            ss =  dps_dict[dp]
            sr = self.save_rate_dict[dp][0] > 0
            dps_dict[dp] = ss or (self.save_data and sr)
        return dps_dict
        

     
    def get_timestamp_string(self):
        return nepi_utils.get_datetime_str_now(add_ms = True, add_us = False)

    def get_filename_path_and_prefix(self):
        if self.save_path is None:
            return ""
        return os.path.join(self.save_path, self.read_write_if.get_filename_prefix())




    #***************************
    # NEPI data saving utility functions
    def save(self,data_product,data,timestamp = None,save_check=True):
        should_save = self.data_product_should_save(data_product)
        snapshot_enabled = self.data_product_snapshot_enabled(data_product)
        # Save data if enabled
        self.msg_if.pub_debug("******", log_name_list = self.log_name_list, throttle_s = 5)
        save_check = [should_save, snapshot_enabled, save_check]
        self.msg_if.pub_debug("Checking save checks: " + data_product + " " + str(save_check) , log_name_list = self.log_name_list, throttle_s = 5)
        if should_save or snapshot_enabled or save_check == False:
            if self.filename_dict['use_utc_tz'] == False:
                timezone = self.timezone
            else:
                timezone = 'UTC'
            self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone) , log_name_list = self.log_name_list, throttle_s = 5)
            self.read_write_if.write_data_file(self.save_path, data, data_product, timezone = timezone, timestamp = timestamp)
            self.data_product_snapshot_reset(data_product)
            self.save_rate_dict[data_product][1] = nepi_utils.get_time()
        self.msg_if.pub_debug("Finished Checking save data: " + data_product , log_name_list = self.log_name_list, throttle_s = 5)
        self.msg_if.pub_debug("******", log_name_list = self.log_name_list, throttle_s = 5)


    def create_filename_msg(self):
        fn_msg = FilenameConfig()
        fn_dict = self.filename_dict
        fn_msg.save_prefix = fn_dict['prefix']
        fn_msg.save_subfolder = fn_dict['subfolder']
        fn_msg.add_timestamp = fn_dict['add_timestamp']
        fn_msg.use_utc_tz = fn_dict['use_utc_tz']
        fn_msg.add_ms = fn_dict['add_ms']
        fn_msg.add_us = fn_dict['add_us']
        fn_msg.add_tz = fn_dict['add_tz']
        return fn_msg


    def publish_status(self):
        if self.node_if is not None:
            save_rates_msg = []
            save_rate_dict = self.save_rate_dict

            self.msg_if.pub_debug("Status pub save_rate_dict " + str(save_rate_dict), log_name_list = self.log_name_list, throttle_s = 5)
            for name in save_rate_dict.keys():
                save_rate_msg = SaveDataRate()
                save_rate_msg.data_product = name
                save_rate_msg.save_rate_hz = save_rate_dict[name][0]
                save_rates_msg.append(save_rate_msg)
                self.msg_if.pub_debug("data_rates_msg " + str(save_rates_msg), log_name_list = self.log_name_list, throttle_s = 5)
            status_msg = SaveDataStatus()
            status_msg.node_name = self.node_name
            status_msg.save_data_topic = self.namespace
            status_msg.filename_config = self.create_filename_msg()
            status_msg.data_dir = self.save_path
            status_msg.filename_prefix = self.filename_dict['prefix']
            status_msg.save_subfolder = self.filename_dict['subfolder']
            status_msg.save_data_utc = self.filename_dict['use_utc_tz']
            status_msg.timezone = self.timezone
            status_msg.data_products = list(save_rate_dict.keys())
            status_msg.save_data_rates = save_rates_msg
            status_msg.save_data_enabled = self.save_data

            if self.all_save_namespace is None:
                status_msg.save_all_enabled = self.save_data
                status_msg.save_all_rate = save_rate_dict['All'][0]
            else:
                status_msg.save_all_enabled = self.save_all_enabled
                status_msg.save_all_rate = self.save_all_rate   

            if self.navpose_save_namespace == self.namespace and 'navposes' in save_rate_dict.keys():
                status_msg.log_navposes_enabled = self.save_data
                status_msg.log_navposes_rate = save_rate_dict['navposes'][0]
            else:
                status_msg.log_navposes_enabled = self.log_navposes_enabled
                status_msg.log_navposes_rate = self.log_navposes_rate                


            if self.save_data_root_directory is not None:
                status_msg.data_dir = self.save_data_root_directory

            if self.filename_dict['use_utc_tz'] == False:
                timezone = self.timezone
            else:
                timezone = 'UTC'
            self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone) , log_name_list = self.log_name_list, throttle_s = 5)
            exp_filename = self.read_write_if.get_example_filename(timezone = timezone)
            status_msg.example_filename = exp_filename
            if self.node_if is not None:
                self.node_if.publish_pub('status_pub', status_msg)

    def init(self, do_updates = False):
        #self.msg_if.pub_warn("Param updated save rate dict: " + str(self.save_rate_dict))
        if self.node_if is not None:
            save_rate_dict = self.node_if.get_param('save_rate_dict')
            for data_product in self.save_rate_dict.keys():
                if data_product in save_rate_dict.keys():
                    self.save_rate_dict[data_product][0] = save_rate_dict[data_product][0]
                self.save_rate_dict[data_product][1] = 0.0 # Reset timer
            self.node_if.set_param('save_rate_dict',self.save_rate_dict)
            self.save_data = self.node_if.get_param('save_data')
            filename_dict =  self.node_if.get_param('filename_dict')
            self.update_filename_dict(filename_dict)
            self.log_navposes_enabled = self.node_if.get_param('log_navposes_enabled')
            self.log_navposes_rate = self.node_if.get_param('log_navposes_rate')
            self.node_if.save_config()
        if do_updates == True:
            if self.navpose_save_namespace is not None:
                self.logNavPoseEnable(self.log_navposes_enabled)
                self.logNavPoseRate(self.log_navposes_rate)
        self.publish_status()
        

    def reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Factory reseting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        if self.node_if is not None:
            pass
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def updaterCb(self,timer):
        tzd = nepi_system.get_timezone()
        last_tz = copy.deepcopy(self.timezone)
        self.timezone = tzd
        self.updater = nepi_sdk.start_timer_process(1, self.updaterCb, oneshot = True)


    def _capabilitiesHandler(self, req):
        return_list = []
        save_rate_dict = self.save_rate_dict
        for d in save_rate_dict:
            return_list.append(SaveDataRate(data_product = d, save_rate_hz = save_rate_dict[d][0]))
        return SaveDataCapabilitiesQueryResponse(return_list)





    def _logNavPoseEnableCb(self, msg):
        self.msg_if.pub_info("Recieved Log NavPose Enable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data

    def logNavPoseEnable(self,enabled):
        self.log_navposes_enabled = enabled
        self.publish_status()
        self.node_if.publish_pub('set_navpose_enable',enabled)

    def _logNavPoseRateCb(self, msg):
        self.msg_if.pub_info("Recieved Log NavPose Rate Update: " + str(msg), log_name_list = self.log_name_list)
        rate = msg.data
        self.logNavPoseRate(rate)


    def logNavPoseRate(self,rate):
        self.log_navposes_rate = rate
        self.publish_status()
        rate_msg = SaveDataRate()
        rate_msg.data_product = 'navposes'
        rate_msg.save_rate_hz = rate
        self.node_if.publish_pub('set_navpose_rate',rate_msg)
        self.publish_status()


    def _logNavPoseStatusCb(self,msg):
        data_product_list = msg.data_products
        if 'navposes' in data_product_list:
            self.log_navposes_enabled = msg.save_data_enabled
            index = data_product_list.index('navposes')
            self.log_navposes_rate = msg.save_data_rates[index]


    def _saveAllStatusCb(self,msg):
        #self.msg_if.pub_warn("Recieved save All status msg: " + str(msg), log_name_list = self.log_name_list)
        data_product_list = msg.data_products
        if 'All' in data_product_list:
            self.save_all_enabled = msg.save_data_enabled
            index = data_product_list.index('All')
            self.save_all_rate = msg.save_data_rates[index].save_rate_hz


    def _saveEnableCb(self, msg):
        self.msg_if.pub_info("Recieved Enable Update: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.save_data_enable(enabled)
        

    def _saveRateCb(self, msg):
        self.msg_if.pub_info("Recieved Rate Update: " + str(msg), log_name_list = self.log_name_list)
        data_product = msg.data_product
        save_rate_hz = msg.save_rate_hz
        self.set_save_rate(data_product,save_rate_hz)
    
        
    def _setPrefixCb(self, msg):
        prefix = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        filename_dict['prefix'] = prefix
        self.update_filename_dict(filename_dict)


    def _setSubfolderCb(self, msg):
        subfolder = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        filename_dict['subfolder'] = subfolder
        self.update_filename_dict(filename_dict)


    def _setLocalTzCb(self, msg):
        use_utc = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        filename_dict['use_utc_tz'] = use_utc
        self.update_filename_dict(filename_dict)


    def _setFilenameCb(self, msg):
        filename_dict = nepi_utils.convert_msg2dict(msg)
        self.update_filename_dict(filename_dict)



    def _snapshotCb(self,msg):
        self.msg_if.pub_info("Recieved Snapshot Trigger", log_name_list = self.log_name_list)
        save_rate_dict = self.save_rate_dict
        for data_product in save_rate_dict.keys():
            save_rate = save_rate_dict[data_product][0]
            enabled = (save_rate > 0.0)
            if enabled:
                self.snapshot_dict[data_product] = True

    def _saveConfigCb(self,msg):
        if self.node_if is not None:
            self.node_if.save_config()

    def _resetCb(self,reset_msg):
        self.reset()


    def _factoryResetCb(self,reset_msg):
        self.factory_reset


    def _publishStatusCb(self, timer):
        self.publish_status()

#######################################
# Transform3DIF

# Transform_List = [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg, heading_deg]

ZERO_TRANSFORM = [0,0,0,0,0,0,0]

class Transform3DIF:


    # Class Vars ####################

    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    transform = ZERO_TRANSFORM
    source = ''
    end = ''
    has_transform = True
    supports_updates = True

    status_msg = Frame3DTransformStatus()
    
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                source_ref_description = '',
                end_ref_description = '',
                get_3d_transform_function = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting Settings IF Initialization Processes", log_name_list = self.log_name_list)
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = self.node_namespace
        namespace = nepi_sdk.create_namespace(namespace,'navpose_frame_transform')
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.source = source_ref_description
        self.end = end_ref_description
        self.get_3d_transform_function = get_3d_transform_function
        if self.get_3d_transform_function is not None:
            self.supports_updates = False

        self.status_msg.has_transform = self.has_transform
        self.status_msg.supports_updates = self.supports_updates
        ##############################  
        # Create NodeClassIF Class  
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }
        
        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'transform': {
                'namespace': self.namespace,
                'factory_val': self.transform
            },
            'source': {
                'namespace': self.namespace,
                'factory_val': self.source
            },
            'end': {
                'namespace': self.namespace,
                'factory_val': self.end
            }
        }

        # Services Config Dict ####################
        self.SRVS_DICT = None
        

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'msg': Frame3DTransformStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
            'transform_pub': {
                'namespace': self.namespace,
                'msg': Frame3DTransform,
                'topic': '',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################

        if self.supports_updates == False:
            self.SUBS_DICT = None
        else:
            self.SUBS_DICT = {
                'clear_navpose_frame_transform': {
                    'namespace': self.namespace,
                    'topic': 'clear_3d_transform',
                    'msg': Empty,
                    'qsize': 5,
                    'callback': self._clearFrame3dTransformCb, 
                    'callback_args': ()
                },
                'set_navpose_frame_transform': {
                    'namespace': self.namespace,
                    'topic': 'set_3d_transform',
                    'msg': Frame3DTransform,
                    'qsize': 5,
                    'callback': self._setFrame3dTransformCb, 
                    'callback_args': ()
                }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                                    configs_dict = self.CONFIGS_DICT,
                                    params_dict = self.PARAMS_DICT,
                                    services_dict = self.SRVS_DICT,
                                    pubs_dict = self.PUBS_DICT,
                                    subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list
        )
   

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()    
    
        nepi_sdk.start_timer_process(1.0, self._publishTransformCb)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

  
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  
    
    def get_namespace(self):
        return self.namespace

    def get_zero_3d_transform(self):
        return ZERO_TRANSFORM

    def get_3d_transform(self):
        transform = None
        if self.get_3d_transform_function is not None:
            transform = self.get_3d_transform_function()
        if transform is None:
            transform = self.transform
        return transform
    
    def get_3d_transform_msg(self):
        tr = self.get_3d_transform()
        tr_msg = nepi_nav.convert_transform_list2msg(tr,
                source_ref_description = self.source,
                end_ref_description = self.end)
        return tr_msg

    def set_3d_transform(self,transform_list):
        if self.supports_updates == True:
            if len(transform_list) == 7:
                self.transform = transform_list
                self.publish_transform()
                self.node_if.set_param('transform',transform_list)

    def clear_3d_transform(self):
        if self.supports_updates == True:
            self.transform = ZERO_TRANSFORM
            self.publish_transform()
            self.node_if.set_param('transform',transform_list)

    def set_has_transform(self,has_transform):
        self.has_trasform = has_transform

    def get_has_transform(self):
        return self.has_transform


    def get_source_description(self):
        return self.source

    def set_source_description(self,source):
        self.source = source
        self.publish_transform()
        self.node_if.set_param('source',source)

    def get_end_description(self):
        return self.end

    def set_end_description(self,end):
        self.end = end
        self.publish_transform()
        self.node_if.set_param('end',end)


    def publish_transform(self):
        transform = self.get_3d_transform()
        transform_msg = nepi_nav.convert_transform_list2msg(transform,
                                                source_ref_description = self.source,
                                                end_ref_description = self.end)
        if self.node_if is not None:
            self.node_if.publish_pub('transform_pub',transform_msg)

    def publish_status(self):
        if self.node_if is not None:
            self.status_msg.has_transform = self.has_transform
            self.node_if.publish_pub('status_pub',self.status_msg)

    def init(self, do_updates = True):
        if self.node_if is not None:
            self.transform = self.node_if.get_param('transform')
            self.source = self.node_if.get_param('source')
            self.end = self.node_if.get_param('end')
        #self.msg_if.pub_debug("Setting init values to param server values: " + str(self.init_settings), log_name_list = self.log_name_list)
        if do_updates:
            pass
        self.publish_status()


    def reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Factory reseting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)


    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _setFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Frame Transform update message: " + str(msg))
        transform_msg = msg
        x = transform_msg.translate_vector.x
        y = transform_msg.translate_vector.y
        z = transform_msg.translate_vector.z
        roll = transform_msg.rotate_vector.x
        pitch = transform_msg.rotate_vector.y
        yaw = transform_msg.rotate_vector.z
        heading = transform_msg.heading_offset
        transform = [x,y,z,roll,pitch,yaw,heading]
        self.set_3d_transform(transform)


    def _clearFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Clear 3D Transform update message: ")
        self.clear_3d_transform()

    def _publishTransformCb(self, timer):
        self.publish_transform()

    def _publishStatusCb(self, timer):
        self.publish_status()



#######################################
# SettingsIF



SETTING_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]
EXAMPLE_CAP_SETTINGS = {"setting_name":{"name":"setting_name","type":"Int","optons":[]}}
EXAMPLE_SETTING  = {"name":"setting_name","type":"Int","value":"0"}
EXAMPLE_FACTORY_SETTINGS = {"setting_name":EXAMPLE_SETTING}
EXAMPLE_SETTINGS = {"setting_name":EXAMPLE_SETTING}

def EXAMPLE_SET_SETTING_FUNCTION(setting):
    success = False
    msg = 'Failed'
    if setting_name in EXAMPLE_SETTINGS.keys():
        EXAMPLE_SETTINGS[setting_name]= setting
        success = True
        msg = 'Success'
    return success, msg

def EXAMPLE_GET_SETTINGS_FUNCTION():
    return EXAMPLE_SETTINGS
'''
EXAMPLE_SETTINGS_DICT = {
                    'capSettings': EXAMPLE_CAP_SETTINGS, 
                    'factorySettings': EXAMPLE_FACTORY_SETTINGS,
                    'setSettingFunction': EXAMPLE_SET_SETTING_FUNCTION, 
                    'getSettingsFunction': EXAMPLE_GET_SETTINGS_FUNCTION
}
'''

class SettingsIF:


    # Class Vars ####################

    msg_if = None
    ready = False
    namespace = '~'

    node_if = None
    
   
    caps_settings = nepi_settings.NONE_CAP_SETTINGS
    factorySettings = nepi_settings.NONE_SETTINGS
    setSettingFunction = None

    caps_response = SettingsCapabilitiesQueryResponse()

    init_settings = dict()
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace = None,
                settings_dict = None,
                allow_cap_updates = False,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting Settings IF Initialization Processes", log_name_list = self.log_name_list)
        

        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = self.node_namespace
        namespace = nepi_sdk.create_namespace(namespace,'settings')
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.allow_cap_updates = allow_cap_updates
        self.msg_if.pub_warn("Initialize Class Variables: " + str(settings_dict))

        if settings_dict is None:
            self.msg_if.pub_warn("Exiting, No Settings_Dict provided", log_name_list = self.log_name_list)
            return
        else:
            try:
                capSettings = settings_dict['capSettings']
                factorySettings = settings_dict['capSettings']
                setSettingFunction = settings_dict['setSettingFunction']
                getSettingsFunction = settings_dict['getSettingsFunction']
            except Exception as e:
                self.msg_if.pub_warn("Exiting, None Valid Settings Dict: " + str(settings_dict) + " : " + str(e), log_name_list = self.log_name_list)
                return


        #  Initialize capabilities info
        if capSettings is None:
            self.cap_settings = nepi_settings.NONE_CAP_SETTINGS
        else:
            self.msg_if.pub_debug("Got Node settings capabilitis dict : " + str(capSettings), log_name_list = self.log_name_list)
            self.cap_settings = capSettings   
        caps_response = nepi_settings.create_capabilities_response(self.cap_settings,has_cap_updates = self.allow_cap_updates)
        self.msg_if.pub_debug("Cap Settings: " + str(caps_response), log_name_list = self.log_name_list)

        if factorySettings is None:
            self.factory_settings = nepi_settings.NONE_SETTINGS
        else:
            self.factory_settings = factorySettings
        self.msg_if.pub_debug(str(self.factory_settings), log_name_list = self.log_name_list)

        if setSettingFunction is None:
            self.setSettingFunction = nepi_settings.UPDATE_NONE_SETTINGS_FUNCTION
        else:
            self.setSettingFunction = setSettingFunction
        
        if getSettingsFunction is None:
            self.getSettingsFunction = nepi_settings.GET_NONE_SETTINGS_FUNCTION
        else:
            self.getSettingsFunction = getSettingsFunction
        #Reset Settings and Update Param Server


        ##############################  
        # Create NodeClassIF Class  
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }
        
        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'settings': {
                'namespace': self.namespace,
                'factory_val': dict()
            }
        }

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': SettingsCapabilitiesQuery,
                'req': SettingsCapabilitiesQueryRequest(),
                'resp': SettingsCapabilitiesQueryResponse(),
                'callback': self._capabilitiesHandler
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'msg': SettingsStatus,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'update_setting': {
                'msg': Setting,
                'namespace': self.namespace,
                'topic': 'update_setting',
                'qsize': 5,
                'callback': self._updateSettingCb,
                'callback_args': None
            },
            'reset_settings': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 5,
                'callback': self._resetSettingsCb,
                'callback_args': None
            }
        }

        if self.allow_cap_updates == True:
            self.SUBS_DICT['update_cap_setting'] = {
                    'msg': SettingCap,
                    'namespace': self.namespace,
                    'topic': 'update_cap_setting',
                    'qsize': 5,
                    'callback': self._updateCapSettingCb,
                    'callback_args': None
                },            


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                                    configs_dict = self.CONFIGS_DICT,
                                    params_dict = self.PARAMS_DICT,
                                    services_dict = self.SRVS_DICT,
                                    pubs_dict = self.PUBS_DICT,
                                    subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list
        )
   

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status() 
    
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb)

  
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def publish_status(self):
        if self.node_if is not None:
            current_settings = self.getSettingsFunction()
            self.node_if.set_param('settings', current_settings)
            cap_settings = self.cap_settings
            #self.msg_if.pub_warn("Settings status: " + str(current_settings) + " : " + str(cap_settings), log_name_list = self.log_name_list, throttle_s = 5.0)
            status_msg = nepi_settings.create_status_msg(current_settings,cap_settings,self.allow_cap_updates)
            status_msg.node_name = self.node_name
            status_msg.settings_topic = self.namespace
            #self.msg_if.pub_debug("Publishing settings status msg: " + str(status_msg), log_name_list = self.log_name_list, throttle_s = 5.0)
            self.node_if.publish_pub('status_pub', status_msg)

    def update_cap_setting(self,cap_setting):
        success = False
        if self.allow_cap_updates == False:
            self.msg_if.pub_warn("Ignoring cap update request. Cap updates not enabled", log_name_list = self.log_name_list)
            return success
        if 'name' in cap_setting.keys():
            name = cap_setting['name']
            if name in self.cap_settings.keys():
                if cap_setting['default_value'] not in cap_setting['options']:
                    if len(cap_setting['options'])>0:
                        cap_setting['default_value'] = cap_setting['options'][0]
                    else:
                        cap_setting['default_value'] = self.cap_settings['default_value']
        success = True
        self.msg_if.pub_warn("Updated Cap Setting: " + str(cap_setting), log_name_list = self.log_name_list)
        return success

    def update_setting(self,setting,do_updates = True, update_param = True):
        success = False
        current_settings = self.getSettingsFunction()
        updated_settings = copy.deepcopy(current_settings)
        self.msg_if.pub_debug("New Setting:" + str(setting), log_name_list = self.log_name_list)
        s_name = setting['name']
        if self.setSettingFunction != None:
            [name_match,type_match,value_match] = nepi_settings.compare_setting_in_settings(setting,current_settings)
            if value_match == False: # name_match would be True for value_match to be True
                self.msg_if.pub_debug("Will try to update setting " + str(setting), log_name_list = self.log_name_list)
                [success,msg] = nepi_settings.try_to_update_setting(setting,current_settings,self.cap_settings,self.setSettingFunction)
                self.msg_if.pub_warn(msg, log_name_list = self.log_name_list)
                if success:
                    if update_param:
                        updated_settings[s_name] = setting
                        self.node_if.set_param('settings', updated_settings)
                    if do_updates:
                        self.publish_status() 
        else:
            self.msg_if.pub_debug("Settings updates ignored. No settings update function defined ", log_name_list = self.log_name_list)
        return success


    def init(self, do_updates = True):
        current_settings = self.getSettingsFunction()
        if self.node_if is not None:
            self.init_settings = self.node_if.get_param('settings')
        #self.msg_if.pub_debug("Setting init values to param server values: " + str(self.init_settings), log_name_list = self.log_name_list)
        if do_updates:
            self.msg_if.pub_info("Applying Init Settings", log_name_list = self.log_name_list)
            self.msg_if.pub_debug(self.init_settings, log_name_list = self.log_name_list)
            if self.init_settings is not None:
                for setting_name in self.init_settings:
                    setting = self.init_settings[setting_name]
                    self.update_setting(setting, do_updates = False, update_param = False)
        self.publish_status()


    def reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
        self.init(do_updates = True)

    def factory_reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Factory reseting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
        self.init(do_updates = True)

       
    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _resetSettingsCb(self, msg):
        self.reset()

    def _capabilitiesHandler(self, req):
        caps_response = nepi_settings.create_capabilities_response(self.cap_settings, has_cap_updates = self.allow_cap_updates)
        return caps_response


    def _publishStatusCb(self, timer):
        self.publish_status()


    def _updateSettingCb(self,msg):
        self.msg_if.pub_info("Received setting update msg: " + str(msg), log_name_list = self.log_name_list)
        setting_dict = nepi_settings.parse_setting_msg(msg)
        self.update_setting(setting_dict, do_updates = True, update_param = True)

    def _updateCapSettingCb(self,msg):
        self.msg_if.pub_info("Received cap setting update msg: " + str(msg), log_name_list = self.log_name_list)
        cap_setting_dict = nepi_settings.parse_cap_setting_msg(msg)
        self.update_cap_setting(cap_setting_dict, do_updates = True, update_param = True)
















############################################
# StatesIF


STATE_TYPES = ["Menu","Discrete","String","Bool","Int","Float"]

EXAMPLE_STATES_DICT = {
                    "state_name": {
                        "name": "None",
                        "node_name": "Node",
                        "description": "None",
                        "type":"Bool",
                        "options": [],
                        "value":"False"
                    }
}


class StatesIF:


    ready = False
    msg_if = None
    namespace = '~'

    get_states_dict_function = None



    #######################
    ### IF Initialization
    def __init__(self, 
                get_states_dict_function, 
                namespace = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting States IF Initialization Processes", log_name_list = self.log_name_list)
        
        #############################
        # Initialize Class Variables
        if namespace is None:
            namespace = self.node_namespace
        self.namespace = nepi_sdk.get_full_namespace(namespace)

        self.get_states_dict_function = get_states_dict_function


        ##############################  
        # Create NodeClassIF Class  

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'states_query': {
                'namespace': self.namespace,
                'topic': 'system_states_query',
                'srv': SystemStatesQuery,
                'req': SystemStatesQueryRequest(),
                'resp': SystemStatesQueryResponse(),                
                'callback': self._statesQueryHandler
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(services_dict = self.SRVS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        success = nepi_sdk.wait()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  


    def get_namespace(self):
        return self.namespace


    ###############################
    # Class Private Methods
    ###############################

    def _statesQueryHandler(self, req):
        resp = SystemStatesQueryResponse()
        try:
            states_dict = self.get_states_dict_function()
            resp = nepi_states.create_query_resp(states_dict)
        except:
            self.msg_if.pub_warn("Failed to create resp msg: " + str(e), log_name_list = self.log_name_list)
        return resp





EXAMPLE_TRIGGER_DICT = {
                    "name":"None",
                    "node_name": '~',
                    "description": "None",
                    "data_str_list":["None"],
                    "time":nepi_utils.get_time() 
}


EXAMPLE_TRIGGERS_DICT = {
                "trigger_name": {
                    "name":"None",
                    "node_name": '~',
                    "description": "None",
                    "data_str_list":["None"],
                    "time":nepi_utils.get_time() 
                    }

}



class TriggersIF:

    msg_if = None
    ready = False
    namespace = '~'

    triggers_dict = dict()

    #######################
    ### IF Initialization
    def __init__(self, 
                triggers_dict = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting Triggers IF Initialization Processes", log_name_list = self.log_name_list)
        
        #############################
        # Initialize Class Variables

        if triggers_dict is None:
            self.triggers_dict = dict()
        else:
            self.triggers_dict = triggers_dict
        ##############################  
        # Create NodeClassIF Class  


        # Services Config Dict ####################
        self.SRVS_DICT = {
            'trigger_query': {
                'namespace': self.base_namespace,
                'topic': 'system_triggers_query',
                'srv': SystemTriggersQuery,
                'req': SystemTriggersQueryRequest(),
                'resp': SystemTriggersQueryResponse(),
                'callback': self._triggersQueryHandler
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'trigger_pub': {
                'msg': SystemTrigger,
                'namespace': self.base_namespace,
                'topic': 'system_triggers',
                'qsize': 1,
                'latch': False
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        success = nepi_sdk.wait()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################



    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace


    def publish_trigger(self, trigger_dict):
        trig_msg = nepi_triggers.create_trigger_msg(self.namespace, trigger_dict)
        self.node_if.publish_pub('trigger_pub',trig_msg)
 

    ###############################
    # Class Private Methods
    ###############################

    def _triggersQueryHandler(self, req):
        resp = SystemTriggersQueryResponse()
        try:
            resp = nepi_triggers.create_query_resp(self.triggers_dict)
        except:
            self.msg_if.pub_warn("Failed to create resp msg: " + str(e), log_name_list = self.log_name_list)
        return resp


