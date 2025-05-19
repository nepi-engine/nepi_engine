#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import time 
import copy

import datetime
import numpy as np
import cv2
import open3d as o3d
import inspect


from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_states
from nepi_sdk import nepi_triggers


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64

from nepi_ros_interfaces.msg import Message

from nepi_ros_interfaces.msg import Setting, Settings, SettingCap, SettingCaps
from nepi_ros_interfaces.srv import SettingsCapabilitiesQuery, SettingsCapabilitiesQueryRequest, SettingsCapabilitiesQueryResponse


from nepi_ros_interfaces.msg import SaveDataRate, SaveDataStatus, FilenameConfig
from nepi_ros_interfaces.srv import DataProductQuery, DataProductQueryRequest, DataProductQueryResponse
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryRequest, SystemStorageFolderQueryResponse


from nepi_ros_interfaces.msg import SystemState
from nepi_ros_interfaces.srv import SystemStatesQuery, SystemStatesQueryRequest, SystemStatesQueryResponse

from nepi_ros_interfaces.msg import SystemTrigger
from nepi_ros_interfaces.srv import SystemTriggersQuery, SystemTriggersQueryRequest, SystemTriggersQueryResponse

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import  NodeClassIF
from nepi_api.data_if import ReadWriteIF
from nepi_api.connect_mgr_if_system import ConnectMgrSystemServicesIF
from nepi_api.connect_mgr_if_time_sync import ConnectMgrTimeSyncIF



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

    node_if = None
    read_write_if = None
 
    snapshot_dict = dict()

    sys_mgr_if = None
    read_write_if = None



    filename_dict = {
        'prefix': "", 
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

    file_prefix = ""
    subfolder = ""

    was_saving = False

    ### IF Initialization
    def __init__(self, 
                data_products = [], 
                factory_rate_dict = None, 
                factory_filename_dict = None, 
                namespace = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

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
            namespace = '~'
        self.namespace = nepi_ros.get_full_namespace(namespace)
        
        tzd = nepi_utils.get_timezone_description(self.DEFAULT_TIMEZONE)
        self.timezone = tzd
        ###############################
        # Connect Sys Mgr Services


        ##############################
        ## Wait for NEPI core managers to start
        self.sys_srv_if = ConnectMgrSystemServicesIF()
        success = self.sys_srv_if.wait_for_services()
        self.save_data_root_directory = self.sys_srv_if.get_sys_folder_path('data',FALLBACK_DATA_FOLDER) 
        
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

        # Wait for Time manager to start to call timezone info
        self.mgr_time_if = ConnectMgrTimeSyncIF()
        success = self.mgr_time_if.wait_for_services()
 
 


        # Setup System IF Classes
        # Initialize with empty dict, then call update function
        self.read_write_if = ReadWriteIF(
                            filename_dict = dict()
                            )
        self.update_filename_dict(factory_filename_dict)

        # Config initial data products dict
        self.msg_if.pub_debug("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^", log_name_list = self.log_name_list)
        self.msg_if.pub_debug("Starting Save_Data_IF with data products: " + str(data_products))
        self.msg_if.pub_debug("Starting Save_Data_IF with rate dict: " + str(factory_rate_dict))
        save_rate_dict = dict()
        save_rate = 0.0
        last_time = 0.0
        max_rate = 100,0
        for data_product in data_products:
            save_rate = 0.0
            last_time = 0.0
            max_rate = 100.0
            save_rate_entry = [save_rate, last_time, max_rate]
            if factory_rate_dict is not None:
                if data_product in factory_rate_dict.keys():
                    save_rate = factory_rate_dict[data_product][0]
            save_rate_entry[0] = save_rate
            save_rate_dict[data_product] = save_rate_entry
            self.snapshot_dict[data_product] = False
        self.save_rate_dict = save_rate_dict
        #self.msg_if.pub_warn("Initialized rate dict: " + str(self.save_rate_dict))
            


        ##############################    
        # Node Setup

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'save_rate_dict': {
                'namespace': self.namespace,
                'factory_val': save_rate_dict
            },
            'save_data': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'filename_dict': {
                'namespace': self.namespace,
                'factory_val': self.filename_dict
            }
        }



        # Services Config Dict ####################
        if self.namespace == self.node_namespace:
            self.SRVS_DICT = {
                'query_data_products': {
                    'namespace': self.namespace,
                    'topic': 'query_data_products',
                    'srv': DataProductQuery,
                    'req': DataProductQueryRequest(),
                    'resp': DataProductQueryResponse(),
                    'callback': self._dataProductQueryHandler
                }
            }
        else:
            self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'msg': SaveDataStatus,
                'topic': 'save_data_status',
                'qsize': 1,
                'latch': True
            }
        }


        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'prefix': {
                'namespace': self.namespace,
                'msg': String,
                'topic': 'save_data_prefix',
                'qsize': 1,
                'callback': self._setPrefixCb, 
                'callback_args': ()
            },
            'save_data_utc': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data_utc',
                'qsize': 1,
                'callback': self._setLocalTzCb, 
                'callback_args': ()
            },
            'filename': {
                'namespace': self.namespace,
                'msg': FilenameConfig,
                'topic': 'filename_config',
                'qsize': 1,
                'callback': self._setFilenameCb, 
                'callback_args': ()
            },
            'rate': {
                'namespace': self.namespace,
                'msg': SaveDataRate,
                'topic': 'save_data_rate',
                'qsize': 1,
                'callback': self._saveRateCb, 
                'callback_args': ()
            },
            'save': {
                'namespace': self.namespace,
                'msg': Bool,
                'topic': 'save_data',
                'qsize': 1,
                'callback': self._saveEnableCb, 
                'callback_args': ()
            },            
            'snapshot': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'snapshot_trigger',
                'qsize': 1,
                'callback': self._snapshotCb,  
                'callback_args': ()
            },
            'reset': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'save_data_reset',
                'qsize': 1,
                'callback': self._resetCb,  
                'callback_args': ()
            },
            'factory_reset': {
                'namespace': self.namespace,
                'msg': Empty,
                'topic': 'save_data_factory_reset',
                'qsize': 1,
                'callback': self._factoryResetCb,  
                'callback_args': ()
            },
            'prefix_all': {
                'namespace': self.base_namespace,
                'msg': String,
                'topic': 'save_data_prefix',
                'qsize': 1,
                'callback': self._setPrefixCb, 
                'callback_args': ()
            },
            'use_local_tz_all': {
                'namespace': self.base_namespace,
                'msg': Bool,
                'topic': 'save_data_utc',
                'qsize': 1,
                'callback': self._setLocalTzCb, 
                'callback_args': ()
            },
            'filename_all': {
                'namespace': self.base_namespace,
                'msg': FilenameConfig,
                'topic': 'filename_config',
                'qsize': 1,
                'callback': self._setFilenameCb, 
                'callback_args': ()
            },
            'rate_all': {
                'namespace': self.base_namespace,
                'msg': SaveDataRate,
                'topic': 'save_data_rate',
                'qsize': 1,
                'callback': self._saveRateCb, 
                'callback_args': ()
            },
            'save_all': {
                'namespace': self.base_namespace,
                'msg': Bool,
                'topic': 'save_data',
                'qsize': 1,
                'callback': self._saveEnableCb, 
                'callback_args': ()
            },  
            'snapshot_all': {
                'namespace': self.base_namespace,
                'msg': Empty,
                'topic': 'snapshot_trigger',
                'qsize': 1,
                'callback': self._snapshotCb, 
                'callback_args': ()
            },            

            'reset_all': {
                'namespace': self.base_namespace,
                'msg': Empty,
                'topic': 'save_data_reset',
                'qsize': 1,
                'callback': self._resetCb,  
                'callback_args': ()
            },
            'factory_reset_all': {
                'namespace': self.base_namespace,
                'msg': Empty,
                'topic': 'save_data_factory_reset',
                'qsize': 1,
                'callback': self._factoryResetCb,  
                'callback_args': ()
            },

        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        nepi_ros.sleep(1)
        #self.node_if.wait_for_ready()

        save_rate_dict = self.node_if.get_param('save_rate_dict')
        for data_product in self.save_rate_dict.keys():
            if data_product in save_rate_dict.keys():
                self.save_rate_dict[data_product][0] = save_rate_dict[data_product][0]
            self.save_rate_dict[data_product][1] = 0.0 # Reset timer
        self.node_if.set_param('save_rate_dict',self.save_rate_dict)
        self.msg_if.pub_warn("Param updated save rate dict: " + str(self.save_rate_dict))

        self.save_data = self.node_if.get_param('save_data')
        filename_dict =  self.node_if.get_param('filename_dict')
        self.update_filename_dict(filename_dict)
        
        self.updater = nepi_ros.start_timer_process(1, self.updaterCb, oneshot = True)
        ##############################
        # Complete Initialization
        self.publish_status()
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
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready    

            

    def register_data_product(self, data_product):
        save_rate_dict = self.save_rate_dict
        if data_product not in save_rate_dict.keys():
            save_rate_dict[data_product] = [1.0, 0.0, 100.0] # Default to 1Hz save rate, max rate = 100.0Hz
            self.save_rate_dict = save_rate_dict
            self.snapshot_dict[data_product] = False
            self.publish_status()
            self.node_if.set_param('save_rate_dict',save_rate_dict)

    def update_filename_dict(self,filename_dict):

        if self.filename_dict != filename_dict:
            cur_prefix = self.filename_dict['prefix']
            if 'prefix' in filename_dict.keys():
                new_prefix = filename_dict['prefix']
            else:
                new_prefix = cur_prefix


            # Get the file floder and file info from prefix
            if '\\' not in new_prefix:
                if new_prefix.find('/') == -1:
                    subfolder = ""
                    file_prefix = new_prefix
                else:
                    prefix_split = new_prefix.rsplit('/',1)
                    subfolder = prefix_split[0]
                    file_prefix = prefix_split[1]
                # Now ensure the directory exists if this prefix defines a subdirectory
            else:
                subfolder = ""
                file_prefix = ""
            self.file_prefix = file_prefix

            # Apply Updates
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]        
            if self.read_write_if is not None:
                filename_dict = copy.deepcopy(self.filename_dict)
                filename_dict['prefix'] = file_prefix
                self.read_write_if.update_filename_dict(filename_dict)
            if self.node_if is not None:
                self.node_if.set_param('filename_dict',self.filename_dict)
            self.publish_status()

            if cur_prefix != new_prefix:
                if subfolder != "" and self.save_data_root_directory != None:
                    full_path = os.path.join(self.save_data_root_directory, subfolder)
                elif self.save_data_root_directory != None:
                    full_path = self.save_data_root_directory
                else:
                    full_path = ""
                #self.msg_if.pub_warn("DEBUG!!!! Computed full path " + full_path + " and parent path " + parent_path)
                if self.subfolder != subfolder:
                    if not os.path.exists(full_path):
                        self.msg_if.pub_debug("Creating new data subdirectory " + full_path)
                        try:
                            os.makedirs(full_path)
                            os.chown(full_path, self.DATA_UID, self.DATA_GID)
                            self.save_path = full_path
                            self.subfolder  = subfolder
                        except Exception as e:
                            self.save_path = self.save_data_root_directory # revert to root folder
                            self.subfolder  = ""
                            self.msg_if.pub_warn("Could not create save folder " + subfolder + str(e) )
                    else:
                        self.save_path = full_path
                        self.subfolder  = subfolder
                    self.publish_status()



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
        self.msg_if.pub_warn("Updated save rate dict: " + str(self.save_rate_dict))   
        self.publish_status()
        self.node_if.set_param('save_rate_dict',save_rate_dict)
        
    def save_data_enable(self, enabled):
        if enabled == True and self.was_saving == False:
            for d in self.save_rate_dict.keys():
                self.save_rate_dict[d][1] = 0.0
            self.was_saving = True
        else:
            self.was_saving = False
        self.save_data = enabled
        self.publish_status()     
        self.node_if.set_param('save_data', enabled)  


    def get_saving_enable(self):
        return self.save_data

    def data_product_saving_enabled(self, data_product):
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

    def data_product_should_save(self, data_product):
        # If saving is disabled for this node, then it is not time to save this data product!
        save_rate_dict = self.save_rate_dict
        #self.msg_if.pub_warn("Checking should save for save rate dict: " + str(save_rate_dict))
        
        if self.save_data == False:
            return False

        if data_product not in save_rate_dict.keys():
            self.msg_if.pub_warn("Unknown data product " + data_product, throttle_s = 5)
            return False

        save_rate = save_rate_dict[data_product][0]
        if save_rate == 0.0:
            return False

        save_period = float(1) / float(save_rate)
        now = nepi_utils.get_time()
        elapsed = now - save_rate_dict[data_product][1]
        #self.msg_if.pub_warn("Checking should save: " + str([save_period,elapsed]))
        if (elapsed >= save_period):
            #self.msg_if.pub_warn("Should save: " + data_product + " : " + str([save_period,elapsed]))
            self.save_rate_dict = save_rate_dict
            return True
        return False



    def data_product_snapshot_enabled(self, data_product):
        try:
            enabled = self.snapshot_dict[data_product]
            return enabled
        except:
            self.msg_if.pub_warn("Unknown snapshot data product " + data_product)
            return False
        return False

    def data_product_snapshot_reset(self, data_product):
        try:
            self.snapshot_dict[data_product] = False
        except:
            self.msg_if.pub_warn("Unknown snapshot data product " + data_product)
            pass
     
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
        #self.msg_if.pub_warn("******", log_name_list = self.log_name_list)
        #self.msg_if.pub_warn("Checking save data: " + data_product + " " + str([saving_is_enabled,should_save,snapshot_enabled,save_check]) )
        if should_save or snapshot_enabled or save_check == False:
            if self.filename_dict['use_utc_tz'] == False:
                timezone = self.timezone
            else:
                timezone = 'UTC'
            #self.msg_if.pub_warn("Saving Data with Timezone: " + str(timezone) )
            self.read_write_if.write_data_file(self.save_path, data, data_product, timezone = timezone, timestamp = timestamp)
            self.data_product_snapshot_reset(data_product)
            self.save_rate_dict[data_product][1] = nepi_utils.get_time()
        #self.msg_if.pub_warn("Finished Checking save data: " + data_product )
        #self.msg_if.pub_warn("******", log_name_list = self.log_name_list)


    def create_filename_msg(self):
        fn_msg = FilenameConfig()
        fn_dict = self.filename_dict
        fn_msg.prefix = fn_dict['prefix']
        fn_msg.add_timestamp = fn_dict['add_timestamp']
        fn_msg.use_utc_tz = fn_dict['use_utc_tz']
        fn_msg.add_ms = fn_dict['add_ms']
        fn_msg.add_us = fn_dict['add_us']
        fn_msg.add_tz = fn_dict['add_tz']
        return fn_msg


    def publish_status(self):
        save_rates_msg = []
        save_rate_dict = self.save_rate_dict
        #self.msg_if.pub_warn("Status pub save_rate_dict " + str(save_rate_dict))
        for name in save_rate_dict.keys():
            save_rate_msg = SaveDataRate()
            save_rate_msg.data_product = name
            save_rate_msg.save_rate_hz = save_rate_dict[name][0]
            save_rates_msg.append(save_rate_msg)
            #self.msg_if.pub_warn("data_rates_msg " + str(save_rates_msg)
        status_msg = SaveDataStatus()
        status_msg.filename_config = self.create_filename_msg()
        status_msg.current_data_dir = self.save_path
        status_msg.current_filename_prefix = self.filename_dict['prefix']
        status_msg.current_subfolder = self.subfolder
        status_msg.save_data_utc = self.filename_dict['use_utc_tz']
        status_msg.current_timezone = self.timezone
        status_msg.save_data_rates = save_rates_msg
        status_msg.save_data = self.save_data

        if self.save_data_root_directory is not None:
            status_msg.current_data_dir = self.save_data_root_directory

        if self.filename_dict['use_utc_tz'] == False:
            timezone = self.timezone
        else:
            timezone = 'UTC'
        #self.msg_if.pub_warn("Saving Data with Timezone: " + str(timezone) )
        exp_filename = self.read_write_if.get_example_filename(timezone = timezone)
        status_msg.example_filename = exp_filename
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)


    ###############################
    # Class Private Methods
    ###############################


    def updaterCb(self,timer):
        time_status_dict = self.mgr_time_if.get_time_status()
        #self.msg_if.pub_warn("Got time status dict " + str(time_status_dict))
        if time_status_dict is not None:
            last_tz = copy.deepcopy(self.timezone)
            tzd = time_status_dict['timezone_description']
            #tzd = nepi_utils.get_timezone_description(tzd)
            self.timezone = tzd
        self.updater = nepi_ros.start_timer_process(1, self.updaterCb, oneshot = True)


    def _dataProductQueryHandler(self, req):
        return_list = []
        save_rate_dict = self.save_rate_dict
        for d in save_rate_dict:
            return_list.append(SaveDataRate(data_product = d, save_rate_hz = save_rate_dict[d][0]))
        return DataProductQueryResponse(return_list)


    def _setPrefixCb(self, msg):
        prefix = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        filename_dict['prefix'] = prefix
        self.update_filename_dict(filename_dict)

    def _setLocalTzCb(self, msg):
        use_utc = msg.data
        filename_dict = copy.deepcopy(self.filename_dict)
        filename_dict['use_utc_tz'] = use_utc
        self.update_filename_dict(filename_dict)

    def _setFilenameCb(self, msg):
        filename_dict = nepi_utils.convert_msg2dict(msg)
        self.update_filename_dict(filename_dict)



    def _saveRateCb(self, msg):
        self.msg_if.pub_info("Recieved Rate Update: " + str(msg))
        data_product = msg.data_product
        save_rate_hz = msg.save_rate_hz
        self.set_save_rate(data_product,save_rate_hz)



    def _saveEnableCb(self, msg):
        enabled = msg.data
        self.save_data_enable(enabled)
        

    def _snapshotCb(self,msg):
        self.msg_if.pub_info("Recieved Snapshot Trigger", log_name_list = self.log_name_list)
        save_rate_dict = self.save_rate_dict
        for data_product in save_rate_dict.keys():
            save_rate = save_rate_dict[data_product][0]
            enabled = (save_rate > 0.0)
            if enabled:
                self.snapshot_dict[data_product] = True


    def _resetCb(self,reset_msg):
        self.msg_if.pub_info("Recieved save data reset msg", log_name_list = self.log_name_list)
        self.node_if.reset_params()
        self.publish_status()

    def _factoryResetCb(self,reset_msg):
        self.msg_if.pub_info("Recieved save data factory reset msg", log_name_list = self.log_name_list)
        self.node_if.factory_reset_params()
        self.publish_status()








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
                    'getSettingsFunction': EXAMPLE_GET_SETTINGS_FUNCTION, 
                    'namespace':  self.node_namespace
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

    capabilities_response = SettingsCapabilitiesQueryResponse()

    init_settings = dict()
    #######################
    ### IF Initialization
    def __init__(self, 
                settings_dict,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

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
        if 'namespace' not in settings_dict.keys():
            namespace = None
        else:
            namespace = settings_dict['namespace']
        if namespace is None:
            namespace = '~'
        self.namespace = nepi_ros.get_full_namespace(namespace)

        capSettings = settings_dict['capSettings']
        factorySettings = settings_dict['capSettings']
        setSettingFunction = settings_dict['setSettingFunction']
        getSettingsFunction = settings_dict['getSettingsFunction']


        #  Initialize capabilities info
        if capSettings is None:
            self.cap_settings = nepi_settings.NONE_CAP_SETTINGS
            cap_setting_msgs_list = nepi_settings.get_cap_setting_msgs_list(self.cap_settings)
            self.capabilities_response.settings_count = len(cap_setting_msgs_list)
            self.capabilities_response.setting_caps_list = cap_setting_msgs_list
            self.factory_settings = nepi_settings.NONE_SETTINGS
        else:
            self.msg_if.pub_debug("Got Node settings capabilitis dict : " + str(capSettings))
            self.cap_settings = capSettings   
            cap_setting_msgs_list = nepi_settings.get_cap_setting_msgs_list(self.cap_settings)
            self.capabilities_response.settings_count = len(cap_setting_msgs_list)
            self.capabilities_response.setting_caps_list = cap_setting_msgs_list
        #self.msg_if.pub_warn("Cap Settings: " + str(self.capabilities_response))  

        if factorySettings is None:
            self.factory_settings = nepi_settings.NONE_SETTINGS
        else:
            self.factory_settings = factorySettings
        #self.msg_if.pub_warn(str(self.factory_settings))

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


        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'settings': {
                'namespace': self.namespace,
                'factory_val': dict()
            }
        }

        # Services Config Dict ####################
        self.SRVS_DICT = {
            'cap_settings': {
                'namespace': self.namespace,
                'topic': 'settings_capabilities_query',
                'srv': SettingsCapabilitiesQuery,
                'req': SettingsCapabilitiesQueryRequest(),
                'resp': SettingsCapabilitiesQueryResponse(),
                'callback': self._provideCapabilitiesHandler
            }
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'msg': Settings,
                'topic': 'settings_status',
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
                'qsize': 1,
                'callback': self._updateSettingCb,
                'callback_args': None
            },
            'reset_setting': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'reset_settings',
                'qsize': 1,
                'callback': self._resetSettingsCb,
                'callback_args': None
            },
            'factory_reset_setting': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'factory_reset_settings',
                'qsize': 1,
                'callback': self._resetFactorySettingsCb,
                'callback_args': None
            },
        }


        # Create Node Class ####################
        self.node_if = NodeClassIF(params_dict = self.PARAMS_DICT,
                                    services_dict = self.SRVS_DICT,
                                    pubs_dict = self.PUBS_DICT,
                                    subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list
        )
   

        self.node_if.wait_for_ready()


        ##############################
        # Run Initialization Processes
        self.initialize_settings(do_updates = True)     
    
        nepi_ros.start_timer_process(1.0, self._publishSettingsCb)

  
        ##############################
        # Complete Initialization
        self.publish_status()
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
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def publish_status(self):
        current_settings = self.getSettingsFunction()
        self.node_if.set_param('settings', current_settings)
        settings_msg = nepi_settings.create_msg_data_from_settings(current_settings)
        if not nepi_ros.is_shutdown():
            #self.msg_if.pub_warn("Publishing settings status msg: " + str(settings_msg))
            self.node_if.publish_pub('status_pub', settings_msg)


    def update_setting(self,new_setting,update_status = True, update_param = True):
        success = False
        current_settings = self.getSettingsFunction()
        updated_settings = copy.deepcopy(current_settings)
        #self.msg_if.pub_warn("New Setting:" + str(new_setting))
        s_name = new_setting['name']
        if self.setSettingFunction != None:
            [name_match,type_match,value_match] = nepi_settings.compare_setting_in_settings(new_setting,current_settings)
            if value_match == False: # name_match would be true for value_match to be true
                self.msg_if.pub_debug("Will try to update setting " + str(new_setting))
                [success,msg] = nepi_settings.try_to_update_setting(new_setting,current_settings,self.cap_settings,self.setSettingFunction)
                self.msg_if.pub_warn(msg)
                if success:
                    if update_param:
                        updated_settings[s_name] = new_setting
                        self.node_if.set_param('settings', updated_settings)
                    if update_status:
                        self.publish_status() 
        else:
            self.msg_if.pub_debug("Settings updates ignored. No settings update function defined ", log_name_list = self.log_name_list)
        return success


    def initialize_settings(self, do_updates = True):
        current_settings = self.getSettingsFunction()
        self.init_settings = self.node_if.get_param('settings')
        #self.msg_if.pub_debug("Setting init values to param server values: " + str(self.init_settings) )
        if do_updates:
            self.reset_settings()


    def reset_settings(self, update_status = True):
        self.msg_if.pub_info("Applying Init Settings", log_name_list = self.log_name_list)
        #self.msg_if.pub_warn(self.init_settings)
        for setting_name in self.init_settings:
            setting = self.init_settings[setting_name]
            self.update_setting(setting, update_status = False, update_param = False)
        if update_status:
            self.publish_status()

    def factory_reset_settings(self, update_params = True, update_status = True):
        self.msg_if.pub_info("Applying Factory Settings", log_name_list = self.log_name_list)
        #self.msg_if.pub_warn(self.init_settings)
        for setting_name in self.factory_settings.keys():
            setting = self.factory_settings[setting_name]
            self.update_setting(setting,update_status = False, update_param = update_params)
        if update_status:
            self.publish_status()

    ###############################
    # Class Private Methods
    ###############################

    def _provideCapabilitiesHandler(self, req):
        return self.capabilities_response

    def _publishSettingsCb(self, timer):
        self.publish_status()


    def _updateSettingCb(self,msg):
        self.msg_if.pub_info("Received settings update msg: " + str(msg))
        #self.msg_if.pub_warn(msg)
        setting = nepi_settings.parse_setting_update_msg_data(msg)
        self.update_setting(setting, update_status = True, update_param = True)

    def _resetSettingsCb(self,msg):
        self.reset_settings()

    def _resetFactorySettingsCb(self,msg):
        self.factory_reset_settings()






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
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

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
            namespace = '~'
        self.namespace = nepi_ros.get_full_namespace(namespace)

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

        self.node_if.wait_for_ready()

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
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  



    ###############################
    # Class Private Methods
    ###############################

    def _statesQueryHandler(self, req):
        resp = SystemStatesQueryResponse()
        try:
            states_dict = self.get_states_dict_function()
            resp = nepi_states.create_query_resp(states_dict)
        except:
            self.msg_if.pub_warn("Failed to create resp msg: " + str(e))
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
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

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

        self.node_if.wait_for_ready()

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
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  


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
            self.msg_if.pub_warn("Failed to create resp msg: " + str(e))
        return resp


