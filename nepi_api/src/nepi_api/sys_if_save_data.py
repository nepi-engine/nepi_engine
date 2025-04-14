#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import datetime
import time
import numpy as np
import cv2
import open3d as o3d

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_save

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_ros_interfaces.msg import SaveData, SaveDataRate, SaveDataStatus
from nepi_ros_interfaces.srv import DataProductQuery, DataProductQueryRequest, DataProductQueryResponse
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryRequest, SystemStorageFolderQueryResponse

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF

DATA_FOLDER_FALLBACK = '/mnt/nepi_storage/data'

SYSTEM_DATA_FOLDER = 'mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']


class SaveDataIF(object):

    ready = None
    namespace = "~"
 
    snapshot_dict = dict()

    

    ### IF Initialization
    def __init__(self, data_products = None, factory_rate_dict = None, namespace = None):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        log_name = self.class_name
        self.msg_if = MsgIF(log_name = log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ############################## 
        # Initialize Class Variables
        if namespace is None:
            namespace = '~'
        self.namespace = nepi_ros.get_full_namespace(namespace)
        
        self.msg_if.pub_info("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
        self.msg_if.pub_info("Starting Save_Data_IF with data products: " + str(data_products))
        # First thing, need to get the data folder
        self.save_data_root_directory = None # Flag it as non-existent
        get_folder_name_service = self.base_namespace + 'system_storage_folder_query'
        self.msg_if.pub_info("Waiting for system storage folder query service " + get_folder_name_service)
        try:
            self.msg_if.pub_info("Getting storage folder query service " + get_folder_name_service)
            folder_query_service = nepi_ros.connect_serivce(self,get_folder_name_service, SystemStorageFolderQuery)
            response = folder_query_service('data')
            self.msg_if.pub_info("Got storage folder path" + str(response))
            time.sleep(1)
            self.save_data_root_directory = response.folder_path
        except Exception as e:
            self.save_data_root_directory = DATA_FOLDER_FALLBACK
            self.msg_if.pub_info("Failed to obtain system data folder, falling back to: " + DATA_FOLDER_FALLBACK + " " + str(e))
            
        # Ensure the data folder exists with proper ownership
        if not os.path.exists(self.save_data_root_directory):
            self.msg_if.publish_warn("Reported data folder does not exist... data saving is disabled")
            self.save_data_root_directory = None # Flag it as non-existent
            return # Don't enable any of the ROS interface stuff
        self.save_path = self.save_data_root_directory

        # And figure out user/group so that we know what ownership to create subfolders with
        stat_info = os.stat(self.save_data_root_directory)
        self.DATA_UID = stat_info.st_uid
        self.DATA_GID = stat_info.st_gid

        save_rate_dict = dict()
        for data_product in data_products:
            set_rate = 0.0
            last_time = 0.0
            max_rate = 100,0
            save_rate_entry = [set_rate, last_time, max_rate]
            if factory_rate_dict is not None:
                if data_product in factory_rate_dict.keys():
                    save_rate = factory_rate_dict[data_product]
            save_rate_dict['data_product'] = save_rate
            self.snapshot_dict['data_product'] = False
            


        ##############################    
        # Node Setup



        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'save_rate_dict': {
                'namespace': self.namespace
                'factory_val': ave_rate_dict
            },
            'save_data': {
                'namespace': self.namespace
                'factory_val': False
            }
        }



        # Services Config Dict ####################
        self.SRVS_DICT = {
            'query_data_products': {
                'namespace': self.namespace,
                'topic': 'query_data_products',
                'svr': DataProductQuery,
                'req': DataProductQueryRequest(),
                'resp': DataProductQueryResponse(),
                'callback': self._dataProductQueryHandler
            }
        }


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
            }.
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
            }.
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
        self.node_if = NodeClassIF(self,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )


        self.node_if.wait_for_ready()

        ##############################
        # Complete Initialization
        self.publish_status()
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready    

    def register_data_product(self, data_product):
        save_rate_dict = nepi_ros.get_param('save_rate_dict')
        if data_product not in save_rate_dict.keys():
            save_rate_dict[data_product] = [1.0, 0.0, 100.0] # Default to 1Hz save rate, max rate = 100.0Hz
            self.node_if.get_param('save_rate_dict',save_rate_dict)
        self.publish_status()    def register_data_product(self, data_product):
        save_rate_dict = self.node_if.get_param('save_rate_dict')
        if data_product not in save_rate_dict.keys():
            save_rate_dict[data_product] = [1.0, 0.0, 100.0] # Default to 1Hz save rate, max rate = 100.0Hz
            self.node_if.get_param('save_rate_dict',save_rate_dict)
        self.publish_status()

    def set_save_prefix(self,prefix = ""):
        if '\\' not in prefix:
            if prefix.find('/') == -1:
                subfolder = ""set
                prefix = prefix
            else:
                prefix_split = prefix.rsplit('/',1)
                subfolder = prefix_split[0]
                prefix = prefix_split[1]
            # Now ensure the directory exists if this prefix defines a subdirectory
        else:
            subfolder = ""
            prefix = ""
        if subfolder != "" and self.save_data_root_directory != None:
            full_path = os.path.join(self.save_data_root_directory, subfolder)
        elif self.save_data_root_directory != None:
            full_path = self.save_data_root_directory
        else:
            full_path = ""
        #self.msg_if.publish_warn("DEBUG!!!! Computed full path " + full_path + " and parent path " + parent_path)
        if not os.path.exists(full_path):
            self.msg_if.pub_info("Creating new data subdirectory " + full_path)
            try:
                os.makedirs(full_path)
                os.chown(full_path, self.DATA_UID, self.DATA_GID)
                self.save_path = full_path
                self.save_data_subfolder  = subfolder
            except Exception as e:
                self.save_path = self.save_data_root_directory # revert to root folder
                self.save_data_subfolder  = ""
                self.msg_if.pub_info("Could not create save folder " + subfolder + str(e) )
        else:
            self.save_path = full_path
            self.save_data_subfolder  = subfolder
        self.prefix = prefix

        self.publish_status()



    def set_save_rate(self,data_product,save_rate_hz=0):
        save_all = SaveDataRate().ALL_DATA_PRODUCTS
        save_rate_dict = self.node_if.get_param('save_rate_dict')
        if (data_product == save_all):
            for d in save_rate_dict:
                # Respect the max save rate
                if save_rate_dict[d][0] > 0:
                    save_rate_dict[d][0] = save_rate_hz if save_rate_hz <= save_rate_dict[d][2] else save_rate_dict[d][2]
        elif (data_product in save_rate_dict):
            save_rate_dict[data_product][0] = save_rate_hz if save_rate_hz <= save_rate_dict[data_product][2] else save_rate_dict[data_product][2]
        else:
            self.msg_if.publish_warn("Requested unknown data product: " + data_product)           
        self.node_if.get_param('save_rate_dict',save_rate_dict)
        self.publish_status()
        

    def set_save_enable(self, save_data = False):
        self.save_data = save_data
        self.node_if.get_param('save_data', save_data)
        self.publish_status()       


    def data_product_saving_enabled(self, data_product):
        # If saving is disabled for this node, then no data products are saving
        try:
            save_rate_dict = self.node_if.get_param('save_rate_dict')
            if not self.save_data:
                return False

            if data_product not in save_rate_dict:
                self.msg_if.publish_warn("Unknown data product " + data_product)
                return False

            save_rate = save_rate_dict[data_product][0]
            return (save_rate > 0.0)
        except:
            return False

    def data_product_should_save(self, data_product):
        # If saving is disabled for this node, then it is not time to save this data product!
        save_rate_dict = self.node_if.get_param('save_rate_dict')
        if self.save_data == False:
            return False

        if data_product not in save_rate_dict:
            self.msg_if.publish_warn("Unknown data product " + data_product)
            return False

        save_rate = save_rate_dict[data_product][0]
        if save_rate == 0.0:
            return False

        save_period = 1.0 / save_rate
        now = nepi_ros.get_rostime_to_sec()
        elapsed = now - save_rate_dict[data_product][1]
        if (elapsed >= save_period):
            save_rate_dict[data_product][1] = now
            self.node_if.get_param('save_rate_dict',save_rate_dict)
            return True
        return False



    def data_product_snapshot_enabled(self, data_product):
        try:
            enabled = self.snapshot_dict[data_product]
            return enabled
        except:
            return False

    def data_product_snapshot_reset(self, data_product):
        try:
            self.snapshot_dict[data_product] = False
        except:
            pass
     



    def get_timestamp_string(self):
        return nepi_utils.get_datetime_str_now(add_ms = True, add_ns = False)

    def get_filename_path_and_prefix(self):
        if self.save_path is None:
            return ""
        return os.path.join(self.save_path, self.prefix)

    def get_full_path_filename(self, timestamp_string, identifier, extension):
        if self.save_path is None:
            return ""
        spacer = ""
        if self.prefix != "":
            if self.prefix[-1] != "_":
                spacer = "_"
        filename = os.path.join(self.save_path, self.prefix + spacer + timestamp_string + "_" + identifier + "." + extension)
        #self.msg_if.pub_info("******* save data filename: " + filename)
        return filename



    #***************************
    # NEPI data saving utility functions

    def save_data2file(self,data_product,data,ros_timestamp,device_name = '',save_check=True,add_text = '', node_name = None):
        if node_name == None:
            node_name = nepi_ros.get_node_name()
        if self.save_data_if is not None:
            saving_is_enabled = self.data_product_saving_enabled(data_product)
            should_save = self.data_product_should_save(data_product)
            snapshot_enabled = self.data_product_snapshot_enabled(data_product)
            # Save data if enabled
            if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
                if add_text != "":
                    add_text = '-' + add_text 
                full_path_filename = self.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                node_name + add_text + "-" + data_product, 'txt')
                if os.path.isfile(full_path_filename) is False:
                    with open(full_path_filename, 'w') as f:
                        f.write(str(data))
                        self.data_product_snapshot_reset(data_product)

    def save_dict2file(self,data_product,data_dict,ros_timestamp,save_check=True,add_text = '', node_name = None):
        if node_name == None:
            node_name = nepi_ros.get_node_name()
        if self.save_data_if is not None:
            saving_is_enabled = self.data_product_saving_enabled(data_product)
            should_save = self.data_product_should_save(data_product)
            snapshot_enabled = self.data_product_snapshot_enabled(data_product)
            # Save data if enabled
            if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
                if add_text != "":
                    add_text = '-' + add_text 
                full_path_filename = self.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                node_name + add_text + "-" + data_product, 'yaml')
                if os.path.isfile(full_path_filename) is False:
                    with open(full_path_filename, 'w') as f:
                        yaml.dump(data_dict, f)
                self.data_product_snapshot_reset(data_product)


    def save_img2file(self,data_product,cv2_img,ros_timestamp,save_check=True,add_text = '', node_name = None):
        if node_name == None:
            node_name = nepi_ros.get_node_name()
        if self.save_data_if is not None:
            saving_is_enabled = self.data_product_saving_enabled(data_product)
            should_save = self.data_product_should_save(data_product)
            snapshot_enabled = self.data_product_snapshot_enabled(data_product)
            # Save data if enabled
            if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
                if cv2_img is not None:
                    if add_text != "":
                    add_text = '-' + add_text 
                    full_path_filename = self.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                node_name + add_text + "-" + data_product, 'png')
                    if os.path.isfile(full_path_filename) is False:
                        cv2.imwrite(full_path_filename, cv2_img)
                        self.data_product_snapshot_reset(data_product)

    def save_ros_img2file(self,data_product,ros_img,ros_timestamp,save_check=True,add_text = '', node_name = None):
        if node_name == None:
            node_name = nepi_ros.get_node_name()
        if self.save_data_if is not None:
            saving_is_enabled = self.data_product_saving_enabled(data_product)
            should_save = self.data_product_should_save(data_product)
            snapshot_enabled = self.data_product_snapshot_enabled(data_product)
            # Save data if enabled
            if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
                if ros_img is not None:
                    cv2_img = nepi_img.rosimg_to_cv2img(ros_img)
                    #nepi_msg.publishMsgWarn(self,'CV2 img size: ' + str(cv2_img.shape))
                    if add_text != "":
                    add_text = '-' + add_text 
                    full_path_filename = self.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                node_name + add_text + "-" + data_product, 'png')
                    if os.path.isfile(full_path_filename) is False:
                        cv2.imwrite(full_path_filename, cv2_img)
                        self.data_product_snapshot_reset(data_product)

                

    def save_pc2file(self,data_product,o3d_pc,ros_timestamp,save_check=True,add_text = '', node_name = None):
        if node_name == None:
            node_name = nepi_ros.get_node_name()
        if self.save_data_if is not None:
            saving_is_enabled = self.data_product_saving_enabled(data_product)
            should_save = self.data_product_should_save(data_product)
            snapshot_enabled = self.data_product_snapshot_enabled(data_product)
            # Save data if enabled
            if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
                if o3d_pc is not None:
                    if add_text != "":
                    add_text = '-' + add_text 
                    full_path_filename = self.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                node_name + add_text + "-" + data_product, 'pcd')
                    if os.path.isfile(full_path_filename) is False:
                        nepi_pc.save_pointcloud(o3d_pc,full_path_filename)
                        self.data_product_snapshot_reset(data_product)

    def save_ros_pc2file(self,data_product,ros_pc,ros_timestamp,save_check=True,add_text = '', node_name = None):
        if node_name == None:
            node_name = nepi_ros.get_node_name()
        if self.save_data_if is not None:
            saving_is_enabled = self.data_product_saving_enabled(data_product)
            should_save = self.data_product_should_save(data_product)
            snapshot_enabled = self.data_product_snapshot_enabled(data_product)
            # Save data if enabled
            if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
                if ros_pc is not None:
                    o3d_pc = nepi_pc.rospc_to_o3dpc(ros_pc, remove_nans=True)
                    if add_text != "":
                    add_text = '-' + add_text 
                    full_path_filename = self.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                node_name + add_text + "-" + data_product, 'pcd')
                    if os.path.isfile(full_path_filename) is False:
                        nepi_pc.save_pointcloud(o3d_pc,full_path_filename)
                        self.data_product_snapshot_reset(data_product)






    def publish_status(self):
        save_rates_msg = []
        save_rate_dict = self.node_if.get_param('save_rate_dict')
        #self.msg_if.publish_warn("save_rate_dict " + str(save_rate_dict))
        for name in save_rate_dict.keys():
            save_rate_msg = SaveDataRate()
            rate = round(save_rate_dict[name][0])
            save_rate_msg.data_product = name
            save_rate_msg.save_rate_hz = rate
            save_rates_msg.append(save_rate_msg)
            #self.msg_if.publish_warn("data_rates_msg " + str(save_rates_msg))
        current_save_data = SaveData(save_data = self.save_data)  
        status_msg = SaveDataStatus()
        status_msg.current_data_dir = ""
        status_msg.current_filename_prefix = self.prefix
        status_msg.save_data_rates = save_rates_msg
        status_msg.save_data = current_save_data

        if self.save_data_root_directory is not None:
            status_msg.current_data_dir = self.save_data_root_directory
        self.node_if.publish_pub('status_pub', status_msg)


    ###############################
    # Class Private Methods
    ###############################

    def _dataProductQueryHandler(self, req):
        return_list = []
        save_rate_dict = self.node_if.get_param('save_rate_dict')
        for d in save_rate_dict:
            return_list.append(SaveDataRate(data_product = d, save_rate_hz = save_rate_dict[d][0]))

        return DataProductQueryResponse(return_list)


    def _setPrefixCb(self, msg):
        new_prefix = msg.data
        self.set_save_prefix(new_prefix)


    def _saveRateCb(self, msg):
        data_product = msg.data_product
        save_rate_hz = msg.save_rate_hz
        self.set_save_rate(data_product,save_rate_hz)



    def _saveEnableCb(self, msg):
        save_data = msg.data
        self.set_save_enable(save_data)
        

    def _snapshotCb(self,msg):
        self.msg_if.pub_info("Recieved Snapshot Trigger")
        save_rate_dict = self.node_if.get_param('save_rate_dict')
        for data_product in save_rate_dict.keys():
            save_rate = save_rate_dict[data_product][0]
            enabled = (save_rate > 0.0)
            if enabled:
                self.snapshot_dict[data_product] = True


    def _resetCb(self,reset_msg):
        self.msg_if.pub_info("Recieved save data reset msg")
        self.node_if.reset_params()
        self.publish_status()

    def _factroyResetCb(self,reset_msg):
        self.msg_if.pub_info("Recieved save data factory reset msg")
        self.node_if.factory_reset_params()
        self.publish_status()



