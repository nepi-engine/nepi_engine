#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#



import rospy
import os
import time
import numpy as np
import cv2
import open3d as o3d

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_save

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header

SYSTEM_DATA_FOLDER = 'mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']


class SaveIF:

    # Save data variables
    filename_dict = {
    'prefix': "", 
    'add_timestamp': True, 
    'add_ms': True,
    'add_ns': False,
    'suffix': ""
    }




    #######################
    ### IF Initialization
    log_name = "ConnectImageIF"
    def __init__(self):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting Initialization Processes")
        ##############################  
        
        self.data_dict = {
            'dict': {
                'data_type': dict,
                'file_types': ['yaml'],
                'read_func': self.read_yaml_2_dict,
                'write_func': self.write_dict_2_yaml
            },
            'image': {
                'data_type': np.ndarray,
                'file_types': ['png','PNG','jpg','jpeg','JPG'],
                'read_func': self.read_img_2_cv2img,
                'write_func': self.write_cv2img_2_img 
            },
            'pointcloud': {
                'data_type': o3d.geometry.PointCloud,
                'file_types': ['pcd'],
                'read_func': self.read_pcb_2_o3dp,
                'write_func': self.write_o3dpc_2_pcd 
            },
        }

        #################################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Initialization Complete")

    

    #######################
    # Class Public Methods
    #######################

    def get_supported_data_dict(self):
        return self.data_dict

    def get_data_folder(self):
        return self.save_folder

    def set_data_folder(self, data_path):
        success = True
        if os.path.exists(data_path) == False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File path does not exist: " + data_path)
        elif os.path.isdir(data_path) == False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File path not a folder: " + data_path)
        else:
            self.save_folder = data_path
            success = True
        return success

    def reset_data_folder(self):
        self.save_folder = SYSTEM_DATA_FOLDER

    def set_filename_config(self, prefix = '', suffix = '', add_time = True, add_ms = True, add_ns = False):
        self.filename_dict = {
        'prefix': prefix, 
        'add_timestamp': add_timestamp, 
        'add_ms': add_ms,
        'add_ns': add_ns,
        'suffix': suffix
        }

    def get_filename_config(self):
        return self.filename_dict


    def get_folder_files(self, ext_str = ""):
        file_list = nepi_utils.get_file_list(self.data_folder,ext_str=ext_str)
        return file_list


    def get_file_times(self,file_list):
        file_times = []
        for file in file_list:
            file_times.append(nepi_save.get_time_from_filename(file))
        return file_times

    def read_yaml_file(filename):
        data_key = 'dict'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(self.data_folder,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def save_dict_file(data, data_name, timestamp = None, ext_str = 'yaml'):
        data_key = 'dict'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Data type not supported: " + str(data_type))
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File type not supported: " + ext_str + " : " + str(file_types))
            else:
                filename = self._createFileName(data_name,ext_str,timestamp)
                file_path = os.path.join(self.data_folder,filename)
                if os.path.exists(file_path) == True:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success


    def read_image_file(filename):
        data_key = 'image'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(self.data_folder,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def save_image_file(data, data_name, timestamp = None, ext_str = 'png'):
        data_key = 'image'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Data type not supported: " + str(data_type))
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File type not supported: " + ext_str + " : " + str(file_types))
            else:
                filename = self._createFileName(data_name,ext_str,timestamp)
                file_path = os.path.join(self.data_folder,filename)
                if os.path.exists(file_path) == True:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success


    def read_pointcloud_file(filename):
        data_key = 'pointcloud'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(self.data_folder,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def save_pointcloud_file(data, data_name, timestamp = None, ext_str = 'pcd'):
        data_key = 'pointcloud'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Data type not supported: " + str(data_type))
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File type not supported: " + ext_str + " : " + str(file_types))
            else:
                filename = self._createFileName(data_name,ext_str,timestamp)
                file_path = os.path.join(self.data_folder,filename)
                if os.path.exists(file_path) == True:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success



    ###############################
    # Class Private Methods
    ###############################

    def _createFileName(self,data_name_str,ext_str,timestamp):
        prefix = self.filename_dict['prefix']
        add_time = self.filename_dict['add_timestamp']
        add_ms = self.filename_dict['add_ms']
        add_ns = self.filename_dict['add_ns']
        suffix = self.filename_dict['suffix']
        filename = nepi_save.create_filename(data_name_str, ext_str, prefix = prefix, suffix = suffix, add_time = add_time, timestamp = timestamp, add_ms = add_ms, add_ns = add_ns)
        return filename