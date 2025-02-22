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
# 1) NEPI IDX AI utility functions
import os
import rospy
import numpy as np
import math
import time
import sys
import cv2
import yaml

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_pc 
from nepi_sdk import nepi_img 
  
#***************************
# Misc file read write utilities

def read_yaml2dict(file_path):
    dict_from_file = dict()
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
        except:
            nepi_msg.publishMsgWarn(self,"Failed to get dict from file: " + file_path + " " + str(e))
    else:
        nepi_msg.publishMsgWarn(self,"Failed to find dict file: " + file_path)
    return dict_from_file

def write_dict2yaml(dict_2_save,file_path,defaultFlowStyle=False,sortKeys=False):
    success = False
    path = os.path.dirname(file_path)
    if os.path.exists(path):
        try:
            with open(file_path, "w") as f:
                yaml.dump(dict_2_save, stream=f, default_flow_style=defaultFlowStyle, sort_keys=sortKeys)
            success = True
        except:
            nepi_msg.publishMsgWarn(self,"Failed to write dict: " + str(dict_2_save) + " to file: " + file_path + " " + str(e))
    else:
        nepi_msg.publishMsgWarn(self,"Failed to find file path: " + path)
    return success
  
  
#***************************
# NEPI data saving utility functions

def save_data2file(self,data_product,data,ros_timestamp,device_name = '',save_check=True,add_text = ''):
    node_name = nepi_ros.get_node_name()
    if self.save_data_if is not None:
        saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
        should_save = self.save_data_if.data_product_should_save(data_product)
        snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
        # Save data if enabled
        if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
            if add_text != "":
                add_text = '-' + add_text 
            full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                            node_name + add_text + "-" + data_product, 'txt')
            if os.path.isfile(full_path_filename) is False:
                with open(full_path_filename, 'w') as f:
                    f.write(str(data))
                    self.save_data_if.data_product_snapshot_reset(data_product)

def save_dict2file(self,data_product,data_dict,ros_timestamp,save_check=True,add_text = ''):
    node_name = nepi_ros.get_node_name()
    if self.save_data_if is not None:
        saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
        should_save = self.save_data_if.data_product_should_save(data_product)
        snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
        # Save data if enabled
        if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
            if add_text != "":
                add_text = '-' + add_text 
            full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                              node_name + add_text + "-" + data_product, 'yaml')
            if os.path.isfile(full_path_filename) is False:
                with open(full_path_filename, 'w') as f:
                    yaml.dump(data_dict, f)
            self.save_data_if.data_product_snapshot_reset(data_product)


def save_img2file(self,data_product,cv2_img,ros_timestamp,save_check=True,add_text = ''):
    node_name = nepi_ros.get_node_name()
    if self.save_data_if is not None:
        saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
        should_save = self.save_data_if.data_product_should_save(data_product)
        snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
        # Save data if enabled
        if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
            if cv2_img is not None:
                if add_text != "":
                  add_text = '-' + add_text 
                full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                              node_name + add_text + "-" + data_product, 'png')
                if os.path.isfile(full_path_filename) is False:
                    cv2.imwrite(full_path_filename, cv2_img)
                    self.save_data_if.data_product_snapshot_reset(data_product)

def save_ros_img2file(self,data_product,ros_img,ros_timestamp,save_check=True,add_text = ''):
    node_name = nepi_ros.get_node_name()
    if self.save_data_if is not None:
        saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
        should_save = self.save_data_if.data_product_should_save(data_product)
        snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
        # Save data if enabled
        if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
            if ros_img is not None:
                cv2_img = nepi_img.rosimg_to_cv2img(ros_img)
                #nepi_msg.publishMsgWarn(self,'CV2 img size: ' + str(cv2_img.shape))
                if add_text != "":
                  add_text = '-' + add_text 
                full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                              node_name + add_text + "-" + data_product, 'png')
                if os.path.isfile(full_path_filename) is False:
                    cv2.imwrite(full_path_filename, cv2_img)
                    self.save_data_if.data_product_snapshot_reset(data_product)

            

def save_pc2file(self,data_product,o3d_pc,ros_timestamp,save_check=True,add_text = ''):
    node_name = nepi_ros.get_node_name()
    if self.save_data_if is not None:
        saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
        should_save = self.save_data_if.data_product_should_save(data_product)
        snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
        # Save data if enabled
        if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
            if o3d_pc is not None:
                if add_text != "":
                  add_text = '-' + add_text 
                full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                              node_name + add_text + "-" + data_product, 'pcd')
                if os.path.isfile(full_path_filename) is False:
                    nepi_pc.save_pointcloud(o3d_pc,full_path_filename)
                    self.save_data_if.data_product_snapshot_reset(data_product)

def save_ros_pc2file(self,data_product,ros_pc,ros_timestamp,save_check=True,add_text = ''):
    node_name = nepi_ros.get_node_name()
    if self.save_data_if is not None:
        saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
        should_save = self.save_data_if.data_product_should_save(data_product)
        snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
        # Save data if enabled
        if (saving_is_enabled and should_save) or snapshot_enabled or save_check == False:
            if ros_pc is not None:
                o3d_pc = nepi_pc.rospc_to_o3dpc(ros_pc, remove_nans=True)
                if add_text != "":
                  add_text = '-' + add_text 
                full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                              node_name + add_text + "-" + data_product, 'pcd')
                if os.path.isfile(full_path_filename) is False:
                    nepi_pc.save_pointcloud(o3d_pc,full_path_filename)
                    self.save_data_if.data_product_snapshot_reset(data_product)
