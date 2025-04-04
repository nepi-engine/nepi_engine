#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import os
import yaml


from nepi_sdk import nepi_ros

from nepi_sdk.nepi_ros import logger as Logger
log_name = "nepi_save"
logger = Logger(log_name = log_name)
  
#########################
### Save Data Helper Functions

def get_save_data_subscriber_namespaces(self):
    topics_list = nepi_ros.find_topics_by_name('save_data')
    namespaces_list = []
    for topic in topics_list:
        namespaces_list.append(os.path.dirname(topic))
    return namespaces_list



#***************************
# Misc File Read Write Functions

def read_yaml_2_dict(file_path):
    return read_yaml2dict(file_path)

def read_yaml2dict(file_path):
    dict_from_file = dict()
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
        except:
            logger.log_info("Failed to get dict from file: " + file_path + " " + str(e))
    else:
        logger.log_info("Failed to find dict file: " + file_path)
    return dict_from_file


def write_dict_2_yaml(dict_2_save, file_path):
    return write_dict2yaml(dict_2_save,file_path)

def write_dict2yaml(dict_2_save,file_path,defaultFlowStyle=False,sortKeys=False):
    success = False
    try:
        with open(file_path, "w") as f:
            yaml.dump(dict_2_save, stream=f, default_flow_style=defaultFlowStyle, sort_keys=sortKeys)
        success = True
    except:
        logger.log_info("Failed to write dict: " + str(dict_2_save) + " to file: " + file_path + " " + str(e))
    return success
  
  

#***************************
# NEPI data saving utility functions

def save_data2file(self,data_product,data,ros_timestamp,device_name = '',save_check=True,add_text = '', node_name = None):
    if node_name == None:
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

def save_dict2file(self,data_product,data_dict,ros_timestamp,save_check=True,add_text = '', node_name = None):
    if node_name == None:
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


def save_img2file(self,data_product,cv2_img,ros_timestamp,save_check=True,add_text = '', node_name = None):
    if node_name == None:
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

def save_ros_img2file(self,data_product,ros_img,ros_timestamp,save_check=True,add_text = '', node_name = None):
    if node_name == None:
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

            

def save_pc2file(self,data_product,o3d_pc,ros_timestamp,save_check=True,add_text = '', node_name = None):
    if node_name == None:
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

def save_ros_pc2file(self,data_product,ros_pc,ros_timestamp,save_check=True,add_text = '', node_name = None):
    if node_name == None:
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
