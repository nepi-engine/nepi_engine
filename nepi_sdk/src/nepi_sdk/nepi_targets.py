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
import numpy as np
import sys
import declxml as xml
import xml.etree.ElementTree as ET

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header

from nepi_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes, AiBoundingBoxes
from nepi_interfaces.msg import Target, Targets, TargetFilter, TargetFilters, TargetingStatus
from nepi_interfaces.msg import Target

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_ais"
logger = Logger(log_name = log_name)


########################
## Library Data

TARGET_SOURCE_MSG_LIST = [ "BoundingBoxes", "AiBoundingBoxes", "Targets", "Image", "NAVPOSE" ]

TARGET_FILTER_OPTIONS = ['SMALLEST','LARGEST']

BLANK_TARGETING_STATUS_DICT = {
'''
string name
string namespace
bool enabled
string state
bool targeting_state

float32 max_proc_rate_hz
float32 max_img_rate_hz

string[] available_targets
string[] selected_targets

string[] available_filters
string[] ordered_filters
string[] enabled_filters
string selected_filter

bool pub_image_enabled

bool overlay_label
bool overlay_range_bearing
bool overlay_nav
bool overlay_pose
bool overlay_clf_name
bool overlay_img_name

##################
string[] target_source_topics
string[] selected_source_topics
string[] selected_img_topics
string[] selected_depth_map_topics
string[] selected_pointcloud_topics
string[] selected_pub_sub_namespaces


###############
float32 avg_image_receive_rate

float32 avg_image_process_time

float32 avg_image_process_latency
float32 avg_image_process_rate
    
float32 avg_targeting_process_time
    
float32 avg_targeting_process_latency
float32 avg_targeting_process_rate

float32 max_targeting_rate
'''

}


BLANK_TARGET_DICT = {
'''
string target_name
string target_uid
float32 target_confidence

# 2D Data ENU Reference Frame
int32 xmin_pixel
int32 xmax_pixel

int32 ymin_pixel
int32 ymax_pixel

int32 width_pixels
int32 height_pixels

float32 area_ratio 
float32 area_pixels
geometry_msgs/Vector3 vel_pixels

# 3D Data in ENU Reference Frame
float32 width_meters
float32 height_meters
float32 depth_meters
float32 area_meters
float32 volume_meters
geometry_msgs/Vector3 vel_xyz_mps
geometry_msgs/Vector3 center_xyz_meters

# Range, Bearing, Nav, and Pose Data ENU Reference Frame
float32 range_m
float32 azimuth_deg
float32 elevation_deg

Target source_nav_pose

# Color Distribution Percent 
float32 color_black	
float32 color_white	
float32 color_red	
float32 color_blue	
float32 color_yellow	
float32 color_cyan	
float32 color_magenta	
float32 color_green	

# Contour Data
int32[] contour_moments
}


BLANK_TARGETS_DICT = {
string name
float64 targeting_timestamp

string source_topic
string source_type
string source_description
float64 source_timestamp
Target source_nav_pose

bool has_2d_data
bool has_3d_data
bool has_range_data
bool has_bearing_data
bool has_nav_data
bool has_pose_data
bool has_color_data
bool has_countour_data
bool has_shape_data

Target[] targets
'''
}

BLANK_TARGET_FILTER_DICT = {

}


BLANK_TARGET_FILTERS_DICT = {

}

########################
## Misc AI Helper Functions
def get_targeting_source_publisher_namespaces():
    namespace = []
    msg_type = 'nepi_interfaces/TargetingStatus'
    namespaces = nepi_sdk.find_topics_by_msg(msg_type)
    for i, namespace in enumerate(namespaces):
        namespaces[i] = os.path.dirname(namespaces[i])
    return namespaces 


def filter_targets_list(targets_list, ordered_targets_list = [], target_filter = 'LARGEST'):
   filtered_target = None
   for i, target in enumerate(targets_list):
        size_dict = dict()
        target_dict = dict()
        sorted_dict = dict()
        center_dict = dict()
        best_dict = dict()
        best_target = None
        for target_name in self.classes:
            size_dict[target_name] = []
            target_dict[target_name] = []
            sorted_dict[target_name] = []
            center_dict[target_name] = []
            best_dict[target_name] = None
        for target in target_dict_list:
            target_name = target['target_name']
            target_size = target['area_ratio']
            size_dict[target_name].append(target_size)
            target_dict[target_name].append(target)
        for target_name in size_dict.keys():
            size_list = size_dict[target_name]
            list_tmp = size_list.copy()
            list_sorted = size_list.copy()
            list_sorted.sort()

            list_index = []
            for x in list_sorted:
                list_index.insert(0,list_tmp.index(x))
                list_tmp[list_tmp.index(x)] = -1
            for ind in list_index:
                sorted_dict[target_name].append(target_dict[target_name][ind])
        for target_name in sorted_dict.keys():
            for target in sorted_dict[target_name]:
                tsize = target['area_ratio']
                best = True
                btarget = best_dict[target_name]
                if btarget is not None:
                    bsize = btarget['area_ratio']
                    if target_filter == 'LARGEST' and tsize < bsize:
                        best = False
                    elif target_filter == 'SMALLEST' and tsize > bsize:
                       best = False
                if best == True:
                    best_dict[target_name] = target
        for target_name in best_dict.keys():
            if len(ordered_targets_list) > 0:
                if target_name in ordered_targets_list:
                    tindex = ordered_targets_list.index(target_name)
                    if best_target is None:
                        best_target = best_dict[target_name]
                    else:
                        btarget_name = best_target['target_name']
                        if btarget_name in ordered_targets_list:
                            bindex = ordered_targets_list.index(btarget_name)
                            if tindex < bindex:
                                best_target = best_dict[target_name]
            else:
                if best_target is None:
                    best_target = best_dict[target_name]
                else:
                    tsize = best_dict[target_name]['area_ratio']
                    bsize = best_dict[target_name]['area_ratio']
                    if target_filter == 'LARGEST' and tsize > bsize:
                        best_dict[target_name]
                    elif target_filter == 'SMALLEST' and tsize < bsize:
                        best_dict[target_name]
                
        return best_target
            

   



def convert_target_dict2msg(target_data_dict, log_name_list = []):
  target_msg = None
  if target_data_dict is None:
    logger.log_info("Got None target dict", throttle_s = 5.0, log_name_list = log_name_list)
  else:
    try:
        target_msg = Target()
        target_msg.target_name = target_data_dict['target_name']
        target_msg.target_uid = target_data_dict['target_uid']
        target_msg.target_confidence = target_data_dict['target_confidence']

        # 2D Data ENU Reference Frame
        target_msg.xmin_pixel = target_data_dict['xmin_pixel']
        target_msg.xmax_pixel = target_data_dict['xmax_pixel']

        target_msg.ymin_pixel = target_data_dict['ymin_pixel']
        target_msg.ymax_pixel = target_data_dict['ymax_pixel']

        target_msg.width_pixels = target_data_dict['width_pixels']
        target_msg.height_pixels = target_data_dict['height_pixels']

      
        target_msg.area_pixels = target_data_dict['area_pixels']
        target_msg.area_ratio = target_data_dict['area_ratio']
        #target_msg.vel_pixels

        # 3D Data in ENU Reference Frame
        # target_msg.width_meters = target_data_dict['width_meters']
        # target_msg.height_meters = target_data_dict['height_meters']
        # target_msg.depth_meters = target_data_dict['depth_meters']
        # target_msg.area_meters = target_data_dict['area_meters']

        #target_msg.center_xyz_meters = target_data_dict['center_xyz_meters']

        # Range, Bearing, Nav, and Pose Data ENU Reference Frame
        target_msg.range_m = target_data_dict['range_m']
        target_msg.azimuth_deg = target_data_dict['azimuth_deg']
        target_msg.elevation_deg = target_data_dict['elevation_deg']
        navpose_dict = target_data_dict['navpose_dict']
        if len(navpose_dict.keys()) == 0:
            navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
            target_msg.source_nav_pose = navpose_msg       
    except Exception as e:
      target_msg = None
      self.msg_if.pub_warn("Failed to convert Target Data dict: " + str(e), throttle_s = 5.0, log_name_list = log_name_list)
  return target_msg

def convert_target_msg2dict(target_msg, log_name_list = []):
  target_data_dict = None
  try:
    target_data_dict = nepi_sdk.convert_msg2dict(target_msg)
  except Exception as e:
    self.msg_if.pub_warn("Failed to convert Target Data msg: " + str(e), throttle_s = 5.0, log_name_list = log_name_list)
  return target_data_dict