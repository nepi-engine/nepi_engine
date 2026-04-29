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
import numpy as np
import sys
import declxml as xml
import xml.etree.ElementTree as ET

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header

from nepi_interfaces.msg import StringArray, ObjectCount, BoundingBox, BoundingBoxes, AiBoundingBoxes

from nepi_interfaces.msg import Target, Targets, TargetingStatus

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_ais"
logger = Logger(log_name = log_name)


########################
## Library Data
BEST_FILTER_OPTIONS = ['SMALLEST','LARGEST','PROBABILITY']

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

Target navpose

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


BLANK_TARGET_SETTINGS_DICT = {
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




def convert_target_dict2msg(target_data_dict, log_name_list = []):
  target_msg = None
  if target_data_dict is None:
    logger.log_info("Got None target dict", throttle_s = 5.0)
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
      logger.log_warn("Failed to convert Target Data dict: " + str(e), throttle_s = 5.0)
  return target_msg

def convert_target_msg2dict(target_msg, log_name_list = []):
  target_data_dict = None
  try:
    target_data_dict = nepi_sdk.convert_msg2dict(target_msg)
  except Exception as e:
    logger.log_warn("Failed to convert Target Data msg: " + str(e), throttle_s = 5.0)
  return target_data_dict


def filter_by_classes(targets_dict_list, class_filter_list):
    #print(targets_dict_list)

    filtered_targets = []
    for name in class_filter_list:
        for target_dict in targets_dict_list:
            if target_dict['target_name'] == name:
                filtered_targets.append(target_dict)
                #logger.log_info("Added target with name: " + str(name))

    # for target_dict in filtered_targets:   
    #     logger.log_info("Returning target with name: " + str(name))
    return filtered_targets
    


def filter_by_area(targets_dict_list, size_min_filter = .01, size_max_filter = .99):
    #print(targets_dict_list)

    filtered_targets = []

    for target_dict in targets_dict_list:
        target_area = target_dict['area_ratio']
        if target_area >= size_min_filter and target_area <= size_max_filter:
            filtered_targets.append(target_dict)
    #logger.log_info("Got Area filtered_targets: " + str(filtered_targets))
    return filtered_targets



# def filter_by_range(self,targets_dict_list, size_min_filter = .01, size_max_filter = .99):
#     ################
#     # Filter by min max range and angles
#     filtered_dict_list = []
#     cur_position = copy.deepcopy(self.current_position)
#     if cur_position is not None:
#       [cur_pan,cur_tilt] = [cur_position[0],cur_position[1]]
#       range_min = self.track_range_min_m
#       range_max = self.track_range_max_m
#       pan_min = self.track_pan_min_deg
#       pan_max = self.track_pan_max_deg
#       tilt_min = self.track_tilt_min_deg
#       tilt_max = self.track_tilt_max_deg

#       for target_dict in targets_dict_list:
#           target_valid = True
#           range_m = target_dict['range_m']
#           if (range_m < range_min or range_m > range_max) and range_m != -999:
#             target_valid = False
#           target_pan_angle = target_dict['azimuth_deg']
#           pan_angle =  cur_pan + target_pan_angle
#           if (pan_angle < pan_min or pan_angle > pan_max) and target_pan_angle != -999:
#             target_valid = False
#           target_tilt_angle = cur_pan + target_dict['elevation_deg']
#           tilt_angle =  cur_tilt + target_tilt_angle
#           if (tilt_angle < tilt_min or tilt_angle > tilt_max) and target_tilt_angle != -999:
#             target_valid = False
#           if target_valid == True:
#             filtered_dict_list.append(target_dict)
#           #self.msg_if.pub_warn("Range Angle Filter returned: " + str(target_dict['target_name']) + " : " + str(target_valid) )
#           #self.msg_if.pub_warn(str([range_m,cur_pan,cur_tilt]))
#           #self.msg_if.pub_warn(str([range_m,target_pan_angle,target_tilt_angle]))
#           #self.msg_if.pub_warn(str([range_m,pan_angle,tilt_angle]))
#     return filtered_dict_list

# def filter_by_bearings(self,targets_dict_list):
#     ################
#     # Filter by min max range and angles
#     filtered_dict_list = []
#     cur_position = copy.deepcopy(self.current_position)
#     if cur_position is not None:
#       [cur_pan,cur_tilt] = [cur_position[0],cur_position[1]]
#       range_min = self.track_range_min_m
#       range_max = self.track_range_max_m
#       pan_min = self.track_pan_min_deg
#       pan_max = self.track_pan_max_deg
#       tilt_min = self.track_tilt_min_deg
#       tilt_max = self.track_tilt_max_deg

#       for target_dict in targets_dict_list:
#           target_valid = True
#           range_m = target_dict['range_m']
#           if (range_m < range_min or range_m > range_max) and range_m != -999:
#             target_valid = False
#           target_pan_angle = target_dict['azimuth_deg']
#           pan_angle =  cur_pan + target_pan_angle
#           if (pan_angle < pan_min or pan_angle > pan_max) and target_pan_angle != -999:
#             target_valid = False
#           target_tilt_angle = cur_pan + target_dict['elevation_deg']
#           tilt_angle =  cur_tilt + target_tilt_angle
#           if (tilt_angle < tilt_min or tilt_angle > tilt_max) and target_tilt_angle != -999:
#             target_valid = False
#           if target_valid == True:
#             filtered_dict_list.append(target_dict)
#           #self.msg_if.pub_warn("Range Angle Filter returned: " + str(target_dict['target_name']) + " : " + str(target_valid) )
#           #self.msg_if.pub_warn(str([range_m,cur_pan,cur_tilt]))
#           #self.msg_if.pub_warn(str([range_m,target_pan_angle,target_tilt_angle]))
#           #self.msg_if.pub_warn(str([range_m,pan_angle,tilt_angle]))
#     return filtered_dict_list


def filter_by_threshold(targets_dict_list, threshold_filter):
    #print(targets_dict_list)

    filtered_targets = []

    for target_dict in targets_dict_list:
        prob = target_dict['probability']
        if prob >= threshold_filter:
            filtered_targets.append(target_dict)
    #logger.log_info("Got Area filtered_targets: " + str(filtered_targets))
    return filtered_targets

def find_best(targets_dict_list, best_filter = 'LARGEST'):
    #print(tracks_dict_list)
    best_target = None
    for target_dict in targets_dict_list:
        
        best = True

        if best_target is not None:
            bsize = best_target['area_ratio']
            tsize = target_dict['area_ratio']
            bprob = best_target['probability']
            tprob = target_dict['probability']
            if best_filter == 'LARGEST' and tsize < bsize:
                best = False
            elif best_filter == 'SMALLEST' and tsize > bsize:
                best = False
            elif best_filter == 'PROPABILITY' and tprob < bprob:
                best = False

        if best == True:
            best_target = target_dict
    #logger.log_info("Got filtered_dict " + str(filtered_track))

            
    return best_target
            