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
from nepi_sdk import nepi_targets

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header

from nepi_interfaces.msg import NavPose, ImageStatus, Localization
from nepi_interfaces.msg import StringArray, BoundingBox, AiBoundingBoxes
from nepi_interfaces.msg import Target, Targets, TargetingStatus

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_ais"
logger = Logger(log_name = log_name)


########################
## Library Data


BEST_FILTER_OPTIONS = nepi_targets.BEST_FILTER_OPTIONS

BLANK_TRACKING_DICT = {
    'targets_topic': 'None',
    'source_topic': 'None',
    'class_filter': 'None',
    'size_min_filter': 0.1,
    'size_max_filter': 0.99,
    'range_min_filter': 0.01,
    'range_max_filter': 1000,
    'threshold_filter': 0.1,
    'best_filter': 'LARGEST'
}


########################
## Misc Tracking Helper Functions

def get_track_source_namespaces( topics_list = None, types_list = None):
    namespace = []
    msg_type = 'nepi_interfaces/TargetingStatus'
    namespaces = nepi_sdk.find_topics_by_msg(msg_type, topics_list = topics_list, types_list = types_list)
    for i, namespace in enumerate(namespaces):
        namespaces[i] = os.path.dirname(namespaces[i])
    return namespaces 

'''
def get_classes_colors_list(classes_str_list):
    rgb_list = []
    if len(classes_str_list) > 0:
        cmap = plt.get_cmap('viridis')
        color_list = cmap(np.linspace(0, 1, len(classes_str_list))).tolist()
        for color in color_list:
            rgb = []
            for i in range(3):
                rgb.append(int(color[i]*255))
            rgb_list.append(rgb)
    return rgb_list
'''


# def convert_tacking_dict2msg(track_data_dict, log_name_list = []):
#   track_msg = None
#   if track_data_dict is None:
#     logger.log_info("Got None track dict", throttle_s = 5.0)
#   else:
#     try:
#         track_msg = Target()
#         track_msg.track_name = track_data_dict['track_name']
#         track_msg.track_uid = track_data_dict['track_uid']
#         track_msg.track_confidence = track_data_dict['track_confidence']

#         # 2D Data ENU Reference Frame
#         track_msg.xmin_pixel = track_data_dict['xmin_pixel']
#         track_msg.xmax_pixel = track_data_dict['xmax_pixel']

#         track_msg.ymin_pixel = track_data_dict['ymin_pixel']
#         track_msg.ymax_pixel = track_data_dict['ymax_pixel']

#         track_msg.width_pixels = track_data_dict['width_pixels']
#         track_msg.height_pixels = track_data_dict['height_pixels']

      
#         track_msg.area_pixels = track_data_dict['area_pixels']
#         track_msg.area_ratio = track_data_dict['area_ratio']
#         #track_msg.vel_pixels

#         # 3D Data in ENU Reference Frame
#         # track_msg.width_meters = track_data_dict['width_meters']
#         # track_msg.height_meters = track_data_dict['height_meters']
#         # track_msg.depth_meters = track_data_dict['depth_meters']
#         # track_msg.area_meters = track_data_dict['area_meters']

#         #track_msg.center_xyz_meters = track_data_dict['center_xyz_meters']

#         # Range, Bearing, Nav, and Pose Data ENU Reference Frame
#         track_msg.range_m = track_data_dict['range_m']
#         track_msg.azimuth_deg = track_data_dict['azimuth_deg']
#         track_msg.elevation_deg = track_data_dict['elevation_deg']
#         navpose_dict = track_data_dict['navpose_dict']
#         if len(navpose_dict.keys()) == 0:
#             navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
#             track_msg.source_nav_pose = navpose_msg       
#     except Exception as e:
#       track_msg = None
#       logger.log_warn("Failed to convert Target Data dict: " + str(e), throttle_s = 5.0)
#   return track_msg

def convert_track_msg2dict(track_msg, log_name_list = []):
  track_data_dict = None
  try:
    track_data_dict = nepi_sdk.convert_msg2dict(track_msg)
  except Exception as e:
    logger.log_warn("Failed to convert Target Data msg: " + str(e), throttle_s = 5.0)
  return track_data_dict



def get_best_from_targets(targets_dict_list,tracking_dict = BLANK_TRACKING_DICT):
   filtered_targets = targets_dict_list
   best_target = None
   for entry in BLANK_TRACKING_DICT.keys():
    if entry not in tracking_dict.keys():
       tracking_dict[entry] = BLANK_TRACKING_DICT[entry]
    
    class_filters = tracking_dict['class_filter']
    filtered_targets = nepi_targets.filter_by_classes(filtered_targets, [class_filters])

    size_max_filter = tracking_dict['size_max_filter']
    size_min_filter = tracking_dict['size_min_filter']
    filtered_targets = nepi_targets.filter_by_area(filtered_targets, size_min_filter = size_min_filter, size_max_filter = size_max_filter)

    threshold_filter = tracking_dict['threshold_filter']
    filtered_targets = nepi_targets.filter_by_threshold(filtered_targets, threshold_filter)
    
    if len(filtered_targets) > 0:
      best_filter = tracking_dict['best_filter']
      best_target = nepi_targets.find_best(filtered_targets, best_filter = best_filter)

            
    return best_target,tracking_dict    

            