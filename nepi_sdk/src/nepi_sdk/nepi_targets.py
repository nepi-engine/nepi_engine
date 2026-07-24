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
log_name = "nepi_targets"
logger = Logger(log_name = log_name)


########################
## Library Data


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

        target_msg.timestamp = target_data_dict['timestamp']

        target_msg.name = target_data_dict['name']
        target_msg.uid = target_data_dict['uid']
        target_msg.confidence = target_data_dict['confidence']

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
        # navpose_dict = target_data_dict['navpose_dict']
        # if len(navpose_dict.keys()) == 0:
        #     navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
        #     target_msg.source_nav_pose = navpose_msg       
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


def get_boxes_list_from_msg(targets_msg):
    targets_list = targets_msg.targets
    boxes_list = []
    for target in detections_list:
        box_dict = {
            'name': target.name ,
            'id': target.id ,
            'uid': target.uid ,
            'prob': target.confidence ,
            'xmin': target.xmin_pixel ,
            'ymin': target.ymin_pixel ,
            'xmax': target.xmax_pixel ,
            'ymax': target.ymax_pixel ,
            'area_ratio': target.area_ratio ,
            'area_pixels': target.area_pixels
        }
        boxes_list.append(box_dict)
    return boxes_list