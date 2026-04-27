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
#from nepi_sdk import nepi_targets

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header

from nepi_interfaces.msg import NavPose, ImageStatus, Localization
from nepi_interfaces.msg import StringArray, BoundingBox, AiBoundingBoxes
from nepi_interfaces.msg import Target, Targets, TargetingStatus

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_ais"
logger = Logger(log_name = log_name)


########################
## Library Data


BEST_FILTER_OPTIONS = ['SMALLEST','LARGEST','PROBABILITY']

BLANK_TRACKING_DICT = {
    'available_process_topics': [],
    'selected_process_topic': '',

    'available_source_topics': [],
    'selected_source_ordered': [],

    'available_classes': [],
    'selected_classes_ordered': [],

    'threshold_filter': 0.1,

    'available_best_filters': [],
    'selected_best_filter': 'LARGEST'
}


########################
## Misc AI Helper Functions


def convert_tacking_dict2msg(track_data_dict, log_name_list = []):
  track_msg = None
  if track_data_dict is None:
    logger.log_info("Got None track dict", throttle_s = 5.0)
  else:
    try:
        track_msg = Target()
        track_msg.track_name = track_data_dict['track_name']
        track_msg.track_uid = track_data_dict['track_uid']
        track_msg.track_confidence = track_data_dict['track_confidence']

        # 2D Data ENU Reference Frame
        track_msg.xmin_pixel = track_data_dict['xmin_pixel']
        track_msg.xmax_pixel = track_data_dict['xmax_pixel']

        track_msg.ymin_pixel = track_data_dict['ymin_pixel']
        track_msg.ymax_pixel = track_data_dict['ymax_pixel']

        track_msg.width_pixels = track_data_dict['width_pixels']
        track_msg.height_pixels = track_data_dict['height_pixels']

      
        track_msg.area_pixels = track_data_dict['area_pixels']
        track_msg.area_ratio = track_data_dict['area_ratio']
        #track_msg.vel_pixels

        # 3D Data in ENU Reference Frame
        # track_msg.width_meters = track_data_dict['width_meters']
        # track_msg.height_meters = track_data_dict['height_meters']
        # track_msg.depth_meters = track_data_dict['depth_meters']
        # track_msg.area_meters = track_data_dict['area_meters']

        #track_msg.center_xyz_meters = track_data_dict['center_xyz_meters']

        # Range, Bearing, Nav, and Pose Data ENU Reference Frame
        track_msg.range_m = track_data_dict['range_m']
        track_msg.azimuth_deg = track_data_dict['azimuth_deg']
        track_msg.elevation_deg = track_data_dict['elevation_deg']
        navpose_dict = track_data_dict['navpose_dict']
        if len(navpose_dict.keys()) == 0:
            navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
            track_msg.source_nav_pose = navpose_msg       
    except Exception as e:
      track_msg = None
      logger.log_warn("Failed to convert Target Data dict: " + str(e), throttle_s = 5.0)
  return track_msg

def convert_track_msg2dict(track_msg, log_name_list = []):
  track_data_dict = None
  try:
    track_data_dict = nepi_sdk.convert_msg2dict(track_msg)
  except Exception as e:
    logger.log_warn("Failed to convert Target Data msg: " + str(e), throttle_s = 5.0)
  return track_data_dict





            