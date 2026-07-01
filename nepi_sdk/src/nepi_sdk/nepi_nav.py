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


# NEPI navigation utility functions include
# 1) NavPose request functions
# 2) NavPose conversion utility functions

import os
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import time
import tf
from pygeodesy.ellipsoidalKarney import LatLon
import copy

from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry

from nepi_interfaces.msg import NavPoseTrack

from nepi_interfaces.msg import NavPose
from nepi_interfaces.msg import NavPoseLocation, NavPoseHeading
from nepi_interfaces.msg import NavPoseOrientation, NavPosePosition
from nepi_interfaces.msg import NavPoseAltitude, NavPoseDepth
from nepi_interfaces.msg import NavPosePanTilt

from nepi_interfaces.msg import Transform
from nepi_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, NavPoseQueryResponse

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils


from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_nav"
logger = Logger(log_name = log_name)


# try and import geoid height calculation module and databases
GEOID_DATABASE_FILE='/mnt/nepi_storage/databases/geoids/egm2008-2_5.pgm' # Ignored if PyGeodesy module or Geoids Database is not available
FALLBACK_GEOID_HEIGHT_M = 0.0 # Ignored if if PyGeodesy module or Geoids Database are available
file_loaded = False
if os.path.exists(GEOID_DATABASE_FILE):
  try:
    import pygeodesy
    GEOID_DATABASE_FILE=GEOID_DATABASE_FILE
    logger.log_info("Loading Geoids Database from: " + GEOID_DATABASE_FILE)
    ginterpolator = pygeodesy.GeoidKarney(GEOID_DATABASE_FILE)
    file_loaded = True
  except Exception as e:
    logger.log_warn("Geoids database failed to import: " + str(e), throttle_s = 5.0)
if file_loaded is False:
  def ginterpolator(single_position):
    return FALLBACK_GEOID_HEIGHT_M



def get_navpose_publisher_namespaces():
    msg_type = 'nepi_interfaces/NavPose'
    return nepi_sdk.find_topics_by_msg(msg_type)

def get_navpose_comp_publisher_namespaces(name, topics_list = None, types_list = None):
    topic_list = []
    msg_list = []
    if name in NAVPOSE_MSG_DICT.keys():
      msg_str_list = list(NAVPOSE_MSG_DICT[name].keys())
      [topic_list,msg_list] = nepi_sdk.find_topics_by_msgs(msg_str_list, topics_list = topics_list, types_list = types_list)
    return topic_list,msg_list



###############
### NavPose Solution Components


NAVPOSE_COMPONENTS = ['location','heading','orientation','position','altitude','depth']

NAVPOSE_MSG_DICT = {
        'location': {
            'nepi_interfaces/NavPoseLocation': NavPoseLocation,
            'sensor_msgs/NavSatFix': NavSatFix,
            'geographic_msgs/GeoPoint': GeoPoint
        },
        'heading': {
            'nepi_interfaces/NavPoseHeading': NavPoseHeading
        },
        'orientation': {
            'nepi_interfaces/NavPoseOrientation': NavPoseOrientation,
            'nav_msgs/Odometry': Odometry,
            'geometry_msgs/Pose': Pose,
            'geometry_msgs/Quaternion': Quaternion
        },
        'position': {
            'nepi_interfaces/NavPosePosition': NavPosePosition,
            'nav_msgs/Odometry': Odometry, 
            'geometry_msgs/Pose': Pose,
            'geometry_msgs/Point ': Point
        },
        'altitude': {
            'nepi_interfaces/NavPoseAltitude': NavPoseAltitude,
            'sensor_msgs/NavSatFix': NavSatFix,
            'geographic_msgs/GeoPoint': GeoPoint
        },
        'depth': {
            'nepi_interfaces/NavPoseDepth': NavPoseDepth, 
            'sensor_msgs/NavSatFix': NavSatFix,
            'geographic_msgs/GeoPoint': GeoPoint      
        },
        'pan_tilt': {
            'nepi_interfaces/NavPosePanTilt': NavPosePanTilt
        }
    }


NAVPOSE_3D_FRAME_OPTIONS = ['base_frame','nepi_frame','sensor_frame','world_frame']
NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED']
NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL'] # ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
NAVPOSE_DEPTH_FRAME_OPTIONS = ['DEPTH']

BLANK_NAVPOSE_INFO_DICT = {
        'frame_nav': 'ENU',
        'frame_alt': 'WGS84',
        'frame_depth': 'DEPTH'
    }

BLANK_HEADING_DATA_DICT = {
    'time_heading': 0.0,
    # Heading should be provided in Degrees True North
    'heading_deg': 0.0,
    # Ground speed in the heading direction, meters per second
    'heading_m_per_sec': 0.0,
}

BLANK_POSITION_DATA_DICT = {
    'time_position': 0.0,
    # Position should be provided in Meters ENU (x,y,z) with x forward, y left, and z up
    'x_m': 0.0,
    'y_m': 0.0,
    'z_m': 0.0,
    # Linear velocity in Meters per second ENU
    'x_m_per_sec': 0.0,
    'y_m_per_sec': 0.0,
    'z_m_per_sec': 0.0,
}

BLANK_ORIENTATION_DATA_DICT = {
    'time_orientation': 0.0,
    # Orientation should be provided in Degrees ENU
    'roll_deg': 0.0,
    'pitch_deg': 0.0,
    'yaw_deg': 0.0,
    # Angular rates in Degrees per second ENU
    'roll_deg_per_sec': 0.0,
    'pitch_deg_per_sec': 0.0,
    'yaw_deg_per_sec': 0.0,
}

BLANK_LOCATION_DATA_DICT = {
    'time_location': 0.0,
    # Location Lat,Long
    'latitude': 0.0,
    'longitude': 0.0,
    # Speed over ground, meters per second
    'location_m_per_sec': 0.0,
}

BLANK_ALTITUDE_DATA_DICT = {
    'time_altitude': 0.0,
    # Altitude should be provided in postivie meters WGS84
    'altitude_m': 0.0,
    # Vertical rate (altitude change), meters per second
    'altitude_m_per_sec': 0.0,
}

BLANK_DEPTH_DATA_DICT = {
    'time_depth': 0.0,
    # Depth should be provided in positive distance from surface in meters
    'depth_m': 0.0,
    # Depth rate (depth change), meters per second
    'depth_m_per_sec': 0.0,
}

BLANK_PAN_TILT_DATA_DICT = {
    'time_pan_tilt': 0.0,
    # Depth should be provided in positive distance from surface in meters
    'pan_deg': 0.0,
    'tilt_deg': 0.0
}



BLANK_NAVPOSE_DICT = {
    'navpose_frame': 'None',
    'navpose_description': 'None',
    'frame_nav': 'ENU',
    'frame_altitude': 'WGS84',
    'frame_depth': 'MSL',

    'geoid_height_meters': 0.0,

    'has_location': False,
    'time_location': 0.0,
    # Location Lat,Long
    'latitude': 0.0,
    'longitude': 0.0,
    # Speed over ground, meters per second
    'location_m_per_sec': 0.0,

    'has_heading': False,
    'time_heading': 0.0,
    # Heading should be provided in Degrees True North
    'heading_deg': 0.0,
    # Ground speed in the heading direction, meters per second
    'heading_m_per_sec': 0.0,

    'has_position': False,
    'time_position': 0.0,
    # Position should be provided in Meters in specified 3d frame (x,y,z) with x forward, y right/left, and z up/down
    'x_m': 0.0,
    'y_m': 0.0,
    'z_m': 0.0,
    # Linear velocity in Meters per second in specified 3d frame
    'x_m_per_sec': 0.0,
    'y_m_per_sec': 0.0,
    'z_m_per_sec': 0.0,

    'has_orientation': False,
    'time_orientation': 0.0,
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 0.0,
    'pitch_deg': 0.0,
    'yaw_deg': 0.0,
    # Angular rates in Degrees per second in specified 3d frame
    'roll_deg_per_sec': 0.0,
    'pitch_deg_per_sec': 0.0,
    'yaw_deg_per_sec': 0.0,

    'has_altitude': False,
    'time_altitude': 0.0,
    # Altitude should be provided in postivie meters in specified alt frame
    'altitude_m': 0.0,
    # Vertical rate (altitude change), meters per second
    'altitude_m_per_sec': 0.0,

    'has_depth': False,
    'time_depth': 0.0,
    # Depth should be provided in positive meters
    'depth_m': 0.0,
    # Depth rate (depth change), meters per second
    'depth_m_per_sec': 0.0,

    'has_pan_tilt': False,
    'time_pan_tilt': 0.0,
    # Pan Tilt should be provided in positive degs
    'pan_deg': 0.0,
    'tilt_deg': 0.0,
    'pan_tilt_heading_deg': 0.0,
    'pan_tilt_x_m': 0.0,
    'pan_tilt_y_m': 0.0,
    'pan_tilt_z_m': 0.0,
    'pan_tilt_roll_deg': 0.0,
    'pan_tilt_pitch_deg': 0.0,
    'pan_tilt_yaw_deg': 0.0,
}



BLANK_TRANSFORM_DICT = {
    'source_ref_description': '',
    'end_ref_descriptions': '',

    # Position should be provided in Meters in specified 3d frame (x,y,z) with x forward, y right/left, and z up/down
    'x_m': 0.0,
    'y_m': 0.0,
    'z_m': 0.0,

    'x_invert': False,
    'y_invert': False,
    'z_invert': False,
  
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 0.0,
    'pitch_deg': 0.0,
    'yaw_deg': 0.0,

    'roll_invert': False,
    'pitch_invert': False,
    'yaw_invert': False,

    'heading_deg': 0.0,
    'heading_invert': False

}


BLANK_NAVPOSE_TRACK_DICT = {
    # altitude in 'WGS84' frame
    # depth in 'DEPTH' frame
    'timestamp': 0.0,
    # Location Lat,Long
    'latitude': 0.0,
    'longitude': 0.0,
    'heading_deg': 0.0,
    'roll_deg': 0.0,
    'pitch_deg': 0.0,
    'yaw_deg': 0.0,
    'altitude_m': 0.0,
    'depth_m': 0.0,
    'pan_deg': 0.0,
    'tilt_deg': 0.0
}

def clear_navpose_dict_comp(comp_name,npdata_dict):
    success = False
    try:
      if comp_name == 'location':
          npdata_dict['has_location'] = False
          npdata_dict['time_location'] = 0.0
          npdata_dict['latitude'] = 0.0
          npdata_dict['longitude'] = 0.0
          npdata_dict['location_m_per_sec'] = 0.0
      if  comp_name == 'heading':
          npdata_dict['has_heading'] = False
          npdata_dict['time_heading'] = 0.0
          npdata_dict['heading_deg'] = 0.0
          npdata_dict['heading_m_per_sec'] = 0.0
      if  comp_name == 'orientation':
          npdata_dict['has_orientation'] = False
          npdata_dict['time_orientation']  = 0.0
          npdata_dict['roll_deg']  = 0.0
          npdata_dict['pitch_deg']  = 0.0
          npdata_dict['yaw_deg']  = 0.0
          npdata_dict['roll_deg_per_sec']  = 0.0
          npdata_dict['pitch_deg_per_sec']  = 0.0
          npdata_dict['yaw_deg_per_sec']  = 0.0
      if  comp_name == 'position':
          npdata_dict['has_position'] = False
          npdata_dict['time_position'] = 0.0
          npdata_dict['x_m']  = 0.0
          npdata_dict['y_m']  = 0.0
          npdata_dict['z_m']  = 0.0
          npdata_dict['x_m_per_sec']  = 0.0
          npdata_dict['y_m_per_sec']  = 0.0
          npdata_dict['z_m_per_sec']  = 0.0
      if  comp_name == 'altitude':
          npdata_dict['has_altitude'] = False
          npdata_dict['time_altitude']  = 0.0
          npdata_dict['altitude_m']  = 0.0
          npdata_dict['altitude_m_per_sec']  = 0.0
      if  comp_name == 'depth':
          npdata_dict['has_depth'] = False
          npdata_dict['time_depth']  = 0.0
          npdata_dict['depth_m']  = 0.0
          npdata_dict['depth_m_per_sec']  = 0.0
      if  comp_name == 'pan_tilt':
          npdata_dict['has_pan_tilt'] = False
          npdata_dict['time_pan_tilt']  = 0.0
          npdata_dict['pan_deg']  = 0.0
          npdata_dict['tilt_deg']  = 0.0
          npdata_dict['pan_tilt_heading_deg']  = 0.0
          npdata_dict['pan_tilt_x_m'] = 0.0
          npdata_dict['pan_tilt_y_m']  = 0.0
          npdata_dict['pan_tilt_z_m']  = 0.0
          npdata_dict['pan_tilt_roll_deg']  = 0.0
          npdata_dict['pan_tilt_pitch_deg'] = 0.0
          npdata_dict['pan_tilt_yaw_deg']  = 0.0
    except:
      pass
    return npdata_dict


def update_navpose_dict_from_dict(npdata_dict_org,npdata_dict_new):
    success = False
    np_dict = copy.deepcopy(npdata_dict_org)
    if npdata_dict_org is not None and npdata_dict_new is not None:
      try:
        if npdata_dict_new['has_location'] == True:
            npdata_dict_org['has_location'] = True
            npdata_dict_org['time_location'] = npdata_dict_new['time_location']
            npdata_dict_org['latitude'] = npdata_dict_new['latitude']
            npdata_dict_org['longitude'] = npdata_dict_new['longitude']
            npdata_dict_org['location_m_per_sec'] = npdata_dict_new['location_m_per_sec']
        if npdata_dict_new['has_heading'] == True:
            npdata_dict_org['has_heading'] = True
            npdata_dict_org['time_heading'] = npdata_dict_new['time_heading']
            npdata_dict_org['heading_deg'] = npdata_dict_new['heading_deg']
            npdata_dict_org['heading_m_per_sec'] = npdata_dict_new['heading_m_per_sec']
        if npdata_dict_new['has_orientation'] == True:
            npdata_dict_org['has_orientation'] = True
            npdata_dict_org['time_orientation'] = npdata_dict_new['time_orientation']
            npdata_dict_org['roll_deg'] = npdata_dict_new['roll_deg']
            npdata_dict_org['pitch_deg'] = npdata_dict_new['pitch_deg']
            npdata_dict_org['yaw_deg'] = npdata_dict_new['yaw_deg']
            npdata_dict_org['roll_deg_per_sec'] = npdata_dict_new['roll_deg_per_sec']
            npdata_dict_org['pitch_deg_per_sec'] = npdata_dict_new['pitch_deg_per_sec']
            npdata_dict_org['yaw_deg_per_sec'] = npdata_dict_new['yaw_deg_per_sec']
        if npdata_dict_new['has_position'] == True:
            npdata_dict_org['has_position'] = True
            npdata_dict_org['time_position'] = npdata_dict_new['time_position']
            npdata_dict_org['x_m'] = npdata_dict_new['x_m']
            npdata_dict_org['y_m'] = npdata_dict_new['y_m']
            npdata_dict_org['z_m'] = npdata_dict_new['z_m']
            npdata_dict_org['x_m_per_sec'] = npdata_dict_new['x_m_per_sec']
            npdata_dict_org['y_m_per_sec'] = npdata_dict_new['y_m_per_sec']
            npdata_dict_org['z_m_per_sec'] = npdata_dict_new['z_m_per_sec']
        if npdata_dict_new['has_altitude'] == True:
            npdata_dict_org['has_altitude'] = True
            npdata_dict_org['time_altitude'] = npdata_dict_new['time_altitude']
            npdata_dict_org['altitude_m'] = npdata_dict_new['altitude_m']
            npdata_dict_org['altitude_m_per_sec'] = npdata_dict_new['altitude_m_per_sec']
        if npdata_dict_new['has_depth'] == True:
            npdata_dict_org['has_depth'] = True
            npdata_dict_org['time_depth'] = npdata_dict_new['time_depth']
            npdata_dict_org['depth_m'] = npdata_dict_new['depth_m']
            npdata_dict_org['depth_m_per_sec'] = npdata_dict_new['depth_m_per_sec']
        if npdata_dict_new['has_pan_tilt'] == True:
            npdata_dict_org['has_pan_tilt'] = True
            npdata_dict_org['time_pan_tilt'] = npdata_dict_new['time_pan_tilt']
            npdata_dict_org['pan_deg'] = npdata_dict_new['pan_deg']
            npdata_dict_org['tilt_deg'] = npdata_dict_new['tilt_deg']
            npdata_dict_org['pan_tilt_heading_deg'] = npdata_dict_new['pan_tilt_heading_deg']
            npdata_dict_org['pan_tilt_x_m'] = npdata_dict_new['pan_tilt_x_m']
            npdata_dict_org['pan_tilt_y_m'] = npdata_dict_new['pan_tilt_y_m']
            npdata_dict_org['pan_tilt_z_m'] = npdata_dict_new['pan_tilt_z_m']
            npdata_dict_org['pan_tilt_roll_deg'] = npdata_dict_new['pan_tilt_roll_deg']
            npdata_dict_org['pan_tilt_pitch_deg'] = npdata_dict_new['pan_tilt_pitch_deg']
            npdata_dict_org['pan_tilt_yaw_deg'] = npdata_dict_new['pan_tilt_yaw_deg']
        np_dict = npdata_dict_org
      except:
        pass
    return np_dict



def update_navpose_resets_dict_from_updates(np_resets_dict, npdata_dict_org, npdata_update_new, npdata_update_last, comp_name, update_on_crossing = False, update_crossing = 0):
    success = False
    if npdata_dict_org is not None and npdata_update_new is not None  and npdata_update_last is not None:
      try:
        if npdata_update_new['has_location'] == True and comp_name == 'location':
            np_resets_dict['has_location'] = True
            if update_on_crossing == False:
              np_resets_dict['time_location'] = npdata_update_new['time_location']
              np_resets_dict['latitude'] = npdata_update_new['latitude']
              np_resets_dict['longitude'] = npdata_update_new['longitude']
            elif npdata_update_last['has_location'] == True:
              np_resets_dict['time_location'] = npdata_update_new['time_location']
              np_resets_dict['latitude'] = npdata_update_new['latitude'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['latitude'], npdata_update_last['latitude']) else np_resets_dict['latitude']
              np_resets_dict['longitude'] = npdata_update_new['longitude'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['longitude'], npdata_update_last['longitude']) else np_resets_dict['longitude']
            np_resets_dict['has_location'] = True
        
            
        if npdata_update_new['has_heading'] == True and comp_name == 'heading':
            np_resets_dict['has_heading'] = True
            if update_on_crossing == False:
              np_resets_dict['time_heading'] = npdata_update_new['time_heading']
              np_resets_dict['heading'] = npdata_update_new['heading'] - npdata_dict_org['heading']
            elif npdata_update_last['has_heading'] == True:
              np_resets_dict['time_heading'] = npdata_update_new['time_heading']
              np_resets_dict['heading'] = npdata_update_new['heading'] - npdata_dict_org['heading'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['heading'], npdata_update_last['heading']) else np_resets_dict['heading']

        if npdata_update_new['has_orientation'] == True and comp_name == 'orientation':
            np_resets_dict['has_orientation'] = True
            if update_on_crossing == False:
              np_resets_dict['time_orientation'] = npdata_update_new['time_orientation']
              np_resets_dict['roll_deg'] = npdata_update_new['roll_deg'] - npdata_dict_org['roll_deg']
              np_resets_dict['pithas_orientationch_deg'] = npdata_dict_org['pitch_deg'] - npdata_update_new['pitch_deg']
              np_resets_dict['yaw_deg'] = npdata_update_new['yaw_deg'] - npdata_dict_org['yaw_deg']
            elif npdata_update_last['has_orientation'] == True:
              np_resets_dict['time_orientation'] = npdata_update_new['time_orientation']
              np_resets_dict['roll_deg'] = -npdata_update_new['roll_deg'] - npdata_dict_org['roll_deg'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['roll_deg'], npdata_update_last['roll_deg']) else np_resets_dict['roll_deg']
              np_resets_dict['pitch_deg'] = npdata_update_new['pitch_deg'] - npdata_dict_org['pitch_deg'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['pitch_deg'], npdata_update_last['pitch_deg']) else np_resets_dict['pitch_deg']
              np_resets_dict['yaw_deg'] = npdata_update_new['yaw_deg'] - npdata_dict_org['yaw_deg'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['yaw_deg'], npdata_update_last['yaw_deg']) else np_resets_dict['yaw_deg']

        if npdata_update_new['has_position'] == True and comp_name == 'position':
            np_resets_dict['has_position'] = True
            if update_on_crossing == False:
              np_resets_dict['time_position'] = npdata_update_new['time_position']
              np_resets_dict['x_m'] = npdata_update_new['x_m'] - npdata_dict_org['x_m']
              np_resets_dict['y_m'] = npdata_update_new['y_m'] - npdata_dict_org['y_m']
              np_resets_dict['z_m'] = npdata_update_new['z_m'] - npdata_dict_org['z_m']
            elif npdata_update_last['has_position'] == True:
              np_resets_dict['time_position'] = npdata_update_new['time_location']
              np_resets_dict['x_m'] = npdata_update_new['x_m'] - npdata_dict_org['x_m'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['x_m'], npdata_update_last['x_m']) else np_resets_dict['x_m']
              np_resets_dict['y_m'] = npdata_update_new['y_m'] - npdata_dict_org['y_m'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['y_m'], npdata_update_last['y_m']) else np_resets_dict['y_m']
              np_resets_dict['z_m'] = npdata_update_new['z_m'] - npdata_dict_org['z_m'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['z_m'], npdata_update_last['z_m']) else np_resets_dict['z_m']

        if npdata_update_new['has_altitude'] == True and comp_name == 'altitude':
            np_resets_dict['has_altitude'] = True
            if update_on_crossing == False:
              np_resets_dict['time_altitude'] = npdata_update_new['time_altitude']
              np_resets_dict['altitude_m'] = npdata_update_new['altitude_m'] - npdata_dict_org['altitude_m']
            elif npdata_update_last['has_altitude'] == True:
              np_resets_dict['time_altitude'] = npdata_update_new['time_altitude']
              np_resets_dict['altitude_m'] = npdata_update_new['altitude_m'] - npdata_dict_org['altitude_m'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['altitude_m'], npdata_update_last['altitude_m']) else np_resets_dict['altitude_m']

        if npdata_update_new['has_depth'] == True and comp_name == 'depth':
            np_resets_dict['has_depth'] = True
            if update_on_crossing == False:
              np_resets_dict['time_detph'] = npdata_update_new['time_depth']
              np_resets_dict['depth_m'] = npdata_update_new['depth_m'] - npdata_dict_org['depth_m']
            elif npdata_update_last['has_depth'] == True:
              np_resets_dict['time_detph'] = npdata_update_new['time_detph']
              np_resets_dict['depth_m'] = npdata_update_new['depth_m'] - npdata_dict_org['depth_m'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['depth_m'], npdata_update_last['depth_m']) else np_resets_dict['depth_m']

        if npdata_update_new['has_pan_tilt'] == True and comp_name == 'pan_tilt':
            np_resets_dict['has_pan_tilt'] = True
            if update_on_crossing == False:
              np_resets_dict['time_pan_tilt'] = npdata_update_new['time_pan_tilt']
              np_resets_dict['pan_deg'] = npdata_update_new['pan_deg'] - npdata_dict_org['pan_deg']
              np_resets_dict['tilt_deg'] = npdata_update_new['tilt_deg'] - npdata_dict_org['tilt_deg']
            elif npdata_update_last['has_pan_tilt'] == True:
              np_resets_dict['time_pan_tilt'] = npdata_update_new['time_pan_tilt']
              np_resets_dict['pan_deg'] = npdata_update_new['pan_deg'] - npdata_dict_org['pan_deg'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['pan_deg'], npdata_update_last['pan_deg']) else np_resets_dict['pan_deg']
              np_resets_dict['tilt_deg'] = npdata_update_new['tilt_deg'] - npdata_dict_org['tilt_deg'] if nepi_utils.value_is_between(update_crossing, npdata_update_new['tilt_deg'], npdata_update_last['tilt_deg']) else np_resets_dict['tilt_deg']
      except Exception as e:
        print("NavPose Dict resets update failed: " + str(e))
    return np_resets_dict


def update_navpose_dict_from_offsets(npdata_dict_org, np_offsets_dict):
    success = False
    
    if npdata_dict_org is not None and np_offsets_dict is not None:
      try:
        if np_offsets_dict['has_location'] == True:
              npdata_dict_org['time_location'] = np_offsets_dict['time_location']
              npdata_dict_org['latitude'] = np_offsets_dict['latitude']
              npdata_dict_org['longitude'] = np_offsets_dict['longitude']
           
            
        if np_offsets_dict['has_heading'] == True:
              npdata_dict_org['time_heading'] = np_offsets_dict['time_heading']
              npdata_dict_org['heading'] = npdata_dict_org['heading'] + np_offsets_dict['heading']
      

        if np_offsets_dict['has_orientation'] == True:
              npdata_dict_org['time_orientation'] = np_offsets_dict['time_orientation']
              npdata_dict_org['roll_deg'] = npdata_dict_org['roll_deg'] + np_offsets_dict['roll_deg']
              npdata_dict_org['pitch_deg'] = npdata_dict_org['pitch_deg'] + np_offsets_dict['pitch_deg']
              npdata_dict_org['yaw_deg'] = npdata_dict_org['yaw_deg'] + np_offsets_dict['yaw_deg']

        if np_offsets_dict['has_position'] == True:
              npdata_dict_org['time_position'] = np_offsets_dict['time_position']
              npdata_dict_org['x_m'] = npdata_dict_org['_'] + np_offsets_dict['x_m']
              npdata_dict_org['y_m'] = npdata_dict_org['y_m'] + np_offsets_dict['y_m']
              npdata_dict_org['z_m'] = npdata_dict_org['z_m'] + np_offsets_dict['z_m']


        if np_offsets_dict['has_altitude'] == True:
              npdata_dict_org['time_altitude'] = np_offsets_dict['time_altitude']
              npdata_dict_org['altitude_m'] = npdata_dict_org['altitude_m'] + np_offsets_dict['altitude_m']
        
        if np_offsets_dict['has_depth'] == True:
              npdata_dict_org['time_detph'] = np_offsets_dict['time_depth']
              npdata_dict_org['depth_m'] = npdata_dict_org['depth_m'] + np_offsets_dict['depth_m']
         

        if np_offsets_dict['has_pan_tilt'] == True:
              npdata_dict_org['time_pan_tilt'] = np_offsets_dict['time_pan_tilt']
              npdata_dict_org['pan_deg'] = npdata_dict_org['pan_deg'] + np_offsets_dict['pan_deg']
              npdata_dict_org['tilt_deg'] = npdata_dict_org['tilt_deg'] + np_offsets_dict['tilt_deg']
      except Exception as e:
        logger.log_warn("NavPose Dict update from offsets failed: " + str(e))
    return npdata_dict_org


def update_navpose_dict_from_msg(name, navpose_dict, msg, transform_dict = None):
    msg_type = msg._type
    
    if name == 'location':
      if msg_type == 'nepi_interfaces/NavPoseLocation':
        try:
          navpose_dict['has_location'] = True
          navpose_dict['time_location'] = msg.timestamp if msg.timestamp != 0.0 else nepi_utils.get_time()
          navpose_dict['latitude'] = msg.latitude
          navpose_dict['longitude'] = msg.longitude
          navpose_dict['location_m_per_sec'] = msg.location_m_per_sec
        except:
          pass
      elif msg_type == 'sensor_msgs/NavSatFix':
        try:
          navpose_dict['has_location'] = True
          navpose_dict['time_location'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          navpose_dict['latitude'] = msg.latitude
          navpose_dict['longitude'] = msg.longitude
        except:
          pass
      elif msg_type == 'geographic_msgs/GeoPoint':
        try:
          navpose_dict['has_location'] = True
          navpose_dict['time_location'] = nepi_utils.get_time()
          navpose_dict['latitude'] = msg.latitude
          navpose_dict['longitude'] = msg.longitude
        except:
          pass

    if name == 'heading':
      if msg_type == 'nepi_interfaces/NavPoseHeading':
        try:
          navpose_dict['has_heading'] = True
          navpose_dict['time_heading'] = msg.timestamp if msg.timestamp != 0.0 else nepi_utils.get_time()
          navpose_dict['heading_deg'] = msg.heading_deg
          navpose_dict['heading_m_per_sec'] = msg.heading_m_per_sec
        except:
          pass
     
          
    if name == 'orientation':
      if msg_type == 'nepi_interfaces/NavPoseOrientation':
        try:
          navpose_dict['has_orientation'] = True
          navpose_dict['time_orientation'] = msg.timestamp if msg.timestamp != 0.0 else nepi_utils.get_time()
          navpose_dict['roll_deg'] = msg.roll_deg
          navpose_dict['pitch_deg'] = msg.pitch_deg
          navpose_dict['yaw_deg'] = msg.yaw_deg
          navpose_dict['roll_deg_per_sec'] = msg.roll_deg_per_sec
          navpose_dict['pitch_deg_per_sec'] = msg.pitch_deg_per_sec
          navpose_dict['yaw_deg_per_sec'] = msg.yaw_deg_per_sec
        except:
          pass
      elif msg_type == 'nav_msgs/Odometry':
        try:
          data = msg.pose.pose.orientation
          quat = [data.x,data.y,data.z,data.w]
          [roll,pitch,yaw] = convert_quat2rpy(quat)
          navpose_dict['has_orientation'] = True
          navpose_dict['time_orientation'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          navpose_dict['roll_deg'] = roll
          navpose_dict['pitch_deg'] = pitch
          navpose_dict['yaw_deg'] = yaw
        except:
          pass
      elif msg_type == 'geometry_msgs/Pose':
        try:
          data = msg.orientation
          quat = [data.x,data.y,data.z,data.w]
          [roll,pitch,yaw] = convert_quat2rpy(quat)
          navpose_dict['has_orientation'] = True
          navpose_dict['time_orientation'] = nepi_utils.get_time()
          navpose_dict['roll_deg'] = roll
          navpose_dict['pitch_deg'] = pitch
          navpose_dict['yaw_deg'] = yaw
        except:
          pass
      elif msg_type == 'geometry_msgs/Quaternion':
        try:
          data = msg
          quat = [data.x,data.y,data.z,data.w]
          [roll,pitch,yaw] = convert_quat2rpy(quat)
          navpose_dict['has_orientation'] = True
          navpose_dict['time_orientation'] = nepi_utils.get_time()
          navpose_dict['roll_deg'] = roll
          navpose_dict['pitch_deg'] = pitch
          navpose_dict['yaw_deg'] = yaw
        except:
          pass

    if name == 'position':
      if msg_type == 'nepi_interfaces/NavPosePosition':
        try:
          navpose_dict['has_position'] = True
          navpose_dict['time_position'] = msg.timestamp if msg.timestamp != 0.0 else nepi_utils.get_time()
          navpose_dict['x_m'] = msg.x_m
          navpose_dict['y_m'] = msg.y_m
          navpose_dict['z_m'] = msg.y_m
          navpose_dict['x_m_per_sec'] = msg.x_m_per_sec
          navpose_dict['y_m_per_sec'] = msg.y_m_per_sec
          navpose_dict['z_m_per_sec'] = msg.z_m_per_sec
        except:
          pass
      elif msg_type == 'nav_msgs/Odometry':
        try:
          data = msg.pose.pose.position
          pos = [data.x,data.y,data.z]
          navpose_dict['has_position'] = True
          navpose_dict['time_position'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          navpose_dict['x_m'] = pos[0]
          navpose_dict['y_m'] = pos[1]
          navpose_dict['z_m'] = pos[2]
        except:
          pass
      elif msg_type == 'geometry_msgs/Pose':
        try:
          data = msg.position
          pos = [data.x,data.y,data.z]
          navpose_dict['has_position'] = True
          navpose_dict['time_position'] = nepi_utils.get_time()
          navpose_dict['x_m'] = pos[0]
          navpose_dict['y_m'] = pos[1]
          navpose_dict['z_m'] = pos[2]
        except:
          pass
      elif msg_type == 'geometry_msgs/Point':
        try:
          data = msg
          pos = [data.x,data.y,data.z]
          navpose_dict['has_position'] = True
          navpose_dict['time_position'] = nepi_utils.get_time()
          navpose_dict['x_m'] = pos[0]
          navpose_dict['y_m'] = pos[1]
          navpose_dict['z_m'] = pos[2]
        except:
          pass

   
    if name == 'altitude':
      if msg_type == 'nepi_interfaces/NavPosePosition':
        try:
          navpose_dict['has_altitude'] = True
          navpose_dict['time_altitude'] = msg.timestamp
          navpose_dict['altitude_m'] = msg.y_m
        except:
          pass
      elif msg_type == 'nepi_interfaces/NavPoseAltitude':
        try:
          navpose_dict['has_altitude'] = True
          navpose_dict['time_altitude'] = msg.timestamp
          navpose_dict['altitude_m'] = msg.altitude_m
          navpose_dict['altitude_m_per_sec'] = msg.altitude_m_per_sec
        except:
          pass
      elif msg_type == 'sensor_msgs/NavSatFix':
        try:
          navpose_dict['has_altitude'] = True
          navpose_dict['time_altitude'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          navpose_dict['altitude_m'] = msg.altitude
        except:
          pass
      elif msg_type == 'geographic_msgs/GeoPoint':
        try:
          navpose_dict['has_altitude'] = True
          navpose_dict['time_altitude'] = nepi_utils.get_time()
          navpose_dict['altitude_m'] = msg.altitude
        except:
          pass

    if name == 'depth':
      if msg_type == 'nepi_interfaces/NavPosePosition':
        try:
          navpose_dict['has_depth'] = True
          navpose_dict['time_depth'] = msg.timestamp
          navpose_dict['depth_m'] = msg.depth_m
        except:
          pass
      elif msg_type == 'nepi_interfaces/NavPoseAltitude':
        try:
          navpose_dict['has_depth'] = True
          navpose_dict['time_depth'] = msg.timestamp
          navpose_dict['depth_m'] = 0 - msg.altitude_m
          navpose_dict['depth_m_per_sec'] = 0 - msg.altitude_m_per_sec
        except:
          pass
      elif msg_type == 'sensor_msgs/NavSatFix':
        try:
          navpose_dict['has_depth'] = True
          navpose_dict['time_depth'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          navpose_dict['depth_m'] = 0 - msg.altitude
        except:
          pass
      elif msg_type == 'geographic_msgs/GeoPoint':
        try:
          navpose_dict['has_depth'] = True
          navpose_dict['time_depth'] = nepi_utils.get_time()
          navpose_dict['depth_m'] = 0 - msg.altitude
        except:
          pass

    if name == 'pan_tilt':
      if msg_type == 'nepi_interfaces/NavPosePanTilt':
        try:
          navpose_dict['has_pan_tilt'] = True
          navpose_dict['time_pan_tilt'] = msg.timestamp
          navpose_dict['pan_deg'] = msg.pan_deg
          navpose_dict['tilt_deg'] = msg.tilt_deg
        except:
          pass
      

    if name == 'pan_tilt':
      if transform_dict is None:
        transform_dict = BLANK_TRANSFORM_DICT
      navpose_dict = update_navpose_dict_pantilt(navpose_dict,transform_dict)
    else:
      if transform_dict is not None:
        if transform_dict != BLANK_TRANSFORM_DICT:
          #logger.log_warn("navpose_dict befor: " + str([navpose_dict['roll_deg'], navpose_dict['pitch_deg'], navpose_dict['yaw_deg']]),)

          navpose_dict = transform_navpose_dict(navpose_dict,transform_dict)
          #logger.log_warn("navpose_dict after: " + str([navpose_dict['roll_deg'], navpose_dict['pitch_deg'], navpose_dict['yaw_deg']]), throttle_s = 5.0)

          #logger.log_warn("navpose_dict: " + str(navpose_dict), throttle_s = 5.0)

    return navpose_dict


#######################
# Transpose Functions


def check_tranform_dict(transform_dict):
  for key in BLANK_TRANSFORM_DICT.keys():
    if key not in transform_dict.keys():
      transform_dict[key] = BLANK_TRANSFORM_DICT[key]
  return transform_dict

def convert_transform_dict2msg(transform_dict, log_name_list = []):
  transform_dict = check_tranform_dict(transform_dict)
  transform_msg = Transform()
  if transform_dict is not None:
    try:
      transform_msg.source_ref_description = transform_dict['source_ref_description']
      transform_msg.end_ref_description = transform_dict['end_ref_description']

      transform_msg.heading_deg = transform_dict['heading_deg'] 
      transform_msg.heading_invert = transform_dict['heading_invert']

      transform_msg.roll_deg = transform_dict['roll_deg']
      transform_msg.pitch_deg = transform_dict['pitch_deg'] 
      transform_msg.yaw_deg = transform_dict['yaw_deg']

      transform_msg.roll_invert = transform_dict['roll_invert']
      transform_msg.pitch_invert = transform_dict['pitch_invert'] 
      transform_msg.yaw_invert = transform_dict['yaw_invert']

      transform_msg.x_m = transform_dict['x_m'] 
      transform_msg.y_m = transform_dict['y_m']
      transform_msg.z_m = transform_dict['z_m']

      transform_msg.x_invert = transform_dict['x_invert'] 
      transform_msg.y_invert = transform_dict['y_invert']
      transform_msg.z_invert = transform_dict['z_invert']

    
    except Exception as e:
      logger.log_warn("Failed to convert Transform Data dict: " + str(e), throttle_s = 5.0, log_name_list = log_name_list)
  return transform_msg

def convert_transform_msg2dict(transform_msg):
  transform_dict = None
  try:
    transform_dict = nepi_sdk.convert_msg2dict(transform_msg)
  except Exception as e:
    logger.log_warn("Failed to convert Transform Msg to dict: " + str(e), throttle_s = 5.0)
  return transform_dict

def transform_object_pose(current_pose, rpy_rotation, units='deg'):
    """
    Transforms an object's XYZRPY pose given a RPY rotation in base frame.
    
    Args:
        current_pose: List or array [x, y, z, r, p, y]
        rpy_rotation: Rotation to apply [r, p, y] (base frame)
        units: 'deg' or 'rad'
        
    Returns:
        Updated [x, y, z, r, p, y]
    """
    # 1. Decompose input
    x, y, z, r, p, y = current_pose
    dr, dp, dy = rpy_rotation
    
    # 2. Create rotation objects
    # Current orientation
    r_obj = R.from_euler('xyz', [r, p, y], degrees=(units=='deg'))
    # Rotation to apply in base frame
    r_base = R.from_euler('xyz', [dr, dp, dy], degrees=(units=='deg'))
    
    # 3. Combine rotations (Base frame rotation = Premultiplication)
    r_new = r_base * r_obj
    new_rpy = r_new.as_euler('xyz', degrees=(units=='deg'))
    
    # 4. Position remains same if rotation is about own origin in base frame
    # If rotation is about base frame origin, position must be rotated:
    new_pos = r_base.apply([x, y, z])
    
    return [new_pos[0], new_pos[1], new_pos[2], 
            new_rpy[0], new_rpy[1], new_rpy[2]]



def transform_navpose_dict(npdata_dict, transform_dict, pt_transform_dict = None, log_name_list = []):
  #logger.log_warn("passing transform_dict: " + str(transform_dict), throttle_s = 5.0)
  #logger.log_warn("passing transform_dict: " + str(npdata_dict), throttle_s = 5.0)
  #logger.log_warn("passing transform_dict: " + str(pt_transform_dict), throttle_s = 5.0)

  success = True
  navpose_dict = copy.deepcopy(npdata_dict)
  if navpose_dict is None:
    success = False
    logger.log_info("Got None navpose dict", throttle_s = 5.0)
  elif transform_dict != BLANK_TRANSFORM_DICT:
      [x,y,z] = [transform_dict['x_m'],transform_dict['y_m'],transform_dict['z_m']]
      [ar,ap,ay] = [transform_dict['roll_deg'],transform_dict['pitch_deg'],transform_dict['yaw_deg']]

      try:

        if npdata_dict['has_location'] == True:
          navpose_dict['has_location'] = True
          navpose_dict['time_location'] = npdata_dict['time_location']
          cur_geo = GeoPoint()
          cur_geo.latitude = npdata_dict['latitude']
          cur_geo.longitude = npdata_dict['longitude']
          tr_geo = get_geopoint_at_enu_point(cur_geo,[x,y,z])
          navpose_dict['latitude'] = tr_geo.latitude
          navpose_dict['longitude'] = tr_geo.longitude


        if npdata_dict['has_heading'] == True:
          navpose_dict['has_heading'] = True
          navpose_dict['time_heading'] = npdata_dict['time_heading']
          invert = -1 if transform_dict['heading_invert'] else 1
          navpose_dict['heading_deg'] = npdata_dict['heading_deg'] * invert  + transform_dict['heading_deg']

        if npdata_dict['has_orientation'] == True:
          navpose_dict['has_orientation'] = True
          navpose_dict['time_orientation'] = npdata_dict['time_orientation']

          invert = -1 if transform_dict['roll_invert'] else 1
          npr = npdata_dict['roll_deg'] * invert
          invert = -1 if transform_dict['pitch_invert'] else 1
          npp = npdata_dict['pitch_deg'] * invert
          invert = -1 if transform_dict['yaw_invert'] else 1
          npy = npdata_dict['yaw_deg'] * invert

          [npr,npp,npy]  = rotate_enu_angles([npr,npp,npy],-ay,'z')
          npy += ay
          #[npr,npp,npy]  = rotate_enu_angles([npr,npp,npy],ap,'y')
          npp += ap
          #[npr,npp,npy]  = rotate_enu_angles([npr,npp,npy],ar,'x')
          npr += ar

          
          [ navpose_dict['roll_deg'], navpose_dict['pitch_deg'], navpose_dict['yaw_deg'] ]=  [npr,npp,npy]
          # Angular rate transform not yet implemented; zero out until derived
          [ navpose_dict['roll_deg_per_sec'], navpose_dict['pitch_deg_per_sec'], navpose_dict['yaw_deg_per_sec'] ]=  [0,0,0]


        if npdata_dict['has_position'] == True:
          navpose_dict['has_position'] = True
          navpose_dict['time_position'] = npdata_dict['time_position']

          invert = -1 if transform_dict['x_invert'] else 1
          npx = npdata_dict['x_deg'] * invert + x
          invert = -1 if transform_dict['y_invert'] else 1
          npy = npdata_dict['y_deg'] * invert + y
          invert = -1 if transform_dict['z_invert'] else 1
          npz = npdata_dict['z_deg'] * invert + z

          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ar,'x')
          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ap,'y')
          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ay,'z')
          
          [ navpose_dict['x_m'], navpose_dict['y_m'], navpose_dict['z_m'] ]=  [npx,npy,npz]

        if npdata_dict['has_altitude'] == True:
          navpose_dict['has_altitude'] = True
          navpose_dict['time_altitude'] = npdata_dict['time_altitude']

          invert = -1 if transform_dict['altitude_m'] else 1
          navpose_dict['altitude_m'] = npdata_dict['altitude_m'] * invert  + transform_dict['altitude_m']

          npx = 0
          invert = -1 if transform_dict['y_invert'] else 1
          npy = npdata_dict['altitude_m'] * invert + y
          npz = 0

          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ar,'x')
          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ap,'y')
          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ay,'z')
          
          navpose_dict['altitude_m'] = npy


        if npdata_dict['has_depth'] == True:
          navpose_dict['has_depth'] = True
          navpose_dict['time_depth'] = npdata_dict['time_depth']

          npx = 0
          invert = -1 if transform_dict['y_invert'] else 1
          npy = npdata_dict['depth_m'] * invert + y
          npz = 0

          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ar,'x')
          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ap,'y')
          [npx,npy,npz]  = rotate_enu_point([npx,npy,npz],ay,'z')
          
          navpose_dict['depth_m'] = npy


        if npdata_dict['has_pan_tilt'] == True:
          navpose_dict['has_pan_tilt'] = True
          navpose_dict['time_pan_tilt'] = npdata_dict['time_pan_tilt']

          invert = -1 if transform_dict['yaw_invert'] else 1
          navpose_dict['pan_deg'] = npdata_dict['pan_deg'] * invert  + transform_dict['yaw_deg']
          invert = -1 if transform_dict['pitch_invert'] else 1
          navpose_dict['tilt_deg'] = npdata_dict['tilt_deg'] * invert  + transform_dict['pitch_deg']
          if pt_transform_dict is None:
            pt_transform_dict = BLANK_TRANSFORM_DICT
          navpose_dict = update_navpose_dict_pantilt(navpose_dict,pt_transform_dict)

          navpose_dict['pan_tilt_heading_deg'] = npdata_dict['pan_tilt_heading_deg']
          navpose_dict['pan_tilt_x_m'] = npdata_dict['pan_tilt_x_m']
          navpose_dict['pan_tilt_y_m'] = npdata_dict['pan_tilt_y_m']
          navpose_dict['pan_tilt_z_m'] = npdata_dict['pan_tilt_z_m']

      except Exception as e:
        success = False
        logger.log_warn("Failed to transfrom NavPose dict: " + str(e), throttle_s = 5.0, log_name_list = log_name_list)
  return navpose_dict



#######################
# NavPose Data Helper Functions


def convert_navpose_amsl2wgs84(npdata_dict):
  if npdata_dict['frame_altitude'] == 'AMSL':
    geopoint_in = GeoPoint()
    geopoint_in.latitude = npdata_dict['latitude']
    geopoint_in.longitude = npdata_dict['longitude']
    geopoint_in.altitude = npdata_dict['altitude_m']
    geopoint_out = convert_amsl_to_wgs84(geopoint_in)
    npdata_dict['latitude'] = geopoint_out.latitude
    npdata_dict['longitude'] = geopoint_out.longitude
    npdata_dict['altitude_m'] = geopoint_out.altitude
  return npdata_dict

def convert_navpose_wgs842amsl(npdata_dict):
  if npdata_dict['frame_altitude'] == 'WGS84':
    geopoint_in = GeoPoint()
    geopoint_in.latitude = npdata_dict['latitude']
    geopoint_in.longitude = npdata_dict['longitude']
    geopoint_in.altitude = npdata_dict['altitude_m']
    geopoint_out = convert_wgs84_to_amsl(geopoint_in)
    npdata_dict['latitude'] = geopoint_out.latitude
    npdata_dict['longitude'] = geopoint_out.longitude
    npdata_dict['altitude_m'] = geopoint_out.altitude
  return npdata_dict  


  
def convert_navpose_enu2ned(npdata_dict):
  rpy_enu_d = [npdata_dict['roll_deg'], npdata_dict['pitch_deg'], npdata_dict['yaw_deg']]
  yaw_ned_d = convert_yaw_enu2ned(rpy_enu_d[2])
  rpy_ned_d = [rpy_enu_d[0],rpy_enu_d[1],yaw_ned_d]
  [npdata_dict['roll_deg'], npdata_dict['pitch_deg'], npdata_dict['yaw_deg']] = rpy_ned_d

  point_ned= [npdata_dict['y_m'], npdata_dict['x_m'], - npdata_dict['z_m']]
  [npdata_dict['x_m'], npdata_dict['y_m'], npdata_dict['z_m']] = point_ned

  return npdata_dict

def convert_navpose_ned2edu(npdata_dict):
  rpy_ned_d = [npdata_dict['roll_deg'], npdata_dict['pitch_deg'], npdata_dict['yaw_deg']]
  yaw_enu_d = convert_yaw_ned2enu(rpy_ned_d[2])
  rpy_enu_d = [rpy_enu_d[0],rpy_enu_d[1],yaw_enu_d]
  [npdata_dict['roll_deg'], npdata_dict['pitch_deg'], npdata_dict['yaw_deg']] = rpy_ned_d

  point_enu = [npdata_dict['y_m'], npdata_dict['x_m'], - npdata_dict['z_m']]
  [npdata_dict['x_m'], npdata_dict['y_m'], npdata_dict['z_m']] = point_enu

  return npdata_dict



def convert_navpose_dict2msg(npdata_dict, log_name_list = []):
  npdata_msg = None
  if npdata_dict is None:
    logger.log_info("Got None navpose dict", throttle_s = 5.0, log_name_list = log_name_list)
  else:

    blank_dict = BLANK_NAVPOSE_DICT
    for entry in blank_dict.keys():
      if entry not in npdata_dict.keys():
        npdata_dict[entry] = blank_dict[entry]
    try:
      np_msg = NavPose()
      np_msg.header.stamp = nepi_sdk.get_msg_stamp()

      np_msg.navpose_frame = npdata_dict['navpose_frame']
      np_msg.navpose_description = npdata_dict['navpose_description']
      np_msg.frame_nav = npdata_dict['frame_nav']
      np_msg.frame_altitude = npdata_dict['frame_altitude']
      np_msg.frame_depth = npdata_dict['frame_depth']

      np_msg.geoid_height_meters = npdata_dict['geoid_height_meters']

      np_msg.has_heading = npdata_dict['has_heading']
      np_msg.time_heading = npdata_dict['time_heading'] if np_msg.has_heading else 0.0
      np_msg.heading_deg = npdata_dict['heading_deg'] if np_msg.has_heading else -999
      np_msg.heading_m_per_sec = npdata_dict['heading_m_per_sec'] if np_msg.has_heading else -999

      np_msg.has_orientation = npdata_dict['has_orientation']
      np_msg.time_orientation = npdata_dict['time_orientation'] if np_msg.has_orientation else 0.0
      np_msg.roll_deg = npdata_dict['roll_deg'] if np_msg.has_orientation else -999
      np_msg.pitch_deg = npdata_dict['pitch_deg'] if np_msg.has_orientation else -999
      np_msg.yaw_deg = npdata_dict['yaw_deg'] if np_msg.has_orientation else -999
      np_msg.roll_deg_per_sec = npdata_dict['roll_deg_per_sec'] if np_msg.has_orientation else -999
      np_msg.pitch_deg_per_sec = npdata_dict['pitch_deg_per_sec'] if np_msg.has_orientation else -999
      np_msg.yaw_deg_per_sec = npdata_dict['yaw_deg_per_sec'] if np_msg.has_orientation else -999

      np_msg.has_position = npdata_dict['has_position']
      np_msg.time_position = npdata_dict['time_position'] if np_msg.has_position else 0.0
      np_msg.x_m = npdata_dict['x_m'] if np_msg.has_position else -999
      np_msg.y_m = npdata_dict['y_m'] if np_msg.has_position else -999
      np_msg.z_m = npdata_dict['z_m'] if np_msg.has_position else -999
      np_msg.x_m_per_sec = npdata_dict['x_m_per_sec'] if np_msg.has_position else -999
      np_msg.y_m_per_sec = npdata_dict['y_m_per_sec'] if np_msg.has_position else -999
      np_msg.z_m_per_sec = npdata_dict['z_m_per_sec'] if np_msg.has_position else -999

      np_msg.has_location = npdata_dict['has_location']
      np_msg.time_location = npdata_dict['time_location'] if np_msg.has_location else 0.0
      np_msg.latitude = npdata_dict['latitude'] if np_msg.has_location else -999
      np_msg.longitude = npdata_dict['longitude'] if np_msg.has_location else -999
      np_msg.location_m_per_sec = npdata_dict['location_m_per_sec'] if np_msg.has_location else -999

      np_msg.has_altitude = npdata_dict['has_altitude']
      np_msg.time_altitude = npdata_dict['time_altitude'] if np_msg.has_altitude else 0.0
      np_msg.altitude_m = npdata_dict['altitude_m'] if np_msg.has_altitude else -999
      np_msg.altitude_m_per_sec = npdata_dict['altitude_m_per_sec'] if np_msg.has_altitude else -999

      np_msg.has_depth = npdata_dict['has_depth']
      np_msg.time_depth = npdata_dict['time_depth'] if np_msg.has_depth else 0.0
      np_msg.depth_m = npdata_dict['depth_m'] if np_msg.has_depth else -999
      np_msg.depth_m_per_sec = npdata_dict['depth_m_per_sec'] if np_msg.has_depth else -999

      np_msg.has_pan_tilt = npdata_dict['has_pan_tilt']
      np_msg.time_pan_tilt = npdata_dict['time_pan_tilt'] if np_msg.has_pan_tilt else 0.0
      np_msg.pan_deg = npdata_dict['pan_deg'] if np_msg.has_pan_tilt else -999
      np_msg.tilt_deg = npdata_dict['tilt_deg'] if np_msg.has_pan_tilt else -999
      np_msg.pan_tilt_heading_deg = npdata_dict['pan_tilt_heading_deg'] if np_msg.has_pan_tilt else 0.0
      np_msg.pan_tilt_x_m = npdata_dict['pan_tilt_x_m'] if np_msg.has_pan_tilt else 0.0
      np_msg.pan_tilt_y_m = npdata_dict['pan_tilt_y_m'] if np_msg.has_pan_tilt else 0.0
      np_msg.pan_tilt_z_m = npdata_dict['pan_tilt_z_m'] if np_msg.has_pan_tilt else 0.0
      np_msg.pan_tilt_roll_deg = npdata_dict['pan_tilt_roll_deg'] if np_msg.has_pan_tilt else 0.0
      np_msg.pan_tilt_pitch_deg = npdata_dict['pan_tilt_pitch_deg'] if np_msg.has_pan_tilt else 0.0
      np_msg.pan_tilt_yaw_deg = npdata_dict['pan_tilt_yaw_deg'] if np_msg.has_pan_tilt else 0.0
    except Exception as e:
      np_msg = None
      logger.log_warn("Failed to convert NavPose Data dict: " + str(e), throttle_s = 5.0, log_name_list = log_name_list)
  return np_msg


def convert_navpose_msg2dict(np_msg, log_name_list = []):
  if np_msg is None:
    return None
  try:
    npdata_dict = copy.deepcopy(BLANK_NAVPOSE_DICT)
    npdata_dict['navpose_frame'] = np_msg.navpose_frame
    npdata_dict['navpose_description'] = np_msg.navpose_description
    npdata_dict['frame_nav'] = np_msg.frame_nav
    npdata_dict['frame_altitude'] = np_msg.frame_altitude
    npdata_dict['frame_depth'] = np_msg.frame_depth
    npdata_dict['geoid_height_meters'] = np_msg.geoid_height_meters
    npdata_dict['has_location'] = np_msg.has_location
    npdata_dict['latitude'] = np_msg.latitude
    npdata_dict['longitude'] = np_msg.longitude
    npdata_dict['location_m_per_sec'] = np_msg.location_m_per_sec
    npdata_dict['has_heading'] = np_msg.has_heading
    npdata_dict['heading_deg'] = np_msg.heading_deg
    npdata_dict['heading_m_per_sec'] = np_msg.heading_m_per_sec
    npdata_dict['has_orientation'] = np_msg.has_orientation
    npdata_dict['roll_deg'] = np_msg.roll_deg
    npdata_dict['pitch_deg'] = np_msg.pitch_deg
    npdata_dict['yaw_deg'] = np_msg.yaw_deg
    npdata_dict['roll_deg_per_sec'] = np_msg.roll_deg_per_sec
    npdata_dict['pitch_deg_per_sec'] = np_msg.pitch_deg_per_sec
    npdata_dict['yaw_deg_per_sec'] = np_msg.yaw_deg_per_sec
    npdata_dict['has_position'] = np_msg.has_position
    npdata_dict['x_m'] = np_msg.x_m
    npdata_dict['y_m'] = np_msg.y_m
    npdata_dict['z_m'] = np_msg.z_m
    npdata_dict['x_m_per_sec'] = np_msg.x_m_per_sec
    npdata_dict['y_m_per_sec'] = np_msg.y_m_per_sec
    npdata_dict['z_m_per_sec'] = np_msg.z_m_per_sec
    npdata_dict['has_altitude'] = np_msg.has_altitude
    npdata_dict['altitude_m'] = np_msg.altitude_m
    npdata_dict['altitude_m_per_sec'] = np_msg.altitude_m_per_sec
    npdata_dict['has_depth'] = np_msg.has_depth
    npdata_dict['depth_m'] = np_msg.depth_m
    npdata_dict['depth_m_per_sec'] = np_msg.depth_m_per_sec
    npdata_dict['has_pan_tilt'] = np_msg.has_pan_tilt
    npdata_dict['pan_deg'] = np_msg.pan_deg
    npdata_dict['tilt_deg'] = np_msg.tilt_deg
    npdata_dict['pan_tilt_heading_deg'] = np_msg.pan_tilt_heading_deg
    npdata_dict['pan_tilt_x_m'] = np_msg.pan_tilt_x_m
    npdata_dict['pan_tilt_y_m'] = np_msg.pan_tilt_y_m
    npdata_dict['pan_tilt_z_m'] = np_msg.pan_tilt_z_m
    npdata_dict['pan_tilt_roll_deg'] = np_msg.pan_tilt_roll_deg
    npdata_dict['pan_tilt_pitch_deg'] = np_msg.pan_tilt_pitch_deg
    npdata_dict['pan_tilt_yaw_deg'] = np_msg.pan_tilt_yaw_deg
  except Exception as e:
    logger.log_warn("Failed to convert NavPose msg to dict: " + str(e), throttle_s = 5.0, log_name_list = log_name_list)
    return None
  return npdata_dict


def convert_navposes_msg2dict(npsdata_msg, log_name_list = []):
  npsdata_dict = None
  for npdata_msg in npsdata_msg:
    try:
      npdata_dict = nepi_sdk.convert_msg2dict(npdata_msg)
      del npdata_dict['header']
      npsdata_dict[npdata_dict['name']] = npdata_dict
    except Exception as e:
      logger.log_warn("Failed to convert NavPose Data msg: " + str(e), throttle_s = 5.0, log_name_list = log_name_list)
  return npsdata_dict

def get_navpose_orientation_enu_degs(navpose_response):
  # Set current orientation vector (roll, pitch, yaw) in degrees enu frame
  orientation_enu_degs = [-999,-999,-999]
  if navpose_response is not None:
    if hasattr(navpose_response,'nav_pose'):
      navpose_msg = navpose_response.nav_pose
    elif hasattr(navpose_response,'timestamp'):
      navpose_msg = navpose_response
    else:
      return orientation_enu_degs
  pose_enu_o = navpose_msg.odom.pose.pose.orientation
  xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
  rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
  orientation_enu_degs = [rpy_enu_d[0],rpy_enu_d[1],rpy_enu_d[2]]
  return orientation_enu_degs

def get_navpose_orientation_ned_degs(navpose_response):
  # Set current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
  orientation_ned_degs = [-999,-999,-999]
  if navpose_response is not None:
    if hasattr(navpose_response,'nav_pose'):
      navpose_msg = navpose_response.nav_pose
    elif hasattr(navpose_response,'timestamp'):
      navpose_msg = navpose_response
    else:
      return orientation_ned_degs
  pose_enu_o = navpose_msg.odom.pose.pose.orientation
  xyzw_enu_o = list([pose_enu_o.x,pose_enu_o.y,pose_enu_o.z,pose_enu_o.w])
  rpy_enu_d = convert_quat2rpy(xyzw_enu_o)
  yaw_ned_d = convert_yaw_enu2ned(rpy_enu_d[2])
  rpy_ned_d = [rpy_enu_d[0],rpy_enu_d[1],yaw_ned_d]
  orientation_ned_degs = [rpy_ned_d[0],rpy_ned_d[1],rpy_ned_d[2]]
  return orientation_ned_degs

def get_navpose_position_enu_m(navpose_response):
  # Set current position vector (x, y, z) in meters enu frame
  position_enu_m = [-999,-999,-999]
  if navpose_response is not None:
    if hasattr(navpose_response,'nav_pose'):
      navpose_msg = navpose_response.nav_pose
    elif hasattr(navpose_response,'timestamp'):
      navpose_msg = navpose_response
    else:
      return position_enu_m
  pose_enu_p = navpose_msg.odom.pose.pose.position
  position_enu_m = [pose_enu_p.x, pose_enu_p.y, pose_enu_p.z]
  return position_enu_m

def get_navpose_position_ned_m(navpose_response):
  # Set current position vector (x, y, z) in meters ned frame
  position_ned_m = [-999,-999,-999]
  if navpose_response is not None:
    if hasattr(navpose_response,'nav_pose'):
      navpose_msg = navpose_response.nav_pose
    elif hasattr(navpose_response,'timestamp'):
      navpose_msg = navpose_response
    else:
      return position_ned_m
  pose_enu_p = navpose_msg.odom.pose.pose.position
  position_ned_m = [pose_enu_p.y, pose_enu_p.x, -pose_enu_p.z]
  return position_ned_m


def get_navpose_location_wgs84_geo(navpose_response): 
  # Set current location vector (lat, long, altitude) in geopoint data with AMSL height
  location_wgs84_geo = [-999,-999,-999]
  if navpose_response is not None:
    if hasattr(navpose_response,'nav_pose'):
      navpose_msg = navpose_response.nav_pose
    elif hasattr(navpose_response,'timestamp'):
      navpose_msg = navpose_response
    else:
      return location_wgs84_geo
  fix_wgs84 = navpose_msg.fix
  location_wgs84_geo =  [fix_wgs84.latitude,fix_wgs84.longitude,fix_wgs84.altitude]
  return location_wgs84_geo


def get_navpose_location_amsl_geo(navpose_response):  
  # Set current location vector (lat, long, altitude) in geopoint data with AMSL height
  location_amsl_geo = [-999,-999,-999]
  if navpose_response is not None:
    if hasattr(navpose_response,'nav_pose'):
      navpose_msg = navpose_response.nav_pose
    elif hasattr(navpose_response,'timestamp'):
      navpose_msg = navpose_response
    else:
      return location_amsl_geo
  geoid_height = get_navpose_geoid_height(navpose_response)
  fix_wgs84 = navpose_msg.fix
  location_amsl_geo =  [fix_wgs84.latitude,fix_wgs84.longitude,(fix_wgs84.altitude + geoid_height)]
  return location_amsl_geo

def get_navpose_geoid_height(navpose_response):
  # Set current location vector (lat, long, altitude) in geopoint data with WGS84 height
  geoid_height = -999
  if navpose_response is not None:
    if hasattr(navpose_response,'nav_pose'):
      navpose_msg = navpose_response.nav_pose
    elif hasattr(navpose_response,'timestamp'):
      navpose_msg = navpose_response
    else:
      return geoid_height
  fix_wgs84 = navpose_msg.fix
  single_position=LatLon(fix_wgs84.latitude,fix_wgs84.longitude)
  geoid_height = ginterpolator(single_position)
  geoid_height =  geoid_height
  return geoid_height
  
def get_navpose_track_msg_from_dict(navpose_dict, log_name_list = []):
    track = BLANK_NAVPOSE_TRACK_DICT
    for key in track.keys():
        if key in navpose_dict.keys():
            track[key] = navpose_dict[key]
    track_msg = NavPoseTrack()
    nepi_sdk.log_msg_debug("Got NavPoseTrack msg: " + str(track_msg), throttle_s = 5.0, log_name_list = log_name_list)
    timestamp = nepi_utils.get_time() 
    track_msg.timestamp = timestamp
    track_msg.date_time = nepi_utils.get_datetime_str_from_timestamp(timestamp)
    track_msg.latitude = track['latitude']
    track_msg.longitude = track['longitude']
    track_msg.heading_deg = track['heading_deg']
    track_msg.roll_deg = track['roll_deg']
    track_msg.pitch_deg = track['pitch_deg']
    track_msg.yaw_deg = track['yaw_deg']
    track_msg.altitude_m = track['altitude_m']
    track_msg.depth_m = track['depth_m']
    track_msg.has_pan_tilt = track['has_pan_tilt']
    track_msg.time_pan_tilt = track['time_pan_tilt']
    track_msg.pan_deg = track['pan_deg']
    track_msg.tilt_deg = track['tilt_deg']
    return track_msg


#######################
# NavPose Conversion Functions

def get_navpose_geoid_height_at_geopoint(geopoint):
  single_position=LatLon(geopoint.latitude,geopoint.longitude)
  geoid_height = ginterpolator(single_position)
  geoid_height =  geoid_height
  return geoid_height
  

def convert_amsl_to_wgs84(geopoint_in):
  geoid_height = get_navpose_geoid_height_at_geopoint(geopoint_in)
  geopoint_out = GeoPoint()
  geopoint_out.latitude = geopoint_in.latitude
  geopoint_out.longitude = geopoint_in.longitude
  geopoint_out.altitude = geopoint_in.altitude - geoid_height
  return geopoint_out
  
def convert_wgs84_to_amsl(geopoint_in):
  geoid_height = get_navpose_geoid_height_at_geopoint(geopoint_in)
  geopoint_out = GeoPoint()
  geopoint_out.latitude = geopoint_in.latitude
  geopoint_out.longitude = geopoint_in.longitude
  geopoint_out.altitude = geopoint_in.altitude + geoid_height
  return geopoint_out



### Function to Convert Quaternion Attitude to Roll, Pitch, Yaw Degrees
def convert_quat2rpy(xyzw_attitude):
  rpy_attitude_rad = tf.transformations.euler_from_quaternion(xyzw_attitude)
  rpy_attitude_deg = np.array(rpy_attitude_rad) * 180/math.pi
  roll_deg = rpy_attitude_deg[0] 
  pitch_deg = rpy_attitude_deg[1] 
  yaw_deg = rpy_attitude_deg[2]
  return rpy_attitude_deg

### Function to Convert Roll, Pitch, Yaw Degrees to Quaternion Attitude
def convert_rpy2quat(rpy_attitude_deg):
  roll_rad = math.radians(rpy_attitude_deg[0])
  pitch_rad = math.radians(rpy_attitude_deg[1]) 
  yaw_rad = math.radians(rpy_attitude_deg[2])
  #xyzw_attitude = tf.transformations.quaternion_from_euler(roll_rad,pitch_rad,yaw_rad,axes="sxyz")
  xyzw_attitude = tf.transformations.quaternion_from_euler(pitch_rad, yaw_rad, roll_rad, axes="ryzx")
  return xyzw_attitude

### Function to Convert Yaw NED to Yaw ENU
def convert_yaw_ned2enu(yaw_ned_deg):
  yaw_enu_deg = 90-yaw_ned_deg
  if yaw_enu_deg < -180:
    yaw_enu_deg = 360 + yaw_enu_deg
  elif yaw_enu_deg > 180:
    yaw_enu_deg = yaw_enu_deg - 360
  return yaw_enu_deg

### Function to Convert Yaw ENU to Yaw NED
def convert_yaw_enu2ned(yaw_enu_deg):
  yaw_ned_deg =  90-yaw_enu_deg
  if yaw_ned_deg < -180:
    yaw_ned_deg = 360 + yaw_ned_deg
  elif yaw_ned_deg > 180:
    yaw_ned_deg = yaw_ned_deg - 360
  return yaw_ned_deg


### Function to Convert Yaw from Body to ENU Frame
def convert_yaw_body2enu(yaw_body_deg,cur_heading_deg):
  cur_yaw_enu_deg = - cur_heading_deg
  if cur_yaw_enu_deg > 180: # Convert to +-180
    cur_yaw_enu_deg = cur_yaw_enu_deg - 360
  yaw_enu_deg =  cur_yaw_enu_deg + yaw_body_deg
  return yaw_enu_deg

### Function to Convert Point from Body to ENU Frame
def convert_point_body2enu(point_body_m,yaw_enu_deg):
  point_bearing_enu_deg = yaw_enu_deg + math.degrees(math.atan2(point_body_m[1],point_body_m[0]))
  point_bearing_enu_rad = math.radians(point_bearing_enu_deg)
  xy_body_m = math.sqrt(point_body_m[0]**2 + point_body_m[1]**2)
  x_enu_m = xy_body_m * math.cos(point_bearing_enu_rad)
  y_enu_m = xy_body_m * math.sin(point_bearing_enu_rad)
  point_enu_m = [x_enu_m,y_enu_m,point_body_m[2]]
  return point_enu_m
  
### Function to Convert from ENU Frame to Point from Body
def convert_point_enu2body(point_enu_m,yaw_enu_deg):
  point_bearing_body_deg = yaw_enu_deg + math.degrees(math.atan2(point_enu_m[1],point_enu_m[0]))
  point_bearing_body_rad = math.radians(point_bearing_body_deg)
  xy_enu_m = math.sqrt(point_enu_m[0]**2 + point_enu_m[1]**2)
  x_body_m = xy_enu_m * math.cos(point_bearing_body_rad)
  y_body_m = xy_enu_m * math.sin(point_bearing_body_rad)
  point_body_m = [x_body_m,y_body_m,point_enu_m[2]]
  return point_body_m


### Function to Convert Yaw from Body to NED Frame
def convert_yaw_body2ned(yaw_body_deg,cur_heading_deg):
  cur_yaw_ned_deg = cur_heading_deg
  if cur_yaw_ned_deg > 180: # Convert to +-180
    cur_yaw_ned_deg = cur_yaw_ned_deg - 360
  yaw_ned_deg =  cur_yaw_ned_deg + yaw_body_deg
  return yaw_ned_deg

### Function to Convert Point from Body to NED Frame
def convert_point_body2ned(point_body_m,yaw_ned_deg):
  point_bearing_ned_deg = yaw_ned_deg - math.degrees(math.atan2(point_body_m[1],point_body_m[0]))
  point_bearing_ned_rad = math.radians(point_bearing_ned_deg)
  xy_body_m = math.sqrt(point_body_m[0]**2 + point_body_m[1]**2)
  x_ned_m = xy_body_m * math.cos(point_bearing_ned_rad)
  y_ned_m = xy_body_m * math.sin(point_bearing_ned_rad)
  point_ned_m = [x_ned_m,y_ned_m,-point_body_m[2]]
  return point_ned_m

  
### Function to get new latlong at body relative point
def get_geopoint_at_body_point(cur_geopoint_geo, cur_bearing_enu_deg, point_body_m):
  # cur_geopoint_geo is list [Lat,Long,Alt] with Alt passed through
  earth_radius_km = 6378.137
  earth_circ_m = np.float64(2 * math.pi * earth_radius_km*1000)
  # Calculate bearing in NED frame
  point_body_m[1] = - point_body_m[1] # adjust for y = right frame from body y left frame.
  cur_bearing_ned_deg = convert_yaw_enu2ned(cur_bearing_enu_deg)
  point_bearing_ned_deg = cur_bearing_ned_deg + math.degrees(math.atan2(point_body_m[1],point_body_m[0]))
  point_bearing_ned_rad = math.radians(point_bearing_ned_deg)
  # Calculate distances NED frame
  delta_body_m = math.sqrt(point_body_m[0]**2+point_body_m[1]**2)
  delta_x_ned_m = delta_body_m * math.cos(point_bearing_ned_rad) # north:pos,south:neg
  delta_y_ned_m = delta_body_m * math.sin(point_bearing_ned_rad) # east:pos,west:neg
  # Calculate New Lat Position
  cur_lat = cur_geopoint_geo.latitude
  m_per_lat = np.float64(earth_circ_m/360)
  delta_lat = delta_x_ned_m / m_per_lat
  new_lat = cur_lat + delta_lat
  # Calculate New Long Position
  cur_long = cur_geopoint_geo.longitude
  m_per_long = m_per_lat * math.cos(math.radians(cur_lat)) 
  delta_long = delta_y_ned_m / m_per_long
  new_long = cur_long + delta_long
  #logger.log_info(point_body_m)
  #logger.log_info(cur_bearing_ned_deg)
  #logger.log_info(point_bearing_ned_deg)
  #logger.log_info(delta_x_ned_m)
  #logger.log_info(delta_y_ned_m)
  #logger.log_info(cur_geopoint_geo.latitude)
  #logger.log_info(new_lat)
  #logger.log_info(cur_geopoint_geo.longitude)
  #logger.log_info(new_long)


  # Return New Geo Position
  new_geopoint_geo=GeoPoint()
  new_geopoint_geo.latitude = new_lat
  new_geopoint_geo.longitude = new_long
  new_geopoint_geo.altitude = cur_geopoint_geo.altitude
  return  new_geopoint_geo


  ### Function to get new latlong at body relative point
def get_geopoint_at_enu_point(cur_geopoint_geo, point_enu_m):
  # cur_geopoint_geo is list [Lat,Long,Alt] with Alt passed through
  earth_radius_km = 6378.137
  earth_circ_m = np.float64(2 * math.pi * earth_radius_km*1000)
  # Calculate point in NED frame
  delta_x_ned_m = point_enu_m[1] # north:pos,south:neg
  delta_y_ned_m = point_enu_m[0] # east:pos,west:neg
  # Calculate New Lat Position
  cur_lat = cur_geopoint_geo.latitude
  m_per_lat = np.float64(earth_circ_m/360)
  delta_lat = delta_x_ned_m / m_per_lat
  new_lat = cur_lat + delta_lat
  # Calculate New Long Position
  cur_long = cur_geopoint_geo.longitude
  m_per_long = m_per_lat * math.cos(math.radians(cur_lat)) 
  delta_long = delta_y_ned_m / m_per_long
  new_long = cur_long + delta_long
  #logger.log_info(point_enu_m)
  #logger.log_info(delta_x_ned_m)
  #logger.log_info(delta_y_ned_m)
  #logger.log_info(cur_geopoint_geo.latitude)
  #logger.log_info(new_lat)
  #logger.log_info(cur_geopoint_geo.longitude)
  #logger.log_info(new_long)


  # Return New Geo Position
  new_geopoint_geo=GeoPoint()
  new_geopoint_geo.latitude = new_lat
  new_geopoint_geo.longitude = new_long
  new_geopoint_geo.altitude = cur_geopoint_geo.altitude + point_enu_m[2]
  return  new_geopoint_geo


### Function to get distance between two geo latlong locations
def distance_geopoints(geopoint1,geopoint2):
  lat1 = math.radians(geopoint1[0])
  lat2 = math.radians(geopoint2[0])
  lon1 = math.radians(geopoint1[1])
  lon2 = math.radians(geopoint2[1])
  # Haversine formula 
  dlon = (lon2 - lon1)
  dlat = (lat2 - lat1)
  a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
  c = 2 * math.asin(math.sqrt(a))

  # Radius of earth in kilometers. Use 3956 for miles
  r = 6371
  xy_m=c*r*1000
  altitude_m = abs(geopoint1[2]-geopoint2[2])
  distance_m = math.sqrt(altitude_m**2 + xy_m**2)
  #logger.log_info("Moving : " + "%.2f" % (xy_m) + " meters in xy plane")
  #logger.log_info("Moving : " + "%.2f" % (altitude_m) + " meters in z axis")
  #logger.log_info("Moving : " + "%.2f" % (distance_m) + " total meters")
 
  # calculate the result
  return(distance_m)
  
### Function to get point between two geo latlong locations
def point_from_geopoints(geopoint1,geopoint2,heading_deg):
  lat1 = math.radians(geopoint1[0])
  lat2 = math.radians(geopoint2[0])
  lon1 = math.radians(geopoint1[1])
  lon2 = math.radians(geopoint2[1])
  # Haversine formula 
  dlon = (lon2 - lon1)
  dlat = (lat2 - lat1)
  a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
  c = 2 * math.asin(math.sqrt(a))

  
  # Radius of earth in kilometers. Use 3956 for miles
  point_bearing_ned_rad = math.radians(heading_deg)
  r = 6371
  xy_m=c*r*1000
  delta_x_ned_m = xy_m * math.cos(point_bearing_ned_rad) # north:pos,south:neg
  delta_y_ned_m = xy_m * math.sin(point_bearing_ned_rad) # east:pos,west:neg
  delta_altitude_m = abs(geopoint1[2]-geopoint2[2])

  return delta_x_ned_m,delta_y_ned_m,delta_altitude_m


def rotate_enu_point(xyz_vector, angle_deg, axis='z'):
    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    
    if axis == 'x': # East
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == 'y': # North
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == 'z': # Up
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")
    new_vector = np.dot(R, xyz_vector).tolist()
    return new_vector


def rotate_enu_angles(rpy_vector, angle_deg, axis='z'):
    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    
    if axis == 'x': # East
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == 'y': # North
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == 'z': # Up
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")
    new_vector = np.dot(R, rpy_vector).tolist()

    return new_vector


################################
### Pan Tilt Utility Functions




def update_transform_from_pantilt(transform, pan_deg, tilt_deg):
    xo = transform['x_m']
    yo = transform['y_m']
    zo = transform['z_m']
    aro = 0 # transform['roll_deg']
    apo = 0 # transform['pitch_deg']
    ayo = 0 # transform['yaw_deg']

    [x,y,z,ar,ap,ay] = [xo,yo,zo,aro,apo,ayo]


    [xt,yt,zt]  = rotate_enu_point([x,y,z],tilt_deg,'y')
    [x,y,z] = [xt,yt,zt]

    [xp,yp,zp]   = rotate_enu_point([x,y,z],pan_deg,'z')
    [x,y,z] = [xp,yp,zp]

    [art,apt,ayt]  = rotate_enu_angles([ar,ap,ay],tilt_deg,'y')
    apt = apt + tilt_deg
    [ar,ap,ay] = [art,apt,ayt]

    [arp,app,ayp]  = rotate_enu_angles([ar,ap,ay],pan_deg,'z')
    ayp = ayp + pan_deg
    [ar,ap,ay] = [arp,app,ayp]

    new_transform = dict(transform)
    new_transform['x_m'] = x
    new_transform['y_m'] = y
    new_transform['z_m'] = z
    new_transform['roll_deg'] = ar
    new_transform['pitch_deg'] = ap
    new_transform['yaw_deg'] = ay
    return new_transform

# # Example Usage:
# # Current: 10m forward, 0 deg rotation
# current = [10, 0, 0, 0, 0, 0] 
# # Rotate 90 deg around Base Z
# pan = 35
# tilt = 35

# new_pose = update_transform_from_pantilt(current, pan, tilt)
# print(f"New Pose: {np.round(new_pose, 2)}")



def update_navpose_dict_pantilt(npdata_dict, transform_dict, log_name_list = []):
  success = True
  if npdata_dict is None:
    success = False
    logger.log_info("Got None navpose dict", throttle_s = 5.0)
  elif npdata_dict['has_pan_tilt'] == True:
      heading_deg = npdata_dict['heading_deg']
      if heading_deg == -999:
        heading_deg = 0
      
      npdata_dict['pan_tilt_heading_deg'] = heading_deg + npdata_dict['pan_deg']


      pt_transform = update_transform_from_pantilt(transform_dict,npdata_dict['pan_deg'],npdata_dict['tilt_deg'])

      x = pt_transform['x_m']
      y = pt_transform['y_m']
      z = pt_transform['z_m']
      roll = pt_transform['roll_deg']
      pitch = pt_transform['pitch_deg']
      yaw = pt_transform['yaw_deg']

      try:

        if npdata_dict['has_orientation'] == True:
          npdata_dict['pan_tilt_roll_deg'] = npdata_dict['roll_deg'] + roll
          npdata_dict['pan_tilt_pitch_deg'] = npdata_dict['pitch_deg'] + pitch
          npdata_dict['pan_tilt_yaw_deg'] = npdata_dict['yaw_deg'] + yaw
        else:
          npdata_dict['pan_tilt_roll_deg'] =  roll
          npdata_dict['pan_tilt_pitch_deg'] =  pitch
          npdata_dict['pan_tilt_yaw_deg'] =  yaw

        if npdata_dict['has_position'] == True:
          npdata_dict['pan_tilt_x_m'] = npdata_dict['x_m'] + x
          npdata_dict['pan_tilt_y_m'] = npdata_dict['y_m'] + y
          npdata_dict['pan_tilt_z_m'] = npdata_dict['z_m'] + z
        else:
          npdata_dict['pan_tilt_x_m'] =  x
          npdata_dict['pan_tilt_y_m'] =  y
          npdata_dict['pan_tilt_z_m'] =  z


      except Exception as e:
        success = False
        logger.log_warn("Failed to transfrom NavPose dict: " + str(e), throttle_s = 5.0, log_name_list = log_name_list)
  return npdata_dict