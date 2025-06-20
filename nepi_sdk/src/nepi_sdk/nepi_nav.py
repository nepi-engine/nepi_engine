#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI navigation utility functions include
# 1) NavPose request functions
# 2) NavPose conversion utility functions

import os
import numpy as np
import math
import time
import tf
from pygeodesy.ellipsoidalKarney import LatLon


from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import Odometry

from nepi_sdk_interfaces.msg import NavPoseMgrStatus, NavPoseMgrCompInfo, NavPoseTrack

from nepi_interfaces.msg import NavPose, DataNavPose
from nepi_interfaces.msg import NavPoseLocation, NavPoseHeading
from nepi_interfaces.msg import NavPoseOrientation, NavPosePosition
from nepi_interfaces.msg import NavPoseAltitude, NavPoseDepth

from nepi_interfaces.msg import Frame3DTransform, Frame3DTransforms
from nepi_interfaces.srv import Frame3DTransformsQuery, Frame3DTransformsQueryRequest, Frame3DTransformsQueryResponse
from nepi_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, NavPoseQueryResponse

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils


from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_nav"
logger = Logger(log_name = log_name)


# try and import geoid height calculation module and databases
GEOID_DATABASE_FILE='/opt/nepi/databases/geoids/egm2008-2_5.pgm' # Ignored if PyGeodesy module or Geoids Database is not available
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
    msg_type = 'nepi_interfaces/DataNavPose'
    return nepi_sdk.find_topics_by_msg(msg_type)

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
        }
    }


def get_navpose_comp_publisher_namespaces(name):
    topic_list = []
    msg_list = []
    if name in NAVPOSE_MSG_DICT.keys():
      msg_str_list = list(NAVPOSE_MSG_DICT[name].keys())
      [topics_list,msg_list] = nepi_sdk.find_topics_by_msgs(msg_str_list)
    return topic_list,msg_list




def update_navpose_dict_from_msg(name, navpose_dict, msg, transform = None):
    msg_type = msg._type
    npdata_dict = BLANK_NAVPOSE_DICT
    
    if name == 'location':
      if msg_type == 'nepi_interfaces/NavPoseLocation':
        try:
          npdata_dict['time_location'] = msg.timestamp
          npdata_dict['latitude'] = msg.latitude
          npdata_dict['longitude'] = msg.longitude
        except:
          pass
      elif msg_type == 'sensor_msgs/NavSatFix':
        try:
          npdata_dict['time_location'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          npdata_dict['latitude'] = msg.latitude
          npdata_dict['longitude'] = msg.longitude
        except:
          pass
      elif msg_type == 'geographic_msgs/GeoPoint':
        try:
          npdata_dict['time_location'] = nepi_utils.get_time()
          npdata_dict['latitude'] = msg.latitude
          npdata_dict['longitude'] = msg.longitude
        except:
          pass

    elif name == 'heading':
      if msg_type == 'nepi_interfaces/NavPoseHeading':
        try:
          npdata_dict['time_heading'] = msg.timestamp
          npdata_dict['heading_deg'] = msg.heading_deg
        except:
          pass
     
          
    if name == 'orientation':
      if msg_type == 'nepi_interfaces/NavPoseOrientation':
        try:
          npdata_dict['time_orientation'] = msg.timestamp
          npdata_dict['roll_deg'] = msg.roll_deg
          npdata_dict['pitch_deg'] = msg.pitch_deg
          npdata_dict['yaw_deg'] = msg.yaw_deg
        except:
          pass
      elif msg_type == 'nav_msgs/Odometry':
        try:
          data = msg.pose.pose.orientation
          quat = [data.x,data.y,data.z,data.w]
          [roll,pitch,yaw] = convert_quat2rpy(quat)
          npdata_dict['time_orientation'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          npdata_dict['roll_deg'] = roll
          npdata_dict['pitch_deg'] = pitch
          npdata_dict['yaw_deg'] = yaw
        except:
          pass
      elif msg_type == 'geometry_msgs/Pose':
        try:
          data = msg.orientation
          quat = [data.x,data.y,data.z,data.w]
          [roll,pitch,yaw] = convert_quat2rpy(quat)
          npdata_dict['time_orientation'] = nepi_utils.get_time()
          npdata_dict['roll_deg'] = roll
          npdata_dict['pitch_deg'] = pitch
          npdata_dict['yaw_deg'] = yaw
        except:
          pass
      elif msg_type == 'geometry_msgs/Quaternion':
        try:
          data = msg
          quat = [data.x,data.y,data.z,data.w]
          [roll,pitch,yaw] = convert_quat2rpy(quat)
          npdata_dict['time_orientation'] = nepi_utils.get_time()
          npdata_dict['roll_deg'] = roll
          npdata_dict['pitch_deg'] = pitch
          npdata_dict['yaw_deg'] = yaw
        except:
          pass

    if name == 'position':
      if msg_type == 'nepi_interfaces/NavPoseOrientation':
        try:
          npdata_dict['time_position'] = msg.timestamp
          npdata_dict['x_m'] = msg.x_m
          npdata_dict['y_m'] = msg.y_m
          npdata_dict['z_m'] = msg.y_m
        except:
          pass
      elif msg_type == 'nav_msgs/Odometry':
        try:
          data = msg.pose.pose.position
          pos = [data.x,data.y,data.z]
          npdata_dict['time_position'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          npdata_dict['roll_deg'] = pos[0]
          npdata_dict['pitch_deg'] = pos[1]
          npdata_dict['yaw_deg'] = pos[2]
        except:
          pass
      elif msg_type == 'geometry_msgs/Pose':
        try:
          data = msg.position
          pos = [data.x,data.y,data.z]
          npdata_dict['time_position'] = nepi_utils.get_time()
          npdata_dict['roll_deg'] = pos[0]
          npdata_dict['pitch_deg'] = pos[1]
          npdata_dict['yaw_deg'] = pos[2]
        except:
          pass
      elif msg_type == 'geometry_msgs/Point':
        try:
          data = msg
          pos = [data.x,data.y,data.z]
          npdata_dict['time_position'] = nepi_utils.get_time()
          npdata_dict['roll_deg'] = pos[0]
          npdata_dict['pitch_deg'] = pos[1]
          npdata_dict['yaw_deg'] = pos[2]
        except:
          pass

   
    if name == 'altitude':
      if msg_type == 'nepi_interfaces/NavPosePosition':
        try:
          npdata_dict['time_altitude'] = msg.timestamp
          npdata_dict['altitude_m'] = msg.altitude_m
        except:
          pass
      elif msg_type == 'sensor_msgs/NavSatFix':
        try:
          npdata_dict['time_altitude'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          npdata_dict['altitude_m'] = msg.altitude
        except:
          pass
      elif msg_type == 'geographic_msgs/GeoPoint':
        try:
          npdata_dict['time_altitude'] = nepi_utils.get_time()
          npdata_dict['altitude_m'] = msg.altitude
        except:
          pass

    if name == 'depth':
      if msg_type == 'nepi_interfaces/NavPosePosition':
        try:
          npdata_dict['time_depth'] = msg.timestamp
          npdata_dict['depth_m'] = msg.depth_m
        except:
          pass
      elif msg_type == 'sensor_msgs/NavSatFix':
        try:
          npdata_dict['time_depth'] = nepi_sdk.sec_from_msg_stamp(msg.header.stamp)
          npdata_dict['depth_m'] = msg.depth
        except:
          pass
      elif msg_type == 'geographic_msgs/GeoPoint':
        try:
          npdata_dict['time_depth'] = nepi_utils.get_time()
          npdata_dict['depth_m'] = msg.depth
        except:
          pass
      
      if transform is not None:
        if transform != ZERO_TRANSFORM:
          npdata_dict = transform_navpose_dict(npdata_dict,transform)
    return npdata_dict


#######################
# 3D Frame Helper Functions

ZERO_TRANSFORM = [0,0,0,0,0,0,0]


def convert_transform_list2msg(transform_list, source_ref_description = '', end_ref_description = ''):
  transform_msg = Frame3DTransform()
  transform_msg.source_ref_description = source_ref_description
  transform_msg.end_ref_description = end_ref_description
  if len(transform_list) == 7:
      transform_msg.translate_vector.x = transform_list[0]
      transform_msg.translate_vector.y = transform_list[1]
      transform_msg.translate_vector.z = transform_list[2]
      transform_msg.rotate_vector.x = transform_list[3]
      transform_msg.rotate_vector.y = transform_list[4]
      transform_msg.rotate_vector.z = transform_list[5]
      transform_msg.heading_offset = transform_list[6]
  return transform_msg

def convert_transform_msg2list(transform_msg):
  transform_list = ZERO_TRANSFORM
  if len(transform_list) == 7:
      transform_list[0] = transform_msg.translate_vector.x
      transform_list[1] = transform_msg.translate_vector.y
      transform_list[2] = transform_msg.translate_vector.z
      transform_list[3] = transform_msg.rotate_vector.x
      transform_list[4] = transform_msg.rotate_vector.y
      transform_list[5] = transform_msg.rotate_vector.z
      transform_list[6] = transform_msg.heading_offset
  return transform_list

def transform_navpose_dict(npdata_dict, transform, output_frame_3d = 'nepi_frame'):
  success = True
  if npdata_dict is None:
    success = False
    logger.log_info("Got None navpose dict", throttle_s = 5.0)
  else:
    if transform != ZERO_TRANSFORM:
      x = transform[0]
      y = transform[1]
      z = transform[2]
      translation_vector = [x, y, z]
      roll = transform[3]
      pitch = transform[4]
      yaw = transform[5]
      rotate_vector = [roll, pitch, yaw]
      navpose_dict = BLANK_NAVPOSE_DICT
      try:

        navpose_dict['frame_3d'] = output_frame_3d

        if npdata_dict['has_location'] == True:
          navpose_dict['latitude'] = npdata_dict['latitude']
          navpose_dict['longitude'] = npdata_dict['longitude']

        if npdata_dict['has_heading'] == True:
          navpose_dict['heading_deg'] = npdata_dict['heading_deg'] - yaw

        if npdata_dict['has_orientation'] == True:
          navpose_dict['roll_deg'] = npdata_dict['roll_deg'] - roll
          navpose_dict['pitch_deg'] = npdata_dict['pitch_deg'] - pitch
          navpose_dict['yaw_deg'] = npdata_dict['yaw_deg'] - yaw

        if npdata_dict['has_position'] == True:
          navpose_dict['x_m'] = npdata_dict['x_m'] - x
          navpose_dict['y_m'] = npdata_dict['y_m'] - y
          navpose_dict['z_m'] = npdata_dict['z_m'] - z


        if npdata_dict['has_altitude'] == True:
          navpose_dict['altitude_m'] = npdata_dict['altitude_m'] - z

        if npdata_dict['has_depth'] == True:
          navpose_dict['depth_m'] = npdata_dict['depth_m'] - z

      except Exception as e:
        success = False
        logger.log_warn("Failed to transfrom NavPose dict: " + str(e), throttle_s = 5.0)
    if success == True:
      pass
  return npdata_dict


#######################
# NavPose Data Helper Functions

NAVPOSE_3D_FRAME_OPTIONS = ['base_frame','nepi_frame','sensor_frame','world_frame']
NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
NAVPOSE_DEPTH_FRAME_OPTIONS = ['MSL','TOC','DF','KB','DEPTH','UKNOWN']

BLANK_HEADING_DATA_DICT = {
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 0.0,
}

BLANK_POSITION_DATA_DICT = {
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters ENU (x,y,z) with x forward, y left, and z up
    'x_m': 0.0,
    'y_m': 0.0,
    'z_m': 0.0,
}

BLANK_ORIENTATION_DATA_DICT = {
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees ENU
    'roll_deg': 0.0,
    'pitch_deg': 0.0,
    'yaw_deg': 0.0,
}

BLANK_LOCATION_DATA_DICT = {
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'latitude': 0.0,
    'longitude': 0.0
}

BLANK_ALTITUDE_DATA_DICT = {
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters WGS84
    'altitude_m': 0.0,
}

BLANK_DEPTH_DATA_DICT = {
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive distance from surface in meters
    'depth_m': 0.0
}



BLANK_NAVPOSE_DICT = {
    'frame_3d': 'sensor_frame',
    'frame_nav': 'ENU',
    'frame_altitude': 'WGS84',
    'frame_depth': 'MSL',

    'geoid_height_meters': 0.0,

    'has_location': False,
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'latitude': 0.0,
    'longitude': 0.0,

    'has_heading': False,
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 0.0,

    'has_position': False,
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters in specified 3d frame (x,y,z) with x forward, y right/left, and z up/down
    'x_m': 0.0,
    'y_m': 0.0,
    'z_m': 0.0,

    'has_orientation': False,
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 0.0,
    'pitch_deg': 0.0,
    'yaw_deg': 0.0,

    'has_altitude': False,
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters in specified alt frame
    'altitude_m': 0.0,

    'has_depth': False,
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive meters
    'depth_m': 0.0
}


BLANK_NAVPOSE_TRACK_DICT = {
    # altitude in 'WGS84' frame
    # depth in 'DEPTH' frame
    'timestamp': nepi_utils.get_time(),
    # Location Lat,Long
    'latitude': 0.0,
    'longitude': 0.0,
    'heading_deg': 0.0,
    'roll_deg': 0.0,
    'pitch_deg': 0.0,
    'yaw_deg': 0.0,
    'altitude_m': 0.0,
    'depth_m': 0.0
}


def update_navpose_dict_from_dict(npdata_dict_org,npdata_dict_new,output_frame_3d = 'nepi_frame'):
    success = False
    np_dict = copy.deepcopy(np_dict_org)
    if npdata_dict_org is not None and npdata_dict_new is not None:
      try:
        npdata_dict_org['frame_3d'] = output_frame_3d
        if npdata_dict_new['has_location'] == True:
            npdata_dict_org['time_location'] = npdata_dict_new['time_location']
            npdata_dict_org['latitude'] = npdata_dict_new['latitude']
            npdata_dict_org['longitude'] = npdata_dict_new['longitude']
        if npdata_dict_new['has_heading'] == True:
            npdata_dict_org['time_heading'] = npdata_dict_new['time_heading']
            npdata_dict_org['heading'] = npdata_dict_new['heading']
        if npdata_dict_new['has_orientation'] == True:
            npdata_dict_org['time_orientation'] = npdata_dict_new['time_orientation']
            npdata_dict_org['roll_deg'] = npdata_dict_new['roll_deg']
            npdata_dict_org['pitch_deg'] = npdata_dict_new['pitch_deg']
            npdata_dict_org['yaw_deg'] = npdata_dict_new['yaw_deg']
        if npdata_dict_new['has_position'] == True:
            npdata_dict_org['time_position'] = npdata_dict_new['time_position']
            npdata_dict_org['x_m'] = npdata_dict_new['x_m']
            npdata_dict_org['y_m'] = npdata_dict_new['y_m']
            npdata_dict_org['z_m'] = npdata_dict_new['z_m']
        if npdata_dict_new['has_altitude'] == True:
            npdata_dict_org['time_altitude'] = npdata_dict_new['time_altitude']
            npdata_dict_org['altitude_m'] = npdata_dict_new['altitude_m']
        if npdata_dict_new['has_depth'] == True:
            npdata_dict_org['time_detph'] = npdata_dict_new['time_depth']
            npdata_dict_org['depth_m'] = npdata_dict_new['depth_m']
        np_dict = npdata_dict_org
      except:
        pass
    return np_dict


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



def convert_navpose_dict2msg(npdata_dict):
  npdata_msg = None
  if npdata_dict is None:
    logger.log_info("Got None navpose dict", throttle_s = 5.0)
  else:
    try:
      np_msg = NavPose()
      np_msg.header.stamp = nepi_sdk.get_msg_stamp()

      np_msg.frame_3d = npdata_dict['frame_3d']
      np_msg.frame_nav = npdata_dict['frame_nav']
      np_msg.frame_altitude = npdata_dict['frame_altitude']
      np_msg.frame_depth = npdata_dict['frame_depth']

      np_msg.geoid_height_meters = npdata_dict['geoid_height_meters']

      np_msg.has_heading = npdata_dict['has_heading']
      np_msg.time_heading = npdata_dict['time_heading']
      np_msg.heading_deg = npdata_dict['heading_deg']

      np_msg.has_orientation = npdata_dict['has_orientation']
      np_msg.time_orientation = npdata_dict['time_orientation']
      np_msg.roll_deg = npdata_dict['roll_deg']
      np_msg.pitch_deg = npdata_dict['pitch_deg']
      np_msg.yaw_deg = npdata_dict['yaw_deg']

      np_msg.has_position = npdata_dict['has_position']
      np_msg.time_position = npdata_dict['time_position']
      np_msg.x_m = npdata_dict['x_m']
      np_msg.y_m = npdata_dict['y_m']
      np_msg.z_m = npdata_dict['z_m']

      np_msg.has_location = npdata_dict['has_location']
      np_msg.time_location = npdata_dict['time_location']
      np_msg.latitude = npdata_dict['latitude']
      np_msg.longitude = npdata_dict['longitude']

      np_msg.has_altitude = npdata_dict['has_altitude']
      np_msg.time_altitude = npdata_dict['time_altitude']
      np_msg.altitude_m = npdata_dict['altitude_m']

      np_msg.has_depth = npdata_dict['has_depth']
      np_msg.time_depth = npdata_dict['time_depth']
      np_msg.depth_m = npdata_dict['depth_m']
    except Exception as e:
      np_msg = None
      logger.log_warn("Failed to convert NavPose Data dict: " + str(e), throttle_s = 5.0)
  return np_msg

def convert_navpose_msg2dict(npdata_msg):
  npdata_dict = None
  try:
    npdata_dict = nepi_sdk.convert_msg2dict(npdata_msg)
    del npdata_dict['header']
  except Exception as e:
    logger.log_warn("Failed to convert NavPose Data msg: " + str(e), throttle_s = 5.0)
  return npdata_dict



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
  
def get_navpose_track_msg_from_dict(navpose_dict):
    track = BLANK_NAVPOSE_TRACK_DICT
    for key in track.keys():
        if key in navpose_dict.keys():
            track[key] = navpose_dict[key]
    track_msg = NavPoseTrack()
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


