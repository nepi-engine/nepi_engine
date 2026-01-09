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
import time 
import copy
import numpy as np
import copy
import threading

os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import open3d as o3d

import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped


from nepi_interfaces.msg import NavPose, NavPoseStatus
from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import DepthMapStatus
from nepi_interfaces.msg import IntensityMapStatus
from nepi_interfaces.msg import PointcloudStatus

from nepi_interfaces.msg import NavPoseTrack
from nepi_interfaces.msg import NavPoseLocation, NavPoseHeading
from nepi_interfaces.msg import NavPoseOrientation, NavPosePosition
from nepi_interfaces.msg import NavPoseAltitude, NavPoseDepth
from nepi_interfaces.msg import NavPosePanTilt
from nepi_interfaces.srv import NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryRequest, NavPoseCapabilitiesQueryResponse



from nepi_interfaces.msg import StringArray, UpdateState, UpdateRatio, ImageWindow, RangeWindow, ImagePixel
from nepi_interfaces.srv import ImageCapabilitiesQuery, ImageCapabilitiesQueryRequest, ImageCapabilitiesQueryResponse

from nepi_interfaces.msg import RangeWindow

from sensor_msgs.msg import PointCloud2

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF



##################################################


API_LIB_FOLDER = "/opt/nepi/nepi_engine/lib/nepi_api"

SYSTEM_DATA_FOLDER = 'mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']


EXAMPLE_FILENAME_DICT = {
        'prefix': "", 
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }



class ReadWriteIF:

    ready = False
    
    # Save data variables
    filename_dict = {
        'prefix': "",
        'suffix': "",
        'add_timestamp': True, 
        'use_utc_tz': True,
        'add_ms': True,
        'add_us': False,
        'add_tz': True,
        'add_node_name': True
        }

    node_if = None

    #######################
    ### IF Initialization
    def __init__(self,
                filename_dict = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)
        

        #############################
        # Initialize Class Variables
        if filename_dict is not None:
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]

        self.data_dict = {
            'dict': {
                'data_type': dict,
                'file_types': ['yaml'],
                'read_function': self.read_dict_file,
                'write_function': self.write_dict_file
            },
            'image': {
                'data_type': np.ndarray,
                'file_types': ['png','PNG','jpg','jpeg','JPG'],
                'read_function': self.read_image_file,
                'write_function': self.write_image_file 
            },
            'pointcloud': {
                'data_type': o3d.geometry.PointCloud,
                'file_types': ['pcd'],
                'read_function': self.read_pointcloud_file,
                'write_function': self.write_pointcloud_file 
            },
        }

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready   
        

    def get_supported_data_dict(self):
        return self.data_dict

    def get_data_folder(self):
        return self.save_folder

    def set_data_folder(self, data_path):
        success = True
        if os.path.exists(data_path) == False:
            self.msg_if.pub_warn("File path does not exist: " + data_path, log_name_list = self.log_name_list)
        elif os.path.isdir(data_path) == False:
            self.msg_if.pub_warn("File path not a folder: " + data_path, log_name_list = self.log_name_list)
        else:
            self.save_folder = data_path
            success = True
        return success

    def reset_data_folder(self):
        self.save_folder = SYSTEM_DATA_FOLDER


    def get_filename_prefix(self):
        return self.filename_dict['prefix']

    def set_filename_prefix(self, prefix = ''):
        self.filename_dict['prefix'] = prefix

    def get_use_utc_tz(self):
        return self.filename_dict['use_utc_tz']

    def set_use_utc_tz(self, use_utc_tz):
        self.filename_dict['use_utc_tz'] = prefix

    def get_add_timestamp(self):
        return self.filename_dict['add_timestamp']

    def set_add_timestamp(self, add_timestamp):
        self.filename_dict['add_timestamp'] = add_timestamp

    def get_add_ms(self):
        return self.filename_dict['add_ms']

    def set_add_ms(self, add_ms):
        self.filename_dict['add_ms'] = add_ms

    def get_add_us(self):
        return self.filename_dict['add_us']

    def set_add_us(self, add_us):
        self.filename_dict['add_us'] = add_us

    def get_add_tz(self):
        return self.filename_dict['add_tz']

    def set_add_tz(self, add_tz):
        self.filename_dict['add_tz'] = prefix


    def update_filename_dict(self, filename_dict):
        if filename_dict is not None:
            for key in self.filename_dict.keys():
                if key in filename_dict.keys():
                    self.filename_dict[key] = filename_dict[key]

    def get_filename_dict(self):
        return self.filename_dict


    def get_folder_files(self, path, ext_str = ""):
        file_list = nepi_utils.get_file_list(path,ext_str=ext_str)
        return file_list

    def get_time_from_filename(self,filename):
        file_time = None
        dt_str = self._getDtStr(filename)
        file_time = nepi_utils.get_time_from_datetime_str(dt_str)
        return file_time



    def write_data_file(self, filepath, data, data_name, timestamp = None, timezone = None):
        data_type = type(data)
        found_type = False
        for data_key in self.data_dict:
            if data_type == self.data_dict[data_key]['data_type']:
                save_function = self.data_dict[data_key]['write_function']
                self.msg_if.pub_debug("Saving Data with Timezone: " + str(timezone), log_name_list = self.log_name_list, throttle_s = 5.0)
                save_function(filepath, data, data_name, timestamp = timestamp, timezone = timezone)
                found_type = True
        if found_type == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type) , log_name_list = self.log_name_list)


    def read_dict_file(self, filepath, filename):
        data_key = 'dict'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
            else:
                data = nepi_utils.read_yaml_2_dict(file_path)
        return data

    def write_dict_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'yaml'):
        data_key = 'dict'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    success = nepi_utils.write_dict_2_yaml(data, file_path)
        return success


    def read_image_file(self, filepath, filename):
        data_key = 'image'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
            else:
                data = nepi_img.read_image_file(file_path)
        return data

    def write_image_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'png'):
        data_key = 'image'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(type(data)), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    success = nepi_img.write_image_file(data, file_path)
        return success


    def read_pointcloud_file(self, filepath, filename):
        data_key = 'pointcloud'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
        else:
            file_path = os.path.join(filepath,filename)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
            else:
                data = nepi_pc.read_pointcloud_file(file_path)
        return data

    def write_pointcloud_file(self, filepath, data, data_name, timestamp = None, timezone = None, ext_str = 'pcd'):
        data_key = 'pointcloud'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type), log_name_list = self.log_name_list)
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types), log_name_list = self.log_name_list)
            else:
                filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path, log_name_list = self.log_name_list)
                else:
                    success = nepi_pc.write_pointcloud_file(file_path)
        return success


    def get_example_filename(self, data_name = 'data_product', timestamp = None, timezone = None, ext_str = 'ext'):
        filename = self._createFileName(data_name, timestamp = timestamp, timezone = timezone, ext_str = ext_str)
        return filename
    ###############################
    # Class Private Methods
    ###############################

    def _createFileName(self, data_name_str, timestamp = None, timezone = None, ext_str = ""):
        if timestamp == None:
            timestamp = nepi_utils.get_time()
        prefix = self.filename_dict['prefix']
        if len(prefix) > 0:
            if prefix[-1] != '_':
                prefix = prefix + '_'
        suffix = self.filename_dict['suffix']
        if len(suffix) > 0:
            if suffix[0] != '_':
                suffix = '_' + suffix
        add_time = self.filename_dict['add_timestamp']
        data_time_str = ''
        if add_time == True:
            time_ns = nepi_sdk.sec_from_timestamp(timestamp)
            add_ms = self.filename_dict['add_ms']
            add_us = self.filename_dict['add_us']
            add_tz = self.filename_dict['add_tz']
            data_time_str = nepi_utils.get_datetime_str_from_timestamp(time_ns, add_ms = add_ms, add_us = add_us, add_tz = add_tz, timezone = timezone) + '_'
        node_name_str = ""
        if self.filename_dict['add_node_name'] == True:
            node_name_str = self.node_name
        if len(ext_str) > 0:
            ext_str  = '.' + ext_str
        if len(data_name_str) >  0:
            data_name_str = '-' + data_name_str
        filename = prefix + data_time_str + node_name_str + data_name_str + suffix + ext_str
        return filename

    
    def _getDtStr(self,filename):
        d_inds = nepi_utils.find_all_indexes(filename, 'D')
        dt_ind = None
        dt_str = filename
        for ind in d_inds:
            if len(filename) >= ind + 1:
                if filename[ind + 1].isdigit() == True:
                    dt_ind = ind
                    break
        dt_str = dt_str[dt_ind:]
        dt_str = dt_str.split('_')[0]
        return dt_str
                



##################################################


EXAMPLE_NAVPOSE_DATA_DICT = {
    'frame_3d': 'nepi_frame',
    'frame_nav': 'ENU',
    'frame_altitude': 'WGS84',
    'frame_depth': 'MSL',

    'geoid_height_meters': 0,


    'has_location': True,
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'latitude': 47.080909,
    'longitude': -120.8787889,

    'has_heading': True,
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 120.50,

    'has_position': True,
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters in specified 3d frame (x,y,z) with x forward, y right/left, and z up/down
    'x_m': 1.234,
    'y_m': 1.234,
    'z_m': 1.234,

    'has_orientation': True,
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,

    'has_altitude': True,
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters in specified altitude_m frame
    'altitude_m': 12.321,

    'has_depth': False,
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive meters
    'depth_m': 0.0,

    'has_pan_tilt': False,
    'time_pan_tilt': nepi_utils.get_time(),
    # Pan Tilt should be provided in positive degs
    'pan_deg': 0.0,
    'tilt_deg': 0.0
}




class NavPoseIF:

    NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
    NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','UKNOWN'] # ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
    NAVPOSE_DEPTH_FRAME_OPTIONS = ['DEPTH','UKNOWN'] # ['MSL','TOC','DF','KB','DEPTH','UKNOWN']

    ready = False
    namespace = '~'

    node_if = None

    status_msg = NavPoseStatus()

    last_pub_time = None

    has_subs = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    frame_3d = 'sensor_frame'
    frame_nav = 'ENU'
    frame_altitude = 'WGS84'
    frame_depth = 'MSL'

    navpose_frames_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_FRAMES_DICT)

    caps_report = NavPoseCapabilitiesQueryResponse()

    data_product = 'navpose'



    def __init__(self, namespace = None,
                data_source_description = 'navpose',
                data_ref_description = 'sensor_center',
                pub_location = False, pub_heading = False,
                pub_orientation = False, pub_position = False,
                pub_altitude = False, pub_depth = False,
                pub_pan_tilt = False,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        # Initialize Class Variables
        if namespace is not None:
            self.namespace = namespace
        namespace = nepi_sdk.create_namespace(namespace,'navpose')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        
        self.pub_location = pub_location
        self.pub_heading = pub_heading
        self.pub_orientation = pub_orientation
        self.pub_position = pub_position
        self.pub_altitude = pub_altitude
        self.pub_depth = pub_depth
        self.pub_pan_tilt = pub_pan_tilt

        # Create Capabilities Report

        self.caps_report.has_location_pub = self.pub_location
        self.caps_report.has_heading_pub = self.pub_heading
        self.caps_report.has_position_pub = self.pub_position
        self.caps_report.has_orientation_pub = self.pub_orientation
        self.caps_report.has_depth_pub = self.pub_depth
        self.caps_report.has_pan_tilt = self.pub_pan_tilt



        # Initialize status message
        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description
        
        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description
        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Services Config Dict ####################     
        self.SRVS_DICT = {
            'navpose_caps_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': NavPoseCapabilitiesQuery,
                'req': NavPoseCapabilitiesQueryRequest(),
                'resp': NavPoseCapabilitiesQueryResponse(),
                'callback': self._provide_capabilities
            }
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None



        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'data_pub': {
                'msg': NavPose,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg': NavPoseStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }

        }

        if self.pub_location == True:
            self.PUBS_DICT['location_pub'] = {
                'msg': NavPoseLocation,
                'namespace': self.namespace,
                'topic': 'location',
                'qsize': 1,
                'latch': False
            }
            
        if self.pub_orientation == True:
            self.PUBS_DICT['orientation_pub'] = {
                'msg': NavPoseOrientation,
                'namespace': self.namespace,
                'topic': 'orientation',
                'qsize': 1,
                'latch': False
            }

        if self.pub_position == True:
            self.PUBS_DICT['position_pub'] = {
                'msg': NavPosePosition,
                'namespace': self.namespace,
                'topic': 'position',
                'qsize': 1,
                'latch': False
            }

        if self.pub_heading == True:
            self.PUBS_DICT['heading_pub'] = {
                'msg': NavPoseHeading,
                'namespace': self.namespace,
                'topic': 'heading',
                'qsize': 1,
                'latch': False
            }

        if self.pub_altitude == True:
            self.PUBS_DICT['altitude_pub'] = {
                'msg': NavPoseAltitude,
                'namespace': self.namespace,
                'topic': 'altitude',
                'qsize': 1,
                'latch': False
            }

        if self.pub_depth == True:
            self.PUBS_DICT['depth_pub'] = {
                'msg': NavPoseDepth,
                'namespace': self.namespace,
                'topic': 'depth',
                'qsize': 1,
                'latch': False
            }

        if self.pub_pan_tilt == True:
            self.PUBS_DICT['pan_tilt_pub'] = {
                'msg': NavPosePanTilt,
                'namespace': self.namespace,
                'topic': 'pan_tilt',
                'qsize': 1,
                'latch': False
            }


        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'reset': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetCb, 
                'callback_args': ()
            }
        }
        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                         msg_if = self.msg_if
                                            )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)
        nepi_sdk.start_timer_process(1.0, self._navposeFramesCheckCb, oneshot = True)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  


    def get_frame_nav_options(self):
        return NAVPOSE_NAV_FRAME_OPTIONS

    def get_frame_altitude_options(self):
        return NAVPOSE_NAV_FRAME_OPTIONS

    def get_frame_depth_options(self):
        return NAVPOSE_DEPTH_FRAME_OPTIONS

    def get_data_products(self):
        return [self.data_product]

    def get_blank_navpose_dict(self):
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        has_subs = copy.deepcopy(self.has_subs)
        # self.msg_if.pub_debug("Returning: " + self.namespace + " " "has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        return has_subs


    # Update System Status
    def publish_navpose(self,navpose_dict, 
                        timestamp = None, 
                        frame_3d_transform = None,
                        device_mount_description = 'fixed'):      
        np_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if navpose_dict is None and self.status_msg is not None:
            return np_dict
        else:
            # Initialize np_dict here so it's available in both branches
            for key in np_dict.keys():
                if key in navpose_dict.keys():
                    np_dict[key] = navpose_dict[key]
            
            self.msg_if.pub_debug("Start Navpose data dict: " + str(np_dict), log_name_list = self.log_name_list, throttle_s = 5.0)

            if timestamp == None:
                timestamp = nepi_utils.get_time()
            else:
                timestamp = nepi_sdk.sec_from_timestamp(timestamp)

            current_time = nepi_utils.get_time()
            get_latency = (current_time - timestamp)
            self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(get_latency), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Start Img Pub Process
            start_time = nepi_utils.get_time()   


            # Transform navpose data frames to nepi standard frames
            if np_dict['frame_nav'] != 'ENU':
                if np_dict['frame_nav'] == 'NED':
                    nepi_nav.convert_navpose_ned2enu(np_dict)
            if np_dict['frame_altitude'] != 'WGS84':
                if np_dict['frame_altitude'] == 'AMSL':
                    nepi_nav.convert_navpose_amsl2wgs84(np_dict)
            if np_dict['frame_depth'] != 'MSL':
                if np_dict['frame_depth'] == 'DEPTH':
                    pass # need to add conversions                 

            self.status_msg.pub_frame_nav = np_dict['frame_nav']
            self.status_msg.pub_frame_altitude = np_dict['frame_altitude']
            self.status_msg.pub_frame_depth = np_dict['frame_depth']

            # Publish nav pose subs
            if self.pub_location == True:
                pub_name = 'location_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_location']
                msg.latitude = np_dict['latitude']
                msg.longitude = np_dict['longitude']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_heading == True:
                pub_name = 'heading_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_heading']
                msg.heading_deg = np_dict['heading_deg']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_orientation == True:
                pub_name = 'orientation_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_orientation']
                msg.roll_deg = np_dict['roll_deg']
                msg.pitch_deg = np_dict['pitch_deg']
                msg.yaw_deg = np_dict['yaw_deg']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_position == True:
                pub_name = 'position_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_position']
                msg.x_m = np_dict['x_m']
                msg.y_m = np_dict['y_m']
                msg.z_m = np_dict['z_m']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_altitude == True:
                pub_name = 'altitude_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_altitude']
                msg.altitude_m = np_dict['altitude_m']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_depth == True:
                pub_name = 'depth_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_depth']
                msg.depth_m = np_dict['depth_m']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_pan_tilt == True:
                pub_name = 'pan_tilt_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_depth']
                msg.pan_deg = np_dict['pan_deg']
                msg.tilt_deg = np_dict['tilt_deg']
                self.node_if.publish_pub(pub_name,msg)

            # Transform navpose in ENU and WSG84 frames
            if frame_3d_transform is not None:
                np_dict = nepi_nav.transform_navpose_dict(np_dict,frame_3d_transform)
                        
           # Transform navpose data frames to system set frames
            frame_nav = self.navpose_frames_dict['frame_nav']
            frame_alt = self.navpose_frames_dict['frame_alt']
            frame_depth = self.navpose_frames_dict['frame_depth']
            
            if np_dict['frame_nav'] != frame_nav:
                if np_dict['frame_nav'] == 'NED' and frame_nav == 'ENU':
                    nepi_nav.convert_navpose_ned2enu(np_dict)
                elif np_dict['frame_nav'] == 'ENU' and frame_nav == 'NED':
                    nepi_nav.convert_navpose_enu2ned(np_dict)
            if np_dict['frame_altitude'] != frame_alt:
                if np_dict['frame_altitude'] == 'AMSL' and frame_alt ==  'WGS84':
                    nepi_nav.convert_navpose_amsl2wgs84(np_dict)
                elif np_dict['frame_altitude'] == 'WGS84' and frame_alt ==  'AMSL':
                    nepi_nav.convert_navpose_wgs842amsl(np_dict)
            #if np_dict['frame_depth'] != 'MSL':
            #    if np_dict['frame_depth'] == 'DEPTH':
            #        pass # need to add conversions                 

            self.status_msg.pub_frame_nav = np_dict['frame_nav']
            self.status_msg.pub_frame_altitude = np_dict['frame_altitude']
            self.status_msg.pub_frame_depth = np_dict['frame_depth']


            data_msg = None
            try:
                data_msg = nepi_nav.convert_navpose_dict2msg(np_dict)
            except Exception as e:
                self.msg_if.pub_warn("Failed to convert navpose data to msg: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
                success = False




            if data_msg is not None:
                try:
                    self.node_if.publish_pub('data_pub', data_msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish navpose data msg: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
                    success = False

            current_time = nepi_utils.get_time()
            pub_latency = (current_time - timestamp)
            process_time = (current_time - start_time)
            self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(pub_latency), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Update Pub Stats
            if self.last_pub_time is None:
                pub_time_sec = 1.0
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time

            self.status_msg.frame_3d = navpose_dict['frame_3d']
            self.status_msg.publishing = True

            self.time_list.pop(0)
            self.time_list.append(pub_time_sec)

        return np_dict


    def unsubsribe(self):
        self.ready = False
        if self.node_if is not None:
            self.node_if.unregister_class()
        time.sleep(1)
        self.namespace = None
        self.status_msg = NavPoseStatus()


    def publish_status(self):
        if self.node_if is not None and self.status_msg is not None:
            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate
           
            self.node_if.publish_pub('status_pub', self.status_msg)



    def init(self, do_updates = False):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        if self.node_if is not None:
            pass
        self.init()

    def factory_reset(self):
        if self.node_if is not None:
            pass
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if has_subs == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        #self.msg_if.pub_warn("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)

    def _navposeFramesCheckCb(self,timer):
        self.navpose_frames_dict = nepi_system.get_navpose_frames(log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._navposeFramesCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        self.publish_status()

    def _provide_capabilities(self, _):
        return self.caps_report



class NavPoseTrackIF:

    NAVPOSE_NAV_FRAME_OPTIONS = ['ENU','NED','UKNOWN']
    NAVPOSE_ALT_FRAME_OPTIONS = ['WGS84','AMSL','UKNOWN'] # ['WGS84','AMSL','AGL','MSL','HAE','BAROMETER','UKNOWN']
    NAVPOSE_DEPTH_FRAME_OPTIONS = ['DEPTH','UKNOWN'] # ['MSL','TOC','DF','KB','DEPTH','UKNOWN']

    MIN_MAX_TRACK_LENGTH = [1,100]
    MIN_MAX_TRACK_SEC = [1,3600]
    ready = False
    namespace = '~'

    node_if = None

    status_msg = NavPoseStatus()

    last_pub_time = None

    has_subs = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    frame_3d = 'sensor_frame'
    frame_nav = 'ENU'
    frame_altitude = 'WGS84'
    frame_depth = 'MSL'

    caps_report = NavPoseCapabilitiesQueryResponse()

    track_length = 1
    track_sec = 60
    track_sec_last = 0.0
    track_msg_list = []

    data_product = 'navpose_track'

    navpose_frames_dict = nepi_nav.BLANK_NAVPOSE_FRAMES_DICT

    def __init__(self, namespace = None,
                data_source_description = 'navpose_track',
                data_ref_description = 'sensor_center',
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        # Initialize Class Variables
        if namespace is not None:
            self.namespace = namespace
        namespace = nepi_sdk.create_namespace(namespace,'navpose')
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        
        # Create Capabilities Report

        self.caps_report.has_location_pub = self.pub_location
        self.caps_report.has_heading_pub = self.pub_heading
        self.caps_report.has_position_pub = self.pub_position
        self.caps_report.has_orientation_pub = self.pub_orientation
        self.caps_report.has_depth_pub = self.pub_depth

        self.caps_report.min_max_track_length = self.MIN_MAX_TRACK_LENGTH
        self.caps_report.min_max_track_sec = self.MIN_MAX_TRACK_SEC



        # Initialize status message
        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description
        
        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description
        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Services Config Dict ####################     
        self.SRVS_DICT = {
            'navpose_caps_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': NavPoseCapabilitiesQuery,
                'req': NavPoseCapabilitiesQueryRequest(),
                'resp': NavPoseCapabilitiesQueryResponse(),
                'callback': self._provide_capabilities
            }
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'track_length': {
                'namespace': self.namespace,
                'factory_val': self.track_length
            },
            'track_sec': {
                'namespace': self.namespace,
                'factory_val': self.track_sec
            }
        }



        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'data_pub': {
                'msg': NavPoseTrack,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg': NavPoseStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'reset': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetCb, 
                'callback_args': ()
            },
            'set_track_length': {
                'msg': Int32,
                'namespace': self.namespace,
                'topic': 'set_track_length',
                'qsize': 1,
                'callback': self._setTrackLengthCb
            },
            'set_track_sec': {
                'msg': Int32,
                'namespace': self.namespace,
                'topic': 'set_track_sec',
                'qsize': 1,
                'callback': self._setTrackSecCb
            },          
            'clear_tracks': {
                'namespace': self.namespace,
                'topic': 'clear_tracks',
                'msg': Empty,
                'qsize': 1,
                'callback': self._clearTracksCb, 
                'callback_args': ()
            }
        }
        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                         msg_if = self.msg_if
                                            )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._navposeFramesCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  


    def get_frame_nav_options(self):
        return NAVPOSE_NAV_FRAME_OPTIONS

    def get_frame_altitude_options(self):
        return NAVPOSE_NAV_FRAME_OPTIONS

    def get_frame_depth_options(self):
        return NAVPOSE_DEPTH_FRAME_OPTIONS

    def get_data_products(self):
        return [self.data_product]

    def get_blank_navpose_dict(self):
        blank_navpose_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return blank_navpose_dict

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        has_subs = copy.deepcopy(self.has_subs)
        # self.msg_if.pub_debug("Returning: " + self.namespace + " " "has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        return has_subs


    # Update System Status
    def publish_navpose_track(self,navpose_dict, 
                        timestamp = None, 
                        frame_3d_transform = None,
                        device_mount_description = 'fixed'):      
        np_dict =  copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        if navpose_dict is None and self.status_msg is not None:
            return np_dict
        else:
            # Initialize np_dict here so it's available in both branches
            self.status_msg.source_frame_nav = navpose_dict['frame_nav']
            self.status_msg.source_frame_altitude = navpose_dict['frame_altitude']
            self.status_msg.source_frame_depth = navpose_dict['frame_depth']
            self.status_msg.device_mount_description = device_mount_description

            for key in np_dict.keys():
                if key in navpose_dict.keys():
                    np_dict[key] = navpose_dict[key]
            
            self.msg_if.pub_debug("Start Navpose data dict: " + str(np_dict), log_name_list = self.log_name_list, throttle_s = 5.0)

            if timestamp == None:
                timestamp = nepi_utils.get_time()
            else:
                timestamp = nepi_sdk.sec_from_timestamp(timestamp)

            current_time = nepi_utils.get_time()
            get_latency = (current_time - timestamp)
            self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(get_latency), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Start Img Pub Process
            start_time = nepi_utils.get_time()   


            # Transform navpose data frames to nepi standard frames
            if np_dict['frame_nav'] != 'ENU':
                if np_dict['frame_nav'] == 'NED':
                    nepi_nav.convert_navpose_ned2enu(np_dict)
            if np_dict['frame_altitude'] != 'WGS84':
                if np_dict['frame_altitude'] == 'AMSL':
                    nepi_nav.convert_navpose_amsl2wgs84(np_dict)
            if np_dict['frame_depth'] != 'MSL':
                if np_dict['frame_depth'] == 'DEPTH':
                    pass # need to add conversions                 

            self.status_msg.pub_frame_nav = np_dict['frame_nav']
            self.status_msg.pub_frame_altitude = np_dict['frame_altitude']
            self.status_msg.pub_frame_depth = np_dict['frame_depth']

            # Publish nav pose subs
            if self.pub_location == True:
                pub_name = 'location_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_location']
                msg.latitude = np_dict['latitude']
                msg.longitude = np_dict['longitude']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_heading == True:
                pub_name = 'heading_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_heading']
                msg.heading_deg = np_dict['heading_deg']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_orientation == True:
                pub_name = 'orientation_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_orientation']
                msg.roll_deg = np_dict['roll_deg']
                msg.pitch_deg = np_dict['pitch_deg']
                msg.yaw_deg = np_dict['yaw_deg']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_position == True:
                pub_name = 'position_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_position']
                msg.x_m = np_dict['x_m']
                msg.y_m = np_dict['y_m']
                msg.z_m = np_dict['z_m']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_altitude == True:
                pub_name = 'altitude_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_altitude']
                msg.altitude_m = np_dict['altitude_m']
                self.node_if.publish_pub(pub_name,msg)

            if self.pub_depth == True:
                pub_name = 'depth_pub'
                msg = self.PUBS_DICT[pub_name]['msg']()
                # gps_fix pub
                msg.timestamp = np_dict['time_depth']
                msg.depth_m = np_dict['depth_m']
                self.node_if.publish_pub(pub_name,msg)


            # Transform navpose in ENU and WSG84 frames
            if frame_3d_transform is not None:
                np_dict = nepi_nav.transform_navpose_dict(np_dict,frame_3d_transform)
                        
            # Transform navpose data frames to system set frames
            frame_nav = self.navpose_frames_dict['frame_nav']
            frame_alt = self.navpose_frames_dict['frame_alt']
            frame_depth = self.navpose_frames_dict['frame_depth']
            
            if np_dict['frame_nav'] != frame_nav:
                if np_dict['frame_nav'] == 'NED' and frame_nav == 'ENU':
                    nepi_nav.convert_navpose_ned2enu(np_dict)
                elif np_dict['frame_nav'] == 'ENU' and frame_nav == 'NED':
                    nepi_nav.convert_navpose_enu2ned(np_dict)
            if np_dict['frame_altitude'] != frame_alt:
                if np_dict['frame_altitude'] == 'AMSL' and frame_alt ==  'WGS84':
                    nepi_nav.convert_navpose_amsl2wgs84(np_dict)
                elif np_dict['frame_altitude'] == 'WGS84' and frame_alt ==  'AMSL':
                    nepi_nav.convert_navpose_wgs842amsl(np_dict)
            #if np_dict['frame_depth'] != 'MSL':
            #    if np_dict['frame_depth'] == 'DEPTH':
            #        pass # need to add conversions                 

            self.status_msg.pub_frame_nav = np_dict['frame_nav']
            self.status_msg.pub_frame_altitude = np_dict['frame_altitude']
            self.status_msg.pub_frame_depth = np_dict['frame_depth']



            try:
                data_msg = nepi_nav.convert_navpose_dict2msg(np_dict)
            except Exception as e:
                self.msg_if.pub_warn("Failed to convert navpose data to msg: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
                success = False
            if data_msg is not None:
                msg = NavPose()
                try:
                    self.node_if.publish_pub('data_pub', msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish navpose data msg: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
                    success = False

            current_time = nepi_utils.get_time()
            pub_latency = (current_time - timestamp)
            process_time = (current_time - start_time)
            self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(pub_latency), log_name_list = self.log_name_list, throttle_s = 5.0)

            # Update Pub Stats
            if self.last_pub_time is None:
                pub_time_sec = 1.0
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time

            self.status_msg.frame_3d = navpose_dict['frame_3d']
            self.status_msg.publishing = True

            self.time_list.pop(0)
            self.time_list.append(pub_time_sec)

            # Update tracks if needed 
            next_sec = self.get_next_track_sec()
            if next_sec <= 0:
                try:
                    track_msg = nepi_nav.get_navpose_track_msg_from_dict(np_dict)
                    if track_msg is not None:
                        if len(self.track_msg_list) >= self.track_length:
                            self.track_msg_list.pop[0]
                        self.track_msg_list.append(track_msg)
                        self.track_sec_last = nepi_utils.get_time()
                except Exception as e:
                    self.msg_if.pub_warn("Failed to convert navpose data to track msg: " + str(e), log_name_list = self.log_name_list, throttle_s = 5.0)
        return np_dict


    def unsubsribe(self):
        self.ready = False
        if self.node_if is not None:
            self.node_if.unregister_class()
        time.sleep(1)
        self.namespace = None
        self.status_msg = NavPoseStatus()



    def get_track_length(self):
        return self.track_length

    def set_track_length(self,length):
        if length < self.MIN_MAX_TRACK_LENGTH[0]:
            length = self.MIN_MAX_TRACK_LENGTH[0]
        if length > self.MIN_MAX_TRACK_LENGTH[1]:
            length = self.MIN_MAX_TRACK_LENGTH[1]
        self.track_length = length
        track_msg_list = copy.deepcopy(self.track_msg_list)
        while len(track_msg_list) > self.track_length:
            track_msg_list.pop[0]
        self.track_msg_list = track_msg_list
        self.node_if.set_param('track_length',length)
        self.publish_status()

    def get_track_sec(self):
        return self.track_sec

    def set_track_sec(self,sec):
        if sec < self.MIN_MAX_TRACK_SEC[0]:
            sec = self.MIN_MAX_TRACK_SEC[0]
        if sec > self.MIN_MAX_TRACK_SEC[1]:
            sec = self.MIN_MAX_TRACK_SEC[1]
        self.track_sec = sec
        self.node_if.set_param('track_sec',sec)
        self.publish_status()

    def get_next_track_sec(self):
        sec = nepi_utils.get_time() - self.track_sec_last
        next_sec = self.track_sec - sec
        if next_sec < 0:
            next_sec = 0
        return next_sec

    def get_tracks(self):
        track_list = []
        for track_msg in self.track_msg_list:
            track_list.append(nepi_sdk.convert_msg2dict(track_msg))
        return track_list
        
    def clear_tracks(self):
        self.track_msg_list = []
        self.track_sec_last = 0.0

    def publish_status(self):
        if self.node_if is not None and self.status_msg is not None:
            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate
           
            self.status_msg.track_length = self.track_length
            self.status_msg.track_sec = self.track_sec
            self.status_msg.track_next_sec = self.get_next_track_sec()
            self.status_msg.track_list = self.track_msg_list
            self.node_if.publish_pub('status_pub', self.status_msg)

        
    def init(self, do_updates = False):
        if self.node_if is not None:
            self.track_length = self.node_if.get_param('track_length')
            self.track_sec = self.node_if.get_param('track_sec')
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        if self.node_if is not None:
            pass
        self.init()

    def factory_reset(self):
        if self.node_if is not None:
            pass
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)


    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if has_subs == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        #self.msg_if.pub_warn("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)

    def _navposeFramesCheckCb(self,timer):
        self.navpose_frames_dict = nepi_system.get_navpose_frames(log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._navposeFramesCheckCb, oneshot = True)

        

    def _publishStatusCb(self,timer):
        self.publish_status()

    def _provide_capabilities(self, _):
        return self.caps_report


    def _setTrackLengthCb(self,msg):
        data = msg.data
        self.set_track_length(data)

    def _setTrackSecCb(self,msg):
        data = msg.data
        self.set_track_sec(data)
    
    def _clearTracksCb(self,msg):
        self.clear_tracks()



##################################################
# BaseImageIF


def get_image_data_product(data_product):
    if data_product is None:
        data_product = 'image'
    elif data_product.find('image') == -1:
        data_product = data_product + '_image'
    return data_product


SUPPORTED_DATA_PRODUCTS = ['image','color_image','bw_image',
                            'intensity_map','depth_map','pointcloud']
ENCODING_OPTIONS = ["mono8",'rgb8','bgr8','32FC1','passthrough']

PERSPECTIVE_OPTIONS = ['pov','top']

EXAMPLE_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = False,
        has_flip_horz = False,
        has_flip_vert = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate_3d = False,
        has_tilt_3d = False
    )

EXAMPLE_FILTERS_DICT = dict(
    # Low_Light = {
    #     'enabled': False,
    #     'function': nepi_img.low_light_filter,
    #     'ratio': 0.5
    # }
)

EXAMPLE_CONTROLS_DICT = dict( 
    resolution_ratio = 1.0,
    auto_adjust_enabled = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    window_ratios = [0,1,0,1],
    rotate_3d_ratio = 0.5,
    tilt_3d_ratio = 0.5       
    )



class BaseImageIF:

    DEFUALT_IMG_WIDTH_PX = 700
    DEFUALT_IMG_HEIGHT_PX = 400

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = False,
        has_flip_horz = False,
        has_flip_vert = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate_3d = False,
        has_tilt_3d = False
        )

    DEFAULT_FILTERS_DICT = dict()

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        rotate_2d_deg = 0,
        flip_horz = False,
        flip_vert = False,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5
        )


    DEFAULT_mouse_override_dict = dict(
        click_callback = None,
        drag_callback = None,
        window_callback = None
    )

    ready = False
    namespace = '~'

    node_if = None

    status_msg = ImageStatus()

    last_width = DEFUALT_IMG_WIDTH_PX
    last_height = DEFUALT_IMG_HEIGHT_PX

    min_range_m = 0
    max_range_m = 0

    perspective = 'pov'

    blank_img = nepi_img.create_cv2_blank_img(last_width, last_height, color = (0, 0, 0) )

    last_pub_time = None

    has_subs = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    dm_topic = ""
    pc_topic = ""

    caps_dict = DEFAULT_CAPS_DICT
    controls_dict = DEFAULT_CONTROLS_DICT
    init_controls_dict = controls_dict



    caps_report = ImageCapabilitiesQueryResponse()


    overlay_size_ratio = 0.5
    overlays_dict = dict(
            overlay_img_name = False,
            overlay_date_time = False,
            overlay_nav = False,
            overlay_pose = False, 
            init_overlay_list = [],
            add_overlay_list = []
    )


    auto_adjust_controls = []
    filter_dict = dict()
    has_filter = False
    filter_options = []
    sel_filters = []


    data_source_description = 'imaging_sensor'
    data_ref_description = 'sensor'

    data_product = 'image'

    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    get_navpose_function = None    
    has_navpose = False
    pub_navpose = False

    perspective = 'pov'

    add_pubs_dict = dict()

    pub_count = 0


    pixel = None
    window = None


    drag_pixel = None
    drag_window = None


    needs_update_callback = None


    mouse_override_dict = copy.deepcopy(DEFAULT_mouse_override_dict)


    x_offset = 0
    y_offset = 0
    x_scaler = 1
    y_scaler = 1

    raw_height = 0
    raw_width = 0
    proc_height = 0
    proc_width = 0

    zoom_ratio = 1
    x_ratio = 0.5
    y_ratio = 0.5
    window_ratios = [0,1,0,1]

    publishing = False

    def __init__(self, 
                namespace , 
                data_product_name,
                data_source_description,
                data_ref_description,
                perspective,
                caps_dict,
                controls_dict, 
                filter_dict,
                params_dict,
                services_dict,
                pubs_dict,
                subs_dict,
                pub_navpose,
                needs_update_callback,
                init_overlay_list,
                get_navpose_function,
                log_name,
                log_name_list,
                msg_if
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        # Initialize Class Variables

        

        if data_product_name is not None:
            self.data_product = data_product_name

        self.msg_if.pub_warn("Got namespace: " + str(namespace), log_name_list = self.log_name_list)
        if namespace is None:
            namespace = copy.deepcopy(self.namespace)
        namespace = nepi_sdk.get_full_namespace(namespace)
        self.namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        self.msg_if.pub_warn("Using data product namespace: " + str(self.namespace), log_name_list = self.log_name_list)
          
        if perspective is not None:
            self.perspective = perspective

        # Setup filter dict
        if filter_dict is not None:
            self.filter_dict = filter_dict
            self.filter_options = list(self.filter_dict.keys())
            if len(self.filter_options) > 0:
                self.has_filter = True

        self.pub_navpose = pub_navpose


        self.needs_update_callback = needs_update_callback

        # Create and update capabilities dictionary
        if caps_dict is not None:
            for cap in self.caps_dict.keys():
                if caps_dict.get(cap) != None:
                    self.caps_dict[cap] = caps_dict[cap]





        self.caps_report.has_resolution = self.caps_dict['has_resolution']
        self.caps_report.has_contrast = self.caps_dict['has_contrast']
        self.caps_report.has_brightness = self.caps_dict['has_brightness']
        self.caps_report.has_threshold = self.caps_dict['has_threshold']
        self.caps_report.has_rotate_2d = self.caps_dict['has_rotate_2d']
        self.caps_report.has_flip_horz = self.caps_dict['has_flip_horz']
        self.caps_report.has_flip_vert = self.caps_dict['has_flip_vert']
        self.caps_report.has_range = self.caps_dict['has_range']
        self.caps_report.has_auto_adjust = self.caps_dict['has_auto_adjust']
        self.caps_report.has_zoom = self.caps_dict['has_zoom']
        self.caps_report.has_pan = self.caps_dict['has_pan']
        self.caps_report.has_window = self.caps_dict['has_window']
        self.caps_report.has_rotate_3d = self.caps_dict['has_rotate_3d']
        self.caps_report.has_tilt_3d = self.caps_dict['has_tilt_3d']

        self.caps_report.has_filters = self.has_filter
        self.caps_report.filter_options = self.filter_options


        dm_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'depth_map')
        self.dm_topic = nepi_sdk.find_topic(dm_ns)
        has_dm = self.dm_topic != ""
        pc_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'pointcloud')
        self.pc_topic = nepi_sdk.find_topic(pc_ns)
        has_pc = self.pc_topic != ""

        self.caps_report.has_depth_map = has_dm
        self.caps_report.depth_map_topic = self.dm_topic
        self.caps_report.has_pointcloud = has_pc
        self.caps_report.pointcloud_topic = self.pc_topic



        # Create and update controls dictionary
        if controls_dict is not None:
            for control in self.controls_dict.keys():
                if controls_dict.get(control) != None:
                    self.controls_dict[control] = controls_dict[control]
        self.init_controls_dict = self.controls_dict

 
        self.overlays_dict['init_overlay_list'] = init_overlay_list

 
        # Initialize Status Msg.  Updated on each publish

        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description
        self.status_msg.publishing = False
        self.status_msg.encoding = 'bgr8'
        self.status_msg.width_px = 0
        self.status_msg.height_px = 0
        self.status_msg.width_deg = 0
        self.status_msg.height_deg = 0
        self.status_msg.perspective = self.perspective
        self.status_msg.auto_adjust_controls = self.auto_adjust_controls
        self.status_msg.get_latency_time = 0
        self.status_msg.pub_latency_time = 0
        self.status_msg.process_time = 0

        ####################
        # Configure get navpose if needed
        if get_navpose_function is not None:
            self.get_navpose_function = get_navpose_function
            self.has_navpose = True



        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {

            'resolution_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["resolution_ratio"]
            },
            'auto_adjust_enabled': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["auto_adjust_enabled"]
            },
            'auto_adjust_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["auto_adjust_ratio"]
            },
            'brightness_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["brightness_ratio"]
            },
            'contrast_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["contrast_ratio"]
            },
            'threshold_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["threshold_ratio"]
            },
            'rotate_2d_deg': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["rotate_2d_deg"]
            },
            'flip_horz': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["flip_horz"]
            },
            'flip_vert': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["flip_vert"]
            },
            'start_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["start_range_ratio"]
            },
            'stop_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["stop_range_ratio"]
            },
            'filter_dict': {
                'namespace': self.namespace,
                'factory_val': self.filter_dict
            },
            'overlay_size_ratio': {
                'namespace': self.namespace,
                'factory_val': self.overlay_size_ratio
            },
            'overlay_img_name': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'overlay_date_time': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'overlay_nav': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'overlay_pose': {
                'namespace': self.namespace,
                'factory_val': False
            },
            'add_overlay_list': {
                'namespace': self.namespace,
                'factory_val': []
            }
        }

        if params_dict is not None:
            self.PARAMS_DICT = params_dict | self.PARAMS_DICT
        else:
            self.PARAMS_DICT = self.PARAMS_DICT

        # Services Config Dict ####################     
        self.SRVS_DICT = {
            'image_caps_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': ImageCapabilitiesQuery,
                'req': ImageCapabilitiesQueryRequest(),
                'resp': ImageCapabilitiesQueryResponse(),
                'callback': self._provide_capabilities
            }
        }

        if services_dict is not None:
            self.SRVS_DICT = services_dict | self.SRVS_DICT
        else:
            self.SRVS_DICT = self.SRVS_DICT
        

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'data_pub': {
                'msg': Image,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg': ImageStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
            'navpose_pub': {
                'msg': NavPose,
                'namespace': self.namespace,
                'topic': 'navpose',
                'qsize': 1,
                'latch': False
            }

        }

        if pubs_dict is not None:
            self.PUBS_DICT = pubs_dict | self.PUBS_DICT
        else:
            self.PUBS_DICT = self.PUBS_DICT        

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'reset_all': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetControlsCb, 
                'callback_args': ()
            },
            'reset_filters': {
                'namespace': self.namespace,
                'topic': 'reset_filters',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetFiltersCb, 
                'callback_args': ()
            },
            'reset_overalys': {
                'namespace': self.namespace,
                'topic': 'reset_overalys',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetOverlaysCb, 
                'callback_args': ()
            },
            'reset_res_orients': {
                'namespace': self.namespace,
                'topic': 'reset_res_orients',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetResOrientsCb, 
                'callback_args': ()
            },
            'reset_renders': {
                'namespace': self.namespace,
                'topic': 'reset_renders',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetRendersCb, 
                'callback_args': ()
            },
            'set_click': {
                'namespace': self.namespace,
                'topic': 'set_click',
                'msg': ImagePixel,
                'qsize': 1,
                'callback': self._clickCb, 
                'callback_args': ()
            },
            'set_drag': {
                'namespace': self.namespace,
                'topic': 'set_drag',
                'msg': ImagePixel,
                'qsize': 1,
                'callback': self._dragCb, 
                'callback_args': ()
            },
            'set_window':  {
                'namespace': self.namespace,
                'topic': 'set_window',
                'msg': ImageWindow,
                'qsize': 1,
                'callback': self._windowCb, 
                'callback_args': ()
            },
            'overlay_size_ratio': {
                'msg': Float32,
                'namespace': self.namespace,
                'topic': 'set_overlay_size_ratio',
                'qsize': 1,
                'callback': self._setOverlaySizeCb
            },
            'overlay_img_name': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_source_name',
                'qsize': 1,
                'callback': self._setOverlayImgNameCb
            },
            'overlay_date_time': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_date_time',
                'qsize': 1,
                'callback': self._setOverlayDateTimeCb
            },
            'overlay_nav': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_nav',
                'qsize': 1,
                'callback': self._setOverlayNavCb
            },
            'overlay_pose': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_pose',
                'qsize': 1,
                'callback': self._setOverlayPoseCb
            },
            'overlay_text': {
                'msg': String,
                'namespace': self.namespace,
                'topic': 'add_overlay_text',
                'qsize': 1,
                'callback': self._setOverlayTextCb
            },
            'overlay_list': {
                'msg': StringArray,
                'namespace': self.namespace,
                'topic': 'set_overlay_list',
                'qsize': 1,
                'callback': self._setOverlayListCb
            },
            'overlay_clear': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'clear_overlay_list',
                'qsize': 1,
                'callback': self._clearOverlayListCb
            }
        }


        # Create subs if required
        if caps_dict['has_resolution'] == True:
            self.SUBS_DICT['set_resolution'] = {
                'namespace': self.namespace,
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setResolutionRatioCb, 
                'callback_args': ()
            }

        if caps_dict['has_auto_adjust'] == True:
            self.SUBS_DICT['set_auto_adjust'] = {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setAutoAdjustCb, 
                'callback_args': ()
            }
            self.SUBS_DICT['set_auto_adjust_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setAutoAdjustRatioCb, 
                'callback_args': ()
            }
        if caps_dict['has_brightness'] == True:
            self.SUBS_DICT['set_brightness'] = {
                'namespace': self.namespace,
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setBrightnessCb, 
                'callback_args': ()
            }
        if caps_dict['has_contrast'] == True:
            self.SUBS_DICT['set_contrast'] = {
                'namespace': self.namespace,
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setContrastCb, 
                'callback_args': ()
            }
        if caps_dict['has_threshold'] == True:
            self.SUBS_DICT['set_thresholding'] = {
                'namespace': self.namespace,
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setThresholdingCb, 
                'callback_args': ()
            }
        if caps_dict['has_rotate_2d'] == True:
            self.SUBS_DICT['rotate_2d'] = {
                'namespace': self.namespace,
                'topic': 'rotate_2d',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setRotate2dCb, 
                'callback_args': ()
            }
        if caps_dict['has_flip_horz'] == True:
            self.SUBS_DICT['set_flip_horz'] = {
                'namespace': self.namespace,
                'topic': 'set_flip_horz',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setFlipHorzCb, 
                'callback_args': ()
            }
        if caps_dict['has_flip_vert'] == True:
            self.SUBS_DICT['set_flip_vert'] = {
                'namespace': self.namespace,
                'topic': 'set_flip_vert',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setFlipVertCb, 
                'callback_args': ()
            }
        if caps_dict['has_range'] == True:
            self.SUBS_DICT['set_range'] = {
                'namespace': self.namespace,
                'topic': 'set_range_ratios',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self._setRangeCb, 
                'callback_args': ()
            }
        if caps_dict['has_zoom'] == True:
            self.SUBS_DICT['set_zoom'] = {
                'namespace': self.namespace,
                'topic': 'set_zoom_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setZoomCb, 
                'callback_args': ()
            }
        if caps_dict['has_pan'] == True:
            self.SUBS_DICT['set_pan_x_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_x_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setPanXCb, 
                'callback_args': ()
            }
            self.SUBS_DICT['set_pan_y_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_y_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setPanYCb, 
                'callback_args': ()
            }

        if caps_dict['has_rotate_3d'] == True:
            self.SUBS_DICT['set_rotate'] = {
                'namespace': self.namespace,
                'topic': 'set_rotate_3d_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setRotateCb, 
                'callback_args': ()
            }

        if caps_dict['has_tilt_3d'] == True:
            self.SUBS_DICT['set_tilt'] = {
                'namespace': self.namespace,
                'topic': 'set_tilt_3d_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setTiltCb, 
                'callback_args': ()
            }
        if self.has_filter == True:
            self.SUBS_DICT['set_filter_enable'] = {
                'namespace': self.namespace,
                'topic': 'set_filter_enable',
                'msg': UpdateState,
                'qsize': 1,
                'callback': self._setFilterEnableCb, 
                'callback_args': ()
            }
            self.SUBS_DICT['set_filter_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_filter_ratio',
                'msg': UpdateRatio,
                'qsize': 1,
                'callback': self._setFilterRatioCb, 
                'callback_args': ()
            }

        if self.has_navpose == False:
            ##############################
            ## Connect NEPI System NavPose
            self.SUBS_DICT['navpose_sub'] = {
                'namespace': self.base_namespace,
                'topic': 'navpose',
                'msg': NavPose,
                'qsize': 1,
                'callback': self._navposeCb, 
                'callback_args': ()
            }


        if subs_dict is not None:
            self.SUBS_DICT = subs_dict | self.SUBS_DICT
        else:
            self.SUBS_DICT = self.SUBS_DICT     


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

        success = nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init(do_updates = True)
        self.publish_status()

        ##############################
        # Start Node Processes
        self.msg_if.pub_warn("Staring subscribers check process", log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        self.msg_if.pub_warn("Starting status publisher process", log_name_list = self.log_name_list)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def add_pub_namespace(self,namespace):
        success = False
        if namespace is None:
            return success
        namespace = nepi_sdk.get_full_namespace(namespace)
        img_ns = nepi_sdk.create_namespace(namespace,self.data_product)
        if img_ns == self.namespace or namespace in self.add_pubs_dict.keys():
            self.msg_if.pub_warn("Image pub namespace allready registered: " + str(namespace), log_name_list = self.log_name_list)
            return success
        else:
            self.msg_if.pub_warn("Adding image pub namespace: " + str(self.namespace), log_name_list = self.log_name_list)
            img_pub_dict ={
                    'msg': Image,
                    'namespace': img_ns,
                    'topic': '',
                    'qsize': 1,
                    'latch': False
                }
            self.node_if.register_pub(img_ns,img_pub_dict)

            status_ns = nepi_sdk.create_namespace(img_ns,'status')
            status_pub_dict ={
                    'msg': ImageStatus,
                    'namespace': status_ns,
                    'topic': '',
                    'qsize': 1,
                    'latch': False
                }
            self.node_if.register_pub(status_ns,status_pub_dict)

            nav_ns = nepi_sdk.create_namespace(img_ns,'navpose')
            nav_pub_dict ={
                    'msg': NavPose,
                    'namespace': nav_ns,
                    'topic': '',
                    'qsize': 1,
                    'latch': False
                }
            self.node_if.register_pub(nav_ns,nav_pub_dict)

            nepi_sdk.sleep(1)
            self.add_pubs_dict[namespace] = [img_ns,status_ns,nav_ns]
            success = True
        return success

    def remove_pub_namespace(self,namespace):
        success = False
        if namespace is None:
            return success
        namespace = nepi_sdk.get_full_namespace(namespace)
        img_ns = nepi_sdk.create_namespace(namespace,self.data_product)
        if img_ns == self.namespace:
            self.msg_if.pub_warn("Can't remove base namespace: " + str(namespace), log_name_list = self.log_name_list)
            return success
        elif namespace in self.add_pubs_dict.keys():
            self.msg_if.pub_warn("Removing image pub namespace: " + str(self.namespace), log_name_list = self.log_name_list)
            [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
            del self.add_pubs_dict[namespace]
            nepi_sdk.sleep(1)
            self.node_if.unregister_pub(img_ns)
            self.node_if.unregister_pub(status_ns)
            self.node_if.unregister_pub(nav_ns)
            success = True
        return success

    def get_data_source_description(self):
        return self.data_source_description

    def get_data_products(self):
        return [self.data_product]

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        return self.has_subs

    def get_navpose_dict(self):
        navpose_dict = None
        if self.get_navpose_function is not None:
            navpose_dict = self.get_navpose_function()
        else:
            navpose_dict = self.navpose_dict
        return navpose_dict
    
    def get_mouse_callback_options(self):
        return list(self.mouse_override_dict.keys())
    
    def set_mouse_callback(self,name,function):
        self.msg_if.pub_warn("Got set mouse callback for: " + str(name), log_name_list = self.log_name_list)
        if name in self.mouse_override_dict.keys():
            self.msg_if.pub_warn("Mouse callback set for: " + str(name), log_name_list = self.log_name_list)
            self.mouse_override_dict[name] = function
        #self.msg_if.pub_info("Updated mouse overide dict: " + str(self.mouse_override_dict), log_name_list = self.log_name_list)

    def clear_mouse_callback(self,name):
        self.msg_if.pub_warn("Got clear mouse callback for: " + str(name), log_name_list = self.log_name_list)
        if name in self.mouse_override_dict.keys():
            self.mouse_override_dict[name] = None   

    def process_cv2_img(self, cv2_img):
        return cv2_img

    def publish_cv2_img(self, cv2_img, 
                        encoding = "bgr8", 
                        timestamp = None, 
                        frame_3d = 'sensor_frame', 
                        width_deg = 100,
                        height_deg = 70,
                        min_range_m = None,
                        max_range_m = None,
                        add_overlay_list = [],
                        device_mount_description = 'fixed',
                        navpose_dict = None,
                        process_data = True,
                        pub_twice = False,
                        add_pubs = []
                        ):
        if self.publishing == False:
            self.publishing = True
            #self.msg_if.pub_debug("Got Image to Publish", log_name_list = self.log_name_list, throttle_s = 5.0)
            success = False
            if cv2_img is None and self.status_msg is not None:
                self.msg_if.pub_warn("Can't publish None image", log_name_list = self.log_name_list)
                return cv2_img

            # Process 
            try: # Catch for lost camera in middle of send
                self.device_mount_description = device_mount_description
                self.status_msg.device_mount_description = self.device_mount_description
                
                self.status_msg.frame_3d = frame_3d

                self.status_msg.encoding = encoding
                #self.msg_if.pub_warn("Got timestamp: " + str(timestamp), log_name_list = self.log_name_list)
                if timestamp == None:
                    timestamp = nepi_utils.get_time()
                else:
                    timestamp = nepi_sdk.sec_from_timestamp(timestamp)
                #self.msg_if.pub_warn("Using timestamp: " + str(timestamp), log_name_list = self.log_name_list)


                current_time = nepi_utils.get_time()
                latency = (current_time - timestamp)
                self.status_msg.get_latency_time = latency
                #self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency), log_name_list = self.log_name_list, throttle_s = 5.0)

                # Start Img Pub Process
                start_time = nepi_utils.get_time()   



                self.status_msg.width_deg = width_deg
                self.status_msg.height_deg = height_deg

                if (min_range_m is not None and max_range_m is not None):
                    self._updateRangesM(min_range_m,max_range_m)
                    self.status_msg.min_range_m = self.min_range_m
                    self.status_msg.max_range_m = self.max_range_m
                else:
                    self.status_msg.min_range_m = 0
                    self.status_msg.max_range_m = 1

                [height,width] = cv2_img.shape[0:2]
                [self.raw_height,self.raw_width] = [height,width]
                #self.msg_if.pub_warn("Got Image size: " + str([height,width]), log_name_list = self.log_name_list)


                if process_data == True:
                    cv2_img = self.process_cv2_img(cv2_img)
                
                if cv2_img is not None:
                    
                    

                    [height,width] = cv2_img.shape[0:2]
                    [self.proc_height,self.proc_width] = [height,width]

                    if height > 5 and width > 5:
                        #self.msg_if.pub_debug("Got Processed size: " + str([height,width]), log_name_list = self.log_name_list)


                        last_width = self.status_msg.width_px
                        last_height = self.status_msg.height_px
                        self.status_msg.width_px = width
                        self.status_msg.height_px = height
                        res_str = str(width) + ":" + str(height)
                        self.status_msg.resolution_current = res_str

                        if navpose_dict is None:
                            navpose_dict = self.get_navpose_dict()

                        
                        # Apply Overlays
                        overlay_list = []
                        if self.status_msg.overlay_img_name == True:
                            overlay = nepi_img.getImgShortName(self.namespace)
                            overlay_list.append(overlay)
                        
                        if self.status_msg.overlay_date_time == True:
                            overlay = nepi_utils.get_datetime_str_from_timestamp(timestamp)
                            overlay = overlay.replace('D','')
                            overlay = overlay.replace('T',' T: ')
                            overlay_list.append(overlay)

                        
                        if self.status_msg.overlay_nav == True or self.status_msg.overlay_pose == True:
                            if navpose_dict is not None:
                                if self.status_msg.overlay_nav == True and navpose_dict is not None:
                                    overlay = 'Lat: ' +  str(round(navpose_dict['latitude'],6)) + ' Long: ' +  str(round(navpose_dict['longitude'],6)) + ' Head: ' +  str(round(navpose_dict['heading_deg'],2))
                                    overlay_list.append(overlay)

                                if self.status_msg.overlay_pose == True and navpose_dict is not None:
                                    overlay = 'Roll: ' +  str(round(navpose_dict['roll_deg'],2)) + ' Pitch: ' +  str(round(navpose_dict['pitch_deg'],2)) + ' Yaw: ' +  str(round(navpose_dict['yaw_deg'],2))
                                    overlay_list.append(overlay)

                        overlay_list = overlay_list + self.overlays_dict['init_overlay_list'] + self.overlays_dict['add_overlay_list'] + add_overlay_list

                        if len(overlay_list) > 0:
                            start_y = (height * 0.01)
                            cv2_img = nepi_img.overlay_text_list(cv2_img, 
                                                    text_list = overlay_list, 
                                                    x_px = 10 , y_px = start_y, 
                                                    color_rgb = (0, 255, 0), 
                                                    apply_shadow = True, 
                                                    size_ratio = self.overlay_size_ratio )

                        
                        if self.node_if is not None and self.has_subs == True:
                            #self.msg_if.pub_warn("Publishing once")
                            #Convert to ros Image message
                            ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
                            sec = nepi_sdk.sec_from_timestamp(timestamp)
                            header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = frame_3d)
                            #self.msg_if.pub_warn("Publishing image with header: " + str(header))
                            ros_img.header = header
                            self.node_if.publish_pub('data_pub', ros_img)

                            for namespace in add_pubs:
                                if namespace in self.add_pubs_dict.keys():
                                    [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
                                    #self.msg_if.pub_warn("Publishing Add Image on namespace: " + str(img_ns), log_name_list = self.log_name_list, throttle_s = 5.0)
                                    self.node_if.publish_pub(img_ns, ros_img)
                            if pub_twice == True:
                                #self.msg_if.pub_warn("Publishing twice: " + str(pub_twice))
                                nepi_sdk.sleep(0.01)
                                self.node_if.publish_pub('data_pub', ros_img)
                                for namespace in add_pubs:
                                    if namespace in self.add_pubs_dict.keys():
                                        [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
                                        self.node_if.publish_pub(img_ns, ros_img)




                        
                        navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
                        if navpose_msg is not None:
                            self.node_if.publish_pub('navpose_pub', navpose_msg)
                            for namespace in add_pubs:
                                if namespace in self.add_pubs_dict.keys():
                                    [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
                                    self.node_if.publish_pub(nav_ns, navpose_msg)
                        
                        # Update stats
                        process_time = round( (nepi_utils.get_time() - start_time) , 3)
                        self.status_msg.process_time = process_time
                        latency = (current_time - timestamp)
                        self.status_msg.pub_latency_time = latency
                        

                        if self.last_pub_time is None:
                            self.last_pub_time = nepi_utils.get_time()
                        else:
                            cur_time = nepi_utils.get_time()
                            pub_time_sec = cur_time - self.last_pub_time
                            self.last_pub_time = cur_time
                            self.status_msg.last_pub_sec = pub_time_sec

                            self.time_list.pop(0)
                            self.time_list.append(pub_time_sec)

                        # Update blank image if needed
                        if last_width != self.status_msg.width_px or last_height != self.status_msg.height_px:
                            self.blank_img = nepi_img.create_cv2_blank_img(width, height, color = (0, 0, 0) )
                        for namespace in add_pubs:
                            if namespace in self.add_pubs_dict.keys():
                                [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace]
                                self.node_if.publish_pub(status_ns, self.status_msg)
            except Exception as e:
                self.msg_if.pub_warn("Failed to publish: " + str(e), log_name_list = self.log_name_list)
        self.publishing = False
        return cv2_img
        

    def publish_msg_img(self, msg_text, timestamp = None, frame_3d = 'nepi_base'):
        cv2_img = nepi_img.overlay_text_autoscale(self.blank_img, text)

        if timestamp == None:
            timestamp = nepi_utils.get_time()

        #Convert to ros Image message
        ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
        sec = nepi_sdk.sec_from_timestamp(timestamp)
        ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = frame_3d)
        if self.node_if is not None:
            self.node_if.publish_pub('data_pub', ros_img)

    

    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_sdk.wait()
        self.namespace = '~'
        self.status_msg = None

    ########################
    # Filter Functions

    def set_auto_adjust_enable(self, enabled):
        if enabled:
            self.msg_if.pub_info("Enabling Auto Adjust", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Disabling Auto Adjust", log_name_list = self.log_name_list)
        self.controls_dict['auto_adjust_enabled'] = enabled
        self.publish_status()  
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('auto_adjust_enabled', enabled)


    def set_auto_adjust_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['auto_adjust_ratio'] = ratio
        self.publish_status()  
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('auto_adjust_ratio', ratio)

    def set_brightness_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['brightness_ratio'] = ratio
        self.publish_status()  
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('brightness_ratio', ratio)


    def set_contrast_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['contrast_ratio'] = ratio
        self.publish_status()  
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('contrast_ratio', ratio)
        


    def set_threshold_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['threshold_ratio'] = ratio
        self.publish_status()  
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('threshold_ratio', ratio)



    def set_filter_enable(self,name, enabled):
        if name in self.filter_dict.keys():
            was_enabled = self.filter_dict[name]['enabled']
            if was_enabled != enabled:
                if enabled == True:
                    self.msg_if.pub_info("Enabling Filter: " + name, log_name_list = self.log_name_list)
                else:
                    self.msg_if.pub_info("Disabling Filter: " + name, log_name_list = self.log_name_list)
                self.filter_dict[name]['enabled'] = enabled
                self.publish_status()  
                self.needs_update()
                if self.node_if is not None:
                    self.node_if.set_param('filter_dict', self.filter_dict)


    def set_filter_ratio(self,name, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        if name in self.filter_dict.keys():
            self.msg_if.pub_info("Setting Filter Ratio: " + name + " : " + str(ratio), log_name_list = self.log_name_list)
            self.filter_dict[name]['ratio'] = ratio
            self.publish_status() 
            self.needs_update()
            self.node_if.set_param('filter_dict', self.filter_dict)

    ########################
    # Res and Orientation Functions

    def set_resolution_ratio(self, ratio):
        if (ratio < 0.2):
            ratio = 0.2
        if (ratio > 1.0):
            ratio = 1.0
        self.controls_dict['resolution_ratio'] = ratio
        self.publish_status() 
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('resolution_ratio', ratio)

    def set_rotate_2d_deg(self, deg):
        deg_int = int(round(deg,0))
        self.controls_dict['rotate_2d_deg'] = deg_int
        self.publish_status() 
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('rotate_2d_deg', deg_int)

    def set_flip_horz(self, enabled):
        self.publish_status() 
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('flip_horz', enabled)

    def set_flip_vert(self, enabled):
        self.publish_status() 
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('flip_vert', enabled)

    ########################
    # Render Functions

    def set_range_ratios(self, start_ratio, stop_ratio):
        if (start_ratio < 0 or stop_ratio > 1 or stop_ratio < start_ratio):
            self.msg_if.pub_error("Range values out of bounds", log_name_list = self.log_name_list)
            self.publishStatus(do_updates=False) # No change
            return

        self.controls_dict['start_range_ratio'] = start_ratio
        self.controls_dict['stop_range_ratio'] = stop_ratio

        self.publish_status() 
        self.needs_update()
        if self.node_if is not None:
            self.node_if.set_param('start_range_ratio', start_ratio)
            self.node_if.set_param('stop_range_ratio', stop_ratio)
      


    def set_zoom_ratio(self, ratio):
        self.drag_pixel = None
        self.drag_window = None

        # Update Ratios

        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_ratio = wrs[0] + (wrs[1] - wrs[0]) / 2
        yr_ratio = wrs[2] + (wrs[3] - wrs[2]) / 2

        xlen_max_r = (1 - abs(xr_ratio - 0.)) * 2
        ylen_max_r = (1 - abs(yr_ratio - 0.5)) * 2
        len_min_r = 0.05
        len_max_r = min(xlen_max_r, ylen_max_r)

        len_zoom = 1 - ratio * len_max_r
        if len_zoom < len_min_r:
            len_zoom = len_min_r

        xr_min = xr_ratio - len_zoom / 2
        xr_max = xr_ratio + len_zoom / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = len_zoom
        if xr_max > 1:
            xr_min = 1 - len_zoom
            xr_max = 1
    
        yr_min = yr_ratio - len_zoom / 2
        yr_max = yr_ratio + len_zoom / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = len_zoom
        if yr_max > 1:
            yr_min = 1 - len_zoom
            yr_max = 1

        self.msg_if.pub_info("Zoom Image Window: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = ratio

        self.publish_status() 
        self.needs_update()


    def set_pixel(self, pixel, color_bgr = [0,0,0,0]):

        self.drag_pixel = None
        self.drag_window = None

    
        # Update Ratios
        xr_ratio = pixel[0] / self.raw_width
        yr_ratio = pixel[1] / self.raw_height
        wrs = copy.deepcopy(self.controls_dict['window_ratios'])

        xr_len = wrs[1] - wrs[0]
        xr_min = xr_ratio - xr_len / 2
        xr_max = xr_ratio + xr_len / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = xr_len
        if xr_max > 1:
            xr_min = 1 - xr_len
            xr_max = 1
    
        yr_len = wrs[3] - wrs[2]
        yr_min = yr_ratio - yr_len / 2
        yr_max = yr_ratio + yr_len / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = yr_len
        if yr_max > 1:
            yr_min = 1 - yr_len
            yr_max = 1

        self.msg_if.pub_info("Pixel Image Window: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = 1 - max(xr_len, yr_len)

        self.publish_status()  
        self.needs_update()    



    def set_x_ratio(self, ratio):

        self.drag_pixel = None
        self.drag_window = None


        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_len = wrs[1] - wrs[0]
        yr_len = wrs[3] - wrs[2]

        r_min = xr_len / 2
        r_max = 1 - xr_len / 2
        xr_r = r_min + (ratio * (r_max - r_min))

        xr_min = xr_r - xr_len / 2
        xr_max = xr_r + xr_len / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = xr_len
        if xr_max > 1:
            xr_min = 1 - xr_len
            xr_max = 1
    
        yr_r = wrs[2] + yr_len / 2
        yr_min = yr_r - xr_len / 2
        yr_max = yr_r + xr_len / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = xr_len
        if yr_max > 1:
            yr_min = 1 - xr_len
            yr_max = 1

        self.msg_if.pub_info("X Ratio Image Window: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = 1 - xr_len

        self.publish_status() 
        self.needs_update()

    def set_y_ratio(self, ratio): 

        self.drag_pixel = None
        self.drag_window = None


        wrs = copy.deepcopy(self.controls_dict['window_ratios'])
        xr_len = wrs[1] - wrs[0]
        yr_len = wrs[3] - wrs[2]

        r_min = yr_len / 2
        r_max = 1 - yr_len / 2
        yr_r = r_min + (ratio * (r_max - r_min))

        yr_min = yr_r - yr_len / 2
        yr_max = yr_r + yr_len / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = yr_len
        if yr_max > 1:
            yr_min = 1 - yr_len
            yr_max = 1
    
        xr_r = wrs[0] + xr_len / 2
        xr_min = xr_r - yr_len / 2
        xr_max = xr_r + yr_len / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = yr_len
        if xr_max > 1:
            xr_min = 1 - yr_len
            xr_max = 1

        self.msg_if.pub_info("Y Ratio Image Window: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = 1 - yr_len

        self.publish_status() 
        self.needs_update()


    def set_window(self, window):

        self.drag_pixel = None
        self.drag_window = None
 
        # Update Ratios
        xr_len = (window[1] - window[0]) / self.raw_width
        yr_len = (window[3] - window[2]) / self.raw_height
        xr_ratio = window[0] / self.raw_width + (xr_len / 2) 
        yr_ratio = window[2] / self.raw_height + (yr_len / 2)

        r_len_max = max(xr_len, yr_len)

        xr_min = xr_ratio - r_len_max / 2
        xr_max = xr_ratio + r_len_max / 2
        if xr_min < 0:
            xr_min = 0
            xr_max = r_len_max
        if xr_max > 1:
            xr_min = 1 - r_len_max
            xr_max = 1
    

        yr_min = yr_ratio - r_len_max / 2
        yr_max = yr_ratio + r_len_max / 2
        if yr_min < 0:
            yr_min = 0
            yr_max = r_len_max
        if yr_max > 1:
            yr_min = 1 - r_len_max
            yr_max = 1

        self.msg_if.pub_warn("Window Image Window set to: " + str([xr_min, xr_max, yr_min, yr_max]), log_name_list = self.log_name_list)
        self.controls_dict['window_ratios'] = [xr_min, xr_max, yr_min, yr_max]
        self.x_ratio = xr_min + (xr_max - xr_min) / 2
        self.y_ratio = yr_min + (yr_max - yr_min) / 2
        self.zoom_ratio = 1 - r_len_max 

        self.publish_status()  
        self.needs_update()    
        self.publish_status()  
        self.needs_update()







    def update_window_ratios(self): 
        self.y_ratio = nepi_utils.check_ratio(ratio)
        self.publish_status() 
        self.needs_update()


    def set_rotate_3d_ratio(self, ratio):
        self.controls_dict['rotate_3d_ratio'] = nepi_utils.check_ratio(ratio)
        self.publish_status()  
        self.needs_update()

    def set_tilt_3d_ratio(self, ratio):
        self.controls_dict['tilt_3d_ratio'] = nepi_utils.check_ratio(ratio)
        self.publish_status()  
        self.needs_update()



    ########################
    # Overlay Functions

    def set_overlay_size_ratio(self, ratio):
        self.overlay_size_ratio = nepi_utils.check_ratio(ratio)
        self.publish_status() 
        self.needs_update()



    def set_overlay_image_name(self,enabled):
        self.overlays_dict['overlay_img_name'] = enabled
        self.publish_status()
        self.needs_update()
        self.node_if.set_param('overlay_img_name', enabled)

    def set_overlay_date_time(self,enabled):
        self.overlays_dict['overlay_date_time'] = enabled
        self.publish_status()  
        self.needs_update()
        self.node_if.set_param('overlay_date_time', enabled)

    def set_overlay_nav(self,enabled):
        self.overlays_dict['overlay_nav'] = enabled
        self.publish_status()  
        self.needs_update()
        self.node_if.set_param('overlay_nav', enabled)

    def set_overlay_pose(self,enabled):
        self.overlays_dict['overlay_pose'] = enabled
        self.publish_status()  
        self.needs_update()
        self.node_if.set_param('overlay_pose', enabled)


    def set_overlay_list(self,overlay_list):
        self.overlays_dict['add_overlay_list'] = overlay_list
        self.publish_status()  
        self.needs_update()
        self.node_if.set_param('add_overlay_list', overlay_list)


    def set_overlay_text(self,overlay_text):
        overlay_list = self.overlays_dict['add_overlay_list']
        overlay_list.append(text)
        self.overlays_dict['add_overlay_list'] = overlay_list
        self.publish_status()  
        self.needs_update()
        self.node_if.set_param('add_overlay_list', overlay_list)


    def clear_overlay_list(self):
        self.overlays_dict['add_overlay_list'] = []
        self.publish_status()  
        self.needs_update()
        self.node_if.set_param('add_overlay_list', [])


    def reset_filters(self):
        
        # First reset controls to init dict to capture non param managed settings
        self.controls_dict = self.init_controls_dict

        self.node_if.factory_reset_param('auto_adjust_enabled')
        self.node_if.factory_reset_param('auto_adjust_ratio')
        self.node_if.factory_reset_param('brightness_ratio')
        self.node_if.factory_reset_param('contrast_ratio')
        self.node_if.factory_reset_param('threshold_ratio')
        self.node_if.factory_reset_param('filter_dict')

        self.controls_dict['auto_adjust_enabled'] = self.node_if.get_param('auto_adjust_enabled')
        self.controls_dict['auto_adjust_ratio'] = self.node_if.get_param('auto_adjust_ratio')
        self.controls_dict['brightness_ratio'] = self.node_if.get_param('brightness_ratio')
        self.controls_dict['contrast_ratio'] = self.node_if.get_param('contrast_ratio')
        self.controls_dict['threshold_ratio'] = self.node_if.get_param('threshold_ratio')
        self.filter_dict = self.node_if.get_param('filter_dict')

        self.publish_status()  
        self.needs_update()


    def reset_overlays(self):
        self.node_if.factory_reset_param('overlay_size_ratio')
        self.node_if.factory_reset_param('overlay_img_name')
        self.node_if.factory_reset_param('overlay_date_time')
        self.node_if.factory_reset_param('overlay_nav')
        self.node_if.factory_reset_param('overlay_pose')
        self.node_if.factory_reset_param('add_overlay_list')

        self.overlays_dict['overlay_img_name'] = self.node_if.get_param('overlay_img_name')
        self.overlays_dict['overlay_date_time'] = self.node_if.get_param('overlay_date_time')
        self.overlays_dict['overlay_nav'] = self.node_if.get_param('overlay_nav')
        self.overlays_dict['overlay_pose'] = self.node_if.get_param('overlay_pose')
        self.overlays_dict['add_overlay_list'] = self.node_if.get_param('add_overlay_list')
        
        self.publish_status()  
        self.needs_update()


    def reset_res_orients(self):
        
        # First reset controls to init dict to capture non param managed settings
        self.controls_dict = self.init_controls_dict

        self.node_if.factory_reset_param('resolution_ratio')
        self.node_if.factory_reset_param('rotate_2d_deg')
        self.node_if.factory_reset_param('flip_horz')
        self.node_if.factory_reset_param('flip_vert')


        self.controls_dict['resolution_ratio'] = self.node_if.get_param('resolution_ratio')
        self.controls_dict['rotate_2d_deg'] = self.node_if.get_param('rotate_2d_deg')
        self.controls_dict['flip_horz'] = self.node_if.get_param('flip_horz')
        self.controls_dict['flip_vert'] = self.node_if.get_param('flip_vert')

        self.filter_dict = self.node_if.get_param('filter_dict')

        self.publish_status()  
        self.needs_update()

    def reset_renders(self):
        self.msg_if.pub_warn("Reseting render values", log_name_list = self.log_name_list)
        self.drag_pixel = None
        self.drag_window = None
        self.zoom_ratio = 0
        self.x_ratio = 0.5
        self.y_ratio = 0.5
        self.x_offset = 0
        self.y_offset = 0
        self.x_scaler = 1
        self.y_scaler = 1
        self.controls_dict = self.init_controls_dict
        self.controls_dict['start_range_ratio'] = 0
        self.controls_dict['stop_range_ratio'] = 1
        self.controls_dict['window_ratios'] = [0,1,0,1]

        self.window_ratios = [0,1,0,1]

        self.controls_dict['rotate_3d_ratio'] = 0.5
        self.controls_dict['tilt_3d_ratio'] = 0.5

        self.node_if.factory_reset_param('start_range_ratio')
        self.node_if.factory_reset_param('stop_range_ratio')

        self.controls_dict['start_range_ratio'] = self.node_if.get_param('start_range_ratio')
        self.controls_dict['stop_range_ratio'] = self.node_if.get_param('stop_range_ratio')



        self.publish_status()  
        self.msg_if.pub_warn("Calling needs update callback", log_name_list = self.log_name_list)
        self.needs_update()


    def publish_status(self):
        if self.node_if is not None and self.status_msg is not None:

            self.status_msg.auto_adjust_enabled = self.controls_dict['auto_adjust_enabled']
            self.status_msg.auto_adjust_ratio = self.controls_dict['auto_adjust_ratio']
            self.status_msg.contrast_ratio = self.controls_dict['contrast_ratio']
            self.status_msg.brightness_ratio = self.controls_dict['brightness_ratio']
            self.status_msg.threshold_ratio = self.controls_dict['threshold_ratio']
            filter_options = []
            filter_states = []
            filter_ratios = []
            for name in self.filter_dict.keys():
                filter_dict = self.filter_dict[name]
                filter_options.append(name)
                filter_states.append(filter_dict['enabled'])
                filter_ratios.append(filter_dict['ratio'])
            self.status_msg.filter_options = filter_options
            self.status_msg.filter_states = filter_states
            self.status_msg.filter_ratios = filter_ratios

            self.status_msg.resolution_ratio = self.controls_dict['resolution_ratio']
            self.status_msg.rotate_2d_deg = self.controls_dict['rotate_2d_deg']
            self.status_msg.flip_horz = self.controls_dict['flip_horz']
            self.status_msg.flip_vert = self.controls_dict['flip_vert']

            self.status_msg.range_ratios.start_range = self.controls_dict['start_range_ratio']
            self.status_msg.range_ratios.stop_range = self.controls_dict['stop_range_ratio']


            self.status_msg.zoom_ratio = self.zoom_ratio
            self.status_msg.pan_x_ratio = self.x_ratio
            self.status_msg.pan_y_ratio = self.y_ratio
            self.status_msg.window_x_ratios.start_range = self.controls_dict['window_ratios'][0]
            self.status_msg.window_x_ratios.stop_range = self.controls_dict['window_ratios'][1]
            self.status_msg.window_y_ratios.start_range = self.controls_dict['window_ratios'][2]
            self.status_msg.window_y_ratios.stop_range = self.controls_dict['window_ratios'][3]
            self.status_msg.rotate_3d_ratio = self.controls_dict['rotate_3d_ratio']
            self.status_msg.tilt_3d_ratio = self.controls_dict['tilt_3d_ratio']


            self.status_msg.depth_map_topic = self.dm_topic
            self.status_msg.pointcloud_topic = self.pc_topic


            self.status_msg.overlay_size_ratio = self.overlay_size_ratio
            self.status_msg.overlay_img_name = self.overlays_dict['overlay_img_name']
            self.status_msg.overlay_date_time =  self.overlays_dict['overlay_date_time']
            self.status_msg.overlay_nav = self.overlays_dict['overlay_nav']
            self.status_msg.overlay_pose = self.overlays_dict['overlay_pose']  
            self.status_msg.base_overlay_list = self.overlays_dict['init_overlay_list']
            self.status_msg.add_overlay_list = self.overlays_dict['add_overlay_list']

            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate
            #self.msg_if.pub_info("Publishing Status Msg: " + str(self.status_msg), log_name_list = self.log_name_list)
            self.node_if.publish_pub('status_pub',self.status_msg)




    def init(self, do_updates = False):
        if self.node_if is not None:
            self.controls_dict['resolution_ratio'] = self.node_if.get_param('resolution_ratio')
            self.controls_dict['auto_adjust_enabled'] = self.node_if.get_param('auto_adjust_enabled')
            self.controls_dict['auto_adjust_ratio'] = self.node_if.get_param('auto_adjust_ratio')
            self.controls_dict['brightness_ratio'] = self.node_if.get_param('brightness_ratio')
            self.controls_dict['contrast_ratio'] = self.node_if.get_param('contrast_ratio')
            self.controls_dict['threshold_ratio'] = self.node_if.get_param('threshold_ratio')

            self.controls_dict['rotate_2d_deg'] = self.node_if.get_param('rotate_2d_deg')
            self.controls_dict['flip_horz'] = self.node_if.get_param('flip_horz')
            self.controls_dict['flip_vert'] = self.node_if.get_param('flip_vert')


            self.controls_dict['window_ratios'] = [0,1,0,1]
            self.controls_dict['start_range_ratio'] = 0
            self.controls_dict['stop_range_ratio'] = 1


            self.filter_dict = self.node_if.get_param('filter_dict')
            self.overlay_size_ratio = self.node_if.get_param('overlay_size_ratio')
            self.overlays_dict['overlay_img_name'] = self.node_if.get_param('overlay_img_name')
            self.overlays_dict['overlay_date_time'] = self.node_if.get_param('overlay_date_time')
            self.overlays_dict['overlay_nav'] = self.node_if.get_param('overlay_nav')
            self.overlays_dict['overlay_pose'] = self.node_if.get_param('overlay_pose')
            self.overlays_dict['add_overlay_list'] = self.node_if.get_param('add_overlay_list')
        if do_updates == True:
            pass
        self.zoom_ratio = 0
        self.x_ratio = 0.5
        self.y_ratio = 0.5
        self.x_offset = 0
        self.y_offset = 0
        self.x_scaler = 1
        self.y_scaler = 1

        self.publish_status()

    def reset(self):
        if self.node_if is not None:
            pass
        self.init()

    def factory_reset(self):
        if self.node_if is not None:
            pass
        self.init()




    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _provide_capabilities(self, _):
        return self.caps_report

    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        for namespace in self.add_pubs_dict.keys():
            [img_ns,status_ns,nav_ns] =  self.add_pubs_dict[namespace] 
            has_subs = has_subs or self.node_if.pub_has_subscribers(img_ns)
        if has_subs == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        #self.msg_if.pub_warn("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.publish_status()

    def needs_update(self):
        if self.needs_update_callback is not None:
            self.needs_update_callback()

    def _updateRangesM(self, min_m, max_m):
        if min_m < 0:
            min_m = 0
        if min_m < max_m:
          self.min_range_m = min_m  
          self.max_range_m = max_m  
        else:
          self.msg_if.pub_warn("Invalid ranges supplied: " + str([min_m,max_m]), log_name_list = self.log_name_list)

    def _clickCb(self,msg):
        self.msg_if.pub_info("Received set click message: " + str(msg), log_name_list = self.log_name_list)
        pixel = [int(msg.x  * self.x_scaler + self.x_offset), int(msg.y  * self.y_scaler + self.y_offset)]
        color_bgr = (msg.b,msg.g,msg.r,msg.a)
        #self.msg_if.pub_info("Checking for click_callback function in mouse overide dict: " + str(self.mouse_override_dict), log_name_list = self.log_name_list)
        if self.mouse_override_dict['click_callback'] is not None:
            try:
                self.mouse_override_dict['click_callback'](pixel[0],pixel[1],msg.b,msg.g,msg.r,msg.a)
            except Exception as e:
                self.msg_if.pub_warn("Failed to call mouse click_callback: " + str(e), log_name_list = self.log_name_list)
        else:
                self.last_click_time = nepi_utils.get_time()
                nepi_sdk.sleep(0.3)
                if self.last_click_time is not None:
                    if (nepi_utils.get_time() - self.last_click_time) >= 0.3:
                        self.last_click_time = None
                        self.msg_if.pub_info("Single Click setting pixel value: " + str(pixel), log_name_list = self.log_name_list)
                        self.set_pixel(pixel,color_bgr)
                    else:
                        self.msg_if.pub_info("Double Click resetting render controls", log_name_list = self.log_name_list)
                        self.reset_renders()
                        self.last_click_time = None

 
    def _dragCb(self,msg):
        self.msg_if.pub_info("Received set drag mouse message: " + str(msg), log_name_list = self.log_name_list)
        self.last_click_time = None
        pixel = [int(msg.x  * self.x_scaler + self.x_offset), int(msg.y  * self.y_scaler + self.y_offset)]
        color_bgr = (msg.b,msg.g,msg.r,msg.a)
        self.msg_if.pub_info("Using drag pixel: " + str(pixel), log_name_list = self.log_name_list)
        if self.mouse_override_dict['drag_callback'] is not None:
            try:
                self.mouse_override_dict['drag_callback'](pixel, color_bgr)
            except Exception as e:
                self.msg_if.pub_warn("Failed to call mouse drag_callback: " + str(e), log_name_list = self.log_name_list)
        else: #if self.zoom_ratio < 0.01:
            if self.drag_window is None:
                self.drag_window = [pixel[0], pixel[0] + 1, pixel[1],  pixel[1] + 1]
            else:
                self.drag_window[1] = pixel[0]
                self.drag_window[3] = pixel[1]
                self.needs_update()




    def _windowCb(self,msg):
        self.msg_if.pub_info("Received set window message: " + str(msg), log_name_list = self.log_name_list)
        window = [int(msg.x_min  * self.x_scaler + self.x_offset) , 
                int(msg.x_max  * self.x_scaler + self.x_offset), 
                int(msg.y_min  * self.y_scaler + self.y_offset), 
                int(msg.y_max * self.y_scaler + self.y_offset)]
        if msg.x_min > msg.x_max:
            window[0] = msg.x_max * self.x_scaler + self.x_offset
            window[1] = msg.x_min * self.x_scaler + self.x_offset
        if msg.y_min > msg.y_max:
            window[2] = msg.y_max  * self.y_scaler + self.y_offset
            window[3] = msg.y_min  * self.y_scaler + self.y_offset

        if self.mouse_override_dict['window_callback'] is not None:
            try:
                self.mouse_override_dict['window_callback'](window)
            except Exception as e:
                self.msg_if.pub_warn("Failed to call mouse window_callback: " + str(e), log_name_list = self.log_name_list)
        else:
            self.set_window(window)
            self.needs_update()
        self.drag_window = None





    ########################
    # Filter Callbacks

    def _setFilterEnableCb(self, msg):
        self.msg_if.pub_info("Received Enable Enhacement message: " + str(msg), log_name_list = self.log_name_list)
        name = msg.name
        enabled = msg.active_state
        self.set_filter_enable(name,enabled) 

    def _setFilterRatioCb(self, msg):
        self.msg_if.pub_info("Received Ehnacement Ratio update message: " + str(msg), log_name_list = self.log_name_list)
        name = msg.name
        ratio = msg.ratio
        self.set_filter_ratio(name,ratio) 


    def _setAutoAdjustCb(self, msg):
        self.msg_if.pub_info("Received Auto Adjust Enable update message: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.set_auto_adjust_enable(enabled)

    def _setAutoAdjustRatioCb(self, msg):
        self.msg_if.pub_info("Received Auto Adjust Ratio update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_auto_adjust_ratio(ratio)

    def _setBrightnessCb(self, msg):
        self.msg_if.pub_info("Received Brightness update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_brightness_ratio(ratio)


    def _setContrastCb(self, msg):
        self.msg_if.pub_info("Received Contrast update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_contrast_ratio(ratio)
        


    def _setThresholdingCb(self, msg):
        self.msg_if.pub_info("Received Threshold update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_threshold_ratio(ratio)



    ########################
    # Res and Orientation Callbacks

    def _setResolutionRatioCb(self, msg):
        self.msg_if.pub_info("Received Resolution update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_resolution_ratio(ratio)


    def _setRotate2dCb(self, msg):
        self.msg_if.pub_info("Received Rotate 2d Deg update message: " + str(msg), log_name_list = self.log_name_list)
        cur_angle = self.controls_dict['rotate_2d_deg']
        new_angle = cur_angle + 90
        if new_angle >= 360:
            new_angle = 0
        new_angle = int(round(new_angle/90.0,0) * 90)
        self.set_rotate_2d_deg(new_angle)

    def _setFlipHorzCb(self, msg):
        self.msg_if.pub_info("Received Flip Horz update message: " + str(msg), log_name_list = self.log_name_list)
        enable = msg.data
        self.set_flip_horz(enable)

    def _setFlipVertCb(self, msg):
        self.msg_if.pub_info("Received Flip Vert update message: " + str(msg), log_name_list = self.log_name_list)
        enable = msg.data
        self.set_flip_vert(enable)

    ########################
    # Render Callbacks

    def _setRangeCb(self, msg):
        self.msg_if.pub_info("Received Range update message: " + str(msg), log_name_list = self.log_name_list)
        start_ratio = msg.start_range
        stop_ratio = msg.stop_range
        self.set_range_ratios(start_ratio,stop_ratio)
      

    def _setZoomCb(self, msg):
        self.msg_if.pub_info("Received Zoom update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_zoom_ratio(ratio)

    def _setPanXCb(self, msg):
        self.msg_if.pub_info("Received Pan Left Right update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_x_ratio(ratio)


    def _setPanYCb(self, msg):
        self.msg_if.pub_info("Received Pan Up Down update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_y_ratio(ratio)


    def _setRotateCb(self, msg):
        self.msg_if.pub_info("Received Rotate update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_rotate_3d_ratio(ratio) 

    def _setTiltCb(self, msg):
        ratio = msg.data
        self.set_tilt_3d_ratio(ratio) 



    ########################
    # Render Callbacks

    def _setOverlaySizeCb(self,msg):
        ratio = msg.data
        self.set_overlay_size_ratio(ratio)


    def _setOverlayImgNameCb(self,msg):
        enabled = msg.data
        self.set_overlay_image_name(enabled)

    def _setOverlayDateTimeCb(self,msg):
        enabled = msg.data
        self.set_overlay_date_time(enabled)

    def _setOverlayNavCb(self,msg):
        enabled = msg.data
        self.set_overlay_nav(enabled)

    def _setOverlayPoseCb(self,msg):
        enabled = msg.data
        self.set_overlay_pose(enabled)

    def _setOverlayListCb(self,msg):
        overlay_list = msg.data
        self.set_overlay_list(overlay_list)


    def _setOverlayTextCb(self,msg):
        overlay_text = msg.data
        self.set_overlay_text(overlay_list)


    def _clearOverlayListCb(self,msg):
        self.clear_overlay_list()

    def _resetControlsCb(self,msg):
        self.reset_filters()
        self.reset_overlays()
        self.reset_res_orients()
        self.reset_renders()

    def _resetFiltersCb(self,msg):
        self.reset_filters()

    def _resetOverlaysCb(self,msg):
        self.reset_overlays()

    def _resetResOrientsCb(self,msg):
        self.reset_res_orients()
    
    def _resetRendersCb(self,msg):
        self.msg_if.pub_warn("Received reset renders message", log_name_list = self.log_name_list)
        self.reset_renders()

    def _navposeCb(self,msg):
        self.navpose_dict = nepi_nav.convert_navpose_msg2dict(msg,self.log_name_list)

##################################################
# ImageIF

class ImageIF(BaseImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = False,
        has_flip_horz = False,
        has_flip_vert = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate_3d = False,
        has_tilt_3d = False
        )

    DEFAULT_FILTERS_DICT = dict(
    )

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5
        )

    params_dict = None
    services_dict = None
    pubs_dict = None
    subs_dict = None

    data_product = 'image'


    auto_adjust_controls = []


    
    def __init__(self, namespace = None , 
                data_product_name = 'image',
                data_source_description = 'image',
                data_ref_description = 'image',
                perspective = 'pov',
                init_overlay_list = [],
                get_navpose_function = None,
                pub_navpose = False,
                needs_update_callback = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):

        self.data_product = data_product_name
        # Call the parent class constructor
        super().__init__(namespace , 
                self.data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT, 
                self.params_dict,
                self.services_dict,
                self.pubs_dict,
                self.subs_dict,
                pub_navpose,
                needs_update_callback,
                init_overlay_list,
                get_navpose_function,
                log_name,
                log_name_list,
                msg_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def process_cv2_img(self, cv2_img):
        return cv2_img


    ###############################
    # Class Private Methods
    ###############################


##################################################
# ColorImageIF

class ColorImageIF(BaseImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = True,
        has_auto_adjust = True,
        has_contrast = True,
        has_brightness = True,
        has_threshold = True,
        has_rotate_2d = True,
        has_flip_horz = True,
        has_flip_vert = True,
        has_range = False,
        has_zoom = True,
        has_pan = True,
        has_window = True,
        has_rotate_3d = False,
        has_tilt_3d = False
        )

    DEFAULT_FILTERS_DICT = dict(
        # Low_Light = {
        #     'enabled': False,
        #     'function': nepi_img.low_light_filter,
        #     'ratio': 0.5
        # }
    )

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5
        )

    params_dict = None
    services_dict = None
    pubs_dict = None
    subs_dict = None

    data_product = 'color_image'


    auto_adjust_controls = ['brightness','contrast','threshold']


    
    def __init__(self, namespace = None , 
                data_product_name = 'color_image',
                data_source_description = 'imaging_sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                init_overlay_list = [],
                get_navpose_function = None,
                pub_navpose = False,
                needs_update_callback = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):

        self.data_product = data_product_name
        # Call the parent class constructor
        super().__init__(namespace , 
                self.data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT, 
                self.params_dict,
                self.services_dict,
                self.pubs_dict,
                self.subs_dict,
                pub_navpose,
                needs_update_callback,
                init_overlay_list,
                get_navpose_function,
                log_name,
                log_name_list,
                msg_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def process_cv2_img(self, cv2_img):
        cv2_shape = cv2_img.shape
        img_width = cv2_shape[1] 
        img_height = cv2_shape[0] 
        ratio = img_width / img_height
        #self.msg_if.pub_info("Image Raw: " + str(cv2_img.shape), log_name_list = self.log_name_list)

        #####################
        # Apply render controls 
        [xr_min,xr_max,yr_min,yr_max] = copy.deepcopy(self.controls_dict['window_ratios'])
        x_min = int(max(0, img_width * xr_min )) 
        x_max = int(min(img_width, img_width * xr_max))
        y_min = int(max(0, img_height * yr_min))
        y_max = int(min(img_height, img_height * yr_max))

        #self.msg_if.pub_warn("Got Image Window: " + str([x_min,x_max,y_min,y_max]), log_name_list = self.log_name_list)

        ##########
        # Show Drag Box if Needed
        drag_window = copy.deepcopy(self.drag_window)

        #self.msg_if.pub_info("Processing drag_window" + str(drag_window), log_name_list = self.log_name_list)
        if drag_window is not None:
            #self.msg_if.pub_info("Processing drag_window" + str(drag_window), log_name_list = self.log_name_list)
            # Define the rectangle parameters
            x1 = min(drag_window[0], drag_window[1])
            x2 = max(drag_window[0], drag_window[1])
            y1 = min(drag_window[2], drag_window[3])
            y2 = max(drag_window[2], drag_window[3])
            color = (0, 200, 0) # Green color in BGR
            alpha = 0.4 # Transparency factor (0.0 for fully transparent, 1.0 for fully opaque)

            # Draw a filled rectangle on the overlay copy
            overlay = cv2_img.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)

            # Blend the overlay with the original image using cv2.addWeighted()
            # The result is stored back into the original 'image' variable (or a new one)
            cv2_img = cv2.addWeighted(overlay, alpha, cv2_img, 1 - alpha, 0)


        cv2_img = cv2_img[y_min:y_max, x_min:x_max]



        #self.msg_if.pub_info("Image Render: " + str(cv2_img.shape), log_name_list = self.log_name_list)




        ##########
        # Apply Resolution Controls
        res_ratio = self.controls_dict['resolution_ratio']
        cv2_shape = cv2_img.shape
        img_width1 = cv2_shape[1] 
        img_height1 = cv2_shape[0] 
        if res_ratio < 0.9:
            [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)

        ##########
        # Apply Oreantation Controls
        degrees = self.controls_dict['rotate_2d_deg']
        fliph = self.controls_dict['flip_horz']
        flipv = self.controls_dict['flip_vert']

        if degrees != 0:
           cv2_img = nepi_img.rotate_degrees(cv2_img,degrees) 

        if fliph == True:
            cv2_img = nepi_img.flip_horz(cv2_img) 

        if flipv == True:
            cv2_img = nepi_img.flip_vert(cv2_img) 

        #self.msg_if.pub_info("Image Orient: " + str(cv2_img.shape), log_name_list = self.log_name_list)

        ###################
        # Apply Filters
        for filter_name in self.filter_dict.keys():
            enabled = self.filter_dict[filter_name]['enabled']
            if enabled == True:
                ratio = self.filter_dict[filter_name]['ratio']
                if ratio > 0.05:
                    function = self.filter_dict[filter_name]['function']
                    cv2_img = function(cv2_img,ratio)


        ##########
        # Apply Adjustment Controls
        auto = self.controls_dict['auto_adjust_enabled']
        auto_ratio = self.controls_dict['auto_adjust_ratio']
        brightness = self.controls_dict['contrast_ratio']
        contrast = self.controls_dict['brightness_ratio']
        threshold = self.controls_dict['threshold_ratio']

        if auto is False:
            cv2_img = nepi_img.adjust_brightness(cv2_img, brightness)
            cv2_img = nepi_img.adjust_contrast(cv2_img, contrast)
            cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold)
        else:
            cv2_img = nepi_img.adjust_auto(cv2_img,auto_ratio)

        #self.msg_if.pub_info("Image Filter: " + str(cv2_img.shape), log_name_list = self.log_name_list)

    

        ##########
        # Update last image info
        cv2_shape = cv2_img.shape
        img_width2 = cv2_shape[1] 
        img_height2 = cv2_shape[0] 

        self.x_offset = x_min 
        self.y_offset = y_min
        self.x_scaler = img_width1 / img_width2
        self.y_scaler = img_height1 / img_height2

        return cv2_img




    ###############################
    # Class Private Methods
    ###############################




##################################################
# DepthMapIF

class DepthMapIF:

    DEFUALT_IMG_WIDTH_PX = 700
    DEFUALT_IMG_HEIGHT_PX = 400

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    ready = False
    namespace = '~'

    node_if = None

    status_msg = DepthMapStatus()

    last_width = DEFUALT_IMG_WIDTH_PX
    last_height = DEFUALT_IMG_HEIGHT_PX

    blank_img = nepi_img.create_cv2_blank_img(DEFUALT_IMG_WIDTH_PX, DEFUALT_IMG_HEIGHT_PX, color = (0, 0, 0) )

    last_pub_time = None

    has_subs = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    #img_pub_file = 'nepi_depth_map_img_pub_node.py'

    min_range_m = 0.0
    max_range_m = 1.0

    data_source_description = 'depth_map_sensor'
    data_ref_description = 'sensor'

    data_product = 'depth_map'
    data_products_list = [data_product]

    image_if = None
    navpose_if = None
    needs_update_callback = None
    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    get_navpose_function = None    
    has_navpose = False

    def __init__(self, namespace = None,
                data_product_name = 'depth_map',
                data_source_description = 'depth_map_sensor',
                data_ref_description = 'sensor',
                perspective = 'POV',
                pub_image = True,
                get_navpose_function = None,
                pub_navpose = False,
                needs_update_callback = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        # Initialize Class Variables
        self.data_product = data_product_name

        self.perspective = perspective
        self.pub_image = pub_image
        self.pub_navpose = pub_navpose
        if get_navpose_function is not None and pub_navpose == True:
            self.get_navpose_function = get_navpose_function
            self.has_navpose = True


        self.needs_update_callback = needs_update_callback

        self.msg_if.pub_warn("Got namespace: " + str(namespace), log_name_list = self.log_name_list)
        if namespace is None:
            namespace = self.namespace
        namespace = nepi_sdk.get_full_namespace(namespace)
        self.namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        self.msg_if.pub_warn("Using data product namespace: " + str(self.namespace), log_name_list = self.log_name_list)

        '''
                default_min_meters = 0.0,
                default_max_meters = 20.0,
        self._updateRangesM(default_min_meters,default_max_meters)
        '''

        # Initialize Status Msg.  Updated on each publish

        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description

        self.status_msg.publishing = False
        self.status_msg.encoding = '32FC1'
        self.status_msg.width_px = 0
        self.status_msg.height_px = 0
        self.status_msg.frame_3d = "sensor_frame"
        self.status_msg.get_latency_time = 0
        self.status_msg.pub_latency_time = 0
        self.status_msg.process_time = 0
        self.status_msg.img_pub_enabled = pub_image
    


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'data_pub': {
                'msg': Image,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg': DepthMapStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
            'navpose_pub': {
                'msg': NavPose,
                'namespace': self.namespace,
                'topic': 'navpose',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = dict()


        if self.has_navpose == False:
            ##############################
            ## Connect NEPI NavPose
            self.SUBS_DICT['navpose_sub'] = {
                'namespace': self.base_namespace,
                'topic': 'navpose',
                'msg': NavPose,
                'qsize': 1,
                'callback': self._navposeCb, 
                'callback_args': ()
            }


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        success = nepi_sdk.wait()

        self.init(do_updates = True)

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)


        ###############################
        #Setup Image Pub if needed
        if pub_image == True:
            img_data_product = get_image_data_product(self.data_product)
            self.data_products_list = self.data_products_list + [img_data_product]
            self.image_if = DepthMapImageIF(namespace = self.namespace, 
                        data_product_name = img_data_product, 
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        perspective = self.perspective,
                        get_navpose_function = self.get_navpose_dict,
                        needs_update_callback = self.needs_update_callback,
                        log_name = img_data_product,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )


        '''
        ##################################
        # Setup navpose data IF if needed
        np_namespace = self.namespace
        if pub_navpose is not None:
            if pub_navpose == True:
                self.navpose_if = NavPoseIF(namespace = np_namespace,
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            log_name = 'navpose',
                            log_name_list = [],
                            msg_if = self.msg_if
                            )
        '''
        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_data_source_description(self):
        return self.data_source_description


    def get_data_products(self):
        return self.data_products_list

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        return self.has_subs

    def get_navpose_dict(self):
        navpose_dict = None
        if self.get_navpose_function is not None:
            navpose_dict = self.get_navpose_function()
        else:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return navpose_dict

    def publish_cv2_depth_map(self, cv2_depth_map, 
                            encoding = '32FC1',
                            width_deg = 100,
                            height_deg = 70,
                             min_range_m = None, 
                             max_range_m = None,
                             timestamp = None,
                             frame_3d = 'sensor_frame',
                             device_mount_description = 'fixed',
                             pub_twice = False):

                                                    

        self.msg_if.pub_debug("Got Image to Publish", log_name_list = self.log_name_list, throttle_s = 5.0)
        success = False
        if cv2_depth_map is None and self.status_msg is not None:
            self.msg_if.pub_info("Can't publish None image", log_name_list = self.log_name_list)
            return False

        self.status_msg.device_mount_description = device_mount_description

        self.status_msg.encoding = encoding

        if timestamp == None:
            timestamp = nepi_utils.get_time()
        else:
            timestamp = nepi_sdk.sec_from_timestamp(timestamp)


        current_time = nepi_utils.get_time()
        latency = (current_time - timestamp)
        self.status_msg.get_latency_time = latency
        self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency), log_name_list = self.log_name_list, throttle_s = 5.0)

        # Start Img Pub Process
        start_time = nepi_utils.get_time()   

        # Publish and Save Raw Image Data if Required  
        [height,width] = cv2_depth_map.shape[0:2]
        last_width = self.status_msg.width_px
        last_height = self.status_msg.height_px
        self.status_msg.width_px = width
        self.status_msg.height_px = height

        self.status_msg.width_deg = width_deg
        self.status_msg.height_deg = height_deg

        if (min_range_m is not None and max_range_m is not None):
            self._updateRangesM(min_range_m,max_range_m)
        else:
            self._updateRangesM(0,1)


        self.status_msg.publishing = True

        navpose_dict = self.get_navpose_dict()
        if self.node_if is not None and self.has_subs == True:
            #Convert to ros Image message
            ros_img = nepi_img.cv2img_to_rosimg(cv2_depth_map, encoding=encoding)
            sec = nepi_sdk.sec_from_timestamp(timestamp)
            ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = frame_3d)
            #self.msg_if.pub_debug("Publishing Image with header: " + str(ros_img.header), log_name_list = self.log_name_list, throttle_s = 5.0)
            self.node_if.publish_pub('data_pub', ros_img)
            if pub_twice == True:
                nepi_sdk.sleep(0.01)
                self.node_if.publish_pub('data_pub', ros_img)
        process_time = round( (nepi_utils.get_time() - start_time) , 3)
        self.status_msg.process_time = process_time
        latency = (current_time - timestamp)
        self.status_msg.pub_latency_time = latency

        cv2_depth_map_img = None
        if self.image_if is not None:
            #self.msg_if.pub_warn("Publishing Image with width_deg: " + str(width_deg), log_name_list = self.log_name_list)
            cv2_depth_map_img = self.image_if.publish_cv2_depth_map_img(cv2_depth_map,
                                width_deg = width_deg,
                                height_deg = height_deg,
                                min_range_m = min_range_m, 
                                max_range_m = max_range_m,
                                timestamp = timestamp,
                                frame_3d = frame_3d,
                                device_mount_description = device_mount_description,
                                navpose_dict = navpose_dict,
                                pub_twice = pub_twice
                            )

        if self.last_pub_time is None:
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
            self.status_msg.last_pub_sec = pub_time_sec

            self.time_list.pop(0)
            self.time_list.append(pub_time_sec)

        navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
        if navpose_msg is not None:
            self.node_if.publish_pub('navpose_pub', navpose_msg)

        return cv2_depth_map, cv2_depth_map_img

    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_sdk.wait()
        self.namespace = '~'
        self.status_msg = None


    def publish_status(self, do_updates = True):
        if self.node_if is not None:
            self.status_msg.min_range_m = self.min_range_m
            self.status_msg.max_range_m = self.max_range_m
            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate

            self.node_if.publish_pub('status_pub',self.status_msg)



    def init(self, do_updates = False):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        if self.node_if is not None:
            pass
        self.init()

    def factory_reset(self):
        if self.node_if is not None:
            pass
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if self.image_if is not None:
            has_subs = has_subs or self.image_if.has_subscribers_check() == True
        if has_subs == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        #self.msg_if.pub_warn("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.publish_status()

    def _updateRangesM(self, min_m, max_m):
        if min_m < 0:
            min_m = 0
        if min_m < max_m:
          self.min_range_m = min_m  
          self.max_range_m = max_m  
        else:
          self.msg_if.pub_warn("Invalid ranges supplied: " + str([min_m,max_m]), log_name_list = self.log_name_list)

    def _navposeCb(self,msg):
        self.navpose_dict = nepi_nav.convert_navpose_msg2dict(msg,self.log_name_list)





##################################################
# DepthMapImageIF

class DepthMapImageIF(BaseImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = True,
        has_flip_horz = True,
        has_flip_vert = True,
        has_range = True,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate_3d = False,
        has_tilt_3d = False
        )

    DEFAULT_FILTERS_DICT = dict()

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5
        )

    params_dict = None
    services_dict = None
    pubs_dict = None
    subs_dict = None

    data_product = 'depth_map'

    auto_adjust_controls = []


    def __init__(self, namespace = None , 
                data_product_name = 'depth_map_image',
                data_source_description = 'depth_map_sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                init_overlay_list = [],
                get_navpose_function = None,
                pub_navpose = False,
                needs_update_callback = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):


        self.data_product = data_product_name
        # Call the parent class constructor
        super().__init__(namespace , 
                self.data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT, 
                self.params_dict,
                self.services_dict,
                self.pubs_dict,
                self.subs_dict,
                pub_navpose,
                needs_update_callback,
                init_overlay_list,
                get_navpose_function,
                log_name,
                log_name_list,
                msg_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################



    ###############################
    # Class Public Methods
    ###############################

    def publish_cv2_depth_map_img(self,cv2_depth_map,
                            width_deg = 100,
                            height_deg = 70,
                            min_range_m = 0, 
                            max_range_m = 1,
                            timestamp = None,
                            frame_3d = 'sensor_frame',
                            device_mount_description = 'fixed',
                            navpose_dict = None,
                            pub_twice = False
                            ):

                
        cv2_img = None
        if cv2_depth_map is not None:
            start_range_ratio = self.controls_dict['start_range_ratio']
            stop_range_ratio = self.controls_dict['stop_range_ratio']
            depth_data = (np.array(cv2_depth_map, dtype=np.float32)) # replace nan values
            # Get range data
            delta_range_m = max_range_m - min_range_m
            # Adjust range Limits if IDX Controls enabled and range ratios are not min/max
            max_range_m = min_range_m + stop_range_ratio * delta_range_m
            min_range_m = min_range_m + start_range_ratio * delta_range_m
            delta_range_m = max_range_m - min_range_m
            # Filter depth_data in range
            depth_data[np.isnan(depth_data)] = max_range_m 
            depth_data[depth_data <= min_range_m] = max_range_m # set to max
            depth_data[depth_data >= max_range_m] = max_range_m # set to max
            # Create colored cv2 depth image
            depth_data = depth_data - min_range_m # Shift down 
            depth_data = np.abs(depth_data - max_range_m) # Reverse for colormap
            depth_data = np.array(255*depth_data/delta_range_m,np.uint8) # Scale for bgr colormap
            cv2_img = cv2.applyColorMap(depth_data, cv2.COLORMAP_JET)

            self.publish_cv2_img(cv2_img,
                                encoding = 'bgr8',
                                width_deg = width_deg,
                                height_deg = height_deg,
                                min_range_m = min_range_m, 
                                max_range_m = max_range_m,
                                timestamp = timestamp,
                                frame_3d = frame_3d,
                                device_mount_description = device_mount_description,
                                navpose_dict = navpose_dict,
                                pub_twice = pub_twice
                             )
            return cv2_img


    ###############################
    # Class Private Methods
    ###############################




##################################################
# PointcloudImageIF

class PointcloudImageIF(BaseImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_rotate_2d = True,
        has_flip_horz = True,
        has_flip_vert = True,
        has_range = True,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate_3d = False,
        has_tilt_3d = False
        )

    DEFAULT_FILTERS_DICT = dict()

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5
        )

    params_dict = None
    services_dict = None
    pubs_dict = None
    subs_dict = None

    data_product = 'depth_map'

    auto_adjust_controls = []

    def __init__(self, namespace = None , 
                data_product_name = 'depth_map_image',
                data_source_description = 'depth_map_sensor',
                data_ref_description = 'sensor',
                perspective = 'pov',
                init_overlay_list = [],
                get_navpose_function = None,
                pub_navpose = False,
                needs_update_callback = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):


        self.data_product = data_product_name
        # Call the parent class constructor
        super().__init__(namespace , 
                self.data_product,
                data_source_description,
                data_ref_description,
                perspective,
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_FILTERS_DICT, 
                self.params_dict,
                self.services_dict,
                self.pubs_dict,
                self.subs_dict,
                pub_navpose,
                needs_update_callback,
                init_overlay_list,
                get_navpose_function,
                log_name,
                log_name_list,
                msg_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################



    ###############################
    # Class Public Methods
    ###############################

    def publish_cv2_depth_map_img(self,cv2_depth_map,
                            width_deg = 100,
                            height_deg = 70,
                            min_range_m = 0, 
                            max_range_m = 1,
                            timestamp = None,
                            frame_3d = 'sensor_frame',
                            device_mount_description = 'fixed',
                            navpose_dict = None,
                            pub_twice = False
                            ):

                
        cv2_img = None
        if cv2_depth_map is not None:
            start_range_ratio = self.controls_dict['start_range_ratio']
            stop_range_ratio = self.controls_dict['stop_range_ratio']
            depth_data = (np.array(cv2_depth_map, dtype=np.float32)) # replace nan values
            # Get range data
            delta_range_m = max_range_m - min_range_m
            # Adjust range Limits if IDX Controls enabled and range ratios are not min/max
            max_range_m = min_range_m + stop_range_ratio * delta_range_m
            min_range_m = min_range_m + start_range_ratio * delta_range_m
            delta_range_m = max_range_m - min_range_m
            # Filter depth_data in range
            depth_data[np.isnan(depth_data)] = max_range_m 
            depth_data[depth_data <= min_range_m] = max_range_m # set to max
            depth_data[depth_data >= max_range_m] = max_range_m # set to max
            # Create colored cv2 depth image
            depth_data = depth_data - min_range_m # Shift down 
            depth_data = np.abs(depth_data - max_range_m) # Reverse for colormap
            depth_data = np.array(255*depth_data/delta_range_m,np.uint8) # Scale for bgr colormap
            cv2_img = cv2.applyColorMap(depth_data, cv2.COLORMAP_JET)

            self.publish_cv2_img(cv2_img,
                                encoding = 'bgr8',
                                width_deg = width_deg,
                                height_deg = height_deg,
                                min_range_m = min_range_m, 
                                max_range_m = max_range_m,
                                timestamp = timestamp,
                                frame_3d = frame_3d,
                                device_mount_description = device_mount_description,
                                navpose_dict = navpose_dict,
                                pub_twice = pub_twice
                             )
            return cv2_img


    ###############################
    # Class Private Methods
    ###############################








##################################################
# PointcloudIF



class PointcloudIF:

    DEFAULT_WIDTH_DEG = 100
    DEFAULT_HEIGHT_DEG = 70

    Factory_Image_Width = 955
    Factory_Image_Height = 600
    Factory_Zoom_Ratio = .5
    Factory_Rotate_3D_Ratio = .5
    Factory_Tilt_3D_Ratio = .5
    Factory_Cam_FOV = 60
    Factory_Cam_View = [3, 0, 0]
    Factory_Cam_Pos = [-5, 0, 0]
    Factory_Cam_Rot = [0, 0, 1]

    #Default Control Values 
    DEFAULT_CONTROLS_DICT = dict( 
        resolution_ratio = 1.0,
        auto_adjust_enabled = False,
        auto_adjust_ratio = 0.3,
        brightness_ratio = 0.5,
        contrast_ratio =  0.5,
        threshold_ratio =  0.0,
        start_range_ratio = 0.0,
        stop_range_ratio = 1.0,
        window_ratios = [0,1,0,1],
        rotate_3d_ratio = 0.5,
        tilt_3d_ratio = 0.5
        )

    ready = False
    namespace = '~'

    node_if = None

    status_msg =  PointcloudStatus()

    last_pub_time = None

    has_subs = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    #img_pub_file = 'nepi_pointcloud_img_pub_node.py'

    min_range_m = 0
    max_range_m = 1

    data_source_description = 'pointcloud_sensor'
    data_ref_description = 'sensor'

    data_product = 'pointcloud'
    data_products_list = [data_product]

    image_if = None
    navpose_if = None
    navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    get_navpose_function = None    
    has_navpose = False

    voxel_callback = None
    voxel = [0,0,0]

    needs_update_callback = None

    def __init__(self, namespace = None,
                data_product_name = 'pointcloud',
                data_source_description = 'pointcloud_sensor',
                data_ref_description = 'sensor',
                perspective = 'POV',
                pub_image = True,
                get_navpose_function = None,
                pub_navpose = False,
                voxel_callback = None,
                needs_update_callback = None,
                init_overlay_list = [],
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

        ##############################    
        # Initialize Class Variables
        self.data_product = data_product_name
        self.data_products_list = self.data_products_list + [get_image_data_product(self.data_product)]
        self.perspective = perspective
        self.pub_image = pub_image
        self.pub_navpose = pub_navpose
        if get_navpose_function is not None and pub_navpose == True:
            self.get_navpose_function = get_navpose_function
            self.has_navpose = True
 
        self.init_overlay_list = init_overlay_list

  
        self.msg_if.pub_warn("Got namespace: " + str(namespace), log_name_list = self.log_name_list)
        if namespace is None:
            namespace = self.namespace
        namespace = nepi_sdk.get_full_namespace(namespace)
        self.namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        self.msg_if.pub_warn("Using data product namespace: " + str(self.namespace), log_name_list = self.log_name_list)


        # Initialize Status Msg.  Updated on each publish
        if data_source_description is None:
            data_source_description = self.data_source_description
        self.data_source_description = data_source_description

        if data_ref_description is None:
            data_ref_description = self.data_ref_description
        self.data_ref_description = data_ref_description

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description

        self.status_msg.node_namespace = self.node_namespace
        self.status_msg.publishing = False
        self.status_msg.has_rgb = False
        self.status_msg.has_intensity = False
        self.status_msg.width = 0
        self.status_msg.height = 0
        self.status_msg.depth = 0
        self.status_msg.point_count = 0,
        self.status_msg.frame_3d = "sensor_frame"
        self.status_msg.get_latency_time
        self.status_msg.pub_latency_time
        self.status_msg.process_time
        self.status_msg.data_pub_enabled = pub_image
        self.status_msg.standard_image_sizes = nepi_img.STANDARD_IMAGE_SIZES


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = None

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'data_pub': {
                'msg': PointCloud2,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg':  PointcloudStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            },
            'navpose_pub': {
                'msg': NavPose,
                'namespace': self.namespace,
                'topic': 'navpose',
                'qsize': 1,
                'latch': False
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = dict()


        if self.has_navpose == False: 
            ##############################
            ## Connect NEPI NavPose
            self.SUBS_DICT['navpose_sub'] = {
                'namespace': self.base_namespace,
                'topic': 'navpose',
                'msg': NavPose,
                'qsize': 1,
                'callback': self._navposeCb, 
                'callback_args': ()
            }



        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                                            )

        
        success = nepi_sdk.wait()

        self.init(do_updates = True)

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        nepi_sdk.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ###############################
        #Setup Image Pub if needed
        # if pub_image == True:
        #     img_data_product = get_image_data_product(self.data_product)
        #     self.data_products_list = self.data_products_list + [img_data_product]
        #     image_if = PointcloudImageIF(namespace = self.namespace, 
        #                 data_product_name = img_data_product, 
        #                 data_source_description = self.data_source_description,
        #                 data_ref_description = self.data_ref_description,
        #                 perspective = self.perspective,
        #                 init_overlay_list = self.init_overlay_list,
        #                 get_navpose_function = self.get_navpose_function,
        #                 needs_update_callback = needs_update_callback,
        #                 log_name = img_data_product,
        #                 log_name_list = self.log_name_list,
        #                 msg_if = self.msg_if
        #                 )


        '''
        ################################
        # Setup navpose data IF if needed
        np_namespace = self.namespace
        if pub_navpose is not None:
            if pub_navpose == True:
                self.navpose_if = NavPoseIF(namespace = np_namespace,
                            data_source_description = self.data_source_description,
                            data_ref_description = self.data_ref_description,
                            log_name = 'navpose',
                            log_name_list = [],
                            msg_if = self.msg_if
                            )
        '''


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################


    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_utils.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_utils.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_data_source_description(self):
        return self.data_source_description


    def get_data_products(self):
        return self.data_products_list

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        has_subs = copy.deepcopy(self.has_subs)
        self.msg_if.pub_warn("Returning: " + self.namespace + " " "has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        return has_subs

    def get_navpose_dict(self):
        navpose_dict = None
        if self.get_navpose_function is not None:
            navpose_dict = self.get_navpose_function()
        else:
            navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        return navpose_dict


    def _setVoxelCb(self,msg):
        self.msg_if.pub_info("Received set voxel message: " + str(msg), log_name_list = self.log_name_list)
        self.voxel = [msg.x,msg.y,msg.z]
        if self.voxel_callback is not None:
            self.voxel_callback(self.voxel)

    def publish_o3d_pc(self,o3d_pc,
                        timestamp = None, 
                        frame_3d = 'sensor_frame', 
                        width_deg = 100,
                        height_deg = 70,
                        min_range_m = None,
                        max_range_m = None,
                        device_mount_description = 'fixed',
                        navpose_dict = None,
                        process_data = True,
                        pub_twice = False,
                        add_pubs = []):
        

        if self.node_if is None and self.status_msg is not None:
            self.msg_if.pub_info("Can't publish on None publisher", log_name_list = self.log_name_list)
            return False
        if o3d_pc is None:
            self.msg_if.pub_info("Can't publish None image", log_name_list = self.log_name_list)
            return False

        self.status_msg.device_mount_description = device_mount_description

        if timestamp == None:
            timestamp = nepi_utils.get_time()
        else:
            timestamp = nepi_sdk.sec_from_timestamp(timestamp)

        self.status_msg.has_rgb = o3d_pc.has_colors()
        self.status_msg.has_intensity = False # Need to add
        #self.status_msg.point_count = o3d_pc.point["colors"].shape[0]

        self.status_msg.width_deg = width_deg
        self.status_msg.height_deg = height_deg

        if (min_range_m is not None and max_range_m is not None):
            self._updateRangesM(min_range_m,max_range_m)
            self.status_msg.min_range_m = self.min_range_m
            self.status_msg.max_range_m = self.max_range_m
        else:
            self.status_msg.min_range_m = 0
            self.status_msg.max_range_m = 1

        current_time = nepi_utils.get_time()
        latency = (current_time - timestamp)
        self.status_msg.get_latency_time = latency
        #self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency), log_name_list = self.log_name_list, throttle_s = 5.0)

        # Start Img Pub Process
        start_time = nepi_utils.get_time()   

        self.status_msg.point_count = o3d_pc.point["colors"].shape[0]


        self.status_msg.publishing = True



        process_time = round( (nepi_utils.get_time() - start_time) , 3)
        self.status_msg.process_time = process_time
        latency = (current_time - timestamp)
        self.status_msg.pub_latency_time = latency


        if self.last_pub_time is None:
            self.last_pub_time = nepi_utils.get_time()
        else:
            cur_time = nepi_utils.get_time()
            pub_time_sec = cur_time - self.last_pub_time
            self.last_pub_time = cur_time
            self.status_msg.last_pub_sec = pub_time_sec

            self.time_list.pop(0)
            self.time_list.append(pub_time_sec)

        navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
        if navpose_msg is not None:
            self.node_if.publish_pub('navpose_pub', navpose_msg)
            
        return o3d_pc

    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_sdk.wait()
        self.node_if = None
        self.namespace = '~'
        self.status_msg = None


    def publish_status(self, do_updates = True):
        if self.node_if is not None and self.status_msg is not None:
            if self.status_msg is None:
                self.status_msg =  PointcloudStatus()
            if do_updates == True:
                self.status_msg.image_width = self.node_if.get_param('image_width')
                self.status_msg.image_height = self.node_if.get_param('image_height')

                range_ratios = RangeWindow()
                range_ratios.start_range =   float(self.node_if.get_param('start_range_ratio'))
                range_ratios.stop_range =   float(self.node_if.get_param('stop_range_ratio'))
                self.status_msg.range_ratios = range_ratios

                self.status_msg.zoom_ratio = self.node_if.get_param('zoom_ratio')
                self.status_msg.rotate_3d_ratio = self.node_if.get_param('rotate_3d_ratio')
                self.status_msg.tilt_3d_ratio = self.node_if.get_param('tilt_3d_ratio')

                fov = self.node_if.get_param('cam_fov')
                self.status_msg.camera_fov = fov

                view = self.node_if.get_param('cam_view')
                cam_view = Vector3()
                cam_view.x = view[0]
                cam_view.y = view[1]
                cam_view.z = view[2]
                self.status_msg.camera_view = cam_view

                pos = self.node_if.get_param('cam_pos')
                cam_pos = Vector3()
                cam_pos.x = pos[0]
                cam_pos.y = pos[1]
                cam_pos.z = pos[2]
                self.status_msg.camera_position = cam_pos

                rot = self.node_if.get_param('cam_rot')
                cam_rot = Vector3()
                cam_rot.x = rot[0]
                cam_rot.y = rot[1]
                cam_rot.z = rot[2]
                self.status_msg.camera_rotation = cam_rot
                
                use_wbg = self.node_if.get_param('use_wbg')
                self.status_msg.white_background = use_wbg

            avg_rate = 0
            if len(self.time_list) > 0:
                avg_time = sum(self.time_list) / len(self.time_list)
                if avg_time > .01:
                    avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate

            self.node_if.publish_pub('status_pub',self.status_msg)


    def init(self, do_updates = False):
        if self.node_if is not None:
            pass
        if do_updates == True:
            pass
        self.publish_status()

    def reset(self):
        if self.node_if is not None:
            pass
        self.init()

    def factory_reset(self):
        if self.node_if is not None:
            pass
        self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init(do_updates = do_updates)

    def _resetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _factoryResetCb(self, do_updates = True):
        self.init(do_updates = do_updates)

    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if self.image_if is not None:
            has_subs = has_subs or self.image_if.has_subscribers_check() == True
        if has_subs == False and self.status_msg is not None:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        self.msg_if.pub_debug("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        self.publish_status(do_updates = True)

    def _updateRangesM(self, min_m, max_m):
        if min_m < 0:
            min_m = 0
        if min_m < max_m:
          self.min_range_m = min_m  
          self.max_range_m = max_m  
        else:
          self.msg_if.pub_warn("Invalid ranges supplied: " + str([min_m,max_m]), log_name_list = self.log_name_list)

    def _navposeCb(self,msg):
        self.navpose_dict = nepi_nav.convert_navpose_msg2dict(msg,self.log_name_list)
