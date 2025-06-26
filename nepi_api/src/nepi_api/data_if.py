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
import cv2
import open3d as o3d

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
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
from nepi_interfaces.srv import NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryRequest, NavPoseCapabilitiesQueryResponse



from nepi_interfaces.msg import StringArray, UpdateState, UpdateRatio, ImageWindowRatios
from nepi_interfaces.srv import ImageCapabilitiesQuery, ImageCapabilitiesQueryRequest, ImageCapabilitiesQueryResponse

from nepi_interfaces.msg import RangeWindow

from sensor_msgs.msg import PointCloud2

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.connect_mgr_if_navpose import ConnectMgrNavPoseIF



##################################################


API_LIB_FOLDER = "/opt/nepi/ros/lib/nepi_api"
AIFS_SHARE_PATH = "/opt/nepi/ros/share/nepi_aifs"
USER_CFG_FOLDER = '/mnt/nepi_storage/user_cfg/ros'

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
                    success = nepi_utils.nepi_utils.write_yaml_2_dict(file_path)(data, file_path)
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
    'depth_m': 0.0
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

    caps_report = NavPoseCapabilitiesQueryResponse()

    data_product = 'navpose'

    def __init__(self, namespace = None,
                data_source_description = 'navpose',
                data_ref_description = 'sensor_center',
                pub_location = False, pub_heading = False,
                pub_orientation = False, pub_position = False,
                pub_altitude = False, pub_depth = False,
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

        # Create Capabilities Report

        self.caps_report.has_location_pub = self.pub_location
        self.caps_report.has_heading_pub = self.pub_heading
        self.caps_report.has_position_pub = self.pub_position
        self.caps_report.has_orientation_pub = self.pub_orientation
        self.caps_report.has_depth_pub = self.pub_depth



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
        self.CFGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Services Config Dict ####################     
        SRVS_DICT = {
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

        #self.node_if.wait_for_ready()
        nepi_sdk.wait()

        ##############################
        # Update vals from param server
        self.init()
        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
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
        return nepi_nav.BLANK_NAVPOSE_DICT

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        has_subs = copy.deepcopy(self.has_subs)
        self.msg_if.pub_debug("Returning: " + self.namespace + " " "has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        return has_subs


    # Update System Status
    def publish_navpose(self,navpose_dict, 
                        timestamp = None, 
                        frame_3d_transform = None,
                        device_mount_description = 'fixed'):      
        np_dict = nepi_nav.BLANK_NAVPOSE_DICT
        if navpose_dict is None:
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

            if frame_3d_transform is not None:
                np_dict = nepi_nav.transform_navpose_dict(np_dict,frame_3d_transform)
                        
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

        return np_dict


    def unsubsribe(self):
        self.ready = False
        if self.node_if is not None:
            self.node_if.unregister_class()
        time.sleep(1)
        self.namespace = None
        self.status_msg = NavPoseStatus()


    def publish_status(self):
        if self.node_if is not None:
            avg_rate = 0
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate
           
            self.node_if.publish_pub('status_pub', self.status_msg)



    def init(self):
        if self.node_if is not None:
            pass
        self.publish_status()

    def reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
            self.init()
            


    def factory_reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Factory reseting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
            self.init()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init()

    def _resetCb(self):
        self.reset()

    def _factoryResetCb(self):
        self.factory_reset()


    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if has_subs == False:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        #self.msg_if.pub_warn("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)


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
        self.CFGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Services Config Dict ####################     
        SRVS_DICT = {
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

        #self.node_if.wait_for_ready()
        nepi_sdk.wait()

        self.track_length = self.node_if.get_param('track_length')
        self.track_sec = self.node_if.get_param('track_sec')
        ##############################
        # Update vals from param server
        self.init()
        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
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
        return nepi_nav.BLANK_NAVPOSE_DICT

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        has_subs = copy.deepcopy(self.has_subs)
        self.msg_if.pub_debug("Returning: " + self.namespace + " " "has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        return has_subs


    # Update System Status
    def publish_navpose_track(self,navpose_dict, 
                        timestamp = None, 
                        frame_3d_transform = None,
                        device_mount_description = 'fixed'):      
        np_dict = nepi_nav.BLANK_NAVPOSE_DICT
        if navpose_dict is None:
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

            if frame_3d_transform is not None:
                np_dict = nepi_nav.transform_navpose_dict(np_dict,frame_3d_transform)
                        
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
        if self.node_if is not None:
            avg_rate = 0
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate
           
            self.status_msg.track_length = self.track_length
            self.status_msg.track_sec = self.track_sec
            self.status_msg.track_next_sec = self.get_next_track_sec()
            self.status_msg.track_list = self.track_msg_list
            self.node_if.publish_pub('status_pub', self.status_msg)



    def init(self):
        if self.node_if is not None:
            self.track_length = self.node_if.get_param('track_length')
            self.track_sec = self.node_if.get_param('track_sec')
        
    def reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
            self.init()
            self.publish_status()


    def factory_reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Factory reseting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
            self.init()
            self.publish_status()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init()

    def _resetCb(self):
        self.reset()

    def _factoryResetCb(self):
        self.factory_reset()


    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if has_subs == False:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        #self.msg_if.pub_warn("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)


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
# ImageIF


def get_image_data_product(data_product):
    if data_product is None:
        data_product = 'image'
    elif data_product.find('image') == -1:
        data_product = data_product + '_image'
    return data_product


SUPPORTED_DATA_PRODUCTS = ['image','color_image','bw_image',
                            'intensity_map','depth_map','pointcloud']
ENCODING_OPTIONS = ["mono8",'rgb8','bgr8','32FC1','passthrough']


EXAMPLE_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate = False,
        has_tilt = False
    )

EXAMPLE_ENHANCEMENTS_DICT = dict(
    low_light = {
        'enabled': False,
        'ratio': 0.0
    }
)

EXAMPLE_CONTROLS_DICT = dict( 
    resolution_ratio = 1.0,
    auto_adjust_enabled = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    start_range_ratio = 0.0,
    stop_range_ratio = 1.0,
    zoom_ratio = 0.5, 
    pan_left_right_ratio = 0.5,
    pan_up_down_ratio = 0.5,
    window_ratios = [0.0,1.0,0.0,1.0],
    rotate_ratio = 0.5,
    tilt_ratio = 0.5       
    )



class ImageIF:

    DEFUALT_IMG_WIDTH_PX = 700
    DEFUALT_IMG_HEIGHT_PX = 400

    DEFAULT_IMG_WIDTH_DEG = 100
    DEFUALT_IMG_HEIGHT_DEG = 70


    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = False,
        has_auto_adjust = False,
        has_contrast = False,
        has_brightness = False,
        has_threshold = False,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate = False,
        has_tilt = False
        )

    DEFAULT_ENHANCEMENTS_DICT = dict()

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
        zoom_ratio = 0.5, 
        pan_left_right_ratio = 0.5,
        pan_up_down_ratio = 0.5,
        window_ratios = [0.0,1.0,0.0,1.0],
        rotate_ratio = 0.5,
        tilt_ratio = 0.5
        )

    ready = False
    namespace = '~'

    node_if = None

    status_msg = ImageStatus()

    last_width = DEFUALT_IMG_WIDTH_PX
    last_height = DEFUALT_IMG_HEIGHT_PX


    blank_img = nepi_img.create_cv2_blank_img(last_width, last_height, color = (0, 0, 0) )

    last_pub_time = None

    nav_mgr_if = None
    nav_mgr_ready = False

    has_subs = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    caps_dict = DEFAULT_CAPS_DICT
    controls_dict = DEFAULT_CONTROLS_DICT
    init_controls_dict = controls_dict



    caps_report = ImageCapabilitiesQueryResponse()

    overlays_dict = dict(
            overlay_img_name = False,
            overlay_date_time = False,
            overlay_nav = False,
            overlay_pose = False, 
            init_overlay_list = [],
            add_overlay_list = []
    )

    enhance_dict = dict()
    has_enhance = False
    enhance_options = []
    sel_enhances = []

    get_navpose_function = None
    has_navpose = False
    navpose_if = None


    data_source_description = 'imaging_sensor'
    data_ref_description = 'sensor'

    data_product = 'image'

    def __init__(self, namespace , 
                data_product_name,
                data_source_description,
                data_ref_description,
                width_deg,
                hieight_deg,
                caps_dict,
                controls_dict, 
                enhance_dict,
                params_dict,
                services_dict,
                pubs_dict,
                subs_dict,
                init_overlay_list,
                get_navpose_function,
                log_name,
                log_name_list,
                node_if,
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
          

        # Setup enhance dict
        if enhance_dict is not None:
            self.enhance_dict = enhance_dict
            self.enhance_options = list(self.enhance_dict.keys())
            if len(self.enhance_options) > 0:
                self.has_enhance = True



        # Create and update capabilities dictionary
        if caps_dict is not None:
            for cap in self.caps_dict.keys():
                if caps_dict.get(cap) != None:
                    self.caps_dict[cap] = caps_dict[cap]

        self.get_navpose_function = get_navpose_function
        if get_navpose_function is not None:
            self.has_navpose = True


        self.caps_report.has_resolution = self.caps_dict['has_resolution']
        self.caps_report.has_contrast = self.caps_dict['has_contrast']
        self.caps_report.has_brightness = self.caps_dict['has_brightness']
        self.caps_report.has_threshold = self.caps_dict['has_threshold']
        self.caps_report.has_range = self.caps_dict['has_range']
        self.caps_report.has_auto_adjust = self.caps_dict['has_auto_adjust']
        self.caps_report.has_zoom = self.caps_dict['has_zoom']
        self.caps_report.has_pan = self.caps_dict['has_pan']
        self.caps_report.has_window = self.caps_dict['has_window']
        self.caps_report.has_rotate = self.caps_dict['has_rotate']
        self.caps_report.has_tilt = self.caps_dict['has_tilt']

        self.caps_report.has_enhances = self.has_enhance
        self.caps_report.enhance_options = self.enhance_options

        dm_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'depth_map')
        dm_topic = nepi_sdk.find_topic(dm_ns)
        has_dm = dm_topic != ""
        pc_ns = nepi_sdk.create_namespace(os.path.dirname(self.namespace),'pointcloud')
        pc_topic = nepi_sdk.find_topic(pc_ns)
        has_pc = pc_topic != ""

        self.caps_report.has_depth_map = has_dm
        self.caps_report.depth_map_topic = dm_topic
        self.caps_report.has_pointcloud = has_pc
        self.caps_report.pointcloud_topic = pc_topic



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
        self.status_msg.get_latency_time = 0
        self.status_msg.pub_latency_time = 0
        self.status_msg.process_time = 0

        ####################
        # Connect to navpose_mgr for data

        ##############################
        ## Connect NEPI NavPose Manager
        self.nav_mgr_if = ConnectMgrNavPoseIF()
        ready = self.nav_mgr_if.wait_for_ready()



        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        PARAMS_DICT = {

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
            'start_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["start_range_ratio"]
            },
            'stop_range_ratio': {
                'namespace': self.namespace,
                'factory_val': self.controls_dict["stop_range_ratio"]
            },
            'enhance_dict': {
                'namespace': self.namespace,
                'factory_val': self.enhance_dict
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
            },
        }

        if params_dict is not None:
            self.PARAMS_DICT = params_dict | PARAMS_DICT
        else:
            self.PARAMS_DICT = PARAMS_DICT

        # Services Config Dict ####################     
        SRVS_DICT = {
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
            self.SRVS_DICT = services_dict | SRVS_DICT
        else:
            self.SRVS_DICT = SRVS_DICT
        

        # Pubs Config Dict ####################
        PUBS_DICT = {
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
            }
        }


        if pubs_dict is not None:
            self.PUBS_DICT = pubs_dict | PUBS_DICT
        else:
            self.PUBS_DICT = PUBS_DICT        

        # Subs Config Dict ####################
        SUBS_DICT = {
            'reset': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetCb, 
                'callback_args': ()
            },
            'reset_controls': {
                'namespace': self.namespace,
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetControlsCb, 
                'callback_args': ()
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
            },
            'reset_overlays': {
                'namespace': self.namespace,
                'topic': 'reset_overlays',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetOverlaysCb, 
                'callback_args': ()
            }
        }
        # Create subs if required
        if caps_dict['has_resolution'] == True:
            SUBS_DICT['set_resolution'] = {
                'namespace': self.namespace,
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setResolutionRatioCb, 
                'callback_args': ()
            }

        if caps_dict['has_auto_adjust'] == True:
            SUBS_DICT['set_auto_adjust'] = {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setAutoAdjustCb, 
                'callback_args': ()
            }
            SUBS_DICT['set_auto_adjust_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_auto_adjust_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setAutoAdjustRatioCb, 
                'callback_args': ()
            }
        if caps_dict['has_brightness'] == True:
            SUBS_DICT['set_brightness'] = {
                'namespace': self.namespace,
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setBrightnessCb, 
                'callback_args': ()
            }
        if caps_dict['has_contrast'] == True:
            SUBS_DICT['set_contrast'] = {
                'namespace': self.namespace,
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setContrastCb, 
                'callback_args': ()
            }
        if caps_dict['has_threshold'] == True:
            SUBS_DICT['set_thresholding'] = {
                'namespace': self.namespace,
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setThresholdingCb, 
                'callback_args': ()
            }
        if caps_dict['has_range'] == True:
            SUBS_DICT['set_range'] = {
                'namespace': self.namespace,
                'topic': 'set_range_ratios',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self._setRangeCb, 
                'callback_args': ()
            }
        if caps_dict['has_zoom'] == True:
            SUBS_DICT['set_zoom'] = {
                'namespace': self.namespace,
                'topic': 'set_zoom_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setZoomCb, 
                'callback_args': ()
            }
        if caps_dict['has_pan'] == True:
            SUBS_DICT['set_pan_left_right'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_left_right_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setPanLrCb, 
                'callback_args': ()
            }
            SUBS_DICT['set_pan_up_down'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_up_down_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setPanUdCb, 
                'callback_args': ()
            }

        if caps_dict['has_window'] == True:
            SUBS_DICT['set_window'] = {
                'namespace': self.namespace,
                'topic': 'set_window_ratios',
                'msg': ImageWindowRatios,
                'qsize': 1,
                'callback': self._setWindowCb, 
                'callback_args': ()
            }
        if caps_dict['has_rotate'] == True:
            SUBS_DICT['set_rotate'] = {
                'namespace': self.namespace,
                'topic': 'set_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setRotateCb, 
                'callback_args': ()
            }

        if caps_dict['has_tilt'] == True:
            SUBS_DICT['set_tilt'] = {
                'namespace': self.namespace,
                'topic': 'set_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setTiltCb, 
                'callback_args': ()
            }

        if self.has_enhance == True:
            SUBS_DICT['set_enhance_enable'] = {
                'namespace': self.namespace,
                'topic': 'set_tilt_ratio',
                'msg': UpdateState,
                'qsize': 1,
                'callback': self._setEnhanceEnableCb, 
                'callback_args': ()
            }
            SUBS_DICT['set_enhance_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_tilt_ratio',
                'msg': UpdateRatio,
                'qsize': 1,
                'callback': self._setEnhanceRatioCb, 
                'callback_args': ()
            }


        if subs_dict is not None:
            self.SUBS_DICT = subs_dict | SUBS_DICT
        else:
            self.SUBS_DICT = SUBS_DICT     


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

        #self.node_if.wait_for_ready()
        nepi_sdk.wait()

        # Setup navpose data IF if needed

       # Setup navpose data IF
        np_namespace = self.namespace
        self.navpose_if = NavPoseIF(namespace = np_namespace,
                        data_source_description = self.data_source_description,
                        data_ref_description = self.data_ref_description,
                        log_name = 'navpose',
                        log_name_list = [],
                        msg_if = self.msg_if
                        )
           


        self.publish_status(do_updates=True)

        #############################
        # Set variables to param values
        self.init()
    


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
        elif self.nav_mgr_ready == True:
            navpose_dict = self.nav_mgr_if.get_navpose_dict()
        else:
            navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
        return navpose_dict



    def publish_cv2_img(self,cv2_img, 
                        encoding = "bgr8", 
                        timestamp = None, 
                        frame_3d = 'sensor_frame', 
                        width_deg = DEFAULT_IMG_WIDTH_DEG,
                        height_deg = DEFUALT_IMG_HEIGHT_DEG,
                        add_overlay_list = [],
                        do_subscriber_check = True,
                        device_mount_description = 'fixed'
                        ):
        self.msg_if.pub_debug("Got Image to Publish", log_name_list = self.log_name_list, throttle_s = 5.0)
        success = False
        if cv2_img is None:
            self.msg_if.pub_info("Can't publish None image", log_name_list = self.log_name_list)
            return cv2_img

        # Process 

        self.data_ref_description = data_ref_description
        self.data_description = get_data_description(self.data_source_description,data_ref_description)
        self.status_msg.data_description = self.data_description

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
 
        [height,width] = cv2_img.shape[0:2]
        last_width = self.status_msg.width_px
        last_height = self.status_msg.height_px
        self.status_msg.width_px = width
        self.status_msg.height_px = height
        res_str = str(width) + ":" + str(height)
        self.status_msg.resolution_current = res_str

        self.status_msg.width_deg = width_deg
        self.status_msg.height_deg = height_deg

        self.msg_if.pub_debug("Got Image size: " + str([height,width]), log_name_list = self.log_name_list, throttle_s = 5.0)

        cv2_img = self._process_image(cv2_img)

        navpose_dict = self.get_navpose_dict()


        # Apply Overlays
        overlay_list = []
        if self.status_msg.overlay_img_name == True:
            overlay = nepi_img.getImgShortName(self.img_namespace)
            overlay_list.append(overlay)
        
        if self.status_msg.overlay_date_time == True:
            date_time = nepi_utils.get_datetime_str_from_timestamp(timestamp)
            overlay_list.append(overlay)


        if self.status_msg.overlay_nav == True or self.status_msg.overlay_pose == True:
                if navpose_dict is not None:
                    if self.status_msg.overlay_nav == True and navpose_dict is not None:
                        overlay = 'Lat: ' +  str(round(navpose_dict['latitude'],6)) + 'Long: ' +  str(round(navpose_dict['longitude'],6)) + 'Head: ' +  str(round(navpose_dict['heading_deg'],2))
                        overlay_list.append(overlay)

                    if self.status_msg.overlay_pose == True and navpose_dict is not None:
                        overlay = 'Roll: ' +  str(round(navpose_dict['roll_deg'],2)) + 'Pitch: ' +  str(round(navpose_dict['pitch_deg'],2)) + 'Yaw: ' +  str(round(navpose_dict['yaw_deg'],2))
                        overlay_list.append(overlay)

        overlay_list = overlay_list + self.overlays_dict['init_overlay_list'] + self.overlays_dict['add_overlay_list'] + add_overlay_list

        cv2_img = nepi_img.overlay_text_list(cv2_img, text_list = overlay_list, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0), apply_shadow = True)


        #Convert to ros Image message
        ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
        sec = nepi_sdk.sec_from_timestamp(timestamp)
        ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = frame_3d)
        self.msg_if.pub_debug("Publishing Image with header: " + str(ros_img.header), log_name_list = self.log_name_list, throttle_s = 5.0)
        self.node_if.publish_pub('data_pub', ros_img)

        if navpose_dict is not None and self.navpose_if is not None:
            self.navpose_if.publish_navpose(navpose_dict, timestamp = timestamp)
        
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
        return cv2_img

    def publish_msg_img(self, msg_text, timestamp = None, frame_3d = 'nepi_base'):
        cv2_img = nepi_img.overlay_text_autoscale(self.blank_img, text)

        if timestamp == None:
            timestamp = nepi_utils.get_time()

        #Convert to ros Image message
        ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
        sec = nepi_sdk.sec_from_timestamp(timestamp)
        ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = frame_3d)
        self.node_if.publish_pub('data_pub', ros_img)


    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_sdk.wait()
        self.namespace = '~'
        self.status_msg = None


    def set_resolution_ratio(self, ratio):
        if (ratio < 0.2):
            ratio = 0.2
        if (ratio > 1.0):
            ratio = 1.0
        self.msg_if.pub_error("Resolution value out of bounds, using: " + str(ratio), log_name_list = self.log_name_list)
        self.controls_dict['resolution_ratio'] = ratio
        self.status_msg.resolution_ratio = ratio
        self.publishStatus(do_updates=False) # Updated inline here
        self.node_if.set_param('resolution_ratio', ratio)


    def set_auto_adjust_enable(self, enabled):
        if enabled:
            self.msg_if.pub_info("Enabling Auto Adjust", log_name_list = self.log_name_list)
        else:
            self.msg_if.pub_info("Disabling Auto Adjust", log_name_list = self.log_name_list)
        self.controls_dict['auto_adjust_enabled'] = enabled
        self.status_msg.auto_adjust_enabled = enabled
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('auto_adjust_enabled', enabled)


    def set_auto_adjust_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['auto_adjust_ratio'] = ratio
        self.status_msg.auto_adjust_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('auto_adjust_ratio', ratio)

    def set_brightness_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['brightness_ratio'] = enabled
        self.status_msg.brightness_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('brightness_ratio', ratio)


    def set_contrast_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['contrast_ratio'] = enabled
        self.status_msg.contrast_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('contrast_ratio', ratio)
        


    def set_threshold_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['threshold_ratio'] = enabled
        self.status_msg.threshold_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('threshold_ratio', ratio)

    def set_range_ratios(self, start_ratio, stop_ratio):
        if (start_ratio < 0 or stop_ratio > 1 or stop_ratio < start_ratio):
            self.msg_if.pub_error("Range values out of bounds", log_name_list = self.log_name_list)
            self.publishStatus(do_updates=False) # No change
            return

        self.controls_dict['start_range_ratio'] = start_ratio
        self.status_msg.range_ratios.start_range = start_ratio
        self.controls_dict['stop_range_ratio'] = stop_ratio
        self.status_msg.range_ratios.stop_range = stop_ratio

        self.publishStatus(do_updates=False) # Updated inline here 

        self.node_if.set_param('start_range_ratio', start_ratio)
        self.node_if.set_param('stop_range_ratio', stop_ratio)
      

    def set_zoom_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['zoom_ratio'] = enabled
        self.status_msg.zoom_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here


    def set_pan_left_right_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['pan_left_right_ratio'] = enabled
        self.status_msg.pan_left_right_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here


    def set_pan_up_down_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['pan_up_down_ratio'] = enabled
        self.status_msg.pan_up_down_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here

    def set_window_ratios(self, x_min, x_max, y_min, y_max):
        if x_min >= 0 and x_max <= 1.0 and x_max > x_min \
            and y_min >= 0 and y_max <= 1.0 and y_max > y_min:
            self.controls_dict['window'] = [x_min,x_max,y_min,y_max]
            self.publishStatus(do_updates=False) # Updated inline here  

    def set_rotate_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['rotate_ratio'] = enabled
        self.status_msg.rotate_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here

    def set_tilt_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        self.controls_dict['tilt_ratio'] = enabled
        self.status_msg.tilt_ratio = ratio
        self.publish_status(do_updates=False) # Updated inline here


    def set_enhance_enable(self,name, enabled):
        if name in self.enhance_dict.keys():
            was_enabled = self.enhance_dict[name]['enabled']
            if was_enabled != enabled:
                if enable == True:
                    self.msg_if.pub_info("Enabling Enhancment: " + name, log_name_list = self.log_name_list)
                else:
                    self.msg_if.pub_info("Disabling Enhancment: " + name, log_name_list = self.log_name_list)
                self.enhance_dict[name]['enabled'] = enabled
                self.publish_status(do_updates=False) # Updated inline here
                self.node_if.set_param('enhance_dict', self.enhance_dict)


    def set_enhance_ratio(self, ratio):
        if ratio < 0:
            ratio = 0
        if ratio > 1.0:
            ratio = 1.0
        if name in self.enhance_dict.keys():
            self.msg_if.pub_info("Setting Enhancment Ratio: " + name + " : " + str(ratio), log_name_list = self.log_name_list)
            self.enhance_dict[name]['ratio'] = ratio
            self.publish_status(do_updates=False) # Updated inline here
            self.node_if.set_param('enhance_dict', self.enhance_dict)




    def set_overlay_image_name(self,enabled):
        self.overlays_dict['overlay_img_name'] = enabled
        self.status_msg.overlay_img_name = enabled
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('overlay_img_name', enabled)

    def set_overlay_date_time(self,enabled):
        self.overlays_dict['overlay_date_time'] = enabled
        self.status_msg.overlay_date_time =  enabled
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('overlay_date_time', enabled)

    def set_overlay_nav(self,enabled):
        self.overlays_dict['overlay_nav'] = enabled
        self.status_msg.overlay_nav = enabled
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('overlay_nav', enabled)

    def set_overlay_pose(self,enabled):
        self.overlays_dict['overlay_pose'] = enabled
        self.status_msg.overlay_pose = enabled
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('overlay_pose', enabled)


    def set_overlay_list(self,overlay_list):
        self.overlays_dict['add_overlay_list'] = overlay_list
        self.status_msg.add_overlay_list = overlay_list  
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('add_overlay_list', overlay_list)


    def set_overlay_text(self,overlay_text):
        overlay_list = self.overlays_dict['add_overlay_list']
        overlay_list.append(text)
        self.overlays_dict['add_overlay_list'] = overlay_list
        self.status_msg.add_overlay_list = overlay_list
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('add_overlay_list', overlay_list)


    def clear_overlay_list(self):
        self.overlays_dict['add_overlay_list'] = []
        self.status_msg.add_overlay_list = []
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('add_overlay_list', [])


    def reset_controls(self):
        
        # First reset controls to init dict to capture non param managed settings
        self.controls_dict = self.init_controls_dict

        self.node_if.factory_reset_param('resolution_ratio')
        self.node_if.factory_reset_param('auto_adjust_enabled')
        self.node_if.factory_reset_param('auto_adjust_ratio')
        self.node_if.factory_reset_param('brightness_ratio')
        self.node_if.factory_reset_param('contrast_ratio')
        self.node_if.factory_reset_param('threshold_ratio')
        self.node_if.factory_reset_param('start_range_ratio')
        self.node_if.factory_reset_param('stop_range_ratio')
        self.node_if.factory_reset_param('enhance_dict')


        self.controls_dict['resolution_ratio'] = self.node_if.get_param('resolution_ratio')
        self.controls_dict['auto_adjust_enabled'] = self.node_if.get_param('auto_adjust_enabled')
        self.controls_dict['auto_adjust_ratio'] = self.node_if.get_param('auto_adjust_ratio')
        self.controls_dict['brightness_ratio'] = self.node_if.get_param('brightness_ratio')
        self.controls_dict['contrast_ratio'] = self.node_if.get_param('contrast_ratio')
        self.controls_dict['threshold_ratio'] = self.node_if.get_param('threshold_ratio')

        self.controls_dict['start_range_ratio'] = self.node_if.get_param('start_range_ratio')
        self.controls_dict['stop_range_ratio'] = self.node_if.get_param('stop_range_ratio')

        self.enhance_dict = self.node_if.get_param('enhance_dict')

        self.publish_status(do_updates=False)

    def reset_overlays(self):
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
        
        self.publish_status(do_updates=False)


    def publish_status(self, do_updates = True):
        if self.node_if is not None:
            self.status_msg.resolution_ratio = self.controls_dict['resolution_ratio']
            self.status_msg.auto_adjust_enabled = self.controls_dict['auto_adjust_enabled']
            self.status_msg.auto_adjust_ratio = self.controls_dict['auto_adjust_ratio']
            self.status_msg.contrast_ratio = self.controls_dict['contrast_ratio']
            self.status_msg.brightness_ratio = self.controls_dict['brightness_ratio']
            self.status_msg.threshold_ratio = self.controls_dict['threshold_ratio']
            self.status_msg.range_ratios.start_range = self.controls_dict['start_range_ratio']
            self.status_msg.range_ratios.stop_range = self.controls_dict['stop_range_ratio']
            self.status_msg.zoom_ratio = self.controls_dict['zoom_ratio']
            self.status_msg.pan_left_right_ratio = self.controls_dict['pan_left_right_ratio']
            self.status_msg.pan_up_down_ratio = self.controls_dict['pan_up_down_ratio']
            self.status_msg.window_ratios.x_min = self.controls_dict['window_ratios'][0]
            self.status_msg.window_ratios.x_max = self.controls_dict['window_ratios'][1]
            self.status_msg.window_ratios.y_min = self.controls_dict['window_ratios'][2]
            self.status_msg.window_ratios.y_max = self.controls_dict['window_ratios'][3]
            self.status_msg.rotate_ratio = self.controls_dict['rotate_ratio']
            self.status_msg.tilt_ratio = self.controls_dict['tilt_ratio']

            enhance_options = []
            enhance_states = []
            enhance_ratios = []
            for name in self.enhance_dict.keys():
                e_dict = self.enhance_dict[name]
                enhance_options.append(name)
                enhance_states.append(e_dict['enabled'])
                enhance_ratios.append(e_dict['ratio'])
            self.status_msg.enhance_options = enhance_options
            self.status_msg.enhance_states = enhance_states
            self.status_msg.enhance_ratios = enhance_ratios

            self.status_msg.overlay_img_name = self.overlays_dict['overlay_img_name']
            self.status_msg.overlay_date_time =  self.overlays_dict['overlay_date_time']
            self.status_msg.overlay_nav = self.overlays_dict['overlay_nav']
            self.status_msg.overlay_pose = self.overlays_dict['overlay_pose']  
            self.status_msg.base_overlay_list = self.overlays_dict['init_overlay_list']
            self.status_msg.add_overlay_list = self.overlays_dict['add_overlay_list']

            avg_rate = 0
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate

            self.node_if.publish_pub('status_pub',self.status_msg)


    def init(self):
        if self.node_if is not None:
            self.controls_dict['resolution_ratio'] = self.node_if.get_param('resolution_ratio')
            self.controls_dict['auto_adjust_enabled'] = self.node_if.get_param('auto_adjust_enabled')
            self.controls_dict['auto_adjust_ratio'] = self.node_if.get_param('auto_adjust_ratio')
            self.controls_dict['brightness_ratio'] = self.node_if.get_param('brightness_ratio')
            self.controls_dict['contrast_ratio'] = self.node_if.get_param('contrast_ratio')
            self.controls_dict['threshold_ratio'] = self.node_if.get_param('threshold_ratio')

            self.controls_dict['start_range_ratio'] = self.node_if.get_param('start_range_ratio')
            self.controls_dict['stop_range_ratio'] = self.node_if.get_param('stop_range_ratio')


            self.enhance_dict = self.node_if.get_param('enhance_dict')

            self.overlays_dict['overlay_img_name'] = self.node_if.get_param('overlay_img_name')
            self.overlays_dict['overlay_date_time'] = self.node_if.get_param('overlay_date_time')
            self.overlays_dict['overlay_nav'] = self.node_if.get_param('overlay_nav')
            self.overlays_dict['overlay_pose'] = self.node_if.get_param('overlay_pose')
            self.overlays_dict['add_overlay_list'] = self.node_if.get_param('add_overlay_list')
        
    def reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
            self.init()
            self.publish_status()


    def factory_reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Factory reseting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
            self.init()
            self.publish_status()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init()

    def _resetCb(self):
        self.reset()

    def _factoryResetCb(self):
        self.factory_reset()


    def _provide_capabilities(self, _):
        return self.caps_report

    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if has_subs == False:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        self.msg_if.pub_debug("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.publish_status()



    def _setResolutionRatioCb(self, msg):
        self.msg_if.pub_info("Recived Resolution update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_resolution_ratio(ratio)



    def _setAutoAdjustCb(self, msg):
        self.msg_if.pub_info("Recived Auto Adjust Enable update message: " + str(msg), log_name_list = self.log_name_list)
        enabled = msg.data
        self.set_auto_adjust(enabled)

    def _setAutoAdjustRatioCb(self, msg):
        self.msg_if.pub_info("Recived Auto Adjust Ratio update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_auto_adjust_ratio(ratio)

    def _setBrightnessCb(self, msg):
        self.msg_if.pub_info("Recived Brightness update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_brightness_ratio(ratio)


    def _setContrastCb(self, msg):
        self.msg_if.pub_info("Recived Contrast update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_contrast_ratio(ratio)
        


    def _setThresholdingCb(self, msg):
        self.msg_if.pub_info("Received Threshold update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_threshold_ratio(ratio)

    def _setRangeCb(self, msg):
        self.msg_if.pub_info("Recived Range update message: " + str(msg), log_name_list = self.log_name_list)
        start_ratio = msg.start_range
        stop_ratio = msg.stop_range
        self.set_range_ratios(start_ratio,stop_ratio)
      

    def _setZoomCb(self, msg):
        self.msg_if.pub_info("Recived Zoom update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_zoom_ratio(ratio)

    def _setPanLrCb(self, msg):
        self.msg_if.pub_info("Recived Pan Left Right update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_pan_left_right_ratio(ratio)


    def _setPanUdCb(self, msg):
        self.msg_if.pub_info("Recived Pan Up Down update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_pan_up_down_ratio(ratio)

    def _setWindowCb(self, msg):
        self.msg_if.pub_info("Recived window update message: " + str(msg), log_name_list = self.log_name_list)
        x_min = msg.x_min
        x_max = msg.x_max
        y_min = msg.y_min
        y_max = msg.y_max
        self.set_window_ratios(x_min,x_max,y_min,y_max)


    def _setRotateCb(self, msg):
        self.msg_if.pub_info("Recived Rotate update message: " + str(msg), log_name_list = self.log_name_list)
        ratio = msg.data
        self.set_rotate_ratio(ratio) 

    def _setTiltCb(self, msg):
        ratio = msg.data
        self.set_tilt_ratio(ratio) 

    def _setEnhanceEnableCb(self, msg):
        self.msg_if.pub_info("Recived Enable Enhacement message: " + str(msg), log_name_list = self.log_name_list)
        name = msg.name
        enabled = msg.active_state
        self.set_enhance_enable(name,enabled) 

    def _setEnhanceRatioCb(self, msg):
        self.msg_if.pub_info("Recived Ehnacement Ratio update message: " + str(msg), log_name_list = self.log_name_list)
        name = msg.name
        ratio = msg.ratio
        self.set_enhance_ratio(name,ratio) 





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
        self.reset_controls()
        self.reset_overlays()

    def _resetControlsCb(self,msg):
        self.reset_controls()

    def _resetOverlaysCb(self,msg):
        self.reset_overlays()

    def _resetEnhancessCb(self,msg):
        self.reset_enhances()

    def _process_image(self, cv2_img):
        return cv2_img
        '''
        # Apply Controls
        enabled = self.status_msg.controls_enabled
        auto = self.status_msg.auto_adjust_enabled
        brightness = self.status_msg.contrast_ratio
        contrast = self.status_msg.brightness_ratio
        threshold = self.status_msg.threshold_ratio
        if enabled == True: 
            #if res_ratio < 0.9:
            #    [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)
            if auto is False:
                cv2_img = nepi_img.adjust_brightness(cv2_img, brightness)
                cv2_img = nepi_img.adjust_contrast(cv2_img, contrast)
                cv2_img = nepi_img.adjust_sharpness(cv2_img, threshold)
            else:
                cv2_img = nepi_img.adjust_auto(cv2_img,0.3)
        return cv2_img
        '''

##################################################
# ColorImageIF

class ColorImageIF(ImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = True,
        has_auto_adjust = True,
        has_contrast = True,
        has_brightness = True,
        has_threshold = True,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate = False,
        has_tilt = False
        )

    DEFAULT_ENHANCEMENTS_DICT = dict(
        low_light = {
            'enabled': False,
            'ratio': 0.0
        }
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
        zoom_ratio = 0.5, 
        pan_left_right_ratio = 0.5,
        pan_up_down_ratio = 0.5,
        window_ratios = [0.0,1.0,0.0,1.0],
        rotate_ratio = 0.5,
        tilt_ratio = 0.5
        )

    data_product = 'color_image'


    
    def __init__(self, namespace = None , 
                data_product_name = 'color_image',
                data_source_description = 'imaging_sensor',
                data_ref_description = 'sensor',
                init_overlay_list = [],
                get_navpose_function = None,
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
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_ENHANCEMENTS_DICT, 
                None,
                None,
                None,
                None,
                init_overlay_list,
                get_navpose_function,
                log_name,
                log_name_list,
                node_if,
                msg_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    ###############################
    # Class Private Methods
    ###############################

    def _process_image(self, cv2_img):
        # Apply Resolution Controls
        res_ratio = self.controls_dict['resolution_ratio']
        if res_ratio < 0.9:
            [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)

        # Apply Low Light Enhancment
        if self.enhance_dict['low_light']['enabled'] == True:
            ratio = self.enhance_dict['low_light']['ratio']
            if ratio > 0.05:
                pass
                #cv2_img = nepi_img.enhance_low_light_dual_illumination(cv2_img)

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
            cv2_img = nepi_img.adjust_auto(cv2_img,0.3)
        return cv2_img


##################################################
# DepthMapImageIF

class DepthMapImageIF(ImageIF):

    #Default Control Values 
    DEFAULT_CAPS_DICT = dict( 
        has_resolution = True,
        has_auto_adjust = True,
        has_contrast = True,
        has_brightness = True,
        has_threshold = True,
        has_range = False,
        has_zoom = False,
        has_pan = False,
        has_window = False,
        has_rotate = False,
        has_tilt = False
        )

    DEFAULT_ENHANCEMENTS_DICT = dict(
        low_light = {
            'enabled': False,
            'ratio': 0.0
        }
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
        zoom_ratio = 0.5, 
        pan_left_right_ratio = 0.5,
        pan_up_down_ratio = 0.5,
        window_ratios = [0.0,1.0,0.0,1.0],
        rotate_ratio = 0.5,
        tilt_ratio = 0.5
        )

    data_product = 'depth_map'

    def __init__(self, namespace = None , 
                data_product_name = 'depth_map_image',
                data_source_description = 'depth_map_sensor',
                data_ref_description = 'sensor',
                init_overlay_list = [],
                get_navpose_function = None,
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
                self.DEFAULT_CAPS_DICT,
                self.DEFAULT_CONTROLS_DICT,
                self.DEFAULT_ENHANCEMENTS_DICT, 
                None,
                None,
                None,
                None,
                init_overlay_list,
                log_name,
                log_name_list,
                node_if,
                msg_if
                )

        ###############################
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        ###############################






    ###############################
    # Class Public Methods
    ###############################


    ###############################
    # Class Private Methods
    ###############################

    def _process_image(self, cv2_img):
        # Apply Resolution Controls
        res_ratio = self.controls_dict['resolution_ratio']
        if res_ratio < 0.9:
            [cv2_img,new_res] = nepi_img.adjust_resolution_ratio(cv2_img, res_ratio)

        # Apply Low Light Enhancment
        if self.enhance_dict['low_light']['enabled'] == True:
            ratio = self.enhance_dict['low_light']['ratio']
            if ratio > 0.05:
                pass
                #cv2_img = nepi_img.enhance_low_light_dual_illumination(cv2_img)

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
            cv2_img = nepi_img.adjust_auto(cv2_img,0.3)
        return cv2_img




##################################################
# DepthMapIF

class DepthMapIF:

    DEFUALT_IMG_WIDTH_PX = 700
    DEFUALT_IMG_HEIGHT_PX = 400

    DEFAULT_IMG_WIDTH_DEG = 100
    DEFUALT_IMG_HEIGHT_DEG = 70

    ready = False
    namespace = '~'

    node_if = None

    status_msg = DepthMapStatus()

    last_width = DEFUALT_IMG_WIDTH_PX
    last_height = DEFUALT_IMG_HEIGHT_PX

    blank_img = nepi_img.create_cv2_blank_img(DEFUALT_IMG_WIDTH_PX, DEFUALT_IMG_HEIGHT_PX, color = (0, 0, 0) )

    last_pub_time = None

    nav_mgr_if = None
    nav_mgr_ready = False

    has_subs = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    img_pub_file = 'nepi_depth_map_img_pub_node.py'

    min_range_m = 0.0
    max_range_m = 20.0



    data_source_description = 'depth_map_sensor'
    data_ref_description = 'sensor'

    data_product = 'depth_map'
    data_products_list = [data_product]

    def __init__(self, namespace = None,
                data_product_name = 'depth_map',
                data_source_description = 'depth_map_sensor',
                data_ref_description = 'sensor',
                enable_data_pub = True,
                max_data_pub_rate = 5,
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

        ###############################
        if enable_data_pub == True:
            ## Get folder info
            #self.mgr_sys_srv_if = ConnectMgrSystemServicesIF()
            #success = self.mgr_sys_srv_if.wait_for_services()
            #if success == False:
            #    nepi_sdk.signal_shutdown(self.node_name + ": Failed to get System Status Msg", log_name_list = self.log_name_list)

            #self.api_lib_folder = mgr_sys_srv_if.get_sys_folder_path('api_lib',API_LIB_FOLDER)
            self.msg_if.pub_debug("Using User Config Folder: " + str(self.api_lib_folder), log_name_list = self.log_name_list)

            self.api_lib_folder = API_LIB_FOLDER
            self.msg_if.pub_info("Using SDK Share Folder: " + str(self.api_lib_folder), log_name_list = self.log_name_list)

            # Launch detection img pub node that handles detection image publishing
            pkg_name = 'nepi_api'
            node_file_folder = self.api_lib_folder
            img_pub_file = self.img_pub_file
            img_pub_file_path = os.path.join(node_file_folder,img_pub_file)
        
            if os.path.exists(img_pub_file_path) == False or enable_data_pub == False:
                self.msg_if.pub_warn("Could not find Img Pub Node file at: " + img_pub_file_path, log_name_list = self.log_name_list)
            else: 
                #Try and launch node
                unique_name = nepi_sdk.get_unique_name_from_namespace(self.namespace,self.base_namespace)
                img_pub_node_name = unique_name + "_depth_map_img_pub"
                img_pub_namespace = self.node_namespace + "_depth_map_img_pub"
                self.msg_if.pub_warn("Launching Depth Map Pub Node: " + img_pub_node_name, log_name_list = self.log_name_list)
                self.msg_if.pub_warn("Launching Depth Map Pub on namespace: " + img_pub_namespace, log_name_list = self.log_name_list)

                # Pre Set Img Pub Params
                dm_data_product = os.path.basename(self.namespace)
                img_data_product = dm_data_product + '_image'
                param_ns = nepi_sdk.create_namespace(img_pub_namespace,'data_product')
                nepi_sdk.set_param(param_ns,img_data_product)

                dm_namespace = self.namespace
                param_ns = nepi_sdk.create_namespace(img_pub_namespace,'dm_namespace')
                nepi_sdk.set_param(param_ns,dm_namespace)
                        

                [success, msg, pub_process] = nepi_sdk.launch_node(pkg_name, img_pub_file, img_pub_node_name)

                self.msg_if.pub_warn("Img Pub Node launch return msg: " + msg, log_name_list = self.log_name_list)


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
        self.status_msg.data_pub_enabled = enable_data_pub
        self.status_msg.max_data_pub_rate = max_data_pub_rate



        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'max_img_pub_rate': {
                'namespace': self.node_namespace,
                'factory_val': max_data_pub_rate
            },
            'render_start_range_ratio': {
                'namespace': self.node_namespace,
                'factory_val': 0.0
            },
            'render_stop_range_ratio': {
                'namespace': self.node_namespace,
                'factory_val': 1.0
            },

        }

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
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'render_range_window': {
                'namespace': self.node_namespace,
                'topic': 'render_set_range_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self._setRenderRangeCb, 
                'callback_args': ()
            },
            'reset_render_controls': {
                'namespace': self.node_namespace,
                'topic': 'render_reset_controls',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetRenderControlsCb, 
                'callback_args': ()
            }
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

        #self.node_if.wait_for_ready()
        nepi_sdk.wait()

        self.publish_status(do_updates=True)
        
        self.init()
        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
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


    def publish_cv2_depth_map(self,cv2_img, encoding = '32FC1',
                             min_range_m = None, 
                             max_range_m = None,
                             timestamp = None,
                             frame_3d = 'sensor_frame',
                             width_deg = ImageIF.DEFAULT_IMG_WIDTH_DEG,
                             height_deg = ImageIF.DEFUALT_IMG_HEIGHT_DEG,
                             device_mount_description = 'fixed'):
        self.msg_if.pub_debug("Got Image to Publish", log_name_list = self.log_name_list, throttle_s = 5.0)
        success = False
        if cv2_img is None:
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
        [height,width] = cv2_img.shape[0:2]
        last_width = self.status_msg.width_px
        last_height = self.status_msg.height_px
        self.status_msg.width_px = width
        self.status_msg.height_px = height

        self.status_msg.width_deg = width_deg
        self.status_msg.height_deg = height_deg

        if (min_range_m is not None and max_range_m is not None):
            self._updateRangesM(min_range_m,max_range_m)
            self.msg_if.pub_debug("Got Image size: " + str([height,width]), log_name_list = self.log_name_list, throttle_s = 5.0)

        if self.has_subs == False:
            self.msg_if.pub_debug("Depthmap has no subscribers", log_name_list = self.log_name_list, throttle_s = 5.0)
            if self.status_msg.publishing == True:
                self.msg_if.pub_warn("Depthmap has no subscribers", log_name_list = self.log_name_list)
            self.status_msg.publishing = False

        else:
            self.msg_if.pub_debug("Depthmap has subscribers, will publish", log_name_list = self.log_name_list, throttle_s = 5.0)
            if self.status_msg.publishing == False:
                self.msg_if.pub_warn("Depthmap has subscribers, will publish", log_name_list = self.log_name_list)
            self.status_msg.publishing = True

            #Convert to ros Image message
            ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
            sec = nepi_sdk.sec_from_timestamp(timestamp)
            ros_img.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = frame_3d)
            self.msg_if.pub_debug("Publishing Image with header: " + str(ros_img.header), log_name_list = self.log_name_list, throttle_s = 5.0)
            self.node_if.publish_pub('data_pub', ros_img)
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
        return True

    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_sdk.wait()
        self.namespace = '~'
        self.status_msg = None


    def publish_status(self, do_updates = True):
        if self.node_if is not None:
            if self.status_msg is None:
                self.status_msg = DepthMapStatus()
            self.status_msg.min_range_m = self.min_range_m
            self.status_msg.max_range_m = self.max_range_m

            if do_updates == True:
                self.status_msg.max_data_pub_rate = self.node_if.get_param('max_img_pub_rate')
                self.status_msg.range_ratios.start_range = self.node_if.get_param('start_range_ratio')
                self.status_msg.range_ratios.stop_range = self.node_if.get_param('stop_range_ratio')

            avg_rate = 0
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate

            self.node_if.publish_pub('status_pub',self.status_msg)

    def init(self):
        pass
        
    def reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
            self.init()
            self.publish_status()


    def factory_reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Factory reseting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
            self.init()
            self.publish_status()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init()

    def _resetCb(self):
        self.reset()

    def _factoryResetCb(self):
        self.factory_reset()

    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if has_subs == False:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        #self.msg_if.pub_debug("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
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





##################################################
# PointcloudIF



class PointcloudIF:

    DEFAULT_IMG_WIDTH_DEG = 100
    DEFUALT_IMG_HEIGHT_DEG = 70

    Factory_Image_Width = 955
    Factory_Image_Height = 600
    Factory_Start_Range_Ratio = 0.0
    Factory_Stop_Range_Ratio = 1.0
    Factory_Zoom_Ratio = .5
    Factory_Rotate_Ratio = .5
    Factory_Tilt_Ratio = .5
    Factory_Cam_FOV = 60
    Factory_Cam_View = [3, 0, 0]
    Factory_Cam_Pos = [-5, 0, 0]
    Factory_Cam_Rot = [0, 0, 1]

    ready = False
    namespace = '~'

    node_if = None

    status_msg =  PointcloudStatus()

    last_pub_time = None

    has_subs = False

    time_list = [0,0,0,0,0,0,0,0,0,0]

    img_pub_file = 'nepi_pointcloud_img_pub_node.py'



    data_source_description = 'pointcloud_sensor'
    data_ref_description = 'sensor'

    data_product = 'pointcloud'
    data_products_list = [data_product]

    def __init__(self, namespace = None,
                data_product_name = 'pointcloud',
                data_source_description = 'pointcloud_sensor',
                data_ref_description = 'sensor',
                enable_data_pub = True,
                max_data_pub_rate = 5,
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


        self.msg_if.pub_warn("Got namespace: " + str(namespace), log_name_list = self.log_name_list)
        if namespace is None:
            namespace = self.namespace
        namespace = nepi_sdk.get_full_namespace(namespace)
        self.namespace = nepi_sdk.create_namespace(namespace,self.data_product)
        self.msg_if.pub_warn("Using data product namespace: " + str(self.namespace), log_name_list = self.log_name_list)


        '''
        if enable_data_pub == True:
            ## Get folder info
            #self.mgr_sys_srv_if = ConnectMgrSystemServicesIF()
            #success = self.mgr_sys_srv_if.wait_for_services()
            #if success == False:
            #    nepi_sdk.signal_shutdown(self.node_name + ": Failed to get System Status Msg", log_name_list = self.log_name_list)

            #self.api_lib_folder = mgr_sys_srv_if.get_sys_folder_path('api_lib',API_LIB_FOLDER)
            #self.msg_if.pub_info("Using User Config Folder: " + str(self.api_lib_folder), log_name_list = self.log_name_list)

            self.api_lib_folder = API_LIB_FOLDER
            self.msg_if.pub_info("Using SDK Share Folder: " + str(self.api_lib_folder), log_name_list = self.log_name_list)



            # Launch detection img pub node that handles detection image publishing
            pkg_name = 'nepi_api'
            node_file_folder = self.api_lib_folder
            img_pub_file = self.img_pub_file
            img_pub_file_path = os.path.join(node_file_folder,img_pub_file)
        
            if os.path.exists(img_pub_file_path) == False or enable_data_pub == False:
                self.msg_if.pub_warn("Could not find Img Pub Node file at: " + img_pub_file_path, log_name_list = self.log_name_list)
            else: 
                #Try and launch node
                unique_name = nepi_sdk.get_unique_name_from_namespace(self.namespace,self.base_namespace)
                img_pub_node_name = unique_name + "_img_pub"
                self.msg_if.pub_warn("Launching Img Pub Node: " + img_pub_node_name, log_name_list = self.log_name_list)
                img_pub_namespace = self.node_namespace + "_img_pub"
                self.msg_if.pub_warn("Launching Img Pub Namespace: " + img_pub_namespace, log_name_list = self.log_name_list)

                # Pre Set Img Pub Params
                pc_data_product = os.path.basename(self.namespace)
                img_data_product = pc_data_product + '_image'
                param_ns = nepi_sdk.create_namespace(img_pub_namespace,'data_product')
                nepi_sdk.set_param(param_ns,img_data_product)

                param_ns = nepi_sdk.create_namespace(img_pub_namespace,'pc_namespace')
                nepi_sdk.set_param(param_ns,self.node_namespace)
                        

                [success, msg, pub_process] = nepi_sdk.launch_node(pkg_name, img_pub_file, img_pub_node_name)

                self.msg_if.pub_warn("Img Pub Node launch return msg: " + msg, log_name_list = self.log_name_list)
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
        self.status_msg.data_pub_enabled = enable_data_pub
        self.status_msg.max_data_pub_rate = max_data_pub_rate
        self.status_msg.standard_image_sizes = nepi_img.STANDARD_IMAGE_SIZES



        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self._initCb,
            'reset_callback': self._resetCb,
            'factory_reset_callback': self._factoryResetCb,
            'init_configs': True,
            'namespace': self.namespace
        }

        # Params Config Dict ####################
        self.PARAMS_DICT = {
            'max_img_pub_rate': {
                'namespace': self.node_namespace,
                'factory_val': max_data_pub_rate
            },
            'render_image_width': {
                'namespace': self.node_namespace,
                'factory_val': Factory_Image_Width
            },
            'render_image_height': {
                'namespace': self.node_namespace,
                'factory_val': Factory_Image_Height
            },
            'render_start_range_ratio': {
                'namespace': self.node_namespace,
                'factory_val': Factory_Start_Range_Ratio
            },
            'render_zoom_ratio': {
                'namespace': self.node_namespace,
                'factory_val': Factory_Zoom_Ratio
            },
            'render_rotate_ratio': {
                'namespace': self.node_namespace,
                'factory_val': Factory_Rotate_Ratio
            },
            'render_tilt_ratio': {
                'namespace': self.node_namespace,
                'factory_val': Factory_Tilt_Ratio
            },
            'render_cam_fov': {
                'namespace': self.node_namespace,
                'factory_val': Factory_Cam_FOV
            },
            'render_cam_view': {
                'namespace': self.node_namespace,
                'factory_val':Factory_Cam_View
            },
            'render_cam_pos': {
                'namespace': self.node_namespace,
                'factory_val':Factory_Cam_Pos
            },
            'render_cam_rot': {
                'namespace': self.node_namespace,
                'factory_val':Factory_Cam_Rot
            },
            'render_use_wbg': {
                'namespace': self.node_namespace,
                'factory_val': False
            },
        }

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
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'set_range_window': {
                'namespace': self.node_namespace,
                'topic': 'render_set_range_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self._setRenderRangeCb, 
                'callback_args': ()
            },
            'set_zoom_ratio': {
                'namespace': self.node_namespace,
                'topic': 'render_set_zoom_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setZoomCb, 
                'callback_args': ()
            },
            'set_rotate_ratio': {
                'namespace': self.node_namespace,
                'topic': 'render_set_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setRotateCb, 
                'callback_args': ()
            },
            'set_tilt_ratio': {
                'namespace': self.node_namespace,
                'topic': 'render_set_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setTiltCb, 
                'callback_args': ()
            },
            'set_camera_fov': {
                'namespace': self.node_namespace,
                'topic': 'render_set_camera_fov',
                'msg': Int32,
                'qsize': 10,
                'callback': self._setCamFovCb, 
                'callback_args': ()
            },
            'set_camera_view': {
                'namespace': self._node_namespace,
                'topic': 'render_set_camera_view',
                'msg': Vector3,
                'qsize': 10,
                'callback': self._setCamViewCb, 
                'callback_args': ()
            },
            'set_camera_position': {
                'namespace': self.node_namespace,
                'topic': 'render_set_camera_position',
                'msg': Vector3,
                'qsize': 10,
                'callback': self._setCamPositionCb, 
                'callback_args': ()
            },
            'set_camera_rotation': {
                'namespace': self.node_namespace,
                'topic': 'render_set_camera_rotation',
                'msg': Vector3,
                'qsize': 10,
                'callback': self._setCamRotationCb, 
                'callback_args': ()
            },
            'set_white_bg_enable': {
                'namespace': self.node_namespace,
                'topic': 'render_set_white_bg_enable',
                'msg': Bool,
                'qsize': 10,
                'callback': self._setWhiteBgCb, 
                'callback_args': ()
            },
            'reset_render_controls': {
                'namespace': self.node_namespace,
                'topic': 'render_reset_controls',
                'msg': Empty,
                'qsize': 1,
                'callback': self._resetRenderControlsCb, 
                'callback_args': ()
            }
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

        
        #self.node_if.wait_for_ready()
        nepi_sdk.wait()

        self.publish_status(do_updates=True)

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
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
        self.msg_if.pub_debug("Returning: " + self.namespace + " " "has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        return has_subs



    def publish_o3d_pc(self,o3d_pc,
                        timestamp = None, 
                        frame_3d = 'sensor_frame', 
                        width_deg = DEFAULT_IMG_WIDTH_DEG,
                        height_deg = DEFUALT_IMG_HEIGHT_DEG,
                        device_mount_description = 'fixed'):

        if self.node_if is None:
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

        current_time = nepi_utils.get_time()
        latency = (current_time - timestamp)
        self.status_msg.get_latency_time = latency
        #self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency), log_name_list = self.log_name_list, throttle_s = 5.0)

        # Start Img Pub Process
        start_time = nepi_utils.get_time()   

        self.status_msg.point_count = o3d_pc.point["colors"].shape[0]

        has_subs = copy.deepcopy(self.has_subs)

        if self.has_subs == False:
            if self.status_msg.publishing == True:
                self.msg_if.pub_warn("Pointcloud has no subscribers", log_name_list = self.log_name_list)
            self.status_msg.publishing = False
        else:
            if self.status_msg.publishing == False:
                self.msg_if.pub_warn("Pointcloud has subscribers, will publish", log_name_list = self.log_name_list)
            self.status_msg.publishing = True
            #Convert to ros Image message
            ros_pc = nepi_pc.o3dpc_to_rospc(o3d_pc, frame_3d = frame_3d)
            sec = nepi_sdk.sec_from_timestamp(timestamp)
            ros_pc.header = nepi_sdk.create_header_msg(time_sec = sec, frame_id = frame_3d)

            process_time = round( (nepi_utils.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time - timestamp)
            self.status_msg.pub_latency_time = latency


            if not nepi_sdk.is_shutdown():
                self.node_if.publish_pub('data_pub', ros_pc)

            if self.last_pub_time is None:
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time
                self.status_msg.last_pub_sec = pub_time_sec

                self.time_list.pop(0)
                self.time_list.append(pub_time_sec)

                
            process_time = round( (nepi_utils.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
        return True

    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_sdk.wait()
        self.node_if = None
        self.namespace = '~'
        self.status_msg = None


    def publish_status(self, do_updates = True):
        if self.node_if is not None:
            if self.status_msg is None:
                self.status_msg =  PointcloudStatus()
            if do_updates == True:
                self.status_msg.max_data_pub_rate = self.node_if.get_param('max_img_pub_rate')
                self.status_msg.image_width = self.node_if.get_param('render_image_width')
                self.status_msg.image_height = self.node_if.get_param('render_image_height')

                range_ratios = RangeWindow()
                range_ratios.start_range =   float(self.node_if.get_param('render_start_range_ratio'))
                range_ratios.stop_range =   float(self.node_if.get_param('render_stop_range_ratio'))
                self.status_msg.range_ratios = range_ratios

                self.status_msg.zoom_ratio = self.node_if.get_param('render_zoom_ratio')
                self.status_msg.rotate_ratio = self.node_if.get_param('render_rotate_ratio')
                self.status_msg.tilt_ratio = self.node_if.get_param('render_tilt_ratio')

                fov = self.node_if.get_param('render_cam_fov')
                self.status_msg.camera_fov = fov

                view = self.node_if.get_param('render_cam_view')
                cam_view = Vector3()
                cam_view.x = view[0]
                cam_view.y = view[1]
                cam_view.z = view[2]
                self.status_msg.camera_view = cam_view

                pos = self.node_if.get_param('render_cam_pos')
                cam_pos = Vector3()
                cam_pos.x = pos[0]
                cam_pos.y = pos[1]
                cam_pos.z = pos[2]
                self.status_msg.camera_position = cam_pos

                rot = self.node_if.get_param('render_cam_rot')
                cam_rot = Vector3()
                cam_rot.x = rot[0]
                cam_rot.y = rot[1]
                cam_rot.z = rot[2]
                self.status_msg.camera_rotation = cam_rot
                
                use_wbg = self.node_if.get_param('render_use_wbg')
                self.status_msg.white_background = use_wbg

            avg_rate = 0
            avg_time = sum(self.time_list) / len(self.time_list)
            if avg_time > .01:
                avg_rate = float(1) / avg_time
            self.status_msg.avg_pub_rate = avg_rate

            self.node_if.publish_pub('status_pub',self.status_msg)


    def init(self):
        pass
        
    def reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Reseting params", log_name_list = self.log_name_list)
            self.node_if.reset_params()
            self.init()
            self.publish_status()


    def factory_reset(self):
        if self.node_if is not None:
            self.msg_if.pub_info("Factory reseting params", log_name_list = self.log_name_list)
            self.node_if.factory_reset_params()
            self.init()
            self.publish_status()

    ###############################
    # Class Private Methods
    ###############################
    def _initCb(self, do_updates = False):
        self.init()

    def _resetCb(self):
        self.reset()

    def _factoryResetCb(self):
        self.factory_reset()

    def _subscribersCheckCb(self,timer):
        has_subs = self.node_if.pub_has_subscribers('data_pub')
        if has_subs == False:
            self.status_msg.publishing = False
        self.has_subs = has_subs
        #self.msg_if.pub_debug("Subs Check End: " + self.namespace + " has subscribers: " + str(has_subs), log_name_list = self.log_name_list, throttle_s = 5.0)
        nepi_sdk.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        self.publish_status(do_updates = True)


    def _setRenderRangeCb(self, msg):
        self.msg_if.pub_info("Recived Range update message: " + str(msg), log_name_list = self.log_name_list)
        self.msg_if.pub_info("Recived update message: " + str(msg), log_name_list = self.log_name_list)
        new_start_range_ratio = msg.start_range
        new_stop_range_ratio = msg.stop_range
        if (new_start_range_ratio < 0 or new_stop_range_ratio > 1 or new_stop_range_ratio < new_start_range_ratio):
            self.msg_if.pub_error("Range values out of bounds", log_name_list = self.log_name_list)
            self.publish_status(do_updates=False) # No change
            return
        else:
            self.status_msg.range_ratios.start_range = new_start_range_ratio
            self.status_msg.range_ratios.stop_range = new_stop_range_ratio
            self.publishStatus(do_updates=False) # Updated inline here  

            self.node_if.set_param('render_start_range_ratio', new_start_range_ratio)
            self.node_if.set_param('render_stop_range_ratio', new_stop_range_ratio)
        
    def _setZoomRatioCb(self,msg):
        self.msg_if.pub_info("Got Zoom Ratio: " + str(msg), log_name_list = self.log_name_list, throttle_s = 1.0)
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.status_msg.zoom_ratio = new_val
            self.publish_status(do_updates=False)
            self.node_if.set_param('render_zoom_ratio',new_val)


    def _setRotateRatioCb(self,msg):
        self.msg_if.pub_info("Got Rotate Ratio: " + str(msg), log_name_list = self.log_name_list, throttle_s = 1.0)
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.status_msg.rotate_ratio = new_val
            self.publish_status(do_updates=False)
            self.node_if.set_param('render_rotate_ratio',new_val)

    def _setTiltRatioCb(self,msg):
        self.msg_if.pub_info("Got Tilt Ratio: " + str(msg), log_name_list = self.log_name_list, throttle_s = 1.0)
        new_val = msg.data
        if new_val >= 0 and new_val <= 1 :
            self.status_msg.tilt_ratio = new_val
            self.publish_status(do_updates=False)
            self.node_if.set_param('render_tilt_ratio',new_val)

    def _setCamFovCb(self,msg):
        self.msg_if.pub_info("Got Cam FOV: " + str(msg), log_name_list = self.log_name_list, throttle_s = 1.0)
        new_val = msg.data
        if new_val > 100:
            new_val = 100
        if new_val < 30:
            new_val = 30
        self.status_msg.camera_fov = new_val
        self.publish_status(do_updates=False)
        self.node_if.set_param('render_cam_fov',new_val)


    def _setCamViewCb(self,msg):
        self.msg_if.pub_info("Got Cam View: " + str(msg), log_name_list = self.log_name_list, throttle_s = 1.0)
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.status_msg.camera_view = new_val
        self.publish_status(do_updates=False)
        self.node_if.set_param('render_cam_view',new_array)


    def _setCamPositionCb(self,msg):
        self.msg_if.pub_info("Got Cam Pos: " + str(msg), log_name_list = self.log_name_list, throttle_s = 1.0)
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.status_msg.camera_position = new_val
        self.publish_status(do_updates=False)
        self.node_if.set_param('render_cam_pos',new_array)

    def _setCamRotationCb(self,msg):
        self.msg_if.pub_info("Got Cam Rotate: " + str(msg), log_name_list = self.log_name_list, throttle_s = 1.0)
        new_array = []
        new_array.append(msg.x)
        new_array.append(msg.y)
        new_array.append(msg.z)
        self.status_msg.camera_rotation = new_val
        self.publish_status(do_updates=False)
        self.node_if.set_param('render_cam_rot',new_array)
    

    def _setWhiteBgCb(self,msg):
        enable = msg.data
        self.status_msg.white_background = enable
        self.publish_status(do_updates=False)
        self.node_if.set_param('render_use_wbg', enable)


    def _resetRenderControlsCb(self,msg):
        self.node_if.reset_params()
        self.publish_status(do_updates=True)

