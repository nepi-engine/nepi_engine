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
import numpy as np
import cv2
import open3d as o3d

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header

from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped

from nepi_ros_interfaces.msg import NavPose, NavPoseData, NavPoseStatus, Heading
from nepi_ros_interfaces.srv import  NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse

from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import StringArray, ImageStatus

from sensor_msgs.msg import PointCloud2
from nepi_ros_interfaces.msg import PointcloudStatus


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF



SYSTEM_DATA_FOLDER = 'mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']


EXAMPLE_FILENAME_DICT = {
    'prefix': "", 
    'add_timestamp': True, 
    'add_ms': True,
    'add_ns': False,
    'suffix': "",
    'add_node_name': False
    }



class ReadWriteIF:

    ready = False
    
    # Save data variables
    filename_dict = {
    'prefix': "", 
    'add_timestamp': True, 
    'add_ms': True,
    'add_ns': False,
    'suffix': "",
    'add_node_name': False
    }

    node_if = None

    #######################
    ### IF Initialization
    def __init__(self,
                filename_dict = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")
        

        #############################
        # Initialize Class Variables
        if filename_dict is not None:
            self.filename_dict = filename_dict

        self.data_dict = {
            'dict': {
                'data_type': dict,
                'file_types': ['yaml'],
                'read_func': self.read_yaml_2_dict,
                'write_func': self.write_dict_2_yaml
            },
            'image': {
                'data_type': np.ndarray,
                'file_types': ['png','PNG','jpg','jpeg','JPG'],
                'read_func': self.read_img_2_cv2img,
                'write_func': self.write_cv2img_2_img 
            },
            'pointcloud': {
                'data_type': o3d.geometry.PointCloud,
                'file_types': ['pcd'],
                'read_func': self.read_pcb_2_o3dp,
                'write_func': self.write_o3dpc_2_pcd 
            },
        }

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    ###############################
    # Class Public Methods
    ###############################

    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready   
        

    def get_supported_data_dict(self):
        return self.data_dict

    def get_data_folder(self):
        return self.save_folder

    def set_data_folder(self, data_path):
        success = True
        if os.path.exists(data_path) == False:
            self.msg_if.pub_warn("File path does not exist: " + data_path)
        elif os.path.isdir(data_path) == False:
            self.msg_if.pub_warn("File path not a folder: " + data_path)
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

    def get_add_timestamp(self):
        return self.filename_dict['add_timestamp']

    def set_add_timestamp(self, add_timestamp):
        self.filename_dict['add_timestamp'] = add_timestamp

    def get_add_ms(self):
        return self.filename_dict['add_ms']

    def set_add_ms(self, add_ms):
        self.filename_dict['add_ms'] = add_ms

    def get_add_ns(self):
        return self.filename_dict['add_ns']

    def set_add_ns(self, add_ns):
        self.filename_dict['add_ns'] = add_ns

    def get_filename_suffix(self):
        return self.filename_dict['suffix']

    def set_filename_suffix(self, suffix = ''):
        self.filename_dict['suffix'] = suffix

    def set_filename_config(self, prefix = '', suffix = '', add_time = True, add_ms = True, add_ns = False):
        self.filename_dict = {
        'prefix': prefix, 
        'add_timestamp': add_timestamp, 
        'add_ms': add_ms,
        'add_ns': add_ns,
        'suffix': suffix
        }

    def get_filename_config(self):
        return self.filename_dict


    def get_folder_files(self, path, ext_str = ""):
        file_list = nepi_utils.get_file_list(path,ext_str=ext_str)
        return file_list

    '''
    def get_file_times(self,file_list):
        file_times = []
        for file in file_list:
            file_times.append(nepi_utils.get_time_from_filename(file))
        return file_times
    '''

    def read_dict_file(self, filepath, filename):
        data_key = 'dict'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(filepath,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def write_dict_file(self, filepath, data, data_name, timestamp = None, ext_str = 'yaml'):
        data_key = 'dict'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type))
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
            else:
                filename = self._createFileName(data_name,ext_str,timestamp)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success


    def read_image_file(self, filepath, filename):
        data_key = 'image'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(filepath,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def write_image_file(self, filepath, data, data_name, timestamp = None, ext_str = 'png'):
        data_key = 'image'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type))
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
            else:
                filename = self._createFileName(data_name,ext_str,timestamp)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success


    def read_pointcloud_file(self, filepath, filename):
        data_key = 'pointcloud'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(filepath,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def write_pointcloud_file(self, filepath, data, data_name, timestamp = None, ext_str = 'pcd'):
        data_key = 'pointcloud'
        success = False
        data_type = self.data_dict[data_key]['data_type']
        if isinstance(data,data_type) == False:
            self.msg_if.pub_warn("Data type not supported: " + str(data_type))
        else:
            file_types = self.data_dict[data_key]['file_types']
            if ext_str not in file_types:
                self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
            else:
                filename = self._createFileName(data_name,ext_str,timestamp)
                file_path = os.path.join(filepath,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success



    ###############################
    # Class Private Methods
    ###############################

    def _createFileName(self,data_name_str,ext_str,timestamp=None):
        prefix = self.filename_dict['prefix']
        if len(prefix) > 0:
            prefix = prefix + '_'
        add_time = self.filename_dict['add_timestamp']
        data_time_str = ''
        if add_time == True:
            time_ns = nepi_ros.time_ns_from_timestamp(timestamp)
            add_ms = self.filename_dict['add_ms']
            add_ns = self.filename_dict['add_ns']
            data_time_str = nepi_utils.get_datetime_str_from_time(time_ns, add_ms = add_ms, add_ns = add_ns) + '_'
        node_name_str = ""
        if self.filename_dict['add_node_name'] == True:
            node_name_str = self.node_name + '_'
        if len(data_name_str) > 0:
            data_name_str = data_name_str + '_'
        suffix = self.filename_dict['suffix']
        if len(suffix) > 0:
            suffix = suffix + '_'
        filename = prefix + data_time_str + node_name_str + data_name_str + suffix + '.' + ext_str
        return filename




NAVPOSE_POS_FRAME_OPTIONS = ['ENU','NED']
NAVPOSE_ALT_FRAME_OPTIONS = ['AMSL','WGS84']

EXAMPLE_NAVPOSE_DATA_DICT = {
                          'time': nepi_utils.get_time(),
                          'frame_id': 'nepi_base',
                          'frame_pos': 'ENU',
                          'frame_alt': 'WGS84',

                          'geoid_height_meters': 0,

                          'heading_deg': 120.50,

                          # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
                          'roll_deg': 30.51,
                          'pitch_deg': 30.51,
                          'yaw_deg': 30.51,

                          # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
                          'x_m': 1.234,
                          'y_m': 1.234,
                          'z_m': 1.234,

                          # Global Location in set altitude frame (lat,long,alt) with alt in meters
                          'lat': 47.080909,
                          'long': -120.8787889,
                          'alt_m': 12.321,
}




class NavPoseIF:

    ready = False
    namespace = '~navpose'

    node_if = None

    status_msg = NavPoseStatus()

    last_pub_time = None

    has_subscribers = False


    def __init__(self, namespace = None, 
        has_location = False, enable_gps_pub = True, 
        has_position = False, has_orientation = False, enable_pose_pub = True, 
        has_heading = False, enable_heading_pub = True):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    
        # Initialize Class Variables
        if namespace is not None:
            self.namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(self.namespace)

        # Initialize Status Msg.  Updated on each publish
        status_msg = NavPoseStatus()
        status_msg.has_location = has_location
        status_msg.has_position = has_position
        status_msg.has_orientation = has_orientation
        status_msg.has_heading = has_heading

        status_msg.frame_id = "nepi_base"
        status_msg.get_latency_time
        status_msg.pub_latency_time
        status_msg.process_time
        self.status_msg = status_msg


        ##############################   
        ## Node Setup

        # Params Config Dict ####################
        self.PARAMS_DICT = {
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'navpose_pub': {
                'msg': NavPoseData,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'gps_pub': {
                'msg': NavSatFix,
                'namespace': os.path.dirname(self.namespace),
                'topic': 'gps_fix',
                'qsize': 1,
                'latch': False
            },
            'pose_pub': {
                'msg': Odometry,
                'namespace': os.path.dirname(self.namespace),
                'topic': 'odom',
                'qsize': 1,
                'latch': False
            },
            'heading_pub': {
                'msg': Heading,
                'namespace': os.path.dirname(self.namespace),
                'topic': 'heading',
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
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.node_if.wait_for_ready()

        ##############################
        # Start Node Processes
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        nepi_ros.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        return self.has_subscribers


    # Update System Status
    def publish_navpose(self,navpose_dict, timestamp = None, frame_id = 'sensor_frame' ):      
        success = True
        if navpose_dict is None:
            success = False
        else:
            # Pub NavPoseData
            if self.node_if is not None:

                if timestamp == None:
                    timestamp = nepi_ros.ros_time_now()

                current_time = nepi_ros.ros_time_now()
                latency = (current_time.to_sec() - timestamp.to_sec())
                self.status_msg.get_latency_time = latency
                #self.msg_if.pub_info("Get Img Latency: {:.2f}".format(latency))

                # Start Img Pub Process
                start_time = nepi_ros.get_time()   

                try:
                    msg = nepi_nav.convert_navposedata_dict2msg(navpose_dict)
                    self.node_if.publish_pub('navpose_pub', msg)
                    if self.last_pub_time is None:
                        self.last_pub_time = nepi_utils.get_time()
                    else:
                        cur_time = nepi_utils.get_time()
                        pub_time_sec = cur_time - self.last_pub_time
                        self.last_pub_time = cur_time
                        self.status_msg.frame_id = navpose_dict['frame_id']
                        self.status_msg.frame_pos = navpose_dict['frame_pos']
                        self.status_msg.frame_alt = navpose_dict['frame_alt']
                        self.status_msg.last_pub_sec = pub_time_sec
                        self.status_msg.fps = float(1) / pub_time_sec
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish navpose data msg: " + str(e))
                    success = False

                process_time = round( (nepi_ros.get_time() - start_time) , 3)
                self.status_msg.process_time = process_time
                latency = (current_time.to_sec() - timestamp.to_sec())
                self.status_msg.pub_latency_time = latency

                if self.enable_gps_pub == True:
                    msg = self.PUBS_DICT['gps_pub']['msg']()
                    # gps_fix pub
                    try:
                        if self.status_msg.has_location == True:
                            if navpose_dict['frame_alt'] == 'AMSL':
                                navpose_dict = nepi_nav.convert_navposedata_amsl2wgs84(navpose_dict)
                                
                            msg.latitude = navpose_dict['latitude']
                            msg.latitude = navpose_dict['longitude']
                            msg.latitude = navpose_dict['altitude']
                        self.node_if.publish_pub('gps_pub',msg)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to publish location data msg: " + str(e))
                        success = False


                if self.enable_pose_pub == True:
                    #Create odom pub
                    msg = self.PUBS_DICT['pose_pub']['msg']()
                    try:
                        pose_msg = None
                        if navpose_dict['frame_pos'] == 'NED':
                            navpose_dict = nepi_nav.convert_navposedata_ned2edu(navpose_dict)
                                    
                        if self.status_msg.has_position == True:
                            point_msg = Point()
                            point_msg.x = navpose_dict['x_m']
                            point_msg.y = navpose_dict['y_m']
                            point_msg.z = navpose_dict['z_m']
                            msg.pose.pose.position = point_msg
                        if self.status_msg.has_orientation == True:
                            rpy = [navpose_dict['roll_deg'],navpose_dict['pitch_deg'],navpose_dict['yaw_deg']]
                            quad = nepi_nav.convert_rpy2quat(rpy)
                            quad_msg = Quaternion()
                            quad_msg.x = quad[0]
                            quad_msg.y = quad[1]
                            quad_msg.z = quad[2]
                            quad_msg.w = quad[3]
                            msg.pose.pose.orientation = quad_msg
                        self.node_if.publish_pub('pose_pub',msg)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to publish pose data msg: " + str(e))
                        success = False

                if self.enable_heading_pub == True:
                    #Create heading pub
                    msg = self.PUBS_DICT['pose_pub']['msg']()
                    try:
                        if self.status_msg.has_heading == True:
                            msg.heading = navpose_dict['heading']
                        self.node_if.publish_pub('heading_pub',msg)
                    except Exception as e:
                        self.msg_if.pub_warn("Failed to publish heading data msg: " + str(e))
                        success = False
        return success


    def unsubsribe(self):
        self.ready = False
        if self.node_if is not None:
            self.node_if.unregister_class()
        time.sleep(1)
        self.namespace = None
        self.status_msg = None

    ###############################
    # Class Private Methods
    ###############################

    def _subscribersCheckCb(self,timer):
        self.has_subscribers = self.node_if.pub_has_subscribers('navpose_pub')
        #self.msg_if.pub_warn("Sub check gotsubscribers: " + str(self.has_subscribers))
        if self.has_subscribers == False:
            self.status_msg.publishing = False
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        if self.node_if is not None and not nepi_ros.is_shutdown():
            self.node_if.publish_pub('status_pub', self.status_msg)






ENCODING_OPTIONS = ["mono8",'rgb8','bgr8','32FC1','passthrough']

DEFUALT_IMG_WIDTH = 700
DEFUALT_IMG_HEIGHT = 400

class ImageIF:

    ready = False
    namespace = '~image'

    node_if = None

    status_msg = ImageStatus()

    blank_img = nepi_img.create_cv2_blank_img(DEFUALT_IMG_WIDTH, DEFUALT_IMG_HEIGHT, color = (0, 0, 0) )

    last_pub_time = None

    nav_mgr_if = None
    nav_mgr_ready = False

    has_subscibers = False


    def __init__(self, namespace = None , init_overlay_list = []):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    
        # Initialize Class Variables
        if namespace is not None:
            self.namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(self.namespace)


        self.init_overlay_list = init_overlay_list

        # Initialize Status Msg.  Updated on each publish
        status_msg = ImageStatus()
        status_msg.publishing = False
        status_msg.encoding = 'bgr8'
        status_msg.width = 0
        status_msg.height = 0
        status_msg.frame_id = "sensor_frame"
        status_msg.depth_map_topic = nepi_img.get_img_depth_map_topic(namespace)
        status_msg.pointcloud_topic = nepi_img.get_img_pointcloud_topic(namespace)
        status_msg.get_latency_time = 0
        status_msg.pub_latency_time = 0
        status_msg.process_time = 0
        self.status_msg = status_msg

        ##############################
        ## Connect NEPI NavPose Manager
        self.nav_mgr_if = ConnectMgrNavPoseIF()
        self.nav_mgr_ready = self.nav_mgr_if.wait_for_ready()

        ##############################   
        ## Node Setup

        # Params Config Dict ####################
        self.PARAMS_DICT = {
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
            'overlay_list': {
                'namespace': self.namespace,
                'factory_val': []
            },
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'image_pub': {
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

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'overlay_img_name': {
                'msg': Bool,
                'namespace': self.namespace,
                'topic': 'set_overlay_img_name',
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
            'overlay_list': {
                'msg': StringArray,
                'namespace': self.namespace,
                'topic': 'set_overlay_list',
                'qsize': 1,
                'callback': self._setOverlayListCb
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.node_if.wait_for_ready()

        ##############################
        # Start Node Processes
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        nepi_ros.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################

    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  


    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        return self.has_subscribers


    def publish_cv2_img(self,cv2_img, encoding = "bgr8", timestamp = None, frame_id = 'sensor_frame', add_overlay_list = []):
        #self.msg_if.pub_warn("Got Image to Publish")
        success = False
        if cv2_img is None:
            self.msg_if.pub_info("Can't publish None image")
            return False

        self.status_msg.encoding = encoding

        if timestamp == None:
            timestamp = nepi_ros.ros_time_now()


        current_time = nepi_ros.ros_time_now()
        latency = (current_time.to_sec() - timestamp.to_sec())
        self.status_msg.get_latency_time = latency
        #self.msg_if.pub_info("Get Img Latency: {:.2f}".format(latency))

        # Start Img Pub Process
        start_time = nepi_ros.get_time()   

        # Publish and Save Raw Image Data if Required  
        [height,width] = cv2_img.shape[0:2]
        last_width = self.status_msg.width
        last_height = self.status_msg.height
        self.status_msg.width = width
        self.status_msg.height = height

        #self.msg_if.pub_warn("Got Image size: " + str([height,width]))

        if self.has_subscribers == False:
            if self.status_msg.publishing == True:
                self.msg_if.pub_warn("Image has no subscribers")
            self.status_msg.publishing = False

        else:
            if self.status_msg.publishing == False:
                self.msg_if.pub_warn("Image has subscribers, will publish")
            self.status_msg.publishing = True

            # Apply Overlays
            overlay_list = []
            if self.node_if.get_param('overlay_img_name') == True:
                overlay = nepi_img.getImgShortName(self.img_namespace)
                overlay_list.append(overlay)
            
            if self.node_if.get_param('overlay_date_time') == True:
                date_time = nepi_ros.get_datetime_str_from_stamp(timestamp)
                overlay_list.append(overlay)

            nav_pose_dict = None
            if self.node_if.get_param('overlay_nav') == True or self.node_if.get_param('overlay_pose') == True:
                if self.nav_mgr_ready == True:
                    nav_pose_dict = self.nav_mgr_if.get_navpose_data_dict()
                    if nav_pose_dict is not None:

                        if self.node_if.get_param('overlay_nav') == True and nav_pose_dict is not None:
                            overlay = 'Lat: ' +  str(round(nav_pose_dict['lat'],6)) + 'Long: ' +  str(round(nav_pose_dict['long'],6)) + 'Head: ' +  str(round(nav_pose_dict['heading_deg'],2))
                            overlay_list.append(overlay)

                        if self.node_if.get_param('overlay_pose') == True and nav_pose_dict is not None:
                            overlay = 'Roll: ' +  str(round(nav_pose_dict['roll_deg'],2)) + 'Pitch: ' +  str(round(nav_pose_dict['pitch_deg'],2)) + 'Yaw: ' +  str(round(nav_pose_dict['yaw_deg'],2))
                            overlay_list.append(overlay)

            overlay_list = overlay_list + self.init_overlay_list + add_overlay_list

            cv2_img = nepi_img.overlay_text_list(cv2_img, text_list = overlay_list, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0), apply_shadow = True)


            #Convert to ros Image message
            ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding=encoding)
            ros_img.header.stamp = timestamp
            ros_img.header.frame_id = frame_id

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time.to_sec() - timestamp.to_sec())
            self.status_msg.pub_latency_time = latency
            
            if not nepi_ros.is_shutdown():
                self.node_if.publish_pub('image_pub', ros_img)
            if self.last_pub_time is None:
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time
                self.status_msg.last_pub_sec = pub_time_sec
                self.status_msg.fps = float(1) / pub_time_sec

        # Update blank image if needed
        if last_width != self.status_msg.width or last_height != self.status_msg.height:
            self.blank_img = nepi_img.create_cv2_blank_img(width, height, color = (0, 0, 0) )
        return True

    def publish_cv2_msg_img(self,text):
        cv2_img = nepi_img.overlay_text_autoscale(self.blank_img, text)
        self.publish_cv2_img(cv2_img)


    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_ros.sleep(1)
        self.namespace = '~'
        self.status_msg = None

    ###############################
    # Class Private Methods
    ###############################

    def _subscribersCheckCb(self,timer):
        self.has_subscribers = self.node_if.pub_has_subscribers('image_pub')
        #self.msg_if.pub_warn("Sub check gotsubscribers: " + str(self.has_subscribers))
        if self.has_subscribers == False:
            self.status_msg.publishing = False
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.status_msg.overlay_img_name = self.node_if.get_param('overlay_img_name')
        self.status_msg.overlay_date_time =  self.node_if.get_param('overlay_date_time')
        self.status_msg.overlay_nav = self.node_if.get_param('overlay_nav')
        self.status_msg.overlay_pose = self.node_if.get_param('overlay_pose')  
        self.status_msg.base_overlay_list = self.init_overlay_list
        self.status_msg.add_overlay_list = add_overlays = self.node_if.get_param('add_overlay_list')

        self.node_if.publish_pub('status_pub',self.status_msg)
        self.img_status_msg.publish(self.status_msg)

    def _setOverlayImgNameCb(self,msg):
        self.node_if.set_param('overlay_img_name', msg.data)
        self.publishStatus()

    def _setOverlayDateTimeCb(self,msg):
        self.node_if.set_param('overlay_date_time', msg.data)
        self.publishStatus()

    def _setOverlayNavCb(self,msg):
        self.node_if.set_param('overlay_nav', msg.data)
        self.publishStatus()

    def _setOverlayPoseCb(self,msg):
        self.node_if.set_param('overlay_pose', msg.data)
        self.publishStatus()

    def _setAddOverlayListCb(self,msg):
        self.node_if.set_param('add_overlay_list', msg.data)
        self.publishStatus()













class PointcloudIF:

    ready = False
    namespace = '~pointcloud'

    node_if = None

    status_msg = PointcloudStatus()

    last_pub_time = None

    has_subscribers = False

    def __init__(self, namespace = None, topic = None):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    
        # Initialize Class Variables
        if namespace is not None:
            self.namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(self.namespace)

        # Initialize Status Msg.  Updated on each publish
        status_msg = PointcloudStatus()
        status_msg.publishing = False
        status_msg.has_rgb = False
        status_msg.has_intensity = False
        status_msg.width = 0
        status_msg.height = 0
        status_msg.depth = 0
        status_msg.point_count = 0,
        status_msg.frame_id = "nepi_base"
        status_msg.get_latency_time
        status_msg.pub_latency_time
        status_msg.process_time
        self.status_msg = status_msg


        ##############################   
        ## Node Setup

        # Params Config Dict ####################
        self.PARAMS_DICT = {
        }

        # Pubs Config Dict ####################
        self.PUBS_DICT = {
            'pointcloud_pub': {
                'msg': PointCloud2,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'latch': False
            },
            'status_pub': {
                'msg': PointcloudStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'latch': True
            }
        }

        # Subs Config Dict ####################
        self.SUBS_DICT = {
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        params_dict = self.PARAMS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.node_if.wait_for_ready()

        ##############################
        # Start Node Processes
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)
        nepi_ros.start_timer_process(1.0, self._publishStatusCb, oneshot = False)

        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    ###############################
    # Class Public Methods
    ###############################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready  

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        return status_dict


    def has_subscribers_check(self):
        return self.has_subscribers


    def publish_o3d_pc(self,o3d_pc, timestamp = None, frame_id = 'sensor_frame'):
        if self.node_if is None:
            self.msg_if.pub_info("Can't publish on None publisher")
            return False
        if o3d_pc is None:
            self.msg_if.pub_info("Can't publish None image")
            return False

        if timestamp == None:
            timestamp = nepi_ros.ros_time_now()

        self.status_msg.has_rgb = o3d_pc.has_colors()
        self.status_msg.point_count = o3d_pc.point["colors"].shape[0]

        current_time = nepi_ros.ros_time_now()
        latency = (current_time.to_sec() - timestamp.to_sec())
        self.status_msg.get_latency_time = latency
        #self.msg_if.pub_info("Get Img Latency: {:.2f}".format(latency))

        # Start Img Pub Process
        start_time = nepi_ros.get_time()   
        
        self.status_msg.has_rgb = o3d_pc.has_colors() 
        self.status_msg.has_intensity = False # Need to add


        ''' # Need to add  
        [height,width,depth] = nepi_pc.shape(o3d_pc)
        self.status_msg.width = width
        self.status_msg.height = height
        self.status_msg.depth = depth
        '''

        self.status_msg.point_count = o3d_pc.point["colors"].shape[0]

        if self.has_subscribers == False:
            if self.status_msg.publishing == True:
                self.msg_if.pub_warn("Image has no subscribers")
            self.status_msg.publishing = False
        else:
            if self.status_msg.publishing == False:
                self.msg_if.pub_warn("Image has subscribers, will publish")
            self.status_msg.publishing = True
            #Convert to ros Image message
            ros_pc = nepi_pc.o3dpc_to_rospc(o3d_pc, frame_id = frame_id)
            ros_pc.header.stamp = timestamp
            ros_pc.header.frame_id = frame_id

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time.to_sec() - timestamp.to_sec())
            self.status_msg.pub_latency_time = latency


            if not nepi_ros.is_shutdown():
                self.node_if.publish_pub('pointcloud_pub', ros_pc)

            if self.last_pub_time is None:
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time
                self.status_msg.last_pub_sec = pub_time_sec
                self.status_msg.fps = float(1) / pub_time_sec
            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
        return True


    def unregister(self):
        self.ready = False
        self.node_if.unregister_class()
        nepi_ros.sleep(1)
        self.node_if = None
        self.namespace = '~'
        self.status_msg = None

    ###############################
    # Class Private Methods
    ###############################

    def _subscribersCheckCb(self,timer):
        self.has_subscribers = self.node_if.pub_has_subscribers('pointcloud_pub')
        #self.msg_if.pub_warn("Sub check gotsubscribers: " + str(self.has_subscribers))
        if self.has_subscribers == False:
            self.status_msg.publishing = False
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        if self.node_if is not None and not nepi_ros.is_shutdown():
            self.node_if.publish_pub('status_pub', self.status_msg)




