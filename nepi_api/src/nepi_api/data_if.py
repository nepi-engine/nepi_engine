#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import rospy
import os
import time
import numpy as np
import cv2
import open3d as o3d

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header

SYSTEM_DATA_FOLDER = 'mnt/nepi_storage/data'

SUPPORTED_DICT_FILE_TYPES = ['yaml']
SUPPORTED_IMG_FILE_TYPES = ['png','PNG','jpg','jpeg','JPG']  
SUPPORTED_PC_FILE_TYPES = ['pcd']
SUPPORTED_VID_FILE_TYPES = ['avi','AVI']

SUPPORTED_DATA_TYPES = ['dict','cv2_image','o3d_pointcloud']


class ReadWriteIF:

    ready = False
    
    # Save data variables
    filename_dict = {
    'prefix': "", 
    'add_timestamp': True, 
    'add_ms': True,
    'add_ns': False,
    'suffix': ""
    }


    #######################
    ### IF Initialization
    log_name = "ConnectImageIF"
    def __init__(self,
                prefix = '', 
                suffix = '', 
                add_time = True, 
                add_ms = True, 
                add_ns = False):
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
        self.filename_dict = {
        'prefix': prefix, 
        'add_timestamp': add_timestamp, 
        'add_ms': add_ms,
        'add_ns': add_ns,
        'suffix': suffix
        }

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


    def get_folder_files(self, ext_str = ""):
        file_list = nepi_utils.get_file_list(self.data_folder,ext_str=ext_str)
        return file_list

    '''
    def get_file_times(self,file_list):
        file_times = []
        for file in file_list:
            file_times.append(nepi_utils.get_time_from_filename(file))
        return file_times
    '''

    def read_yaml_file(self, filename):
        data_key = 'dict'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(self.data_folder,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def write_dict_file(self, data, data_name, timestamp = None, ext_str = 'yaml'):
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
                file_path = os.path.join(self.data_folder,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success


    def read_image_file(self, filename):
        data_key = 'image'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(self.data_folder,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def write_image_file(self, data, data_name, timestamp = None, ext_str = 'png'):
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
                file_path = os.path.join(self.data_folder,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success


    def read_pointcloud_file(self, filename):
        data_key = 'pointcloud'
        data = None
        ext_str = os.path.splitext(filename)[1]
        file_types = self.data_dict[data_key]['file_types']
        if ext_str not in file_types:
            self.msg_if.pub_warn("File type not supported: " + ext_str + " : " + str(file_types))
        else:
            file_path = os.path.join(self.data_folder,filename)
            read_data = self.data_dict[data_key]['read_func'](file_path)
            data_type = self.data_dict[data_key]['data_type']
            if isinstance(read_data,data_type) == False:
                self.msg_if.pub_warn("Data type not supported: " + str(data_type))
            else:
                data = read_data
        return data

    def write_pointcloud_file(self, data, data_name, timestamp = None, ext_str = 'pcd'):
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
                file_path = os.path.join(self.data_folder,filename)
                if os.path.exists(file_path) == True:
                    self.msg_if.pub_warn("File already exists: " + file_path)
                else:
                    success = self.data_dict[data_key]['write_func'](data, file_path)
        return success



    ###############################
    # Class Private Methods
    ###############################

    def _createFileName(self,data_name_str,ext_str,timestamp):
        prefix = self.filename_dict['prefix']
        add_time = self.filename_dict['add_timestamp']
        if add_time == True:
            time_ns = nepi_ros.sec_from_ros_stamp(timestamp)
            add_ms = self.filename_dict['add_ms']
            add_ns = self.filename_dict['add_ns']
            data_time_str = nepi_utils.get_datetime_str_from_time(time_ns, add_ms = add_ms, add_ns = add_ns)
        suffix = self.filename_dict['suffix']
        filename = prefix + '_' + data_time_str + '_' + data_name + '_' + suffix + '.' + ext_str
        return filename




import os
import rospy
import time
from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped

from nepi_ros_interfaces.msg import NavPose, NavPoseData, NavPoseStatus, Heading
from nepi_ros_interfaces.srv import  NavPoseCapabilitiesQuery, NavPoseCapabilitiesQueryResponse

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF

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

    pub_components = False

    has_subscribers = False


    def __init__(self, namespace = None, 
        has_location = False, enable_gps_pub = False, 
        has_orientation = False, has_position = False, enable_pose_pub = False, 
        has_heading = False, enable_heading_pub = False):
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

        self.pub_components = pub_components

        # Initialize Status Msg.  Updated on each publish
        status_msg = PointcloudStatus()
        status_msg.publishing = False

        status_msg.has_location = has_location
        status_msg.has_position = has_position
        status_msg.has_pose = has_pose
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
        self.node_if = NodeClassIF(self,
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
    def publish_navpose(self,navpose_dict, ros_timestamp = None, frame_id = 'sensor_frame' ):      
        success = True
        if navpose_dict is None:
            success = False
        else:
            # Pub NavPoseData
            if self.node_if is not None:

                if ros_timestamp == None:
                    ros_timestamp = nepi_ros.ros_time_now()

                current_time = nepi_ros.ros_time_now()
                latency = (current_time.to_sec() - ros_timestamp.to_sec())
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
                latency = (current_time.to_sec() - ros_timestamp.to_sec())
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
        self.has_subscribers = self.node_if.pub_has_subscribers('data_pub')
        #self.msg_if.pub_warn("Sub check gotsubscribers: " + str(self.has_subscribers))
        if self.has_subscribers == False:
            self.status_msg.publishing = False
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        if self.node_if is not None and not nepi_ros.is_shutdown():
            self.node_if.publish_pub('status_pub', self.status_msg)


import os
import rospy
import time
import cv2
import open3d as o3d

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import PointCloud2

from nepi_ros_interfaces.msg import PointcloudStatus

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc

from nepi_api.node_if import NodeClassIF
from nepi_api.sys_if_msg import MsgIF


class PointcloudIF:

    ready = False
    namespace = '~pointcloud'

    node_if = None

    status_msg = PointcloudStatus()

    last_pub_time = None

    has_subscribers = False

    def __init__(self, namespace = None):
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
        self.node_if = NodeClassIF(self,
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


    def publish_o3d_pc(self,o3d_pc, ros_timestamp = None, frame_id = 'sensor_frame'):
        if self.node_if is None:
            self.msg_if.pub_info("Can't publish on None publisher")
            return False
        if o3d_pc is None:
            self.msg_if.pub_info("Can't publish None image")
            return False

        if ros_timestamp == None:
            ros_timestamp = nepi_ros.ros_time_now()

        self.status_msg.has_rgb = o3d_pc.has_colors()
        self.status_msg.point_count = o3d_pc.point["colors"].shape[0]

        current_time = nepi_ros.ros_time_now()
        latency = (current_time.to_sec() - ros_timestamp.to_sec())
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
            ros_pc.header.stamp = ros_timestamp
            ros_pc.header.frame_id = frame_id

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.status_msg.process_time = process_time
            latency = (current_time.to_sec() - ros_timestamp.to_sec())
            self.status_msg.pub_latency_time = latency


            if not nepi_ros.is_shutdown():
                self.node_if.publish_pub('data_pub', ros_pc)

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
        self.has_subscribers = self.node_if.pub_has_subscribers('data_pub')
        #self.msg_if.pub_warn("Sub check gotsubscribers: " + str(self.has_subscribers))
        if self.has_subscribers == False:
            self.status_msg.publishing = False
        nepi_ros.start_timer_process(1.0, self._subscribersCheckCb, oneshot = True)


    def _publishStatusCb(self,timer):
        if self.node_if is not None and not nepi_ros.is_shutdown():
            self.node_if.publish_pub('status_pub', self.status_msg)




