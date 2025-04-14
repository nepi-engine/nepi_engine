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

NAVPOSE_3D_FRAME_OPTIONS = ['ENU','NED']
NAVPOSE_ALT_FRAME_OPTIONS = ['AMSL','WGS84']

EXAMPLE_NAVPOSE_DATA_DICT = {
                          'time': nepi_utils.get_time(),
                          'frame_3d': 'ENU',
                          'frame_alt': 'WGS84',

                          'geoid_height_meters': 0,

                          'has_heading': True,
                          'heading_deg': 120.50,

                          'has_oreientation': True,
                          # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
                          'roll_deg': 30.51,
                          'pitch_deg': 30.51,
                          'yaw_deg': 30.51,

                          'has_position': True,
                          # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
                          'x_m': 1.234,
                          'y_m': 1.234,
                          'z_m': 1.234,

                          'has_location': True,
                          # Global Location in set altitude frame (lat,long,alt) with alt in meters
                          'lat': 47.080909,
                          'long': -120.8787889,
                          'alt_m': 12.321,
}




class NavPoseIF:

    ready = False
    namespace = '~pointcloud'

    node_if = None

    status_msg = PointcloudStatus()

    last_pub_time = None

    pub_components = False

    has_subscribers = False


    def __init__(self, namespace = None, enable_gps_pub = False, enable_pose_pub = False, enable_heading_pub = False):
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
        status_msg.has_rgb = False
        status_msg.has_intensity = False
        status_msg.width = 0
        status_msg.height = 0
        status_msg.depth = 0
        status_msg.point_count = 0,
        status_msg.frame_id = "None"
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

                try:
                    msg = nepi_nav.convert_navposedata_dict2msg(navpose_dict)
                    self.node_if.publish_pub('navpose_pub', msg)
                    if self.last_pub_time is None:
                        self.last_pub_time = nepi_utils.get_time()
                    else:
                        cur_time = nepi_utils.get_time()
                        pub_time_sec = cur_time - self.last_pub_time
                        self.last_pub_time = cur_time
                        self.status_msg.last_pub_sec = pub_time_sec
                        self.status_msg.fps = float(1) / pub_time_sec
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish navpose data msg: " + str(e))
                    success = False

            if self.enable_gps_pub == True:

                # gps_fix pub
                try:
                    loc_msg = None
                    if navpose_dict['frame_alt'] == 'AMSL':
                        navpose_dict = nepi_nav.convert_navposedata_amsl2wgs84(navpose_dict)
                    msg = self.pubs_dict['loc']['msg']()
                    msg.latitude = navpose_dict['latitude']
                    msg.latitude = navpose_dict['longitude']
                    msg.latitude = navpose_dict['altitude']
                    self.node_if.publish_pub('gps_pub',msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish location data msg: " + str(e))
                    success = False


            if self.enable_pose_pub == True:
                #Create odom pub
                try:
                    pose_msg = None
                    if navpose_dict['frame_3d'] == 'NED':
                        navpose_dict = nepi_nav.convert_navposedata_ned2edu(navpose_dict)
                    
                    point_msg = Point()
                    point_msg.x = navpose_dict['x_m']
                    point_msg.y = navpose_dict['y_m']
                    point_msg.z = navpose_dict['z_m']

                    rpy = [navpose_dict['roll_deg'],navpose_dict['pitch_deg'],navpose_dict['yaw_deg']]
                    quad = nepi_nav.convert_rpy2quat(rpy)
                    quad_msg = Quaternion()
                    quad_msg.x = quad[0]
                    quad_msg.y = quad[1]
                    quad_msg.z = quad[2]
                    quad_msg.w = quad[3]
                    msg = self.pubs_dict['pose']['msg']()
                    msg.pose.pose.position = point_msg
                    msg.pose.pose.orientation = quad_msg
                    self.node_if.publish_pub('pose_pub',msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish location data msg: " + str(e))
                    success = False

            if self.enable_heading_pub == True:
                #Create heading pub
                try:
                    msg = navpose_dict['heading']
                    self.node_if.publish_pub('heading_pub',msg)
                except Exception as e:
                    self.msg_if.pub_warn("Failed to publish location data msg: " + str(e))
                    success = False
        return success


    def unsubsribe(self):
        self.ready = False
        if self.pc_pub is not None:
            self.msg_if.pub_info("Unsubsribing Image Publisher on topic: " + self.pc_pub_topic)
            self.pc_pub.unsubscribe()
        if self.status_pub is not None:
            self.msg_if.pub_info("Unsubsribing Status Publisher on topic: " + self.status_pub_topic)
            self.status_pub.unsubsribe()
        time.sleep(1)
        self.namespace = '~'
        self.pc_pub = None
        self.status_pub = None
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
        if self.status_pub is not None and not nepi_ros.is_shutdown():
            self.img_status_msg.publish(self.status_msg)


