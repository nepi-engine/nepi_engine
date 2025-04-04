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

from nepi_ros_interfaces.msg import NavPose, NavPoseData
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
    pub_namespace = '~'
    has_subscribers = False


    pubs_dict = {
        'nav':{
            'msg': NavPoseData,
            'name':  'navpose',
            'pub':  None
        },
        'loc':{
            'msg': NavSatFix,
            'name':  'gps_fix',
            'pub':  None
        },
        'pose':{
            'msg': Odometry,
            'name':  'odom',
            'pub':  None
        },
        'head':{
            'msg': Float64,
            'name':  'heading',
            'pub':  None
        }
    }

    status_msg = None

    last_pub_time = None

    #######################
    ### IF Initialization
    log_name = 'NavPoseIF'
    def __init__(self, data_name = 'navpose', pub_namespace = None):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        log_name = self.class_name
        self.msg_if = MsgIF(log_name = self.class_name + ": " + data_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################    

        if pub_namespace is not None:
            self.pub_namespace = pub_namespace
        
        # Creat NavPose Pub
        pub_dict = self.pubs_dict['nav']
        name = pub_dict['name']
        msg = pub_dict['msg']
        topic = os.path.join(self.pub_namespace,name)
        self.pubs_dict['nav']['pub'] = rospy.Publisher(topic, msg, queue_size = 1)

        '''
        # Create Status Pub
        topic = os.path.join(topic,'status')
        self.msg_if.pub_info("Creating Status Publisher on topic: " + topic)
        self.pc_status_pub = rospy.Publisher(topic, PointcloudStatus, queue_size=1, tcp_nodelay=True)
        '''


        #Create gps_fix pub
        self.has_loc = test_dict['has_location']
        if self.has_loc == True:
            pub_dict = self.pubs_dict['loc']
            name = pub_dict['name']
            msg = pub_dict['msg']
            topic = os.path.join(self.pub_namespace,name)
            self.pubs_dict['loc']['pub'] = rospy.Publisher(topic, msg, queue_size = 1)

        #Create odom pub
        self.has_pose = test_dict['has_position'] or test_dict['has_orientation']
        if self.has_pose == True:
            pub_dict = self.pubs_dict['pose']
            name = pub_dict['name']
            msg = pub_dict['msg']
            topic = os.path.join(self.pub_namespace,name)
            self.pubs_dict['pose']['pub'] = rospy.Publisher(topic, msg, queue_size = 1)

        #Create heading pub
        self.has_heading = test_dict['has_heading']
        if self.has_heading == True:
            pub_dict = self.pubs_dict['head']
            name = pub_dict['name']
            msg = pub_dict['msg']
            topic = os.path.join(self.pub_namespace,name)
            self.pubs_dict['head']['pub'] = rospy.Publisher(topic, msg, queue_size = 1)


        rospy.Timer(rospy.Duration(1), self._publishStatusCb, oneshot = False)
        time.sleep(1)

        self.ready = True

        #################################
        self.msg_if.pub_info("Initialization Complete")



    ###############################
    # Class Public Methods
    ###############################


    ################################
    # Location functions

    def check_ready(self):
        return self.ready  


    def get_namespace(self):
        return self.namespace
    

    def publish_navpose_dict(self, navpose_dict, ros_timestamp = None, frame_id = 'sensor_frame' ):
        success = self._publishDict(navpose_, ros_timestamp = os_timestamp, frame_id = frame_id)
        return success


    def unregister_navpose_if(self): 
        success = False
        if self.pubs_dict['nav']['pub'] is None:
            self.msg_if.pub_war("NavPose IF not running")
            success = True
        else:
            self.msg_if.pub_info("Killing NavPose IF pubs")
            for key in self.pubs_dict.keys():
                pub = self.pubs_dict[key]['pub']
                if pub is not None:
                    try:
                        pub.unregister()
                        success = True
                        self.pubs_dict[key]['pub'] = None
                    except:
                        pass
            time.sleep(1)
            self.ready = False
            self.pub_namespace = '~'
        return success




    ###############################
    # Class Private Methods
    ###############################
    def _navposeCapabilitiesHandler(self, _):
        return self.navpose_capabilities_report    


    # Update System Status
    def _publishDict(self,navpose_dict, ros_timestamp = None, frame_id = 'sensor_frame' ):      
        success = True
        if nav_dict is None:
            success = False
        else:
            # Pub NavPoseData
            if self.pubs_dict['nav']['pub'] is not None:

                if ros_timestamp == None:
                    ros_timestamp = nepi_ros.ros_time_now()




                try:
                    msg = nepi_nav.convert_navposedata_dict2msg(nav_dict)
                    self.pubs_dict['nav']['pub'].publish(msg)
                    if self.last_pub_time is None:
                        self.last_pub_time = nepi_utils.get_time()
                    else:
                        cur_time = nepi_utils.get_time()
                        pub_time_sec = cur_time - self.last_pub_time
                        self.last_pub_time = cur_time
                        self.status_msg.last_pub_sec = pub_time_sec
                        self.status_msg.fps = float(1) / pub_time_sec
                except Exception as e:
                    self.msg_if.pub_war("Failed to publish navpose data msg: " + str(e))
                    success = False

            # gps_fix pub
            if self.pubs_dict['loc']['pub'] is not None:
                try:
                    loc_msg = None
                    if nav_dict['frame_alt'] == 'AMSL':
                        nav_dict = nepi_nav.convert_navposedata_amsl2wgs84(nav_dict)
                    msg = self.pubs_dict['loc']['msg']()
                    msg.latitude = nav_dict['latitude']
                    msg.latitude = nav_dict['longitude']
                    msg.latitude = nav_dict['altitude']
                    self.pubs_dict['loc']['pub'].publish(msg)
                except Exception as e:
                    self.msg_if.pub_war("Failed to publish location data msg: " + str(e))
                    success = False

            #Create odom pub
            if  self.pubs_dict['pose']['pub'] is not None:
                try:
                    pose_msg = None
                    if nav_dict['frame_3d'] == 'NED':
                        nav_dict = nepi_nav.convert_navposedata_ned2edu(nav_dict)

                    
                    point_msg = Point()
                    point_msg.x = nav_dict['x_m']
                    point_msg.y = nav_dict['y_m']
                    point_msg.z = nav_dict['z_m']

                    rpy = [nav_dict['roll_deg'],nav_dict['pitch_deg'],nav_dict['yaw_deg']]
                    quad = nepi_nav.convert_rpy2quat(rpy)
                    quad_msg = Quaternion()
                    quad_msg.x = quad[0]
                    quad_msg.y = quad[1]
                    quad_msg.z = quad[2]
                    quad_msg.w = quad[3]
                    msg = self.pubs_dict['pose']['msg']()
                    msg.pose.pose.position = point_msg
                    msg.pose.pose.orientation = quad_msg
                    self.pubs_dict['pose']['pub'].publish(msg)
                except Exception as e:
                    self.msg_if.pub_war("Failed to publish location data msg: " + str(e))
                    success = False

            #Create heading pub
            if self.pubs_dict['head']['pub'] is not None:
                try:
                    msg = nav_dict['heading']
                    self.pubs_dict['head']['pub'].publish(msg)
                except Exception as e:
                    self.msg_if.pub_war("Failed to publish location data msg: " + str(e))
                    success = False
        return success
           

    def _subscribersCheckCb(self,timer):
        self.has_subscribers = (self.nav_pub.get_num_connections() > 0)
        nepi_ros.sleep(1)
        rospy.Timer(rospy.Duration(.1), self.subscribersCheckCb, oneshot = True)

    def _publishStatusCb(self,timer):
        self.img_status_msg.publish(self.status_msg)