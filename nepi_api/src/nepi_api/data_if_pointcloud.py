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

EXAMPLE_STATUS_DICT = {
    'pub_topic': 'None',
    'data_name_name': 'None',

    'publishing': False,
    'encoding': 'None',
    'has_rgb': False,
    'point_count': 0,
    'width': 0,
    'height': 0,
    'depth': 0,
    'frame_id': 'None',

    'get_latency_time':0,
    'pub_latency_time':0,
    'process_time':0
}

ENCODING_OPTIONS = ["mono8",'rgb8','bgr8','32FC1','passthrough']

class PointcloudIF:

    ready = False
    status_msg = PointcloudStatus()
    status_dict = None
    pc_pub_topic = "None"
    pc_pub = None
    status_pub_topic = "None"
    status_pub = None
    has_subscribers = False

    pub_namespace = '~'

    last_pub_time = None


    def __init__(self, data_name = 'pointcloud', encoding = 'bgr8', pub_namespace = None):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name + ": " + data_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################      
        if pub_namespace is not None:
            self.pub_namespace = pub_namespace


        # Create Data Pub
        success = False
        topic = os.path.join(self.pub_namespace,data_name)
        self.msg_if.pub_info("Creating Image Publisher on topic: " + topic)
        self.pc_pub = rospy.Publisher(topic, PointCloud2, queue_size=1, tcp_nodelay=True)

        # Create Status Pub
        topic = os.path.join(topic,'status')
        self.msg_if.pub_info("Creating Status Publisher on topic: " + topic)
        self.pc_status_pub = rospy.Publisher(topic, PointcloudStatus, queue_size=1, tcp_nodelay=True)

        # Initialize Status Msg.  Updated on each publish
        status_msg = PointcloudStatus()
        status_msg.data_name = data_name
        status_msg.publishing = False
        status_msg.encoding = encoding
        status_msg.has_rgb = False,
        status_msg.point_count = 0,
        status_msg.width = 0
        status_msg.height = 0
        status_msg.depth = 0
        status_msg.frame_id = "None"
        status_msg.get_latency_time
        status_msg.pub_latency_time
        status_msg.process_time
        self.status_msg = status_msg

        time.sleep(1)


        # Start subscribers check callback
        rospy.Timer(rospy.Duration(.1), self._subscribersCheckCb, oneshot = True)
        rospy.Timer(rospy.Duration(1), self._publishStatusCb, oneshot = False)
        time.sleep(1)

        #################################
        self.ready = True
        self.msg_if.pub_info("Initialization Complete")



    ###############################
    # Class Public  Methods
    ###############################
    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        success = False
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
        return self.status_dict


    def has_subscribers_check(self):
        return self.has_subscribers


    def publish_o3d_pc(self,o3d_pc, ros_timestamp = None, frame_id = 'sensor_frame'):
        if self.pc_pub is None:
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
        
        '''
        # Publish and Save Raw Image Data if Required  
        [height,width] = o3d_pc.shape[0:-1]
        self.status_msg.width = width
        self.status_msg.height = height
        '''



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
                self.pc_pub.publish(ros_pc)

            if self.last_pub_time is None:
                self.last_pub_time = nepi_utils.get_time()
            else:
                cur_time = nepi_utils.get_time()
                pub_time_sec = cur_time - self.last_pub_time
                self.last_pub_time = cur_time
                self.status_msg.last_pub_sec = pub_time_sec
                self.status_msg.fps = float(1) / pub_time_sec
        return True


    def unsubsribe(self):
        self.ready = False
        if self.pc_pub is not None:
            self.msg_if.pub_info("Unsubsribing Image Publisher on topic: " + self.pc_pub_topic)
            self.pc_pub.unsubscribe()
        if self.status_pub is not None:
            self.msg_if.pub_info("Unsubsribing Status Publisher on topic: " + self.status_pub_topic)
            self.status_pub.unsubsribe()
        time.sleep(1)
        self.pub_namespace = '~'
        self.pc_pub = None
        self.status_pub = None
        self.status_msg = None

    ###############################
    # Class Private Methods
    ###############################

    def _subscribersCheckCb(self,timer):
        self.has_subscribers = (self.pc_pub.get_num_connections() > 0)
        if self.has_subscribers == False:
            self.status_msg.publishing = False
        nepi_ros.sleep(1)
        rospy.Timer(rospy.Duration(.1), self._subscribersCheckCb, oneshot = True)

    def _publishStatusCb(self,timer):
        if self.status_pub is not None and not nepi_ros.is_shutdown():
            self.img_status_msg.publish(self.status_msg)