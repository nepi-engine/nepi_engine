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