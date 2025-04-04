#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_msg
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc


import rospy
import os
import time
import copy
import cv2
import open3d as o3d
import threading



from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import PointlCoud2

from nepi_ros_interfaces.msg import PointcloudStatus


#You must use the classes pc_dict_lock.aquire() and pc_dict_lock.release() thread save functions 
#When accessing the classes pc_dict


EXAMPLE_PC_INFO_DICT = {
    'has_mmap': False,
    'mmap_id': '',
    'mmap_info_dict': dict(),
    'depth_map_topic': None,
    'pointcloud_img_topic': None,
    'get_latency_time':0,
    'pub_latency_time':0,
    'process_time':0,
}

EXAMPLE_PC_DICT = {       
    'o3d_pc': None,
    'width': 0,
    'height': 0,
    'depth': 0,
    'point_count': 0,
    'has_rgb': False,
    'timestamp': nepi_ros.get_time(),
    'ros_pc_topic': 'None',
    'ros_pc_header': Header(),
    'ros_pc_stamp': Header().stamp,
}




class ConnectPointcloudIF:
    """AI is creating summary for 

    :return: [description]
    :rtype: [type]
    """
 
    #######################
    ### IF Initialization
    log_name = "ConnectPointcloudIF"
    def __init__(self, 
                getPointcloudCallbackFunction = False,
                pointcloudPreprocessFunction = None
                ):
        ####  IF INIT SETUP ####
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Starting Initialization Processes")
        ##############################  
        self.getPointcloudCallbackFunction = getPointcloudCallbackFunction 
        self.pointcloudPreprocessFunction = pointcloudPreprocessFunction

        self.connected = False
        self.status_connected = False
        self.pc_sub = None
        self.pc_subscribed = False
        self.pc_topic = None
        self.pc_status_msg = None
        self.pc_info_dict = None
        self.pc_dict = None
        self.pc_dict_lock = threading.Lock()
        self.get_pc = False
        self.got_pc = False

        #################################
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Initialization Complete")
    

    #######################
    # Class Public Methods
    #######################

    def connect_pointcloud_topic(self,pc_topic, timeout = 5):
        """AI is creating summary for connect_pointcloud_topic

        :param pc_topic: [description]
        :type pc_topic: [type]
        :param timeout: [description], defaults to 5
        :type timeout: int, optional
        :return: [description]
        :rtype: [type]
        """
        success = False
        if pc_topic is None or pc_topic == "None":
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Can't subscribe to None topic")
        else:
            # Try to find img topic
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for pointcloud on topic: " + str(pc_topic))
            found_topic = nepi_ros.wait_for_topic(pc_topic, timeout = timeout)
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Found topic: " + found_topic)
            if found_topic != "":
                # Subscribe to topic
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Subscribing to pointcloud topic: " + pc_topic)
                pc_subscribed = self._subscribePcTopic(pc_topic)
                self.pc_subscribed = pc_subscribed
                if pc_subscribed == True:
                    # Wait for pointcloud
                    nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for pointcloud")
                    connected = False
                    timer = 0
                    time_start = nepi_ros.get_time()
                    while connected == False and timer < timeout and not rospy.is_shutdown():
                        nepi_ros.sleep(.2)
                        connected = self.connected
                        timer = nepi_ros.get_time() - time_start
                    
                    if connected == True:
                        success = True
                        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Pointcloud Connected")
                    else:
                        self._unsubscribePcTopic()
                        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
                else:
                    #################################
                    nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Subscribe")
            else:
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to find pointcloud topic: " + pc_topic)
        return success


    def disconnect_pointcloud_topic(self):

        success = False
        if self.pc_subscribed == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Already unsubsribed")
        else:
            #################################
            ## Subscribe to pointcloud if requested
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Unsubscribing to pointcloud topic: " + str(self.pc_topic))
            pc_unsubscribed = self._unsubscribePcTopic()

            if pc_unsubscribed == True:
                success = True
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Unsubsribe Succeeded")
            else:
                #################################
                nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Unsubscribe")

    def get_pc_topic(self):
        return self.pc_topic

    def get_info_dict(self):
        return self.pc_info_dict

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to Connect")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Connected")
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        success = False
        nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Waiting for status connection")
        timer = 0
        time_start = nepi_ros.get_time()
        while self.status_connected == False and timer < timeout and not nepi_ros.is_shutdown():
            nepi_ros.sleep(.1)
            timer = nepi_ros.get_time() - time_start
        if self.status_connected == False:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Failed to connect to status msg")
        else:
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Status Connected")
        return self.status_connected

    def get_status_dict(self):
        pc_status_dict = None
        if self.status_msg is not None:
            pc_status_dict = nepi_ros.convert_msg2dict(self.pc_status_msg)
        return self.pc_status_dict



    

    def read_get_got_pc_states(self):
        return self.get_pc, self.got_pc

    def set_get_pc(self,state):
        self.get_pc = state
        return True

    def get_pc_dict(self):
        self.pc_dict_lock.acquire()
        pc_dict = copy.deepcopy(self.pc_dict)
        self.pc_dict = None
        self.pc_dict_lock.release()
        return pc_dict


    ###############################
    # Class Private Methods
    ###############################

    def _subscribePcTopic(self,pc_topic):
            if pc_topic == "None" or pc_topic == "":
                return False
            if self.pc_sub is not None:
                success = self._unsubscribePcTopic(pc_topic)

            self.connected = False 
            # Create img info dict
            self.pc_info_dict = dict()  
            self.pc_info_dict['has_mmap'] = False
            self.pc_info_dict['mmap_id'] = ""
            self.pc_info_dict['mmap_info_dict'] = dict()

            self.pc_info_dict['depth_map_topic'] = nepi_pc.get_pc_depth_map_topic(pc_topic)
            self.pc_info_dict['pointcloud_img_topic'] = nepi_pc.get_pc_pointcloud_img_topic(pc_topic)

            self.pc_info_dict['get_latency_time'] = 0
            self.pc_info_dict['pub_latency_time'] = 0
            self.pc_info_dict['process_time'] = 0 

          
            nepi_msg.publishMsgInfo(self,'Subsribing to pointcloud topic: ' + pc_topic)
            self.pc_sub = rospy.Subscriber(pc_topic, PointlCoud2, self._pointcloudCb, queue_size=1, callback_args=(pc_topic))
            self.pc_topic = pc_topic
            # Setup Pointcloud Status Topic Subscriber in case there is one
            self.status_connected = False
            self.status_msg = None
            status_topic = os.path.join(pc_topic,'status')
            nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Subscribing to pointcloud status topic: " + status_topic)
            pc_subscribed = rospy.Subscriber(status_topic, PointcloudStatus, self._pointcloudStatusCb, queue_size=1, callback_args=(pc_topic))
            time.sleep(1)
  
            return True
        

    def _unsubscribePcTopic(self):
        success = False
        self.connected = False
        if self.pc_sub is not None:
            nepi_msg.publishMsgWarn(self,'Unregistering topic: ' + str(self.pc_topic))
            try:
                self.connected = False 
                self.pc_sub.unregister()
                time.sleep(1)
                self.pc_topic = None
                self.pc_subscribed = False
                self.connected = False 
                self.pc_sub = None
                self.pc_dict = None
                success = True
            except Exception as e:
                nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Failed to unregister pc_sub:  " + str(e))
        return success


    def _pointcloudCb(self,pointcloud_msg, args):      
        self.connected = True
        pc_topic = args
        #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got img for topic:  " + pc_topic)

        # Process ros pointcloud message
        current_time = nepi_ros.ros_time_now()
        ros_timestamp = pointcloud_msg.header.stamp
        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.pc_info_dict['get_latency_time'] = latency
        #nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Get Pc Latency: {:.2f}".format(latency))

        start_time = nepi_ros.get_time()   


        get_pointcloud = (self.getPointcloudCallbackFunction is not None or self.get_pc == True)
        #nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Got Pc with get_pc: " + str(self.get_pc) + " got_pc: " + str(self.got_pc))
        if get_pointcloud == True:
            self.get_pc = False
           # nepi_msg.publishMsgWarn(self,":" + self.log_name + ":Got get_pc flag. Processing img for topic:  " + pc_topic)
            ##############################
            ### Preprocess Pointcloud
            
            o3d_pc = nepi_pc.rospc_to_o3dpc(pointcloud_msg, remove_nans=True)

            if self.pointcloudPreprocessFunction is not None:
                try:
                    o3d_pc = self.pointcloudPreprocessFunction(o3d_pc)
                except Exception as e:
                    nepi_msg.publishMsgWarn(self,":" + self.log_name + ": Provided Pointcloud Preprocess Function failed:  " + str(e))

            o3d_pc = self.pointcloudPreprocessFunction(o3d_pc)
            pc_dict = dict()
            pc_dict['o3d_pc'] = o3d_pc

            pc_dict['width'] = 0 
            pc_dict['height'] = 0
            pc_dict['depth'] = 0
            pc_dict['point_count'] = o3d_pc.point["colors"].shape[0]
            pc_dict['has_rgb'] = o3d_pc.has_colors() 

            ros_timestamp = pointcloud_msg.header.stamp
            pc_dict['timestamp'] = nepi_ros.sec_from_ros_time(ros_timestamp)
            pc_dict['ros_pc_topic'] = pc_topic
            pc_dict['ros_pc_header'] = pointcloud_msg.header
            pc_dict['ros_pc_stamp'] = ros_timestamp
            ##############################

            if self.getPointcloudCallbackFunction is not None:
                self.getPointcloudCallbackFunction(pc_dict)
            else:
                self.pc_dict_lock.acquire()
                self.pc_dict = pc_dict
                self.pc_dict_lock.release()
                self.got_pc = True

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            self.pc_info_dict['process_time'] = process_time

        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.pc_info_dict['pub_latency_time'] = latency
        #nepi_msg.publishMsgInfo(self,":" + self.log_name + ": Pc Pub Latency: {:.2f}".format(latency))


    def _pointcloudStatusCb(self,status_msg, args):      
        self.status_connected = True
        pc_topic = args
        self.pc_status_msg = status_msg