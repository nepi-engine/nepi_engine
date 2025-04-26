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
import cv2
import open3d as o3d
import threading

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image

from nepi_ros_interfaces.msg import ImageStatus

from sensor_msgs.msg import PointCloud2
from nepi_ros_interfaces.msg import PointcloudStatus

from nepi_ros_interfaces.msg import NavPoseData, NavPoseStatus

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF



############################################################
'''
EXAMPLE_DATA_DICT = {   
    'namespace':  self.node_namespace,     
    'data': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_ros.get_time(),
    'ros_img_header': Header(),
    'ros_img_stamp': Header().stamp,
    'get_latency_time': 0,
    'got_latency_time': 0,
    'process_time': 0 
}
'''

class ConnectImageIF:

    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    data_dict = None
    data_dict_lock = threading.Lock()

    get_data = False
    got_data = False



    #######################
    ### IF Initialization
    log_name = "ConnectImageIF"
    def __init__(self, 
                namespace,
                preprocess_function = None,
                callback_function = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name + ": " + img_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################    
        # Initialize Class Variables

        if namespace is None:
            namespace = self.node_namespace
        else:
            namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(namespace)

        self.preprocessFunction = preprocess_function
        self.callbackFunction = callback_function



        ##############################   
        ## Node Setup

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'img_sub': {
                'msg': Image,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'callback': self._dataCb
            },
            'status_sub': {
                'msg': ImageStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.con_node_if.wait_for_ready()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################

    #######################
    # Class Public Methods
    #######################


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

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.status_connected == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        return self.img_status_dict

    def set_get_img(self,state):
        self.get_data = state
        return True

    def read_get_got_states(self):
        return [self.get_data, self.got_data]

    def get_data_dict(self):
        self.data_dict_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_dict_lock.release()
        return data_dict

    def unregister(self):
        self._unsubscribeTopic()

    ###############################
    # Class Private Methods
    ###############################
   

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_info("Unregistering topic: " + str(self.namespace))
            try:
                self.connected = False 
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success


    def _dataCb(self,data_msg):      
        #self.msg_if.pub_warn("Got img for topic:  " + self.namespace)

        # Process ros image message
        current_time = nepi_ros.ros_time_now()
        ros_timestamp = data_msg.header.stamp
        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        data_dict['get_latency_time'] = latency
        #self.msg_if.pub_info("Get Img Latency: {:.2f}".format(latency))

        start_time = nepi_ros.get_time()   


        get_data = (self.callbackFunction is not None or self.get_data == True)
        #self.msg_if.pub_warn("Got Img with get_data: " + str(self.get_data) + " got_data: " + str(self.got_data))
        if get_data == True:
            self.get_data = False
           # self.msg_if.pub_warn("Got get_data flag. Processing img for topic:  " + self.namespace)
            ##############################
            ### Preprocess Image
            
            data = nepi_img.rosimg_to_cv2img(data_msg)

            if self.preprocessFunction is not None:
                try:
                    data = self.preprocessFunction(data)
                except Exception as e:
                    self.msg_if.pub_warn("Provided Image Preprocess Function failed:  " + str(e))

            data = self.preprocessFunction(data)
            data_dict = dict()
            data_dict['namespace'] = self.namespace
            data_dict['data'] = data
            height, width = data.shape[:2]
            data_dict['width'] = width 
            data_dict['height'] = height 
            ros_timestamp = data_msg.header.stamp
            data_dict['timestamp'] = nepi_ros.sec_from_ros_time(ros_timestamp)
            data_dict['ros_img_header'] = data_msg.header
            data_dict['ros_img_stamp'] = ros_timestamp
            ##############################

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            data_dict['process_time'] = process_time

            latency = (current_time.to_sec() - ros_timestamp.to_sec())
            data_dict['got_latency_time'] = latency
            #self.msg_if.pub_info("Img Pub Latency: {:.2f}".format(latency))

            if self.callbackFunction is not None:
                self.callbackFunction(data_dict)
            else:
                self.data_dict_lock.acquire()
                self.data_dict = data_dict
                self.data_dict_lock.release()
                self.got_data = True
        self.connected = True


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg



############################################################


'''
EXAMPLE_DATA_DICT = {      
    'namespace':  self.node_namespace, 
    'data': None,
    'width': 0,
    'height': 0,
    'depth': 0,
    'point_count': 0,
    'has_rgb': False,
    'timestamp': nepi_ros.get_time(),
    'ros_pc_header': Header(),
    'ros_pc_stamp': Header().stamp,
    'get_latency_time':0,
    'pub_latency_time':0,
    'process_time':0
}
'''

class ConnectPointcloudIF:
    """AI is creating summary for 

    :return: [description]
    :rtype: [type]
    """

    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False


    data_dict = None
    data_dict_lock = threading.Lock()

    get_data = False
    got_data = False


 
    #######################
    ### IF Initialization
    log_name = "ConnectPointcloudIF"
    def __init__(self, 
                namespace,
                preprocess_function = None,
                callback_function = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name + ": " + pc_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################    
        # Initialize Class Variables

        if namespace is None:
            namespace = self.node_namespace
        else:
            namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(namespace)

        self.preprocessFunction = preprocess_function
        self.callbackFunction = callback_function


        ##############################   
        ## Node Setup

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'pc_sub': {
                'msg': PointCloud2,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'callback': self._dataCb
            },
            'status_sub': {
                'msg': PointcloudStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.con_node_if.wait_for_ready()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################
    

    #######################
    # Class Public Methods
    #######################


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

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.status_connected == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        return self.img_status_dict

    def set_get_img(self,state):
        self.get_data = state
        return True

    def read_get_got_states(self):
        return [self.get_data, self.got_data]

    def get_data_dict(self):
        self.data_dict_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_dict_lock.release()
        return data_dict

    def unregister(self):
        self._unsubscribeTopic()

    ###############################
    # Class Private Methods
    ###############################
   

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering topic: " + str(self.namespace))
            try:
                self.connected = False 
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success



    def _dataCb(self,data_msg):      
        self.connected = True
        #self.msg_if.pub_warn("Got pc for topic:  " + self.namespace)

        # Process ros pointcloud message
        current_time = nepi_ros.ros_time_now()
        ros_timestamp = data_msg.header.stamp
        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.data_dict['get_latency_time'] = latency
        #self.msg_if.pub_info("Get Pc Latency: {:.2f}".format(latency))

        start_time = nepi_ros.get_time()   

        get_data = (self.callbackFunction is not None or self.get_data == True)
        #self.msg_if.pub_warn("Got Pc with get_data: " + str(self.get_data) + " got_data: " + str(self.got_data))
        if get_data == True:
            self.get_data = False
           # self.msg_if.pub_warn("Got get_data flag. Processing pc for topic:  " + self.namespace)
            ##############################
            ### Preprocess Pointcloud
            
            data = nepi_pc.rospc_to_o3dpc(data_msg, remove_nans=True)

            if self.pointcloudPreprocessFunction is not None:
                try:
                    data = self.pointcloudPreprocessFunction(data)
                except Exception as e:
                    self.msg_if.pub_warn("Provided Pointcloud Preprocess Function failed:  " + str(e))

            data = self.pointcloudPreprocessFunction(data)
            data_dict = dict()
            data_dict['namespace'] = self.namespace
            data_dict['data'] = data

            data_dict['has_rgb'] = data.has_colors() 
            data_dict['has_intensity'] = False # Need to add

            data_dict['point_count'] = data.point["colors"].shape[0]

            data_dict['width'] = 0  # Need to add
            data_dict['height'] = 0 # Need to add
            data_dict['depth'] = 0 # Need to add

            ros_timestamp = data_msg.header.stamp
            data_dict['timestamp'] = nepi_ros.sec_from_ros_time(ros_timestamp)
            data_dict['ros_pc_header'] = data_msg.header
            data_dict['ros_pc_stamp'] = ros_timestamp
            ##############################

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            data_dict['process_time'] = process_time

            latency = (current_time.to_sec() - ros_timestamp.to_sec())
            data_dict['got_latency_time'] = latency
            #self.msg_if.pub_info("Img Pub Latency: {:.2f}".format(latency))

            if self.callbackFunction is not None:
                self.callbackFunction(data_dict)
            else:
                self.data_dict_lock.acquire()
                self.data_dict = data_dict
                self.data_dict_lock.release()
                self.got_data = True
        self.connected = True


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.pc_status_msg = status_msg








############################################################










NAVPOSE_POS_FRAME_OPTIONS = ['ENU','NED']
NAVPOSE_ALT_FRAME_OPTIONS = ['AMSL','WGS84']
'''
EXAMPLE_DATA_DICT = {   
    'namespace': self.node_namespace,     
    'data': None,
    'timestamp': nepi_ros.get_time(),
    'ros_img_header': Header(),
    'ros_pc_stamp': Header().stamp,
    'get_latency_time':0,
    'pub_latency_time':0,
    'process_time':0
}
'''

class ConnectNavPoseIF:

    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    data_dict = None
    data_dict_lock = threading.Lock()

    get_data = False
    got_data = False



    #######################
    ### IF Initialization
    def __init__(self, 
                namespace,
                preprocess_function = None,
                callback_function = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name + ": " + img_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################    
        # Initialize Class Variables

        if namespace is None:
            namespace = self.node_namespace
        else:
            namespace = namespace
        self.namespace = nepi_ros.get_full_namespace(namespace)

        self.preprocessFunction = preprocess_function
        self.callbackFunction = callback_function


        ##############################   
        ## Node Setup

        # Subs Config Dict ####################
        self.SUBS_DICT = {
            'img_sub': {
                'msg': NavPoseData,
                'namespace': self.namespace,
                'topic': '',
                'qsize': 1,
                'callback': self._dataCb
            },
            'status_sub': {
                'msg': NavPoseStatus,
                'namespace': self.namespace,
                'topic': 'status',
                'qsize': 1,
                'callback': self._statusCb
            }
        }

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        self.con_node_if.wait_for_ready()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################

    #######################
    # Class Public Methods
    #######################


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

    def get_namespace(self):
        return self.namespace


    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.connected == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_ros.get_time()
            while self.status_connected == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_ros.convert_msg2dict(self.status_msg)
        return self.img_status_dict

    def set_get_img(self,state):
        self.get_data = state
        return True

    def read_get_got_states(self):
        return [self.get_data, self.got_data]

    def get_data_dict(self):
        self.data_dict_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_dict_lock.release()
        return data_dict

    def unregister(self):
        self._unsubscribeTopic()

    ###############################
    # Class Private Methods
    ###############################
   

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_info("Unregistering topic: " + str(self.namespace))
            try:
                self.connected = False 
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.connected = False 
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success


    def _dataCb(self,data_msg):      
        #self.msg_if.pub_warn("Got img for topic:  " + self.namespace)

        # Process ros image message
        current_time = nepi_ros.ros_time_now()
        ros_timestamp = data_msg.header.stamp
        latency = (current_time.to_sec() - ros_timestamp.to_sec())
        self.data_dict['get_latency_time'] = latency
        #self.msg_if.pub_info("Get Img Latency: {:.2f}".format(latency))

        start_time = nepi_ros.get_time()   


        get_data = (self.callbackFunction is not None or self.get_data == True)
        #self.msg_if.pub_warn("Got Img with get_data: " + str(self.get_data) + " got_data: " + str(self.got_data))
        if get_data == True:
            self.get_data = False
           # self.msg_if.pub_warn("Got get_data flag. Processing img for topic:  " + self.namespace)
            ##############################
            ### Preprocess Image
            
            data = nepi_nav.convert_navposedata_msg2dict(data_msg)

            if self.preprocessFunction is not None:
                try:
                    data = self.preprocessFunction(data)
                except Exception as e:
                    self.msg_if.pub_warn("Provided Image Preprocess Function failed:  " + str(e))

            data = self.preprocessFunction(data)
            data_dict = dict()
            data_dict['namespace'] = self.namespace
            data_dict['data'] = data
            ros_timestamp = data_msg.header.stamp
            data_dict['timestamp'] = nepi_ros.sec_from_ros_time(ros_timestamp)
            data_dict['ros_img_header'] = data_msg.header
            data_dict['ros_img_stamp'] = ros_timestamp
            ##############################

            process_time = round( (nepi_ros.get_time() - start_time) , 3)
            data_dict['process_time'] = process_time

            latency = (current_time.to_sec() - ros_timestamp.to_sec())
            data_dict['got_latency_time'] = latency
            #self.msg_if.pub_info("Img Pub Latency: {:.2f}".format(latency))

            if self.callbackFunction is not None:
                self.callbackFunction(data_dict)
            else:
                self.data_dict_lock.acquire()
                self.data_dict = data_dict
                self.data_dict_lock.release()
                self.got_data = True
        self.connected = True


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.status_msg = status_msg





