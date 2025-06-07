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
import copy
import cv2
import open3d as o3d
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image

from nepi_sdk_interfaces.msg import ImageStatus

from sensor_msgs.msg import PointCloud2
from nepi_sdk_interfaces.msg import PointcloudStatus

from nepi_sdk_interfaces.msg import NavPoseData, NavPoseStatus

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF



############################################################
'''
EXAMPLE_DATA_DICT = {   
    'namespace':  self.node_namespace,     
    'data': None,
    'width': 0,
    'height': 0,
    'timestamp': nepi_sdk.get_time(),
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
    def __init__(self, 
                namespace,
                preprocess_function = None,
                callback_function = None,
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
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################

    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
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
        current_time = nepi_sdk.get_msg_stamp()
        get_msg_stampstamp = data_msg.header.stamp
        latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
        data_dict['get_latency_time'] = latency
        #self.msg_if.pub_debug("Get Img Latency: {:.2f}".format(latency))

        start_time = nepi_sdk.get_time()   


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

            data_dict = dict()
            data_dict['namespace'] = self.namespace
            data_dict['data'] = data
            height, width = data.shape[:2]
            data_dict['width'] = width 
            data_dict['height'] = height 
            get_msg_stampstamp = data_msg.header.stamp
            data_dict['timestamp'] = nepi_sdk.sec_from_msg_stamp(get_msg_stampstamp)
            data_dict['ros_img_header'] = data_msg.header
            data_dict['ros_img_stamp'] = get_msg_stampstamp
            ##############################

            process_time = round( (nepi_sdk.get_time() - start_time) , 3)
            data_dict['process_time'] = process_time

            latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
            data_dict['got_latency_time'] = latency
            #self.msg_if.pub_debug("Img Pub Latency: {:.2f}".format(latency))

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
    'timestamp': nepi_sdk.get_time(),
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
    def __init__(self, 
                namespace,
                preprocess_function = None,
                callback_function = None,
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
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return self.status_connected

    def get_status_dict(self):
        img_status_dict = None
        if self.status_msg is not None:
            img_status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
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
        current_time = nepi_sdk.get_msg_stamp()
        get_msg_stampstamp = data_msg.header.stamp
        latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
        self.data_dict['get_latency_time'] = latency
        #self.msg_if.pub_debug("Get Pc Latency: {:.2f}".format(latency))

        start_time = nepi_sdk.get_time()   

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

            data_dict = dict()
            data_dict['namespace'] = self.namespace
            data_dict['data'] = data

            data_dict['has_rgb'] = data.has_colors() 
            data_dict['has_intensity'] = False # Need to add

            data_dict['point_count'] = data.point["colors"].shape[0]

            data_dict['width'] = 0  # Need to add
            data_dict['height'] = 0 # Need to add
            data_dict['depth'] = 0 # Need to add

            get_msg_stampstamp = data_msg.header.stamp
            data_dict['timestamp'] = nepi_sdk.sec_from_msg_stamp(get_msg_stampstamp)
            data_dict['ros_pc_header'] = data_msg.header
            data_dict['ros_pc_stamp'] = get_msg_stampstamp
            ##############################

            process_time = round( (nepi_sdk.get_time() - start_time) , 3)
            data_dict['process_time'] = process_time

            latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
            data_dict['got_latency_time'] = latency
            #self.msg_if.pub_debug("Img Pub Latency: {:.2f}".format(latency))

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


'''
EXAMPLE_NAVPOSE_DATA_DICT = {
    'frame_3d': 'ENU',
    'frame_altitude': 'WGS84',

    'geoid_height_meters': 0,

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
    'time_oreientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees in specified 3d frame
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,

    'has_location': True,
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'lat': 47.080909,
    'long': -120.8787889,

    'has_altitude': True,
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters in specified alt frame
    'altitude_m': 12.321,

    'has_depth': False,
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive meters
    'depth_m': 0.0
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


 
    #######################
    ### IF Initialization
    def __init__(self, 
                namespace,
                callback_function = None,
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

    
        self.callbackFunction = callback_function


        ##############################   
        ## Node Setup

        # Pubs Config Dict ####################
        self.SUBS_DICT = {
            'navpose_sub': {
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

        # Subs Config Dict ####################

        # Create Node Class ####################
        self.con_node_if = ConnectNodeClassIF(
                        subs_dict = self.SUBS_DICT,
                                            log_name_list = self.log_name_list,
                                            msg_if = self.msg_if
                                            )
  

        #self.con_node_if.wait_for_ready()
        nepi_sdk.wait()


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ###############################
    

    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready  

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.connected


    def check_status_connection(self):
        return self.status_connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        if self.con_node_if is not None:
            self.msg_if.pub_info("Waiting for status connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.status_connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.status_connected == False:
                self.msg_if.pub_info("Failed to connect to status msg", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Status Connected", log_name_list = self.log_name_list)
        return self.status_connected

    def get_status_dict(self):
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return self.status_dict


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
        #self.msg_if.pub_warn("Got pc for topic:  " + self.namespace)

        # Process ros navpose message
        current_time = nepi_sdk.get_msg_stamp()
        get_msg_stampstamp = data_msg.header.stamp
        latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
        self.data_dict['get_latency_time'] = latency
        #self.msg_if.pub_debug("Get Pc Latency: {:.2f}".format(latency))

        start_time = nepi_sdk.get_time()   

        get_data = True # (self.callbackFunction is not None or self.get_data == True)
        #self.msg_if.pub_warn("Got Pc with get_data: " + str(self.get_data) + " got_data: " + str(self.got_data))
        if get_data == True:
            self.get_data = False
           # self.msg_if.pub_warn("Got get_data flag. Processing pc for topic:  " + self.namespace)
            ##############################
            ### Preprocess NavPose
            
            data_dict = nepi_nav.convert_navposedata_msg2dict(data_mgs)


            process_time = round( (nepi_sdk.get_time() - start_time) , 3)
            data_dict['process_time'] = process_time

            latency = (current_time.to_sec() - get_msg_stampstamp.to_sec())
            data_dict['got_latency_time'] = latency
            #self.msg_if.pub_debug("Img Pub Latency: {:.2f}".format(latency))

            if self.callbackFunction is not None:
                self.callbackFunction(data_dict)

            self.data_dict_lock.acquire()
            self.data_dict = data_dict
            self.data_dict_lock.release()
            self.got_data = True
        self.connected = True


    def _statusCb(self,status_msg):      
        self.status_connected = True
        self.pc_status_msg = status_msg

