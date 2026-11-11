#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import os
import time
import copy
import math
import threading
import numpy as np


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_pc

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Vector3
from nepi_interfaces.msg import SaveDataRate
from nepi_interfaces.msg import ColorBGR
from nepi_interfaces.msg import StringArray, UpdateBool, UpdateFloat
from nepi_interfaces.msg import RangeWindow, ImageMouseEvent
from nepi_interfaces.msg import ImageCrosshair, ImageTarget
from nepi_interfaces.msg import ImageStatus
from nepi_interfaces.msg import NavPose, NavPoseStatus
from nepi_interfaces.msg import DepthMapStatus
from nepi_interfaces.msg import PointcloudStatus

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF
from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.connect_data_product_if import ConnectDataProductIF
from nepi_api.connect_data_product_if_image import ConnectBaseImageIF




#########################################
# Connect DepthMap IF Class
#########################################
# Client-side connect class mirroring data_if.DepthMapIF. DepthMapIF is a
# standalone (non-BaseImageIF) data interface that publishes its depth map as an
# Image on its base namespace and a DepthMapStatus on <namespace>/status, so
# this connect class subclasses ConnectDataProductIF and supplies the DepthMapStatus
# status type. Everything the connect pattern shares is inherited; what stays
# here is what ConnectDataProductIF has no generic form of -- the DepthMapStatus field
# accessors, the cached depth map getter, and a _dataCb that decodes the Image
# to cv2 and records latency rather than flattening the msg to a dict.


DEPTH_MAP_CONNECT_ID = 'CONNECT_DEPTH_MAP'
DEPTH_MAP_CONNECT_STATUS_MSG = 'DepthMapStatus'
DEPTH_MAP_CONNECT_STATUS_MSG_CLASS = DepthMapStatus
DEPTH_MAP_CONNECT_DATA_MSG = Image
DEPTH_MAP_CONNECT_NAME = 'connect_depth_map'


class ConnectDepthMapIF(ConnectDataProductIF):

    # ADD Additional Connect Callback Functions

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = DEPTH_MAP_CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
                selected_topic = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  

        
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_info("Starting " + str(self.class_name) + " Initialization Processes")

        self.connect_name = connect_name
        if namespace is None:
            namespace = self.node_namespace
        namespace = namespace.replace('/' + self.connect_name, '')
        self.namespace = nepi_sdk.create_namespace(namespace,self.connect_name)
        self.node_if_prefix = self.namespace.replace(self.node_namespace + '/','') + '/'

        super().__init__(
                connect_id = DEPTH_MAP_CONNECT_ID,
                connect_name = connect_name,
                namespace = namespace,
                auto_select_enabled = auto_select_enabled,
                selected_topic = selected_topic,
                connect_status_msg = DEPTH_MAP_CONNECT_STATUS_MSG,
                connect_status_msg_class = DEPTH_MAP_CONNECT_STATUS_MSG_CLASS,
                status_callback = status_callback,
                connect_data = connect_data,
                connect_data_msg = DEPTH_MAP_CONNECT_DATA_MSG,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                connect_topic_controls_dict = None,
                has_navpose = True,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is ConnectDepthMapIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    def get_encoding(self):
        """Return the depth map encoding reported by the connected data source.

        Returns:
            str: The encoding string (e.g. '32FC1'), or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.encoding

    def get_image_size_px(self):
        """Return the depth map dimensions in pixels reported by the connected data source.

        Returns:
            list: A two-element list [width_px, height_px], or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.width_px, self.status_msg.height_px]

    def get_depth_map_dict(self):
        """Retrieve and consume the latest captured depth map data dictionary.

        Thread-safe. Clears the stored data after retrieval so subsequent calls
        return None until a new frame arrives.

        Returns:
            dict: The depth map data dictionary containing the cv2 depth map,
                dimensions, timestamps, and latency metrics, or None if no depth
                map is available.
        """
        self.data_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_lock.release()
        return data_dict


    ###############################
    # Class Private Methods
    ###############################

    # Overrides ConnectDataProductIF._dataCb, which flattens the msg with
    # convert_msg2dict. A depth map is an Image carrying float distance data, so
    # it is decoded to cv2 here and returned with its dimensions, timestamps and
    # get/process/got latency.
    def _dataCb(self,data_msg):


        current_time = nepi_sdk.get_msg_stamp()
        msg_stamp = data_msg.header.stamp
        get_latency = (current_time.to_sec() - msg_stamp.to_sec())

        start_time = nepi_sdk.get_time()

        ##############################
        ### Preprocess Depth Map
        data = nepi_img.rosimg_to_cv2img(data_msg)


        data_dict = dict()
        data_dict['namespace'] = self.selected_topic
        data_dict['depthmap'] = data
        data_dict['navpose_dict'] = self.get_navpose_dict()
        height, width = data.shape[:2]
        data_dict['width'] = width
        data_dict['height'] = height
        data_dict['timestamp'] = nepi_sdk.sec_from_msg_stamp(msg_stamp)
        data_dict['ros_img_header'] = data_msg.header
        data_dict['ros_img_stamp'] = msg_stamp
        data_dict['get_latency_time'] = get_latency
        ##############################

        process_time = round( (nepi_sdk.get_time() - start_time) , 3)
        data_dict['process_time'] = process_time

        got_latency = (nepi_sdk.get_msg_stamp().to_sec() - msg_stamp.to_sec())
        data_dict['got_latency_time'] = got_latency

        if self.data_callback is not None:
            self.data_callback(data_dict)

        self.data_lock.acquire()
        self.data_dict = data_dict
        self.data_lock.release()




#########################################
# Connect DepthMapImage IF Class
#########################################


DEPTH_MAP_IMAGE_CONNECT_NAME = 'depth_map_image_connect'


class ConnectDepthMapImageIF(ConnectBaseImageIF):
    """Connect class mirroring data_if.DepthMapImageIF.

    Adds nothing beyond the shared ConnectBaseImageIF image connect pattern; the
    connected 'depth_map_image' data product is selected by namespace.
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = DEPTH_MAP_IMAGE_CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
                selected_topic = None,
                status_callback = None,
                connect_data = True,
                data_callback = None,
                filter_topic_list = [],
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  

        
        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_info("Starting " + str(self.class_name) + " Initialization Processes")

        self.namespace = nepi_sdk.create_namespace(self.node_namespace,connect_name)
        self.node_if_prefix = self.namespace.replace(self.node_namespace + '/','') + '/'

        super().__init__(
                connect_name = connect_name,
                namespace = namespace,
                auto_select_enabled = auto_select_enabled,
                selected_topic = selected_topic,
                status_callback = status_callback,
                connect_data = connect_data,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = msg_if,
                node_if = node_if
                )
        ####  IF INIT SETUP ####
        ###############################

        ##############################
        # Start NodeIF Processes
        if type(self) is ConnectDepthMapImageIF:
            self.start()



