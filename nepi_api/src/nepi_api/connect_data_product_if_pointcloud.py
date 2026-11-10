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
# Connect Pointcloud IF Class
#########################################
# Client-side connect class mirroring data_if.PointcloudIF. PointcloudIF is a
# standalone (non-BaseImageIF) data interface that publishes a PointCloud2 on
# its base namespace and a PointcloudStatus on <namespace>/status, so this
# connect class subclasses ConnectDataProductIF and supplies the PointcloudStatus
# status type. Everything the connect pattern shares is inherited; what stays
# here is what ConnectDataProductIF has no generic form of -- the PointcloudStatus
# field accessors, the cached pointcloud getter, and a _dataCb that converts the
# PointCloud2 to an Open3D cloud and records latency.


POINTCLOUD_CONNECT_ID = 'POINTCLOUD'
POINTCLOUD_CONNECT_STATUS_MSG = 'PointcloudStatus'
POINTCLOUD_CONNECT_STATUS_MSG_CLASS = PointcloudStatus
POINTCLOUD_CONNECT_DATA_MSG = PointCloud2
POINTCLOUD_CONNECT_NAME = 'pointcloud_connect'


class ConnectPointcloudIF(ConnectDataProductIF):

    # ADD Additional Connect Callback Functions

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = POINTCLOUD_CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
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

        connect_topic_controls_dict = {
            self.node_if_prefix + 'pointcloud_reset_controls': {
                'namespace': 'unknown',
                'topic': 'reset_controls',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_set_clip_enable': {
                'namespace': 'unknown',
                'topic': 'set_clip_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_set_clip_selection': {
                'namespace': 'unknown',
                'topic': 'set_clip_selection',
                'msg': String,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_set_range_clip_m': {
                'namespace': 'unknown',
                'topic': 'set_range_clip_m',
                'msg': RangeWindow,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_set_clip_bounding_box3d_topic': {
                'namespace': 'unknown',
                'topic': 'set_clip_bounding_box3d_topic',
                'msg': String,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_set_voxel_downsample_size': {
                'namespace': 'unknown',
                'topic': 'set_voxel_downsample_size',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_uniform_downsample_k_points': {
                'namespace': 'unknown',
                'topic': 'uniform_downsample_k_points',
                'msg': Int32,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_outlier_removal_num_neighbors': {
                'namespace': 'unknown',
                'topic': 'outlier_removal_num_neighbors',
                'msg': Int32,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_set_rotate_ratio': {
                'namespace': 'unknown',
                'topic': 'set_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_set_tilt_ratio': {
                'namespace': 'unknown',
                'topic': 'set_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'pointcloud_set_render_enable': {
                'namespace': 'unknown',
                'topic': 'set_render_enable',
                'msg': Bool,
                'qsize': 1,
            }
        }

        super().__init__(
                connect_id = POINTCLOUD_CONNECT_ID,
                connect_name = connect_name,
                namespace = namespace,
                auto_select_enabled = auto_select_enabled,
                connect_status_msg = POINTCLOUD_CONNECT_STATUS_MSG,
                connect_status_msg_class = POINTCLOUD_CONNECT_STATUS_MSG_CLASS,
                status_callback = status_callback,
                connect_data = connect_data,
                connect_data_msg = POINTCLOUD_CONNECT_DATA_MSG,
                data_callback = data_callback,
                filter_topic_list = filter_topic_list,
                connect_topic_controls_dict = connect_topic_controls_dict,
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
        if type(self) is ConnectPointcloudIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    def get_point_count(self):
        """Return the point count reported by the connected pointcloud source.

        Returns:
            int: The number of points in the pointcloud, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.point_count

    def get_has_rgb(self):
        """Return whether the connected pointcloud source provides RGB color data.

        Returns:
            bool: True if the pointcloud carries RGB data, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.has_rgb

    def get_has_intensity(self):
        """Return whether the connected pointcloud source provides intensity data.

        Returns:
            bool: True if the pointcloud carries intensity data, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.has_intensity

    def get_pointcloud_dict(self):
        """Retrieve and consume the latest captured pointcloud data dictionary.

        Thread-safe. Clears the stored data after retrieval so subsequent calls
        return None until a new frame arrives.

        Returns:
            dict: The pointcloud data dictionary containing the Open3D point cloud,
                point count, timestamps, and latency metrics, or None if no
                pointcloud is available.
        """
        self.data_lock.acquire()
        data_dict = copy.deepcopy(self.data_dict)
        self.data_dict = None
        self.data_lock.release()
        return data_dict


    #################
    ## Control Functions

    def reset_controls(self):
        """Reset all pointcloud process controls to their initialized values.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_reset_controls', Empty())

    def set_clip_enable(self, enable):
        """Enable or disable pointcloud clipping.

        Args:
            enable (bool): True to enable clipping, False to disable.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_set_clip_enable', Bool(enable))

    def set_clip_selection(self, selection):
        """Select which clipping mode is applied.

        Args:
            selection (str): Clip selection name, from the clip_options list in
                the pointcloud status message.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_set_clip_selection', String(selection))

    def set_range_clip_m(self, range_window_msg):
        """Set the clipping range window in meters.

        Args:
            range_window_msg (RangeWindow): Caller-built RangeWindow message
                carrying the minimum and maximum clip range in meters.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_set_range_clip_m', range_window_msg)

    def set_clip_bounding_box3d_topic(self, topic):
        """Set the topic supplying the 3D bounding box used for clipping.

        Args:
            topic (str): Namespace of the bounding box 3D topic.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_set_clip_bounding_box3d_topic', String(topic))

    def set_voxel_downsample_size(self, size_m):
        """Set the voxel downsample size.

        Args:
            size_m (float): Voxel edge length in meters. Zero disables voxel
                downsampling.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_set_voxel_downsample_size', Float32(size_m))

    def uniform_downsample_k_points(self, k_points):
        """Set the uniform downsample point count.

        Args:
            k_points (int): Number of points to keep. Zero disables uniform
                downsampling.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_uniform_downsample_k_points', Int32(k_points))

    def outlier_removal_num_neighbors(self, num_neighbors):
        """Set the outlier removal neighbor count.

        Args:
            num_neighbors (int): Neighbor count used for statistical outlier
                removal. Zero disables outlier removal.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_outlier_removal_num_neighbors', Int32(num_neighbors))

    def set_rotate_ratio(self, ratio):
        """Set the pointcloud render rotation ratio.

        Args:
            ratio (float): Rotation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_set_rotate_ratio', Float32(ratio))

    def set_tilt_ratio(self, ratio):
        """Set the pointcloud render tilt ratio.

        Args:
            ratio (float): Tilt as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_set_tilt_ratio', Float32(ratio))

    def set_render_enable(self, enable):
        """Enable or disable pointcloud render image publishing.

        Args:
            enable (bool): True to enable rendering, False to disable.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'pointcloud_set_render_enable', Bool(enable))


    ###############################
    # Class Private Methods
    ###############################

    # Overrides ConnectDataProductIF._dataCb, which flattens the msg with
    # convert_msg2dict. A PointCloud2 is converted to an Open3D cloud here and
    # returned with its point count, timestamps and get/process/got latency.
    def _dataCb(self,data_msg):

        current_time = nepi_sdk.get_msg_stamp()
        msg_stamp = data_msg.header.stamp
        get_latency = (current_time.to_sec() - msg_stamp.to_sec())

        start_time = nepi_sdk.get_time()



        ##############################
        ### Preprocess Pointcloud
        data = nepi_pc.rospc_to_o3dpc(data_msg)

        data_dict = dict()
        data_dict['namespace'] = self.selected_topic
        data_dict['pointcloud'] = data
        data_dict['navpose_dict'] = self.get_navpose_dict()
        data_dict['point_count'] = len(data.points) if data is not None else 0
        data_dict['timestamp'] = nepi_sdk.sec_from_msg_stamp(msg_stamp)
        data_dict['ros_pc_header'] = data_msg.header
        data_dict['ros_pc_stamp'] = msg_stamp
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
# Connect PointcloudImage IF Class
#########################################


POINTCLOUD_IMAGE_CONNECT_NAME = 'pointcloud_image_connect'


class ConnectPointcloudImageIF(ConnectBaseImageIF):
    """Connect class mirroring data_if.PointcloudImageIF.

    Adds nothing beyond the shared ConnectBaseImageIF image connect pattern; the
    connected 'pointcloud_image' data product is selected by namespace.
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = POINTCLOUD_IMAGE_CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
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
        if type(self) is ConnectPointcloudImageIF:
            self.start()
