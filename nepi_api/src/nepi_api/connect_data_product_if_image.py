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

from nepi_api.connect_data_product_if import ConnectDataProductIF





#########################################
# Connect BaseImage IF Class
#########################################
# Client-side connect class mirroring data_if.BaseImageIF, the shared parent of
# the image data-interface family. ConnectDataProductIF already implements the full
# image connect pattern (ImageStatus status + Image data), so ConnectBaseImageIF
# subclasses it and supplies only the image msg types. Every data_if.BaseImageIF
# descendant publishes the same ImageStatus/Image pair, so the image connect
# subclasses below differ from this one only in the connect name they register
# under -- and that name must differ, because it becomes the connect namespace.


BASE_IMAGE_CONNECT_ID = 'IMAGE'
BASE_IMAGE_CONNECT_STATUS_MSG = 'ImageStatus'
BASE_IMAGE_CONNECT_STATUS_MSG_CLASS = ImageStatus
BASE_IMAGE_CONNECT_DATA_MSG = Image
BASE_IMAGE_CONNECT_NAME = 'base_image_connect'


class ConnectBaseImageIF(ConnectDataProductIF):
    """Shared parent connect class for the image data-interface family.

    Mirrors data_if.BaseImageIF. Supplies the ImageStatus/Image msg types to
    ConnectDataProductIF, which provides the complete connect implementation
    (status/ready/connection handling, save/config methods, and the private ROS
    callbacks).
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = BASE_IMAGE_CONNECT_NAME,
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

        # Controls Config Dict ####################
        # Mirrors the data_if.BaseImageIF subscriber set. Only the subscribers
        # advertised on the data product namespace appear here -- that namespace is
        # what the connect side's selected_topic resolves to. The 'unknown' namespace
        # is the sentinel ConnectDataProductIF replaces with the connected topic at subscribe
        # time. Every image connect subclass inherits this set unchanged.
        connect_topic_controls_dict = {

            # Reset commands
            self.node_if_prefix + 'image_reset': {
                'namespace': 'unknown',
                'topic': 'reset',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_reset_filters': {
                'namespace': 'unknown',
                'topic': 'reset_filters',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_reset_overlays': {
                'namespace': 'unknown',
                'topic': 'reset_overlays',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_reset_settings': {
                'namespace': 'unknown',
                'topic': 'reset_settings',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_reset_renders': {
                'namespace': 'unknown',
                'topic': 'reset_renders',
                'msg': Empty,
                'qsize': 1,
            },

            # 3D render controls
            self.node_if_prefix + 'image_render_3d_controls': {
                'namespace': 'unknown',
                'topic': 'render_3d_controls',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_reset_render_3d_controls': {
                'namespace': 'unknown',
                'topic': 'reset_render_3d_controls',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_reset_render_3d_position': {
                'namespace': 'unknown',
                'topic': 'reset_render_3d_position',
                'msg': Empty,
                'qsize': 1,
            },

            # Mouse event injection
            self.node_if_prefix + 'image_mouse_event': {
                'namespace': 'unknown',
                'topic': 'mouse_event',
                'msg': ImageMouseEvent,
                'qsize': 1,
            },

            # Overlay text controls
            self.node_if_prefix + 'image_set_overlay_text_enable': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_click_text_enable': {
                'namespace': 'unknown',
                'topic': 'click_text_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_size_ratio': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_vert_ratio': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_vert_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_horz_ratio': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_horz_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_transparency_ratio': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_color_rgb': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_source_name': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_source_name',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_date_time': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_date_time',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_nav': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_nav',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_pose': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_pose',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_add_overlay_text': {
                'namespace': 'unknown',
                'topic': 'add_overlay_text',
                'msg': String,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_overlay_text_list': {
                'namespace': 'unknown',
                'topic': 'set_overlay_text_list',
                'msg': StringArray,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_clear_overlay_text_list': {
                'namespace': 'unknown',
                'topic': 'clear_overlay_text_list',
                'msg': Empty,
                'qsize': 1,
            },

            # Crosshair overlay controls
            self.node_if_prefix + 'image_crosshairs_enable': {
                'namespace': 'unknown',
                'topic': 'crosshairs_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_crosshairs_size_ratio': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_crosshairs_thickness_ratio': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_thickness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_crosshairs_text_ratio': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_text_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_crosshairs_transparency_ratio': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_crosshairs_color_rgb': {
                'namespace': 'unknown',
                'topic': 'set_crosshairs_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_overlay_crosshair_names': {
                'namespace': 'unknown',
                'topic': 'overlay_crosshair_names',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_overlay_crosshair_pixels': {
                'namespace': 'unknown',
                'topic': 'overlay_crosshair_pixels',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_overlay_crosshair_degrees': {
                'namespace': 'unknown',
                'topic': 'overlay_crosshair_degrees',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_overlay_crosshair_text': {
                'namespace': 'unknown',
                'topic': 'overlay_crosshair_text',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_click_crosshair_enable': {
                'namespace': 'unknown',
                'topic': 'click_crosshair_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_add_crosshair_pixel': {
                'namespace': 'unknown',
                'topic': 'add_crosshair_pixel',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_add_crosshair_ratios': {
                'namespace': 'unknown',
                'topic': 'add_crosshair_ratios',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_add_crosshair_degree_offsets': {
                'namespace': 'unknown',
                'topic': 'add_crosshair_degree_offsets',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_remove_crosshair': {
                'namespace': 'unknown',
                'topic': 'remove_crosshair',
                'msg': String,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_clear_crosshairs': {
                'namespace': 'unknown',
                'topic': 'clear_crosshairs',
                'msg': Empty,
                'qsize': 1,
            },

            # Target overlay controls
            self.node_if_prefix + 'image_targets_enable': {
                'namespace': 'unknown',
                'topic': 'targets_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_targets_size_ratio': {
                'namespace': 'unknown',
                'topic': 'set_targets_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_targets_thickness_ratio': {
                'namespace': 'unknown',
                'topic': 'set_targets_thickness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_targets_text_ratio': {
                'namespace': 'unknown',
                'topic': 'set_targets_text_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_targets_transparency_ratio': {
                'namespace': 'unknown',
                'topic': 'set_targets_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_targets_color_rgb': {
                'namespace': 'unknown',
                'topic': 'set_targets_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_overlay_target_names': {
                'namespace': 'unknown',
                'topic': 'overlay_target_names',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_overlay_target_pixels': {
                'namespace': 'unknown',
                'topic': 'overlay_target_pixels',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_overlay_target_degrees': {
                'namespace': 'unknown',
                'topic': 'overlay_target_degrees',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_overlay_target_text': {
                'namespace': 'unknown',
                'topic': 'overlay_target_text',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_click_target_enable': {
                'namespace': 'unknown',
                'topic': 'click_target_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_add_target_pixel': {
                'namespace': 'unknown',
                'topic': 'add_target_pixel',
                'msg': ImageTarget,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_add_target_ratios': {
                'namespace': 'unknown',
                'topic': 'add_target_ratios',
                'msg': ImageTarget,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_add_target_degree_offsets': {
                'namespace': 'unknown',
                'topic': 'add_target_degree_offsets',
                'msg': ImageTarget,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_remove_target': {
                'namespace': 'unknown',
                'topic': 'remove_target',
                'msg': String,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_clear_targets': {
                'namespace': 'unknown',
                'topic': 'clear_targets',
                'msg': Empty,
                'qsize': 1,
            },

            # Aspect and stream controls
            self.node_if_prefix + 'image_set_aspect_adjust_enable': {
                'namespace': 'unknown',
                'topic': 'set_aspect_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_aspect_adjust_ratio': {
                'namespace': 'unknown',
                'topic': 'set_aspect_adjust_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_aspect_adjust_by_ratio': {
                'namespace': 'unknown',
                'topic': 'set_aspect_adjust_by_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_stream_compression_enable': {
                'namespace': 'unknown',
                'topic': 'set_stream_compression_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_stream_compression_ratio': {
                'namespace': 'unknown',
                'topic': 'set_stream_compression_ratio',
                'msg': Float32,
                'qsize': 1,
            },

            # Live adjustment controls
            self.node_if_prefix + 'image_set_live_adjust_enable': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_live_adjust_rotate_ratio': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_live_adjust_rotate_deg': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_live_adjust_x_ratio': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_live_adjust_x_pixel': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_live_adjust_x_deg': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_live_adjust_y_ratio': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_live_adjust_y_pixel': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_live_adjust_y_deg': {
                'namespace': 'unknown',
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 1,
            },

            # Capability-gated controls. data_if.BaseImageIF advertises these only when

            # the matching caps_dict flag is set, and the connect side cannot know the

            # server's configuration, so they are always registered.
            self.node_if_prefix + 'image_set_resolution_ratio': {
                'namespace': 'unknown',
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_auto_adjust_enable': {
                'namespace': 'unknown',
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_auto_adjust_ratio': {
                'namespace': 'unknown',
                'topic': 'set_auto_adjust_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_brightness_ratio': {
                'namespace': 'unknown',
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_contrast_ratio': {
                'namespace': 'unknown',
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_threshold_ratio': {
                'namespace': 'unknown',
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_rotate_2d': {
                'namespace': 'unknown',
                'topic': 'rotate_2d',
                'msg': Empty,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_rotate_2d_deg': {
                'namespace': 'unknown',
                'topic': 'set_rotate_2d_deg',
                'msg': Int32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_rotate_2d_swap_box': {
                'namespace': 'unknown',
                'topic': 'set_rotate_2d_swap_box',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_flip_horz': {
                'namespace': 'unknown',
                'topic': 'set_flip_horz',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_flip_vert': {
                'namespace': 'unknown',
                'topic': 'set_flip_vert',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_range_ratios': {
                'namespace': 'unknown',
                'topic': 'set_range_ratios',
                'msg': RangeWindow,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_zoom_ratio': {
                'namespace': 'unknown',
                'topic': 'set_zoom_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_pan_x_ratio': {
                'namespace': 'unknown',
                'topic': 'set_pan_x_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_pan_y_ratio': {
                'namespace': 'unknown',
                'topic': 'set_pan_y_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_zoom_3d_ratio': {
                'namespace': 'unknown',
                'topic': 'set_zoom_3d_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_rotate_3d_ratio': {
                'namespace': 'unknown',
                'topic': 'set_rotate_3d_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_tilt_3d_ratio': {
                'namespace': 'unknown',
                'topic': 'set_tilt_3d_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_filter_enable': {
                'namespace': 'unknown',
                'topic': 'set_filter_enable',
                'msg': UpdateBool,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_filter_ratio': {
                'namespace': 'unknown',
                'topic': 'set_filter_ratio',
                'msg': UpdateFloat,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_camera_fov': {
                'namespace': 'unknown',
                'topic': 'set_camera_fov',
                'msg': Int32,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_camera_view': {
                'namespace': 'unknown',
                'topic': 'set_camera_view',
                'msg': Vector3,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_camera_position': {
                'namespace': 'unknown',
                'topic': 'set_camera_position',
                'msg': Vector3,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_camera_rotation': {
                'namespace': 'unknown',
                'topic': 'set_camera_rotation',
                'msg': Vector3,
                'qsize': 1,
            },
            self.node_if_prefix + 'image_set_white_bg_enable': {
                'namespace': 'unknown',
                'topic': 'set_white_bg_enable',
                'msg': Bool,
                'qsize': 1,
            }
        }

        super().__init__(
                connect_id = BASE_IMAGE_CONNECT_ID,
                connect_name = connect_name,
                namespace = namespace,
                auto_select_enabled = auto_select_enabled,
                connect_status_msg = BASE_IMAGE_CONNECT_STATUS_MSG,
                connect_status_msg_class = BASE_IMAGE_CONNECT_STATUS_MSG_CLASS,
                status_callback = status_callback,
                connect_data = connect_data,
                connect_data_msg = BASE_IMAGE_CONNECT_DATA_MSG,
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
        if type(self) is ConnectBaseImageIF:
            self.start()


    #######################
    # Class Public Methods
    #######################

    #################
    ## Control Functions

    # Reset commands

    def reset(self):
        """Publish a reset command, restoring all image controls to their initialized state.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_reset', Empty())

    def reset_filters(self):
        """Publish a reset command for the image filter settings.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_reset_filters', Empty())

    def reset_overlays(self):
        """Publish a reset command for the image overlay settings.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_reset_overlays', Empty())

    def reset_settings(self):
        """Publish a reset command for the image driver settings.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_reset_settings', Empty())

    def reset_renders(self):
        """Publish a reset command for the image render settings.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_reset_renders', Empty())

    # 3D render controls

    def render_3d_controls(self, enable):
        """Enable or disable the 3D render controls on the connected image source.

        Args:
            enable (bool): True to enable the 3D render controls, False to disable them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_render_3d_controls', Bool(enable))

    def reset_render_3d_controls(self):
        """Publish a reset command for the 3D render controls.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_reset_render_3d_controls', Empty())

    def reset_render_3d_position(self):
        """Publish a reset command for the 3D render camera position.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_reset_render_3d_position', Empty())

    # Mouse event injection

    def mouse_event(self, mouse_event_msg):
        """Publish a mouse event against the connected image source.

        Args:
            mouse_event_msg (ImageMouseEvent): A fully-populated ImageMouseEvent msg describing the click, drag, window or scroll event.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_mouse_event', mouse_event_msg)

    # Overlay text controls

    def set_overlay_text_enable(self, enable):
        """Enable or disable the image text overlay.

        Args:
            enable (bool): True to draw the overlay text, False to hide it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_enable', Bool(enable))

    def set_click_text(self, enable):
        """Enable or disable click-driven overlay text placement.

        Args:
            enable (bool): True to enable click-to-place text, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_click_text_enable', Bool(enable))

    def set_overlay_text_size_ratio(self, ratio):
        """Set the overlay text size ratio.

        Args:
            ratio (float): Overlay text size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_size_ratio', Float32(ratio))

    def set_overlay_text_vert_ratio(self, ratio):
        """Set the overlay text vertical position ratio.

        Args:
            ratio (float): Overlay text vertical position as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_vert_ratio', Float32(ratio))

    def set_overlay_text_horz_ratio(self, ratio):
        """Set the overlay text horizontal position ratio.

        Args:
            ratio (float): Overlay text horizontal position as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_horz_ratio', Float32(ratio))

    def set_overlay_text_transparency_ratio(self, ratio):
        """Set the overlay text transparency ratio.

        Args:
            ratio (float): Overlay text transparency as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_transparency_ratio', Float32(ratio))

    def set_overlay_text_color_rgb(self, color_msg):
        """Set the overlay text color.

        Args:
            color_msg (ColorBGR): A ColorBGR msg carrying the overlay text color.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_color_rgb', color_msg)

    def set_overlay_text_image_name(self, enable):
        """Enable or disable the data source name in the overlay text.

        Args:
            enable (bool): True to include the source name in the overlay text, False to omit it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_source_name', Bool(enable))

    def set_overlay_text_date_time(self, enable):
        """Enable or disable the date and time in the overlay text.

        Args:
            enable (bool): True to include the date and time in the overlay text, False to omit it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_date_time', Bool(enable))

    def set_overlay_text_nav(self, enable):
        """Enable or disable navigation data in the overlay text.

        Args:
            enable (bool): True to include navigation data in the overlay text, False to omit it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_nav', Bool(enable))

    def set_overlay_text_pose(self, enable):
        """Enable or disable pose data in the overlay text.

        Args:
            enable (bool): True to include pose data in the overlay text, False to omit it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_pose', Bool(enable))

    def set_overlay_text(self, text):
        """Append a line to the overlay text list.

        Args:
            text (str): The text line to append to the overlay text list.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_add_overlay_text', String(text))

    def set_overlay_text_list(self, text_list_msg):
        """Replace the overlay text list.

        Args:
            text_list_msg (StringArray): A StringArray msg carrying the full overlay text list.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_overlay_text_list', text_list_msg)

    def clear_overlay_text_list(self):
        """Clear the overlay text list.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_clear_overlay_text_list', Empty())

    # Crosshair overlay controls

    def set_crosshairs_enable(self, enable):
        """Enable or disable the crosshair overlays.

        Args:
            enable (bool): True to draw the crosshair overlays, False to hide them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_crosshairs_enable', Bool(enable))

    def set_crosshairs_size_ratio(self, ratio):
        """Set the crosshair size ratio.

        Args:
            ratio (float): Crosshair size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_crosshairs_size_ratio', Float32(ratio))

    def set_crosshairs_thickness_ratio(self, ratio):
        """Set the crosshair line thickness ratio.

        Args:
            ratio (float): Crosshair line thickness as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_crosshairs_thickness_ratio', Float32(ratio))

    def set_crosshairs_text_ratio(self, ratio):
        """Set the crosshair label text size ratio.

        Args:
            ratio (float): Crosshair label text size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_crosshairs_text_ratio', Float32(ratio))

    def set_crosshairs_transparency_ratio(self, ratio):
        """Set the crosshair transparency ratio.

        Args:
            ratio (float): Crosshair transparency as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_crosshairs_transparency_ratio', Float32(ratio))

    def set_crosshairs_color_rgb(self, color_msg):
        """Set the crosshair overlay color.

        Args:
            color_msg (ColorBGR): A ColorBGR msg carrying the crosshair color.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_crosshairs_color_rgb', color_msg)

    def set_overlay_crosshair_names(self, enable):
        """Enable or disable crosshair name labels.

        Args:
            enable (bool): True to label crosshairs with their names, False to omit the labels.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_overlay_crosshair_names', Bool(enable))

    def set_overlay_crosshair_pixels(self, enable):
        """Enable or disable crosshair pixel coordinate labels.

        Args:
            enable (bool): True to label crosshairs with their pixel coordinates, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_overlay_crosshair_pixels', Bool(enable))

    def set_overlay_crosshair_degrees(self, enable):
        """Enable or disable crosshair degree offset labels.

        Args:
            enable (bool): True to label crosshairs with their degree offsets, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_overlay_crosshair_degrees', Bool(enable))

    def set_overlay_crosshair_text(self, enable):
        """Enable or disable crosshair message labels.

        Args:
            enable (bool): True to label crosshairs with their message strings, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_overlay_crosshair_text', Bool(enable))

    def set_click_crosshair(self, enable):
        """Enable or disable click-driven crosshair placement.

        Args:
            enable (bool): True to enable click-to-place crosshairs, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_click_crosshair_enable', Bool(enable))

    def add_crosshair_pixel(self, crosshair_msg):
        """Add a crosshair positioned by pixel coordinates.

        Args:
            crosshair_msg (ImageCrosshair): An ImageCrosshair msg whose x_pixel and y_pixel fields locate the crosshair.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_add_crosshair_pixel', crosshair_msg)

    def add_crosshair_ratios(self, crosshair_msg):
        """Add a crosshair positioned by image ratios.

        Args:
            crosshair_msg (ImageCrosshair): An ImageCrosshair msg whose x_ratio and y_ratio fields locate the crosshair.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_add_crosshair_ratios', crosshair_msg)

    def add_crosshair_degree_offsets(self, crosshair_msg):
        """Add a crosshair positioned by degree offsets from image center.

        Args:
            crosshair_msg (ImageCrosshair): An ImageCrosshair msg whose x_offset_deg and y_offset_deg fields locate the crosshair.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_add_crosshair_degree_offsets', crosshair_msg)

    def remove_crosshair(self, name):
        """Remove a named crosshair.

        Args:
            name (str): Name of the crosshair to remove.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_remove_crosshair', String(name))

    def clear_crosshairs(self):
        """Remove every crosshair from the connected image source.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_clear_crosshairs', Empty())

    # Target overlay controls

    def set_targets_enable(self, enable):
        """Enable or disable the target overlays.

        Args:
            enable (bool): True to draw the target overlays, False to hide them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_targets_enable', Bool(enable))

    def set_targets_size_ratio(self, ratio):
        """Set the target size ratio.

        Args:
            ratio (float): Target size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_targets_size_ratio', Float32(ratio))

    def set_targets_thickness_ratio(self, ratio):
        """Set the target line thickness ratio.

        Args:
            ratio (float): Target line thickness as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_targets_thickness_ratio', Float32(ratio))

    def set_targets_text_ratio(self, ratio):
        """Set the target label text size ratio.

        Args:
            ratio (float): Target label text size as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_targets_text_ratio', Float32(ratio))

    def set_targets_transparency_ratio(self, ratio):
        """Set the target transparency ratio.

        Args:
            ratio (float): Target transparency as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_targets_transparency_ratio', Float32(ratio))

    def set_targets_color_rgb(self, color_msg):
        """Set the target overlay color.

        Args:
            color_msg (ColorBGR): A ColorBGR msg carrying the target color.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_targets_color_rgb', color_msg)

    def set_overlay_target_names(self, enable):
        """Enable or disable target name labels.

        Args:
            enable (bool): True to label targets with their names, False to omit the labels.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_overlay_target_names', Bool(enable))

    def set_overlay_target_pixels(self, enable):
        """Enable or disable target pixel coordinate labels.

        Args:
            enable (bool): True to label targets with their pixel coordinates, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_overlay_target_pixels', Bool(enable))

    def set_overlay_target_degrees(self, enable):
        """Enable or disable target degree offset labels.

        Args:
            enable (bool): True to label targets with their degree offsets, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_overlay_target_degrees', Bool(enable))

    def set_overlay_target_text(self, enable):
        """Enable or disable target message labels.

        Args:
            enable (bool): True to label targets with their message strings, False to omit them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_overlay_target_text', Bool(enable))

    def set_click_target(self, enable):
        """Enable or disable click-driven target placement.

        Args:
            enable (bool): True to enable click-to-place targets, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_click_target_enable', Bool(enable))

    def add_target_pixel(self, target_msg):
        """Add a target positioned by pixel coordinates.

        Args:
            target_msg (ImageTarget): An ImageTarget msg whose x_pixel and y_pixel fields locate the target.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_add_target_pixel', target_msg)

    def add_target_ratios(self, target_msg):
        """Add a target positioned by image ratios.

        Args:
            target_msg (ImageTarget): An ImageTarget msg whose x_ratio and y_ratio fields locate the target.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_add_target_ratios', target_msg)

    def add_target_degree_offsets(self, target_msg):
        """Add a target positioned by degree offsets from image center.

        Args:
            target_msg (ImageTarget): An ImageTarget msg whose x_offset_deg and y_offset_deg fields locate the target.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_add_target_degree_offsets', target_msg)

    def remove_target(self, name):
        """Remove a named target.

        Args:
            name (str): Name of the target to remove.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_remove_target', String(name))

    def clear_targets(self):
        """Remove every target from the connected image source.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_clear_targets', Empty())

    # Aspect and stream controls

    def set_aspect_adjust_enable(self, enable):
        """Enable or disable image aspect ratio adjustment.

        Args:
            enable (bool): True to enable aspect ratio adjustment, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_aspect_adjust_enable', Bool(enable))

    def set_aspect_adjust_ratio(self, ratio):
        """Set the image aspect adjustment ratio.

        Args:
            ratio (float): Target aspect ratio value.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_aspect_adjust_ratio', Float32(ratio))

    def set_aspect_adjust_by_ratio(self, ratio):
        """Set the image aspect adjustment by relative ratio.

        Args:
            ratio (float): Relative aspect adjustment as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_aspect_adjust_by_ratio', Float32(ratio))

    def set_stream_compression_enable(self, enable):
        """Enable or disable image stream compression.

        Args:
            enable (bool): True to compress the published image stream, False to publish uncompressed.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_stream_compression_enable', Bool(enable))

    def set_stream_compression_ratio(self, ratio):
        """Set the image stream compression ratio.

        Args:
            ratio (float): Stream compression as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_stream_compression_ratio', Float32(ratio))

    # Live adjustment controls

    def set_live_adjust_enable(self, enable):
        """Enable or disable live image adjustments.

        Args:
            enable (bool): True to enable live image adjustments, False to disable them.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_enable', Bool(enable))

    def set_live_adjust_rotate_ratio(self, ratio):
        """Set the live adjustment rotation ratio.

        Args:
            ratio (float): Live rotation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_rotate_ratio', Float32(ratio))

    def set_live_adjust_rotate_deg(self, deg):
        """Set the live adjustment rotation in degrees.

        Args:
            deg (float): Live rotation in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_rotate_deg', Float32(deg))

    def set_live_adjust_x_ratio(self, ratio):
        """Set the live adjustment horizontal translation ratio.

        Args:
            ratio (float): Live horizontal translation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_x_ratio', Float32(ratio))

    def set_live_adjust_x_pixel(self, pixel):
        """Set the live adjustment horizontal translation in pixels.

        Args:
            pixel (int): Live horizontal translation in pixels.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_x_pixel', Int32(pixel))

    def set_live_adjust_x_deg(self, deg):
        """Set the live adjustment horizontal translation in degrees.

        Args:
            deg (float): Live horizontal translation in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_x_deg', Float32(deg))

    def set_live_adjust_y_ratio(self, ratio):
        """Set the live adjustment vertical translation ratio.

        Args:
            ratio (float): Live vertical translation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_y_ratio', Float32(ratio))

    def set_live_adjust_y_pixel(self, pixel):
        """Set the live adjustment vertical translation in pixels.

        Args:
            pixel (int): Live vertical translation in pixels.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_y_pixel', Int32(pixel))

    def set_live_adjust_y_deg(self, deg):
        """Set the live adjustment vertical translation in degrees.

        Args:
            deg (float): Live vertical translation in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_live_adjust_y_deg', Float32(deg))

    # Capability-gated controls. data_if.BaseImageIF advertises these only when

    # the matching caps_dict flag is set, and the connect side cannot know the

    # server's configuration, so they are always registered.

    def set_resolution_ratio(self, ratio):
        """Set the image resolution ratio.

        Args:
            ratio (float): Image resolution as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_resolution_ratio', Float32(ratio))

    def set_auto_adjust_enable(self, enable):
        """Enable or disable image auto adjustment.

        Args:
            enable (bool): True to enable auto adjustment, False to disable it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_auto_adjust_enable', Bool(enable))

    def set_auto_adjust_ratio(self, ratio):
        """Set the image auto adjustment ratio.

        Args:
            ratio (float): Auto adjustment strength as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_auto_adjust_ratio', Float32(ratio))

    def set_brightness_ratio(self, ratio):
        """Set the image brightness ratio.

        Args:
            ratio (float): Image brightness as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_brightness_ratio', Float32(ratio))

    def set_contrast_ratio(self, ratio):
        """Set the image contrast ratio.

        Args:
            ratio (float): Image contrast as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_contrast_ratio', Float32(ratio))

    def set_threshold_ratio(self, ratio):
        """Set the image threshold ratio.

        Args:
            ratio (float): Image threshold as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_threshold_ratio', Float32(ratio))

    def rotate_2d(self):
        """Publish a single 2D rotation step command.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_rotate_2d', Empty())

    def set_rotate_2d_deg(self, deg):
        """Set the 2D image rotation in degrees.

        Args:
            deg (int): 2D rotation in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_rotate_2d_deg', Int32(deg))

    def set_rotate_2d_swap_box(self, enable):
        """Enable or disable bounding box swapping on 2D rotation.

        Args:
            enable (bool): True to swap the bounding box on rotation, False to leave it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_rotate_2d_swap_box', Bool(enable))

    def set_flip_horz(self, enable):
        """Enable or disable horizontal image flip.

        Args:
            enable (bool): True to flip the image horizontally, False to leave it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_flip_horz', Bool(enable))

    def set_flip_vert(self, enable):
        """Enable or disable vertical image flip.

        Args:
            enable (bool): True to flip the image vertically, False to leave it.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_flip_vert', Bool(enable))

    def set_range_ratios(self, range_msg):
        """Set the image range window ratios.

        Args:
            range_msg (RangeWindow): A RangeWindow msg carrying the start and stop range ratios.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_range_ratios', range_msg)

    def set_zoom_ratio(self, ratio):
        """Set the image zoom ratio.

        Args:
            ratio (float): Image zoom as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_zoom_ratio', Float32(ratio))

    def set_pan_x_ratio(self, ratio):
        """Set the image horizontal pan ratio.

        Args:
            ratio (float): Horizontal pan as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_pan_x_ratio', Float32(ratio))

    def set_pan_y_ratio(self, ratio):
        """Set the image vertical pan ratio.

        Args:
            ratio (float): Vertical pan as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_pan_y_ratio', Float32(ratio))

    def set_zoom_3d_ratio(self, ratio):
        """Set the 3D render zoom ratio.

        Args:
            ratio (float): 3D render zoom as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_zoom_3d_ratio', Float32(ratio))

    def set_rotate_3d_ratio(self, ratio):
        """Set the 3D render rotation ratio.

        Args:
            ratio (float): 3D render rotation as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_rotate_3d_ratio', Float32(ratio))

    def set_tilt_3d_ratio(self, ratio):
        """Set the 3D render tilt ratio.

        Args:
            ratio (float): 3D render tilt as a 0.0-1.0 ratio.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_tilt_3d_ratio', Float32(ratio))

    def set_filter_enable(self, update_msg):
        """Enable or disable a named image filter.

        Args:
            update_msg (UpdateBool): An UpdateBool msg naming the filter and carrying its enable value.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_filter_enable', update_msg)

    def set_filter_ratio(self, update_msg):
        """Set the ratio of a named image filter.

        Args:
            update_msg (UpdateFloat): An UpdateFloat msg naming the filter and carrying its ratio value.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_filter_ratio', update_msg)

    def set_camera_fov(self, fov_deg):
        """Set the 3D render camera field of view.

        Only advertised by sources reporting the 3D camera capability.

        Args:
            fov_deg (int): Camera field of view in degrees.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_camera_fov', Int32(fov_deg))

    def set_camera_view(self, vector3_msg):
        """Set the 3D render camera view vector.

        Only advertised by sources reporting the 3D camera capability.

        Args:
            vector3_msg (Vector3): Caller-built Vector3 msg carrying the view vector.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_camera_view', vector3_msg)

    def set_camera_position(self, vector3_msg):
        """Set the 3D render camera position.

        Only advertised by sources reporting the 3D camera capability.

        Args:
            vector3_msg (Vector3): Caller-built Vector3 msg carrying the position.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_camera_position', vector3_msg)

    def set_camera_rotation(self, vector3_msg):
        """Set the 3D render camera rotation.

        Only advertised by sources reporting the 3D camera capability.

        Args:
            vector3_msg (Vector3): Caller-built Vector3 msg carrying the rotation.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_camera_rotation', vector3_msg)

    def set_white_bg_enable(self, enable):
        """Enable or disable the white render background.

        Only advertised by sources reporting the 3D camera capability.

        Args:
            enable (bool): True for a white background, False otherwise.

        Returns:
            bool: True if the command was published, False if there is no node
                interface or no connected source.
        """
        if self.node_if is None or self.connected == False:
            return False
        return self.node_if.publish_pub(self.node_if_prefix + 'image_set_white_bg_enable', Bool(enable))


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
        data_dict['color_image'] = data
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
# Connect ColorImage IF Class
#########################################


COLOR_IMAGE_CONNECT_NAME = 'color_image_connect'


class ConnectColorImageIF(ConnectBaseImageIF):
    """Connect class mirroring data_if.ColorImageIF.

    Adds nothing beyond the shared ConnectBaseImageIF image connect pattern; the
    connected 'color_image' data product is selected by namespace.
    """

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = COLOR_IMAGE_CONNECT_NAME,
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
        if type(self) is ConnectColorImageIF:
            self.start()




from nepi_api.messages_if import MsgIF


SYSTEM_ALL_TOPIC = 'all'

#########################################
# Connect Image IF All Class
#########################################
IMAGE_ALL_TOPIC = 'images'


class ConnectImagesAllIF:

    # ADD Additional Connect Callback Functions


    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    connected = False

    connect_topic_pubs_dict = None
    #######################
    ### IF Initialization
    def __init__(self,
                msg_if = None,
                node_if = None
                ):


        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()



        ##############################  
        # Connect Msg Class

        if msg_if is None:
            self.msg_if = MsgIF()
        else:
            self.msg_if = msg_if
        self.msg_if.pub_info("Starting " + str(self.class_name) + " Initialization Processes")

        ##############################  
        # Setup All Namespace

        # Collective controls publish on the shared images all_namespace, which fans a
        # single command out to every IDX image.
        all_namespace = nepi_sdk.create_namespace(self.base_namespace, SYSTEM_ALL_TOPIC)
        all_namespace = nepi_sdk.create_namespace(all_namespace, IMAGE_ALL_TOPIC)
        self.all_namespace = all_namespace
        self.msg_if.pub_warn("Using connect all_namespace: " + str(self.all_namespace) )


        ##############################  
        # Connect Node Class

        if node_if is None:
            self.msg_if.pub_info("Node IF Not Provided")
            return
        self.node_if = node_if









        # Publishers Config Dict ####################
        self.connect_topic_pubs_dict = {
            'all_overlay_text_enable': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_text_size_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_overlay_text_horz_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_horz_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_overlay_text_vert_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_vert_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_overlay_text_transparency_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_overlay_text_color_rgb': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            'all_overlay_text_source_name': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_source_name',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_text_date_time': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_date_time',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_text_nav': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_nav',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_text_pose': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_pose',
                'msg': Bool,
                'qsize': 1,
            },
            # 'all_click_text_enable': {
            #     'namespace': self.all_namespace,
            #     'topic': 'click_text_enable',
            #     'msg': Bool,
            #     'qsize': 1,
            # },
            'all_overlay_text': {
                'namespace': self.all_namespace,
                'topic': 'add_overlay_text',
                'msg': String,
                'qsize': 1,
            },
            'all_overlay_text_list': {
                'namespace': self.all_namespace,
                'topic': 'set_overlay_text_list',
                'msg': StringArray,
                'qsize': 1,
            },
            'all_overlay_text_clear': {
                'namespace': self.all_namespace,
                'topic': 'clear_overlay_text_list',
                'msg': Empty,
                'qsize': 1,
            },
            'all_crosshairs_enable': {
                'namespace': self.all_namespace,
                'topic': 'crosshairs_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'all_crosshairs_size_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_crosshairs_thickness_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_thickness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_crosshairs_text_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_text_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_crosshairs_transparency_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_crosshairs_color_rgb': {
                'namespace': self.all_namespace,
                'topic': 'set_crosshairs_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            'all_overlay_crosshair_names': {
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_names',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_pixels': {
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_pixels',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_degrees': {
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_degrees',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_text': {
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_text',
                'msg': Bool,
                'qsize': 1,
            },
            # 'all_click_crosshair_enable': {
            #     'namespace': self.all_namespace,
            #     'topic': 'click_crosshair_enable',
            #     'msg': Bool,
            #     'qsize': 1,
            # },
            # 'all_add_crosshair_pixel': {
            #     'namespace': self.all_namespace,
            #     'topic': 'add_crosshair_pixel',
            #     'msg': ImageCrosshair,
            #     'qsize': 1,
            # },
            'all_add_crosshair_ratios': {
                'namespace': self.all_namespace,
                'topic': 'add_crosshair_ratios',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'all_add_crosshair_degree_offsets': {
                'namespace': self.all_namespace,
                'topic': 'add_crosshair_degree_offsets',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'all_remove_crosshair': {
                'namespace': self.all_namespace,
                'topic': 'remove_crosshair',
                'msg': String,
                'qsize': 1,
            },
            'all_clear_crosshairs': {
                'namespace': self.all_namespace,
                'topic': 'clear_crosshairs',
                'msg': Empty,
                'qsize': 1,
            },
            'all_targets_enable': {
                'namespace': self.all_namespace,
                'topic': 'targets_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'all_targets_size_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_targets_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_targets_thickness_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_targets_thickness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_targets_text_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_targets_text_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_targets_transparency_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_targets_transparency_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_targets_color_rgb': {
                'namespace': self.all_namespace,
                'topic': 'set_targets_color_rgb',
                'msg': ColorBGR,
                'qsize': 1,
            },
            'all_overlay_target_names': {
                'namespace': self.all_namespace,
                'topic': 'overlay_target_names',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_target_pixels': {
                'namespace': self.all_namespace,
                'topic': 'overlay_target_pixels',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_target_degrees': {
                'namespace': self.all_namespace,
                'topic': 'overlay_target_degrees',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_target_text': {
                'namespace': self.all_namespace,
                'topic': 'overlay_target_text',
                'msg': Bool,
                'qsize': 1,
            },
            # 'all_click_target_enable': {
            #     'namespace': self.all_namespace,
            #     'topic': 'click_target_enable',
            #     'msg': Bool,
            #     'qsize': 1,
            # },
            # 'all_add_target_pixel': {
            #     'namespace': self.all_namespace,
            #     'topic': 'add_target_pixel',
            #     'msg': ImageTarget,
            #     'qsize': 1,
            # },
            'all_add_target_ratios': {
                'namespace': self.all_namespace,
                'topic': 'add_target_ratios',
                'msg': ImageTarget,
                'qsize': 1,
            },
            'all_add_target_degree_offsets': {
                'namespace': self.all_namespace,
                'topic': 'add_target_degree_offsets',
                'msg': ImageTarget,
                'qsize': 1,
            },
            'all_remove_target': {
                'namespace': self.all_namespace,
                'topic': 'remove_target',
                'msg': String,
                'qsize': 1,
            },
            'all_clear_targets': {
                'namespace': self.all_namespace,
                'topic': 'clear_targets',
                'msg': Empty,
                'qsize': 1,
            },
            'all_set_aspect_adjust_enable': {
                'namespace': self.all_namespace,
                'topic': 'set_aspect_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'all_set_aspect_adjust_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_aspect_adjust_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_aspect_adjust_by_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_aspect_adjust_by_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_stream_compression_enable': {
                'namespace': self.all_namespace,
                'topic': 'set_stream_compression_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'all_set_stream_compression_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_stream_compression_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_enable': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'all_set_live_adjust_rotate_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_rotate_deg': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_pixel': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_deg': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_ratio': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_pixel': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_deg': {
                'namespace': self.all_namespace,
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 1,
            }
        }

        if self.node_if is not None:
            self.node_if.register_pubs(self.connect_topic_pubs_dict)
            self.connecting = False
            self.connected = True
            self.connected_topic = self.all_namespace
            self.status_msg = None
            



        ##############################
        # Initialize Class Variables


        ##############################
        # Complete Initialization
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")
        ###############################


    #######################
    # Class Public Methods
    #######################


    def get_ready_state(self):
        """Return the ready state of the interface.

        Returns:
            bool: True if the interface has completed initialization, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timeout = float('inf') ):
        """Block until the interface is ready or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the interface became ready, False if the timeout was reached.
        """
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.ready

    def get_namespace(self):
        """Return the fully-resolved ROS namespace the collective image controls publish on.

        Returns:
            str: The shared images namespace (base namespace + '/images') used to fan a
                single command out to every IDX image.
        """
        return self.all_namespace


    #################
    ## Image Text Overlays

    def set_overlay_text_enable(self, enable):
        """Enable or disable all text overlays on every IDX image output.

        Args:
            enable (bool): True to render the text overlays, False to hide them.
        """
        pub_name = 'all_overlay_text_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_size_ratio(self, size_ratio):
        """Set the overlay text size on every IDX image output.

        Args:
            size_ratio (float): Desired overlay text size as a ratio from 0.0 (smallest) to 1.0 (largest).
        """
        pub_name = 'all_overlay_text_size_ratio'
        msg = size_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_horz_ratio(self, horz_ratio):
        """Set the horizontal position of the text overlay block on every IDX image output.

        Args:
            horz_ratio (float): Position from the image left edge as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_overlay_text_horz_ratio'
        msg = horz_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_vert_ratio(self, vert_ratio):
        """Set the vertical position of the text overlay block on every IDX image output.

        Args:
            vert_ratio (float): Position from the image top edge as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_overlay_text_vert_ratio'
        msg = vert_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_transparency_ratio(self, transparency_ratio):
        """Set the text overlay transparency on every IDX image output.

        Args:
            transparency_ratio (float): Transparency as a ratio from 0.0 (opaque) to 1.0 (invisible).
        """
        pub_name = 'all_overlay_text_transparency_ratio'
        msg = transparency_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_color_rgb(self, color_rgb = (0,255,0)):
        """Set the text overlay color on every IDX image output.

        Args:
            color_rgb (tuple): Three-element (r, g, b) color, each component 0-255.
        """
        pub_name = 'all_overlay_text_color_rgb'
        msg = ColorBGR()
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_source_name(self, enable):
        """Enable or disable the image source name overlay on every IDX image output.

        Args:
            enable (bool): True to overlay the image source name, False to hide it.
        """
        pub_name = 'all_overlay_text_source_name'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_date_time(self, enable):
        """Enable or disable the date and time overlay on every IDX image output.

        Args:
            enable (bool): True to overlay the date and time, False to hide it.
        """
        pub_name = 'all_overlay_text_date_time'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_nav(self, enable):
        """Enable or disable the navigation data overlay on every IDX image output.

        Args:
            enable (bool): True to overlay navigation data, False to hide it.
        """
        pub_name = 'all_overlay_text_nav'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_pose(self, enable):
        """Enable or disable the pose data overlay on every IDX image output.

        Args:
            enable (bool): True to overlay pose data, False to hide it.
        """
        pub_name = 'all_overlay_text_pose'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_click_text_enable(self, enable):
        """Enable or disable placing the text overlay block at the next image click.

        Args:
            enable (bool): True to move the text block on the next click, False to disable it.
        """
        pub_name = 'all_click_text_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def add_overlay_text(self, text):
        """Add a line of custom text to every IDX image overlay.

        Args:
            text (str): The text string to add to the overlays.
        """
        pub_name = 'all_overlay_text'
        msg = String()
        msg.data = text
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_text_list(self, overlay_text_list):
        """Set the full list of custom overlay text lines on every IDX image output.

        Args:
            overlay_text_list (list): A list of text strings to display as image overlays.
        """
        pub_name = 'all_overlay_text_list'
        msg = StringArray()
        msg.array = overlay_text_list
        self.node_if.publish_pub(pub_name, msg)

    def clear_overlay_text_list(self):
        """Clear all custom overlay text lines from every IDX image output.
        """
        pub_name = 'all_overlay_text_clear'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    #################
    ## Image Crosshair Overlays
    def set_crosshairs_enable(self, enable):
        """Enable or disable crosshair overlays on every IDX image output.

        Args:
            enable (bool): True to overlay crosshairs, False to hide them.
        """
        pub_name = 'all_crosshairs_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_crosshairs_size_ratio(self, size_ratio):
        """Set the crosshair size on every IDX image output.

        Args:
            size_ratio (float): Desired crosshair size as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_crosshairs_size_ratio'
        msg = size_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_crosshairs_thickness_ratio(self, thickness_ratio):
        """Set the crosshair line thickness on every IDX image output.

        Args:
            thickness_ratio (float): Desired line thickness as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_crosshairs_thickness_ratio'
        msg = thickness_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_crosshairs_text_ratio(self, text_ratio):
        """Set the crosshair label text size on every IDX image output.

        Args:
            text_ratio (float): Desired label text size as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_crosshairs_text_ratio'
        msg = text_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_crosshairs_transparency_ratio(self, transparency_ratio):
        """Set the crosshair transparency on every IDX image output.

        Args:
            transparency_ratio (float): Transparency as a ratio from 0.0 (opaque) to 1.0 (invisible).
        """
        pub_name = 'all_crosshairs_transparency_ratio'
        msg = transparency_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_crosshairs_color_rgb(self, color_rgb = (0,255,0)):
        """Set the crosshair color on every IDX image output.

        Args:
            color_rgb (tuple): Three-element (r, g, b) color, each component 0-255.
        """
        pub_name = 'all_crosshairs_color_rgb'
        msg = ColorBGR()
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_crosshair_names(self, enable):
        """Enable or disable crosshair name labels on every IDX image output.

        Args:
            enable (bool): True to overlay crosshair names, False to hide them.
        """
        pub_name = 'all_overlay_crosshair_names'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_crosshair_pixels(self, enable):
        """Enable or disable crosshair pixel coordinate labels on every IDX image output.

        Args:
            enable (bool): True to overlay crosshair pixel coordinates, False to hide them.
        """
        pub_name = 'all_overlay_crosshair_pixels'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_crosshair_degrees(self, enable):
        """Enable or disable crosshair degree coordinate labels on every IDX image output.

        Args:
            enable (bool): True to overlay crosshair degree coordinates, False to hide them.
        """
        pub_name = 'all_overlay_crosshair_degrees'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_crosshair_text(self, enable):
        """Enable or disable crosshair messages coordinate labels on every IDX image output.

        Args:
            enable (bool): True to overlay crosshair messages coordinates, False to hide them.
        """
        pub_name = 'all_overlay_crosshair_text'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_click_crosshair_enable(self, enable):
        """Enable or disable adding a crosshair at the clicked location on every IDX image.

        Args:
            enable (bool): True to add a crosshair on image click, False to disable it.
        """
        pub_name = 'all_click_crosshair_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def add_crosshair_pixel(self, name, x_pixel, y_pixel, color_rgb = (0,255,0), msg_str = ''):
        """Add a named crosshair at a pixel location on every IDX image output.

        Args:
            name (str): Identifier for the crosshair.
            x_pixel (int): Horizontal pixel location of the crosshair from the image left edge.
            y_pixel (int): Vertical pixel location of the crosshair from the image top edge.
        """
        pub_name = 'all_add_crosshair_pixel'
        msg = ImageCrosshair()
        msg.name = name
        msg.x_pixel = x_pixel
        msg.y_pixel = y_pixel
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        msg.msg_str = msg_str
        self.node_if.publish_pub(pub_name, msg)

    def add_crosshair_ratios(self, name, x_ratio, y_ratio, color_rgb = (0,255,0)):
        """Add a named crosshair at a pixel location on every IDX image output.

        Args:
            name (str): Identifier for the crosshair.
            x_pixel (int): Horizontal pixel location of the crosshair from the image left edge.
            y_pixel (int): Vertical pixel location of the crosshair from the image top edge.
        """
        pub_name = 'all_add_crosshair_ratios'
        msg = ImageCrosshair()
        msg.name = name
        msg.x_ratio = x_ratio
        msg.y_ratio = y_ratio
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        self.node_if.publish_pub(pub_name, msg)

    def add_crosshair_degree_offsets(self, name, x_offset_deg, y_offset_deg, color_rgb = (0,255,0)):
        """Add a named crosshair at a degree offset on every IDX image output.

        Args:
            name (str): Identifier for the crosshair.
            x_offset_deg (float): Horizontal degree offset of the crosshair from image center.
            y_offset_deg (float): Vertical degree offset of the crosshair from image center.
        """
        pub_name = 'all_add_crosshair_degree_offsets'
        msg = ImageCrosshair()
        msg.name = name
        msg.x_offset_deg = x_offset_deg
        msg.y_offset_deg = y_offset_deg
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        self.node_if.publish_pub(pub_name, msg)

    def remove_crosshair(self, name):
        """Remove a named crosshair from every IDX image output.

        Args:
            name (str): Identifier of the crosshair to remove.
        """
        pub_name = 'all_remove_crosshair'
        msg = String()
        msg.data = name
        self.node_if.publish_pub(pub_name, msg)

    def clear_crosshairs(self):
        """Remove all crosshairs from every IDX image output.
        """
        pub_name = 'all_clear_crosshairs'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    #################
    ## Image Target Overlays

    def set_targets_enable(self, enable):
        """Enable or disable target overlays on every IDX image output.

        Args:
            enable (bool): True to overlay targets, False to hide them.
        """
        pub_name = 'all_targets_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_targets_size_ratio(self, size_ratio):
        """Set the target marker size on every IDX image output.

        Args:
            size_ratio (float): Desired target size as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_targets_size_ratio'
        msg = size_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_targets_thickness_ratio(self, thickness_ratio):
        """Set the target marker line thickness on every IDX image output.

        Args:
            thickness_ratio (float): Desired line thickness as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_targets_thickness_ratio'
        msg = thickness_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_targets_text_ratio(self, text_ratio):
        """Set the target label text size on every IDX image output.

        Args:
            text_ratio (float): Desired label text size as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_targets_text_ratio'
        msg = text_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_targets_transparency_ratio(self, transparency_ratio):
        """Set the target marker transparency on every IDX image output.

        Args:
            transparency_ratio (float): Transparency as a ratio from 0.0 (opaque) to 1.0 (invisible).
        """
        pub_name = 'all_targets_transparency_ratio'
        msg = transparency_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_targets_color_rgb(self, color_rgb = (0,255,0)):
        """Set the target marker color on every IDX image output.

        Args:
            color_rgb (tuple): Three-element (r, g, b) color, each component 0-255.
        """
        pub_name = 'all_targets_color_rgb'
        msg = ColorBGR()
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_target_names(self, enable):
        """Enable or disable target name labels on every IDX image output.

        Args:
            enable (bool): True to overlay target names, False to hide them.
        """
        pub_name = 'all_overlay_target_names'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_target_pixels(self, enable):
        """Enable or disable target pixel coordinate labels on every IDX image output.

        Args:
            enable (bool): True to overlay target pixel coordinates, False to hide them.
        """
        pub_name = 'all_overlay_target_pixels'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_target_degrees(self, enable):
        """Enable or disable target degree coordinate labels on every IDX image output.

        Args:
            enable (bool): True to overlay target degree coordinates, False to hide them.
        """
        pub_name = 'all_overlay_target_degrees'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_target_text(self, enable):
        """Enable or disable target message labels on every IDX image output.

        Args:
            enable (bool): True to overlay target messages, False to hide them.
        """
        pub_name = 'all_overlay_target_text'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_click_target_enable(self, enable):
        """Enable or disable adding a target at the clicked location on every IDX image.

        Args:
            enable (bool): True to add a target on image click, False to disable it.
        """
        pub_name = 'all_click_target_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def add_target_pixel(self, name, x_pixel, y_pixel, color_rgb = (0,255,0), msg_str = ''):
        """Add a named target at a pixel location on every IDX image output.

        Args:
            name (str): Identifier for the target.
            x_pixel (int): Horizontal pixel location of the target from the image left edge.
            y_pixel (int): Vertical pixel location of the target from the image top edge.
            color_rgb (tuple): Three-element (r, g, b) marker color, each component 0-255.
            msg_str (str): Optional message rendered beside the target.
        """
        pub_name = 'all_add_target_pixel'
        msg = ImageTarget()
        msg.name = name
        msg.x_pixel = x_pixel
        msg.y_pixel = y_pixel
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        msg.msg_str = msg_str
        self.node_if.publish_pub(pub_name, msg)

    def add_target_ratios(self, name, x_ratio, y_ratio, color_rgb = (0,255,0), msg_str = ''):
        """Add a named target at an image ratio location on every IDX image output.

        Args:
            name (str): Identifier for the target.
            x_ratio (float): Horizontal location as a ratio from 0.0 (left) to 1.0 (right).
            y_ratio (float): Vertical location as a ratio from 0.0 (top) to 1.0 (bottom).
            color_rgb (tuple): Three-element (r, g, b) marker color, each component 0-255.
            msg_str (str): Optional message rendered beside the target.
        """
        pub_name = 'all_add_target_ratios'
        msg = ImageTarget()
        msg.name = name
        msg.x_ratio = x_ratio
        msg.y_ratio = y_ratio
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        msg.msg_str = msg_str
        self.node_if.publish_pub(pub_name, msg)

    def add_target_degree_offsets(self, name, x_offset_deg, y_offset_deg, color_rgb = (0,255,0), msg_str = ''):
        """Add a named target at a degree offset on every IDX image output.

        Args:
            name (str): Identifier for the target.
            x_offset_deg (float): Horizontal degree offset of the target from image center.
            y_offset_deg (float): Vertical degree offset of the target from image center.
            color_rgb (tuple): Three-element (r, g, b) marker color, each component 0-255.
            msg_str (str): Optional message rendered beside the target.
        """
        pub_name = 'all_add_target_degree_offsets'
        msg = ImageTarget()
        msg.name = name
        msg.x_offset_deg = x_offset_deg
        msg.y_offset_deg = y_offset_deg
        msg.r = color_rgb[0]
        msg.g = color_rgb[1]
        msg.b = color_rgb[2]
        msg.msg_str = msg_str
        self.node_if.publish_pub(pub_name, msg)

    def remove_target(self, name):
        """Remove a named target from every IDX image output.

        Args:
            name (str): Identifier of the target to remove.
        """
        pub_name = 'all_remove_target'
        msg = String()
        msg.data = name
        self.node_if.publish_pub(pub_name, msg)

    def clear_targets(self):
        """Remove all targets from every IDX image output.
        """
        pub_name = 'all_clear_targets'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    #################
    ## Aspect Controls

    def set_aspect_adjust_enable(self, enable):
        """Enable or disable aspect ratio adjustment on every IDX image output.

        Args:
            enable (bool): True to apply the selected aspect ratio, False to publish the original.
        """
        pub_name = 'all_set_aspect_adjust_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_aspect_adjust_ratio(self, aspect_ratio):
        pub_name = 'all_set_aspect_adjust_ratio'
        msg = Float32()
        msg.data = aspect_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_aspect_adjust_by_ratio(self, ratio):
        pub_name = 'all_set_aspect_adjust_by_ratio'
        msg = Float32()
        msg.data = ratio
        self.node_if.publish_pub(pub_name, msg)

    #################
    ## Stream Compression Controls
    def set_stream_compression_enable(self, enable):
        """Enable or disable HTTP stream compression on every IDX image output.

        Args:
            enable (bool): True to compress the stream, False to publish uncompressed.
        """
        pub_name = 'all_set_stream_compression_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_stream_compression_ratio(self, compression_ratio):
        """Set the HTTP stream compression level on every IDX image output.

        Args:
            compression_ratio (float): Compression level as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_set_stream_compression_ratio'
        msg = compression_ratio
        self.node_if.publish_pub(pub_name, msg)

    #################
    ## Live Adjust Controls

    def set_live_adjust_enable(self, enable):
        """Enable or disable the live rotate and translate adjustments on every IDX image.

        Args:
            enable (bool): True to apply the live adjustments, False to bypass them.
        """
        pub_name = 'all_set_live_adjust_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_live_adjust_rotate_ratio(self, rotate_ratio):
        """Set the live rotation adjustment of every IDX image as a ratio.

        Args:
            rotate_ratio (float): Desired rotation as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_set_live_adjust_rotate_ratio'
        msg = rotate_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_live_adjust_rotate_deg(self, rotate_deg):
        """Set the live rotation adjustment of every IDX image in degrees.

        Args:
            rotate_deg (float): Desired rotation in degrees.
        """
        pub_name = 'all_set_live_adjust_rotate_deg'
        msg = rotate_deg
        self.node_if.publish_pub(pub_name, msg)

    def set_live_adjust_x_ratio(self, x_ratio):
        """Set the live horizontal translation adjustment of every IDX image as a ratio.

        Args:
            x_ratio (float): Desired horizontal translation as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_set_live_adjust_x_ratio'
        msg = x_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_live_adjust_x_pixel(self, x_pixel):
        """Set the live horizontal translation adjustment of every IDX image in pixels.

        Args:
            x_pixel (int): Desired horizontal translation in pixels.
        """
        pub_name = 'all_set_live_adjust_x_pixel'
        msg = x_pixel
        self.node_if.publish_pub(pub_name, msg)

    def set_live_adjust_x_deg(self, x_deg):
        """Set the live horizontal translation adjustment of every IDX image in degrees.

        Args:
            x_deg (float): Desired horizontal translation in degrees.
        """
        pub_name = 'all_set_live_adjust_x_deg'
        msg = x_deg
        self.node_if.publish_pub(pub_name, msg)

    def set_live_adjust_y_ratio(self, y_ratio):
        """Set the live vertical translation adjustment of every IDX image as a ratio.

        Args:
            y_ratio (float): Desired vertical translation as a ratio from 0.0 to 1.0.
        """
        pub_name = 'all_set_live_adjust_y_ratio'
        msg = y_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_live_adjust_y_pixel(self, y_pixel):
        """Set the live vertical translation adjustment of every IDX image in pixels.

        Args:
            y_pixel (int): Desired vertical translation in pixels.
        """
        pub_name = 'all_set_live_adjust_y_pixel'
        msg = y_pixel
        self.node_if.publish_pub(pub_name, msg)

    def set_live_adjust_y_deg(self, y_deg):
        """Set the live vertical translation adjustment of every IDX image in degrees.

        Args:
            y_deg (float): Desired vertical translation in degrees.
        """
        pub_name = 'all_set_live_adjust_y_deg'
        msg = y_deg
        self.node_if.publish_pub(pub_name, msg)

    ###############################
    # Class Private Methods
    ###############################







