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
import numpy as np


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_interfaces.msg import StringArray, ColorBGR, ImageCrosshair, ImageTarget

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
            self.msg_if = MsgIF(log_name = self.class_name)
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
            'all_overlay_crosshair_messages': {
                'namespace': self.all_namespace,
                'topic': 'overlay_crosshair_messages',
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
            'all_overlay_target_messages': {
                'namespace': self.all_namespace,
                'topic': 'overlay_target_messages',
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
                'msg': String,
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
    ## Collective Image Overlay and Crosshair Controls

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

    def set_overlay_crosshair_messages(self, enable):
        """Enable or disable crosshair messages coordinate labels on every IDX image output.

        Args:
            enable (bool): True to overlay crosshair messages coordinates, False to hide them.
        """
        pub_name = 'all_overlay_crosshair_messages'
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
    ## Collective Image Target Controls

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

    def set_overlay_target_messages(self, enable):
        """Enable or disable target message labels on every IDX image output.

        Args:
            enable (bool): True to overlay target messages, False to hide them.
        """
        pub_name = 'all_overlay_target_messages'
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
    ## Collective Aspect and Stream Controls

    def set_aspect_adjust_enable(self, enable):
        """Enable or disable aspect ratio adjustment on every IDX image output.

        Args:
            enable (bool): True to apply the selected aspect ratio, False to publish the original.
        """
        pub_name = 'all_set_aspect_adjust_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_aspect_adjust_ratio(self, aspect_ratio):
        """Select the aspect ratio applied to every IDX image output.

        Args:
            aspect_ratio (str): One of the aspect_ratio_options reported in ImageStatus.
        """
        pub_name = 'all_set_aspect_adjust_ratio'
        msg = String()
        msg.data = aspect_ratio
        self.node_if.publish_pub(pub_name, msg)

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
    ## Collective Live Adjust Controls

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







