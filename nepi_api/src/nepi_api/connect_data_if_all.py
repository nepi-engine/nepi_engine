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
from nepi_interfaces.msg import StringArray, ImageCrosshair

from nepi_api.messages_if import MsgIF



#########################################
# Connect Image IF All Class
#########################################
IMAGE_IF_ALL_TOPIC = 'images'


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
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,'rbx')

        ##############################  
        # Connect Msg Class

        if msg_if is None:
            self.msg_if = MsgIF(log_name = self.class_name)
        else:
            self.msg_if = msg_if
        self.msg_if.pub_info("Starting " + str(self.class_name) + " Initialization Processes")

        ##############################  
        # Connect Node Class

        if node_if is None:
            self.msg_if.pub_info("Node IF Not Provided")
            return
        self.node_if = node_if



        # Collective controls publish on the shared images namespace, which fans a
        # single command out to every IDX image.
        self.namespace = nepi_sdk.create_namespace(self.base_namespace, IMAGE_IF_ALL_TOPIC)
        self.msg_if.pub_warn("Using connect namespace: " + str(self.namespace) )





        # Publishers Config Dict ####################
        self.connect_topic_pubs_dict = {
            'all_overlay_size_ratio': {
                'namespace': self.namespace,
                'topic': 'set_overlay_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_overlay_source_name': {
                'namespace': self.namespace,
                'topic': 'set_overlay_source_name',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_date_time': {
                'namespace': self.namespace,
                'topic': 'set_overlay_date_time',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_nav': {
                'namespace': self.namespace,
                'topic': 'set_overlay_nav',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_pose': {
                'namespace': self.namespace,
                'topic': 'set_overlay_pose',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_text': {
                'namespace': self.namespace,
                'topic': 'add_overlay_text',
                'msg': String,
                'qsize': 1,
            },
            'all_overlay_list': {
                'namespace': self.namespace,
                'topic': 'set_overlay_list',
                'msg': StringArray,
                'qsize': 1,
            },
            'all_overlay_clear': {
                'namespace': self.namespace,
                'topic': 'clear_overlay_list',
                'msg': Empty,
                'qsize': 1,
            },
            'all_overlay_crosshairs': {
                'namespace': self.namespace,
                'topic': 'all_overlay_crosshairs',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_names': {
                'namespace': self.namespace,
                'topic': 'all_overlay_crosshair_names',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_pixels': {
                'namespace': self.namespace,
                'topic': 'all_overlay_crosshair_pixels',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_degrees': {
                'namespace': self.namespace,
                'topic': 'overlay_crosshair_degrees',
                'msg': Bool,
                'qsize': 1,
            },
            'all_click_crosshair_enable': {
                'namespace': self.namespace,
                'topic': 'click_crosshair_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'all_add_crosshair_pixel': {
                'namespace': self.namespace,
                'topic': 'add_crosshair_pixel',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'all_add_crosshair_ratios': {
                'namespace': self.namespace,
                'topic': 'add_crosshair_ratios',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'all_add_crosshair_degree_offsets': {
                'namespace': self.namespace,
                'topic': 'add_crosshair_degree_offsets',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'all_remove_crosshair': {
                'namespace': self.namespace,
                'topic': 'remove_crosshair',
                'msg': String,
                'qsize': 1,
            },
            'all_clear_crosshairs': {
                'namespace': self.namespace,
                'topic': 'clear_crosshairs',
                'msg': Empty,
                'qsize': 1,
            },
            'all_set_live_adjust_rotate_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_rotate_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_ratio': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_pixel': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_deg': {
                'namespace': self.namespace,
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 1,
            }
        }

        if self.node_if is not None:
            self.node_if.register_pubs(self.connect_topic_pubs_dict)
            self.connecting = False
            self.connected = True
            self.connected_topic = self.namespace
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
        return nepi_sdk.create_namespace(self.namespace)


    #################
    ## Collective Image Overlay and Crosshair Controls

    def set_overlay_size_ratio(self, size_ratio):
        """Set the overlay text and graphic size on every IDX image output.

        Args:
            size_ratio (float): Desired overlay size as a ratio from 0.0 (smallest) to 1.0 (largest).
        """
        pub_name = 'all_overlay_size_ratio'
        msg = size_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_source_name(self, enable):
        """Enable or disable the image source name overlay on every IDX image output.

        Args:
            enable (bool): True to overlay the image source name, False to hide it.
        """
        pub_name = 'all_overlay_source_name'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_date_time(self, enable):
        """Enable or disable the date and time overlay on every IDX image output.

        Args:
            enable (bool): True to overlay the date and time, False to hide it.
        """
        pub_name = 'all_overlay_date_time'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_nav(self, enable):
        """Enable or disable the navigation data overlay on every IDX image output.

        Args:
            enable (bool): True to overlay navigation data, False to hide it.
        """
        pub_name = 'all_overlay_nav'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_pose(self, enable):
        """Enable or disable the pose data overlay on every IDX image output.

        Args:
            enable (bool): True to overlay pose data, False to hide it.
        """
        pub_name = 'all_overlay_pose'
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

    def set_overlay_list(self, overlay_list):
        """Set the full list of custom overlay text lines on every IDX image output.

        Args:
            overlay_list (list): A list of text strings to display as image overlays.
        """
        pub_name = 'all_overlay_list'
        msg = StringArray()
        msg.array = overlay_list
        self.node_if.publish_pub(pub_name, msg)

    def clear_overlay_list(self):
        """Clear all custom overlay text lines from every IDX image output.
        """
        pub_name = 'all_overlay_clear'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def set_overlay_crosshairs(self, enable):
        """Enable or disable crosshair overlays on every IDX image output.

        Args:
            enable (bool): True to overlay crosshairs, False to hide them.
        """
        pub_name = 'all_overlay_crosshairs'
        msg = enable
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

    def set_click_crosshair_enable(self, enable):
        """Enable or disable adding a crosshair at the clicked location on every IDX image.

        Args:
            enable (bool): True to add a crosshair on image click, False to disable it.
        """
        pub_name = 'all_click_crosshair_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def add_crosshair_pixel(self, name, x_pixel, y_pixel, color_rgb = (0,255,0)):
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
    ## Collective Live Adjust Controls

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







