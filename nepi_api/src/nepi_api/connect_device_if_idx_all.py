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
from nepi_interfaces.msg import DeviceIDXStatus
from nepi_interfaces.msg import StringArray, ImageCrosshair

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF
from nepi_api.connect_node_if import ConnectNodeClassIF




#########################################
# Connect IF Class
#########################################


CONNECT_ID='IDX'
CONNECT_STATUS_MSG='DeviceIDXStatus'
CONNECT_NAME='idx_all_connect'


CONNECTED_TIMEOUT = 2


class ConnectIDXAllImagesIF(ConnectNodeIF):

    # ADD Additional Connect Callback Functions


    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    status_msg = None
    connected = False
    last_status_time = 0

    statusCb = None # Backwards Compatibility

    connect_topic_subs_dict = None
    connect_topic_pubs_dict = None
    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = CONNECT_NAME,
                namespace = None,
                statusCb = None,
                show_selector = True,
                show_controls = True,
                show_data = True,
                log_name = None,
                log_name_list = [],
                msg_if = None,
                node_if = None
                ):

        super().__init__(
                connect_id = CONNECT_ID,
                connect_status_msg = CONNECT_STATUS_MSG,
                connect_name = connect_name,
                selected_topic = namespace,
                auto_select_enabled = True,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = None,
                node_if = None
                )
        ####  IF INIT SETUP ####

        self.wait_for_connect_ready()



        ##############################
        # Initialize Class Variables

        self.statusCb = statusCb


        ##############################
        # Start updater process
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)

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
        return nepi_sdk.create_namespace(self.base_namespace, 'images')

    def check_connection(self):
        """Check whether at least one IDX device is currently connected.

        Returns:
            bool: True if a status message has been received within the connection timeout window,
                False otherwise.
        """
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until an IDX device is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if connection was established, False if the timeout was reached.
        """
        if self.node_if is not None and self.selected_topic != 'None':
            self.msg_if.pub_info("Waiting for connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to Connect")
            else:
                self.msg_if.pub_info("Connected")
        return self.connected


    def check_status_connection(self):
        """Check whether the status topic from an IDX device is currently connected.

        Returns:
            bool: True if status messages are being received, False otherwise.
        """
        return self.connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until an IDX device status topic is connected or the timeout expires.

        Args:
            timeout (float, optional): Maximum number of seconds to wait. Defaults to float('inf').

        Returns:
            bool: True if the status connection was established, False if the timeout was reached.
        """
        if self.node_if is not None and self.selected_topic != 'None':
            self.msg_if.pub_info("Waiting for status connection")
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.connected == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.connected == False:
                self.msg_if.pub_info("Failed to connect to status msg")
            else:
                self.msg_if.pub_info("Status Connected")
        return self.connected

    def get_status_dict(self):
        """Return the latest IDX device status as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent DeviceIDXStatus message,
                or None if no status has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_status_msg(self):
        """Return the latest IDX device status as a msg.

        Returns:
            dict: A msg representation of the most recent DeviceIDXStatus message,
                or None if no status has been received yet.
        """
        return self.status_msg

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

    def add_crosshair_pixel(self, name, x_pixel, y_pixel):
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
        self.node_if.publish_pub(pub_name, msg)

    def add_crosshair_degrees(self, name, x_offset_deg, y_offset_deg):
        """Add a named crosshair at a degree offset on every IDX image output.

        Args:
            name (str): Identifier for the crosshair.
            x_offset_deg (float): Horizontal degree offset of the crosshair from image center.
            y_offset_deg (float): Vertical degree offset of the crosshair from image center.
        """
        pub_name = 'all_add_crosshair_degrees'
        msg = ImageCrosshair()
        msg.name = name
        msg.x_offset_deg = x_offset_deg
        msg.y_offset_deg = y_offset_deg
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

    def updaterCb(self,timer):
        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time )
        if self.connected == True:
            if (cur_time - last_time) > CONNECTED_TIMEOUT:
                self.connected = False
                self.status_msg = None

        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)




    def subscribe_topic(self, topic):
        self.msg_if.pub_warn("subscribe_idx_all_topic Called")

        success = False
        success = self.unsubscribe_topic()

        # Collective controls publish on the shared images namespace, which fans a
        # single command out to every IDX image.
        images_namespace = nepi_sdk.create_namespace(self.base_namespace, 'images')

        # Subscribers Config Dict ####################
        self.connect_topic_subs_dict = {
            'status_sub': {
                'namespace': self.selected_topic,
                'topic': 'status',
                'msg': DeviceIDXStatus,
                'qsize': 10,
                'callback': self._statusCb
            }
        }



        # Publishers Config Dict ####################
        self.connect_topic_pubs_dict = {
            'all_overlay_size_ratio': {
                'namespace': images_namespace,
                'topic': 'set_overlay_size_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_overlay_source_name': {
                'namespace': images_namespace,
                'topic': 'set_overlay_source_name',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_date_time': {
                'namespace': images_namespace,
                'topic': 'set_overlay_date_time',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_nav': {
                'namespace': images_namespace,
                'topic': 'set_overlay_nav',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_pose': {
                'namespace': images_namespace,
                'topic': 'set_overlay_pose',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_text': {
                'namespace': images_namespace,
                'topic': 'add_overlay_text',
                'msg': String,
                'qsize': 1,
            },
            'all_overlay_list': {
                'namespace': images_namespace,
                'topic': 'set_overlay_list',
                'msg': StringArray,
                'qsize': 1,
            },
            'all_overlay_clear': {
                'namespace': images_namespace,
                'topic': 'clear_overlay_list',
                'msg': Empty,
                'qsize': 1,
            },
            'all_overlay_crosshairs': {
                'namespace': images_namespace,
                'topic': 'all_overlay_crosshairs',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_names': {
                'namespace': images_namespace,
                'topic': 'all_overlay_crosshair_names',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_pixels': {
                'namespace': images_namespace,
                'topic': 'all_overlay_crosshair_pixels',
                'msg': Bool,
                'qsize': 1,
            },
            'all_overlay_crosshair_degrees': {
                'namespace': images_namespace,
                'topic': 'overlay_crosshair_degrees',
                'msg': Bool,
                'qsize': 1,
            },
            'all_click_crosshair_enable': {
                'namespace': images_namespace,
                'topic': 'click_crosshair_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'all_add_crosshair_pixel': {
                'namespace': images_namespace,
                'topic': 'add_crosshair_pixel',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'all_add_crosshair_degrees': {
                'namespace': images_namespace,
                'topic': 'add_crosshair_degrees',
                'msg': ImageCrosshair,
                'qsize': 1,
            },
            'all_remove_crosshair': {
                'namespace': images_namespace,
                'topic': 'remove_crosshair',
                'msg': String,
                'qsize': 1,
            },
            'all_clear_crosshairs': {
                'namespace': images_namespace,
                'topic': 'clear_crosshairs',
                'msg': Empty,
                'qsize': 1,
            },
            'all_set_live_adjust_rotate_ratio': {
                'namespace': images_namespace,
                'topic': 'set_live_adjust_rotate_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_rotate_deg': {
                'namespace': images_namespace,
                'topic': 'set_live_adjust_rotate_deg',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_ratio': {
                'namespace': images_namespace,
                'topic': 'set_live_adjust_x_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_pixel': {
                'namespace': images_namespace,
                'topic': 'set_live_adjust_x_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            'all_set_live_adjust_x_deg': {
                'namespace': images_namespace,
                'topic': 'set_live_adjust_x_deg',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_ratio': {
                'namespace': images_namespace,
                'topic': 'set_live_adjust_y_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_pixel': {
                'namespace': images_namespace,
                'topic': 'set_live_adjust_y_pixel',
                'msg': Int32,
                'qsize': 1,
            },
            'all_set_live_adjust_y_deg': {
                'namespace': images_namespace,
                'topic': 'set_live_adjust_y_deg',
                'msg': Float32,
                'qsize': 1,
            }


        }

        if self.node_if is not None:
            self.node_if.register_pubs(self.connect_topic_pubs_dict)
            self.node_if.register_subs(self.connect_topic_subs_dict)
            self.connecting = True
            self.connected = False
            self.connected_topic = 'None'
            self.status_msg = None

        return success





    def unsubscribe_topic(self):
        success = False
        if self.connecting == True or self.connected == True:
            self.msg_if.pub_warn("unsubscribe_topic Called")

            if self.node_if is not None:
                if self.connect_topic_subs_dict is not None:
                    for sub_name in self.connect_topic_subs_dict.keys():
                        self.node_if.unregister_sub(sub_name)
            self.connect_topic_subs_dict = None

            if self.node_if is not None:
                if self.connect_topic_pubs_dict is not None:
                    for pub_name in self.connect_topic_pubs_dict.keys():
                        self.node_if.unregister_pub(pub_name)
            self.connect_topic_pubs_dict = None

            nepi_sdk.sleep(1)
            self.connecting = False
            self.connected = False
            self.connected_topic = 'None'
            self.status_msg = None
            success = True
        return success


    def _statusCb(self,status_msg):
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self.msg_if.pub_warn("Connected to IDX Status:  " + str(self.selected_topic))
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg

        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)
