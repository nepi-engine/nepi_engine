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
from nepi_interfaces.msg import SaveDataRate, SaveDataStatus
from nepi_interfaces.msg import DeviceNPXStatus

from nepi_api.messages_if import MsgIF

from nepi_api.connect_device_if import ConnectDeviceIF
from nepi_api.connect_node_if import ConnectNodeClassIF



#########################################
# Connect IF Class
#########################################


CONNECT_ID='NPX'
CONNECT_STATUS_MSG='DeviceNPXStatus'
CONNECT_STATUS_MSG_CLASS=DeviceNPXStatus
CONNECT_NAME='npx_connect'


CONNECTED_TIMEOUT = 2


class ConnectNPXDeviceIF(ConnectDeviceIF):

    # ADD Additional Connect Callback Functions

    save_data_status_msg = None

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
                exclude_namespaces_list = None,
                selected_topic = None,
                statusCb = None,
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        ##############################
        # Initialize Class Variables
        # Everything this class owns is set before ConnectDeviceIF.__init__, which
        # starts the timers and subscriptions that reach it.

        # Controls Config Dict ####################
        # One publisher per device control topic. The 'unknown' namespace is the
        # sentinel ConnectDeviceIF replaces with the selected device on each
        # selection, and it prefixes each key with self.node_if_prefix.
        connect_topic_controls_dict = {
            'set_max_update_rate': {
                'namespace': 'unknown',
                'topic': 'set_max_update_rate',
                'msg': Float32,
                'qsize': 1,
            }
        }

        super().__init__(
                connect_id = CONNECT_ID,
                connect_name = connect_name,
                namespace = namespace,
                connect_status_msg = CONNECT_STATUS_MSG,
                connect_status_msg_class = CONNECT_STATUS_MSG_CLASS,
                exclude_namespaces_list = exclude_namespaces_list,
                auto_select_enabled = auto_select_enabled,
                selected_topic = selected_topic,
                statusCb = statusCb,
                connect_topic_controls_dict = connect_topic_controls_dict,
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
        if type(self) is ConnectNPXDeviceIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    def get_navpose_capabilities(self):
        """Return the navpose capability flags reported by the connected NPX device.

        Returns:
            dict: A dictionary with keys has_location, has_heading, has_orientation,
                has_position, has_altitude, has_depth, and has_pan_tilt, or None if no
                status has been received.
        """
        if self.status_msg is not None:
            return {
                'has_location': self.status_msg.has_location,
                'has_heading': self.status_msg.has_heading,
                'has_orientation': self.status_msg.has_orientation,
                'has_position': self.status_msg.has_position,
                'has_altitude': self.status_msg.has_altitude,
                'has_depth': self.status_msg.has_depth,
                'has_pan_tilt': self.status_msg.has_pan_tilt
            }

    def get_navpose_frame(self):
        """Return the navpose frame currently reported by the connected NPX device.

        Returns:
            str: The navpose frame identifier, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.navpose_frame



    def set_max_update_rate(self, update_rate):
        """Set the maximum navpose update rate of the NPX device.

        Args:
            update_rate (float): Desired maximum update rate in Hz.
        """
        pub_name = self.node_if_prefix + 'set_max_update_rate'
        msg = update_rate
        self.node_if.publish_pub(pub_name, msg)

    def set_navpose_frame(self, navpose_frame):
        """Set the navpose frame for the NPX device.

        The 3.4 NPX device subscribes no set_navpose_frame topic, so nothing is
        sent and a warning is logged.

        Args:
            navpose_frame (str): Desired navpose frame identifier string.
        """
        self.msg_if.pub_warn("Navpose frame changes are not supported by the connected NPX device: " + str(navpose_frame), throttle_s = 5.0)

    #################
    ## Save Data Functions

    def get_save_data_products(self):
        """Return the list of available save data products for this device.

        Returns:
            list: A list of data product identifiers supported by the save data interface,
                or an empty list if no status has been received.
        """
        # The device reports its save data products in its own status msg: NodeIF
        # copies save_data_if.get_data_products() into the data_products field.
        data_products = []
        if self.status_msg is not None:
            data_products = list(self.status_msg.data_products)
        return data_products

    def get_save_data_status_dict(self):
        """Return the current save data status as a dictionary.

        Returns:
            dict: A dictionary representation of the save data interface status,
                or None if no save data status has been received.
        """
        status_dict = None
        if self.save_data_status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.save_data_status_msg)
        return status_dict

    def save_data_enable_pub(self,enable):
        """Enable or disable data saving on the device.

        Args:
            enable (bool): True to enable data saving, False to disable it.
        """
        self.node_if.publish_pub(self.node_if_prefix + 'save_data_enable', enable)

    def save_data_prefix_pub(self,prefix):
        """Publish an updated filename prefix for saved data files.

        Args:
            prefix (str): The prefix string to prepend to saved data filenames.
        """
        self.node_if.publish_pub(self.node_if_prefix + 'save_data_prefix', prefix)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        """Publish an updated save rate for a data product.

        Args:
            rate_hz (float): Desired save rate in Hz.
            data_product (int, optional): Identifier for the specific data product to update.
                Defaults to SaveDataRate.ALL_DATA_PRODUCTS.
        """
        msg = SaveDataRate()
        msg.data_product = data_product
        msg.save_rate_hz = rate_hz
        self.node_if.publish_pub(self.node_if_prefix + 'save_data_rate', msg)

    def save_data_snapshot_pub(self):
        """Trigger a one-shot snapshot save of current data on the device.
        """
        self.node_if.publish_pub(self.node_if_prefix + 'save_data_snapshot', Empty())

    def save_data_factory_reset_pub(self):
        """Request a factory reset of the device's save data configuration.

        The device save data interface subscribes no factory reset topic (its one
        reset topic restores the last saved config), so nothing is sent and a
        warning is logged.
        """
        self.msg_if.pub_warn("Save data factory reset is not supported by the connected device", throttle_s = 5.0)

    ###############################
    # Class Private Methods
    ###############################

    def subscribe_topic(self, topic):
        success = super().subscribe_topic(topic)

        # Save data topics: the device's SaveDataIF sits at <node>/save_data.
        save_data_namespace = nepi_sdk.create_namespace(self.device_node_namespace, 'save_data')
        save_data_pubs_dict = {
            self.node_if_prefix + 'save_data_enable': {
                'namespace': save_data_namespace,
                'topic': 'save_data_enable',
                'msg': Bool,
                'qsize': 1,
            },
            self.node_if_prefix + 'save_data_prefix': {
                'namespace': save_data_namespace,
                'topic': 'save_data_prefix',
                'msg': String,
                'qsize': 1,
            },
            self.node_if_prefix + 'save_data_rate': {
                'namespace': save_data_namespace,
                'topic': 'save_data_rate',
                'msg': SaveDataRate,
                'qsize': 1,
            },
            self.node_if_prefix + 'save_data_snapshot': {
                'namespace': save_data_namespace,
                'topic': 'snapshot_trigger',
                'msg': Empty,
                'qsize': 1,
            }
        }
        save_data_subs_dict = {
            self.node_if_prefix + 'save_data_status_sub': {
                'namespace': save_data_namespace,
                'topic': 'status',
                'msg': SaveDataStatus,
                'qsize': 1,
                'callback': self._saveDataStatusCb
            }
        }
        self._registerConnectTopics(pubs_dict = save_data_pubs_dict, subs_dict = save_data_subs_dict)

        return success


    def unsubscribe_topic(self):
        success = super().unsubscribe_topic()
        self.save_data_status_msg = None
        return success


    def _saveDataStatusCb(self,status_msg):
        self.save_data_status_msg = status_msg

