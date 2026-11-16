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
from nepi_interfaces.msg import DeviceIDXStatus
from nepi_interfaces.msg import RangeWindow


from nepi_api.messages_if import MsgIF

from nepi_api.connect_device_if import ConnectDeviceIF
from nepi_api.connect_node_if import ConnectNodeClassIF



#########################################
# Connect IF Class
#########################################


CONNECT_ID='IDX'
CONNECT_STATUS_MSG='DeviceIDXStatus'
CONNECT_STATUS_MSG_CLASS=DeviceIDXStatus
CONNECT_NAME='idx_connect'


CONNECTED_TIMEOUT = 2


class ConnectIDXDeviceIF(ConnectDeviceIF):

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
            'disable': {
                'namespace': 'unknown',
                'topic': 'disable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_width_deg': {
                'namespace': 'unknown',
                'topic': 'set_width_deg',
                'msg': Int32,
                'qsize': 1,
            },
            'set_height_deg': {
                'namespace': 'unknown',
                'topic': 'set_height_deg',
                'msg': Int32,
                'qsize': 1,
            },
            'set_auto_adjust_enable': {
                'namespace': 'unknown',
                'topic': 'set_auto_adjust_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_brightness_ratio': {
                'namespace': 'unknown',
                'topic': 'set_brightness_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_contrast_ratio': {
                'namespace': 'unknown',
                'topic': 'set_contrast_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_threshold_ratio': {
                'namespace': 'unknown',
                'topic': 'set_threshold_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_resolution_ratio': {
                'namespace': 'unknown',
                'topic': 'set_resolution_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_max_framerate': {
                'namespace': 'unknown',
                'topic': 'set_max_framerate',
                'msg': Float32,
                'qsize': 1,
            },
            'set_range_window': {
                'namespace': 'unknown',
                'topic': 'set_range_window',
                'msg': RangeWindow,
                'qsize': 1,
            },
            'reset_controls': {
                'namespace': 'unknown',
                'topic': 'reset_controls',
                'msg': Empty,
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
        if type(self) is ConnectIDXDeviceIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    def get_data_products(self):
        """Return the list of data products the connected IDX device reports.

        Returns:
            list: The data product identifiers from the latest status, or an empty list
                if no status has been received.
        """
        if self.status_msg is not None:
            return list(self.status_msg.data_products)
        return []

    def get_range_limits_m(self):
        """Return the current and adjusted range limits in meters for the device.

        Returns:
            list: A four-element list [min_range_m, max_range_m, min_range_m_adj, max_range_m_adj],
                or None if no status has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.min_range_m, self.status_msg.max_range_m,
                    self.status_msg.min_range_m_adj, self.status_msg.max_range_m_adj]



    def set_disable(self, disable):
        """Enable or disable the IDX device data output.

        Args:
            disable (bool): True to disable the device, False to enable it.
        """
        pub_name = self.node_if_prefix + 'disable'
        msg = disable
        self.node_if.publish_pub(pub_name, msg)

    def set_width_deg(self, width_deg):
        """Set the horizontal field of view of the device output in degrees.

        Args:
            width_deg (int): Desired output width in degrees.
        """
        pub_name = self.node_if_prefix + 'set_width_deg'
        msg = width_deg
        self.node_if.publish_pub(pub_name, msg)

    def set_height_deg(self, height_deg):
        """Set the vertical field of view of the device output in degrees.

        Args:
            height_deg (int): Desired output height in degrees.
        """
        pub_name = self.node_if_prefix + 'set_height_deg'
        msg = height_deg
        self.node_if.publish_pub(pub_name, msg)

    def set_auto_adjust_enable(self, enable):
        """Enable or disable automatic image adjustment on the device.

        Args:
            enable (bool): True to enable auto adjust, False to disable it.
        """
        pub_name = self.node_if_prefix + 'set_auto_adjust_enable'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_brightness_ratio(self, brightness_ratio):
        """Set the brightness of the device output.

        Args:
            brightness_ratio (float): Desired brightness as a ratio from 0.0 to 1.0.
        """
        pub_name = self.node_if_prefix + 'set_brightness_ratio'
        msg = brightness_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_contrast_ratio(self, contrast_ratio):
        """Set the contrast of the device output.

        Args:
            contrast_ratio (float): Desired contrast as a ratio from 0.0 to 1.0.
        """
        pub_name = self.node_if_prefix + 'set_contrast_ratio'
        msg = contrast_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_threshold_ratio(self, threshold_ratio):
        """Set the thresholding level of the device output.

        Args:
            threshold_ratio (float): Desired threshold as a ratio from 0.0 to 1.0.
        """
        pub_name = self.node_if_prefix + 'set_threshold_ratio'
        msg = threshold_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_resolution_ratio(self, resolution_ratio):
        """Set the output resolution of the device.

        Args:
            resolution_ratio (float): Desired resolution as a ratio from 0.0 (lowest) to 1.0 (highest).
        """
        pub_name = self.node_if_prefix + 'set_resolution_ratio'
        msg = resolution_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_max_framerate(self, max_framerate):
        """Set the maximum output framerate of the device in frames per second.

        Args:
            max_framerate (float): Desired maximum framerate in Hz.
        """
        pub_name = self.node_if_prefix + 'set_max_framerate'
        msg = max_framerate
        self.node_if.publish_pub(pub_name, msg)

    def set_range_window(self, start_range, stop_range):
        """Set the range window of the device as start and stop range ratios.

        Args:
            start_range (float): Start of the range window as a ratio of max range (0.0 - 1.0).
            stop_range (float): Stop of the range window as a ratio of max range (0.0 - 1.0).
        """
        pub_name = self.node_if_prefix + 'set_range_window'
        msg = RangeWindow()
        msg.start_range = start_range
        msg.stop_range = stop_range
        self.node_if.publish_pub(pub_name, msg)

    def reset_controls(self):
        """Reset the device image and data controls to their default values.
        """
        pub_name = self.node_if_prefix + 'reset_controls'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

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

