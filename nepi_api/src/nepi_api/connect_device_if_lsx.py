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
from nepi_interfaces.msg import DeviceLSXStatus

from nepi_api.messages_if import MsgIF

from nepi_api.connect_device_if import ConnectDeviceIF
from nepi_api.connect_node_if import ConnectNodeClassIF



#########################################
# Connect IF Class
#########################################


CONNECT_ID='LSX'
CONNECT_STATUS_MSG='DeviceLSXStatus'
CONNECT_STATUS_MSG_CLASS=DeviceLSXStatus
CONNECT_NAME='lsx_connect'


CONNECTED_TIMEOUT = 2


class ConnectLSXDeviceIF(ConnectDeviceIF):

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
        # The device subscribes standby on 'set_empty'; set_standby mirrors that.
        connect_topic_controls_dict = {
            'set_standby': {
                'namespace': 'unknown',
                'topic': 'set_empty',
                'msg': Bool,
                'qsize': 1,
            },
            'turn_on_off': {
                'namespace': 'unknown',
                'topic': 'turn_on_off',
                'msg': Bool,
                'qsize': 1,
            },
            'set_intensity_ratio': {
                'namespace': 'unknown',
                'topic': 'set_intensity_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_color': {
                'namespace': 'unknown',
                'topic': 'set_color',
                'msg': String,
                'qsize': 1,
            },
            'set_kelvin': {
                'namespace': 'unknown',
                'topic': 'set_kelvin',
                'msg': Int32,
                'qsize': 1,
            },
            'set_blink_interval': {
                'namespace': 'unknown',
                'topic': 'set_blink_interval',
                'msg': Float32,
                'qsize': 1,
            },
            'set_strobe_enable': {
                'namespace': 'unknown',
                'topic': 'set_strobe_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_navpose_frame': {
                'namespace': 'unknown',
                'topic': 'set_navpose_frame',
                'msg': String,
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
        if type(self) is ConnectLSXDeviceIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    def check_on_off_state(self):
        """Return the current on/off state of the connected LSX device.

        Returns:
            bool: True if the light is on, False otherwise, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.on_off_state

    def check_standby_state(self):
        """Return the current standby state of the connected LSX device.

        Returns:
            bool: True if the device is in standby, False otherwise, or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.standby_state

    def get_intensity_ratio(self):
        """Return the current luminance intensity ratio of the connected LSX device.

        Returns:
            float: The intensity as a ratio from 0.0 to 1.0, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.intensity_ratio



    def set_standby(self, standby):
        """Enable or disable standby mode on the LSX device.

        Args:
            standby (bool): True to put the device in standby, False to resume normal operation.
        """
        # NOTE: device_if_lsx subscribes standby on the 'set_empty' topic; this
        # publisher intentionally mirrors that device topic so the command reaches
        # the device. See the controls dict in __init__.
        pub_name = self.node_if_prefix + 'set_standby'
        msg = standby
        self.node_if.publish_pub(pub_name, msg)

    def turn_on_off(self, turn_on):
        """Turn the LSX device output on or off.

        Args:
            turn_on (bool): True to turn the light on, False to turn it off.
        """
        pub_name = self.node_if_prefix + 'turn_on_off'
        msg = turn_on
        self.node_if.publish_pub(pub_name, msg)

    def set_intensity_ratio(self, intensity_ratio):
        """Set the luminance intensity of the LSX device.

        Args:
            intensity_ratio (float): Desired intensity as a ratio from 0.0 (off) to 1.0 (max).
        """
        pub_name = self.node_if_prefix + 'set_intensity_ratio'
        msg = intensity_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_color(self, color):
        """Set the color of the LSX device output.

        Args:
            color (str): Desired color setting string supported by the device.
        """
        pub_name = self.node_if_prefix + 'set_color'
        msg = color
        self.node_if.publish_pub(pub_name, msg)

    def set_kelvin(self, kelvin):
        """Set the color temperature of the LSX device output in kelvin.

        Args:
            kelvin (int): Desired color temperature in kelvin.
        """
        pub_name = self.node_if_prefix + 'set_kelvin'
        msg = kelvin
        self.node_if.publish_pub(pub_name, msg)

    def set_blink_interval(self, blink_interval):
        """Set the blink interval of the LSX device.

        Args:
            blink_interval (float): Desired blink interval in seconds.
        """
        pub_name = self.node_if_prefix + 'set_blink_interval'
        msg = blink_interval
        self.node_if.publish_pub(pub_name, msg)

    def set_strobe_enable(self, strobe_enable):
        """Enable or disable strobe mode on the LSX device.

        Args:
            strobe_enable (bool): True to enable strobe, False to disable it.
        """
        pub_name = self.node_if_prefix + 'set_strobe_enable'
        msg = strobe_enable
        self.node_if.publish_pub(pub_name, msg)

    def set_navpose_frame(self, navpose_frame):
        """Set the navpose mounting frame for the LSX device.

        Args:
            navpose_frame (str): Desired navpose frame identifier string.
        """
        pub_name = self.node_if_prefix + 'set_navpose_frame'
        msg = navpose_frame
        self.node_if.publish_pub(pub_name, msg)

    def reset_controls(self):
        """Reset the device controls to their default values.
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

