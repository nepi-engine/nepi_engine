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
from nepi_interfaces.msg import SaveDataRate
from nepi_interfaces.msg import DeviceLSXStatus

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF
from nepi_api.connect_node_if import ConnectNodeClassIF




#########################################
# Connect IF Class
#########################################


CONNECT_ID='LSX'
CONNECT_STATUS_MSG='DeviceLSXStatus'
CONNECT_NAME='lsx_connect'


CONNECTED_TIMEOUT = 2


class ConnectLSXDeviceIF(ConnectNodeIF):

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
        """Return the fully-resolved ROS namespace for the connected LSX device.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.selected_topic

    def check_connection(self):
        """Check whether the device is currently connected.

        Returns:
            bool: True if a status message has been received within the connection timeout window,
                False otherwise.
        """
        return self.connected

    def wait_for_connection(self, timeout = float('inf') ):
        """Block until the device is connected or the timeout expires.

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
        """Check whether the status topic from the device is currently connected.

        Returns:
            bool: True if status messages are being received, False otherwise.
        """
        return self.connected

    def wait_for_status_connection(self, timeout = float('inf') ):
        """Block until the device status topic is connected or the timeout expires.

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
        """Return the latest device status as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent DeviceLSXStatus message,
                or None if no status has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_status_msg(self):
        """Return the latest device status as a msg.

        Returns:
            dict: A msg representation of the most recent DeviceLSXStatus message,
                or None if no status has been received yet.
        """
        return self.status_msg

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
        # the device. See the connect pubs dict below.
        pub_name = 'set_standby'
        msg = standby
        self.node_if.publish_pub(pub_name, msg)

    def turn_on_off(self, turn_on):
        """Turn the LSX device output on or off.

        Args:
            turn_on (bool): True to turn the light on, False to turn it off.
        """
        pub_name = 'turn_on_off'
        msg = turn_on
        self.node_if.publish_pub(pub_name, msg)

    def set_intensity_ratio(self, intensity_ratio):
        """Set the luminance intensity of the LSX device.

        Args:
            intensity_ratio (float): Desired intensity as a ratio from 0.0 (off) to 1.0 (max).
        """
        pub_name = 'set_intensity_ratio'
        msg = intensity_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_color(self, color):
        """Set the color of the LSX device output.

        Args:
            color (str): Desired color setting string supported by the device.
        """
        pub_name = 'set_color'
        msg = color
        self.node_if.publish_pub(pub_name, msg)

    def set_kelvin(self, kelvin):
        """Set the color temperature of the LSX device output in kelvin.

        Args:
            kelvin (int): Desired color temperature in kelvin.
        """
        pub_name = 'set_kelvin'
        msg = kelvin
        self.node_if.publish_pub(pub_name, msg)

    def set_blink_interval(self, blink_interval):
        """Set the blink interval of the LSX device.

        Args:
            blink_interval (float): Desired blink interval in seconds.
        """
        pub_name = 'set_blink_interval'
        msg = blink_interval
        self.node_if.publish_pub(pub_name, msg)

    def set_strobe_enable(self, strobe_enable):
        """Enable or disable strobe mode on the LSX device.

        Args:
            strobe_enable (bool): True to enable strobe, False to disable it.
        """
        pub_name = 'set_strobe_enable'
        msg = strobe_enable
        self.node_if.publish_pub(pub_name, msg)

    def set_navpose_frame(self, navpose_frame):
        """Set the navpose mounting frame for the LSX device.

        Args:
            navpose_frame (str): Desired navpose frame identifier string.
        """
        pub_name = 'set_navpose_frame'
        msg = navpose_frame
        self.node_if.publish_pub(pub_name, msg)

    def reset_controls(self):
        """Reset the device controls to their default values.
        """
        pub_name = 'reset_controls'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def save_config(self):
        """Publish a save configuration command to persist current settings on the device.
        """
        self.node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        """Publish a reset configuration command to restore the last saved settings on the device.
        """
        self.node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        """Publish a factory reset command to restore factory default settings on the device.
        """
        self.node_if.publish_pub('factory_reset_config',Empty())

    #################
    ## Save Data Functions

    def get_save_data_products(self):
        """Return the list of available save data products for this device.

        Returns:
            list: A list of data product identifiers supported by the save data interface.
        """
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_save_data_status_dict(self):
        """Return the current save data status as a dictionary.

        Returns:
            dict: A dictionary representation of the save data interface status.
        """
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def save_data_enable_pub(self,enable):
        """Enable or disable data saving on the device.

        Args:
            enable (bool): True to enable data saving, False to disable it.
        """
        self.con_save_data_if.save_data_pub(enable)

    def save_data_prefix_pub(self,prefix):
        """Publish an updated filename prefix for saved data files.

        Args:
            prefix (str): The prefix string to prepend to saved data filenames.
        """
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def save_data_rate_pub(self,rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        """Publish an updated save rate for a data product.

        Args:
            rate_hz (float): Desired save rate in Hz.
            data_product (int, optional): Identifier for the specific data product to update.
                Defaults to SaveDataRate.ALL_DATA_PRODUCTS.
        """
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def save_data_snapshot_pub(self):
        """Trigger a one-shot snapshot save of current data on the device.
        """
        self.con_save_data_if.publish_pub()

    def save_data_factory_reset_pub(self):
        """Publish a factory reset command to restore the save data configuration to defaults.
        """
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_save_data_if.publish_pub(pub_name,msg)

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
        self.msg_if.pub_warn("subscribe_lsx_topic Called")

        success = False
        success = self.unsubscribe_topic()

        # Subscribers Config Dict ####################
        self.connect_topic_subs_dict = {
            'status_sub': {
                'namespace': self.selected_topic,
                'topic': 'status',
                'msg': DeviceLSXStatus,
                'qsize': 10,
                'callback': self._statusCb
            }
        }



        # Publishers Config Dict ####################
        self.connect_topic_pubs_dict = {
            'set_standby': {
                'namespace': self.selected_topic,
                'topic': 'set_empty',
                'msg': Bool,
                'qsize': 1,
            },
            'turn_on_off': {
                'namespace': self.selected_topic,
                'topic': 'turn_on_off',
                'msg': Bool,
                'qsize': 1,
            },
            'set_intensity_ratio': {
                'namespace': self.selected_topic,
                'topic': 'set_intensity_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_color': {
                'namespace': self.selected_topic,
                'topic': 'set_color',
                'msg': String,
                'qsize': 1,
            },
            'set_kelvin': {
                'namespace': self.selected_topic,
                'topic': 'set_kelvin',
                'msg': Int32,
                'qsize': 1,
            },
            'set_blink_interval': {
                'namespace': self.selected_topic,
                'topic': 'set_blink_interval',
                'msg': Float32,
                'qsize': 1,
            },
            'set_strobe_enable': {
                'namespace': self.selected_topic,
                'topic': 'set_strobe_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_navpose_frame': {
                'namespace': self.selected_topic,
                'topic': 'set_navpose_frame',
                'msg': String,
                'qsize': 1,
            },
            'reset_controls': {
                'namespace': self.selected_topic,
                'topic': 'reset_controls',
                'msg': Empty,
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
            self.msg_if.pub_warn("Connected to LSX Status:  " + str(self.selected_topic))
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg

        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)
