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
from nepi_interfaces.msg import SaveDataRate, SaveDataStatus, NavPosePanTilt
from nepi_interfaces.msg import DevicePTXStatus
from nepi_interfaces.msg import PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, SingleAxisTimedSpeedMove

from nepi_api.messages_if import MsgIF

from nepi_api.connect_device_if import ConnectDeviceIF
from nepi_api.connect_node_if import ConnectNodeClassIF



#########################################
# Connect IF Class
#########################################


CONNECT_ID='PTX'
CONNECT_STATUS_MSG='DevicePTXStatus'
CONNECT_STATUS_MSG_CLASS=DevicePTXStatus
CONNECT_NAME='ptx_connect'


CONNECTED_TIMEOUT = 2


class ConnectPTXDeviceIF(ConnectDeviceIF):

    # ADD Additional Connect Callback Functions

    save_data_status_msg = None
    navpose_msg = None
    pan_tilt_position = [0,0]
    last_pan_tilt_position = [0,0]
    pan_moving = False
    tilt_moving = False

    panTiltCb = None # Backwards Compatibility
    stopPanCb = None # Backwards Compatibility
    stopTiltCb = None # Backwards Compatibility

    speed_max_dps = 10

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
                exclude_namespaces_list = None,
                selected_topic = None,
                panTiltCb = None,
                stopPanCb = None,
                stopTiltCb = None,
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
        self.panTiltCb = panTiltCb
        self.stopPanCb = stopPanCb
        self.stopTiltCb = stopTiltCb

        # Controls Config Dict ####################
        # One publisher per device control topic. The 'unknown' namespace is the
        # sentinel ConnectDeviceIF replaces with the selected device on each
        # selection, and it prefixes each key with self.node_if_prefix.
        connect_topic_controls_dict = {
            'speed_ratio': {
                'namespace': 'unknown',
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'pan_speed_ratio': {
                'namespace': 'unknown',
                'topic': 'set_pan_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'tilt_speed_ratio': {
                'namespace': 'unknown',
                'topic': 'set_tilt_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'stop_moving': {
                'namespace': 'unknown',
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
            },
            'stop_pan': {
                'namespace': 'unknown',
                'topic': 'stop_pan',
                'msg': Empty,
                'qsize': 1,
            },
            'stop_tilt': {
                'namespace': 'unknown',
                'topic': 'stop_tilt',
                'msg': Empty,
                'qsize': 1,
            },
            'goto_to_position': {
                'namespace': 'unknown',
                'topic': 'goto_position',
                'msg': PanTiltPosition,
                'qsize': 1,
            },
            'goto_to_pan_position': {
                'namespace': 'unknown',
                'topic': 'goto_pan_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_to_tilt_position': {
                'namespace': 'unknown',
                'topic': 'goto_tilt_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_pan_ratio': {
                'namespace': 'unknown',
                'topic': 'goto_pan_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_tilt_ratio': {
                'namespace': 'unknown',
                'topic': 'goto_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'jog_timed_pan': {
                'namespace': 'unknown',
                'topic': 'jog_timed_pan',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
            },
            'jog_timed_tilt': {
                'namespace': 'unknown',
                'topic': 'jog_timed_tilt',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
            },
            'jog_timed_pan_speed_ratio': {
                'namespace': 'unknown',
                'topic': 'jog_timed_pan_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
            },
            'jog_timed_tilt_speed_ratio': {
                'namespace': 'unknown',
                'topic': 'jog_timed_tilt_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
            },
            'reverse_pan_enabled': {
                'namespace': 'unknown',
                'topic': 'set_reverse_pan_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'reverse_tilt_enabled': {
                'namespace': 'unknown',
                'topic': 'set_reverse_tilt_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_soft_limits': {
                'namespace': 'unknown',
                'topic': 'set_soft_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
            },
            'go_home': {
                'namespace': 'unknown',
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
            },
            'set_home_position': {
                'namespace': 'unknown',
                'topic': 'set_home_position',
                'msg': PanTiltPosition,
                'qsize': 1,
            },
            'set_home_position_here': {
                'namespace': 'unknown',
                'topic': 'set_home_position_here',
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
        if type(self) is ConnectPTXDeviceIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    # def get_navpose_dict(self):
    #     navpose_dict = None
    #     if self.navpose_msg is not None:
    #         navpose_dict = nepi_nav.convert_navpose_msg2dict(self.navpose_msg)
    #     return navpose_dict
    
    def get_pan_tilt_hard_limits(self):
        """Return the hardware hardstop limits for pan and tilt axes.

        Returns:
            list: A four-element list [pan_min_deg, pan_max_deg, tilt_min_deg, tilt_max_deg]
                representing the physical hardstop limits in degrees, or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.pan_min_hardstop_deg, self.status_msg.pan_max_hardstop_deg, self.status_msg.tilt_min_hardstop_deg, self.status_msg.tilt_max_hardstop_deg]
    
    def get_pan_tilt_soft_limits(self):
        """Return the software softstop limits for pan and tilt axes.

        Returns:
            list: A four-element list [pan_min_deg, pan_max_deg, tilt_min_deg, tilt_max_deg]
                representing the software softstop limits in degrees, or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.pan_min_softstop_deg, self.status_msg.pan_max_softstop_deg, self.status_msg.tilt_min_softstop_deg, self.status_msg.tilt_max_softstop_deg]

    def get_pan_tilt_max_speed_dps(self):
        """Return the maximum pan/tilt speed in degrees per second.

        Returns:
            float: The max speed the device last reported, which is what a speed ratio
                of 1.0 means, or the class default before any status has arrived.
        """
        return self.speed_max_dps
    

    def get_pan_tilt_position(self):
        """Return the most recently reported pan and tilt position.

        Returns:
            list: A two-element list [pan_deg, tilt_deg] with the current position in degrees.
        """
        return [self.pan_tilt_position[0], self.pan_tilt_position[1]]

    def check_pan_moving(self):
        """Check whether the pan axis is currently in motion.

        Returns:
            bool: True if the pan axis has moved more than 0.1 degrees since the last update
                cycle, False otherwise.
        """
        return self.pan_moving
    
    def check_tilt_moving(self):
        """Check whether the tilt axis is currently in motion.

        Returns:
            bool: True if the tilt axis has moved more than 0.1 degrees since the last update
                cycle, False otherwise.
        """
        return self.tilt_moving



    def set_speed_ratio(self, speed_ratio):
        """Publish a speed ratio command to the PTX device.

        Args:
            speed_ratio (float): Desired motion speed as a ratio from 0.0 (slowest) to 1.0 (fastest).
        """
        pub_name = self.node_if_prefix + 'speed_ratio'
        msg = speed_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_pan_speed_ratio(self, speed_ratio):
        """Publish a pan-axis speed ratio command to a device that supports separate pan/tilt speed control.

        Args:
            speed_ratio (float): Desired pan speed as a ratio from 0.0 (slowest) to 1.0 (fastest).
        """
        pub_name = self.node_if_prefix + 'pan_speed_ratio'
        msg = speed_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_tilt_speed_ratio(self, speed_ratio):
        """Publish a tilt-axis speed ratio command to a device that supports separate pan/tilt speed control.

        Args:
            speed_ratio (float): Desired tilt speed as a ratio from 0.0 (slowest) to 1.0 (fastest).
        """
        pub_name = self.node_if_prefix + 'tilt_speed_ratio'
        msg = speed_ratio
        self.node_if.publish_pub(pub_name, msg)

    def stop_moving(self):
        """Publish a stop command to halt all motion on the PTX device.
        """
        pub_name = self.node_if_prefix + 'stop_moving'
        msg = Empty()
        self.node_if.publish_pub(pub_name,msg)

    def stop_pan(self):
        """Publish a stop command to halt pan-axis motion on the PTX device.
        """
        pub_name = self.node_if_prefix + 'stop_pan'
        msg = Empty()
        self.node_if.publish_pub(pub_name,msg)

    def stop_tilt(self):
        """Publish a stop command to halt tilt-axis motion on the PTX device.
        """
        pub_name = self.node_if_prefix + 'stop_tilt'
        msg = Empty()
        self.node_if.publish_pub(pub_name,msg)


    def goto_to_position(self,pan_deg,tilt_deg):
        """Command the PTX device to move to an absolute pan and tilt position.

        Args:
            pan_deg (float): Target pan angle in degrees.
            tilt_deg (float): Target tilt angle in degrees.
        """
        pub_name = self.node_if_prefix + 'goto_to_position'
        msg = PanTiltPosition()
        msg.pan_deg = pan_deg
        msg.tilt_deg = tilt_deg
        self.node_if.publish_pub(pub_name,msg)

    def goto_to_pan_position(self,pan_position):
        """Command the PTX device to move the pan axis to an absolute position.

        Args:
            pan_position (float): Target pan angle in degrees.
        """
        #self.msg_if.pub_warn("connect goto pan pos: " + str(pan_position))

        pub_name = self.node_if_prefix + 'goto_to_pan_position'
        msg = pan_position
        self.node_if.publish_pub(pub_name,msg)
        
    def goto_to_tilt_position(self,tilt_position):
        """Command the PTX device to move the tilt axis to an absolute position.

        Args:
            tilt_position (float): Target tilt angle in degrees.
        """
        pub_name = self.node_if_prefix + 'goto_to_tilt_position'
        msg = tilt_position
        self.node_if.publish_pub(pub_name,msg)

    def goto_pan_ratio(self,pan_ratio):
        """Command the PTX device to move the pan axis to a normalized ratio position.

        Args:
            pan_ratio (float): Target pan position as a ratio from 0.0 to 1.0, where 0.0
                corresponds to the minimum softstop and 1.0 to the maximum softstop.
        """
        pub_name = self.node_if_prefix + 'goto_pan_ratio'
        msg = pan_ratio
        self.node_if.publish_pub(pub_name,msg)        

    def goto_tilt_ratio(self, tilt_ratio):
        """Command the PTX device to move the tilt axis to a normalized ratio position.

        Args:
            tilt_ratio (float): Target tilt position as a ratio from 0.0 to 1.0, where 0.0
                corresponds to the minimum softstop and 1.0 to the maximum softstop.
        """
        pub_name = self.node_if_prefix + 'goto_tilt_ratio'
        msg = tilt_ratio
        self.node_if.publish_pub(pub_name,msg)

    def jog_timed_pan(self, direction, duration_s = -1):
        """Command the PTX device to jog the pan axis in a direction for a set duration.

        Args:
            direction (int): Direction indicator for the pan jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = self.node_if_prefix + 'jog_timed_pan'
        msg = SingleAxisTimedMove()
        # Direction indicator
        msg.direction = direction
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.node_if.publish_pub(pub_name,msg)
        
    def jog_timed_tilt(self, direction, duration_s = -1):
        """Command the PTX device to jog the tilt axis in a direction for a set duration.

        Args:
            direction (int): Direction indicator for the tilt jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = self.node_if_prefix + 'jog_timed_tilt'
        msg = SingleAxisTimedMove()
        # Direction indicator
        msg.direction = direction
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.node_if.publish_pub(pub_name,msg) 

    def jog_timed_speed_ratio_pan(self, direction, speed_ratio = 1, duration_s = -1):
        """Command the PTX device to jog the pan axis in a direction at speed_ratio for a set duration.

        Args:
            direction (int): Direction indicator for the pan jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = self.node_if_prefix + 'jog_timed_pan_speed_ratio'
        msg = SingleAxisTimedSpeedMove()
        # Direction indicator
        msg.direction = direction
        msg.speed_ratio = speed_ratio
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.node_if.publish_pub(pub_name,msg)
        
    def jog_timed_speed_ratio_tilt(self, direction, speed_ratio = 1, duration_s = -1):
        """Command the PTX device to jog the tilt axis in a direction speed_ratio for a set duration.

        Args:
            direction (int): Direction indicator for the tilt jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = self.node_if_prefix + 'jog_timed_tilt_speed_ratio'
        msg = SingleAxisTimedSpeedMove()
        # Direction indicator
        msg.direction = direction
        msg.speed_ratio = speed_ratio
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.node_if.publish_pub(pub_name,msg) 


    def jog_timed_speed_dps_pan(self, direction, speed_dps = 1, duration_s = -1):
        """Command the PTX device to jog the pan axis in a direction at speed_dps for a set duration.

        Args:
            direction (int): Direction indicator for the pan jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = self.node_if_prefix + 'jog_timed_pan_speed_ratio'
        msg = SingleAxisTimedSpeedMove()
        # Direction indicator
        msg.direction = direction

        if speed_dps < 0:
            speed_dps = 0
        if speed_dps > self.speed_max_dps:
            speed_dps = self.speed_max_dps
        msg.speed_ratio = speed_dps / self.speed_max_dps
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.node_if.publish_pub(pub_name,msg)
        
    def jog_timed_speed_dps_tilt(self, direction, speed_dps = 1, duration_s = -1):
        """Command the PTX device to jog the tilt axis in a direction speed_dps for a set duration.

        Args:
            direction (int): Direction indicator for the tilt jog (positive or negative).
            duration_s (float): Duration of the jog in seconds. Pass -1.0 for an indefinite move.
        """
        pub_name = self.node_if_prefix + 'jog_timed_tilt_speed_ratio'
        msg = SingleAxisTimedSpeedMove()
        # Direction indicator
        msg.direction = direction
        if speed_dps < 0:
            speed_dps = 0
        if speed_dps > self.speed_max_dps:
            speed_dps = self.speed_max_dps
        msg.speed_ratio = speed_dps / self.speed_max_dps
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.node_if.publish_pub(pub_name,msg) 


    def reverse_pan_enabled(self, reverse_pan):
        """Enable or disable pan direction reversal on the PTX device.

        Args:
            reverse_pan (bool): True to reverse the pan axis direction, False for normal direction.
        """
        pub_name = self.node_if_prefix + 'reverse_pan_enabled'
        msg = reverse_pan
        self.node_if.publish_pub(pub_name,msg)

    def reverse_tilt_enabled(self, reverse_tilt):
        """Enable or disable tilt direction reversal on the PTX device.

        Args:
            reverse_tilt (bool): True to reverse the tilt axis direction, False for normal direction.
        """
        pub_name = self.node_if_prefix + 'reverse_tilt_enabled'
        msg = reverse_tilt
        self.node_if.publish_pub(pub_name,msg)

    def go_home(self):
        """Command the PTX device to move to its configured home position.
        """
        pub_name = self.node_if_prefix + 'go_home'
        msg = Empty()
        self.node_if.publish_pub(pub_name,msg)

    def set_home_position(self,pan_deg,tilt_deg):
        """Set the home position for the PTX device to specified pan and tilt angles.

        Args:
            pan_deg (float): Desired home pan angle in degrees.
            tilt_deg (float): Desired home tilt angle in degrees.
        """
        pub_name = self.node_if_prefix + 'set_home_position'
        msg = PanTiltPosition()
        msg.pan_deg = pan_deg
        msg.tilt_deg = tilt_deg
        self.node_if.publish_pub(pub_name,msg) 

    def set_home_position_here(self):
        """Set the home position to the PTX device's current position.
        """
        pub_name = self.node_if_prefix + 'set_home_position_here'
        msg = Empty()
        self.node_if.publish_pub(pub_name,msg)  

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

    def save_data_reset_pub(self):
        """Publish a reset command that restores the device's last saved save data configuration.
        """
        self.node_if.publish_pub(self.node_if_prefix + 'save_data_reset', Empty())

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
   
    def updaterCb(self,timer):
        if self.connected == True:
            self.pan_moving = abs(self.pan_tilt_position[0] - self.last_pan_tilt_position[0]) > 0.1
            self.tilt_moving = abs(self.pan_tilt_position[1] - self.last_pan_tilt_position[1]) > 0.1
            self.last_pan_tilt_position = [self.pan_tilt_position[0], self.pan_tilt_position[1]]
        else:
            self.navpose_msg = None
            self.pan_moving = False
            self.tilt_moving = False
        super().updaterCb(timer)


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
            },
            self.node_if_prefix + 'save_data_reset': {
                'namespace': save_data_namespace,
                'topic': 'reset_save_data',
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
            },
            # No 3.4 PTX device publishes these two; kept so a device that does
            # still reaches stopPanCb and stopTiltCb.
            self.node_if_prefix + 'stop_pan_callback': {
                'namespace': self.selected_topic,
                'topic': 'stop_pan_callback',
                'msg': Empty,
                'qsize': 5,
                'callback': self._stopPanCb,
                'callback_args': ()
            },
            self.node_if_prefix + 'stop_tilt_callback': {
                'namespace': self.selected_topic,
                'topic': 'stop_tilt_callback',
                'msg': Empty,
                'qsize': 5,
                'callback': self._stopTiltCb,
                'callback_args': ()
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


    def _statusCb(self,status_msg):
        # Cache only a usable max: the jog_timed_speed_dps_* methods divide by it.
        if status_msg.speed_max_dps > 0:
            self.speed_max_dps = status_msg.speed_max_dps
        # 3.4 PTX devices publish no pan_tilt topic; the position rides in the status.
        self.pan_tilt_position = [status_msg.pan_now_deg, status_msg.tilt_now_deg]
        if self.panTiltCb is not None:
            self.panTiltCb(self.pan_tilt_position[0], self.pan_tilt_position[1])
        super()._statusCb(status_msg)


    def _stopPanCb(self,msg):    
        #self.msg_if.pub_warn("Got Stop Pan msg: " + str(msg))
        if self.stopPanCb is not None:
            self.stopPanCb()

    def _stopTiltCb(self,msg):    
        #self.msg_if.pub_warn("Got Stop Tilt msg: " + str(msg))
        if self.stopTiltCb is not None:
            self.stopTiltCb()