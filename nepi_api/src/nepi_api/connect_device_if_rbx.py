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
from geographic_msgs.msg import GeoPoint

from nepi_interfaces.msg import SaveDataRate
from nepi_interfaces.msg import DeviceRBXStatus, DeviceRBXInfo
from nepi_interfaces.msg import MotorStatus, MotorsStatus
from nepi_interfaces.msg import ErrorBounds, MotorControl
from nepi_interfaces.msg import GotoPose, GotoPosition, GotoLocation

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF
from nepi_api.connect_node_if import ConnectNodeClassIF




#########################################
# Connect IF Class
#########################################


CONNECT_ID='RBX'
CONNECT_STATUS_MSG='DeviceRBXStatus'
CONNECT_NAME='rbx_connect'


CONNECTED_TIMEOUT = 2


class ConnectRBXDeviceIF(ConnectNodeIF):

    # ADD Additional Connect Callback Functions


    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    status_msg = None
    info_msg = None
    motors_status_msg = None
    connected = False
    last_status_time = 0

    statusCb = None # Backwards Compatibility
    infoCb = None # Backwards Compatibility
    motorStatusCb = None # Backwards Compatibility

    connect_topic_subs_dict = None
    connect_topic_pubs_dict = None
    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = CONNECT_NAME,
                namespace = None,
                statusCb = None,
                infoCb = None,
                motorStatusCb = None,
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
        self.infoCb = infoCb
        self.motorStatusCb = motorStatusCb


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
        """Return the fully-resolved ROS namespace for the connected RBX device.

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
            dict: A dictionary representation of the most recent DeviceRBXStatus message,
                or None if no status has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_status_msg(self):
        """Return the latest device status as a msg.

        Returns:
            DeviceRBXStatus: The most recent DeviceRBXStatus message,
                or None if no status has been received yet.
        """
        return self.status_msg

    def get_info_dict(self):
        """Return the latest device info as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent DeviceRBXInfo message,
                or None if no info has been received yet.
        """
        info_dict = None
        if self.info_msg is not None:
            info_dict = nepi_sdk.convert_msg2dict(self.info_msg)
        return info_dict

    def get_info_msg(self):
        """Return the latest device info as a msg.

        Returns:
            DeviceRBXInfo: The most recent DeviceRBXInfo message,
                or None if no info has been received yet.
        """
        return self.info_msg

    def get_motors_status_dict(self):
        """Return the latest multi-motor status as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent MotorsStatus message,
                or None if no motor status has been received yet.
        """
        motors_dict = None
        if self.motors_status_msg is not None:
            motors_dict = nepi_sdk.convert_msg2dict(self.motors_status_msg)
        return motors_dict

    def get_motors_status_msg(self):
        """Return the latest multi-motor status as a msg.

        Returns:
            MotorsStatus: The most recent MotorsStatus message,
                or None if no motor status has been received yet.
        """
        return self.motors_status_msg

    def get_state(self):
        """Return the index of the device's current state option.

        Returns:
            int: The enumerated state index reported in DeviceRBXInfo, or None if no
                info has been received.
        """
        if self.info_msg is not None:
            return self.info_msg.state

    def get_mode(self):
        """Return the index of the device's current mode option.

        Returns:
            int: The enumerated mode index reported in DeviceRBXInfo, or None if no
                info has been received.
        """
        if self.info_msg is not None:
            return self.info_msg.mode

    def get_home_location(self):
        """Return the device's configured home location.

        Returns:
            list: A three-element list [latitude, longitude, altitude_m] reported in
                DeviceRBXInfo, or None if no info has been received.
        """
        if self.info_msg is not None:
            return [self.info_msg.home_lat, self.info_msg.home_long, self.info_msg.home_alt]

    def get_cmd_timeout(self):
        """Return the goto command timeout in seconds.

        Returns:
            float: The command timeout reported in DeviceRBXInfo, or None if no info
                has been received.
        """
        if self.info_msg is not None:
            return self.info_msg.cmd_timeout

    def get_process_current(self):
        """Return a human-readable description of the current control process.

        Returns:
            str: The current process description ('None' if idle), or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.process_current

    def get_process_last(self):
        """Return a human-readable description of the last control process.

        Returns:
            str: The last process description ('None' if none), or None if no status
                has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.process_last

    def check_ready(self):
        """Check whether the device is ready to accept a new command.

        Returns:
            bool: True if the device is ready (not busy), False if a command is in
                progress, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.ready

    def get_battery(self):
        """Return the device battery charge state.

        Returns:
            float: Battery charge as a ratio from 0.0 to 1.0, or -999 if unavailable, or
                None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.battery

    def check_manual_controls_ready(self):
        """Check whether manual motor control mode is ready.

        Returns:
            bool: True if manual control mode is ready, False otherwise, or None if no
                status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.manual_control_mode_ready

    def check_autonomous_controls_ready(self):
        """Check whether autonomous control mode is ready.

        Returns:
            bool: True if autonomous control mode is ready, False otherwise, or None if
                no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.autonomous_control_mode_ready

    def check_cmd_success(self):
        """Check whether the last command control action succeeded.

        Returns:
            bool: True if the last command met its movement error bounds before timing
                out, False otherwise, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.cmd_success

    def get_errors_current(self):
        """Return the signed error values for the current control process.

        Returns:
            GotoErrors: The current error values, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.errors_current

    def get_errors_prev(self):
        """Return the signed error values for the previous control process.

        Returns:
            GotoErrors: The previous error values, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.errors_prev

    def get_motor_control_settings(self):
        """Return the current manual motor control settings.

        Returns:
            list: A list of MotorControl entries reporting the current per-motor speed
                ratios, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.current_motor_control_settings

    def get_last_cmd_string(self):
        """Return the last command string issued to the device.

        Returns:
            str: The last command string (suitable for an rbx automation script), or None
                if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.last_cmd_string

    def get_last_error_message(self):
        """Return the last error message reported by the device.

        Returns:
            str: The last error message string, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.last_error_message



    #################
    ## Control Commands

    def set_state(self, state_ind):
        """Command the RBX device to change to a state option by index.

        Args:
            state_ind (int): Index into the device's enumerated state options.
        """
        pub_name = 'set_state'
        msg = state_ind
        self.node_if.publish_pub(pub_name, msg)

    def set_mode(self, mode_ind):
        """Command the RBX device to change to a mode option by index.

        Args:
            mode_ind (int): Index into the device's enumerated mode options.
        """
        pub_name = 'set_mode'
        msg = mode_ind
        self.node_if.publish_pub(pub_name, msg)

    def setup_action(self, action_ind):
        """Command the RBX device to run a setup action by index.

        Args:
            action_ind (int): Index into the device's enumerated setup action options.
        """
        pub_name = 'setup_action'
        msg = action_ind
        self.node_if.publish_pub(pub_name, msg)

    def go_action(self, action_ind):
        """Command the RBX device to run a go action by index.

        Args:
            action_ind (int): Index into the device's enumerated go action options.
        """
        pub_name = 'go_action'
        msg = action_ind
        self.node_if.publish_pub(pub_name, msg)

    def set_goto_error_bounds(self, max_distance_error_m, max_rotation_error_deg, min_stabilize_time_s):
        """Set the goto command error bounds on the RBX device.

        Args:
            max_distance_error_m (float): Maximum allowed translation error in meters.
            max_rotation_error_deg (float): Maximum allowed rotation error in degrees.
            min_stabilize_time_s (float): Minimum time in seconds that error values must
                stay within bounds before a goal is considered reached.
        """
        pub_name = 'set_goto_error_bounds'
        msg = ErrorBounds()
        msg.max_distance_error_m = max_distance_error_m
        msg.max_rotation_error_deg = max_rotation_error_deg
        msg.min_stabilize_time_s = min_stabilize_time_s
        self.node_if.publish_pub(pub_name, msg)

    def set_goto_timeout(self, timeout_s):
        """Set the goto command timeout on the RBX device.

        Args:
            timeout_s (int): Command timeout in seconds.
        """
        pub_name = 'set_goto_timeout'
        msg = timeout_s
        self.node_if.publish_pub(pub_name, msg)

    def set_motor_control(self, motor_ind, speed_ratio):
        """Command a single motor's speed ratio in manual control mode.

        Args:
            motor_ind (int): Index of the motor to command.
            speed_ratio (float): Desired speed as a ratio from 0.0 (off) to 1.0 (max).
        """
        pub_name = 'set_motor_control'
        msg = MotorControl()
        msg.motor_ind = motor_ind
        msg.speed_ratio = speed_ratio
        self.node_if.publish_pub(pub_name, msg)

    def go_home(self):
        """Command the RBX device to move to its configured home location.
        """
        pub_name = 'go_home'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def set_home(self, latitude, longitude, altitude):
        """Set the home location for the RBX device.

        Args:
            latitude (float): Home latitude in degrees. Pass -999 to keep the current value.
            longitude (float): Home longitude in degrees. Pass -999 to keep the current value.
            altitude (float): Home altitude in meters. Pass -999 to keep the current value.
        """
        pub_name = 'set_home'
        msg = GeoPoint()
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        self.node_if.publish_pub(pub_name, msg)

    def set_home_current(self):
        """Set the home location to the RBX device's current location.
        """
        pub_name = 'set_home_current'
        msg = GotoLocation()
        self.node_if.publish_pub(pub_name, msg)

    def goto_location(self, lat, long, altitude_meters, yaw_deg):
        """Command the RBX device to move to an absolute geographic location.

        Args:
            lat (float): Target latitude in degrees. Pass -999 to keep the current value.
            long (float): Target longitude in degrees. Pass -999 to keep the current value.
            altitude_meters (float): Target altitude in meters. Pass -999 to keep the current value.
            yaw_deg (float): Target yaw in degrees (-180 to 180). Pass -999 to keep the current value.
        """
        pub_name = 'goto_location'
        msg = GotoLocation()
        msg.lat = lat
        msg.long = long
        msg.altitude_meters = altitude_meters
        msg.yaw_deg = yaw_deg
        self.node_if.publish_pub(pub_name, msg)

    def goto_position(self, x_meters, y_meters, z_meters, yaw_deg):
        """Command the RBX device to move to a relative body-frame position.

        Args:
            x_meters (float): Target forward offset in meters. Zero keeps the current position on this axis.
            y_meters (float): Target left offset in meters. Zero keeps the current position on this axis.
            z_meters (float): Target up offset in meters. Zero keeps the current position on this axis.
            yaw_deg (float): Target yaw in degrees (-180 to 180). Zero keeps the current yaw.
        """
        pub_name = 'goto_position'
        msg = GotoPosition()
        msg.x_meters = x_meters
        msg.y_meters = y_meters
        msg.z_meters = z_meters
        msg.yaw_deg = yaw_deg
        self.node_if.publish_pub(pub_name, msg)

    def goto_pose(self, roll_deg, pitch_deg, yaw_deg):
        """Command the RBX device to move to an absolute orientation pose.

        Args:
            roll_deg (float): Target roll in degrees (-180 to 180). Pass -999 to keep the current value.
            pitch_deg (float): Target pitch in degrees (-180 to 180). Pass -999 to keep the current value.
            yaw_deg (float): Target yaw in degrees (-180 to 180). Pass -999 to keep the current value.
        """
        pub_name = 'goto_pose'
        msg = GotoPose()
        msg.roll_deg = roll_deg
        msg.pitch_deg = pitch_deg
        msg.yaw_deg = yaw_deg
        self.node_if.publish_pub(pub_name, msg)

    def go_stop(self):
        """Command the RBX device to stop all motion.
        """
        pub_name = 'go_stop'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def set_image_topic(self, topic):
        """Set the image topic the RBX device associates with its status overlay.

        Args:
            topic (str): Partial or full ROS namespace string, or '' for a black image background.
        """
        pub_name = 'set_image_topic'
        msg = topic
        self.node_if.publish_pub(pub_name, msg)

    def enable_image_overlay(self, enable):
        """Enable or disable the status message overlay on the RBX device image.

        Args:
            enable (bool): True to overlay status on the image, False to disable it.
        """
        pub_name = 'enable_image_overlay'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_process_name(self, name):
        """Set the human-readable name of the current control process on the RBX device.

        Args:
            name (str): The process name to report in device status.
        """
        pub_name = 'set_process_name'
        msg = name
        self.node_if.publish_pub(pub_name, msg)

    def set_navpose_frame(self, navpose_frame):
        """Set the navpose frame for the RBX device.

        Args:
            navpose_frame (str): Desired navpose frame identifier string.
        """
        pub_name = 'set_navpose_frame'
        msg = navpose_frame
        self.node_if.publish_pub(pub_name, msg)

    def request_status_update(self):
        """Request that the RBX device immediately republish its status message.
        """
        pub_name = 'publish_status'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def request_info_update(self):
        """Request that the RBX device immediately republish its info message.
        """
        pub_name = 'publish_info'
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
                self.info_msg = None
                self.motors_status_msg = None

        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)




    def subscribe_topic(self, topic):
        self.msg_if.pub_warn("subscribe_rbx_topic Called")

        success = False
        success = self.unsubscribe_topic()

        # Subscribers Config Dict ####################
        self.connect_topic_subs_dict = {
            'status_sub': {
                'namespace': self.selected_topic,
                'topic': 'status',
                'msg': DeviceRBXStatus,
                'qsize': 10,
                'callback': self._statusCb
            },
            'info_sub': {
                'namespace': self.selected_topic,
                'topic': 'info',
                'msg': DeviceRBXInfo,
                'qsize': 1,
                'callback': self._infoCb,
                'callback_args': ()
            },
            'motor_status_sub': {
                'namespace': self.selected_topic,
                'topic': 'motor_status',
                'msg': MotorsStatus,
                'qsize': 10,
                'callback': self._motorStatusCb,
                'callback_args': ()
            }
        }



        # Publishers Config Dict ####################
        self.connect_topic_pubs_dict = {
            'set_state': {
                'namespace': self.selected_topic,
                'topic': 'set_state',
                'msg': Int32,
                'qsize': 1,
            },
            'set_mode': {
                'namespace': self.selected_topic,
                'topic': 'set_mode',
                'msg': Int32,
                'qsize': 1,
            },
            'setup_action': {
                'namespace': self.selected_topic,
                'topic': 'setup_action',
                'msg': Int32,
                'qsize': 1,
            },
            'go_action': {
                'namespace': self.selected_topic,
                'topic': 'go_action',
                'msg': Int32,
                'qsize': 1,
            },
            'set_goto_error_bounds': {
                'namespace': self.selected_topic,
                'topic': 'set_goto_error_bounds',
                'msg': ErrorBounds,
                'qsize': 1,
            },
            'set_goto_timeout': {
                'namespace': self.selected_topic,
                'topic': 'set_goto_timeout',
                'msg': UInt32,
                'qsize': 1,
            },
            'set_motor_control': {
                'namespace': self.selected_topic,
                'topic': 'set_motor_control',
                'msg': MotorControl,
                'qsize': 20,
            },
            'go_home': {
                'namespace': self.selected_topic,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
            },
            'set_home': {
                'namespace': self.selected_topic,
                'topic': 'set_home',
                'msg': GeoPoint,
                'qsize': 1,
            },
            'set_home_current': {
                'namespace': self.selected_topic,
                'topic': 'set_home_current',
                'msg': GotoLocation,
                'qsize': 1,
            },
            'goto_location': {
                'namespace': self.selected_topic,
                'topic': 'goto_location',
                'msg': GotoLocation,
                'qsize': 1,
            },
            'goto_position': {
                'namespace': self.selected_topic,
                'topic': 'goto_position',
                'msg': GotoPosition,
                'qsize': 1,
            },
            'goto_pose': {
                'namespace': self.selected_topic,
                'topic': 'goto_pose',
                'msg': GotoPose,
                'qsize': 1,
            },
            'go_stop': {
                'namespace': self.selected_topic,
                'topic': 'go_stop',
                'msg': Empty,
                'qsize': 1,
            },
            'set_image_topic': {
                'namespace': self.selected_topic,
                'topic': 'set_image_topic',
                'msg': String,
                'qsize': 1,
            },
            'enable_image_overlay': {
                'namespace': self.selected_topic,
                'topic': 'enable_image_overlay',
                'msg': Bool,
                'qsize': 1,
            },
            'set_process_name': {
                'namespace': self.selected_topic,
                'topic': 'set_process_name',
                'msg': String,
                'qsize': 1,
            },
            'set_navpose_frame': {
                'namespace': self.selected_topic,
                'topic': 'set_navpose_frame',
                'msg': String,
                'qsize': 1,
            },
            'publish_status': {
                'namespace': self.selected_topic,
                'topic': 'publish_status',
                'msg': Empty,
                'qsize': 1,
            },
            'publish_info': {
                'namespace': self.selected_topic,
                'topic': 'publish_info',
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
            self.info_msg = None
            self.motors_status_msg = None

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
            self.info_msg = None
            self.motors_status_msg = None
            success = True
        return success


    def _statusCb(self,status_msg):
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self.msg_if.pub_warn("Connected to RBX Status:  " + str(self.selected_topic))
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg

        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)

    def _infoCb(self,info_msg):
        self.info_msg = info_msg
        if self.infoCb is not None:
            info_dict = self.get_info_dict()
            self.infoCb(info_dict)

    def _motorStatusCb(self,motors_status_msg):
        self.motors_status_msg = motors_status_msg
        if self.motorStatusCb is not None:
            motors_dict = self.get_motors_status_dict()
            self.motorStatusCb(motors_dict)
