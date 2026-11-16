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
from geometry_msgs.msg import Twist

from nepi_interfaces.msg import SaveDataRate, SaveDataStatus
# 3.4.0 PORT: DeviceRBXInfo is gone; its fields ride in DeviceRBXStatus, see report
from nepi_interfaces.msg import DeviceRBXStatus
# 3.4.0 PORT: motors are per-motor SVX, see report
#from nepi_interfaces.msg import MotorStatus, MotorsStatus
from nepi_interfaces.msg import ErrorBounds
from nepi_interfaces.msg import GotoPose, GotoPosition, GotoLocation, GotoVelocity

from nepi_interfaces.srv import DeviceRBXStatusQuery, DeviceRBXStatusQueryRequest, DeviceRBXStatusQueryResponse

from nepi_api.messages_if import MsgIF

from nepi_api.connect_device_if import ConnectDeviceIF
from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.connect_node_if import ConnectNodeServicesIF



#########################################
# Connect IF Class
#########################################


CONNECT_ID='RBX'
CONNECT_STATUS_MSG='DeviceRBXStatus'
CONNECT_STATUS_MSG_CLASS=DeviceRBXStatus
CONNECT_NAME='rbx_connect'


CONNECTED_TIMEOUT = 2


class ConnectRBXDeviceIF(ConnectDeviceIF):

    # ADD Additional Connect Callback Functions

    info_msg = None
    motors_status_msg = None
    save_data_status_msg = None

    infoCb = None # Backwards Compatibility
    motorStatusCb = None # Backwards Compatibility

    connect_topic_srvs_dict = None
    con_services_if = None

    dataCB = None

    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = CONNECT_NAME,
                namespace = None,
                auto_select_enabled = True,
                exclude_namespaces_list = None,
                selected_topic = None,
                statusCb = None,
                infoCb = None,
                dataCB = None,
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
        self.infoCb = infoCb
        self.dataCB = dataCB

        # Controls Config Dict ####################
        # One publisher per device control topic. The 'unknown' namespace is the
        # sentinel ConnectDeviceIF replaces with the selected device on each
        # selection, and it prefixes each key with self.node_if_prefix.
        connect_topic_controls_dict = {
            'set_state': {
                'namespace': 'unknown',
                'topic': 'set_state',
                'msg': Int32,
                'qsize': 1,
            },
            'set_mode': {
                'namespace': 'unknown',
                'topic': 'set_mode',
                'msg': Int32,
                'qsize': 1,
            },
            'setup_action': {
                'namespace': 'unknown',
                'topic': 'setup_action',
                'msg': Int32,
                'qsize': 1,
            },
            'go_action': {
                'namespace': 'unknown',
                'topic': 'go_action',
                'msg': Int32,
                'qsize': 1,
            },
            'set_goto_error_bounds': {
                'namespace': 'unknown',
                'topic': 'set_goto_error_bounds',
                'msg': ErrorBounds,
                'qsize': 1,
            },
            'set_goto_timeout': {
                'namespace': 'unknown',
                'topic': 'set_goto_timeout',
                'msg': UInt32,
                'qsize': 1,
            },
            'go_home': {
                'namespace': 'unknown',
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
            },
            'set_home': {
                'namespace': 'unknown',
                'topic': 'set_home',
                'msg': GeoPoint,
                'qsize': 1,
            },
            'set_home_current': {
                'namespace': 'unknown',
                'topic': 'set_home_current',
                'msg': GotoLocation,
                'qsize': 1,
            },
            'goto_location': {
                'namespace': 'unknown',
                'topic': 'goto_location',
                'msg': GotoLocation,
                'qsize': 1,
            },
            'goto_position': {
                'namespace': 'unknown',
                'topic': 'goto_position',
                'msg': GotoPosition,
                'qsize': 1,
            },
            'goto_pose': {
                'namespace': 'unknown',
                'topic': 'goto_pose',
                'msg': GotoPose,
                'qsize': 1,
            },
            'goto_velocity': {
                'namespace': 'unknown',
                'topic': 'goto_velocity',
                'msg': GotoVelocity,
                'qsize': 1,
            },
            'set_teleop_velocity': {
                'namespace': 'unknown',
                'topic': 'set_teleop_velocity',
                'msg': Twist,
                'qsize': 1,
            },
            'go_stop': {
                'namespace': 'unknown',
                'topic': 'go_stop',
                'msg': Empty,
                'qsize': 1,
            },
            'enable_image_overlay': {
                'namespace': 'unknown',
                'topic': 'enable_image_overlay',
                'msg': Bool,
                'qsize': 1,
            },
            'set_process_name': {
                'namespace': 'unknown',
                'topic': 'set_process_name',
                'msg': String,
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
        if type(self) is ConnectRBXDeviceIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    def get_info_dict(self):
        """Return the latest device info as a dictionary.

        Returns:
            dict: A dictionary representation of the most recent DeviceRBXStatus message,
                or None if no info has been received yet.
        """
        info_dict = None
        if self.info_msg is not None:
            info_dict = nepi_sdk.convert_msg2dict(self.info_msg)
        return info_dict

    def get_info_msg(self):
        """Return the latest device info as a msg.

        Returns:
            DeviceRBXStatus: The most recent DeviceRBXStatus message,
                or None if no info has been received yet.
        """
        return self.info_msg

    def get_capabilities(self):
        """Query and return the RBX device capabilities report.

        Returns:
            DeviceRBXStatusQueryResponse: The capabilities response, carrying the
                device's DeviceRBXStatus in device_status, or None if no device
                is selected or the call failed.
        """
        resp = None
        if self.con_services_if is not None:
            resp = self.con_services_if.call_service(self.node_if_prefix + 'status_query', DeviceRBXStatusQueryRequest())
        return resp

    def get_goto_capabilities(self):
        """Return the goto capability flags the connected RBX device reports.

        Each flag says whether the device advertises that goto command at all.
        A device that reports False for a flag does not subscribe to the
        matching command topic, so publishing to it does nothing.

        Returns:
            dict: Keys has_goto_pose, has_goto_position, has_goto_location and
                has_goto_velocity, or None if the capabilities query failed.
        """
        resp = self.get_capabilities()
        if resp is None:
            return None
        caps = resp.device_status
        return {
            'has_goto_pose': caps.has_goto_pose,
            'has_goto_position': caps.has_goto_position,
            'has_goto_location': caps.has_goto_location,
            'has_goto_velocity': caps.has_goto_velocity
        }

    # 3.4.0 PORT: motors are per-motor SVX, see report
    #def get_motors_status_dict(self):
        #"""Return the latest multi-motor status as a dictionary.

        #Returns:
            #dict: A dictionary representation of the most recent MotorsStatus message,
                #or None if no motor status has been received yet.
        #"""
        #motors_dict = None
        #if self.motors_status_msg is not None:
            #motors_dict = nepi_sdk.convert_msg2dict(self.motors_status_msg)
        #return motors_dict

    #def get_motors_status_msg(self):
        #"""Return the latest multi-motor status as a msg.

        #Returns:
            #MotorsStatus: The most recent MotorsStatus message,
                #or None if no motor status has been received yet.
        #"""
        #return self.motors_status_msg

    def get_state(self):
        """Return the index of the device's current state option.

        Returns:
            int: The enumerated state index reported in DeviceRBXStatus, or None if no
                info has been received.
        """
        if self.info_msg is not None:
            return self.info_msg.state

    def get_mode(self):
        """Return the index of the device's current mode option.

        Returns:
            int: The enumerated mode index reported in DeviceRBXStatus, or None if no
                info has been received.
        """
        if self.info_msg is not None:
            return self.info_msg.mode

    def get_home_location(self):
        """Return the device's configured home location.

        Returns:
            list: A three-element list [latitude, longitude, altitude_m] reported in
                DeviceRBXStatus, or None if no info has been received.
        """
        if self.info_msg is not None:
            return [self.info_msg.home_lat, self.info_msg.home_long, self.info_msg.home_alt]

    def get_cmd_timeout(self):
        """Return the goto command timeout in seconds.

        Returns:
            float: The command timeout reported in DeviceRBXStatus, or None if no info
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

    # 3.4.0 PORT: motors are per-motor SVX, see report
    #def get_motor_control_settings(self):
        #"""Return the current manual motor control settings.

        #Returns:
            #list: A list of MotorControl entries reporting the current per-motor speed
                #ratios, or None if no status has been received.
        #"""
        #if self.status_msg is not None:
            #return self.status_msg.current_motor_control_settings

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
        pub_name = self.node_if_prefix + 'set_state'
        msg = state_ind
        self.node_if.publish_pub(pub_name, msg)

    def set_mode(self, mode_ind):
        """Command the RBX device to change to a mode option by index.

        Args:
            mode_ind (int): Index into the device's enumerated mode options.
        """
        pub_name = self.node_if_prefix + 'set_mode'
        msg = mode_ind
        self.node_if.publish_pub(pub_name, msg)

    def setup_action(self, action_ind):
        """Command the RBX device to run a setup action by index.

        Args:
            action_ind (int): Index into the device's enumerated setup action options.
        """
        pub_name = self.node_if_prefix + 'setup_action'
        msg = action_ind
        self.node_if.publish_pub(pub_name, msg)

    def go_action(self, action_ind):
        """Command the RBX device to run a go action by index.

        Args:
            action_ind (int): Index into the device's enumerated go action options.
        """
        pub_name = self.node_if_prefix + 'go_action'
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
        pub_name = self.node_if_prefix + 'set_goto_error_bounds'
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
        pub_name = self.node_if_prefix + 'set_goto_timeout'
        msg = timeout_s
        self.node_if.publish_pub(pub_name, msg)

    # 3.4.0 PORT: motors are per-motor SVX, see report
    #def set_motor_control(self, motor_ind, speed_ratio):
        #"""Command a single motor's speed ratio in manual control mode.

        #Args:
            #motor_ind (int): Index of the motor to command.
            #speed_ratio (float): Desired speed as a ratio from 0.0 (off) to 1.0 (max).
        #"""
        #pub_name = 'set_motor_control'
        #msg = MotorControl()
        #msg.motor_ind = motor_ind
        #msg.speed_ratio = speed_ratio
        #self.node_if.publish_pub(pub_name, msg)

    def go_home(self):
        """Command the RBX device to move to its configured home location.
        """
        pub_name = self.node_if_prefix + 'go_home'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def set_home(self, latitude, longitude, altitude):
        """Set the home location for the RBX device.

        Args:
            latitude (float): Home latitude in degrees. Pass -999 to keep the current value.
            longitude (float): Home longitude in degrees. Pass -999 to keep the current value.
            altitude (float): Home altitude in meters. Pass -999 to keep the current value.
        """
        pub_name = self.node_if_prefix + 'set_home'
        msg = GeoPoint()
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        self.node_if.publish_pub(pub_name, msg)

    def set_home_current(self):
        """Set the home location to the RBX device's current location.
        """
        pub_name = self.node_if_prefix + 'set_home_current'
        msg = GotoLocation()
        self.node_if.publish_pub(pub_name, msg)

    def goto_location(self, lat, long, altitude_meters, yaw_deg):
        """Command the RBX device to move to an absolute geographic location.

        Args:
            lat (float): Target latitude in degrees. Pass -999 to keep the current value.
            long (float): Target longitude in degrees. Pass -999 to keep the current value.
            altitude_meters (float): Target altitude in meters. Pass -999 to keep the current value.
            yaw_deg (float): Target yaw, ENU yaw in degrees, 0 = east, counter-clockwise positive (-180 to 180). Pass -999 to keep the current value.
        """
        pub_name = self.node_if_prefix + 'goto_location'
        msg = GotoLocation()
        msg.lat = lat
        msg.long = long
        msg.altitude_meters = altitude_meters
        msg.yaw_deg = yaw_deg
        self.node_if.publish_pub(pub_name, msg)

    def goto_position(self, x_meters, y_meters, z_meters, yaw_deg):
        """Command the RBX device to move to a relative body-frame position.

        Body FLU frame: x forward, y left, z up, yaw counter-clockwise positive.

        Args:
            x_meters (float): Target forward offset in meters. Zero keeps the current position on this axis.
            y_meters (float): Target left offset in meters. Zero keeps the current position on this axis.
            z_meters (float): Target up offset in meters. Zero keeps the current position on this axis.
            yaw_deg (float): Target body yaw change in degrees (-180 to 180), counter-clockwise positive (turn left). Zero keeps the current yaw.
        """
        pub_name = self.node_if_prefix + 'goto_position'
        msg = GotoPosition()
        msg.x_meters = x_meters
        msg.y_meters = y_meters
        msg.z_meters = z_meters
        msg.yaw_deg = yaw_deg
        self.node_if.publish_pub(pub_name, msg)

    def goto_velocity(self, x_mps, y_mps, z_mps, yaw_degps, duration_s):
        """Command the RBX device to hold a body-frame velocity for a fixed time.

        Open loop and timed, unlike goto_position: there is no target to
        converge on, the device applies the commanded velocity and is expected
        to stop itself when duration_s elapses.

        Args:
            x_mps (float): Forward velocity in meters per second, robot body frame.
            y_mps (float): Left velocity in meters per second, robot body frame.
            z_mps (float): Up velocity in meters per second, robot body frame.
            yaw_degps (float): Yaw rate in degrees per second, positive to port.
            duration_s (float): How long the device holds the command, in seconds.
        """
        pub_name = self.node_if_prefix + 'goto_velocity'
        msg = GotoVelocity()
        msg.x_mps = x_mps
        msg.y_mps = y_mps
        msg.z_mps = z_mps
        msg.yaw_degps = yaw_degps
        msg.duration_s = duration_s
        self.node_if.publish_pub(pub_name, msg)

    def set_teleop_velocity(self, linear_x, linear_y, linear_z, angular_z):
        """Send one teleop velocity command to the selected RBX device.

        Ratios from -1 to 1 in the body FLU frame, passed through unchanged to
        the device's set_teleop_velocity topic: x forward, y left, z up, and
        angular z counter-clockwise. The device acts on it only while its
        teleop controls are ready, and a driver may time a command out, so a
        caller holding a velocity sends it every cycle.

        Args:
            linear_x (float): Forward ratio, -1 to 1.
            linear_y (float): Left ratio, -1 to 1.
            linear_z (float): Up ratio, -1 to 1.
            angular_z (float): Counter-clockwise turn ratio, -1 to 1.
        """
        pub_name = self.node_if_prefix + 'set_teleop_velocity'
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.z = angular_z
        self.node_if.publish_pub(pub_name, msg)

    def goto_pose(self, roll_deg, pitch_deg, yaw_deg):
        """Command the RBX device to move to an absolute orientation pose.

        Args:
            roll_deg (float): Target roll in degrees (-180 to 180). Pass -999 to keep the current value.
            pitch_deg (float): Target pitch in degrees (-180 to 180). Pass -999 to keep the current value.
            yaw_deg (float): Target yaw, ENU yaw in degrees, 0 = east, counter-clockwise positive (-180 to 180). Pass -999 to keep the current value.
        """
        pub_name = self.node_if_prefix + 'goto_pose'
        msg = GotoPose()
        msg.roll_deg = roll_deg
        msg.pitch_deg = pitch_deg
        msg.yaw_deg = yaw_deg
        self.node_if.publish_pub(pub_name, msg)

    def go_stop(self):
        """Command the RBX device to stop all motion.
        """
        pub_name = self.node_if_prefix + 'go_stop'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def set_image_topic(self, topic):
        """Set the image topic the RBX device associates with its status overlay.

        The 3.4 RBX device subscribes no matching topic, so nothing is sent and
        a warning is logged.

        Args:
            topic (str): Partial or full ROS namespace string, or '' for a black image background.
        """
        self.msg_if.pub_warn("Image topic selection is not supported by the connected RBX device: " + str(topic), throttle_s = 5.0)

    def enable_image_overlay(self, enable):
        """Enable or disable the status message overlay on the RBX device image.

        Args:
            enable (bool): True to overlay status on the image, False to disable it.
        """
        pub_name = self.node_if_prefix + 'enable_image_overlay'
        msg = enable
        self.node_if.publish_pub(pub_name, msg)

    def set_process_name(self, name):
        """Set the human-readable name of the current control process on the RBX device.

        Args:
            name (str): The process name to report in device status.
        """
        pub_name = self.node_if_prefix + 'set_process_name'
        msg = name
        self.node_if.publish_pub(pub_name, msg)

    def set_navpose_frame(self, navpose_frame):
        """Set the navpose frame for the RBX device.

        The 3.4 RBX device subscribes no matching topic, so nothing is sent and
        a warning is logged.

        Args:
            navpose_frame (str): Desired navpose frame identifier string.
        """
        self.msg_if.pub_warn("Navpose frame changes are not supported by the connected RBX device: " + str(navpose_frame), throttle_s = 5.0)

    def request_status_update(self):
        """Request that the RBX device immediately republish its status message.

        The 3.4 RBX device subscribes no matching topic, so nothing is sent and
        a warning is logged.
        """
        self.msg_if.pub_warn("Status update requests are not supported by the connected RBX device; it publishes status on its own timer", throttle_s = 5.0)

    def request_info_update(self):
        """Request that the RBX device immediately republish its info message.

        The 3.4 RBX device subscribes no matching topic, so nothing is sent and
        a warning is logged.
        """
        self.msg_if.pub_warn("Info update requests are not supported by the connected RBX device; its info rides in its status", throttle_s = 5.0)

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

    def updaterCb(self,timer):
        if self.connected == False:
            self.info_msg = None
            self.motors_status_msg = None
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

        # Services Config Dict ####################
        # Client proxies, held in this connect's own ConnectNodeServicesIF: the node_if
        # is a NodeClassIF, whose services registry builds servers only (it reads each
        # entry's 'callback') and has no call_service. Unregistered on teardown.
        self.connect_topic_srvs_dict = {
            self.node_if_prefix + 'status_query': {
                'namespace': self.selected_topic,
                'topic': 'status_query',
                'srv': DeviceRBXStatusQuery,
                'req': DeviceRBXStatusQueryRequest(),
                'resp': DeviceRBXStatusQueryResponse()
            }
        }
        if self.con_services_if is None:
            self.con_services_if = ConnectNodeServicesIF(msg_if = self.msg_if)
        self.con_services_if.add_services(self.connect_topic_srvs_dict)
        return success


    def unsubscribe_topic(self):
        success = super().unsubscribe_topic()
        if self.con_services_if is not None:
            self.con_services_if.unregister_services()
        self.connect_topic_srvs_dict = None
        self.save_data_status_msg = None
        self.info_msg = None
        self.motors_status_msg = None
        return success


    def _saveDataStatusCb(self,status_msg):
        self.save_data_status_msg = status_msg


    def _statusCb(self,status_msg):
        super()._statusCb(status_msg)
        if self.dataCB is not None:
            status_dict = self.get_status_dict()
            self.dataCB(status_dict)
        # 3.4.0 PORT: the DeviceRBXInfo fields ride in DeviceRBXStatus, see report
        self._infoCb(status_msg)


    def _infoCb(self,info_msg):
        self.info_msg = info_msg
        if self.infoCb is not None:
            info_dict = self.get_info_dict()
            self.infoCb(info_dict)
