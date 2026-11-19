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

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_interfaces.msg import DeviceSVXStatus
from nepi_interfaces.msg import SingleAxisTimedSpeedMove
from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryRequest, DeviceInfoQueryResponse
from nepi_interfaces.srv import DeviceSVXStatusQuery, DeviceSVXStatusQueryRequest, DeviceSVXStatusQueryResponse

from nepi_api.messages_if import MsgIF

from nepi_api.connect_device_if import ConnectDeviceIF
from nepi_api.connect_node_if import ConnectNodeClassIF
from nepi_api.connect_node_if import ConnectNodeServicesIF


#########################################
# Connect IF Class
#########################################

# Auto-discovering connect interface for the SVX (servo) device interface,
# modeled on ConnectPTXDeviceIF. One SVX device = one servo, so this is PTX
# collapsed to a single axis. It subclasses ConnectDeviceIF, whose ConnectNodeIF
# parent owns the <node>/svx_connect connect namespace (ConnectIFStatus selector
# state plus the select_topic subscriber), auto-discovers the devices publishing
# nepi_interfaces/DeviceSVXStatus, and connects to the selected device.
#
# The reported position is the device's pos_now, which an open-loop servo driver
# reports as the last commanded value. The API speaks degrees, matching
# device_if_svx.py.
#
# Registry keys carry self.node_if_prefix (this connect's own namespace below the
# node, e.g. 'pan_servo/'), the convention every NodeIF-based connect uses. A
# per-class 'svx_' prefix is not enough: a pan/tilt pair holds two SVX connects,
# and publish_pub is keyed, so on a shared node_if each would rebind the other's
# publishers. ROS names derive from namespace+topic, not the key, so the prefix is
# wire-invisible.

CONNECT_ID='SVX'
CONNECT_STATUS_MSG='DeviceSVXStatus'
CONNECT_STATUS_MSG_CLASS=DeviceSVXStatus
CONNECT_NAME='svx_connect'


CONNECTED_TIMEOUT = 2


class ConnectSVXDeviceIF(ConnectDeviceIF):

    # ADD Additional Connect Callback Functions

    position_deg = 0
    last_position_deg = 0
    moving = False

    speed_max_dps = 10
    # Last speed ratio reported by the device, or the last one commanded through this
    # interface before any status arrived. Cached the same way speed_max_dps is, so
    # set_max_speed_ratio() can answer without a status msg.
    speed_ratio = 0.5

    connect_topic_srvs_dict = None
    con_services_if = None
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
            'goto_position': {
                'namespace': 'unknown',
                'topic': 'goto_position',
                'msg': Float32,
                'qsize': 1,
            },
            'set_speed_ratio': {
                'namespace': 'unknown',
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'set_speed_max_dps': {
                'namespace': 'unknown',
                'topic': 'set_speed_max',
                'msg': Float32,
                'qsize': 1,
            },
            'set_reverse_enable': {
                'namespace': 'unknown',
                'topic': 'set_reverse_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_spin_direction': {
                'namespace': 'unknown',
                'topic': 'set_direction',
                'msg': Float32,
                'qsize': 1,
            },
            'move_direction': {
                'namespace': 'unknown',
                'topic': 'jog_timed_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
            },
            'stop_moving': {
                'namespace': 'unknown',
                'topic': 'stop_moving',
                'msg': Empty,
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
                'msg': Float32,
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
        if type(self) is ConnectSVXDeviceIF:
            self.start()


    #######################
    # Class Public Methods
    #######################


    def set_selected_topic(self, selected_topic):
        """Select which discovered SVX device this interface connects to, and report the result.

        Thin wrapper over the base-class selector that returns the resulting selection.
        The base method persists the choice and silently ignores an undiscovered topic,
        but returns nothing, so a caller that tracks the selection has no way to learn
        whether its request was taken.

        Args:
            selected_topic (str): Fully-qualified device namespace, which must be one of
                the entries from get_available_topics(), or the string 'None' to deselect.
                An undiscovered topic is ignored rather than rejected with an error.

        Returns:
            str: The topic selected after this call -- the requested topic if it was
                accepted, otherwise the selection already in effect. Never None.
        """
        super().set_selected_topic(selected_topic)
        return self.selected_topic

    def get_capabilities(self):
        """Query and return the SVX device capabilities report.

        Returns:
            DeviceSVXStatusQueryResponse: The status response, carrying the device's
                DeviceSVXStatus in device_status, or None if the call failed.
        """
        resp = None
        if self.con_services_if is not None:
            resp = self.con_services_if.call_service(self.node_if_prefix + 'status_query', DeviceSVXStatusQueryRequest())
        return resp

    def get_device_info(self):
        """Query and return the SVX device info report.

        Returns:
            DeviceInfoQueryResponse: The device info response carrying device name, path,
                node name and namespace, serial number, and hardware and software versions,
                or None if the call failed.
        """
        resp = None
        if self.con_services_if is not None:
            resp = self.con_services_if.call_service(self.node_if_prefix + 'device_info_query', DeviceInfoQueryRequest())
        return resp

    def get_servo_capabilities(self):
        """Return the servo capability flags reported by the connected SVX device.

        Returns:
            dict: A dictionary with keys has_absolute_positioning, has_adjustable_speed,
                has_limit_controls, has_homing, has_set_home, and has_spin, or None if no
                status has been received. The keys differ from DeviceSVXStatus 3.4 and map
                onto its has_position_control, has_speed_control, has_limit_controls,
                has_go_home_control, has_set_home_control and is_spin fields.
        """
        if self.status_msg is not None:
            return {
                'has_absolute_positioning': self.status_msg.has_position_control,
                'has_adjustable_speed': self.status_msg.has_speed_control,
                'has_limit_controls': self.status_msg.has_limit_controls,
                'has_homing': self.status_msg.has_go_home_control,
                'has_set_home': self.status_msg.has_set_home_control,
                'has_spin': self.status_msg.is_spin
            }

    def get_servo_soft_limits(self):
        """Return the software softstop limits for the servo axis.

        Returns:
            list: A two-element list [min_deg, max_deg] representing the software softstop
                limits in degrees, or None if no status has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.min_soft_limit, self.status_msg.max_soft_limit]

    def get_servo_max_speed_dps(self):
        """Return the servo maximum speed in degrees per second.

        Returns:
            float: The maximum speed in degrees per second, which is what a speed ratio
                of 1.0 means.
        """
        return self.speed_max_dps

    def get_max_speed_dps(self):
        """Return the servo maximum speed in degrees per second.

        Driver-facing name for the same cached value get_servo_max_speed_dps() reports.
        Safe to call before a topic is selected: it answers with the class default until
        a device status arrives, so callers can scale a ratio by it without a None check.

        Returns:
            float: The maximum speed in degrees per second, which is what a speed ratio
                of 1.0 means. Never None.
        """
        return self.speed_max_dps

    def get_servo_position(self):
        """Return the most recently reported servo position.

        Returns:
            float: The current position in degrees (last commanded value, open loop).
        """
        return self.position_deg

    def get_servo_home_position(self):
        """Return the servo home position reported by the connected SVX device.

        Returns:
            float: The home position in degrees, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.pos_home

    def get_speed_ratio(self):
        """Return the servo speed ratio reported by the connected SVX device.

        Returns:
            float: The speed as a ratio from 0.0 to 1.0, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.speed_ratio

    def get_speed_dps(self):
        """Return the servo speed in degrees per second reported by the connected SVX device.

        This is the device's measured speed (speed_now in DeviceSVXStatus), the
        magnitude of its recent position change per second.

        Returns:
            float: The current speed in degrees per second, or None if no status has
                been received.
        """
        if self.status_msg is not None:
            return self.status_msg.speed_now

    def get_spin_direction(self):
        """Return the spin direction reported by the connected SVX device.

        Returns:
            int: 1 for clockwise, -1 for counter-clockwise (0 if the device cannot report
                it), or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.dir_now

    def check_moving(self):
        """Check whether the servo is currently in motion.

        Returns:
            bool: True if the servo has moved more than 0.1 degrees since the last update
                cycle, False otherwise.
        """
        return self.moving

    def check_spinning(self):
        """Check whether the servo is in continuous rotation.

        Reports the continuous-mode state rather than the instantaneous speed, so it
        stays True at a speed ratio of 0.0.

        Returns:
            bool: True if the device reports continuous mode (is_continous in
                DeviceSVXStatus), False otherwise, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.is_continous



    def goto_position(self, position_deg):
        """Command the servo to move to an absolute position.

        Args:
            position_deg (float): Target position angle in degrees.
        """
        pub_name = self.node_if_prefix + 'goto_position'
        msg = float(position_deg)
        self.node_if.publish_pub(pub_name, msg)

    def move_direction(self, direction):
        """Jog the servo in a direction at the speed the device is already set to.

        The move runs until it is stopped -- with stop_moving(), go_home(), or a position
        command -- or until the servo reaches its limit. A caller that wants a timed jog
        owns the stop: check the return, wait, then call stop_moving().

        Args:
            direction (int): Sign of the requested motion, 1 for positive and -1 for
                negative. Only the sign is used; 0 counts as positive.

        Returns:
            bool: True if the command was published, False if no device is connected, in
                which case no motion was started and no stop is owed.
        """
        return self.moveDirection(direction, None)

    def move_direction_speed(self, direction, speed_dps):
        """Jog the servo in a direction at a speed given in degrees per second.

        Same as move_direction() except the speed is set as part of the command. The
        speed is converted to a ratio against the device's reported maximum, matching
        what ConnectPTXDeviceIF.jog_timed_speed_dps_pan does, since the SVX speed
        control speaks ratios.

        Args:
            direction (int): Sign of the requested motion, 1 for positive and -1 for
                negative. Only the sign is used; 0 counts as positive.
            speed_dps (float): Requested speed in degrees per second, clamped to the
                range 0 to get_max_speed_dps(). A value of 0 leaves the device's current
                speed unchanged.

        Returns:
            bool: True if the command was published, False if no device is connected, in
                which case no motion was started and no stop is owed.
        """
        # A zero speed, or no known max, keeps the documented "leave the device's
        # current speed unchanged" meaning: moveDirection reads None that way.
        speed_max_dps = self.speed_max_dps
        if speed_max_dps is None or speed_max_dps <= 0 or float(speed_dps) <= 0:
            speed_ratio = None
        else:
            speed_ratio = float(speed_dps) / float(speed_max_dps)
        return self.moveDirection(direction, speed_ratio)

    def set_speed_ratio(self, speed_ratio):
        """Publish a speed ratio command to the servo.

        In continuous mode this is the rotation speed rather than the move speed.

        See also set_speed_dps() for the same command in degrees per second,
        set_speed_max_dps() and set_max_speed_dps() to change what a ratio of 1.0 means,
        and set_max_speed_ratio(), which despite its name is another route to this same
        command.

        Args:
            speed_ratio (float): Desired motion speed as a ratio from 0.0 (slowest) to 1.0 (fastest).
        """
        pub_name = self.node_if_prefix + 'set_speed_ratio'
        msg = float(speed_ratio)
        self.node_if.publish_pub(pub_name, msg)

    def set_speed_dps(self, speed_dps):
        """Publish a speed command to the servo in degrees per second.

        The 3.4 SVX device takes speed as a ratio only, so this converts the value
        against the device's last reported max speed and clamps the result into the
        0.0 to 1.0 range: a value above the max saturates at full speed rather than
        raising. In continuous mode this is the rotation speed rather than the move
        speed.

        See also set_speed_ratio() for the same command as a 0.0 to 1.0 dial,
        set_speed_max_dps() and set_max_speed_dps() to change what full speed means, and
        set_max_speed_ratio(), which despite its name is another speed ratio setter.

        Args:
            speed_dps (float): Desired motion speed in degrees per second.
        """
        speed_max_dps = self.speed_max_dps
        if speed_max_dps is None or speed_max_dps <= 0:
            speed_ratio = 0.0
        else:
            speed_ratio = float(speed_dps) / float(speed_max_dps)
        if speed_ratio < 0.0:
            speed_ratio = 0.0
        if speed_ratio > 1.0:
            speed_ratio = 1.0
        pub_name = self.node_if_prefix + 'set_speed_ratio'
        msg = speed_ratio
        self.node_if.publish_pub(pub_name, msg)

    def set_speed_max_dps(self, speed_max_dps):
        """Set the maximum servo speed in degrees per second, the value a ratio of 1.0 means.

        This changes the scale, not the current speed. To command a speed, use
        set_speed_dps() or set_speed_ratio().

        See also set_max_speed_dps(), the driver-facing name for this same command, and
        set_max_speed_ratio(), which despite its name sets a speed ratio and no maximum.

        Args:
            speed_max_dps (float): Maximum speed in degrees per second, which is what a
                speed ratio of 1.0 means.
        """
        pub_name = self.node_if_prefix + 'set_speed_max_dps'
        msg = float(speed_max_dps)
        self.node_if.publish_pub(pub_name, msg)

    def set_max_speed_dps(self, speed_max_dps):
        """Set the maximum servo speed in degrees per second, the value a ratio of 1.0 means.

        Driver-facing name for set_speed_max_dps(). Publishes nothing and reports False
        if no device is connected yet, rather than raising. This changes the scale, not
        the current speed.

        See also set_speed_dps() and set_speed_ratio() to command a speed, and
        set_max_speed_ratio(), which despite its similar name sets a speed ratio and no
        maximum.

        Args:
            speed_max_dps (float): Maximum speed in degrees per second, greater than 0,
                which is what a speed ratio of 1.0 means. The device ignores values of
                0 or less.

        Returns:
            bool: True if the command was published, False if there is no connected device.
        """
        if self.node_if is None:
            return False
        self.set_speed_max_dps(speed_max_dps)
        return True

    def set_max_speed_ratio(self, speed_ratio = None):
        """Set the servo speed ratio, or read back the current one.

        WARNING: despite its name this sets the speed ratio and not any maximum. It
        publishes on the same set_speed_ratio topic that set_speed_ratio() does and
        changes nothing about speed_max_dps. Confusing it with set_max_speed_dps() causes
        defects. The name is kept only because live external call sites
        outside this workspace depend on it.

        See also set_speed_ratio() for the same command without the misleading name,
        set_speed_dps() to command a speed in real units, and set_speed_max_dps() /
        set_max_speed_dps(), which are the two methods that really do set the maximum.

        Called with a ratio this publishes a speed command; called with no argument it
        publishes nothing and only reports. Either way it returns a usable number, so a
        caller can read the speed before a topic is selected.

        Args:
            speed_ratio (float, optional): Desired motion speed as a ratio from 0.0
                (slowest) to 1.0 (fastest); values outside that range are clamped into
                it. Defaults to None, which reads the current ratio without commanding
                a change. In continuous mode this is the rotation speed rather than the
                move speed.

        Returns:
            float: The speed ratio from 0.0 to 1.0 last reported by the device, or the
                last value commanded through this interface if no status has arrived yet.
                Never None.
        """
        if speed_ratio is not None:
            ratio = float(speed_ratio)
            if ratio < 0.0:
                ratio = 0.0
            if ratio > 1.0:
                ratio = 1.0
            self.speed_ratio = ratio
            if self.node_if is not None:
                self.node_if.publish_pub(self.node_if_prefix + 'set_speed_ratio', ratio)
        return self.speed_ratio

    def set_reverse_enable(self, reverse_enable):
        """Enable or disable direction reversal on the servo.

        Args:
            reverse_enable (bool): True to reverse the axis direction, False for normal direction.
        """
        pub_name = self.node_if_prefix + 'set_reverse_enable'
        msg = bool(reverse_enable)
        self.node_if.publish_pub(pub_name, msg)

    def set_continuous_mode(self, continuous_enable):
        """Declare the attached servo continuous-rotation or positional.

        The 3.4 SVX device sets continuous mode from its driver and subscribes no
        topic to change it, so nothing is sent and a warning is logged.

        Args:
            continuous_enable (bool): True for a continuous-rotation servo, False for a
                positional servo.
        """
        self.msg_if.pub_warn("Continuous mode changes are not supported by the connected SVX device: " + str(continuous_enable), throttle_s = 5.0)

    def set_spin_direction(self, spin_direction):
        """Set the spin direction for the servo.

        Args:
            spin_direction (int): Intended direction, 1 = clockwise, -1 = counter-clockwise.
        """
        # The device's set_direction topic is Float32 and uses only the sign
        pub_name = self.node_if_prefix + 'set_spin_direction'
        msg = float(spin_direction)
        self.node_if.publish_pub(pub_name, msg)

    def stop_moving(self):
        """Publish a stop command to halt motion on the servo.
        """
        pub_name = self.node_if_prefix + 'stop_moving'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def go_home(self):
        """Command the servo to move to its configured home position.

        For a continuous servo the home degree is the neutral pulse, so this is also
        the stop command.
        """
        pub_name = self.node_if_prefix + 'go_home'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def set_home_position(self, position_deg):
        """Set the home position for the servo to a specified angle.

        Args:
            position_deg (float): Desired home position angle in degrees.
        """
        pub_name = self.node_if_prefix + 'set_home_position'
        msg = float(position_deg)
        self.node_if.publish_pub(pub_name, msg)

    def set_home_position_here(self):
        """Set the home position to the servo's current position.
        """
        pub_name = self.node_if_prefix + 'set_home_position_here'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def reset_device(self):
        """Command the servo device to reset to its default state.

        The 3.4 SVX device subscribes no reset topic, so nothing is sent and a
        warning is logged.
        """
        self.msg_if.pub_warn("Device reset is not supported by the connected SVX device", throttle_s = 5.0)

    ###############################
    # Class Private Methods
    ###############################

    # Shared body of move_direction/move_direction_speed. speed_ratio None means "leave
    # the device's speed alone". The 3.4 device hands the message's ratio straight to
    # its driver, so that case sends the device's own last reported ratio, not 0.0.
    #
    # duration_s is always -1.0 (indefinite): the caller owns the timed stop, and the
    # device has no move timer. Success is reported as "published", not as the return of
    # node_if.publish_pub -- ConnectNodeClassIF.publish_pub always returns False (it
    # assigns to a misspelled local), and a False here would cost the caller its stop.
    def moveDirection(self, direction, speed_ratio):
        if self.node_if is None or self.connected == False:
            return False
        msg = SingleAxisTimedSpeedMove()
        msg.direction = 1 if float(direction) >= 0 else -1
        if speed_ratio is None:
            speed_ratio = self.speed_ratio
        ratio = float(speed_ratio)
        if ratio < 0.0:
            ratio = 0.0
        if ratio > 1.0:
            ratio = 1.0
        msg.speed_ratio = ratio
        msg.duration_s = -1.0
        self.node_if.publish_pub(self.node_if_prefix + 'move_direction', msg)
        return True


    def updaterCb(self,timer):
        if self.connected == True:
            self.moving = abs(self.position_deg - self.last_position_deg) > 0.1
            self.last_position_deg = self.position_deg
        else:
            self.moving = False
        super().updaterCb(timer)


    def subscribe_topic(self, topic):
        success = super().subscribe_topic(topic)

        # Services Config Dict ####################
        # Client proxies, held in this connect's own ConnectNodeServicesIF: the node_if
        # is a NodeClassIF, whose services registry builds servers only (it reads each
        # entry's 'callback') and has no call_service. Unregistered on teardown.
        self.connect_topic_srvs_dict = {
            self.node_if_prefix + 'status_query': {
                'namespace': self.selected_topic,
                'topic': 'status_query',
                'srv': DeviceSVXStatusQuery,
                'req': DeviceSVXStatusQueryRequest(),
                'resp': DeviceSVXStatusQueryResponse()
            },
            self.node_if_prefix + 'device_info_query': {
                'namespace': self.selected_topic,
                'topic': 'device_info_query',
                'srv': DeviceInfoQuery,
                'req': DeviceInfoQueryRequest(),
                'resp': DeviceInfoQueryResponse()
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
        self.moving = False
        return success


    def _statusCb(self,status_msg):
        # Cache only a usable max: move_direction_speed and set_speed_dps divide by it.
        if status_msg.speed_max > 0:
            self.speed_max_dps = status_msg.speed_max
        self.speed_ratio = status_msg.speed_ratio
        self.position_deg = status_msg.pos_now
        super()._statusCb(status_msg)

