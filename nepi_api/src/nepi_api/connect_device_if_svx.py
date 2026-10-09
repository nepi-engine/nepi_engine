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

import time
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nepi_interfaces.msg import DeviceSVXStatus
from nepi_interfaces.msg import SingleAxisTimedSpeedMove
from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryRequest, DeviceInfoQueryResponse
from nepi_interfaces.srv import SVXCapabilitiesQuery, SVXCapabilitiesQueryRequest, SVXCapabilitiesQueryResponse

from nepi_api.messages_if import MsgIF

from nepi_api.connect_node_if import ConnectNodeIF
from nepi_api.connect_node_if import ConnectNodeClassIF


#########################################
# Connect IF Class
#########################################

# Auto-discovering connect interface for the SVX (servo) device interface,
# modeled on ConnectPTXDeviceIF. One SVX device = one servo, so this is PTX
# collapsed to a single axis. It subclasses ConnectNodeIF, which owns the
# <node>/svx_connect connect namespace (ConnectIFStatus selector state plus the
# select_topic subscriber), auto-discovers the devices publishing
# nepi_interfaces/DeviceSVXStatus, and connects to the selected device.
#
# Open loop: reported position is the last commanded value; nothing is measured.
# The API speaks degrees, matching device_if_svx.py.
#
# Registry keys are prefixed 'svx_' throughout. Four of the SVX control topics
# (stop_moving, go_home, set_home_position, set_home_position_here) are named
# identically in ConnectPTXDeviceIF, and publish_pub is keyed -- so on a shared
# node_if an unprefixed key would silently rebind the pan-tilt's publisher, or
# have its own rebound. ROS names derive from namespace+topic, not the key, so
# the prefix is wire-invisible.

CONNECT_ID='SVX'
CONNECT_STATUS_MSG='DeviceSVXStatus'
CONNECT_NAME='svx_connect'


CONNECTED_TIMEOUT = 2


class ConnectSVXDeviceIF(ConnectNodeIF):

    # ADD Additional Connect Callback Functions


    msg_if = None
    ready = False
    namespace = '~'

    node_if = None

    status_msg = None
    connected = False
    last_status_time = 0
    position_deg = 0
    last_position_deg = 0
    moving = False

    statusCb = None # Backwards Compatibility

    speed_max_dps = 10
    # Last speed ratio reported by the device, or the last one commanded through this
    # interface before any status arrived. Cached the same way speed_max_dps is, so
    # set_max_speed_ratio() can answer without a status msg.
    speed_ratio = 0.5

    connect_topic_subs_dict = None
    connect_topic_pubs_dict = None
    connect_topic_srvs_dict = None
    #######################
    ### IF Initialization
    def __init__(self,
                connect_name = CONNECT_NAME,
                namespace = None,
                statusCb = None,
                auto_select_enabled = True,
                show_selector = True,
                show_controls = True,
                show_data = True,
                msg_if = None,
                node_if = None
                ):
        self.msg_if = msg_if
        self.node_if = node_if
        # auto_select_enabled is exposed because a consumer that owns MORE THAN ONE
        # servo interface must be able to turn it off. With it on, every interface
        # independently grabs the first discovered SVX device, so a pan/tilt pair
        # both land on the same servo channel.
        super().__init__(
                connect_id = CONNECT_ID,
                connect_status_msg = CONNECT_STATUS_MSG,
                connect_name = connect_name,
                selected_topic = namespace,
                auto_select_enabled = auto_select_enabled,
                show_selector = show_selector,
                show_controls = show_controls,
                show_data = show_data,
                msg_if = self.msg_if,
                node_if = self.node_if
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
        """Return the fully-resolved ROS namespace for the connected SVX device.

        Returns:
            str: The fully-qualified namespace string used for topic and service resolution.
        """
        return self.selected_topic

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
            dict: A dictionary representation of the most recent DeviceSVXStatus message,
                or None if no status has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    def get_status_msg(self):
        """Return the latest device status as a msg.

        Returns:
            DeviceSVXStatus: The most recent DeviceSVXStatus message, or None if no status
                has been received yet.
        """
        return self.status_msg

    def get_capabilities(self):
        """Query and return the SVX device capabilities report.

        Returns:
            SVXCapabilitiesQueryResponse: The capabilities response, or None if the call failed.
        """
        resp = None
        if self.node_if is not None:
            resp = self.node_if.call_service('svx_capabilities_query', SVXCapabilitiesQueryRequest())
        return resp

    def get_device_info(self):
        """Query and return the SVX device info report.

        Returns:
            DeviceInfoQueryResponse: The device info response carrying device name, path,
                node name and namespace, serial number, and hardware and software versions,
                or None if the call failed.
        """
        resp = None
        if self.node_if is not None:
            resp = self.node_if.call_service('svx_device_info_query', DeviceInfoQueryRequest())
        return resp

    def get_servo_capabilities(self):
        """Return the servo capability flags reported by the connected SVX device.

        Returns:
            dict: A dictionary with keys has_absolute_positioning, has_adjustable_speed,
                has_limit_controls, has_homing, has_set_home, and has_spin, or None if no
                status has been received.
        """
        if self.status_msg is not None:
            return {
                'has_absolute_positioning': self.status_msg.has_absolute_positioning,
                'has_adjustable_speed': self.status_msg.has_adjustable_speed,
                'has_limit_controls': self.status_msg.has_limit_controls,
                'has_homing': self.status_msg.has_homing,
                'has_set_home': self.status_msg.has_set_home,
                'has_spin': self.status_msg.has_spin
            }

    def get_servo_soft_limits(self):
        """Return the software softstop limits for the servo axis.

        Returns:
            list: A two-element list [min_deg, max_deg] representing the software softstop
                limits in degrees, or None if no status has been received.
        """
        if self.status_msg is not None:
            return [self.status_msg.min_softstop_deg, self.status_msg.max_softstop_deg]

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
            return self.status_msg.home_pos_deg

    def get_speed_ratio(self):
        """Return the servo speed ratio reported by the connected SVX device.

        Returns:
            float: The speed as a ratio from 0.0 to 1.0, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.speed_ratio

    def get_speed_dps(self):
        """Return the servo speed in degrees per second reported by the connected SVX device.

        This is the device's own speed_ratio multiplied by its current speed_max_dps,
        computed on the device rather than at the call site.

        Returns:
            float: The commanded speed in degrees per second, or None if no status has
                been received.
        """
        if self.status_msg is not None:
            return self.status_msg.speed_now_dps

    def get_spin_direction(self):
        """Return the spin direction reported by the connected SVX device.

        Returns:
            int: 1 for clockwise, -1 for counter-clockwise, or None if no status has been
                received.
        """
        if self.status_msg is not None:
            return self.status_msg.spin_direction

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
            bool: True if the device is in continuous mode and has not been stopped,
                False otherwise, or None if no status has been received.
        """
        if self.status_msg is not None:
            return self.status_msg.is_spinning



    def goto_position(self, position_deg):
        """Command the servo to move to an absolute position.

        Args:
            position_deg (float): Target position angle in degrees.
        """
        pub_name = 'svx_goto_position'
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
        speed_max_dps = self.speed_max_dps
        if speed_max_dps is None or speed_max_dps <= 0:
            speed_ratio = 0.0
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
        pub_name = 'svx_set_speed_ratio'
        msg = float(speed_ratio)
        self.node_if.publish_pub(pub_name, msg)

    def set_speed_dps(self, speed_dps):
        """Publish a speed command to the servo in degrees per second.

        The device converts the value against its current speed_max_dps and clamps the
        result into the 0.0 to 1.0 ratio range, so a value above speed_max_dps saturates
        at full speed rather than raising. In continuous mode this is the rotation speed
        rather than the move speed.

        See also set_speed_ratio() for the same command as a 0.0 to 1.0 dial,
        set_speed_max_dps() and set_max_speed_dps() to change what full speed means, and
        set_max_speed_ratio(), which despite its name is another speed ratio setter.

        Args:
            speed_dps (float): Desired motion speed in degrees per second.
        """
        pub_name = 'svx_set_speed_dps'
        msg = float(speed_dps)
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
        pub_name = 'svx_set_speed_max_dps'
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
        changes nothing about speed_max_dps. Confusing it with set_max_speed_dps() has
        already caused a defect. The name is kept only because live external call sites
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
                self.node_if.publish_pub('svx_set_speed_ratio', ratio)
        return self.speed_ratio

    def set_reverse_enable(self, reverse_enable):
        """Enable or disable direction reversal on the servo.

        Args:
            reverse_enable (bool): True to reverse the axis direction, False for normal direction.
        """
        pub_name = 'svx_set_reverse_enable'
        msg = bool(reverse_enable)
        self.node_if.publish_pub(pub_name, msg)

    def set_continuous_mode(self, continuous_enable):
        """Declare the attached servo continuous-rotation or positional.

        Continuous mode is device configuration, not a driver capability: enabling it
        switches the reported has_spin flag on and makes set_speed_ratio and
        set_spin_direction drive rotation instead of a position move.

        Args:
            continuous_enable (bool): True for a continuous-rotation servo, False for a
                positional servo.
        """
        pub_name = 'svx_set_continuous_mode'
        msg = bool(continuous_enable)
        self.node_if.publish_pub(pub_name, msg)

    def set_spin_direction(self, spin_direction):
        """Set the spin direction for the servo.

        Args:
            spin_direction (int): Intended direction, 1 = clockwise, -1 = counter-clockwise.
        """
        pub_name = 'svx_set_spin_direction'
        msg = int(spin_direction)
        self.node_if.publish_pub(pub_name, msg)

    def stop_moving(self):
        """Publish a stop command to halt motion on the servo.
        """
        pub_name = 'svx_stop_moving'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def go_home(self):
        """Command the servo to move to its configured home position.

        For a continuous servo the home degree is the neutral pulse, so this is also
        the stop command.
        """
        pub_name = 'svx_go_home'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def set_home_position(self, position_deg):
        """Set the home position for the servo to a specified angle.

        Args:
            position_deg (float): Desired home position angle in degrees.
        """
        pub_name = 'svx_set_home_position'
        msg = float(position_deg)
        self.node_if.publish_pub(pub_name, msg)

    def set_home_position_here(self):
        """Set the home position to the servo's current position.
        """
        pub_name = 'svx_set_home_position_here'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def reset_device(self):
        """Command the servo device to reset to its default state.
        """
        pub_name = 'svx_reset_device'
        msg = Empty()
        self.node_if.publish_pub(pub_name, msg)

    def save_config(self):
        """Publish a save configuration command to persist current settings on the device.
        """
        self.node_if.publish_pub('svx_save_config',Empty())

    def reset_config(self):
        """Publish a reset configuration command to restore the last saved settings on the device.
        """
        self.node_if.publish_pub('svx_reset_config',Empty())

    def factory_reset_config(self):
        """Publish a factory reset command to restore factory default settings on the device.
        """
        self.node_if.publish_pub('svx_factory_reset_config',Empty())

    ###############################
    # Class Private Methods
    ###############################

    # Shared body of move_direction/move_direction_speed. speed_ratio None means "leave
    # the device's speed alone", which the device reads as a ratio of 0.0.
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
            msg.speed_ratio = 0.0
        else:
            ratio = float(speed_ratio)
            if ratio < 0.0:
                ratio = 0.0
            if ratio > 1.0:
                ratio = 1.0
            msg.speed_ratio = ratio
        msg.duration_s = -1.0
        self.node_if.publish_pub('svx_move_direction', msg)
        return True


    def updaterCb(self,timer):
        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time )
        if self.connected == True:
            if (cur_time - last_time) > CONNECTED_TIMEOUT:
                self.connected = False
                self.status_msg = None
                self.moving = False

        if self.connected == True:
            self.moving = abs(self.position_deg - self.last_position_deg) > 0.1
            self.last_position_deg = self.position_deg

        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)




    def subscribe_topic(self, topic):
        self.msg_if.pub_debug("subscribe_svx_topic Called")

        success = False
        success = self.unsubscribe_topic()

        # Subscribers Config Dict ####################
        self.connect_topic_subs_dict = {
            'svx_status_sub': {
                'namespace': self.selected_topic,
                'topic': 'status',
                'msg': DeviceSVXStatus,
                'qsize': 10,
                'callback': self._statusCb
            }
        }



        # Publishers Config Dict ####################
        # One publisher per SVX control topic, plus the three config topics
        # NodeConfigsIF subscribes on the same device namespace.
        self.connect_topic_pubs_dict = {
            'svx_goto_position': {
                'namespace': self.selected_topic,
                'topic': 'goto_position',
                'msg': Float32,
                'qsize': 1,
            },
            'svx_set_speed_ratio': {
                'namespace': self.selected_topic,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'svx_set_speed_dps': {
                'namespace': self.selected_topic,
                'topic': 'set_speed_dps',
                'msg': Float32,
                'qsize': 1,
            },
            'svx_set_speed_max_dps': {
                'namespace': self.selected_topic,
                'topic': 'set_speed_max_dps',
                'msg': Float32,
                'qsize': 1,
            },
            'svx_set_reverse_enable': {
                'namespace': self.selected_topic,
                'topic': 'set_reverse_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'svx_set_continuous_mode': {
                'namespace': self.selected_topic,
                'topic': 'set_continuous_mode',
                'msg': Bool,
                'qsize': 1,
            },
            'svx_set_spin_direction': {
                'namespace': self.selected_topic,
                'topic': 'set_spin_direction',
                'msg': Int32,
                'qsize': 1,
            },
            'svx_move_direction': {
                'namespace': self.selected_topic,
                'topic': 'move_direction',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
            },
            'svx_stop_moving': {
                'namespace': self.selected_topic,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
            },
            'svx_go_home': {
                'namespace': self.selected_topic,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
            },
            'svx_set_home_position': {
                'namespace': self.selected_topic,
                'topic': 'set_home_position',
                'msg': Float32,
                'qsize': 1,
            },
            'svx_set_home_position_here': {
                'namespace': self.selected_topic,
                'topic': 'set_home_position_here',
                'msg': Empty,
                'qsize': 1,
            },
            'svx_reset_device': {
                'namespace': self.selected_topic,
                'topic': 'reset_device',
                'msg': Empty,
                'qsize': 1,
            },
            'svx_save_config': {
                'namespace': self.selected_topic,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': 1,
            },
            'svx_reset_config': {
                'namespace': self.selected_topic,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': 1,
            },
            'svx_factory_reset_config': {
                'namespace': self.selected_topic,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': 1,
            }


        }


        # Services Config Dict ####################
        # Re-registered on every selection rather than unregistered on teardown:
        # register_service replaces the keyed entry, which drops the old proxy and
        # builds one on the newly selected device.
        self.connect_topic_srvs_dict = {
            'svx_capabilities_query': {
                'namespace': self.selected_topic,
                'topic': 'capabilities_query',
                'srv': SVXCapabilitiesQuery,
                'req': SVXCapabilitiesQueryRequest(),
                'resp': SVXCapabilitiesQueryResponse()
            },
            'svx_device_info_query': {
                'namespace': self.selected_topic,
                'topic': 'device_info_query',
                'srv': DeviceInfoQuery,
                'req': DeviceInfoQueryRequest(),
                'resp': DeviceInfoQueryResponse()
            }
        }

        if self.node_if is not None:
            self.node_if.register_pubs(self.connect_topic_pubs_dict)
            self.node_if.register_subs(self.connect_topic_subs_dict)
            for srv_name in self.connect_topic_srvs_dict.keys():
                self.node_if.register_service(srv_name, self.connect_topic_srvs_dict[srv_name])
            self.connecting = True
            self.connected = False
            self.connected_topic = 'None'
            self.status_msg = None

        return success





    def unsubscribe_topic(self):
        success = False
        if self.connecting == True or self.connected == True:
            self.msg_if.pub_debug("unsubscribe_topic Called")

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

            # Service proxies are left registered here and replaced by the next
            # subscribe_topic; see the services dict above.
            self.connect_topic_srvs_dict = None

            nepi_sdk.sleep(1)
            self.connecting = False
            self.connected = False
            self.connected_topic = 'None'
            self.status_msg = None
            self.moving = False
            success = True
        return success


    def _statusCb(self,status_msg):
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self._announceConnected('SVX')
            self.connecting = False
            self.connected_topic = self.selected_topic
        self.connected = True
        self.status_msg = status_msg

        self.speed_max_dps = status_msg.speed_max_dps
        self.speed_ratio = status_msg.speed_ratio
        self.position_deg = status_msg.position_now_deg
        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)
