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
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_interfaces.msg import RangeWindow, NavPose, NavPosePanTilt, SaveDataRate
from nepi_interfaces.msg import DevicePTXStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove
from nepi_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryRequest, PTXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.device_if_npx import NPXDeviceIF


from nepi_api.connect_node_if import ConnectNodeClassIF

#########################################
# Node Class
#########################################

class ConnectPTXDeviceIF:
    """Client-side interface for connecting to a remote PTX (pan/tilt) device node.

    Subscribes to status, pan/tilt position, and stop-callback topics published
    by a ``PTXActuatorIF`` node, and exposes publisher methods that send commands
    to that node.  Intended to be instantiated by any node that needs to drive or
    monitor a PTX device over ROS without owning the hardware driver directly.

    Attributes:
        CONNECTED_TIMEOUT (int): Seconds of silence before the device is
            considered disconnected.
        namespace (str): Full ROS namespace of the remote PTX device node.
        ready (bool): True once initialization completes successfully.
        connected (bool): True while status messages are arriving within
            CONNECTED_TIMEOUT.
        status_msg (DevicePTXStatus): Most recent status message received, or
            None if not yet connected.
        pan_tilt_position (list[float]): Most recent [pan_deg, tilt_deg].
        pan_moving (bool): True when pan position changed since last update.
        tilt_moving (bool): True when tilt position changed since last update.
    """

    CONNECTED_TIMEOUT = 2
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    status_msg = None
    connected = False
    last_status_time = 0
    navpose_msg = None
    pan_tilt_position = [0,0]
    last_pan_tilt_position = [0,0]
    pan_moving = False
    tilt_moving = False

    statusCb = None
    navposeCb = None
    panTiltCb = None
    
    
    stopPanCb = None
    stopTiltCb = None
    #######################
    ### IF Initialization
    def __init__(self,
                namespace = None,
                statusCb = None,
                panTiltCb = None,
                navposeCb = None,
                stopPanCb = None,
                stopTiltCb = None,
                log_name = None,
                log_name_list = [],
                msg_if = None
                ):
        """Initialize the ConnectPTXDeviceIF and subscribe to the target PTX node.

        Resolves the given namespace to a full ROS namespace, builds publisher
        and subscriber dictionaries, creates a ConnectNodeClassIF, and starts
        a periodic updater timer.  Returns immediately (before the remote node
        is necessarily reachable); callers should use ``wait_for_ready()`` or
        ``wait_for_connection()`` before issuing commands.

        Args:
            namespace (str, optional): ROS namespace of the remote PTX device
                node (relative or absolute).  If None, initialization is
                skipped and the object is left in a not-ready state.
            statusCb (callable, optional): Called with a status dict each time
                a new ``DevicePTXStatus`` message arrives.
            panTiltCb (callable, optional): Called with ``(pan_deg, tilt_deg)``
                each time a new pan/tilt position message arrives.
            navposeCb (callable, optional): Reserved for future navpose
                updates; not currently used by this class.
            stopPanCb (callable, optional): Called with no arguments when a
                stop-pan callback topic message arrives.
            stopTiltCb (callable, optional): Called with no arguments when a
                stop-tilt callback topic message arrives.
            log_name (str, optional): Additional log name appended to log
                output.
            log_name_list (list, optional): Initial list of log name prefixes.
            msg_if (MsgIF, optional): Shared MsgIF instance.  A new one is
                created if None.

        Note:
            If namespace is None the method returns after creating only the
            MsgIF — no subscriptions or publishers are created.
        """
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ##############################    
        # Initialize Class Variables

        if namespace is None:
            return

        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.msg_if.pub_info("Using PT Namespace: " + self.namespace)

        self.statusCb = statusCb
        self.navposeCb = navposeCb
        self.panTiltCb = panTiltCb


        self.stopPanCb = stopPanCb
        self.stopTiltCb = stopTiltCb


        ##############################   
        ## Node Setup

        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
                'namespace': self.namespace
        }


        # Services Config Dict ####################
        self.SRVS_DICT = None


        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'stop_moving': {
                'namespace': self.namespace,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
            },
            'goto_to_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': PanTiltPosition,
                'qsize': 1,
            },
            'goto_to_pan_position': {
                'namespace': self.namespace,
                'topic': 'goto_pan_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_to_tilt_position': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_position',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_pan_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_pan_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'goto_tilt_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
            },
            'jog_timed_pan': {
                'namespace': self.namespace,
                'topic': 'jog_timed_pan',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
            },
            'jog_timed_tilt': {
                'namespace': self.namespace,
                'topic': 'jog_timed_tilt',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
            },
            'reverse_pan_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_pan_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'reverse_tilt_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_tilt_enable',
                'msg': Bool,
                'qsize': 1,
            },
            'set_soft_limits': {
                'namespace': self.namespace,
                'topic': 'set_soft_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
            },
            'go_home': {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
            },
            'set_home_position': {
                'namespace': self.namespace,
                'topic': 'set_home_position',
                'msg': PanTiltPosition,
                'qsize': 1,
            },
            'set_home_position_here': {
                'namespace': self.namespace,
                'topic': 'set_home_position_here',
                'msg': Empty,
                'qsize': 1,
            }


        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DevicePTXStatus,
                'qsize': 10,
                'callback': self._statusCb
            },
            'pan_tilt_position': {
                'namespace': self.namespace,
                'topic': 'pan_tilt',
                'msg': NavPosePanTilt,
                'qsize': 1,
                'callback': self._panTiltCb, 
                'callback_args': ()
            },
            'stop_pan_callback': {
                'topic': 'stop_pan_callback',
                'msg': Empty,
                'namespace': self.namespace,
                'qsize': 5,
                'callback': self._stopPanCb, 
                'callback_args': ()
            },
            'stop_tilt_callback': {
                'msg': Empty,
                'namespace': self.namespace,
                'topic': 'stop_tilt_callback',
                'qsize': 5,
                'callback': self._stopTiltCb, 
                'callback_args': ()
            }
        }


        # Create Node Class ####################
        
        self.con_node_if = ConnectNodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = [],
                        msg_if = None
        )

        

        self.con_node_if.wait_for_ready()


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
        """Return whether initialization has completed.

        Returns:
            bool: True if ``__init__`` completed successfully, False otherwise.
        """
        return self.ready

    def wait_for_ready(self, timout = float('inf') ):
        """Block until initialization is complete or the timeout expires.

        Args:
            timout (float): Maximum seconds to wait.  Defaults to infinity.

        Returns:
            bool: True if ready, False if timed out.

        Note:
            Parameter name is intentionally misspelled ``timout`` (upstream
            convention); do not rename without auditing all callers.
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
        """Return the fully-resolved ROS namespace of the remote PTX device node.

        Returns:
            str: Full ROS namespace string.
        """
        return self.namespace

    def check_connection(self):
        """Return whether the remote PTX device node is currently connected.

        Returns:
            bool: True if a status message was received within
            ``CONNECTED_TIMEOUT`` seconds.
        """
        return self.connected

    def wait_for_connection(self, timout = float('inf') ):
        """Block until a status message is received or the timeout expires.

        Args:
            timout (float): Maximum seconds to wait.  Defaults to infinity.

        Returns:
            bool: True if connected, False if timed out.
        """
        if self.con_node_if is not None:
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
        """Return whether the status topic is currently receiving messages.

        Returns:
            bool: True if connected (same as ``check_connection``).
        """
        return self.connected

    def wait_for_status_connection(self, timout = float('inf') ):
        """Block until a status message arrives or the timeout expires.

        Args:
            timout (float): Maximum seconds to wait.  Defaults to infinity.

        Returns:
            bool: True if connected, False if timed out.
        """
        if self.con_node_if is not None:
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
        """Return the most recent device status as a dictionary.

        Returns:
            dict: Status fields from the last ``DevicePTXStatus`` message,
            converted via ``nepi_sdk.convert_msg2dict``.  None if no status
            message has been received yet.
        """
        status_dict = None
        if self.status_msg is not None:
            status_dict = nepi_sdk.convert_msg2dict(self.status_msg)
        return status_dict

    # def get_navpose_dict(self):
    #     navpose_dict = None
    #     if self.navpose_msg is not None:
    #         navpose_dict = nepi_nav.convert_navpose_msg2dict(self.navpose_msg)
    #     return navpose_dict
    
    def get_pan_tilt_hard_limits(self):
        """Return the hardware hard-stop limits for pan and tilt axes.

        Returns:
            list[float]: ``[pan_min_deg, pan_max_deg, tilt_min_deg,
            tilt_max_deg]`` from the last status message.  None if no status
            message has been received.

        Note:
            The fourth element in the returned list reads
            ``pan_max_hardstop_deg`` (not ``tilt_max_hardstop_deg``) due to a
            known bug in the upstream status message field assignment.
        """
        if self.status_msg is not None:
            return [self.status_msg.pan_min_hardstop_deg, self.status_msg.pan_max_hardstop_deg, self.status_msg.tilt_min_hardstop_deg, self.status_msg.pan_max_hardstop_deg]
    
    def get_pan_tilt_soft_limits(self):
        """Return the configurable soft-stop limits for pan and tilt axes.

        Returns:
            list[float]: ``[pan_min_deg, pan_max_deg, tilt_min_deg,
            tilt_max_deg]`` from the last status message.  None if no status
            message has been received.

        Note:
            The fourth element reads ``pan_max_softstop_deg`` (not
            ``tilt_max_softstop_deg``) due to a known bug in the upstream
            field assignment — mirrors the hard-limits bug.
        """
        if self.status_msg is not None:
            return [self.status_msg.pan_min_softstop_deg, self.status_msg.pan_max_softstop_deg, self.status_msg.tilt_min_softstop_deg, self.status_msg.pan_max_softstop_deg]


    def get_pan_tilt_position(self):
        """Return the most recently reported pan and tilt position.

        Returns:
            list[float]: ``[pan_deg, tilt_deg]`` snapshot from the last
            pan/tilt topic message.  Returns ``[0, 0]`` before the first
            message arrives.
        """
        return [self.pan_tilt_position[0], self.pan_tilt_position[1]]

    def check_pan_moving(self):
        """Return whether the pan axis is currently in motion.

        Returns:
            bool: True if the pan position changed by more than 0.1 degrees
            since the last updater cycle.
        """
        return self.pan_moving

    def check_tilt_moving(self):
        """Return whether the tilt axis is currently in motion.

        Returns:
            bool: True if the tilt position changed by more than 0.1 degrees
            since the last updater cycle.
        """
        return self.tilt_moving

    def unregister(self):
        """Unsubscribe from all PTX topics and release ROS resources."""
        self._unsubscribeTopic()

    def set_speed_ratio(self, speed_ratio):
        """Publish a speed ratio command to the remote PTX device node.

        Args:
            speed_ratio (float): Desired speed as a fraction of maximum,
                in the range [0.0, 1.0].

        Note:
            ROS topic: ``<namespace>/set_speed_ratio`` (Float32).
        """
        pub_name = 'speed_ratio'
        msg = speed_ratio
        self.con_node_if.publish_pub(pub_name,msg)

    def stop_moving(self):
        """Publish a stop-moving command to halt all axes on the remote PTX node.

        Note:
            ROS topic: ``<namespace>/stop_moving`` (Empty).
        """
        pub_name = 'stop_moving'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)


    def goto_to_position(self, pan_deg, tilt_deg):
        """Command the remote PTX device to move to an absolute pan/tilt position.

        Args:
            pan_deg (float): Target pan angle in degrees.
            tilt_deg (float): Target tilt angle in degrees.

        Note:
            ROS topic: ``<namespace>/goto_position`` (PanTiltPosition).
        """
        pub_name = 'goto_to_position'
        msg = PanTiltPosition()
        msg.pan_deg = pan_deg
        msg.tilt_deg = tilt_deg
        self.con_node_if.publish_pub(pub_name,msg)

    def goto_to_pan_position(self, pan_position):
        """Command the remote PTX device to move to an absolute pan position.

        Args:
            pan_position (float): Target pan angle in degrees.

        Note:
            ROS topic: ``<namespace>/goto_pan_position`` (Float32).
        """
        #self.msg_if.pub_warn("connect goto pan pos: " + str(pan_position))

        pub_name = 'goto_to_pan_position'
        msg = pan_position
        self.con_node_if.publish_pub(pub_name,msg)
        
    def goto_to_tilt_position(self, tilt_position):
        """Command the remote PTX device to move to an absolute tilt position.

        Args:
            tilt_position (float): Target tilt angle in degrees.

        Note:
            ROS topic: ``<namespace>/goto_tilt_position`` (Float32).
        """
        pub_name = 'goto_to_tilt_position'
        msg = tilt_position
        self.con_node_if.publish_pub(pub_name,msg)

    def goto_pan_ratio(self, pan_ratio):
        """Command the remote PTX device to move pan to a normalized ratio position.

        Args:
            pan_ratio (float): Target pan position as a ratio in [0.0, 1.0],
                where 0.0 and 1.0 correspond to the soft-stop limits.

        Note:
            ROS topic: ``<namespace>/goto_pan_ratio`` (Float32).
        """
        pub_name = 'goto_pan_ratio'
        msg = pan_ratio
        self.con_node_if.publish_pub(pub_name,msg)        

    def goto_tilt_ratio(self, tilt_ratio):
        """Command the remote PTX device to move tilt to a normalized ratio position.

        Args:
            tilt_ratio (float): Target tilt position as a ratio in [0.0, 1.0],
                where 0.0 and 1.0 correspond to the soft-stop limits.

        Note:
            ROS topic: ``<namespace>/goto_tilt_ratio`` (Float32).
        """
        pub_name = 'goto_tilt_ratio'
        msg = tilt_ratio
        self.con_node_if.publish_pub(pub_name,msg)

    def jog_timed_pan(self, direction, duration_s):
        """Command the remote PTX device to jog the pan axis for a fixed duration.

        Args:
            direction (int): Jog direction indicator (e.g., +1 or -1).
            duration_s (float): Duration in seconds.  Pass -1.0 for infinite
                duration (jog until stopped).

        Note:
            ROS topic: ``<namespace>/jog_timed_pan`` (SingleAxisTimedMove).
        """
        pub_name = 'jog_timed_pan'
        msg = SingleAxisTimedMove()
        # Direction indicator
        msg.direction = direction
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.con_node_if.publish_pub(pub_name,msg)
        
    def jog_timed_tilt(self, direction, duration_s):
        """Command the remote PTX device to jog the tilt axis for a fixed duration.

        Args:
            direction (int): Jog direction indicator (e.g., +1 or -1).
            duration_s (float): Duration in seconds.  Pass -1.0 for infinite
                duration (jog until stopped).

        Note:
            ROS topic: ``<namespace>/jog_timed_tilt`` (SingleAxisTimedMove).
        """
        pub_name = 'jog_timed_tilt'
        msg = SingleAxisTimedMove()
        # Direction indicator
        msg.direction = direction
        # Duration, -1.0 for infinite duration
        msg.duration_s = duration_s
        self.con_node_if.publish_pub(pub_name,msg) 

    def reverse_pan_enabled(self, reverse_pan):
        """Enable or disable pan-axis direction reversal on the remote PTX node.

        Args:
            reverse_pan (bool): True to invert pan direction, False for normal.

        Note:
            ROS topic: ``<namespace>/set_reverse_pan_enable`` (Bool).
        """
        pub_name = 'reverse_pan_enabled'
        msg = reverse_pan
        self.con_node_if.publish_pub(pub_name,msg)

    def reverse_tilt_enabled(self, reverse_tilt):
        """Enable or disable tilt-axis direction reversal on the remote PTX node.

        Args:
            reverse_tilt (bool): True to invert tilt direction, False for normal.

        Note:
            ROS topic: ``<namespace>/set_reverse_tilt_enable`` (Bool).
        """
        pub_name = 'reverse_tilt_enabled'
        msg = reverse_tilt
        self.con_node_if.publish_pub(pub_name,msg)

    def go_home(self):
        """Command the remote PTX device to return to its home position.

        Note:
            ROS topic: ``<namespace>/go_home`` (Empty).
        """
        pub_name = 'go_home'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)

    def set_home_position(self, pan_deg, tilt_deg):
        """Set the home position of the remote PTX device to specific angles.

        Args:
            pan_deg (float): Home pan angle in degrees.
            tilt_deg (float): Home tilt angle in degrees.

        Note:
            ROS topic: ``<namespace>/set_home_position`` (PanTiltPosition).
        """
        pub_name = 'set_home_position'
        msg = PanTiltPosition()
        msg.pan_deg = pan_deg
        msg.tilt_deg = tilt_deg
        self.con_node_if.publish_pub(pub_name,msg) 

    def set_home_position_here(self):
        """Capture the current position as the new home position on the remote PTX node.

        Note:
            ROS topic: ``<namespace>/set_home_position_here`` (Empty).
        """
        pub_name = 'set_home_position_here'
        msg = Empty()
        self.con_node_if.publish_pub(pub_name,msg)  

    def save_config(self):
        """Publish a save-config command to the remote PTX node."""
        self.con_node_if.publish_pub('save_config',Empty())

    def reset_config(self):
        """Publish a reset-config command to restore saved settings on the remote PTX node."""
        self.con_node_if.publish_pub('reset_config',Empty())

    def factory_reset_config(self):
        """Publish a factory-reset command to restore default settings on the remote PTX node."""
        self.con_node_if.publish_pub('factory_reset_config',Empty())

    #################
    ## Save Data Functions

    def get_save_data_products(self):
        """Return the list of available save-data products from the remote node.

        Returns:
            list: Data product identifiers reported by the save-data interface.
        """
        data_products = self.con_save_data_if.get_data_products()
        return data_products

    def get_save_data_status_dict(self):
        """Return the current save-data status from the remote node as a dict.

        Returns:
            dict: Save-data status fields from the remote save-data interface.
        """
        status_dict = self.con_save_data_if.get_status_dict()
        return status_dict

    def save_data_enable_pub(self, enable):
        """Publish a save-data enable/disable command to the remote node.

        Args:
            enable (bool): True to enable data saving, False to disable.
        """
        self.con_save_data_if.save_data_pub(enable)

    def save_data_prefix_pub(self, prefix):
        """Publish a filename prefix to use for saved data on the remote node.

        Args:
            prefix (str): Filename prefix string.
        """
        self.con_save_data_if.save_data_prefix_pub(prefix)

    def save_data_rate_pub(self, rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS):
        """Publish a save-data rate command to the remote node.

        Args:
            rate_hz (float): Desired save rate in Hz.
            data_product (int): Data product identifier.  Defaults to
                ``SaveDataRate.ALL_DATA_PRODUCTS``.
        """
        self.con_save_data_if.publish_pub(rate_hz, data_product = SaveDataRate.ALL_DATA_PRODUCTS)

    def save_data_snapshot_pub(self):
        """Trigger a one-shot data snapshot on the remote node."""
        self.con_save_data_if.publish_pub()

    def save_data_reset_pub(self):
        """Publish a save-data reset command to the remote node."""
        self.con_save_data_if.publish_pub(pub_name,msg)

    def save_data_factory_reset_pub(self):
        """Publish a factory-reset command to the save-data interface on the remote node."""
        pub_name = 'factory_reset'
        msg = Empty()
        self.con_save_data_if.publish_pub(pub_name,msg)

    ###############################
    # Class Private Methods
    ###############################
   
    def updaterCb(self, timer):
        cur_time = nepi_utils.get_time()
        last_time = copy.deepcopy(self.last_status_time )
        if self.connected == True:
            if (cur_time - last_time) > self.CONNECTED_TIMEOUT:
                self.connected = False 
                self.status_msg = None
                self.navpose_msg = None
                self.data_dict = None
                self.pan_moving = False
                self.tilt_moving = False

        if self.connected == True:
            self.pan_moving = abs(self.pan_tilt_position[0] - self.last_pan_tilt_position[0]) > 0.1
            self.tilt_moving = abs(self.pan_tilt_position[1] - self.last_pan_tilt_position[1]) > 0.1
  
        nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot = True)

    def _unsubscribeTopic(self):
        success = False
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering topic: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.connected = False 
                self.status_msg = None
                self.navpose_msg = None
                self.data_dict = None
                success = True
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister image:  " + str(e))
        return success


    def _statusCb(self,status_msg):  
        self.last_status_time = nepi_utils.get_time()
        if self.connected == False:
            self.msg_if.pub_warn("Connected to PT Status:  " + str(self.namespace))
        self.connected = True
        self.status_msg = status_msg
        if self.statusCb is not None:
            status_dict = self.get_status_dict()
            self.statusCb(status_dict)

    def _panTiltCb(self,pan_tilt_msg):    
        #self.msg_if.pub_warn("Connected to PT Position")  
        self.pan_tilt_position = [pan_tilt_msg.pan_deg, pan_tilt_msg.tilt_deg]
        if self.panTiltCb is not None:
            #self.msg_if.pub_warn("panTiltCb: " + str(self.panTiltCb))
            self.panTiltCb(self.pan_tilt_position[0], self.pan_tilt_position[1])
    

    def _stopPanCb(self,msg):    
        self.msg_if.pub_warn("Got Stop Pan msg: " + str(msg))
        if self.stopPanCb is not None:
            self.stopPanCb()

    def _stopTiltCb(self,msg):    
        self.msg_if.pub_warn("Got Stop Tilt msg: " + str(msg))
        if self.stopTiltCb is not None:
            self.stopTiltCb()