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

import numpy as np

import cv2
import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system
# nepi_pc (imports open3d) MUST come before nepi_img (imports scipy.sparse.linalg,
# matplotlib, cv_bridge) -- confirmed live on aarch64/glibc 2.31: those heavier
# libraries' own native TLS usage exhausts the process's static TLS block, so
# open3d's OpenMP extension (libgomp.so.1) then fails importing with
# "cannot allocate memory in static TLS block" -- 100% reproducible in this
# order, 100% avoided by importing open3d (via nepi_pc) first. This broke
# EVERY RBXRobotIF-based driver (rbx_sim, rbx_ardupilot, ...) on this
# platform. Order matters here; do not "clean up" back to alphabetical.
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_img
from nepi_sdk import nepi_nav

from nepi_sdk import nepi_process_rbx

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from pygeodesy.ellipsoidalKarney import LatLon


from nepi_interfaces.msg import ErrorBounds

from nepi_interfaces.msg import GotoPose, GotoPosition, GotoLocation, GotoVelocity, GotoErrors, GotoGoals

from nepi_interfaces.msg import DeviceRBXStatus
from nepi_interfaces.srv import DeviceRBXStatusQuery, DeviceRBXStatusQueryResponse, DeviceRBXStatusQueryRequest


from nepi_api.data_product_if_image import ColorImageIF

from nepi_api.device_if_npx import NPXDeviceIF


from nepi_api.process_if import ProcessIF

from nepi_api.device_if import DeviceIF

from nepi_api.connect_data_product_if_image import ConnectColorImageIF



EXAMPLE_HEADING_DATA_DICT = {
    'time_heading': nepi_utils.get_time(),
    # Heading should be provided in Degrees True North
    'heading_deg': 120.50,
}

EXAMPLE_POSITION_DATA_DICT = {
    'time_position': nepi_utils.get_time(),
    # Position should be provided in Meters, local ENU world frame (x,y,z) with
    # x east, y north, and z up. Body-relative commands (goto_position, teleop,
    # goto_velocity) use the body FLU frame instead: x forward, y left, z up.
    'x_m': 1.234,
    'y_m': 1.234,
    'z_m': 1.234,
}

EXAMPLE_ORIENTATION_DATA_DICT = {
    'time_orientation': nepi_utils.get_time(),
    # Orientation should be provided in Degrees ENU, in the NavPose convention
    # (nepi_nav): yaw 0 faces east and is positive counter-clockwise seen from
    # above; roll and pitch are the NavPose roll/pitch values
    'roll_deg': 30.51,
    'pitch_deg': 30.51,
    'yaw_deg': 30.51,
}

EXAMPLE_LOCATION_DATA_DICT = {
    'time_location': nepi_utils.get_time(),
    # Location Lat,Long
    'lat': 47.080909,
    'long': -120.8787889,
}

EXAMPLE_ALTITUDE_DATA_DICT = {
    'time_altitude': nepi_utils.get_time(),
    # Altitude should be provided in postivie meters WGS84 (height above the ellipsoid)
    'altitude_m': 12.321,
}

EXAMPLE_ALTITUDE_DATA_DICT = {
    'time_depth': nepi_utils.get_time(),
    # Depth should be provided in positive distance from surface in meters
    'altitude_m': 0
}


class RBXRobotIF(DeviceIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = DeviceRBXStatus()
    DATA_MSG = None

    ID = 'rbx'
    DESCRIPTION = 'Robotic Device'

    DATA_SOURCE_DESCRIPTION = 'Robot'
    DATA_REF_DESCRIPTION = 'Robot Frame Center'

    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict( 
    )

    #########################
    ## Managed by NodeIF
    #########################

    # id = ''
    # name = ''
    # namespace = '~'

    # status_msg = None
    # data_msg = None

    # msg_if = None
    # node_if = None
    # settings_if = None
    # save_data_if = None
    # transform_if = None
    # navpose_if = None    

    # callback_dict = dict()

    # active_topics = []
    # active_topic_types = []
    # active_services = []

    # caps_dict = dict()

    #########################
    ## Managed by DataProductIF
    #########################
    # SYSTEM_IF_DICT = dict()

    # data_source_description = 'sensor'
    # data_ref_description = 'sensor'

    # device_name = ''
    # path = ''
    # serial_num = ''
    # hw_version = ''
    # sw_version = ''

    # factory_controls_dict = dict()


    # msg_if = None
    # node_if = None
    # settings_if = None
    # save_data_if = None
    # transform_if = None
    # navpose_if = None

    # color_image_if = None
    # depthmap_if = None
    # pointcloud_if = None

    #########################
    ## CUSTOM
    #########################
    ready = False

    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    STATUS_UPDATE_RATE_HZ = 2
    UPDATE_NAVPOSE_RATE_HZ = 10
    CHECK_SAVE_DATA_RATE_HZ = 40
    IMAGE_SUB_BUFF_SIZE = 2**24 # rospy's receive buffer; its 64 KB default holds less than one frame (nepi_sdk.create_subscriber)
    IMAGE_SOURCE_QUIET_SEC = 1.0 # publish_status republishes the image while the source has been silent this long

    # Factory Control Values 
    FACTORY_GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
    FACTORY_GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
    FACTORY_GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding
    FACTORY_CMD_TIMEOUT_SEC = 25 # Any action that changes 
    FACTORY_HOME_LOCATION = [47.6540828,-122.3187578,0.0]

    MIN_MAX_PROCESS_RATES = [0,10]
    DEFAULT_PROCESS_RATE = 10
    # Define class variables
    data_products_list = []


    factory_device_name = ''

    factory_controls = dict()

    caps_dict = dict()

    servo_if_dict = dict()

    BLANK_PROCESS_UPDATE_DICT = dict(
        set_home =   None,
        go_home = None,
        go_stop = None,
        goto_pose = None,
        goto_position = None,
        goto_location = None,
        goto_velocity = None,
        go_action = None,
    )

    process_update_dict = copy.deepcopy(BLANK_PROCESS_UPDATE_DICT)

    BLANK_PROCESS_DATA_DICT = dict(
        ###############
        # Provided By Caller
    ###############
        # Updated By Caller, Then Clears
        set_home =   None,
        go_home = None,
        go_stop = None,
        goto_pose = None,
        goto_position = None,
        goto_location = None,
        goto_velocity = None,
        go_action = None,

        ###############
        # Set by Process
        has_x_control = False,
        has_y_control = False,
        has_z_control = False,

        has_roll_control = False,
        has_pitch_control = False,
        has_yaw_control = False,

        has_set_home = False,
        has_go_home = False,
        has_go_stop = False,
        has_goto_pose = False,
        has_goto_position = False,
        has_goto_location = False,
        has_go_velocity = False,

        go_action_options = [],

        ###############
        # Updated by Process, Caller Waits/Blocks for True/False Response
        set_home_success = None,
        go_home_success = None,
        go_stop_success = None,
        goto_pose_success = None,
        goto_position_success = None,
        goto_location_success = None,
        goto_velocity_success = None,
        go_action_success = None    
    )

    connected = False
    rbx_process_module = nepi_process_rbx
    rbx_process_name = nepi_process_rbx.NAME
    process_if = None
    process_data_dict = copy.deepcopy(BLANK_PROCESS_DATA_DICT)
    process_ready = False

    states = []
    modes = []
    setup_actions = []
    go_actions = []

    msg_if = None
    node_if = None
    node_if_shared = True
    settings_if = None
    save_data_if = None
    transform_if = None
    navpose_if = None

    connect_navpose_if = None

    image_if = None

    connect_inmage_if = None


    status_msg_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)
    check_save_data_interval_sec = float(1)/CHECK_SAVE_DATA_RATE_HZ
    update_navpose_interval_sec = float(1)/UPDATE_NAVPOSE_RATE_HZ

    ## Initialize Class Variables
    current_heading_deg = 0
    current_orientation_enu_degs = [0,0,0]
    current_orientation_ned_degs = [0,0,0]
    current_position_enu_m = [0,0,0]
    current_position_ned_m = [0,0,0]
    current_location_amsl_geo = [0,0,0]
    current_location_wgs84_geo = [0,0,0]
    current_geoid_height_m = 0
    
    home_location = [0,0,0]
    last_cmd_string = ""

    rbx_state_last = None
    rbx_mode_last = None


    servo_if_dict = dict()



    image_overlay_enabled = True
    image_blank = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
    ### IF Initialization
    def __init__(self, device_info,
                 axisCapabilities,getBatteryPercentFunction,
                 states,getStateIndFunction,setStateIndFunction,
                 modes,getModeIndFunction,setModeIndFunction,
                 checkStopFunction,
                 setup_actions, setSetupActionIndFunction,
                 go_actions, setGoActionIndFunction,
                 getSettingsFunction=None, setSettingFunction=None, 
                 data_source_description = None,
                 data_ref_description = None,
                 getHomeFunction=None,setHomeFunction=None,

                 teleopControlsReadyFunction=None,
                 setTeleopVelocityFunction=None,
                 autonomousControlsReadyFunction=None,
                 goHomeFunction=None, goStopFunction=None, 
                 gotoPoseFunction=None, gotoPositionFunction=None, gotoLocationFunction=None,
                 gotoVelocityFunction=None,
                 manualControlsReadyFunction=None,
                 servo_if_dict = None,
                msg_if = None,
                node_if = None,
                settings_if = None,
                save_data_if = None,
                connect_navpose_if = None,
                ):
        """Creates the RBX robot interface and its command, status and image topics.

        Frame contract. World quantities are ENU (x east, y north, z up). A
        yaw_* value is ENU yaw: 0 faces east, positive counter-clockwise seen
        from above. Roll and pitch follow the NavPose convention of nepi_nav.
        Body-relative quantities are FLU (x forward, y left, z up, yaw positive
        counter-clockwise). heading_* values are compass headings (degrees
        clockwise from true north). Latitude and longitude are WGS84 degrees.
        The interface hands every driver function ENU or FLU values; a driver
        converts anything its hardware does not take as ENU/FLU itself.

        Args:
            device_info (dict): Device name, path, serial number and versions.
            axisCapabilities (dict): has_*_control flags for the x, y, z, roll,
                pitch and yaw axes.
            getBatteryPercentFunction (callable): Returns battery charge 0.0-1.0.
            states (list): State names; getStateIndFunction/setStateIndFunction
                get and set an index into it.
            modes (list): Mode names; getModeIndFunction/setModeIndFunction get
                and set an index into it.
            checkStopFunction (callable): Returns True once after a stop request.
            setup_actions (list): Setup action names, run by index with
                setSetupActionIndFunction.
            go_actions (list): Go action names, run by index with
                setGoActionIndFunction.
            getSettingsFunction (callable, optional): Returns the settings
                controls dict.
            setSettingFunction (callable, optional): Sets one setting by name.
            getHomeFunction (callable, optional): Returns the home GeoPoint,
                WGS84 latitude/longitude with altitude in meters above the
                WGS84 ellipsoid.
            setHomeFunction (callable, optional): Receives a home GeoPoint in
                the same frame as getHomeFunction returns.
            teleopControlsReadyFunction (callable, optional): Returns True while
                teleop commands will be acted on.
            setTeleopVelocityFunction (callable, optional): Receives
                (linear_x, linear_y, linear_z, angular_z) ratios in [-1, 1] in
                the body FLU frame: x forward, y left, z up, angular_z positive
                counter-clockwise (turn left). Passed through unconverted.
            autonomousControlsReadyFunction (callable, optional): Returns True
                while goto commands will be acted on.
            goHomeFunction (callable, optional): Goes home; returns success.
            goStopFunction (callable, optional): Stops motion; returns success.
            gotoPoseFunction (callable, optional): Receives a target attitude
                [roll_deg, pitch_deg, yaw_deg] in ENU, NavPose convention.
            gotoPositionFunction (callable, optional): Receives
                (point_enu_m, orientation_enu_deg): a geometry_msgs Point that
                is the ENU OFFSET (meters, x east, y north, z up) from the
                current position, and the absolute target attitude
                [roll_deg, pitch_deg, yaw_deg] in ENU.
            gotoLocationFunction (callable, optional): Receives
                (geopoint_amsl, orientation_enu_deg): a GeoPoint with WGS84
                latitude/longitude and altitude in meters AMSL, and the
                absolute target attitude [roll_deg, pitch_deg, yaw_deg] in ENU.
            gotoVelocityFunction (callable, optional): Receives
                (x_mps, y_mps, z_mps, yaw_degps, duration_s) in the body FLU
                frame, yaw rate positive counter-clockwise.
            manualControlsReadyFunction (callable, optional): Returns True while
                manual (servo) controls will be acted on.
            servo_if_dict (dict, optional): DeviceSVXIF instances by name.
            msg_if, node_if, settings_if, save_data_if, connect_navpose_if
                (optional): Shared interfaces; built here when None.
        """

        ##############################
        # Instance Copies of Class State
        ##############################
        self.caps_dict = copy.deepcopy(self.caps_dict)
        self.process_update_dict = copy.deepcopy(self.process_update_dict)
        self.process_data_dict = copy.deepcopy(self.process_data_dict)

        ##############################
        # Update Class Variables
        ##############################

        self.start_time = nepi_utils.get_time()

    



        ##########################
        # Create and start initializing Capabilities values



        # Copy the has_*_control axis flags into caps_dict unconditionally (the
        # class-level caps_dict starts empty, so the old "already a key" filter
        # dropped every one). NodeIF fills status fields by name from caps_dict,
        # so only keys that are DeviceRBXStatus fields are copied.
        if axisCapabilities is not None:
          for key in axisCapabilities.keys():
             if key in DeviceRBXStatus.__slots__:
                self.caps_dict[key] = axisCapabilities[key]       


        self.getHomeFunction = getHomeFunction
        self.setHomeFunction  = setHomeFunction
        self.caps_dict['has_set_home'] = self.setHomeFunction is not None
       

        self.goHomeFunction = goHomeFunction
        self.caps_dict['has_go_home'] = self.goHomeFunction is not None

        self.goStopFunction = goStopFunction
        self.caps_dict['has_go_stop'] = self.goStopFunction is not None

        self.gotoPoseFunction = gotoPoseFunction
        self.caps_dict['has_goto_pose'] = self.gotoPoseFunction is not None
         
        self.gotoPositionFunction = gotoPositionFunction
        self.caps_dict['has_goto_position'] = self.gotoPositionFunction is not None          

        self.gotoLocationFunction = gotoLocationFunction
        self.caps_dict['has_goto_location'] = self.gotoLocationFunction is not None

        self.gotoVelocityFunction = gotoVelocityFunction
        self.caps_dict['has_go_velocity'] = self.gotoVelocityFunction is not None

        self.getBatteryPercentFunction = getBatteryPercentFunction
        self.caps_dict['has_battery_feedback'] = self.getBatteryPercentFunction is not None

        ## Setup Manual Controls

        self.manualControlsReadyFunction = manualControlsReadyFunction
        if servo_if_dict is None:
           servo_if_dict = dict()
        self.servo_if_dict = servo_if_dict
        has_manual_controls = False
        if len(list(self.servo_if_dict.keys())) > 0:
           has_manual_controls = True
        self.caps_dict['has_manual_controls'] = has_manual_controls
          
    
        # Setup Autonomous Controls
        self.autonomousControlsReadyFunction = autonomousControlsReadyFunction
        self.caps_dict['has_autonomous_controls'] = self.autonomousControlsReadyFunction is not None





        ## Setup Teleop Controls
        # Driver-optional, like manual controls: both functions default to None,
        # and set_teleop_velocity commands are ignored unless the driver supplies
        # setTeleopVelocityFunction. There is no has_teleop_controls capability
        # flag; readiness is reported live in teleop_control_mode_ready (mirroring
        # manual/autonomous_control_mode_ready) so a client can tell "driver not
        # ready" apart from "broken" before sending a command, and is checked
        # again at command time in setTeleopVelocityCb.
        self.teleopControlsReadyFunction = teleopControlsReadyFunction
        self.setTeleopVelocityFunction = setTeleopVelocityFunction




  



        callback_dict = {
        'set_home': None,
        'go_home': None,
        'go_stop': None,
        'goto_pose': None,
        'goto_position': None,
        'goto_location': None,
        'goto_velocity': None,
        'mode': None,
        'state': None,
        'setup_action': None,
        'go_action': None
        }


        self.caps_dict['states'] = states
        self.getStateIndFunction = getStateIndFunction
        self.setStateIndFunction = setStateIndFunction

        self.caps_dict['modes'] = modes
        self.getModeIndFunction = getModeIndFunction
        self.setModeIndFunction = setModeIndFunction
        
        self.checkStopFunction = checkStopFunction

        self.caps_dict['setup_actions'] = setup_actions
        self.setSetupActionIndFunction = setSetupActionIndFunction

        self.caps_dict['go_actions'] = go_actions
        self.setGoActionIndFunction = setGoActionIndFunction




        # Initialize goto goals and errors messages
        self.goals_msg = GotoGoals()
        self.errors_msg = GotoErrors()


        ##########################
        # Initialize status message







        ##############################  
        # Setup NodeIF Updates
        ##############################  

        ##################
        update_info_dict = dict()
        ##################        
        update_caps_dict = self.caps_dict
        ##################        
        update_config_control_dict = dict()
        ##################       
        update_node_controls_dict = dict()
        ##################
        update_stats_dict = dict()
        ##################
        update_rui_dict = dict()
        ##################
        update_callback_dict = dict()

        ###########
        # NODE IF SETUP

        # Call the parent class constructor
        super().__init__(                
                device_info = device_info, 
                getSettingsFunction=getSettingsFunction,
                setSettingFunction=setSettingFunction, 
                factoryControls = dict(), 
                data_source_description = data_source_description,
                data_ref_description = data_ref_description,
                info_dict = update_info_dict,
                caps_dict = update_caps_dict, 
                stats_dict = update_stats_dict, 
                rui_dict = update_rui_dict, 
                config_control_dict = update_config_control_dict, 
                node_controls_dict = update_node_controls_dict, 
                callback_dict = update_callback_dict,              
                msg_if = msg_if,
                node_if = node_if,
                settings_if = settings_if,
                save_data_if = save_data_if,
                connect_navpose_if = connect_navpose_if
                )

        ready = self.wait_for_ready()
        ######################################################

        #######################
        # Update NodeIF NodeClass
        #######################

        alt_namespace = None
        if self.device_name != self.node_name:
            alt_namespace = self.node_namespace.replace(self.node_name,self.device_name)

        # Params Config Dict ####################
        params_dict = {
            self.node_if_prefix + 'cmd_timeout': {
                'name': 'cmd_timeout',
                'namespace': self.namespace,
                'factory_val': self.FACTORY_CMD_TIMEOUT_SEC
            },
            self.node_if_prefix + 'home_location': {
                'name': 'home_location',
                'namespace': self.namespace,
                'factory_val': self.FACTORY_HOME_LOCATION
            },
            self.node_if_prefix + 'max_error_m': {
                'name': 'max_error_m',
                'namespace': self.namespace,
                'factory_val': self.FACTORY_GOTO_MAX_ERROR_M
            },
            self.node_if_prefix + 'max_error_deg': {
                'name': 'max_error_deg',
                'namespace': self.namespace,
                'factory_val': self.FACTORY_GOTO_MAX_ERROR_DEG
            },
            self.node_if_prefix + 'stabilized_sec': {
                'name': 'stabilized_sec',
                'namespace': self.namespace,
                'factory_val': self.FACTORY_GOTO_STABILIZED_SEC
            },
            self.node_if_prefix + 'image_overlay_enabled': {
                'name': 'image_overlay_enabled',
                'namespace': self.namespace,
                'factory_val': self.image_overlay_enabled
            },
        }
    
        # Services Config Dict ####################
        services_dict = {
            self.node_if_prefix + 'status_query': {
                'namespace': self.namespace,
                'topic': 'status_query',
                'srv': DeviceRBXStatusQuery,
                'req': DeviceRBXStatusQueryRequest(),
                'resp': DeviceRBXStatusQueryResponse(),
                'callback': self.status_query_callback
            }
        }

        # Publishers Config Dict ####################
        pubs_dict = {}
                            

        # Subscribers Config Dict ####################
        subs_dict = {
            self.node_if_prefix + 'set_goto_error_bounds': {
                'namespace': self.namespace,
                'topic': 'set_goto_error_bounds',
                'msg': ErrorBounds,
                'qsize': None,
                'callback': self.setErrorBoundsCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'set_state': {
                'namespace': self.namespace,
                'topic': 'set_state',
                'msg': Int32,
                'qsize': None,
                'callback': self.setStateCb, 
                'callback_args': ()
            },

            self.node_if_prefix + 'set_mode': {
                'namespace': self.namespace,
                'topic': 'set_mode',
                'msg': Int32,
                'qsize': None,
                'callback': self.setModeCb, 
                'callback_args': ()
            },

            self.node_if_prefix + 'setup_action': {
                'namespace': self.namespace,
                'topic': 'setup_action',
                'msg': Int32,
                'qsize': None,
                'callback': self.setupActionCb,
                'callback_args': ()
            },

 

            self.node_if_prefix + 'set_teleop_velocity': {
                'namespace': self.namespace,
                'topic': 'set_teleop_velocity',
                # Stock geometry_msgs/Twist, not a new nepi_interfaces message.
                # linear.x/y/z and angular.z are the only fields any driver
                # reads; angular.x/y (roll/pitch RATE) are deliberately unused --
                # see setTeleopVelocityCb.
                'msg': Twist,
                # qsize=1, unlike set_motor_control above: a keyboard-driven
                # teleop client re-sends on every key event, so only the newest
                # command matters, never a queued-up backlog of stale ones.
                'qsize': 1,
                'callback': self.setTeleopVelocityCb,
                'callback_args': ()
            },

            self.node_if_prefix + 'go_action': {
                'namespace': self.namespace,
                'topic': 'go_action',
                'msg': Int32,
                'qsize': None,
                'callback': self.goActionCb,
                'callback_args': ()
            },

            self.node_if_prefix + 'set_goto_timeout': {
                'namespace': self.namespace,
                'topic': 'set_goto_timeout',
                'msg': UInt32,
                'qsize': None,
                'callback': self.setCmdTimeoutCb,
                'callback_args': ()
            },

            self.node_if_prefix + 'go_home': {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': None,
                'callback': self.goHomeCb,
                'callback_args': ()
            },

            self.node_if_prefix + 'set_home': {
                'namespace': self.namespace,
                'topic': 'set_home',
                'msg': GeoPoint,
                'qsize': None,
                'callback': self.setHomeCb, 
                'callback_args': ()
            },

            self.node_if_prefix + 'set_home_current': {
                'namespace': self.namespace,
                'topic': 'set_home_current',
                'msg': GotoLocation,
                'qsize': None,
                'callback': self.setHomeCurrentCb, 
                'callback_args': ()
            },

            self.node_if_prefix + 'goto_location': {
                'namespace': self.namespace,
                'topic': 'goto_location',
                'msg': GotoLocation,
                'qsize': None,
                'callback': self.gotoLocationCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'goto_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': GotoPosition,
                'qsize': None,
                'callback': self.gotoPositionCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'goto_pose': {
                'namespace': self.namespace,
                'topic': 'goto_pose',
                'msg': GotoPose,
                'qsize': None,
                'callback': self.gotoPoseCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'goto_velocity': {
                'namespace': self.namespace,
                'topic': 'goto_velocity',
                'msg': GotoVelocity,
                'qsize': None,
                'callback': self.gotoVelocityCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'go_stop': {
                'namespace': self.namespace,
                'topic': 'go_stop',
                'msg': Empty,
                'qsize': None,
                'callback': self.goStopCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'enable_image_overlay': {
                'namespace': self.namespace,
                'topic': 'enable_image_overlay',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableImageOverlayCb, 
                'callback_args': ()
            },
            self.node_if_prefix + 'set_process_name': {
                'namespace': self.namespace,
                'topic': 'set_process_name',
                'msg': String,
                'qsize': None,
                'callback': self.setProcessNameCb, 
                'callback_args': ()
            },
        }




        #######################
        self.add_params(params_dict)
        self.add_services(services_dict)
        self.add_pubs(pubs_dict)
        self.add_subs(subs_dict)
        nepi_sdk.sleep(0.1)



        ######################################################
        # Setup Connect Image IF Classes ####################
        self.connect_img_if = ConnectColorImageIF(data_callback = self.imageCb, 
                                                  msg_if = self.msg_if, 
                                                  node_if = self.node_if)



        ######################################################
        # Setup RBX Process IF ####################
        self.rbx_process_callback_dict = dict(
            process_update_callback = None,
            settings_updated_callback = None,
            controls_updated_callback = None,
        )


        self.rbx_process_config_dict = dict(
            has_sources = False,
            multi_source_enabled = False,
            has_auto_select = False,
            auto_select_enabled = True,
            selected_sources = [],
            
            has_enable = False,

            has_process_rate = True,
            min_max_process_rates = self.MIN_MAX_PROCESS_RATES,
            default_process_rate = self.DEFAULT_PROCESS_RATE,

            has_image_rate = False,
            min_max_image_rates = [1,10],
            default_image_rate = 10,
            has_use_last_image = False,

            has_process_pub = True,
            has_process_enable = True,
            has_process_reload = False,

            has_results_pub = True,

            has_save_data = True,
            has_config = True,

            has_image_pub = True,

            has_status_pub = True,
            throttle_status_sec = 0.1
        )

    
        self.process_if = ProcessIF(
                    process_name = self.rbx_process_name,
                    process_group = self.rbx_process_name,
                    process_description = self.rbx_process_name,
                    process_module = self.rbx_process_module,
                    callback_dict = self.rbx_process_callback_dict,
                    config_dict = self.rbx_process_config_dict,
                    msg_if = self.msg_if,
                    node_if = self.node_if
                )




        ######################################################
        # Setup Image IF Classes ####################
        self.msg_if.pub_info("Starting Image IF Initialization" )



        image_if_callback_dict = dict(
            needs_update_callback = None,
            mouse_event_callback = None,
            click_pixel_callback = None,
            click_angle_callback = None,
            drag_callback = None,
            window_callback = None,
            scroll_callback = None,
            frame_updated_callback = None,
            options_updated_callback = None,
            get_source_status_callback = self.connect_img_if.get_status_dict,
            get_source_navpose_callback = self.connect_img_if.get_navpose_dict,
            pre_process_callback = self.process_if.pre_process_image_function,
            post_process_callback = self.process_if.post_process_image_function
        )

        self.image_if = ColorImageIF(namespace = self.node_namespace,
                        callback_dict = image_if_callback_dict,
                        msg_if = self.msg_if,
                        node_if = self.node_if,                                                  
                        save_data_if = self.save_data_if,
                        navpose_if = self.navpose_if
                        )
        self.if_dict['image_if'] = self.image_if
        
        self.process_if.set_process_if_dict('image_if',self.image_if)

        #########################
        # Update Status Msg
        ##############################

        
        self.status_msg.device_name = self.device_name
        self.status_msg.device_path = self.path
        self.status_msg.device_node_name = self.node_name
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version


        self.status_msg.source_description = self.data_source_description
        self.status_msg.source_ref_description = self.data_ref_description

        self.status_msg.cmd_success = False


        self.status_msg.process_current = "None"
        self.status_msg.process_last = "None"
        self.status_msg.ready = False
        self.status_msg.battery = 0
        self.errors_msg.x_m = 0
        self.errors_msg.y_m = 0
        self.errors_msg.z_m = 0
        self.errors_msg.heading_deg = 0
        self.errors_msg.roll_deg = 0
        self.errors_msg.pitch_deg = 0
        self.errors_msg.yaw_deg = 0
        self.status_msg.errors_current = self.errors_msg
        errors_prev_msg = GotoErrors()
        errors_prev_msg.x_m = 0
        errors_prev_msg.y_m = 0
        errors_prev_msg.z_m = 0
        errors_prev_msg.heading_deg = 0
        errors_prev_msg.roll_deg = 0
        errors_prev_msg.pitch_deg = 0
        errors_prev_msg.yaw_deg = 0
        self.status_msg.errors_prev = errors_prev_msg
        self.goals_msg.x_m = 0
        self.goals_msg.y_m = 0
        self.goals_msg.z_m = 0
        self.goals_msg.heading_deg = 0
        self.goals_msg.roll_deg = 0
        self.goals_msg.pitch_deg = 0
        self.goals_msg.yaw_deg = 0
        self.status_msg.goals_current = self.goals_msg
        self.status_msg.last_error_message = ""


        ##############################
        # Start Node Processes
        ##############################



       ####################################
        ## Initiation Complete
        self.connected = True
        self.status_msg.ready = True 

        ##############################
        # Start Node Processes
        nepi_sdk.start_timer_process(1, self.blankImagePubCb)
        # processCb re-arms itself as a oneshot at the process rate
        nepi_sdk.start_timer_process(1, self.processCb, oneshot = True)
        # Refresh the current_* control attributes from this device's navpose.
        # The goto convergence loops, takeoff checks and home location all read
        # current_*; without this timer they stayed at their [0,0,0] defaults.
        nepi_sdk.start_timer_process(self.update_navpose_interval_sec, self._updateNavposeTimerCb)
        ####################################
        self.ready = True
        self.msg_if.pub_info("Initialization Complete" + ": " + self.node_name, color = 'GREEN')
        ####################################

        ##############################
        # Start NodeIF Processes
        if type(self) is RBXRobotIF:
            self.start()


    ###############################
    # NodeIF Class Override Methods
    ###############################

    def update_status_msg(self, status_msg):
        """Builds and publishes the DevicePTXStatus message to the status topic.

        Reads current pan/tilt position from the driver if absolute positioning is
        available, computes smoothed axis speeds, recalculates ratio and degree fields
        for current and goal positions against soft-stop limits, then publishes the
        assembled DevicePTXStatus message and a NavPosePanTilt message.

        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.

        Returns:
            float: Elapsed time in seconds spent building and publishing the status.
        """


        if self.ready == False:
            return status_msg

        status_msg.connected = self.connected

        status_msg.state = self.getStateIndFunction()
        status_msg.mode = self.getModeIndFunction()

    
      
        if self.getBatteryPercentFunction is not None:
          self.rbx_battery = self.getBatteryPercentFunction()
        else:
          self.rbx_battery = -999

        status_msg.last_cmd_string = self.last_cmd_string

        ## Update Control Info
        if self.manualControlsReadyFunction is not None:
          status_msg.manual_control_mode_ready = self.manualControlsReadyFunction()
        else:
          status_msg.manual_control_mode_ready = self.caps_dict['has_manual_controls'] 

        if self.teleopControlsReadyFunction is not None:
          status_msg.teleop_control_mode_ready = self.teleopControlsReadyFunction()
        else:
          status_msg.teleop_control_mode_ready = False

        # 

        if self.autonomousControlsReadyFunction is not None:
          status_msg.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
        else:
          status_msg.autonomous_control_mode_ready = False

         # Create Status Info Text List
        if status_msg.battery < 0.1:
            battery_string = "No Reading"
        else:
            battery_string = '%.2f' % status_msg.battery

        state_ind = self.getStateIndFunction()
        if len(self.caps_dict['states']) > 0 and state_ind >= 0 and state_ind <= (len(self.caps_dict['states'])):
              state_name = self.caps_dict['states'][state_ind]
        else:
              state_name = "Not Set"


        	
        mode_ind = self.getModeIndFunction()
        if len(self.caps_dict['modes']) > 0 and mode_ind >= 0 and mode_ind < (len(self.caps_dict['modes']) ):
              mode_name = self.caps_dict['modes'][mode_ind]
        else:
              mode_name = "Not Set"
        
 

        status_msg.data_products = self.data_products_list

        # The option lists live in caps_dict under 'states'/'modes'/
        # 'setup_actions'/'go_actions', which are not the status field names,
        # so NodeIF's name-matched status fill never set them and the RUI's
        # state, mode and action selectors (NepiDeviceRBX.js) stayed empty.
        status_msg.state_options = list(self.caps_dict['states'])
        status_msg.mode_options = list(self.caps_dict['modes'])
        status_msg.setup_action_options = list(self.caps_dict['setup_actions'])
        status_msg.go_action_options = list(self.caps_dict['go_actions'])
        # Same name mismatch: the capability key is 'has_go_velocity', the
        # status field is has_goto_velocity.
        status_msg.has_goto_velocity = self.caps_dict['has_go_velocity']


        try:
           status_msg.manual_control_mode_ready = self.manualControlsReadyFunction()
        except:
           pass

        try:
           status_msg.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
        except:
           pass

        try:
           status_msg.teleop_control_mode_ready = self.teleopControlsReadyFunction()
        except:
           pass

        
        error_bounds = ErrorBounds()
        error_bounds.max_distance_error_m = self.node_if.get_param(self.node_if_prefix + 'max_error_m')
        error_bounds.max_rotation_error_deg = self.node_if.get_param(self.node_if_prefix + 'max_error_deg')
        error_bounds.min_stabilize_time_s = self.node_if.get_param(self.node_if_prefix + 'stabilized_sec')
        status_msg.error_bounds = error_bounds
        status_msg.cmd_timeout = self.node_if.get_param(self.node_if_prefix + 'cmd_timeout')
        status_msg.state = self.getStateIndFunction()
        status_msg.mode = self.getModeIndFunction()
        if self.getHomeFunction is not None:
          home_geo = self.getHomeFunction()
          home_location = [home_geo.latitude,home_geo.longitude,home_geo.altitude]   
        else: # No Home Information
          home_location = [-999,-999,-999]    
        status_msg.home_lat = home_location[0]
        status_msg.home_long = home_location[1]
        status_msg.home_alt = home_location[2]


        ###################
        # TOPIC UPDATES
        
        ########
        # NAVPOSE DATA
        try:
           navpose_topic = self.navpose_if.get_namespace()
        except:
           navpose_topic = ''
        status_msg.navpose_topic = navpose_topic


        try:
           navpose_msg = self.navpose_if.get_navpose_msg()
           status_msg.navpose_msg = navpose_msg
        except:
           pass

        try:
           connect_navpose_topic = self.connect_navpose_if.get_namespace()
        except:
           connect_navpose_topic = ''
        status_msg.connect_navpose_topic = connect_navpose_topic


        ########
        # IMAGE DATA
        try:
           pub_image_topic = self.image_if.get_namespace()
        except:
           pub_image_topic = ''
        status_msg.pub_image_topic = pub_image_topic


        try:
           connect_image_topic = self.connect_img_if.get_namespace()
        except:
           connect_image_topic = ''
        status_msg.connect_image_topic = connect_image_topic


        ########
        # Servo DATA
        servo_names = []
        servo_topics = []
        for servo_name in self.servo_if_dict.keys():
            servo_if = self.servo_if_dict[servo_name]
            try:
                servo_topics.append(servo_if.get_namespace())
                servo_names.append(servo_name)
            except:
                pass

        status_msg.servo_names = servo_names
        status_msg.servo_topics = servo_topics


        return status_msg

             
   ###############################
    # Class Methods
    ###############################
    def _updateNavposeTimerCb(self, timer):
        # Periodic bridge from the device's own navpose to the current_* control
        # attributes (see setup note). The now-dead navposesSysCb used to be the
        # only caller of _updateCurrentNavpose.
        self._updateCurrentNavpose()

    def _updateCurrentNavpose(self):
        # Reads this device's navpose (navpose_if), which republishes the source
        # selected on connect_navpose_if -- for a driver that hosts its own
        # NPXDeviceIF and selects it, that is the driver's own navpose. Values
        # are in the NavPose convention: ENU orientation and position, compass
        # heading_deg, WGS84 location. The *_ned copies below are kept for
        # external readers only; no control logic in this class uses them.
        navpose_dict = self.get_navpose_dict()
        if navpose_dict is None:
            return
        try:
            # NavPoseIF converts to the system frame_nav setting, so the dict can
            # arrive as NED. All control math here is ENU: convert a copy back.
            if navpose_dict.get('frame_nav', 'ENU') == 'NED':
                navpose_dict = nepi_nav.convert_navpose_ned2enu(copy.deepcopy(navpose_dict))
            # Likewise for the system frame_altitude setting: an AMSL-tagged dict is
            # converted back, so altitude_m here is always ellipsoid height.
            if navpose_dict.get('frame_altitude', 'WGS84') == 'AMSL':
                navpose_dict = nepi_nav.convert_navpose_amsl2wgs84(copy.deepcopy(navpose_dict))
            self.navpose_dict = navpose_dict
            self.current_heading_deg = navpose_dict['heading_deg']
            self.current_geoid_height_m = navpose_dict['geoid_height_meters']
            # Orientation: native ENU vector, plus NED conversion
            self.current_orientation_enu_degs = [navpose_dict['roll_deg'], navpose_dict['pitch_deg'], navpose_dict['yaw_deg']]
            ned_dict = nepi_nav.convert_navpose_enu2ned(copy.deepcopy(navpose_dict))
            self.current_orientation_ned_degs = [ned_dict['roll_deg'], ned_dict['pitch_deg'], ned_dict['yaw_deg']]
            # Position: native ENU vector, plus NED conversion
            self.current_position_enu_m = [navpose_dict['x_m'], navpose_dict['y_m'], navpose_dict['z_m']]
            self.current_position_ned_m = [ned_dict['x_m'], ned_dict['y_m'], ned_dict['z_m']]
            # Location: WGS84 geopoint, plus AMSL. AMSL = ellipsoid altitude -
            # geoid height, using this navpose's own geoid_height_meters, the
            # same value setpoint_location_global_wgs84 sends the setpoint with.
            self.current_location_wgs84_geo = [navpose_dict['latitude'], navpose_dict['longitude'], navpose_dict['altitude_m']]
            self.current_location_amsl_geo = [navpose_dict['latitude'], navpose_dict['longitude'], navpose_dict['altitude_m'] - self.current_geoid_height_m]
        except Exception as e:
            self.msg_if.pub_warn("Failed to update current navpose: " + str(e), throttle_s = 5.0 )
            

    ##############################
    # RBX Settings Topic Callbacks

    # ToDo: Create a custom RBX status message
    ### Callback to set state
    def setStateCb(self,state_msg):
        self.msg_if.pub_info("Received set state message" )
        self.msg_if.pub_info(state_msg)
        state_val = state_msg.data
        self.setState(state_val)
        

    ### Function to set state
    def setState(self,new_state_ind):
        if new_state_ind < 0 or new_state_ind > (len(self.caps_dict['states'])-1):
            self.update_error_msg("No matching rbx state found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.update_current_goals( [0,0,0,0,0,0,0] )
            self.status_msg.process_current = self.caps_dict['states'][new_state_ind]
            self.rbx_state_last = self.status_msg.state
            self.msg_if.pub_info("Waiting for rbx state " + self.caps_dict['states'][new_state_ind] + " to set" )
            self.setStateIndFunction(new_state_ind)
            time.sleep(1)
            self.msg_if.pub_info("Current rbx state is " + self.caps_dict['states'][self.getStateIndFunction()])
            self.status_msg.process_last = self.caps_dict['states'][new_state_ind]
            self.status_msg.process_current = "None"
            str_val = self.caps_dict['states'][new_state_ind]
            self.last_cmd_string = "set_rbx_state(self,'" + str_val + "',timeout_sec = " + str(self.status_msg.cmd_timeout)
        self.publish_status()
        

    ### Callback to set mode
    def setModeCb(self,mode_msg):
        self.msg_if.pub_info("Received set mode message" )
        self.msg_if.pub_info(mode_msg)
        mode_val = mode_msg.data
        self.setMode(mode_val)

    ### Function to set mode
    def setMode(self,new_mode_ind):
        if new_mode_ind < 0 or new_mode_ind > (len(self.caps_dict['modes'])-1):
            self.update_error_msg("No matching rbx mode found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.update_current_goals( [0,0,0,0,0,0,0] )
            self.status_msg.process_current = self.caps_dict['modes'][new_mode_ind]
            self.msg_if.pub_info("Setting rbx mode to : " + self.caps_dict['modes'][new_mode_ind])
            self.setModeIndFunction(new_mode_ind)
            self.msg_if.pub_info("Current rbx mode is " + self.caps_dict['modes'][self.getModeIndFunction()])
            self.status_msg.process_last = self.caps_dict['modes'][new_mode_ind]
            self.status_msg.process_current = "None"
            str_val = self.caps_dict['modes'][new_mode_ind]
            self.last_cmd_string = "set_rbx_mode(self,'" + str_val + "',timeout_sec = " + str(self.status_msg.cmd_timeout)
        self.publish_status()

    ### Callback to execute action
    def setupActionCb(self,action_msg):
        self.msg_if.pub_info("Received setup action message" )
        self.msg_if.pub_info(action_msg)
        action_ind = action_msg.data
        if self.setSetupActionIndFunction is not None:
            if action_ind < 0 or action_ind > (len(self.caps_dict['setup_actions'])-1):
                self.update_error_msg("No matching rbx action found")
            else:
                if self.status_msg.ready is False:
                    self.update_error_msg("Another Command Process is Active")
                    self.update_error_msg("Ignoring this Request")
                else:
                    self.status_msg.process_current = self.caps_dict['setup_actions'][action_ind]
                    self.status_msg.ready = False
                    self.rbx_cmd_success_current = False
                    # Clear the previous run's error before starting. Nothing else
                    # clears last_error_message, so a failed action (e.g. a LAUNCH
                    # that would not ARM) stayed on the RUI even after a later
                    # attempt succeeded. Cleared at action START, not on success,
                    # so an error raised DURING the action -- including FCU prearm
                    # text relayed by update_error_msg -- survives to be read.
                    self.status_msg.last_error_message = ""
                    self.msg_if.pub_info("Starting action: " + self.caps_dict['setup_actions'][action_ind])
                    success = self.setSetupActionIndFunction(action_ind)
                    self.rbx_cmd_success_current = success
                    if success:
                      self.msg_if.pub_info("Finished action: " + self.caps_dict['setup_actions'][action_ind])
                    else:
                      self.msg_if.pub_info("Action: " + self.caps_dict['setup_actions'][action_ind] + " Failed to complete" )
                    self.status_msg.process_last = self.caps_dict['setup_actions'][action_ind]
                    self.status_msg.process_current = "None"
                    self.status_msg.cmd_success = self.rbx_cmd_success_current
                    time.sleep(0.5)
                    self.status_msg.ready = True

                    str_val = self.caps_dict['setup_actions'][action_ind]
                    self.last_cmd_string = "setup_rbx_action(self,'" + str_val + "',timeout_sec = " + str(self.status_msg.cmd_timeout)
                    self.publish_status()
        else:
            self.update_error_msg("Ignoring Setup Action command, no Set Action Function")




    ### Callback to start rbx set goto goals process
    def setErrorBoundsCb(self,error_bounds_msg):
        self.msg_if.pub_info("Received set goals message" )
        self.msg_if.pub_info(error_bounds_msg)
        self.node_if.set_param(self.node_if_prefix + 'max_error_m', error_bounds_msg.max_distance_error_m)
        self.node_if.set_param(self.node_if_prefix + 'max_error_deg', error_bounds_msg.max_rotation_error_deg)
        self.node_if.set_param(self.node_if_prefix + 'stabilized_sec', error_bounds_msg.min_stabilize_time_s)
        self.status_msg.error_bounds = error_bounds_msg
        self.publish_status()

    ### Callback to set cmd timeout
    def setCmdTimeoutCb(self,cmd_timeout_msg):
        self.msg_if.pub_info("Received set timeout message" )
        self.msg_if.pub_info(cmd_timeout_msg)
        self.node_if.set_param(self.node_if_prefix + 'cmd_timeout', cmd_timeout_msg.data)
        self.status_msg.cmd_timeout = cmd_timeout_msg.data 
        self.publish_status()


    ### Callback to add overlay to image topic source
    def enableImageOverlayCb(self,enable_msg):
        enabled = enable_msg.data
        self.set_value('image_overlay_enabled', enabled)
    
    ### Callback to set current process name
    def setProcessNameCb(self,set_process_name_msg):
        self.msg_if.pub_info("Received set process name message" )
        self.msg_if.pub_info(set_process_name_msg)
        self.status_msg.process_current = (set_process_name_msg.data)

    def setNavposeFrameCb(self,msg):
        self.msg_if.pub_info("Recived set navpose_frame message: " + str(msg))
        self.navpose_frame = msg.data
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param(self.node_if_prefix + 'navpose_frame', self.navpose_frame)
 
    

    ##############################
    # RBX Control Topic Callbacks


    ### Callback to set teleop (keyboard-driven) velocity
    def setTeleopVelocityCb(self,twist_msg):
        self.msg_if.pub_info("Received teleop velocity message")
        if self.setTeleopVelocityFunction is None:
            self.update_error_msg("Ignoring teleop velocity command, no teleop function")
            return
        if self.teleopControlsReadyFunction is None or self.teleopControlsReadyFunction() is not True:
            self.update_error_msg("Ignoring teleop velocity command, " + self._notReadyReason("Teleop Controls"))
            return
        # Body FLU ratios in [-1,1], passed through unconverted: linear.x
        # positive is forward, linear.y positive is LEFT, linear.z positive is
        # up, angular.z positive is counter-clockwise (turn left). The driver
        # converts to whatever frame its hardware takes. angular.x/y
        # (roll-rate/pitch-rate) are deliberately not read here: a rover has no
        # meaning for them at all, and for a drone the teleop horizontal
        # controls are body-frame VELOCITY (linear.x, linear.y) -- not raw
        # attitude-rate setpoints, which would fight the flight controller's
        # own attitude loop and are a riskier command to accept from a
        # keyboard with no stick centering/failsafe.
        self.setTeleopVelocityFunction(twist_msg.linear.x, twist_msg.linear.y,
                                       twist_msg.linear.z, twist_msg.angular.z)


    ### Callback to set home
    def setHomeCb(self,geo_msg):
        self.msg_if.pub_info("Received set home message" )
        self.msg_if.pub_info(geo_msg)
        new_home_loc = [geo_msg.latitude,geo_msg.longitude,geo_msg.altitude]
        for i, loc in enumerate(new_home_loc):
          if loc == -999:
            new_home_loc[i] = self.current_location_wgs84_geo[i]

        if self.process_data_dict['has_set_home'] == True:
           self.process_data_dict['set_home_success'] = None 
           self.process_update_dict['set_home'] = new_home_loc

        elif self.setHomeFunction is not None:
            new_home_geo = GeoPoint()
            new_home_geo.latitude = new_home_loc[0]
            new_home_geo.longitude = new_home_loc[1]
            new_home_geo.altitude = new_home_loc[2]
            self.setHomeFunction(new_home_geo)


    ### Callback to set home current
    def setHomeCurrentCb(self,empty_msg):
        self.msg_if.pub_info("Received set home current message" )
        if self.setHomeFunction is not None:
            # setHomeFunction takes a GeoPoint (as setHomeCb and
            # ApplyConfigUpdates pass); current_location_wgs84_geo is a list.
            cur_loc = list(self.current_location_wgs84_geo)
            new_home_geo = GeoPoint()
            new_home_geo.latitude = cur_loc[0]
            new_home_geo.longitude = cur_loc[1]
            new_home_geo.altitude = cur_loc[2]
            self.setHomeFunction(new_home_geo)

          


    ### Callback to start rbx go home
    def goHomeCb(self,home_msg):
        self.msg_if.pub_info("Received go home message" )
        if self.process_data_dict['has_go_home'] == True or self.goHomeFunction is not None:
            self.status_msg.process_current = "Go Home"
            self.rbx_cmd_success_current = False
            self.status_msg.ready = False
            self.update_prev_errors()
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.update_current_goals( [0,0,0,0,0,0,0] )

            if self.process_data_dict['has_go_home'] == True:
                self.process_data_dict['go_home_success'] = None 
                self.process_update_dict['go_home'] = True
                success = False
                while self.process_ready == True and not nepi_sdk.is_shutdown():
                    success = self.process_data_dict['go_home_success'] == True
                self.rbx_cmd_success_current = success
                self.process_data_dict['go_home_success'] = None 
            elif self.goHomeFunction is not None:
                self.rbx_cmd_success_current = self.goHomeFunction()

            self.status_msg.process_last = "Go Home"
            self.status_msg.process_current = "None"
            self.status_msg.cmd_success = self.rbx_cmd_success_current
            time.sleep(0.5)
            self.status_msg.ready = True
            self.last_cmd_string = "go_rbx_home(self,timeout_sec = " + str(self.status_msg.cmd_timeout)
            self.publish_status()

    ### Callback to start rbx stop
    def goStopCb(self,stop_msg):
        self.msg_if.pub_info("Received go stop message" )
        self.msg_if.pub_info(stop_msg)
        # No time.sleep(1) here, unlike the goto callbacks: this callback has no
        # "another command is active" check to debounce, and a one-second delay
        # on a stop let a running goto loop keep driving toward its old target
        # for that whole second before the stop was even issued.
        if self.process_data_dict['has_go_stop'] == True or  self.goStopFunction is not None:
            self.status_msg.process_current = "Stop"
            self.rbx_cmd_success_current = False
            self.status_msg.ready = False
            self.update_prev_errors()
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.update_current_goals( [0,0,0,0,0,0,0] )

            if self.process_data_dict['has_go_stop'] == True:
                self.process_data_dict['go_stop_success'] = None 
                self.process_update_dict['go_stop'] = True
                success = False
                while self.process_ready == True and not nepi_sdk.is_shutdown():
                    success = self.process_data_dict['go_stop_success'] == True
                self.rbx_cmd_success_current = success
                self.process_data_dict['go_stop_success'] = None 
            elif self.goStopFunction is not None:
                self.rbx_cmd_success_current = self.goStopFunction()
            self.status_msg.process_last = "Stop"
            self.status_msg.process_current = "None"
            self.status_msg.cmd_success = self.rbx_cmd_success_current
            time.sleep(0.5)
            self.status_msg.ready = True
            self.last_cmd_string = "go_rbx_stop(self,timeout_sec = " + str(self.status_msg.cmd_timeout)
            self.publish_status()

 
  ### Callback to execute action
    def goActionCb(self,action_msg):
        self.msg_if.pub_info("Received go action message" )
        self.msg_if.pub_info(action_msg)
        action_ind = action_msg.data
        if action_ind < 0 or action_ind > (len(self.caps_dict['go_actions'])-1):
            self.update_error_msg("No matching rbx action found")
            return

        action_name = self.caps_dict['go_actions'][action_ind]
        if action_name in self.process_data_dict['go_action_options'] == True or self.setGoActionIndFunction is not None:


            if self.status_msg.ready is False:
                self.update_error_msg("Another GoTo Command Process is Active")
                self.update_error_msg("Ignoring this Request")
            else:
                self.status_msg.process_current = self.caps_dict['go_actions'][action_ind]
                self.status_msg.ready = False
                self.rbx_cmd_success_current = False
                self.msg_if.pub_info("Starting action: " + self.caps_dict['go_actions'][action_ind])

                if action_name in self.process_data_dict['go_action_options'] == True: 
                    self.process_data_dict['go_action_success'] = None 
                    self.process_update_dict['go_action'] = action_name
                    success = False
                    while self.process_ready == True and not nepi_sdk.is_shutdown():
                        success = self.process_data_dict['go_action_success'] == True
                    self.rbx_cmd_success_current = success
                    self.process_data_dict['go_action_success'] = None 
                elif self.setGoActionIndFunction is not None:
                    success = self.setGoActionIndFunction(action_ind)
                    self.rbx_cmd_success_current = success
                if success:
                    self.msg_if.pub_info("Finished action: " + self.caps_dict['go_actions'][action_ind])
                else:
                    self.msg_if.pub_info("Action: " + self.caps_dict['go_actions'][action_ind] + " Failed to complete" )
                self.status_msg.process_last = self.caps_dict['go_actions'][action_ind]
                self.status_msg.process_current = "None"
                self.status_msg.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.status_msg.ready = True

                str_val = self.caps_dict['go_actions'][action_ind]
                self.last_cmd_string = "go_rbx_action(self,'" + str_val + "',timeout_sec = " + str(self.status_msg.cmd_timeout)
                self.publish_status()
        else:
            self.update_error_msg("Ignoring Go Action command, no Set Action Function")


    ### Callback to start rbx goto pose process
    def gotoPoseCb(self,pose_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Pose Message" )
        self.msg_if.pub_info(pose_cmd_msg)
        time.sleep(1)
        if self.autonomousControlsReadyFunction() is True or self.process_data_dict['has_goto_pose'] == True:
            setpoint_data=[pose_cmd_msg.roll_deg,pose_cmd_msg.pitch_deg,pose_cmd_msg.yaw_deg]
            if self.status_msg.ready is False:
                self.update_error_msg("Ignoring GoTo POSE Request, Another GoTo Command Process is Active")
            else:
                self.status_msg.process_current = "GoTo Pose"
                self.status_msg.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.update_current_goals( [0,0,0,0,0,0,0] )

                self.rbx_cmd_success_current = self.setpoint_attitude_enu(setpoint_data)
                self.status_msg.process_last = "GoTo Pose"
                self.status_msg.process_current = "None"
                self.status_msg.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.status_msg.ready = True

                str_val = str(setpoint_data)
                self.last_cmd_string = "goto_rbx_pose(self,'" + str_val + "',timeout_sec = " + str(self.status_msg.cmd_timeout)
                self.publish_status()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")



    ### Callback to start rbx goto velocity process
    #
    # STRUCTURALLY DIFFERENT from gotoPoseCb and gotoPositionCb above, and this
    # is the thing a later reader will get wrong. Those two call
    # setpoint_attitude_enu / setpoint_position_local_body, which are BLOCKING
    # CONVERGENCE LOOPS: they hold a target and compare it against the live
    # navpose until the error is inside bounds or cmd_timeout expires.
    #
    # goto_velocity has no target to converge on. It is OPEN LOOP. The injected
    # gotoVelocityFunction owns the timing and is expected to block for
    # duration_s, and NOTHING IN THIS CLASS MEASURES THE RESULT -- the errors are
    # zeroed on entry and never updated, and cmd_success is whatever the driver
    # function returned, not a verdict on where the robot ended up. A device is
    # expected to stop itself when duration_s elapses; this class does not stop
    # it.
    def gotoVelocityCb(self,velocity_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Velocity Message" )
        self.msg_if.pub_info(velocity_cmd_msg)
        if self.autonomousControlsReadyFunction() is True or self.process_data_dict['has_go_velocity'] == True:
            setpoint_data=[velocity_cmd_msg.x_mps,velocity_cmd_msg.y_mps,velocity_cmd_msg.z_mps,
                           velocity_cmd_msg.yaw_degps,velocity_cmd_msg.duration_s]
            if self.status_msg.ready is False:
                self.update_error_msg("Ignoring GoTo Velocity Request, Another GoTo Command Process is Active")
            else:
                self.status_msg.process_current = "GoTo Velocity"
                self.status_msg.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.update_current_goals( [0,0,0,0,0,0,0] )


                if self.process_data_dict['has_go_velocity'] == True:
                    self.process_data_dict['go_velocity_success'] = None 
                    self.process_update_dict['go_velocity'] = setpoint_data
                    success = False
                    while self.process_ready == True and not nepi_sdk.is_shutdown():
                        success = self.process_data_dict['go_velocity_success'] == True
                    self.rbx_cmd_success_current = success
                    self.process_data_dict['go_velocity_success'] = None 
                elif self.gotoVelocityFunction is not None:
                    self.rbx_cmd_success_current = self.gotoVelocityFunction(velocity_cmd_msg.x_mps,
                                                                             velocity_cmd_msg.y_mps,
                                                                             velocity_cmd_msg.z_mps,
                                                                             velocity_cmd_msg.yaw_degps,
                                                                             velocity_cmd_msg.duration_s)
                self.status_msg.process_last = "GoTo Velocity"
                self.status_msg.process_current = "None"
                self.status_msg.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.status_msg.ready = True

                str_val = str(setpoint_data)
                self.last_cmd_string = "goto_rbx_velocity(self,'" + str_val + "',timeout_sec = " + str(self.status_msg.cmd_timeout)
                self.publish_status()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")


    ### Callback to start rbx goto position process
    def gotoPositionCb(self,position_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Position Command Message" )
        self.msg_if.pub_info(position_cmd_msg)
        time.sleep(1)
        # Gated on autonomous readiness like gotoPoseCb/gotoVelocityCb/
        # gotoLocationCb. The old "manual_control_mode_ready is False" gate was
        # inverted: a driver whose manual controls are always ready (rbx_ardupilot)
        # could never goto position, and ones gated on a live connection
        # (rbx_gazebo, custom_robot) only accepted it while disconnected.
        if self.autonomousControlsReadyFunction() is True or self.process_data_dict['has_goto_position'] == True:
            setpoint_data=[position_cmd_msg.x_meters,position_cmd_msg.y_meters,position_cmd_msg.z_meters,position_cmd_msg.yaw_deg]
            if self.status_msg.ready is False:
                self.update_error_msg("Ignoring GoTo Position Request, Another GoTo Command Process is Active")
            else:
                self.status_msg.process_current = "GoTo Position"
                self.status_msg.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.update_current_goals( [0,0,0,0,0,0,0] )

                self.rbx_cmd_success_current = self.setpoint_position_local_body(setpoint_data)
                self.status_msg.process_last = "GoTo Position"
                self.status_msg.process_current = "None"
                self.status_msg.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.status_msg.ready = True
                
                str_val = str(setpoint_data)
                self.last_cmd_string = "goto_rbx_position(self,'" + str_val + "',timeout_sec = " + str(self.status_msg.cmd_timeout)
                self.publish_status()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to start rbx goto location subscriber
    def gotoLocationCb(self,location_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Location Message" )
        self.msg_if.pub_info(location_cmd_msg)
        if self.autonomousControlsReadyFunction() is True or self.process_data_dict['has_goto_location'] == True:
            setpoint_data=[location_cmd_msg.lat,location_cmd_msg.long,location_cmd_msg.altitude_meters,location_cmd_msg.yaw_deg]
            if self.status_msg.ready is False:
                self.update_error_msg("Ignoring GoTo Location Request, Another GoTo Command Process is Active")
            else:
                self.status_msg.process_current = "GoTo Location"
                self.status_msg.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.update_current_goals( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.setpoint_location_global_wgs84(setpoint_data)
                self.status_msg.process_last = "GoTo Location"
                self.status_msg.process_current = "None"
                self.status_msg.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.status_msg.ready = True

                str_val = str(setpoint_data)
                self.last_cmd_string = "goto_rbx_location(self,'" + str_val + "',timeout_sec = " + str(self.status_msg.cmd_timeout)
                self.publish_status()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")



    def ApplyConfigUpdates(self):
        if self.setHomeFunction is not None:
          home_location = self.node_if.get_param(self.node_if_prefix + 'home_location')
          geo_home = GeoPoint()
          geo_home.latitude = home_location[0]
          geo_home.longitude = home_location[1]
          geo_home.altitude = home_location[2]
          self.setHomeFunction(geo_home)

  
    def setCurrentAsDefault(self):
        if self.settings_if is not None:
          # initialize_settings() has never existed on SettingsIF -- this raised
          # AttributeError on every setCurrentAsDefault(). init() is the method.
          self.settings_if.init(do_updates = False)
        pass # We only use the param server, no member variables to apply to param server
   
    def status_query_callback(self, _):
        """Handles an DeviceRBXStatusQuery service request.

        Args:
            _: Unused service request object.

        Returns:
            DeviceRBXStatusQueryResponse: The current DeviceRBXStatus in
                device_status, which carries the supported control axes,
                states, modes, actions, and data products.
        """
        return DeviceRBXStatusQueryResponse(device_status = self.status_msg)
    

 

    #######################
    # RBX IF Methods

    ### Function to set and check setpoint attitude ENU command
    ###################################################
    # Input is [ROLL_ENU_DEG, PITCH_ENU_DEG, YAW_ENU_DEG] in the NavPose convention
    # Sent to the driver in ENU, unconverted
    ###################################################
    def setpoint_attitude_enu(self,setpoint_attitude):
      """Commands the robot to a target attitude in the ENU world frame.

      Sends the ENU attitude to gotoPoseFunction unconverted, then blocks
      until the attitude goal is reached within error bounds or the command
      times out. Goals are the target ENU attitude; errors are goal minus
      current, wrapped to (-180, 180].

      Args:
          setpoint_attitude (list): Target attitude as
              [ROLL_ENU_DEG, PITCH_ENU_DEG, YAW_ENU_DEG] in the NavPose
              convention (yaw 0 faces east, positive counter-clockwise). Use
              -999 for any axis to hold the current value; a held axis is
              never sent as -999 and never counted as an error.

      Returns:
          bool: True if the attitude setpoint was reached within bounds and
              timeout, False otherwise.
      """
      # setpoint_attitude is [ROLL_ENU_DEG, PITCH_ENU_DEG, YAW_ENU_DEG]
      # Use value -999 to use current value
      cmd_success = True
      timeout_sec = self.status_msg.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.update_current_goals( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info("" )
      self.msg_if.pub_info("************************" )
      self.msg_if.pub_info("Starting Setpoint Attitude Process" )
      ##############################################
      # Capture Current NavPoses Data
      ##############################################
      start_orientation_enu_degs=list(self.current_orientation_enu_degs)
      self.msg_if.pub_info("Attitude Current ENU Roll, Pitch, Yaw in Degrees" )
      self.msg_if.pub_info(["%.2f" % start_orientation_enu_degs[0],"%.2f" % start_orientation_enu_degs[1],"%.2f" % start_orientation_enu_degs[2]])
      ##############################################
      # Condition Inputs
      ##############################################
      input_attitude_enu_degs = list(setpoint_attitude)
      # Set new attitude in degs ENU. A -999 (held) axis takes the current
      # value, so -999 is never sent as a command.
      new_attitude_enu_degs=list(start_orientation_enu_degs) # Initialize with start values
      for ind in range(3): # Overwrite current with new if set and valid
        if setpoint_attitude[ind] != -999:
          new_attitude_enu_degs[ind]=setpoint_attitude[ind]
        # Condition to (-180,180] deg
        new_attitude_enu_degs[ind] = self._wrapAngleDeg(new_attitude_enu_degs[ind])
      self.msg_if.pub_info("Attitude Goal ENU Roll, Pitch, Yaw in Degrees" )
      self.msg_if.pub_info(["%.2f" % new_attitude_enu_degs[0],"%.2f" % new_attitude_enu_degs[1],"%.2f" % new_attitude_enu_degs[2]])
      ##############################################
      ## Send Setpoint Message and Check for Success
      ##############################################
      # Update current goals with ENU attitude goal in degrees
      self.update_current_goals( [None,None,None,None,new_attitude_enu_degs[0],new_attitude_enu_degs[1],new_attitude_enu_degs[2]] )
      self.msg_if.pub_info("Sending Setpoint Attitude Command" )
      if self.process_data_dict['has_goto_pose'] == True:
        self.process_data_dict['go_attitude_success'] = None 
        self.process_update_dict['go_attitude'] = new_attitude_enu_degs
      elif self.gotoPoseFunction is not None:
        self.gotoPoseFunction(new_attitude_enu_degs)
      self.msg_if.pub_info("Waiting for Attitude Setpoint to complete" )
      setpoint_attitude_reached = False
      stabilize_timer=0
      timeout_timer = 0 # Initialize timeout timer
      attitude_errors = [] # Initialize running list of errors
      attitude_errors_degs = [0.0,0.0,0.0]
      time2sleep = 0.1
      while setpoint_attitude_reached is False and not nepi_sdk.is_shutdown():  # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
            self.msg_if.pub_info("Setpoint Attitude received Stop Command" )
            # Hold the current attitude on stop (same source used below for error calc).
            new_attitude_enu_degs = list(self.current_orientation_enu_degs)
            self.update_current_goals( [None,None,None,None,new_attitude_enu_degs[0],new_attitude_enu_degs[1],new_attitude_enu_degs[2]] )
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment message timer
        timeout_timer = timeout_timer+time2sleep
        # Calculate setpoint attitude errors: goal minus current, wrapped
        cur_attitude_enu_degs = [self.current_orientation_enu_degs[0],self.current_orientation_enu_degs[1],self.current_orientation_enu_degs[2]]
        attitude_errors_degs = [0.0,0.0,0.0]
        for ind in range(3):
          if input_attitude_enu_degs[ind] != -999: # Ignore error check if set to current
            attitude_errors_degs[ind] = self._wrapAngleDeg(new_attitude_enu_degs[ind] - cur_attitude_enu_degs[ind])
        max_attutude_error_deg = max(np.abs(attitude_errors_degs))
        # Check for setpoint position local point goal
        if  setpoint_attitude_reached is False:
          if stabilize_timer > self.status_msg.error_bounds.min_stabilize_time_s:
            max_attitude_errors = max(attitude_errors) # Get max from error window
            attitude_errors = [max_attutude_error_deg] # reset running list of errors
            if max_attitude_errors < self.status_msg.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Attitude Setpoint Reached" )
              setpoint_attitude_reached = True
          else:
            attitude_errors.append(max_attutude_error_deg) # append last
        # Reset timer if past
        if stabilize_timer > self.status_msg.error_bounds.min_stabilize_time_s:
          stabilize_timer=0 # Reset timer
        self.update_current_errors( [None,None,None,None,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
      if cmd_success:
        self.msg_if.pub_info("************************" )
        self.msg_if.pub_info("Setpoint Reached" )
      self.update_current_errors( [None,None,None,None,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
      return cmd_success
      



    ### Function to set and check setpoint position local body command
    ###################################################
    # Input is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YAW_BODY_DEGREES]
    # Converted to Local ENU Frame before sending
    # Local Body Position Setpoint Function use these body relative (FLU) x,y,z,yaw conventions
    # x+ axis is forward
    # y+ axis is left
    # z+ axis is up
    # Only yaw orientation updated
    # yaw+ counter clockwise seen from above (0 degrees faces x+ and rotates using right hand rule around the z+ up axis)
    #####################################################
    def setpoint_position_local_body(self,setpoint_position):
      """Commands the robot to a target position in the local body frame.

      Converts the body-relative input position and yaw to the ENU frame,
      sends a position setpoint, then blocks until the position and yaw goals
      are reached within error bounds or the command times out.

      Body frame conventions (FLU): x+ forward, y+ left, z+ up, yaw+
      counter-clockwise seen from above. The driver receives the ENU offset
      from the current position (x east, y north, z up) and the absolute ENU
      target attitude. Goals: x/y/z are the body FLU offsets, yaw is the
      absolute ENU yaw goal. Errors are goal minus current in the same frames
      (x/y/z rotated back into the start body frame), yaw wrapped to
      (-180, 180].

      Args:
          setpoint_position (list): Target position and yaw as
              [X_BODY_M, Y_BODY_M, Z_BODY_M, YAW_BODY_DEG]. 0 on an axis means
              no move on that axis. Use -999 for any element to hold the
              current value; a held element is not commanded as a number and
              not counted as an error.

      Returns:
          bool: True if both position and yaw setpoints were reached within
              bounds and timeout, False otherwise.
      """
      # setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YAW_BODY_DEGREES]
      # use value 0 for no change, -999 to hold the axis and skip its error check
      cmd_success = True
      timeout_sec = self.status_msg.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.update_current_goals( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info("************************" )
      self.msg_if.pub_info("Starting Setpoint Position Local Process" )
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_geopoint_wgs84 = list(self.current_location_wgs84_geo)
      #self.msg_if.pub_info("Start Location WSG84 geopoint" )
      #self.msg_if.pub_info(" Lat, Long, Alt" )
      #self.msg_if.pub_info(["%.2f" % start_geopoint_wgs84[0],"%.2f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
      start_position_enu_m = list(self.current_position_enu_m)
      self.msg_if.pub_info("Start Position ENU degs" )
      self.msg_if.pub_info(" X, Y, Z" )
      self.msg_if.pub_info(["%.2f" % start_position_enu_m[0],"%.2f" % start_position_enu_m[1],"%.2f" % start_position_enu_m[2]])   
      start_orientation_enu_degs=list(self.current_orientation_enu_degs)
      self.msg_if.pub_info("Start Orientation ENU degs" )
      self.msg_if.pub_info(" Roll, Pitch, Yaw" )
      self.msg_if.pub_info(["%.2f" % start_orientation_enu_degs[0],"%.2f" % start_orientation_enu_degs[1],"%.2f" % start_orientation_enu_degs[2]])

      start_yaw_enu_deg = start_orientation_enu_degs[2]
      self.msg_if.pub_info("Start Yaw ENU degs" )
      self.msg_if.pub_info(start_yaw_enu_deg) 
      start_heading_deg=self.current_heading_deg
      #self.msg_if.pub_info("Start Heading degs" )
      #self.msg_if.pub_info(start_heading_deg)   
      ##############################################
      # Condition Body Input Data
      ##############################################
      # Condition Point Input
      input_point_body_m=list(setpoint_position[0:3])
      self.msg_if.pub_info("Input Postion Body  X, Y, Z in Meters" )
      self.msg_if.pub_info(["%.2f" % input_point_body_m[0],"%.2f" % input_point_body_m[1],"%.2f" % input_point_body_m[2]])
      # A -999 (held) axis is a zero offset, so -999 is never sent as a -999 m move
      new_point_body_m=list(input_point_body_m)
      for ind in range(3):
        if input_point_body_m[ind] == -999:
          new_point_body_m[ind] = 0
      # Condition Orienation Input
      input_yaw_body_deg = setpoint_position[3]
      self.msg_if.pub_info("Yaw Input Body Degrees" )
      self.msg_if.pub_info(["%.2f" % input_yaw_body_deg])   
      ##############################################
      # Convert Body Data to ENU Data
      ##############################################
      # Set new yaw orientation in ENU degrees
      offset_enu_m = nepi_nav.convert_point_body2enu(new_point_body_m,start_yaw_enu_deg)
      self.msg_if.pub_info("Point Goal Offsets ENU Meters" )
      self.msg_if.pub_info(" X, Y, Z" )
      self.msg_if.pub_info(["%.2f" % offset_enu_m[0],"%.2f" % offset_enu_m[1],"%.2f" % offset_enu_m[2]])
      new_x_enu_m = start_position_enu_m[0] + offset_enu_m[0]
      new_y_enu_m = start_position_enu_m[1] + offset_enu_m[1]
      new_z_enu_m = start_position_enu_m[2] + offset_enu_m[2]
      new_position_enu_m = [new_x_enu_m,new_y_enu_m,new_z_enu_m]
      self.msg_if.pub_info("Point Goal ENU X, Y, Z in Meters" )
      self.msg_if.pub_info(["%.2f" % new_position_enu_m[0],"%.2f" % new_position_enu_m[1],"%.2f" % new_position_enu_m[2]])

      # A -999 (held) yaw keeps the start yaw rather than adding -999 degrees
      new_yaw_enu_deg = start_yaw_enu_deg
      if input_yaw_body_deg != -999:
        new_yaw_enu_deg = start_yaw_enu_deg + input_yaw_body_deg
      # Condition to (-180,180] deg
      new_yaw_enu_deg = self._wrapAngleDeg(new_yaw_enu_deg)
      self.msg_if.pub_info("Yaw Goal ENU Degrees" )
      self.msg_if.pub_info(["%.2f" % new_yaw_enu_deg])
      ##############################################
      # Create Point and Pose Data
      ##############################################
      # New Point ENU in meters
      new_point_enu_m=Point()
      new_point_enu_m.x = offset_enu_m[0]
      new_point_enu_m.y = offset_enu_m[1]
      new_point_enu_m.z = offset_enu_m[2]
      self.msg_if.pub_info("Position Goal ENU X, Y, Z in Meters" )
      self.msg_if.pub_info(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])

      new_orientation_enu_deg = [start_orientation_enu_degs[0],start_orientation_enu_degs[1],new_yaw_enu_deg]
      self.msg_if.pub_info("Orienation Goal ENU  Roll, Pitch, Yaw in Degrees" )
      self.msg_if.pub_info(["%.2f" % new_orientation_enu_deg[0],"%.2f" % new_orientation_enu_deg[1],"%.2f" % new_orientation_enu_deg[2]])
      ##############################################
      ## Send Message and Check for Setpoint Success
      ##############################################
      # Update current goals with body FLU offset goal in meters and ENU yaw goal in degrees
      goal_point_body_m = list(new_point_body_m)
      goal_yaw_enu_deg = new_yaw_enu_deg
      self.update_current_goals( [goal_point_body_m[0],goal_point_body_m[1],goal_point_body_m[2],None,None,None,goal_yaw_enu_deg] )
      self.msg_if.pub_info("Sending Setpoint Position Local Command" )
      if self.process_data_dict['has_goto_position'] == True:
        self.process_data_dict['go_position_success'] = None 
        self.process_update_dict['go_position'] = [new_point_enu_m,new_orientation_enu_deg]
      elif self.gotoPositionFunction is not None:
        self.gotoPositionFunction(new_point_enu_m,new_orientation_enu_deg)
      self.msg_if.pub_info("Waiting for Position Setpoint to complete" )
      setpoint_position_local_point_reached = False
      setpoint_position_local_yaw_reached = False
      stabilize_timer=0
      point_errors = [] # Initialize running list of errors
      yaw_errors = [] # Initialize running list of errors
      point_body_errors_m = [0.0,0.0,0.0]
      yaw_enu_error_deg = 0.0
      timeout_timer = 0 # Initialize timeout timer
      time2sleep = 0.1
      while (setpoint_position_local_point_reached is False or setpoint_position_local_yaw_reached is False) and not nepi_sdk.is_shutdown():  # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
            self.msg_if.pub_info("Setpoint Position received Stop Command" )
            new_position_enu_m = copy.deepcopy(self.current_position_enu_m)
            goal_offset_enu_m = np.array(new_position_enu_m) - np.array(start_position_enu_m)
            goal_point_body_m = nepi_nav.convert_point_enu2body(goal_offset_enu_m,start_yaw_enu_deg)
            for ind in range(3): # Zero offset goal if set to current
              if input_point_body_m[ind] == -999:
                goal_point_body_m[ind] = 0
            self.update_current_goals( [goal_point_body_m[0],goal_point_body_m[1],goal_point_body_m[2],None,None,None,None] )
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment message timer
        timeout_timer = timeout_timer+time2sleep
        # Calculate setpoint position errors: goal minus current, ENU then
        # rotated into the start body FLU frame the goals are expressed in
        point_enu_errors_m = np.array(new_position_enu_m) - np.array(self.current_position_enu_m)
        point_body_errors_m = list(nepi_nav.convert_point_enu2body(point_enu_errors_m,start_yaw_enu_deg))
        for ind in range(3):
          if input_point_body_m[ind] == -999: # Ignore error check if set to current
            point_body_errors_m[ind] = 0
        max_point_body_errors_m = np.max(np.abs(point_body_errors_m))
        # Calculate setpoint yaw enu error: goal minus current, wrapped
        if input_yaw_body_deg == -999: # Ignore error check if set to current
          setpoint_position_local_yaw_reached = True
          yaw_enu_error_deg = 0
          max_yaw_enu_error_deg = 0
        else:
          cur_yaw_enu_deg = self.current_orientation_enu_degs[2]
          yaw_enu_error_deg =  self._wrapAngleDeg(new_yaw_enu_deg - cur_yaw_enu_deg)
          max_yaw_enu_error_deg = abs(yaw_enu_error_deg)
        # Check for setpoint position local point goal
        if  setpoint_position_local_point_reached is False:
          if stabilize_timer > self.status_msg.error_bounds.min_stabilize_time_s:
            max_point_errors = max(point_errors) # Get max from error window
            point_errors = [max_point_body_errors_m] # reset running list of errors
            if max_point_errors < self.status_msg.error_bounds.max_distance_error_m:
              self.msg_if.pub_info("Position Setpoint Reached" )
              setpoint_position_local_point_reached = True
          else:
            point_errors.append(max_point_body_errors_m) # append last
        # Check for setpoint position yaw point goal
        if  setpoint_position_local_yaw_reached is False:
          if stabilize_timer > self.status_msg.error_bounds.min_stabilize_time_s:
            max_yaw_errors = max(yaw_errors) # Get max from error window
            yaw_errors = [max_yaw_enu_error_deg] # reset running list of errors
            if max_yaw_errors < self.status_msg.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Yaw Setpoint Reached" )
              setpoint_position_local_yaw_reached = True
          else:
            yaw_errors.append(max_yaw_enu_error_deg) # append last
        # Reset timer if past
        if stabilize_timer > self.status_msg.error_bounds.min_stabilize_time_s:
          stabilize_timer=0 # Reset timer
        self.update_current_errors(  [point_body_errors_m[0],point_body_errors_m[1],point_body_errors_m[2],None,None,None,yaw_enu_error_deg] )
      if cmd_success:
        self.msg_if.pub_info("Setpoint Reached" )
        self.msg_if.pub_info("************************" )
      self.update_current_errors(  [point_body_errors_m[0],point_body_errors_m[1],point_body_errors_m[2],None,None,None,yaw_enu_error_deg] )
      return cmd_success



    ### Function to set and check setpoint location global geopoint and yaw command
    ###################################################
    # Input is [LAT, LONG, ALT_WGS84, YAW_ENU_DEGREES]
    # Converted to AMSL Altitude before sending; yaw is sent as ENU, unconverted
    # Altitude is specified as meters above the WGS-84 ellipsoid and converted to AMSL
    # (WGS84 - the navpose geoid_height_meters, the geoid's height above the
    # ellipsoid) before sending
    # Yaw is specified as ENU yaw degrees: 0 faces east, positive counter-clockwise
    #####################################################
    def setpoint_location_global_wgs84(self,setpoint_location):
      """Commands the robot to a global WGS84 geographic position.

      Converts WGS84 altitude to AMSL, sends the location setpoint with an ENU
      attitude to gotoLocationFunction, then blocks until the geopoint and yaw
      goals are reached within error bounds or the command times out.

      Goals are the ENU offsets from the start location in meters (x east,
      y north, z up) and the ENU yaw goal. Errors are goal minus current in
      the same frames (x east, y north, z up, yaw wrapped to (-180, 180]).

      Args:
          setpoint_location (list): Target location as
              [LAT, LONG, ALT_WGS84_M, YAW_ENU_DEG]. Latitude and longitude
              in decimal degrees, altitude in meters above the WGS84
              ellipsoid, yaw in ENU degrees (0 faces east, positive
              counter-clockwise). Use -999 for any element to hold the current
              value; a held element is not commanded as a number and not
              counted as an error.

      Returns:
          bool: True if both location and yaw setpoints were reached within
              bounds and timeout, False otherwise.
      """
      # setpoint_location is [LAT, LONG, ALT_WGS84, YAW_ENU_DEGREES +-180]
      # Use value -999 to use current value
      cmd_success = True
      timeout_sec = self.status_msg.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.update_current_goals( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info("************************" )
      self.msg_if.pub_info("Starting Setpoint Location Global Process" )
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_geopoint_wgs84 = list(self.current_location_wgs84_geo)  
      start_orientation_enu_degs=list(self.current_orientation_enu_degs)
      start_yaw_enu_deg = start_orientation_enu_degs[2]
      self.msg_if.pub_info("Start Yaw ENU degs" )
      self.msg_if.pub_info(start_yaw_enu_deg) 
      start_heading_deg=self.current_heading_deg
      self.msg_if.pub_info("Start Heading degs" )
      self.msg_if.pub_info(start_heading_deg)
      start_geoid_height_m = self.current_geoid_height_m
      # Meters per degree. Longitude degrees shrink with cos(latitude).
      m_per_deg_lat = 111139
      m_per_deg_long = m_per_deg_lat * np.cos(np.radians(start_geopoint_wgs84[0]))
      ##############################################
      # Condition Input Data
      ##############################################
      # Condition Location Input
      input_geopoint_wgs84 = list(setpoint_location[0:3])
      new_geopoint_wgs84=list(start_geopoint_wgs84) # Initialize with start
      for ind in range(3): # Overwrite current with new if set and valid
        if input_geopoint_wgs84[ind] != -999:
          new_geopoint_wgs84[ind]=input_geopoint_wgs84[ind]
      # Condition Yaw Input
      input_yaw_enu_deg = setpoint_location[3]
      new_yaw_enu_deg = start_yaw_enu_deg # Initialize to current
      if input_yaw_enu_deg != -999: # Replace if not -999
        new_yaw_enu_deg = input_yaw_enu_deg
      # Condition to (-180,180] degs
      new_yaw_enu_deg = self._wrapAngleDeg(new_yaw_enu_deg)
      ##############################################
      # Create Global AMSL Location and ENU Orienation Setpoint Values
      ##############################################
      # New Global location
      new_geopoint_amsl=GeoPoint()
      new_geopoint_amsl.latitude = new_geopoint_wgs84[0]
      new_geopoint_amsl.longitude = new_geopoint_wgs84[1]
      # AMSL = ellipsoid altitude - geoid height (N is the geoid above the
      # ellipsoid). A held (-999) altitude is the start ellipsoid altitude, so
      # it converts to the start AMSL altitude, not a jump of N meters.
      new_geopoint_amsl.altitude = new_geopoint_wgs84[2] - start_geoid_height_m
      self.msg_if.pub_info("Location Goal Lat, Long, Alt_AMSL" )
      self.msg_if.pub_info(["%.8f" % new_geopoint_amsl.latitude,"%.8f" % new_geopoint_amsl.longitude,"%.2f" % new_geopoint_amsl.altitude])
      # New Local Orienation ENU in degs  
      new_orientation_enu_deg = [start_orientation_enu_degs[0],start_orientation_enu_degs[1],new_yaw_enu_deg]
      self.msg_if.pub_info("Orienation Goal ENU  Roll, Pitch, Yaw in Degrees" )
      self.msg_if.pub_info(["%.2f" % new_orientation_enu_deg[0],"%.2f" % new_orientation_enu_deg[1],"%.2f" % new_orientation_enu_deg[2]])
      ##############################################
      ## Send Message and Check for Setpoint Success
      ##############################################
      # Update current goals with start to goal ENU offset in meters (x east,
      # y north, z up) and ENU yaw goal in degrees
      self.update_current_goals( [(new_geopoint_wgs84[1] - start_geopoint_wgs84[1])*m_per_deg_long,(new_geopoint_wgs84[0] - start_geopoint_wgs84[0])*m_per_deg_lat,new_geopoint_wgs84[2] - start_geopoint_wgs84[2],None,None,None,new_yaw_enu_deg] )
      self.msg_if.pub_info("Sending MAVLINK Setpoint Position Local Command" )
      if self.process_data_dict['has_goto_location'] == True:
        self.process_data_dict['go_location_success'] = None 
        self.process_update_dict['go_location'] = [new_geopoint_amsl,new_orientation_enu_deg]
      elif self.gotoLocationFunction is not None:
        self.gotoLocationFunction(new_geopoint_amsl,new_orientation_enu_deg)
      self.msg_if.pub_info(" checking for Setpoint Reached" )
      setpoint_location_global_geopoint_reached = False
      setpoint_location_global_yaw_reached = False 
      self.msg_if.pub_info("Waiting for Position Local Setpoint to complete" )
      stabilize_timer=0
      geopoint_errors = [] # Initialize running list of errors
      yaw_errors = [] # Initialize running list of errors
      geopoint_errors_m = [0.0,0.0,0.0]
      yaw_enu_error_deg = 0.0
      timeout_timer = 0 # Initialize timeout timer
      time2sleep = 0.1
      while (setpoint_location_global_geopoint_reached is False or setpoint_location_global_yaw_reached is False) and not nepi_sdk.is_shutdown(): # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
          self.msg_if.pub_info("Setpoint Location received Stop Command" )
          new_geopoint_wgs84 = copy.deepcopy(self.current_location_wgs84_geo)
          self.update_current_goals( [(new_geopoint_wgs84[1] - start_geopoint_wgs84[1])*m_per_deg_long,(new_geopoint_wgs84[0] - start_geopoint_wgs84[0])*m_per_deg_lat,new_geopoint_wgs84[2] - start_geopoint_wgs84[2],None,None,None,None] )
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment self.update_error_msg message timer
        timeout_timer = timeout_timer+time2sleep        
        # Calculate setpoint position and yaw errors: goal minus current, ENU order
        geopoint_errors_geo = np.array(new_geopoint_wgs84) - np.array(self.current_location_wgs84_geo)
        geopoint_errors_m = [geopoint_errors_geo[1]*m_per_deg_long,geopoint_errors_geo[0]*m_per_deg_lat,geopoint_errors_geo[2]]
        # Ignore error check if set to current. Input order is lat, long, alt;
        # error order is x east (long), y north (lat), z up (alt)
        if input_geopoint_wgs84[0] == -999.0:
          geopoint_errors_m[1] = 0
        if input_geopoint_wgs84[1] == -999.0:
          geopoint_errors_m[0] = 0
        if input_geopoint_wgs84[2] == -999.0:
          geopoint_errors_m[2] = 0
        max_geopoint_error_m = np.max(np.abs(geopoint_errors_m))
        if input_yaw_enu_deg == -999: # Ignore error check if set to current
          setpoint_location_global_yaw_reached = True
          yaw_enu_error_deg = 0
          max_yaw_enu_error_deg = 0
        else:
          cur_yaw_enu_deg = self.current_orientation_enu_degs[2]
          yaw_enu_error_deg =  self._wrapAngleDeg(new_yaw_enu_deg - cur_yaw_enu_deg)
          max_yaw_enu_error_deg = abs(yaw_enu_error_deg)
        # Check for setpoint position global goal
        if  setpoint_location_global_geopoint_reached is False:
          if stabilize_timer > self.status_msg.error_bounds.min_stabilize_time_s:
            max_geopoint_errors = max(geopoint_errors) # Get max from error window
            geopoint_errors = [max_geopoint_error_m] # reset running list of errors
            if max_geopoint_errors < self.status_msg.error_bounds.max_distance_error_m:
              self.msg_if.pub_info("Location Setpoint Reached" )
              setpoint_location_global_geopoint_reached = True
          else:
            geopoint_errors.append(max_geopoint_error_m) # append last
        # Check for setpoint position yaw goal
        if  setpoint_location_global_yaw_reached is False:
          if stabilize_timer > self.status_msg.error_bounds.min_stabilize_time_s:
            max_yaw_errors = max(yaw_errors) # Get max from error window
            yaw_errors = [max_yaw_enu_error_deg] # reset running list of errors
            if max_yaw_errors < self.status_msg.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Yaw Setpoint Reached" )
              setpoint_location_global_yaw_reached = True
          else:
            yaw_errors.append(max_yaw_enu_error_deg) # append last
        # Reset timer if past. Same bound as the checks above: a fixed 1 s
        # reset never let the timer pass a min_stabilize_time_s above 1 s, so
        # such a goto could never complete.
        if stabilize_timer > self.status_msg.error_bounds.min_stabilize_time_s:
          stabilize_timer=0 # Reset timer
        self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],None,None,None,yaw_enu_error_deg] )
      if cmd_success:
        self.msg_if.pub_info("Setpoint Reached" )
        self.msg_if.pub_info("************************" )
      self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],None,None,None,yaw_enu_error_deg] )
      return cmd_success

    def _wrapAngleDeg(self, angle_deg):
      # Wrap an angle in degrees to (-180, 180]
      wrapped = float(angle_deg) % 360.0
      if wrapped > 180.0:
        wrapped = wrapped - 360.0
      return wrapped

    #######################
    # Class Utility Functions
    
    ### Function for updating current goto error values
    def update_current_errors(self,error_list):
      """Updates the current goto error values in the status message.

      Expects exactly seven elements corresponding to x, y, z translation
      errors in meters and heading, roll, pitch, yaw rotation errors in
      degrees. Each element that is not None is written into the persistent
      errors message. A None element leaves that axis at its last value, so a
      command that only measures some axes can update those alone. The
      errors message is then assigned to the status message. Does nothing if
      the list length is not seven.

      Each error is goal minus current, in the same frame and units as the
      goal it pairs with (see update_current_goals); angles are wrapped to
      (-180, 180]. A held (-999) axis reports 0.

      Args:
          error_list (list): Error values in the order
              [x_m, y_m, z_m, heading_deg, roll_deg, pitch_deg, yaw_deg].
              Use None for any element to leave that axis unchanged.
      """
      if len(error_list) == 7:
        if error_list[0] is not None:
          self.errors_msg.x_m = error_list[0]
        if error_list[1] is not None:
          self.errors_msg.y_m = error_list[1]
        if error_list[2] is not None:
          self.errors_msg.z_m = error_list[2]
        if error_list[3] is not None:
          self.errors_msg.heading_deg = error_list[3]
        if error_list[4] is not None:
          self.errors_msg.roll_deg = error_list[4]
        if error_list[5] is not None:
          self.errors_msg.pitch_deg = error_list[5]
        if error_list[6] is not None:
          self.errors_msg.yaw_deg = error_list[6]

        self.status_msg.errors_current = self.errors_msg
      else:
        self.msg_if.pub_info("Skipping current error update. Error list to short" )

    ### Function for updating current goto goal values
    def update_current_goals(self,goal_list):
      """Updates the current goto goal values in the status message.

      Expects exactly seven elements corresponding to x, y, z translation
      goals in meters and heading, roll, pitch, yaw rotation goals in
      degrees. Each element that is not None is written into the persistent
      goals message. A None element leaves that axis at its last value, so a
      command that only targets some axes can update those alone. The goals
      message is then assigned to the status message. Does nothing if the
      list length is not seven.

      Frames: x_m/y_m/z_m are body FLU offsets (x forward, y left, z up) for
      goto_position, and ENU offsets from the start location (x east,
      y north, z up) for goto_location and takeoff. heading_deg is a compass
      heading. roll_deg/pitch_deg/yaw_deg are ENU in the NavPose convention
      (yaw 0 faces east, positive counter-clockwise).

      Args:
          goal_list (list): Goal values in the order
              [x_m, y_m, z_m, heading_deg, roll_deg, pitch_deg, yaw_deg].
              Use None for any element to leave that axis unchanged.
      """
      if len(goal_list) == 7:
        if goal_list[0] is not None:
          self.goals_msg.x_m = goal_list[0]
        if goal_list[1] is not None:
          self.goals_msg.y_m = goal_list[1]
        if goal_list[2] is not None:
          self.goals_msg.z_m = goal_list[2]
        if goal_list[3] is not None:
          self.goals_msg.heading_deg = goal_list[3]
        if goal_list[4] is not None:
          self.goals_msg.roll_deg = goal_list[4]
        if goal_list[5] is not None:
          self.goals_msg.pitch_deg = goal_list[5]
        if goal_list[6] is not None:
          self.goals_msg.yaw_deg = goal_list[6]

        self.status_msg.goals_current = self.goals_msg
      else:
        self.msg_if.pub_info("Skipping current goal update. Goal list to short" )

    ### Function for updating last goto error values
    def update_prev_errors(self):
        """Copies the current goto error values into the previous error slot.

        Called before a new command is issued so the last command's errors
        are preserved in the status message for comparison.
        """
        errors_msg = GotoErrors()
        errors_msg.x_m = self.status_msg.errors_current.x_m
        errors_msg.y_m = self.status_msg.errors_current.y_m
        errors_msg.z_m = self.status_msg.errors_current.z_m
        errors_msg.heading_deg = self.status_msg.errors_current.heading_deg
        errors_msg.roll_deg = self.status_msg.errors_current.roll_deg
        errors_msg.pitch_deg = self.status_msg.errors_current.pitch_deg
        errors_msg.yaw_deg = self.status_msg.errors_current.yaw_deg
        self.status_msg.errors_prev = errors_msg


    def update_error_msg(self,error_msg):
      """Logs an error and stores it in the status message.

      Args:
          error_msg (str): The error description to log and persist.
      """
      self.msg_if.pub_info(error_msg)
      self.status_msg.last_error_message = error_msg

    # Best-effort, driver-agnostic reason a manual/teleop command was rejected.
    # A flight-capable driver reports ARM/DISARM through the same
    # states/getStateIndFunction this class already holds, so when the robot
    # has an "ARM" state and is not in it, say so. Robots without an "ARM" state
    # (rovers have none) keep the plain "<controls_name> not Ready" wording.
    def _notReadyReason(self, controls_name):
        reason = controls_name + " not Ready"
        if self.caps_dict['states'] and "ARM" in self.caps_dict['states'] and self.getStateIndFunction is not None:
            try:
                state_ind = self.getStateIndFunction()
                if 0 <= state_ind < len(self.caps_dict['states']) and self.caps_dict['states'][state_ind] != "ARM":
                    reason = reason + " (vehicle is not armed -- arm it first)"
            except Exception:
                pass
        return reason

    def get_motor_controls_status_msg(self,motor_controls):
      """Builds a list of MotorControl messages from a speed-ratio list.

      Args:
          motor_controls (list): Ordered list of speed ratios, one per motor.

      Returns:
          list: A list of MotorControl messages with motor_ind and speed_ratio
              populated for each motor.
      """
      mcs = []
    #   for i in range(len(motor_controls)):
    #     mc = MotorControl()
    #     mc.motor_ind = i
    #     mc.speed_ratio = motor_controls[i]
    #     mcs.append(mc)
      return mcs


    ####################################
    ### Callback for rbx status publisher



    def get_capabilities_msg(self):
        if self.process_if is None:
           self.process_data_dict = copy.deepcopy(self.BLANK_PROCESS_DATA_DICT)
           self.process_ready = False
        else:
           self.process_ready = self.process_if.get_process_ready()
           if self.process_ready == False:
            self.process_data_dict = copy.deepcopy(self.BLANK_PROCESS_DATA_DICT)

        process_dict = copy.deepcopy(self.process_data_dict)

        caps_msg = DeviceRBXStatusQuery()
        caps_msg.has_x_control = self.caps_dict['has_x_control'] or process_dict.get('has_x_control',False)
        caps_msg.has_y_control = self.caps_dict['has_y_control'] or process_dict.get('has_y_control',False)
        caps_msg.has_z_control = self.caps_dict['has_z_control'] or process_dict.get('has_z_control',False)

        caps_msg.has_roll_control = self.caps_dict['has_roll_control'] or process_dict.get('has_roll_control',False)
        caps_msg.has_pitch_control = self.caps_dict['has_pitch_control'] or process_dict.get('has_pitch_control',False)
        caps_msg.has_yaw_control = self.caps_dict['has_yaw_control'] or process_dict.get('has_yaw_control',False)

        caps_msg.has_manual_controls = self.caps_dict['has_manual_controls'] 

        caps_msg.has_set_home = self.caps_dict['has_set_home'] or process_dict.get('has_set_home',False)
        caps_msg.has_home = self.caps_dict['has_go_home'] or process_dict.get('has_go_home',False)
        caps_msg.has_stop = self.caps_dict['has_go_stop'] or process_dict.get('has_go_stop',False)
        caps_msg.has_pose = self.caps_dict['has_goto_pose'] or process_dict.get('has_goto_pose',False)
        caps_msg.has_position = self.caps_dict['has_goto_position'] or process_dict.get('has_goto_position',False)
        caps_msg.has_location = self.caps_dict['has_goto_location'] or process_dict.get('has_goto_location',False)
        caps_msg.has_velocity = self.caps_dict['has_go_velocity'] or process_dict.get('has_go_velocity',False)

        process_has_auto = process_dict.get('has_set_home',False) or \
                        process_dict.get('has_go_home',False) or \
                        process_dict.get('has_go_stop',False) or \
                        process_dict.get('has_goto_pose',False) or \
                        process_dict.get('has_goto_position',False) or \
                        process_dict.get('has_goto_location',False) or \
                        process_dict.get('has_go_velocity',False)

        caps_msg.has_autonomous_controls = self.caps_dict['has_autonomous_controls'] or process_has_auto


        caps_msg.state_options =  list(dict.fromkeys(self.caps_dict['modes'] + process_dict['mode_options']))
        caps_msg.state_options =  list(dict.fromkeys(self.caps_dict['states'] + process_dict['state_options']))
        caps_msg.setup_action_options = list(dict.fromkeys(self.caps_dict['setup_actions'] + process_dict['setup_action_options']))
        caps_msg.go_action_options = list(dict.fromkeys(self.caps_dict['go_actions'] + process_dict['go_action_options']))

        return caps_msg

    ####################################
    ### Callback for rbx proccess

    def processCb(self, timer):
        #self.msg_if.pub_warn("Starting Process Thread", throttle_s = 10)
        start_time = nepi_utils.get_time()

        if self.process_if is None:
           nepi_sdk.start_timer_process(1, self.processCb, oneshot = True)
           return
        #####################
        # Setup Process Input Data
        #####################
        
        # RBX DATA
        self.process_data_dict['rbx_status_dict'] =  self.get_status_dict()
        self.process_data_dict['rbx_navpose_dict'] =  self.get_navpose_dict()


        ########
        # Image DATA
        cv2_img = None
        cv2_navpose_dict = None
        cv2_status = None
        try:
            if self.connect_img_if.check_connection() == True:
                cv2_img = self.connect_img_if.get_data_dict()['color_image']
                cv2_status = self.connect_img_if.get_status_dict()
                cv2_navpose_dict = self.connect_img_if.get_navpose_dict()
        except:
            pass

        self.process_data_dict['cv2_img'] = cv2_img
        self.process_data_dict['cv2_status_dict'] = cv2_status
        self.process_data_dict['cv2_navpose_dict'] = cv2_navpose_dict



        ########
        # Servo DATA
        servo_names = []
        servo_status_dict = dict()
        servo_if_dict = dict()
        for servo_name in self.servo_if_dict.keys():
            servo_if = self.servo_if_dict[servo_name]
            try:

                servo_status_dict[servo_name] = servo_if.get_status_dict()
                servo_names.append(servo_name)
                servo_if_dict[servo_name] = servo_if
            except:
                pass
        self.process_if.set_process_if_dict('servo_if_dict',servo_if_dict)
        self.process_data_dict['servo_names'] = servo_names
        self.process_data_dict['servo_status_dict'] = servo_status_dict

        #################
        ## Apply and Clear Updates
       
        for key in self.process_update_dict.keys():
           self.process_data_dict[key] = self.process_update_dict[key]
        self.process_update_dict = copy.deepcopy(self.BLANK_PROCESS_UPDATE_DICT)


        #####################
        # Run Process
        #####################

        source_image_topic = self.status_msg.connect_image_topic
        auto_results_dict = self.process_if.process_results(source_topic = source_image_topic)
        # self.msg_if.pub_warn("Got auto_results_dict: " + str(auto_results_dict), throttle_s = 10)    
        self.process_data_dict = self.process_if.get_data_dict()

  
        #####################
        # Apply Process Results
        #####################



        
        #############################
        # Publish Status and Set Next Process
        self.publish_status()
        max_hz = self.process_if.get_process_rate()
        if max_hz < 1:
            max_hz = 1
        process_delay = (float(1) / max_hz) - (nepi_utils.get_time() - start_time)
        next_process_delay = max(0.01, process_delay)
        # Re-arm this loop, not updaterCb. Re-arming the updater here ran the
        # process body exactly once and then drove the 1 Hz updater at the process
        # rate instead.
        nepi_sdk.start_timer_process(next_process_delay, self.processCb, oneshot = True)


    def blankImagePubCb(self, timer):
        img_connected = False
        try:
           img_connected = self.connect_img_if.check_connection()
        except:
           pass
        if img_connected == False:
            try:
                self.image_if.publish_cv2_img(self.image_blank)
            except:
                pass      


    def imageCb(self,data_dict):
        cv2_img = data_dict['color_image']
        self.image_if.publish_cv2_img(cv2_img)
