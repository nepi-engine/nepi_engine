#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import os
import time
import threading
import subprocess
import numpy as np
import math
import tf
import random
import sys
import cv2
import copy

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img
from nepi_sdk import nepi_pc
from nepi_sdk import nepi_nav



from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from pygeodesy.ellipsoidalKarney import LatLon

from nepi_ros_interfaces.msg import RBXInfo, RBXStatus, AxisControls, RBXErrorBounds, RBXGotoErrors, RBXMotorControl, \
     RBXGotoPose, RBXGotoPosition, RBXGotoLocation, RBXMotorControl
from nepi_ros_interfaces.srv import NavPoseQuery, NavPoseQueryRequest, NavPoseQueryResponse
from nepi_ros_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse, RBXCapabilitiesQueryRequest


from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF

from nepi_api.data_if import ImageIF
from nepi_api.device_if_npx import NPXDeviceIF





EXAMPLE_NAVPOSE_DATA_DICT = {
                          'frame_3d': 'ENU',
                          'frame_alt': 'WGS84',

                          'geoid_height_meters': 0,

                          'has_heading': True,
                          'time_heading': nepi_utils.get_time(),
                          'heading_deg': 120.50,

                          'has_oreientation': True,
                          'time_oreientation': nepi_utils.get_time(),
                          # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
                          'roll_deg': 30.51,
                          'pitch_deg': 30.51,
                          'yaw_deg': 30.51,

                          'has_position': True,
                          'time_position': nepi_utils.get_time(),
                          # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
                          'x_m': 1.234,
                          'y_m': 1.234,
                          'z_m': 1.234,

                          'has_location': True,
                          'time_location': nepi_utils.get_time(),
                          # Global Location in set altitude frame (lat,long,alt) with alt in meters
                          'lat': 47.080909,
                          'long': -120.8787889,

                          'has_altitude': True,
                          'time_altitude': nepi_utils.get_time(),
                          'alt_m': 12.321,
    
                          'has_depth': False,
                          'time_depth': nepi_utils.get_time(),
                          'alt_m': 0
}


class RBXRobotIF:
    # Default Global Values
    BAD_NAME_CHAR_LIST = [" ","/","'","-","$","#"]
    STATUS_UPDATE_RATE_HZ = 2
    UPDATE_NAVPOSE_RATE_HZ = 10
    CHECK_SAVE_DATA_RATE_HZ = 40
    
    # Factory Control Values 
    FACTORY_GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
    FACTORY_GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
    FACTORY_GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding
    FACTORY_CMD_TIMEOUT_SEC = 25 # Any action that changes 
    FACTORY_IMAGE_INPUT_TOPIC_NAME = "color_2d_image" # Partial or full ROS namespace string, "" for black image 
    FACTORY_HOME_LOCATION = [47.6540828,-122.3187578,0.0]


    # Define class variables
    ready = False

    factory_device_name = None
    init_device_name = None
    factory_controls = None
 

    states = []
    modes = []
    setup_actions = []
    go_actions = []
    data_products_list = ['image']

    settings_if = None
    save_data_if = None
    save_cfg_if = None
    
    rbx_status_pub_interval = float(1)/float(STATUS_UPDATE_RATE_HZ)
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

    rbx_image_sub = None
    rbx_state_last = None
    rbx_mode_last = None
    rbx_image_blank = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
    cv2_img = np.zeros((350, 700, 3), dtype = np.uint8) # Empty Black Image
    rbx_image_source_last = "None"
    init_image_status_overlay = False

    ### IF Initialization
    log_name = "RBXRobotIF"
    def __init__(self, device_info, capSettings, 
                 factorySettings, settingUpdateFunction, getSettingsFunction,
                 axisControls,getBatteryPercentFunction,
                 states,getStateIndFunction,setStateIndFunction,
                 modes,getModeIndFunction,setModeIndFunction,
                 checkStopFunction,
                 setup_actions, setSetupActionIndFunction,
                 go_actions, setGoActionIndFunction,
                 getHomeFunction=None,setHomeFunction=None,
                 manualControlsReadyFunction=None,
                 getMotorControlRatios=None,
                 setMotorControlRatio=None,
                 autonomousControlsReadyFunction=None,
                 goHomeFunction=None, goStopFunction=None, 
                 gotoPoseFunction=None, gotoPositionFunction=None, gotoLocationFunction=None,
                 getNavPoseDictFunction=None, 
                 has_heading = False, has_position = False, has_orientation = False, 
                 has_location = False, has_altitude = False, has_depth = False,
                 setFakeGPSFunction = None
                 ):

        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = os.path.join(self.base_namespace,self.node_name)

        ##############################  
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")


        ############################## 
        # Initialize Class Variables
        
        self.robot_name = device_info["robot_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.factory_device_name = device_info["robot_name"] + "_" + device_info["identifier"]

        self.states = states
        self.getStateIndFunction = getStateIndFunction
        self.setStateIndFunction = setStateIndFunction

        self.modes = modes
        self.getModeIndFunction = getModeIndFunction
        self.setModeIndFunction = setModeIndFunction
        
        self.checkStopFunction = checkStopFunction

        self.setup_actions = setup_actions
        self.setSetupActionIndFunction = setSetupActionIndFunction

        # Create and start initializing Status values
        self.rbx_status=RBXStatus()
        self.rbx_status.process_current = "None"
        self.rbx_status.process_last = "None"
        self.rbx_status.ready = False
        self.rbx_status.battery = 0
        errors_msg = RBXGotoErrors()
        errors_msg.x_m = 0
        errors_msg.y_m = 0
        errors_msg.z_m = 0
        errors_msg.heading_deg = 0
        errors_msg.roll_deg = 0
        errors_msg.pitch_deg = 0
        errors_msg.yaw_deg = 0
        self.rbx_status.errors_current = errors_msg
        self.rbx_status.errors_prev = errors_msg
        self.rbx_status.last_error_message = ""

        self.rbx_info=RBXInfo()
        self.rbx_info.connected = False
        self.rbx_info.serial_num = self.serial_num
        self.rbx_info.hw_version = self.hw_version
        self.rbx_info.sw_version = self.sw_version
        self.rbx_info.standby = False
        self.rbx_info.state = -999
        self.rbx_info.mode = -999
        self.rbx_info.home_lat = -999
        self.rbx_info.home_long = -999
        self.rbx_info.home_alt = -999
        self.rbx_info.state = self.getStateIndFunction()
        self.rbx_info.mode = self.getModeIndFunction()



        # Create and start initializing Capabilities values
        self.nav_pose_capabilities_report = NavPoseCapabilitiesQueryResponse()
        self.nav_pose_capabilities_report.has_gps = gpsTopic is not None
        self.nav_pose_capabilities_report.has_orientation = odomTopic is not None
        self.nav_pose_capabilities_report.has_heading = headingTopic is not None

        self.capabilities_report = RBXCapabilitiesQueryResponse()
        if axisControls == None:
          axis_controls = AxisControls()
          axis_controls.x = False
          axis_controls.y = False
          axis_controls.z = False
          axis_controls.roll = False
          axis_controls.pitch = False
          axis_controls.yaw = False
        self.capabilities_report.control_support = axisControls
        
        self.capabilities_report.state_options = states
        self.capabilities_report.mode_options = modes
        self.capabilities_report.setup_action_options = setup_actions
        self.capabilities_report.go_action_options = go_actions
        self.capabilities_report.data_products = self.data_products_list

        self.setFakeGPSFunction = setFakeGPSFunction
        if setFakeGPSFunction is not None:
          self.capabilities_report.has_fake_gps = True        

        else:
          self.capabilities_report.has_fake_gps = False

        self.getBatteryPercentFunction = getBatteryPercentFunction
        if self.getBatteryPercentFunction is not None:
          self.capabilities_report.has_battery_feedback = True
        else:
          self.capabilities_report.has_battery_feedback = False

        ## Setup Manual Controls
        self.manualControlsReadyFunction = manualControlsReadyFunction
        if self.manualControlsReadyFunction is not None:
          self.rbx_status.manual_control_mode_ready = self.manualControlsReadyFunction()
          manual_controls_ready = self.manualControlsReadyFunction()
          if manual_controls_ready:
            if self.setMotorControlRatio is not None:
              mc = RBXMotorControl()
              mc.speed_ratio = 0.0
              for i in range(len(self.getMotorControlRatios())):
                mc.motor_ind = i
                self.setMotorControlRatio(mc)
        else:
          self.rbx_status.manual_control_mode_ready = False

        if self.getMotorControlRatios is not None:
          motor_controls_status_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
          motor_controls_status_msg = self.get_motor_controls_status_msg([])
        self.rbx_status.current_motor_control_settings = motor_controls_status_msg

        self.setMotorControlRatio = setMotorControlRatio
        if self.setMotorControlRatio is None:
            self.capabilities_report.has_manual_controls = False
        else:
            self.capabilities_report.has_manual_controls = True
        self.getMotorControlRatios = getMotorControlRatios
        if self.getMotorControlRatios is not None:
            motor_controls_info_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
            motor_controls_info_msg = self.get_motor_controls_status_msg([])

        # Setup Autonomous Contros
        self.autonomousControlsReadyFunction = autonomousControlsReadyFunction
        if self.autonomousControlsReadyFunction is not None:
          self.rbx_status.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
          self.capabilities_report.has_autonomous_controls = True
        else:
          self.rbx_status.autonomous_control_mode_ready = False
          self.capabilities_report.has_autonomous_controls = False


        self.go_actions = go_actions
        self.setGoActionIndFunction = setGoActionIndFunction

  
        self.getHomeFunction = getHomeFunction
        self.setHomeFunction  = setHomeFunction
        if self.setHomeFunction is None :
            self.capabilities_report.has_set_home = False
        else:
            self.capabilities_report.has_set_home = True
     

        self.goHomeFunction = goHomeFunction
        if self.goHomeFunction is None:
            self.capabilities_report.has_go_home = False
        else:
            self.capabilities_report.has_go_home = True


        self.goStopFunction = goStopFunction
        if self.goStopFunction is None:
            self.capabilities_report.has_go_stop = False
        else:
            self.capabilities_report.has_go_stop = True


        self.gotoPoseFunction = gotoPoseFunction
        if self.gotoPoseFunction is None:
            self.capabilities_report.has_goto_pose = False
        else:
            self.capabilities_report.has_goto_pose = True            

        self.gotoPositionFunction = gotoPositionFunction
        if self.gotoPositionFunction is None:
            self.capabilities_report.has_goto_position = False
        else:
            self.capabilities_report.has_goto_position = True            

        self.gotoLocationFunction = gotoLocationFunction
        if self.gotoLocationFunction is None:
            self.capabilities_report.has_goto_location = False
        else:
            self.capabilities_report.has_goto_location = True

        self.rbx_status.cmd_success = False



        ##################################################
        ### Node Class Setup

        self.msg_if.pub_info("Starting Node IF Initialization")
        # Configs Config Dict ####################
        self.CFGS_DICT = {
                'init_callback': self.initCb,
                'reset_callback': self.resetCb,
                'factory_reset_callback': self.factoryResetCb,
                'init_configs': True,
                'namespace':  self.node_namespace
        }

        self.PARAMS_DICT = {
            'rbx/device_name': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_device_name
            },
            'rbx/cmd_timeout': {
                'namespace': self.node_namespace,
                'factory_val': self.FACTORY_CMD_TIMEOUT_SEC
            },
            'rbx/home_location': {
                'namespace': self.node_namespace,
                'factory_val': self.FACTORY_HOME_LOCATION
            },
            'rbx/fake_gps_enabled': {
                'namespace': self.node_namespace,
                'factory_val': False
            },
            'rbx/max_error_m': {
                'namespace': self.node_namespace,
                'factory_val': self.FACTORY_GOTO_MAX_ERROR_M
            },
            'rbx/max_error_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.FACTORY_GOTO_MAX_ERROR_DEG
            },
            'rbx/stabilized_sec': {
                'namespace': self.node_namespace,
                'factory_val': self.FACTORY_GOTO_STABILIZED_SEC
            },
            'rbx/image_source': {
                'namespace': self.node_namespace,
                'factory_val': self.FACTORY_IMAGE_INPUT_TOPIC_NAME
            },
            'rbx/image_status_overlay': {
                'namespace': self.node_namespace,
                'factory_val': False
            },

        }
        

        # Services Config Dict ####################

        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.node_namespace,
                'topic': 'rbx/capabilities_query',
                'svr': RBXCapabilitiesQuery,
                'req': RBXCapabilitiesQueryRequest(),
                'resp': RBXCapabilitiesQueryResponse(),
                'callback': self.capabilities_query_callback
            }
        }

        self.PUBS_DICT = {
            'rbx_info_pub': {
                'namespace': self.node_namespace,
                'topic': 'rbx/info',
                'msg': RBXInfo,
                'qsize': 1,
                'latch': True
            },
            'rbx_status_pub': {
                'namespace': self.node_namespace,
                'topic': 'rbx/status',
                'msg': RBXStatus,
                'qsize': 1,
                'latch': True
            },
            'rbx_status_str_pub': {
                'namespace': self.node_namespace,
                'topic': 'rbx/status_str',
                'msg': String,
                'qsize': 1,
                'latch': True
            },            
            'set_gps_pub': {
                'namespace': self.node_namespace,
                'topic': 'NEPI_SET_NAVPOSE_GPS_TOPIC',
                'msg': String,
                'qsize': 1,
                'latch': None
            },
            'set_orientation_pub': {
                'namespace': self.node_namespace,
                'topic': 'NEPI_SET_NAVPOSE_ORIENTATION_TOPIC',
                'msg': String,
                'qsize': 1,
                'latch': None
            },
            'set_heading_pub': {
                'namespace': self.node_namespace,
                'topic': 'NEPI_SET_NAVPOSE_HEADING_TOPIC',
                'msg': String,
                'qsize': 1,
                'latch': None
            },            
            'set_gps_timesync_pub': {
                'namespace': self.node_namespace,
                'topic': 'NEPI_ENABLE_NAVPOSE_GPS_CLOCK_SYNC_TOPIC',
                'msg': Bool,
                'qsize': 1,
                'latch': None
            }            
        }
     
        self.SUBS_DICT = {
            'set_goto_error_bounds': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_goto_error_bounds',
                'msg': RBXErrorBounds,
                'qsize': None,
                'callback': self.setErrorBoundsCb, 
                'callback_args': ()
            },

            'update_device_name': {
                'namespace': self.node_namespace,
                'topic': 'rbx/update_device_name',
                'msg': String,
                'qsize': 1,
                'callback': self.updateDeviceNameCb, 
                'callback_args': ()
            },

            'reset_device_name': {
                'namespace': self.node_namespace,
                'topic': 'rbx/reset_device_name',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetDeviceNameCb, 
                'callback_args': ()
            },

            'enable_fake_gps': {
                'namespace': self.node_namespace,
                'topic': 'rbx/enable_fake_gps',
                'msg': Bool,
                'qsize': None,
                'callback': self.fakeGPSEnableCb, 
                'callback_args': ()
            },

            'set_state': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_state',
                'msg': Int32,
                'qsize': None,
                'callback': self.setStateCb, 
                'callback_args': ()
            },

            'set_mode': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_mode',
                'msg': Int32,
                'qsize': None,
                'callback': self.setModeCb, 
                'callback_args': ()
            },

            'setup_action': {
                'namespace': self.node_namespace,
                'topic': 'rbx/setup_action',
                'msg': RBXMotorControl,
                'qsize': None,
                'callback': self.setupActionCb, 
                'callback_args': ()
            },

            'set_motor_control': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_motor_control',
                'msg': RBXMotorControl,
                'qsize': None,
                'callback': self.setMotorControlCb, 
                'callback_args': ()
            },

            'go_action': {
                'namespace': self.node_namespace,
                'topic': 'rbx/go_action',
                'msg': UInt32,
                'qsize': None,
                'callback': self.setCmdTimeoutCb, 
                'callback_args': ()
            },

            'set_goto_timeout': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_goto_timeout',
                'msg': UInt32,
                'qsize': None,
                'callback': self.goHomeCb, 
                'callback_args': ()
            },

            'go_home': {
                'namespace': self.node_namespace,
                'topic': 'rbx/go_home',
                'msg': Empty,
                'qsize': None,
                'callback': self.setHomeCb, 
                'callback_args': ()
            },

            'set_home': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_home',
                'msg': GeoPoint,
                'qsize': None,
                'callback': self.SUB_CALLBACK, 
                'callback_args': ()
            },

            'set_home_current': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_home_current',
                'msg': RBXGotoLocation,
                'qsize': None,
                'callback': self.setHomeCurrentCb, 
                'callback_args': ()
            },

            'goto_location': {
                'namespace': self.node_namespace,
                'topic': 'rbx/goto_location',
                'msg': RBXGotoLocation,
                'qsize': None,
                'callback': self.gotoLocationCb, 
                'callback_args': ()
            },
            'goto_position': {
                'namespace': self.node_namespace,
                'topic': 'rbx/goto_position',
                'msg': RBXGotoPosition,
                'qsize': None,
                'callback': self.gotoPositionCb, 
                'callback_args': ()
            },
            'goto_pose': {
                'namespace': self.node_namespace,
                'topic': 'rbx/goto_pose',
                'msg': RBXGotoPose,
                'qsize': None,
                'callback': self.gotoPoseCb, 
                'callback_args': ()
            },
            'go_stop': {
                'namespace': self.node_namespace,
                'topic': 'rbx/go_stop',
                'msg': Empty,
                'qsize': None,
                'callback': self.goStopCb, 
                'callback_args': ()
            },
            'set_image_topic': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_image_topic',
                'msg': String,
                'qsize': None,
                'callback': self.setImageTopicCb, 
                'callback_args': ()
            },
            'enable_image_overlay': {
                'namespace': self.node_namespace,
                'topic': 'rbx/enable_image_overlay',
                'msg': Bool,
                'qsize': None,
                'callback': self.enableImageOverlayCb, 
                'callback_args': ()
            },
            'publish_status': {
                'namespace': self.node_namespace,
                'topic': 'rbx/publish_status',
                'msg': Empty,
                'qsize': None,
                'callback': self.publishStatusCb, 
                'callback_args': ()
            },
            'publish_info': {
                'namespace': self.node_namespace,
                'topic': 'rbx/publish_info',
                'msg': Empty,
                'qsize': None,
                'callback': self.publishInfoCb, 
                'callback_args': ()
            },
            'set_process_name': {
                'namespace': self.node_namespace,
                'topic': 'rbx/set_process_name',
                'msg': String,
                'qsize': None,
                'callback': self.setProcessNameCb, 
                'callback_args': ()
            }
        }


        # Create Node Class ####################
        self.NODE_IF = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_class_name = True
        )

        ready = self.NODE_IF.wait_for_ready()


        # Setup Data IF Classes ####################
        self.msg_if.pub_info("Starting Image IF Initialization")
        image_if = ImageIF(namespace = self.node_namespace, topic = 'image')



        # Setup System IF Classes ####################
        self.msg_if.pub_info("Starting Settings IF Initialization")
        if capSettings is not None:
          self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction, 
                    'namespace': self.node_namespace
        }
        else:
          self.SETTINGS_DICT = {
                    'capSettings': nepi_settings.NONE_CAP_SETTINGS, 
                    'factorySettings': nepi_settings.NONE_SETTINGS,
                    'setSettingFunction': nepi_settings.UPDATE_NONE_SETTINGS_FUNCTION, 
                    'getSettingsFunction': nepi_settings.GET_NONE_SETTINGS_FUNCTION, 
                    'namespace': self.node_namespace
        }
        self.settings_if = SettingsIF(self.SETTINGS_DICT)


        # Setup Save Data IF Class ####################
        self.msg_if.pub_info("Starting Save Data IF Initialization")
        factory_data_rates = {}
        for d in self.data_products_list:
            factory_data_rates[d] = [1.0, 0.0, 100.0] # Default to 0Hz save rate, set last save = 0.0, max rate = 100.0Hz

        factory_filename_dict = {
            'prefix': "", 
            'add_timestamp': True, 
            'add_ms': True,
            'add_ns': False,
            'suffix': "",
            'add_node_name': True
            }

        self.save_data_if = SaveDataIF(data_product_names = self.data_products_list,
                                    factory_data_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict)


   
        # Create a NavPose Device IF
        self.getNavPoseDictFunction = getNavPoseDictFunction
        if getNavPoseDictFunction is not None:
            self.msg_if.pub_info("Starting NPX Device IF Initialization")
            navpose_if = NPXDeviceIF(device_info, 
                                    has_heading = has_heading,
                                    has_position = has_position,
                                    has_orientation = has_orientation,
                                    has_location = has_location,
                                    has_altitude = has_altitude,
                                    has_depth = has_depth,
                                    getNavPoseDictFunction = self.getNavPoseDictFunction,
                                    pub_rate = 10)

        time.sleep(1)



        ###############################
        # Finish Initialization
        # Start NavPose Data Updater
        NAVPOSE_SERVICE_NAME = nepi_ros.create_namespace(self.base_namespace,"nav_pose_query")
        self.msg_if.pub_info("Waiting for NEPI NavPose query service on: " + NAVPOSE_SERVICE_NAME)
        nepi_ros.wait_for_service(NAVPOSE_SERVICE_NAME)
        self.msg_if.pub_info("Connecting to NEPI NavPose query service at: " + NAVPOSE_SERVICE_NAME)
        self.get_navpose_service = nepi_ros.create_serviceProxy(NAVPOSE_SERVICE_NAME, NavPoseQuery)
        time.sleep(1)
        self.nepi_ros.start_timer_process(self.update_navpose_interval_sec, self.updateNavPoseCb)


        ####################################
        ## Initiation Complete
        self.rbx_info.connected = True
        self.rbx_status.ready = True 
        self.initCb(do_updates = True)
        self.nepi_ros.start_timer_process(self.rbx_status_pub_interval, self.statusPublishCb)
        self.publishInfo()
        self.publishStatus()
        self.ready = True
        self.msg_if.pub_info("RBX IF Initialization Complete")
         

    def initConfig(self):
        self.initCb(do_updates = True)

    def initCb(self, do_updates = False):
      if do_updates == True:
        self.resetCb(do_updates)

    def resetCb(self, do_updates = True):
        if do_updates == True:  
          self.ApplyConfigUpdates()
        self.publishInfo()

    def factoryResetCb(self):
        self.settings_if.factory_reset_settings()
        if self.setMotorControlRatio is not None:
          mc = RBXMotorControl()
          mc.speed_ratio = 0.0
          for i in range(len(self.getMotorControlRatios())):
            mc.motor_ind = i
            self.setMotorControlRatio(mc)
        self.ApplyConfigUpdates()
        self.publishInfo()

    def updateDeviceNameCb(self, msg):
        self.msg_if.pub_info("Received Device Name update msg")
        #self.msg_if.pub_info(msg)
        new_device_name = msg.data
        self.updateDeviceName(new_device_name)

    def updateDeviceName(self, new_device_name):
        valid_name = True
        for char in self.BAD_NAME_CHAR_LIST:
            if new_device_name.find(char) != -1:
                valid_name = False
        if valid_name is False:
            self.update_error_msg("Received invalid device name update: " + new_device_name)
        else:
            self.nepi_if.set_param('rbx/device_name', new_device_name)
        self.device_save_config_pub.publish(Empty())
        self.publishInfo()


    def resetDeviceNameCb(self,msg):
        self.msg_if.pub_info("Received Device Name reset msg")
        #self.msg_if.pub_info(msg)
        self.resetDeviceName()

    def resetDeviceName(self):
        self.nepi_if.set_param('rbx/device_name', self.factory_device_name)
        self.device_save_config_pub.publish(Empty())
        self.publishInfo()


    

    ##############################
    ### Update image source subscriber
    def imageSubscriberCb(self,img_msg):
        #Convert image from ros to cv2
        cv2_img = nepi_img.rosimg_to_cv2img(img_msg, "bgr8")
        self.cv2_img = cv2_img

    ##############################
    # RBX Settings Topic Callbacks

    # ToDo: Create a custom RBX status message
    ### Callback to set state
    def setStateCb(self,state_msg):
        self.msg_if.pub_info("Received set state message")
        self.msg_if.pub_info(state_msg)
        state_val = state_msg.data
        self.setState(state_val)
        

    ### Function to set state
    def setState(self,new_state_ind):
        if new_state_ind < 0 or new_state_ind > (len(self.states)-1):
            self.update_error_msg("No matching rbx state found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_current = self.states[new_state_ind]
            self.rbx_state_last = self.rbx_info.state
            self.msg_if.pub_info("Waiting for rbx state " + self.states[new_state_ind] + " to set")
            self.setStateIndFunction(new_state_ind)
            time.sleep(1)
            self.msg_if.pub_info("Current rbx state is " + self.states[self.getStateIndFunction()])
            self.rbx_status.process_last = self.states[new_state_ind]
            self.rbx_status.process_current = "None"
            str_val = self.states[new_state_ind]
            self.last_cmd_string = "nepi_rbx.set_rbx_state(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
        self.publishInfo()
        

    ### Callback to set mode
    def setModeCb(self,mode_msg):
        self.msg_if.pub_info("Received set mode message")
        self.msg_if.pub_info(mode_msg)
        mode_val = mode_msg.data
        self.setMode(mode_val)

    ### Function to set mode
    def setMode(self,new_mode_ind):
        if new_mode_ind < 0 or new_mode_ind > (len(self.modes)-1):
            self.update_error_msg("No matching rbx mode found")
        else:
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_status.process_current = self.modes[new_mode_ind]
            self.msg_if.pub_info("Setting rbx mode to : " + self.modes[new_mode_ind])
            self.setModeIndFunction(new_mode_ind)
            self.msg_if.pub_info("Current rbx mode is " + self.modes[self.getModeIndFunction()])
            self.rbx_status.process_last = self.modes[new_mode_ind]
            self.rbx_status.process_current = "None"
            str_val = self.modes[new_mode_ind]
            self.last_cmd_string = "nepi_rbx.set_rbx_mode(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
        self.publishInfo()

    ### Callback to execute action
    def setupActionCb(self,action_msg):
        self.msg_if.pub_info("Received setup action message")
        self.msg_if.pub_info(action_msg)
        action_ind = action_msg.data
        if self.setSetupActionIndFunction is not None:
            if action_ind < 0 or action_ind > (len(self.setup_actions)-1):
                self.update_error_msg("No matching rbx action found")
            else:
                if self.rbx_status.ready is False:
                    self.update_error_msg("Another Command Process is Active")
                    self.update_error_msg("Ignoring this Request")
                else:
                    self.rbx_status.process_current = self.setup_actions[action_ind]
                    self.rbx_status.ready = False
                    self.rbx_cmd_success_current = False
                    self.msg_if.pub_info("Starting action: " + self.setup_actions[action_ind])
                    success = self.setSetupActionIndFunction(action_ind)
                    self.rbx_cmd_success_current = success
                    if success:
                      self.msg_if.pub_info("Finished action: " + self.setup_actions[action_ind])
                    else:
                      self.msg_if.pub_info("Action: " + self.setup_actions[action_ind] + " Failed to complete")
                    self.rbx_status.process_last = self.setup_actions[action_ind]
                    self.rbx_status.process_current = "None"
                    self.rbx_status.cmd_success = self.rbx_cmd_success_current
                    time.sleep(0.5)
                    self.rbx_status.ready = True

                    str_val = self.setup_actions[action_ind]
                    self.last_cmd_string = "nepi_rbx.setup_rbx_action(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                    self.publishInfo()
        else:
            self.update_error_msg("Ignoring Setup Action command, no Set Action Function")


    ### Callback to start rbx set goto goals process
    def setErrorBoundsCb(self,error_bounds_msg):
        self.msg_if.pub_info("Received set goals message")
        self.msg_if.pub_info(error_bounds_msg)
        self.nepi_if.set_param('rbx/max_error_m', error_bounds_msg.max_distance_error_m)
        self.nepi_if.set_param('rbx/max_error_deg', error_bounds_msg.max_rotation_error_deg)
        self.nepi_if.set_param('rbx/stabilized_sec', error_bounds_msg.min_stabilize_time_s)
        self.rbx_info.error_bounds = error_bounds_msg
        self.publishInfo()

    ### Callback to set cmd timeout
    def setCmdTimeoutCb(self,cmd_timeout_msg):
        self.msg_if.pub_info("Received set timeout message")
        self.msg_if.pub_info(cmd_timeout_msg)
        self.nepi_if.set_param('rbx/cmd_timeout', cmd_timeout_msg.data)
        self.rbx_info.cmd_timeout = cmd_timeout_msg.data 
        self.publishInfo()


    ### Callback to image topic source
    def setImageTopicCb(self,set_image_topic_msg):
        self.msg_if.pub_info("Received set image topic message")
        self.msg_if.pub_info(set_image_topic_msg)
        self.nepi_if.set_param('rbx/image_source', set_image_topic_msg.data)
        self.publishInfo()


    ### Callback to add overlay to image topic source
    def enableImageOverlayCb(self,enable_msg):
        self.msg_if.pub_info("Received enable image overlay message")
        self.msg_if.pub_info(enable_msg)
        self.nepi_if.set_param('rbx/image_status_overlay', enable_msg.data)
        self.publishInfo()

    ### Callback to set current process name
    def setProcessNameCb(self,set_process_name_msg):
        self.msg_if.pub_info("Received set process name message")
        self.msg_if.pub_info(set_process_name_msg)
        self.rbx_status.process_current = (set_process_name_msg.data)

         
    

    ##############################
    # RBX Control Topic Callbacks

   ### Callback to set manual motor control ratio
    def setMotorControlCb(self,motor_msg):
        self.msg_if.pub_info("Received set motor control ratio message")
        self.msg_if.pub_info(motor_msg)
        if self.setMotorControl is not None:
          new_motor_ctrl = mode_msg.data
          self.setMotorControl(new_motor_ctrl)

    ### Function to set motor control
    def setMotorControl(self,new_motor_ctrl):
        if self.manualControlsReadyFunction() is True and self.setMotorControlRatio is not None:
            m_ind = new_motor_ctrl.motor_ind
            m_sr = new_motor_ctrl.speed_ratio
            m_len = len(self.getMotorControlRatios())
            if m_ind > (m_len -1):
                self.update_error_msg("New Motor Control Ind " + str(m_ind) + " is out of range")
            elif new_motor_ctrl < 0 or new_motor_ctrl > 1:
                self.update_error_msg("New Motor Control Speed Ratio " + str(m_sr) + " is out of range")
            elif self.setMotorControlRatio is not None:
                self.setMotorControlRatio(m_ind,m_sr)
        else:
            self.update_error_msg("Ignoring Set Motor Control msg, Manual Controls not Ready")     


 


    ### Callback to set home
    def setHomeCb(self,geo_msg):
        self.msg_if.pub_info("Received set home message")
        self.msg_if.pub_info(geo_msg)
        new_home_loc = [geo_msg.latitude,geo_msg.longitude,geo_msg.altitude]
        for i, loc in enumerate(new_home_loc):
          if loc == -999:
            new_home_loc[i] = self.current_location_wgs84_geo[i]
        if self.setHomeFunction is not None:
            new_home_geo = GeoPoint()
            new_home_geo.latitude = new_home_loc[0]
            new_home_geo.longitude = new_home_loc[1]
            new_home_geo.altitude = new_home_loc[2]
            self.setHomeFunction(new_home_geo)


    ### Callback to set home current
    def setHomeCurrentCb(self,empty_msg):
        self.msg_if.pub_info("Received set home current message")
        if self.setHomeFunction is not None:
            self.setHomeFunction(self.current_location_wgs84_geo)

          


    ### Callback to start rbx go home
    def goHomeCb(self,home_msg):
        self.msg_if.pub_info("Received go home message")
        if self.goHomeFunction is not None:
            self.rbx_status.process_current = "Go Home"
            self.rbx_cmd_success_current = False
            self.rbx_status.ready = False
            self.update_prev_errors()
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_cmd_success_current = self.goHomeFunction()
            self.rbx_status.process_last = "Go Home"
            self.rbx_status.process_current = "None"
            self.rbx_status.cmd_success = self.rbx_cmd_success_current
            time.sleep(0.5)
            self.rbx_status.ready = True
            self.last_cmd_string = "nepi_rbx.go_rbx_home(self,timeout_sec = " + str(self.rbx_info.cmd_timeout)
            self.publishInfo()

    ### Callback to start rbx stop
    def goStopCb(self,stop_msg):
        self.msg_if.pub_info("Received go stop message")
        self.msg_if.pub_info(stop_msg)
        time.sleep(1)
        if self.goStopFunction is not None:
            self.rbx_status.process_current = "Stop"
            self.rbx_cmd_success_current = False
            self.rbx_status.ready = False
            self.update_prev_errors()
            self.update_current_errors( [0,0,0,0,0,0,0] )
            self.rbx_cmd_success_current = self.goStopFunction()
            self.rbx_status.process_last = "Stop"
            self.rbx_status.process_current = "None"
            self.rbx_status.cmd_success = self.rbx_cmd_success_current
            time.sleep(0.5)
            self.rbx_status.ready = True
            self.last_cmd_string = "nepi_rbx.go_rbx_stop(self,timeout_sec = " + str(self.rbx_info.cmd_timeout)
            self.publishInfo()

 
  ### Callback to execute action
    def goActionCb(self,action_msg):
        self.msg_if.pub_info("Received go action message")
        self.msg_if.pub_info(action_msg)
        action_ind = action_msg.data
        if self.setGoActionIndFunction is not None:
            if action_ind < 0 or action_ind > (len(self.go_actions)-1):
                self.update_error_msg("No matching rbx action found")
            else:
                if self.rbx_status.ready is False:
                    self.update_error_msg("Another GoTo Command Process is Active")
                    self.update_error_msg("Ignoring this Request")
                else:
                    self.rbx_status.process_current = self.go_actions[action_ind]
                    self.rbx_status.ready = False
                    self.rbx_cmd_success_current = False
                    self.msg_if.pub_info("Starting action: " + self.go_actions[action_ind])
                    success = self.setGoActionIndFunction(action_ind)
                    self.rbx_cmd_success_current = success
                    if success:
                      self.msg_if.pub_info("Finished action: " + self.go_actions[action_ind])
                    else:
                      self.msg_if.pub_info("Action: " + self.go_actions[action_ind] + " Failed to complete")
                    self.rbx_status.process_last = self.go_actions[action_ind]
                    self.rbx_status.process_current = "None"
                    self.rbx_status.cmd_success = self.rbx_cmd_success_current
                    time.sleep(0.5)
                    self.rbx_status.ready = True

                    str_val = self.go_actions[action_ind]
                    self.last_cmd_string = "nepi_rbx.go_rbx_action(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                    self.publishInfo()
        else:
            self.update_error_msg("Ignoring Go Action command, no Set Action Function")


    ### Callback to start rbx goto pose process
    def gotoPoseCb(self,pose_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Pose Message")
        self.msg_if.pub_info(pose_cmd_msg)
        time.sleep(1)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[pose_cmd_msg.roll_deg,pose_cmd_msg.pitch_deg,pose_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo POSE Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Pose"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.setpoint_attitude_ned(setpoint_data)
                self.rbx_status.process_last = "GoTo Pose"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.rbx_status.ready = True

                str_val = str(setpoint_data)
                self.last_cmd_string = "nepi_rbx.goto_rbx_pose(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                self.publishInfo()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")



    ### Callback to start rbx goto position process
    def gotoPositionCb(self,position_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Position Command Message")
        self.msg_if.pub_info(position_cmd_msg)
        time.sleep(1)
        if self.rbx_status.manual_control_mode_ready is False:
            setpoint_data=[position_cmd_msg.x_meters,position_cmd_msg.y_meters,position_cmd_msg.z_meters,position_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo Position Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Position"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.setpoint_position_local_body(setpoint_data)
                self.rbx_status.process_last = "GoTo Position"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.rbx_status.ready = True
                
                str_val = str(setpoint_data)
                self.last_cmd_string = "nepi_rbx.goto_rbx_position(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                self.publishInfo()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to start rbx goto location subscriber
    def gotoLocationCb(self,location_cmd_msg):
        self.msg_if.pub_info("Recieved GoTo Location Message")
        self.msg_if.pub_info(location_cmd_msg)
        if self.autonomousControlsReadyFunction() is True:
            setpoint_data=[location_cmd_msg.lat,location_cmd_msg.long,location_cmd_msg.altitude_meters,location_cmd_msg.yaw_deg]
            if self.rbx_status.ready is False:
                self.update_error_msg("Ignoring GoTo Location Request, Another GoTo Command Process is Active")
            else:
                self.rbx_status.process_current = "GoTo Location"
                self.rbx_status.ready = False
                self.rbx_cmd_success_current = False
                self.update_current_errors( [0,0,0,0,0,0,0] )
                self.rbx_cmd_success_current = self.setpoint_location_global_wgs84(setpoint_data)
                self.rbx_status.process_last = "GoTo Location"
                self.rbx_status.process_current = "None"
                self.rbx_status.cmd_success = self.rbx_cmd_success_current
                time.sleep(0.5)
                self.rbx_status.ready = True

                str_val = str(setpoint_data)
                self.last_cmd_string = "nepi_rbx.goto_rbx_location(self,'" + str_val + "',timeout_sec = " + str(self.rbx_info.cmd_timeout)
                self.publishInfo()
        else:
            self.update_error_msg("Ignoring Go command, Autononous Controls not Ready")

    ### Callback to enble fake gps
    def fakeGPSEnableCb(self,msg):
        self.msg_if.pub_info("Received set set fake gps enable message")
        self.msg_if.pub_info(msg)
        self.nepi_if.set_param('rbx/fake_gps_enabled', msg.data)
        self.setFakeGPSFunction(msg.data)
        self.publishStatus()
        self.publishInfo()

    ### Setup a regular background navpose get and update navpose data
    def updateNavPoseCb(self,timer):
        # Get current NEPI NavPose data from NEPI ROS nav_pose_query service call
        try:
            nav_pose_response = self.get_navpose_service(NavPoseQueryRequest())
            #self.msg_if.pub_info(nav_pose_response)
            # Get current navpose
            current_navpose = nav_pose_response.nav_pose
            # Get current heading in degrees
            self.current_heading_deg = nepi_nav.get_navpose_heading_deg(nav_pose_response)
            # Get current orientation vector (roll, pitch, yaw) in degrees enu frame
            self.current_orientation_enu_degs = nepi_nav.get_navpose_orientation_enu_degs(nav_pose_response)
            # Get current orientation vector (roll, pitch, yaw) in degrees ned frame +-180
            self.current_orientation_ned_degs = nepi_nav.get_navpose_orientation_ned_degs(nav_pose_response)
            # Get current position vector (x, y, z) in meters enu frame
            self.current_position_enu_m = nepi_nav.get_navpose_position_enu_m(nav_pose_response)
            # Get current position vector (x, y, z) in meters ned frame
            self.current_position_ned_m = nepi_nav.get_navpose_position_ned_m(nav_pose_response)
            # Get current geoid hieght
            self.current_geoid_height_m =  nepi_nav.get_navpose_geoid_height(nav_pose_response)
            # Get current location vector (lat, long, alt) in geopoint data with WGS84 height
            self.current_location_wgs84_geo =  nepi_nav.get_navpose_location_wgs84_geo(nav_pose_response) 
            # Get current location vector (lat, long, alt) in geopoint data with AMSL height
            self.current_location_amsl_geo =  nepi_nav.get_navpose_location_amsl_geo(nav_pose_response)
    ##      self.msg_if.pub_info(self.current_geoid_height_m)
    ##      self.msg_if.pub_info(self.current_location_wgs84_geo)
    ##      self.msg_if.pub_info(self.current_location_amsl_geo)
        except Exception as e:
            self.update_error_msg("navpose service call failed: " + str(e))


    def ApplyConfigUpdates(self):
        if self.setFakeGPSFunction is not None:
          fake_gps_enabled = self.nepi_if.get_param('rbx/fake_gps_enabled')
          self.setFakeGPSFunction(fake_gps_enabled) 
        if self.setHomeFunction is not None:
          home_location = self.nepi_if.get_param('rbx/home_location')
          geo_home = GeoPoint()
          geo_home.latitude = home_location[0]
          geo_home.longitude = home_location[1]
          geo_home.altitude = home_location[2]
          self.setHomeFunction(geo_home)

  
    def setCurrentAsDefault(self):
        if self.settings_if is not None:
          self.settings_if.initialize_settings(do_updates = False)
        pass # We only use the param server, no member variables to apply to param server
   
    def capabilities_query_callback(self, _):
        return self.capabilities_report
    
    def navpose_capabilities_query_callback(self, _):
        return self.navpose_capabilities_report  

           

  # RBX Status Topic Publishers

    def publishInfoCb(self, msg):
        self.publishInfo()
    
    
    def publishInfo(self):
        self.rbx_info.device_name = self.nepi_if.get_param('rbx/device_name')
        error_bounds = RBXErrorBounds()
        error_bounds.max_distance_error_m = self.nepi_if.get_param('rbx/max_error_m')
        error_bounds.max_rotation_error_deg = self.nepi_if.get_param('rbx/max_error_deg')
        error_bounds.min_stabilize_time_s = self.nepi_if.get_param('rbx/stabilized_sec')
        self.rbx_info.error_bounds = error_bounds
        self.rbx_info.cmd_timeout = self.nepi_if.get_param('rbx/cmd_timeout')
        self.rbx_info.image_status_overlay = self.nepi_if.get_param('rbx/image_status_overlay') 
        self.rbx_info.state = self.getStateIndFunction()
        self.rbx_info.mode = self.getModeIndFunction()
        if self.getHomeFunction is not None:
          home_geo = self.getHomeFunction()
          home_location = [home_geo.latitude,home_geo.longitude,home_geo.altitude]   
        else: # No Home Information
          home_location = [-999,-999,-999]    
        self.rbx_info.home_lat = home_location[0]
        self.rbx_info.home_long = home_location[1]
        self.rbx_info.home_alt = home_location[2]
        self.rbx_info.fake_gps_enabled = self.nepi_if.get_param('rbx/fake_gps_enabled')

        if not self.nepi_ros.is_shutdown():
            #self.msg_if.pub_info(self.rbx_info)
            self.rbx_info_pub.publish(self.rbx_info)

    ### Callback for rbx status publisher
    def statusPublishCb(self,timer):
        self.publishStatus()

    def publishStatusCb(self, msg):
        self.publishStatus()

    def publishStatus(self):
        # Update NavPose Info
        if self.getBatteryPercentFunction is not None:
          self.rbx_battery = self.getBatteryPercentFunction()
        else:
          self.rbx_battery = -999
        self.rbx_status.current_lat = self.current_location_wgs84_geo[0]
        self.rbx_status.current_long  = self.current_location_wgs84_geo[1]
        self.rbx_status.current_altitude  = self.current_location_wgs84_geo[2]
        self.rbx_status.current_heading = self.current_heading_deg
        self.rbx_status.current_roll = self.current_orientation_ned_degs[0]
        self.rbx_status.current_pitch  = self.current_orientation_ned_degs[1]
        self.rbx_status.current_yaw = self.current_orientation_ned_degs[2]

        self.rbx_status.last_cmd_string = self.last_cmd_string
        self.rbx_status.fake_gps_enabled = self.nepi_if.get_param('rbx/fake_gps_enabled')

        ## Update Control Info
        if self.manualControlsReadyFunction is not None:
          self.rbx_status.manual_control_mode_ready = self.manualControlsReadyFunction()
        else:
          self.rbx_status.manual_control_mode_ready = False

        if self.getMotorControlRatios is not None:
            motor_controls_msg = self.get_motor_controls_status_msg(self.getMotorControlRatios())
        else:
            motor_controls_msg = self.get_motor_controls_status_msg([])
        self.rbx_status.current_motor_control_settings = motor_controls_msg


        if self.autonomousControlsReadyFunction is not None:
          self.rbx_status.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
        else:
          self.rbx_status.autonomous_control_mode_ready = False

         # Create Status Info Text List
        status_str_msg = []
        if self.rbx_status.battery < 0.1:
            battery_string = "No Reading"
        else:
            battery_string = '%.2f' % self.rbx_status.battery

        state_ind = self.getStateIndFunction()
        if len(self.states) > 0 and state_ind >= 0 and state_ind <= (len(self.states)):
              state_name = self.states[state_ind]
        else:
              state_name = "Not Set"
        status_str_msg.append("State Current: " + state_name)

        	
        mode_ind = self.getModeIndFunction()
        if len(self.modes) > 0 and mode_ind >= 0 and mode_ind < (len(self.modes) ):
              mode_name = self.modes[mode_ind]
        else:
              mode_name = "Not Set"
        status_str_msg.append("Mode Current: " + mode_name)
        
        if self.rbx_mode_last is not None:
          status_str_msg.append("Mode Last: " + self.modes[self.rbx_mode_last])
        else:
          status_str_msg.append("Mode Last: None")
        status_str_msg.append("Ready: " + str(self.rbx_status.ready))
        status_str_msg.append("")
        status_str_msg.append("Current Process: " + self.rbx_status.process_current)
        status_str_msg.append(" XYZ Errors Meters: ")
        status_str_msg.append(" " + '%.2f' % self.rbx_status.errors_current.x_m + "  " + '%.2f' % self.rbx_status.errors_current.y_m + "  " + '%.2f' % self.rbx_status.errors_current.z_m)
        status_str_msg.append(" RPY Errors Degrees: ")
        status_str_msg.append(" " + '%.2f' % self.rbx_status.errors_current.roll_deg + "  " + '%.2f' % self.rbx_status.errors_current.pitch_deg + "  " + '%.2f' % self.rbx_status.errors_current.yaw_deg)
        status_str_msg.append("")
        status_str_msg.append("Last Process: " + self.rbx_status.process_last)
        status_str_msg.append(" Success: " + str(self.rbx_status.cmd_success))
        status_str_msg.append(" XYZ Errors Meters: ")
        status_str_msg.append(" " + '%.2f' % self.rbx_status.errors_prev.x_m + "  " + '%.2f' % self.rbx_status.errors_prev.y_m + "  " + '%.2f' % self.rbx_status.errors_prev.z_m)
        status_str_msg.append(" RPY Errors Degrees: ")
        status_str_msg.append(" " + '%.2f' % self.rbx_status.errors_prev.roll_deg + "  " + '%.2f' % self.rbx_status.errors_prev.pitch_deg + "  " + '%.2f' % self.rbx_status.errors_prev.yaw_deg)
        status_str_msg.append("")

   

        self.status_str_msg = status_str_msg
        if not self.nepi_ros.is_shutdown():
            self.rbx_status_pub.publish(self.rbx_status)
            self.rbx_status_str_pub.publish(str(status_str_msg))

        # Create ROS Image message
        cv2_img = copy.deepcopy(self.cv2_img) # Initialize status image
        # Overlay status info on image
        if self.rbx_info.image_status_overlay:
            box_x = 10
            box_y = 10
            box_w = 350
            box_h = 450
            # Add status box overlay
            cv2.rectangle(cv2_img, (box_x, box_y), (box_w, box_h), (255, 255, 255), -1)
            # Overlay Status Text List
            x=box_x+10 
            y=box_y+20
            status_str_msg = self.status_str_msg
            for text in status_str_msg:
                self.statusTextOverlay(cv2_img,text,x, y)
                y = y + 20
        # Publish new image to ros
        if not self.nepi_ros.is_shutdown():
            self.image_if.publish_cv2_img(cv2_img)
            # You can view the enhanced_2D_image topic at 
            # //192.168.179.103:9091/ in a connected web browser
        self.save_data_if.save_img2file('image',cv2_img,img_out_msg.header.stamp)

        ## Update image source topic and subscriber if changed from last time.
        image_source = self.nepi_if.get_param('rbx/image_source')
        image_topic = nepi_ros.find_topic(image_source)
        if image_topic != "":
          if image_topic != self.rbx_image_source_last:
              if self.rbx_image_sub != None:
                  try:
                    self.msg_if.pub_info("Unsubscribing from image source: " + image_topic)
                    self.rbx_image_sub.unregister()
                    self.rbx_image_sub = None
                    time.sleep(1)
                  except Exception as e:
                    self.msg_if.pub_info(e)
          if self.rbx_image_sub == None:
            self.msg_if.pub_info("Subscribing to image topic: " + image_topic)
            self.rbx_image_sub = self.nepi_ros.create_subscriber(image_topic, Image, self.imageSubscriberCb, queue_size = 1)
            self.nepi_if.set_param('rbx/image_source', image_topic)
        else:
              image_topic = "None"
        if image_topic == "None":
              self.cv2_img = self.rbx_image_blank # Set to blank image if source topic is cleared.
        # Set info.image_source value
        self.rbx_info.image_source = image_topic
        self.rbx_image_source_last = image_topic
 
 
        
    ## Status Text Overlay Function
    def statusTextOverlay(self,cv_image,status_text,x,y):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (x,y)
        fontScale              = 0.5
        fontColor              = (0, 0, 0)
        thickness              = 1
        lineType               = 1
        cv2.putText(cv_image,status_text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)


    #######################
    # RBX IF Methods

    ### Function to set and check setpoint attitude NED command
    ###################################################
    # Input is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
    # Converted to ENU befor sending message
    ###################################################
    def setpoint_attitude_ned(self,setpoint_attitude):
      # setpoint_attitude is [ROLL_NED_DEG, PITCH_NED_DEG, YEW_NED_DEGREES]
      # Use value -999 to use current value
      cmd_success = True
      timeout_sec = self.rbx_info.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info("")
      self.msg_if.pub_info("************************")
      self.msg_if.pub_info("Starting Setpoint Attitude Process")
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_orientation_ned_degs=list(self.current_orientation_ned_degs)
      self.msg_if.pub_info("Attitude Current NED Roll, Pitch, Yaw in Degrees")
      self.msg_if.pub_info(["%.2f" % start_orientation_ned_degs[0],"%.2f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
      ##############################################
      # Condition Inputs
      ##############################################
      input_attitude_ned_degs = list(setpoint_attitude)
      #self.msg_if.pub_info("Attitude Input NED Roll, Pitch, Yaw in Degrees")
      #self.msg_if.pub_info(["%.2f" % input_attitude_ned_degs[0],"%.2f" % input_attitude_ned_degs[1],"%.2f" % input_attitude_ned_degs[2]])
      # Set new attitude in degs NED
      new_attitude_ned_degs=list(start_orientation_ned_degs) # Initialize with start values
      for ind in range(3): # Overwrite current with new if set and valid
        if setpoint_attitude[ind] != -999:
          new_attitude_ned_degs[ind]=setpoint_attitude[ind]
        # Condition to +-180 deg
        if new_attitude_ned_degs[ind] > 180:
          new_attitude_ned_degs[ind] = new_attitude_ned_degs[ind] - 360
      #self.msg_if.pub_info("Attitude Input Conditioned NED Roll, Pitch, Yaw in Degrees")
      #self.msg_if.pub_info(["%.2f" % new_attitude_ned_degs[0],"%.2f" % new_attitude_ned_degs[1],"%.2f" % new_attitude_ned_degs[2]])
      ##############################################
      # Convert NED attitude to Pose
      ##############################################
      # Convert to ROS ENU attitude degs and create ENU quaternion setpoint attitude goal
      yaw_enu_deg = nepi_nav.convert_yaw_ned2enu(new_attitude_ned_degs[2])
      new_attitude_enu_degs = [new_attitude_ned_degs[0],new_attitude_ned_degs[1],yaw_enu_deg]
      self.msg_if.pub_info("Attitude Goal ENU Roll, Pitch, Yaw in Degrees")
      self.msg_if.pub_info(["%.2f" % new_attitude_enu_degs[0],"%.2f" % new_attitude_enu_degs[1],"%.2f" % new_attitude_enu_degs[2]])
      ##############################################
      ## Send Setpoint Message and Check for Success
      ##############################################
      self.msg_if.pub_info("Sending Setpoint Attitude Command")
      self.gotoPoseFunction(new_attitude_enu_degs)
      self.msg_if.pub_info("Waiting for Attitude Setpoint to complete")
      setpoint_attitude_reached = False
      stabilize_timer=0
      timeout_timer = 0 # Initialize timeout timer
      attitude_errors = [] # Initialize running list of errors
      time2sleep = 0.1
      while setpoint_attitude_reached is False and not self.nepi_ros.is_shutdown():  # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
            self.msg_if.pub_info("Setpoint Attitude received Stop Command")
            new_attitude_ned_degs = copy.deepcopy(cur_attitude_ned_degs)
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment message timer
        timeout_timer = timeout_timer+time2sleep
        # Calculate setpoint attitude errors
        cur_attitude_ned_degs = [self.current_orientation_ned_degs[0],self.current_orientation_ned_degs[1],self.current_orientation_ned_degs[2]]
        attitude_errors_degs = np.array(new_attitude_ned_degs) - np.array(cur_attitude_ned_degs)
        for ind in range(3):
          if input_attitude_ned_degs[ind] == -999.0: # Ignore error check if set to current
            attitude_errors_degs[ind]=0.0
        max_attutude_error_deg = max(abs(attitude_errors_degs))
        # Check for setpoint position local point goal
        if  setpoint_attitude_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_attitude_errors = max(attitude_errors) # Get max from error window
            attitude_errors = [max_attutude_error_deg] # reset running list of errors
            if max_attitude_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Attitude Setpoint Reached")
              setpoint_attitude_reached = True
          else:
            attitude_errors.append(max_attutude_error_deg) # append last
        # Reset timer if past
        if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
          stabilize_timer=0 # Reset timer
        self.update_current_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
      if cmd_success:
        self.msg_if.pub_info("************************")
        self.msg_if.pub_info("Setpoint Reached")
      self.update_current_errors( [0,0,0,0,attitude_errors_degs[0],attitude_errors_degs[1],attitude_errors_degs[2]]  )
      return cmd_success
      


    ### Function to set and check setpoint position local body command
    ###################################################
    # Input is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
    # Converted to Local ENU Frame before sending
    # Local Body Position Setpoint Function use these body relative x,y,z,yaw conventions
    # x+ axis is forward
    # y+ axis is left
    # z+ axis is up
    # Only yaw orientation updated
    # yaw+ counter clockwise from x axis (0 degrees faces x+ and rotates using right hand rule around z+ axis down)
    #####################################################
    def setpoint_position_local_body(self,setpoint_position):
      # setpoint_position is [X_BODY_METERS, Y_BODY_METERS, Z_BODY_METERS, YEW_BODY_DEGREES]
      # use value 0 for no change
      cmd_success = True
      timeout_sec = self.rbx_info.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info(":" + self.log_name + ': ')
      self.msg_if.pub_info("************************")
      self.msg_if.pub_info("Starting Setpoint Position Local Process")
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_geopoint_wgs84 = list(self.current_location_wgs84_geo)
      #self.msg_if.pub_info("Start Location WSG84 geopoint")
      #self.msg_if.pub_info(" Lat, Long, Alt")
      #self.msg_if.pub_info(["%.2f" % start_geopoint_wgs84[0],"%.2f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
      start_position_enu_m = list(self.current_position_enu_m)
      self.msg_if.pub_info("Start Position ENU degs")
      self.msg_if.pub_info(" X, Y, Z")
      self.msg_if.pub_info(["%.2f" % start_position_enu_m[0],"%.2f" % start_position_enu_m[1],"%.2f" % start_position_enu_m[2]])   
      start_orientation_enu_degs=list(self.current_orientation_enu_degs)
      self.msg_if.pub_info("Start Orientation ENU degs")
      self.msg_if.pub_info(" Roll, Pitch, Yaw")
      self.msg_if.pub_info(["%.2f" % start_orientation_enu_degs[0],"%.2f" % start_orientation_enu_degs[1],"%.2f" % start_orientation_enu_degs[2]])

      start_yaw_enu_deg = start_orientation_enu_degs[2]
      self.msg_if.pub_info("Start Yaw ENU degs")
      self.msg_if.pub_info(start_yaw_enu_deg) 
      start_heading_deg=self.current_heading_deg
      #self.msg_if.pub_info("Start Heading degs")
      #self.msg_if.pub_info(start_heading_deg)   
      ##############################################
      # Condition Body Input Data
      ##############################################
      # Condition Point Input
      input_point_body_m=setpoint_position[0:3]
      self.msg_if.pub_info("Input Postion Body  X, Y, Z in Meters")
      self.msg_if.pub_info(["%.2f" % input_point_body_m[0],"%.2f" % input_point_body_m[1],"%.2f" % input_point_body_m[2]])
      new_point_body_m=list(input_point_body_m) # No conditioning required
      #self.msg_if.pub_info("Point Conditioned Body Meters")
      #self.msg_if.pub_info(" X, Y, Z")
      #self.msg_if.pub_info(["%.2f" % new_point_body_m[0],"%.2f" % new_point_body_m[1],"%.2f" % new_point_body_m[2]])
      # Condition Orienation Input
      input_yaw_body_deg = setpoint_position[3]
      self.msg_if.pub_info("Yaw Input Body Degrees")
      self.msg_if.pub_info(["%.2f" % input_yaw_body_deg])   
      ##############################################
      # Convert Body Data to ENU Data
      ##############################################
      # Set new yaw orientation in ENU degrees
      offset_enu_m = nepi_nav.convert_point_body2enu(new_point_body_m,start_yaw_enu_deg)
      self.msg_if.pub_info("Point Goal Offsets ENU Meters")
      self.msg_if.pub_info(" X, Y, Z")
      self.msg_if.pub_info(["%.2f" % offset_enu_m[0],"%.2f" % offset_enu_m[1],"%.2f" % offset_enu_m[2]])
      new_x_enu_m = start_position_enu_m[0] + offset_enu_m[0]
      new_y_enu_m = start_position_enu_m[1] + offset_enu_m[1]
      new_z_enu_m = start_position_enu_m[2] + offset_enu_m[2]
      new_position_enu_m = [new_x_enu_m,new_y_enu_m,new_z_enu_m]
      self.msg_if.pub_info("Point Goal ENU X, Y, Z in Meters")
      self.msg_if.pub_info(["%.2f" % new_position_enu_m[0],"%.2f" % new_position_enu_m[1],"%.2f" % new_position_enu_m[2]])

      new_yaw_enu_deg = start_yaw_enu_deg + input_yaw_body_deg
        # Condition to +-180 deg
      if new_yaw_enu_deg > 180:
        new_yaw_enu_deg = new_yaw_enu_deg - 360
      elif new_yaw_enu_deg < -180:
        new_yaw_enu_deg = 360 + new_yaw_enu_deg
      self.msg_if.pub_info("Yaw Goal ENU Degrees")
      self.msg_if.pub_info(["%.2f" % new_yaw_enu_deg])
      ##############################################
      # Create Point and Pose Data
      ##############################################
      # New Point ENU in meters
      new_point_enu_m=Point()
      new_point_enu_m.x = offset_enu_m[0]
      new_point_enu_m.y = offset_enu_m[1]
      new_point_enu_m.z = offset_enu_m[2]
      self.msg_if.pub_info("Position Goal ENU X, Y, Z in Meters")
      self.msg_if.pub_info(["%.2f" % new_point_enu_m.x,"%.2f" % new_point_enu_m.y,"%.2f" % new_point_enu_m.z])

      new_orientation_enu_deg = [start_orientation_enu_degs[0],start_orientation_enu_degs[1],new_yaw_enu_deg]
      self.msg_if.pub_info("Orienation Goal ENU  Roll, Pitch, Yaw in Degrees")
      self.msg_if.pub_info(["%.2f" % new_orientation_enu_deg[0],"%.2f" % new_orientation_enu_deg[1],"%.2f" % new_orientation_enu_deg[2]])
      ##############################################
      ## Send Message and Check for Setpoint Success
      ##############################################
      self.msg_if.pub_info("Sending Setpoint Position Local Command")
      self.gotoPositionFunction(new_point_enu_m,new_orientation_enu_deg)
      self.msg_if.pub_info("Waiting for Position Setpoint to complete")
      setpoint_position_local_point_reached = False
      setpoint_position_local_yaw_reached = False
      stabilize_timer=0
      point_errors = [] # Initialize running list of errors
      yaw_errors = [] # Initialize running list of errors
      timeout_timer = 0 # Initialize timeout timer
      time2sleep = 0.1
      while setpoint_position_local_point_reached is False or setpoint_position_local_yaw_reached is False and not self.nepi_ros.is_shutdown():  # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
            self.msg_if.pub_info("Setpoint Position received Stop Command")
            new_position_enu_m = copy.deepcopy(self.current_position_enu_m)
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment message timer
        timeout_timer = timeout_timer+time2sleep
        # Calculate setpoint position enu errors    
        point_body_errors_m = [0,0,0] # initialize for later
        point_enu_errors_m = np.array(self.current_position_enu_m) - np.array(new_position_enu_m)
        for ind in range(3):
          if input_point_body_m == -999: # Ignore error check if set to current
            point_enu_errors_m[ind] = 0
        max_point_enu_errors_m = np.max(np.abs(point_enu_errors_m))
        # Calculate setpoint yaw enu error
        if input_yaw_body_deg == -999: # Ignore error check if set to current
          setpoint_position_local_yaw_reached = True
          max_yaw_enu_error_deg = 0
        else:
          cur_yaw_enu_deg = self.current_orientation_enu_degs[2]
          yaw_enu_error_deg =  cur_yaw_enu_deg - new_yaw_enu_deg
          max_yaw_enu_error_deg = abs(yaw_enu_error_deg)
        # Check for setpoint position local point goal
        if  setpoint_position_local_point_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_point_errors = max(point_errors) # Get max from error window
            point_errors = [max_point_enu_errors_m] # reset running list of errors
            if max_point_errors < self.rbx_info.error_bounds.max_distance_error_m:
              self.msg_if.pub_info("Position Setpoint Reached")
              setpoint_position_local_point_reached = True
          else:
            point_errors.append(max_point_enu_errors_m) # append last
        # Check for setpoint position yaw point goal
        if  setpoint_position_local_yaw_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_yaw_errors = max(yaw_errors) # Get max from error window
            yaw_errors = [max_yaw_enu_error_deg] # reset running list of errors
            if max_yaw_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Yaw Setpoint Reached")
              setpoint_position_local_yaw_reached = True
          else:
            yaw_errors.append(max_yaw_enu_error_deg) # append last
        # Reset timer if past
        if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
          stabilize_timer=0 # Reset timer
        point_body_errors_m = nepi_nav.convert_point_enu2body(point_enu_errors_m,start_yaw_enu_deg)
        self.update_current_errors(  [point_body_errors_m[1],point_body_errors_m[0],point_body_errors_m[2],0,0,0,max_yaw_enu_error_deg] )
      if cmd_success:
        self.msg_if.pub_info("Setpoint Reached")
        self.msg_if.pub_info("************************")
      point_body_errors_m = nepi_nav.convert_point_enu2body(point_enu_errors_m,start_yaw_enu_deg)
      self.update_current_errors(  [point_body_errors_m[1],point_body_errors_m[0],point_body_errors_m[2],0,0,0,max_yaw_enu_error_deg] )
      return cmd_success



    ### Function to set and check setpoint location global geopoint and yaw command
    ###################################################
    # Input is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
    # Converted to AMSL Altitude and ENU Yaw berore sending
    # Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
    # Yaw is specified in NED frame degrees 0-360 or +-180 
    #####################################################
    def setpoint_location_global_wgs84(self,setpoint_location):
      # setpoint_location is [LAT, LONG, ALT_WGS84, YEW_NED_DEGREES 0-360 or +-180]
      # Use value -999 to use current value
      cmd_success = True
      timeout_sec = self.rbx_info.cmd_timeout
      self.update_prev_errors()
      self.update_current_errors( [0,0,0,0,0,0,0] )
      self.msg_if.pub_info(":" + self.log_name + ': ')
      self.msg_if.pub_info("************************")
      self.msg_if.pub_info("Starting Setpoint Location Global Process")
      ##############################################
      # Capture Current NavPose Data
      ##############################################
      start_geopoint_wgs84 = list(self.current_location_wgs84_geo)  
      #self.msg_if.pub_info("Start Location WSG84 geopoint")
      #self.msg_if.pub_info(" Lat, Long, Alt")
      #self.msg_if.pub_info(["%.6f" % start_geopoint_wgs84[0],"%.6f" % start_geopoint_wgs84[1],"%.2f" % start_geopoint_wgs84[2]])
      start_orientation_ned_degs=list(self.current_orientation_ned_degs)
      #self.msg_if.pub_info("Start Orientation NED degs")
      #self.msg_if.pub_info(" Roll, Pitch, Yaw")
      #self.msg_if.pub_info(["%.6f" % start_orientation_ned_degs[0],"%.6f" % start_orientation_ned_degs[1],"%.2f" % start_orientation_ned_degs[2]])
      start_yaw_ned_deg = start_orientation_ned_degs[2]
      if start_yaw_ned_deg < 0:
        start_yaw_ned_deg = start_yaw_ned_deg + 360
      #self.msg_if.pub_info("Start Yaw NED degs 0-360")
      self.msg_if.pub_info(start_yaw_ned_deg) 
      start_heading_deg=self.current_heading_deg
      self.msg_if.pub_info("Start Heading degs")
      self.msg_if.pub_info(start_heading_deg)
      start_geoid_height_m = self.current_geoid_height_m
      ##############################################
      # Condition NED Input Data
      ##############################################
      # Condition Location Input
      input_geopoint_wgs84 = list(setpoint_location[0:3])
      #self.msg_if.pub_info("Location Input Global Geo")
      #self.msg_if.pub_info(" Lat, Long, Alt_WGS84")
      #self.msg_if.pub_info(["%.8f" % input_geopoint_wgs84[0],"%.8f" % input_geopoint_wgs84[1],"%.2f" % input_geopoint_wgs84[2]])
      new_geopoint_wgs84=list(start_geopoint_wgs84) # Initialize with start
      for ind in range(3): # Overwrite current with new if set and valid
        if input_geopoint_wgs84[ind] != -999:
          new_geopoint_wgs84[ind]=input_geopoint_wgs84[ind]
      #self.msg_if.pub_info("Location Input Conditioned Global Geo")
      #self.msg_if.pub_info(" Lat, Long, Alt_WGS84")
      #self.msg_if.pub_info(["%.8f" % new_geopoint_wgs84[0],"%.8f" % new_geopoint_wgs84[1],"%.2f" % new_geopoint_wgs84[2]])
      # Condition Yaw Input
      input_yaw_ned_deg = setpoint_location[3]
      #self.msg_if.pub_info("Yaw Input NED Degrees")
      #self.msg_if.pub_info(["%.2f" % input_yaw_ned_deg])
      new_yaw_ned_deg = start_yaw_ned_deg # Initialize to current
      if input_yaw_ned_deg != -999: # Replace if not -999
        new_yaw_ned_deg = input_yaw_ned_deg
      # Condition to 0-360 degs
      if new_yaw_ned_deg < 0:
        new_yaw_ned_deg = new_yaw_ned_deg + 360
      #self.msg_if.pub_info("Yaw Input Conditioned NED Degrees 0-360")
      #self.msg_if.pub_info(["%.2f" % new_yaw_ned_deg])      
      ##############################################
      # Create Global AMSL Location and NED Orienation Setpoint Values
      ##############################################
      # New Global location ENU in meters
      new_geopoint_amsl=GeoPoint()
      new_geopoint_amsl.latitude = new_geopoint_wgs84[0]
      new_geopoint_amsl.longitude = new_geopoint_wgs84[1]
      new_geopoint_amsl.altitude = new_geopoint_wgs84[2] + start_geoid_height_m
      self.msg_if.pub_info("Location Goal Lat, Long, Alt_AMSL")
      self.msg_if.pub_info(["%.8f" % new_geopoint_amsl.latitude,"%.8f" % new_geopoint_amsl.longitude,"%.2f" % new_geopoint_amsl.altitude])
      # New Local Orienation NED in degs  
      new_orientation_ned_deg = [start_orientation_ned_degs[0],start_orientation_ned_degs[1],new_yaw_ned_deg]
      self.msg_if.pub_info("Orienation Goal NED  Roll, Pitch, Yaw in Degrees")
      self.msg_if.pub_info(["%.2f" % new_orientation_ned_deg[0],"%.2f" % new_orientation_ned_deg[1],"%.2f" % new_orientation_ned_deg[2]])
      ##############################################
      ## Send Message and Check for Setpoint Success
      ##############################################
      self.msg_if.pub_info("Sending MAVLINK Setpoint Position Local Command")
      self.gotoLocationFunction(new_geopoint_amsl,new_orientation_ned_deg)
      self.msg_if.pub_info(" checking for Setpoint Reached")
      setpoint_location_global_geopoint_reached = False
      setpoint_location_global_yaw_reached = False 
      self.msg_if.pub_info("Waiting for Position Local Setpoint to complete")
      stabilize_timer=0
      geopoint_errors = [] # Initialize running list of errors
      yaw_errors = [] # Initialize running list of errors
      timeout_timer = 0 # Initialize timeout timer
      time2sleep = 0.1
      while (setpoint_location_global_geopoint_reached is False or setpoint_location_global_yaw_reached is False) and not self.nepi_ros.is_shutdown(): # Wait for setpoint goal to be set
        if self.checkStopFunction() is True:
          self.msg_if.pub_info("Setpoint Location received Stop Command")
          new_geopoint_wgs84 = copy.deepcopy(self.current_location_wgs84_geo)
        if timeout_timer > timeout_sec:
          self.update_error_msg("Setpoint cmd timed out")
          cmd_success = False
          break
        time.sleep(time2sleep) # update setpoint position at 50 Hz
        stabilize_timer=stabilize_timer+time2sleep # Increment self.update_error_msg message timer
        timeout_timer = timeout_timer+time2sleep        
        # Calculate setpoint position and yaw errors
        geopoint_errors_geo = np.array(self.current_location_wgs84_geo) - np.array(new_geopoint_wgs84)
        geopoint_errors_m = [geopoint_errors_geo[0]*111139,geopoint_errors_geo[1]*111139,geopoint_errors_geo[2]]
        for ind in range(3):  # Ignore error check if set to current
          if input_geopoint_wgs84[ind] == -999.0:
            geopoint_errors_m[ind] = 0
        max_geopoint_error_m = np.max(np.abs(geopoint_errors_m))
        if input_yaw_ned_deg == -999: # Ignore error check if set to current
          setpoint_location_global_yaw_reached = True
          max_yaw_ned_error_deg = 0
        else:
          cur_yaw_ned_deg = self.current_orientation_ned_degs[2]
          if cur_yaw_ned_deg < 0:
            cur_yaw_ned_deg = cur_yaw_ned_deg + 360
          yaw_ned_error_deg =  cur_yaw_ned_deg - new_yaw_ned_deg
          max_yaw_ned_error_deg = abs(yaw_ned_error_deg)
        # Check for setpoint position global goal
        if  setpoint_location_global_geopoint_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_geopoint_errors = max(geopoint_errors) # Get max from error window
            geopoint_errors = [max_geopoint_error_m] # reset running list of errors
            if max_geopoint_errors < self.rbx_info.error_bounds.max_distance_error_m:
              self.msg_if.pub_info("Location Setpoint Reached")
              setpoint_location_global_geopoint_reached = True
          else:
            geopoint_errors.append(max_geopoint_error_m) # append last
        # Check for setpoint position yaw goal
        if  setpoint_location_global_yaw_reached is False:
          if stabilize_timer > self.rbx_info.error_bounds.min_stabilize_time_s:
            max_yaw_errors = max(yaw_errors) # Get max from error window
            yaw_errors = [max_yaw_ned_error_deg] # reset running list of errors
            if max_yaw_errors < self.rbx_info.error_bounds.max_rotation_error_deg:
              self.msg_if.pub_info("Yaw Setpoint Reached")
              setpoint_location_global_yaw_reached = True
          else:
            yaw_errors.append(max_yaw_ned_error_deg) # append last
        # Reset timer if past
        if stabilize_timer > 1:
          stabilize_timer=0 # Reset timer
        self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
      if cmd_success:
        self.msg_if.pub_info("Setpoint Reached")
        self.msg_if.pub_info("************************")
      self.update_current_errors( [geopoint_errors_m[0],geopoint_errors_m[1],geopoint_errors_m[2],0,0,0,max_yaw_ned_error_deg] )
      return cmd_success

    #######################
    # Class Utility Functions
    
    ### Function for updating current goto error values
    def update_current_errors(self,error_list):
      if len(error_list) == 7:
        errors_msg = RBXGotoErrors()
        errors_msg.x_m = error_list[0]
        errors_msg.y_m = error_list[1]
        errors_msg.z_m = error_list[2]
        errors_msg.heading_deg = error_list[3]
        errors_msg.roll_deg = error_list[4]
        errors_msg.pitch_deg = error_list[5]
        errors_msg.yaw_deg = error_list[6]

        self.rbx_status.errors_current = errors_msg
      else:
        self.msg_if.pub_info("Skipping current error update. Error list to short")

    ### Function for updating last goto error values
    def update_prev_errors(self):
        errors_msg = RBXGotoErrors()
        errors_msg.x_m = self.rbx_status.errors_current.x_m
        errors_msg.y_m = self.rbx_status.errors_current.y_m
        errors_msg.z_m = self.rbx_status.errors_current.z_m
        errors_msg.heading_deg = self.rbx_status.errors_current.heading_deg
        errors_msg.roll_deg = self.rbx_status.errors_current.roll_deg
        errors_msg.pitch_deg = self.rbx_status.errors_current.pitch_deg
        errors_msg.yaw_deg = self.rbx_status.errors_current.yaw_deg
        self.rbx_status.errors_prev = errors_msg


    def update_error_msg(self,error_msg):
      self.msg_if.pub_info(error_msg)
      self.rbx_status.last_error_message = error_msg

    def get_motor_controls_status_msg(self,motor_controls):
      mcs = []
      for i in range(len(motor_controls)):
        mc = RBXMotorControl()
        mc.motor_ind = i
        mc.speed_ratio = motor_controls[i]
        mcs.append(mc)
      return mcs


       


    
