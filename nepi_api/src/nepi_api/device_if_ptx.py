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
import copy 
import copy

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_ros_interfaces.msg import RangeWindow
from nepi_ros_interfaces.msg import PTXStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, AbsolutePanTiltWaypoint
from nepi_ros_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryRequest, PTXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.device_if_npx import NPXDeviceIF

#from nepi_api.device_if_npx import NPXDeviceIF



class PTXActuatorIF:
    MAX_STATUS_UPDATE_RATE = 3

    PTX_DIRECTION_POSITIVE = 1
    PTX_DIRECTION_NEGATIVE = -1
    WAYPOINT_COUNT = 256

    # Backup Factory Control Values 
    FACTORY_CONTROLS_DICT = {
                'frame_id' : 'ptx_frame',
                'yaw_joint_name' : 'ptx_yaw_joint',
                'pitch_joint_name' : 'ptx_pitch_joint',
                'reverse_yaw_control' : False,
                'reverse_pitch_control' : False,
                'speed_ratio' : 0.5
    }

    data_products_list = ['orientation']

    orientation_dict = {
        'time_orientation': nepi_utils.get_time(),
        # Orientation should be provided in Degrees ENU
        'roll_deg': 0.0,
        'pitch_deg': 0.0,
        'yaw_deg': 0.0,
    }

    ready = False

    has_absolute_positioning = False
    has_timed_positioning = False
    has_adjustable_limits = False
    has_speed_control = False
    has_auto_pan = False
    has_auto_tilt = False
    has_homing = False
    has_waypoints = False


    # Define some member variables
    yaw_now_deg = 0.0
    yaw_goal_deg = -999
    yaw_home_pos_deg = 0.0
    min_yaw_softstop_deg = 0.0
    max_yaw_softstop_deg = 0.0
    pitch_now_deg = 0.0
    pitch_goal_deg = -999
    pitch_home_pos_deg = 0.0
    min_pitch_softstop_deg = 0.0
    max_pitch_softstop_deg = 0.0

    home_yaw_deg = 0.0
    home_pitch_deg = 0.0

    reverse_yaw_control = False
    ryi = 1
    reverse_pitch_control = False
    rpi = 1

    last_yaw = 0
    last_pitch = 0

    max_yaw_hardstop_deg = 0
    min_yaw_hardstop_deg = 0

    max_pitch_hardstop_deg = 0
    min_pitch_hardstop_deg = 0

    max_yaw_softstop_deg = 0
    min_yaw_softstop_deg = 0

    max_pitch_softstop_deg = 0
    min_pitch_softstop_deg = 0

    speed_ratio = 0.0

    is_auto_pan = False
    auto_pan = False
    auto_pan_min = -10
    auto_pan_max = 10

    is_auto_tilt = False
    auto_tilt = False
    auto_tilt_min = -10
    auto_tilt_max = 10


    ### IF Initialization
    def __init__(self,  device_info, 
                 capSettings, factorySettings, 
                 settingUpdateFunction, getSettingsFunction,
                 factoryControls , # Dictionary to be supplied by parent, specific key set is required
                 factoryLimits,
                 capabilities_dict, # Dictionary to be supplied by parent, specific key set is required
                 stopMovingCb, # Required; no args
                 moveYawCb, # Required; direction and time args
                 movePitchCb, # Required; direction and time args
                 setSoftLimitsCb=None,
                 getSoftLimitsCb=None,
                 setSpeedRatioCb=None, # None ==> No speed adjustment capability; Speed ratio arg
                 getSpeedRatioCb=None, # None ==> No speed adjustment capabilitiy; Returns speed ratio
                 gotoPositionCb=None, # None ==> No absolute positioning capability (yaw_deg, pitch_deg, speed, float move_timeout_s) 
                 gotoPanPositionCb=None, # None ==> No absolute positioning capability (yaw_deg, pitch_deg, speed, float move_timeout_s) 
                 gotoTiltPositionCb=None, # None ==> No absolute positioning capability (yaw_deg, pitch_deg, speed, float move_timeout_s) 
                 goHomeCb=None, # None ==> No native driver homing capability, can still use homing if absolute positioning is supported
                 setHomePositionCb=None, # None ==> No native driver home absolute setting capability, can still use it if absolute positioning is supported
                 setHomePositionHereCb=None, # None ==> No native driver home instant capture capability, can still use it if absolute positioning is supported
                 gotoWaypointCb=None, # None ==> No native driver support for waypoints, can still use if absolute positioning is supported
                 setWaypointCb=None, # None ==> No native driver support for absolute waypoints, can still use if absolute positioning is supported
                 setWaypointHereCb=None, # None ==> No native driver support for instant waypoints, can still use if absolute positioning is supported
                 capSettingsNavPose=None, factorySettingsNavPose=None, 
                 settingUpdateFunctionNavPose=None, getSettingsFunctionNavPose=None,
                 getHeadingCb = None, getPositionCb = None, getOrientationCb = None,
                 getLocationCb = None, getAltitudeCb = None, getDepthCb = None,
                 max_navpose_update_rate = 10,
                 deviceResetCb = None,
                 log_name = None,
                 log_name_list = [],
                 msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_ros.get_base_namespace()
        self.node_name = nepi_ros.get_node_name()
        self.node_namespace = nepi_ros.get_node_namespace()

        ##############################  
        # Create Msg Class
        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF()
        self.log_name_list = copy.deepcopy(log_name_list)
        self.log_name_list.append(self.class_name)
        if log_name is not None:
            self.log_name_list.append(log_name)
        self.msg_if.pub_info("Starting PTX Device IF Initialization Processes", log_name_list = self.log_name_list)


        ############################## 
        # Initialize Class Variables
        self.device_name = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        # Update status update rate
        if max_navpose_update_rate == None:
            rate = 1.0
        elif max_navpose_update_rate > self.MAX_STATUS_UPDATE_RATE:
            rate = self.MAX_STATUS_UPDATE_RATE
        elif max_navpose_update_rate < 1.0:
            rate = 1.0
        else:
            rate = max_navpose_update_rate
        self.position_update_rate = rate

        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]



        self.deviceResetCb = deviceResetCb
        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]

        self.setSoftLimitsCb = setSoftLimitsCb
        self.getSoftLimitsCb = getSoftLimitsCb
        self.setSpeedRatioCb = setSpeedRatioCb
        self.getSpeedRatioCb = getSpeedRatioCb

        self.stopMovingCb = stopMovingCb


        self.moveYawCb = moveYawCb
        self.movePitchCb = movePitchCb
        if self.moveYawCb is not None and self.movePitchCb is not None:
            self.has_timed_positioning = True
        else:
            self.has_timed_positioning = False


        self.gotoPositionCb = gotoPositionCb
        if gotoPanPositionCb is not None:
            self.gotoPanPositionCb = gotoPanPositionCb
        else:
            if self.gotoPositionCb is not None:
                self.gotoPanPositionCb = self.gotoPanPositionCurTiltCb
            else:
                self.gotoPanPositionCb = None

        if gotoTiltPositionCb is not None:
            self.gotoTiltPositionCb = gotoTiltPositionCb
        else:
            self.gotoTiltPositionCb = None




        self.goHomeCb = goHomeCb
        self.setHomePositionCb = setHomePositionCb
        self.setHomePositionHereCb = setHomePositionHereCb
        self.gotoWaypointCb = gotoWaypointCb
        self.setWaypointCb = setWaypointCb
        self.setWaypointHereCb = setWaypointHereCb



        if getOrientationCb is not None:
            self.has_position_feedback = True
        else:
            self.has_position_feedback = True

              
        if (capabilities_dict['has_absolute_positioning'] == True):
            self.has_absolute_positioning = True
        else:
            self.has_absolute_positioning = False


        if factoryLimits is not None:
            self.factoryLimits = factoryLimits  
        else:
            # Hard limits
            self.factoryLimits['max_yaw_hardstop_deg'] = 0
            self.factoryLimits['min_yaw_hardstop_deg'] = 0
            self.factoryLimits['max_pitch_hardstop_deg'] = 0
            self.factoryLimits['min_pitch_hardstop_deg'] = 0
  
            # Soft limits
            self.factoryLimits['max_yaw_softstop_deg'] = 0
            self.factoryLimits['min_yaw_softstop_deg'] = 0
            self.factoryLimits['max_pitch_softstop_deg'] = 0
            self.factoryLimits['min_pitch_softstop_deg'] = 0


        self.max_yaw_softstop_deg = self.factoryLimits['max_yaw_softstop_deg']
        self.min_yaw_softstop_deg =  self.factoryLimits['min_yaw_softstop_deg']

        self.max_pitch_softstop_deg = self.factoryLimits['max_pitch_softstop_deg']
        self.min_pitch_softstop_deg = self.factoryLimits['min_pitch_softstop_deg']


        # Gather capabilities - Config file takes precedence over parent-supplied defaults

        if capabilities_dict['has_limit_control'] == True:
            if self.setSoftLimitsCb is not None and self.setSoftLimitsCb is not None:
                self.has_adjustable_limits = True
            else:
                self.msg_if.pub_warn("Inconsistent capabilities: adjustable speed reports true, but no callback provided", log_name_list = self.log_name_list)
                self.has_adjustable_limits = False
        else:
            self.has_adjustable_limits = False


        # Gather capabilities - Config file takes precedence over parent-supplied defaults
        if capabilities_dict['has_speed_control'] == True:
            if self.setSpeedRatioCb is not None and self.getSpeedRatioCb is not None:
                self.has_adjustable_speed = True
            else:
                self.msg_if.pub_warn("Inconsistent capabilities: adjustable speed reports true, but no callback provided", log_name_list = self.log_name_list)
                self.has_adjustable_speed = False
        else:
            self.has_adjustable_speed = False
            self.setSpeedRatioCb = None
            self.getSpeedRatioCb = self.getZeroCb

        
        # Create Capabilities Report
        self.capabilities_report = PTXCapabilitiesQueryResponse()


        self.capabilities_report.has_absolute_positioning = self.has_absolute_positioning
        # Positioning and soft limits setup if available
        if self.capabilities_report.has_absolute_positioning == True:
            if (getOrientationCb is None):
                self.msg_if.pub_warn("Inconsistent capabilities: absolute positioning reports true, but no callback provided", log_name_list = self.log_name_list)
                # We require both command and feedback reporting to support absolute positioning
                self.capabilities_report.has_absolute_positioning = False

        self.capabilities_report.has_timed_positioning = self.has_timed_positioning


        if self.gotoPanPositionCb is not None and self.gotoTiltPositionCb is not None:
            self.capabilities_report.has_seperate_pan_tilt = True
        else:
            self.capabilities_report.has_seperate_pan_tilt = False




        if self.capabilities_report.has_absolute_positioning == True:
            self.capabilities_report.has_auto_pan = True
            if self.capabilities_report.has_seperate_pan_tilt == True:
                self.capabilities_report.has_auto_tilt = True
            else:
                self.capabilities_report.has_auto_tilt = False

        else:
            self.capabilities_report.has_auto_pan =  False
            self.capabilities_report.has_auto_tilt =  False

        self.has_auto_pan = self.capabilities_report.has_auto_pan
        self.has_auto_tilt = self.capabilities_report.has_auto_tilt

        if self.gotoPanPositionCb is not None and self.gotoTiltPositionCb is not None:
            self.capabilities_report.has_seperate_pan_tilt = True
        else:
            self.capabilities_report.has_seperate_pan_tilt = False




        if self.capabilities_report.has_adjustable_speed == False:
            if self.setSpeedRatioCb is not None and self.getSpeedRatioCb is not None:
                self.capabilities_report.has_adjustable_speed = True
            else:
                self.msg_if.pub_warn("Inconsistent capabilities: adjustable speed reports true, but no callback provided", log_name_list = self.log_name_list)
                self.capabilities_report.has_adjustable_speed = False


        self.capabilities_report.has_homing = capabilities_dict['has_homing']
        # Homing setup
        if self.capabilities_report.has_homing == True:
            self.goHomeCb = goHomeCb
            self.setHomePositionCb = setHomePositionCb
            self.setHomePositionHereCb = setHomePositionHereCb
        
            if self.goHomeCb is None and self.capabilities_report.has_absolute_positioning == False:
                self.msg_if.pub_warn("Inconsistent capabilities: homing reports true, but no goHome callback provided and no absolute positioning", log_name_list = self.log_name_list)
                self.capabilities_report.has_homing = False
                
            self.home_yaw_deg = 0.0
            self.home_pitch_deg = 0.0

        self.has_homing = self.capabilities_report.has_homing
                

        self.capabilities_report.has_waypoints = capabilities_dict['has_waypoints']
        # Waypoint setup
        if self.capabilities_report.has_waypoints == True:
            self.gotoWaypointCb = gotoWaypointCb
            self.setWaypointCb = setWaypointCb
            self.setWaypointHereCb = setWaypointHereCb

            if self.gotoWaypointCb is None:
                self.msg_if.pub_warn("Inconsistent capabilities: waypoints reports true, but no gotoWaypoint callback provided", log_name_list = self.log_name_list)
                self.capabilities_report.has_waypoints = False

        self.has_waypoints = self.capabilities_report.has_waypoints

        # Set up status message static values
        self.status_msg = PTXStatus()
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version


        ##################################################
        ### Node Class Setup

        self.msg_if.pub_info("Starting Node IF Initialization", log_name_list = self.log_name_list)
        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace':  self.node_namespace
        }


        # Params Config Dict ####################

        self.PARAMS_DICT = {
            'has_absolute_positioning': {
                'namespace': self.node_namespace,
                'factory_val': capabilities_dict['has_absolute_positioning']
            },
            'has_speed_control': {
                'namespace': self.node_namespace,
                'factory_val': capabilities_dict['has_speed_control']
            },
            'speed_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['speed_ratio']
            },
            'has_homing': {
                'namespace': self.node_namespace,
                'factory_val': capabilities_dict['has_homing']
            },
            'has_waypoints': {
                'namespace': self.node_namespace,
                'factory_val': capabilities_dict['has_waypoints']
            },

            'home_position/yaw_deg': {
                'namespace': self.node_namespace,
                'factory_val': 0.0
            },
            'home_position/pitch_deg': {
                'namespace': self.node_namespace,
                'factory_val': 0.0
            }, 
            'frame_id': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['frame_id']
            },            
            'yaw_joint_name': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['yaw_joint_name']
            },            
            'pitch_joint_name': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['pitch_joint_name']
            },            
            'reverse_yaw_control': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['reverse_yaw_control']
            },            
            'reverse_pitch_control': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['reverse_pitch_control']
            },            
            'max_yaw_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_yaw_hardstop_deg']
            },            
            'min_yaw_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_yaw_hardstop_deg']
            },            
            'max_pitch_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_pitch_hardstop_deg']
            },            
            'min_pitch_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_pitch_hardstop_deg']
            },            
            'max_yaw_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_yaw_softstop_deg']
            },            
            'min_yaw_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_yaw_softstop_deg']
            },            
            'max_pitch_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_pitch_softstop_deg']
            },           
            'min_pitch_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_pitch_softstop_deg']
            },           
            'auto_pan_enabled': {
                'namespace': self.node_namespace,
                'factory_val': False
            },           
            'min_auto_pan_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_yaw_softstop_deg']
            },           
            'max_auto_pan_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_yaw_softstop_deg']
            },           
            'auto_tilt_enabled': {
                'namespace': self.node_namespace,
                'factory_val': False
            },           
            'min_auto_tilt_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_pitch_softstop_deg']
            },           
            'max_auto_tilt_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_pitch_softstop_deg']
            },        
        }
        

        # Services Config Dict ####################

        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.node_namespace,
                'topic': 'ptx/capabilities_query',
                'srv': PTXCapabilitiesQuery,
                'req': PTXCapabilitiesQueryRequest(),
                'resp': PTXCapabilitiesQueryResponse(),
                'callback': self.provideCapabilities
            }
        }


        self.PUBS_DICT = {
            'joint_pub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/joint_states',
                'msg': JointState,
                'qsize': 10,
                'latch': False
            },
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'ptx/status',
                'msg': PTXStatus,
                'qsize': 10,
                'latch': False
            }
        }



        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'speed_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setSpeedRatioHandler, 
                'callback_args': ()
            },
            'stop_moving': {
                'namespace': self.node_namespace,
                'topic': 'ptx/stop_moving',
                'msg': Empty,
                'qsize': 1,
                'callback': self.stopMovingHandler, 
                'callback_args': ()
            },
            'goto_to_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_to_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self.gotoPositionHandler, 
                'callback_args': ()
            },
            'goto_to_pan_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_pan_to_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoPanPositionHandler, 
                'callback_args': ()
            },
            'goto_to_tilt_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_tilt_to_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoTiltPositionHandler, 
                'callback_args': ()
            },
            'jog_to_yaw_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_to_yaw_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.jogToYawRatioHandler, 
                'callback_args': ()
            },
            'jog_to_pitch_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_to_pitch_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.jogToPitchRatioHandler, 
                'callback_args': ()
            },
            'jog_timed_yaw': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_timed_yaw',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self.jogTimedYawHandler, 
                'callback_args': ()
            },
            'jog_timed_pitch': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_timed_pitch',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self.jogTimedPitchHandler, 
                'callback_args': ()
            },
            'reverse_yaw_control': {
                'namespace': self.node_namespace,
                'topic': 'ptx/reverse_yaw_control',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setReverseYawControl, 
                'callback_args': ()
            },
            'reverse_pitch_control': {
                'namespace': self.node_namespace,
                'topic': 'ptx/reverse_pitch_control',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setReversePitchControl, 
                'callback_args': ()
            },
            'set_soft_limits': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_soft_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
                'callback': self.setSoftstopHandler, 
                'callback_args': ()
            },
            'go_home': {
                'namespace': self.node_namespace,
                'topic': 'ptx/go_home',
                'msg': Empty,
                'qsize': 1,
                'callback': self.goHomeHandler, 
                'callback_args': ()
            },
            'set_home_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_home_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self.setHomePositionHandler, 
                'callback_args': ()
            },
            'set_home_position_here': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_home_position_here',
                'msg': Empty,
                'qsize': 1,
                'callback': self.setHomePositionHereHandler, 
                'callback_args': ()
            },
            'goto_waypoint': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_waypoint',
                'msg': UInt8,
                'qsize': 1,
                'callback': self.gotoWaypointHandler, 
                'callback_args': ()
            },
            'set_waypoint': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_waypoint',
                'msg': AbsolutePanTiltWaypoint,
                'qsize': 1,
                'callback': self.setWaypointHandler, 
                'callback_args': ()
            },
            'set_waypoint_here': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_waypoint_here',
                'msg': UInt8,
                'qsize': 1,
                'callback': self.setWaypointHereHandler, 
                'callback_args': ()
            },
            'set_auto_pan': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_auto_pan_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setAutoPanHandler, 
                'callback_args': ()
            },
            'set_auto_pan_window': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_auto_pan_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self.setAutoPanWindowHandler, 
                'callback_args': ()
            },
            'set_auto_tilt': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_auto_tilt_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setAutoTiltHandler, 
                'callback_args': ()
            },
            'set_auto_tilt_window': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_auto_tilt_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self.setAutoTiltWindowHandler, 
                'callback_args': ()
            }
        }
        
        


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CFGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )

        ready = self.node_if.wait_for_ready()


        # Setup Settings IF Class ####################
        self.msg_if.pub_info("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = nepi_ros.create_namespace(self.node_namespace,'ptx')

        self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction, 
                    'namespace':  settings_ns
        }
        self.settings_if = SettingsIF(self.SETTINGS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )


        # Setup System IF Classes ####################
        if getOrientationCb is not None:
            self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
            factory_data_rates = {}
            for d in self.data_products_list:
                factory_data_rates[d] = [10.0, 0.0, 100.0] # Default to 10Hz save rate, set last save = 0.0, max rate = 100.0Hz

            factory_filename_dict = {
                'prefix': "", 
                'add_timestamp': True, 
                'add_ms': True,
                'add_us': False,
                'suffix': "",
                'add_node_name': True
                }
                
            sd_namespace = nepi_ros.create_namespace(self.node_namespace,'ptx')
            self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict,
                                    namespace = sd_namespace,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )


            time.sleep(1)


            # Create a NavPose Device IF
            self.capSettingsNavPose = capSettingsNavPose
            self.factorySettingsNavPose=factorySettingsNavPose
            self.settingUpdateFunctionNavPose=settingUpdateFunctionNavPose 
            self.getSettingsFunctionNavPose=getSettingsFunctionNavPose

            self.getHeadingCb = getHeadingCb  
            self.getPositionCb = getPositionCb
            self.getOrientationCb = getOrientationCb
            self.getLocationCb = getLocationCb
            self.getAltitudeCb = getAltitudeCb
            self.getDepthCb = getDepthCb
            self.max_navpose_update_rate = max_navpose_update_rate

            has_navpose = ( getHeadingCb is not None or \
            getPositionCb is not None or \
            getOrientationCb is not None or \
            getLocationCb is not None or \
            getAltitudeCb is not None or \
            getDepthCb is not None )
            
            if has_navpose == True:
                self.msg_if.pub_warn("Starting NPX Device IF Initialization", log_name_list = self.log_name_list)
                self.npx_if = NPXDeviceIF(device_info, 
                    capSettings = self.capSettingsNavPose,
                    factorySettings = self.factorySettingsNavPose,
                    settingUpdateFunction = self.settingUpdateFunctionNavPose, 
                    getSettingsFunction = self.getSettingsFunctionNavPose,
                    getHeadingCb = self.getHeadingCb, 
                    getPositionCb = self.getPositionCb, 
                    getOrientationCb = self.getOrientationCb,
                    getLocationCb = self.getLocationCb, 
                    getAltitudeCb = self.getAltitudeCb, 
                    getDepthCb = self.getDepthCb,
                    max_navpose_update_rate = self.max_navpose_update_rate,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )


        time.sleep(1)
   
        ###############################
        # Finish Initialization
        self.max_yaw_softstop_deg = self.node_if.get_param('max_yaw_softstop_deg')
        self.min_yaw_softstop_deg =  self.node_if.get_param('min_yaw_softstop_deg')

        self.max_pitch_softstop_deg = self.node_if.get_param('max_pitch_softstop_deg')
        self.min_pitch_softstop_deg = self.node_if.get_param('min_pitch_softstop_deg')

        if self.setSoftLimitsCb is not None:
            self.setSoftLimitsCb(self.max_yaw_softstop_deg,
                                self.min_yaw_softstop_deg,
                                self.max_pitch_softstop_deg,
                                self.min_pitch_softstop_deg)

        if self.capabilities_report.has_adjustable_speed == False:
            if self.setSpeedRatioCb is not None and self.getSpeedRatioCb is not None:
                speed_ratio = self.node_if.get_param('speed_ratio')
                self.speed_ratio = speed_ratio
                self.setSpeedRatioCb(speed_ratio)

                
        self.home_yaw_deg = self.node_if.get_param('home_position/yaw_deg')
        self.home_pitch_deg = self.node_if.get_param('home_position/pitch_deg')



        self.frame_id = self.node_if.get_param('frame_id')
        self.yaw_joint_name = self.node_if.get_param('yaw_joint_name')
        self.pitch_joint_name = self.node_if.get_param('pitch_joint_name')
        self.reverse_yaw_control = self.node_if.get_param('reverse_yaw_control')
        self.reverse_pitch_control = self.node_if.get_param('reverse_pitch_control')
        self.msg_if.pub_debug("Factory Controls Dict: " + str(self.factory_controls_dict))
        self.msg_if.pub_debug("reverse_yaw_control: " + str(self.reverse_yaw_control))
        # set class reverse int values
        ryi = 1
        if self.reverse_yaw_control:
            ryi = -1
        self.ryi = ryi
        rpi = 1
        if self.reverse_pitch_control:
            rpi = -1
        self.rpi = rpi
        

        self.status_msg.header.frame_id = self.frame_id


        # And joint state status static values
        self.joint_state_msg = JointState()
        # Skip the header -- we just copy it from the status message each time
        self.joint_state_msg.name = (self.yaw_joint_name, self.pitch_joint_name)
        self.joint_state_msg.position.append(0.0) # Yaw
        self.joint_state_msg.position.append(0.0) # Pitch

        # And odom static values
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.device_name + '_fixed_frame'
        self.odom_msg.child_frame_id = self.device_name + '_rotating_frame'
        
             




        self.initCb(do_updates = True)
        self.publish_status(do_updates = True)



        # Periodic publishing
        status_update_rate = self.position_update_rate  
        if status_update_rate > 10:
            status_update_rate = 10
        self.status_update_rate = status_update_rate
        self.msg_if.pub_info("Starting pt status publisher at hz: " + str(self.status_update_rate))    
        status_pub_delay = float(1.0) / self.status_update_rate
        self.msg_if.pub_info("Starting pt status publisher at sec delay: " + str(status_pub_delay))
        nepi_ros.start_timer_process(status_pub_delay, self.publishJointStateAndStatus, oneshot = True)

        '''
        # Start Auto Pan and Tilt Processes
        if self.gotoPanPositionCb is not None:
            nepi_ros.start_timer_process(1.0, self.autoPanProcess, oneshot = True)
        if self.gotoTiltPositionCb is not None:
            nepi_ros.start_timer_process(1.0, self.autoTiltProcess, oneshot = True)
        '''

        self.publish_status()
        self.ready = True
        self.msg_if.pub_info("Initialization Complete", log_name_list = self.log_name_list)



    ###############################
    # Class Methods

    def gotoPanPositionCurTiltCb(self, pan_deg):
        if self.gotoPositionCb is not None and self.getPositionCb is not None:
            [cur_pan,cur_tilt] = self.getPositionCb()
            self.gotPositionCb(pan_deg,cur_tilt)
            

    def autoPanProcess(self):
        pass
        '''
        if self.auto_pan == True:
            self.msg_if.pub_warn("Starting Pan Scan Process") 
            self.is_auto_pan = True
            if was_scanning == False:
                self.start_scanning = True
            self.publish_status()
            scan_speed_ratio = self.status_msg.scan_speed_ratio
            scan_tilt_offset = self.status_msg.scan_tilt_offset
            

            # Check tilt limits
            if scan_tilt_offset < min_tilt:
            scan_tilt_offset = min_tilt
            if scan_tilt_offset > max_tilt:
            scan_tilt_offset = max_tilt
            self.node_if.set_param("scan_tilt_offset",scan_tilt_offset)


            if self.has_adjustable_speed == True and self.cur_speed_ratio != scan_speed_ratio:
            try:
                self.set_pt_speed_ratio_pub.publish(scan_speed_ratio)
            except:
                pass
 
            if self.has_position_feedback == True:
            if (pan_cur < (min_pan + 10)):
                pan_tilt_pos_msg = PanTiltPosition()
                pan_tilt_pos_msg.yaw_deg = max_pan
                pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
                self.msg_if.pub_warn("Scanning to pan tilt : " + str(pan_tilt_pos_msg))
                self.last_scan_goal = pan_tilt_pos_msg
                try:
                self.set_pt_position_pub.publish(pan_tilt_pos_msg)
                self.pan_tilt_goal_deg = [pan_tilt_pos_msg.yaw_deg,pan_tilt_pos_msg.pitch_deg]
                self.current_scan_dir = 1
                self.publish_status(do_updates = False)
                except Exception as e:
                self.msg_if.pub_warn("Scanning to max_pan excpetion: " + str(e))
                #self.msg_if.pub_warn("Scanning to max_pan")
                self.start_scanning = False

            elif (pan_cur > (max_pan - 10)):
                pan_tilt_pos_msg = PanTiltPosition()
                pan_tilt_pos_msg.yaw_deg = min_pan
                pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
                self.msg_if.pub_warn("Scanning to pan tilt : " + str(pan_tilt_pos_msg))
                self.last_scan_goal = pan_tilt_pos_msg
                try:
                self.set_pt_position_pub.publish(pan_tilt_pos_msg)
                self.pan_tilt_goal_deg = [pan_tilt_pos_msg.yaw_deg,pan_tilt_pos_msg.pitch_deg]
                self.current_scan_dir = -1
                self.publish_status(do_updates = False)
                except:
                self.msg_if.pub_warn("Scanning to min_pan excpetion: " + str(e))
                #self.msg_if.pub_warn("Scanning to min_pan")
                self.start_scanning = False

            elif self.start_scanning == True:
                if self.current_scan_dir > 0:
                pan_tilt_pos_msg = PanTiltPosition()
                pan_tilt_pos_msg.yaw_deg = max_pan
                pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
                self.msg_if.pub_warn("Starting Scan to pan tilt : " + str(pan_tilt_pos_msg))
                self.last_scan_goal = pan_tilt_pos_msg
                try:
                    self.set_pt_position_pub.publish(pan_tilt_pos_msg)
                    self.pan_tilt_goal_deg = [pan_tilt_pos_msg.yaw_deg,pan_tilt_pos_msg.pitch_deg]
                    self.publish_status(do_updates = False)
                except:
                    self.msg_if.pub_warn("Starting Scan to max_pan excpetion: " + str(e))
                #self.msg_if.pub_warn("Starting to max_pan")
                self.start_scanning = False
                else:
                pan_tilt_pos_msg = PanTiltPosition()
                pan_tilt_pos_msg.yaw_deg = min_pan
                pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
                self.msg_if.pub_warn("Starting Scan to pan tilt : " + str(pan_tilt_pos_msg))
                self.last_scan_goal = pan_tilt_pos_msg
                try:
                    self.set_pt_position_pub.publish(pan_tilt_pos_msg)
                    self.pan_tilt_goal_deg = [pan_tilt_pos_msg.yaw_deg,pan_tilt_pos_msg.pitch_deg]
                    self.publish_status(do_updates = False)
                except:
                    self.msg_if.pub_warn("Starting Scan to min_pan excpetion: " + str(e))
                self.msg_if.pub_warn("Starting to min_pan")
                self.start_scanning = False
            else:
                self.msg_if.pub_warn("Cont Scan to pan tilt : " + str(self.last_scan_goal))
                pan_tilt_pos_msg = self.last_scan_goal
                pan_tilt_pos_msg.pitch_deg = scan_tilt_offset
                try:
                self.set_pt_position_pub.publish(pan_tilt_pos_msg)
                self.pan_tilt_goal_deg = [pan_tilt_pos_msg.yaw_deg,pan_tilt_pos_msg.pitch_deg]
                self.current_scan_dir = 1
                self.publish_status(do_updates = False)
                except Exception as e:
                self.msg_if.pub_warn("Cont to max_pan excpetion: " + str(e))
        self.msg_if.pub_warn("Ending Scan Process") 
        '''

    def check_ready(self):
        return self.ready  

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_ros.get_time()
            while self.ready == False and timer < timeout and not nepi_ros.is_shutdown():
                nepi_ros.sleep(.1)
                timer = nepi_ros.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready   


    def yawRatioToDeg(self, ratio):
        yaw_deg = 0
        if self.reverse_yaw_control == False:
           yaw_deg =  self.min_yaw_softstop_deg + (1-ratio) * (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
        else:
           yaw_deg =  self.max_yaw_softstop_deg - (1-ratio)  * (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
        return  yaw_deg
    
    def yawDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse_yaw_control == False:
          ratio = 1 - (deg - self.min_yaw_softstop_deg) / (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
        else:
          ratio = (deg - self.min_yaw_softstop_deg) / (self.max_yaw_softstop_deg - self.min_yaw_softstop_deg)
        return (ratio)
     
    def pitchDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse_pitch_control == False:
          ratio = 1 - (deg - self.min_pitch_softstop_deg) / (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
        else:
          ratio = (deg - self.min_pitch_softstop_deg) / (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
        return ratio
    
    def pitchRatioToDeg(self, ratio):
        pitch_deg = 0
        if self.reverse_pitch_control == False:
           pitch_deg =  self.min_pitch_softstop_deg + (1-ratio) * (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
        else:
           pitch_deg =  (self.max_pitch_softstop_deg) - (1-ratio) * (self.max_pitch_softstop_deg - self.min_pitch_softstop_deg)
        return  pitch_deg

    def publishJointStateAndStatus(self, Timer):
        self.msg_if.pub_debug("Publishing status", log_name_list = self.log_name_list)
        pub_time = self.publish_status()
        pub_time = round(pub_time, 3)
        self.msg_if.pub_debug("Published status with process time: " + str(pub_time))

        status_pub_delay = float(1.0) / self.status_update_rate
        if pub_time > status_pub_delay:
            status_pub_delay = 0.01
        else:
            status_pub_delay = status_pub_delay - pub_time
        nepi_ros.start_timer_process(status_pub_delay, self.publishJointStateAndStatus, oneshot = True)

    def publish_status(self, do_updates = False):
        start_time = nepi_utils.get_time()
        self.status_msg.header.stamp = nepi_ros.ros_time_now()
        #self.msg_if.pub_info("Entering Publish Status", log_name_list = self.log_name_list)
        if self.capabilities_report.has_absolute_positioning == True and self.npx_if is not None:
            navpose_dict = self.npx_if.get_navpose_dict()
            #self.msg_if.pub_info("Got Nav Pose Dict: " + str(navpose_dict) , log_name_list = self.log_name_list)
            yaw_now_deg = navpose_dict['yaw_deg']  * self.ryi
            pitch_now_deg = navpose_dict['pitch_deg']  * self.rpi
            got_time = nepi_utils.get_time() - start_time
            got_time = round(got_time, 3)
            #self.msg_if.pub_debug("Got PT status with time: " + str(got_time))

            if round(yaw_now_deg,5) != round(self.yaw_goal_deg,5) or round(pitch_now_deg,5) != round(self.pitch_goal_deg,5):
                self.status_msg.yaw_now_deg = yaw_now_deg
                self.status_msg.pitch_now_deg = pitch_now_deg

                '''
                if self.yaw_goal_deg == -999:
                    self.yaw_goal_deg = yaw_now_deg
                if self.pitch_goal_deg == -999:
                    self.pitch_goal_deg = pitch_now_deg
                '''
                self.status_msg.reverse_yaw_control = self.reverse_yaw_control
                max_yaw_hs = self.max_yaw_hardstop_deg
                min_yaw_hs = self.min_yaw_hardstop_deg
                max_yaw_ss = self.max_yaw_softstop_deg
                min_yaw_ss = self.min_yaw_softstop_deg
                if self.reverse_yaw_control:
                    max_yaw_hardstop_deg = -1*min_yaw_hs
                    min_yaw_hardstop_deg = -1*max_yaw_hs
                    max_yaw_softstop_deg = -1*min_yaw_ss
                    min_yaw_softstop_deg = -1*max_yaw_ss
                else:
                    max_yaw_hardstop_deg = max_yaw_hs
                    min_yaw_hardstop_deg = min_yaw_hs
                    max_yaw_softstop_deg = max_yaw_ss
                    min_yaw_softstop_deg = min_yaw_ss
                self.status_msg.yaw_max_hardstop_deg = max_yaw_hardstop_deg
                self.status_msg.yaw_min_hardstop_deg = min_yaw_hardstop_deg
                self.status_msg.yaw_max_softstop_deg = max_yaw_softstop_deg
                self.status_msg.yaw_min_softstop_deg = min_yaw_softstop_deg
                self.status_msg.yaw_goal_deg = self.yaw_goal_deg * self.ryi
                self.status_msg.yaw_home_pos_deg = self.home_yaw_deg * self.ryi

                yaw_now_ratio =  1 - (yaw_now_deg - min_yaw_softstop_deg) / (max_yaw_softstop_deg - min_yaw_softstop_deg) 
                self.msg_if.pub_debug("yaw_now, min_yaw, max_yaw, yaw_now_ratio: " + str([yaw_now_deg,min_yaw_softstop_deg,max_yaw_softstop_deg,yaw_now_ratio]))
                if yaw_now_ratio < 0:
                    yaw_now_ratio = 0
                elif yaw_now_ratio > 1:
                    yaw_now_ratio = 1
                self.status_msg.yaw_now_ratio = yaw_now_ratio 
                yaw_goal_ratio =  1 - ((self.status_msg.yaw_goal_deg) - min_yaw_softstop_deg) / (max_yaw_softstop_deg - min_yaw_softstop_deg) 
                self.msg_if.pub_debug("yaw_now, min_yaw, max_yaw, yaw_goal_ratio: " + str([yaw_now_deg,min_yaw_softstop_deg,max_yaw_softstop_deg,yaw_goal_ratio]))
                if yaw_goal_ratio < 0:
                    yaw_goal_ratio = 0
                elif yaw_goal_ratio > 1:
                    yaw_goal_ratio = 1
                self.status_msg.yaw_goal_ratio = yaw_goal_ratio 


                self.status_msg.reverse_pitch_control = self.reverse_pitch_control

                max_pitch_hs = self.max_pitch_hardstop_deg
                min_pitch_hs = self.min_pitch_hardstop_deg
                max_pitch_ss = self.max_pitch_softstop_deg
                min_pitch_ss = self.min_pitch_softstop_deg
                if self.reverse_pitch_control:
                    max_pitch_hardstop_deg = -1*min_pitch_hs
                    min_pitch_hardstop_deg = -1*max_pitch_hs
                    max_pitch_softstop_deg = -1*min_pitch_ss
                    min_pitch_softstop_deg = -1*max_pitch_ss
                else:
                    max_pitch_hardstop_deg = max_pitch_hs
                    min_pitch_hardstop_deg = min_pitch_hs
                    max_pitch_softstop_deg = max_pitch_ss
                    min_pitch_softstop_deg = min_pitch_ss

                self.status_msg.pitch_max_hardstop_deg = max_pitch_hardstop_deg
                self.status_msg.pitch_min_hardstop_deg = min_pitch_hardstop_deg
                self.status_msg.pitch_max_softstop_deg = max_pitch_softstop_deg
                self.status_msg.pitch_min_softstop_deg = min_pitch_softstop_deg

                self.status_msg.pitch_goal_deg = self.pitch_goal_deg * self.rpi
                self.status_msg.pitch_home_pos_deg = self.home_pitch_deg * self.rpi

                pitch_now_ratio =  1 - ((pitch_now_deg * self.rpi) - min_pitch_softstop_deg) / (max_pitch_softstop_deg - min_pitch_softstop_deg) 
                self.msg_if.pub_debug("pitch_now, min_pitch, max_pitch,pitch_now_ratio: " + str([pitch_now_deg,min_pitch_softstop_deg,max_pitch_softstop_deg,pitch_now_ratio]))
                if pitch_now_ratio < 0:
                    pitch_now_ratio = 0
                elif pitch_now_ratio > 1:
                    pitch_now_ratio = 1
                self.status_msg.pitch_now_ratio = pitch_now_ratio 
                pitch_goal_ratio =  1 - (self.status_msg.pitch_goal_deg - min_pitch_softstop_deg) / (max_pitch_softstop_deg - min_pitch_softstop_deg) 
                self.msg_if.pub_debug("pitch_now, min_pitch, max_pitch, pitch_goal_ratio: " + str([pitch_now_deg,min_pitch_softstop_deg,max_pitch_softstop_deg,pitch_goal_ratio]))
                if pitch_goal_ratio < 0:
                    pitch_goal_ratio = 0
                elif pitch_goal_ratio > 1:
                    pitch_goal_ratio = 1
                self.status_msg.pitch_goal_ratio = pitch_goal_ratio 

                self.status_msg.speed_ratio = self.speed_ratio


        if do_updates == True:
            if self.getSoftLimitsCb is not None:
                [min_yaw,max_yaw,min_pitch,max_pitch] = self.getSoftLimitsCb()
                if min_yaw != -999:
                    self.min_yaw_softstop_deg = min_yaw
                    self.node_if.set_param('min_yaw_softstop_deg', self.min_yaw_softstop_deg)
                if max_yaw != -999:
                    self.max_yaw_softstop_deg = max_yaw
                    self.node_if.set_param('max_yaw_softstop_deg',self.max_yaw_softstop_deg)

                if min_pitch != -999:
                    self.min_pitch_softstop_deg = min_pitch
                    self.node_if.set_param('min_pitch_softstop_deg', self.min_pitch_softstop_deg)
                if max_pitch != -999:
                    self.max_pitch_softstop_deg = max_pitch
                    self.node_if.set_param('max_pitch_softstop_deg',self.max_pitch_softstop_deg)

            if self.capabilities_report.has_adjustable_speed == True:
                self.status_msg.speed_ratio = self.getSpeedRatioCb()
                self.speed_ratio = self.status_msg.speed_ratio

        self.status_msg.has_position_feedback = self.has_position_feedback
        self.status_msg.has_adjustable_speed = self.has_adjustable_speed
        self.status_msg.has_auto_pan = self.has_auto_pan
        self.status_msg.has_auto_tilt = self.has_auto_tilt
        self.status_msg.has_homing = self.has_homing
        self.status_msg.has_waypoints = self.has_waypoints

        self.msg_if.pub_debug("Publishing Status", log_name_list = self.log_name_list)

        self.node_if.publish_pub('status_pub',self.status_msg)


        yaw_rad = 0.01745329 * self.status_msg.yaw_now_deg
        pitch_rad = 0.01745329 * self.status_msg.pitch_now_deg

        # And joint state if appropriate
        self.joint_state_msg.header.stamp = self.status_msg.header.stamp
        self.joint_state_msg.position[0] = yaw_rad
        self.joint_state_msg.position[1] = pitch_rad
        self.msg_if.pub_debug("Publishing Joint", log_name_list = self.log_name_list)
        self.node_if.publish_pub('joint_pub',self.joint_state_msg)

        pub_time = nepi_utils.get_time() - start_time


        return pub_time


    def positionWithinSoftLimits(self, yaw_deg, pitch_deg):

        if (yaw_deg < self.min_yaw_softstop_deg) or (yaw_deg > self.max_yaw_softstop_deg) or \
           (pitch_deg < self.min_pitch_softstop_deg) or (pitch_deg > self.max_pitch_softstop_deg):
            return False
        
        return True

    '''
    def setHardstopHandler(self, msg):
        min_yaw = msg.min_yaw_deg
        max_yaw = msg.max_yaw_deg
        min_pitch = msg.min_pitch_deg
        max_pitch = msg.max_pitch_deg

        valid = False
        if min_yaw < max_yaw and min_pitch < max_pitch:
            if min_yaw >= self.factoryLimits['min_yaw_hardstop_deg'] and max_yaw <= self.factoryLimits['max_yaw_hardstop_deg']:
                if min_pitch >= self.factoryLimits['min_pitch_hardstop_deg'] and max_pitch <= self.factoryLimits['max_pitch_hardstop_deg']:

                    self.node_if.set_param('max_yaw_hardstop_deg', max_yaw)
                    self.node_if.set_param('min_yaw_hardstop_deg', min_yaw)
                    self.node_if.set_param('max_pitch_hardstop_deg', max_pitch)
                    self.node_if.set_param('min_pitch_hardstop_deg', min_pitch)

                    self.node_if.set_param('max_yaw_softstop_deg', max_yaw)
                    self.node_if.set_param('min_yaw_softstop_deg', min_yaw)
                    self.node_if.set_param('max_pitch_softstop_deg', max_pitch)
                    self.node_if.set_param('min_pitch_softstop_deg', min_pitch)
                    valid = True

                    self.initCb(do_updates = True)


        if valid == False:
            self.msg_if.pub_warn("Invalid hardstop requested " + str(msg))
    '''

    def setSoftstopHandler(self, msg):
        min_yaw = msg.min_yaw_deg
        max_yaw = msg.max_yaw_deg
        min_pitch = msg.min_pitch_deg
        max_pitch = msg.max_pitch_deg

        max_yaw_hardstop_deg = self.node_if.get_param('max_yaw_hardstop_deg')
        min_yaw_hardstop_deg = self.node_if.get_param('min_yaw_hardstop_deg')
        max_pitch_hardstop_deg = self.node_if.get_param('max_pitch_hardstop_deg')
        min_pitch_hardstop_deg = self.node_if.get_param('min_pitch_hardstop_deg')

        valid = False
        if min_yaw < max_yaw and max_yaw <= max_yaw_hardstop_deg and min_yaw < max_yaw:  
            if min_pitch >= min_pitch_hardstop_deg and max_pitch <= max_pitch_hardstop_deg and min_pitch < max_pitch:
                self.node_if.set_param('max_yaw_softstop_deg', max_yaw)
                self.node_if.set_param('min_yaw_softstop_deg', min_yaw)
                self.node_if.set_param('max_pitch_softstop_deg', max_pitch)
                self.node_if.set_param('min_pitch_softstop_deg', min_pitch)
                valid = True


                if self.setSoftLimitsCb is not None:
                    self.setSoftLimitsCb(min_yaw,max_yaw,min_pitch,max_pitch)
                self.initCb(do_updates = True)


        if valid == False:
            self.msg_if.pub_warn("Invalid softstop requested " + str(msg))
        

   
    def setSpeedRatioHandler(self, msg):
        if self.capabilities_report.has_adjustable_speed == True:
            speed_cur = self.getSpeedRatioCb()
            speed_ratio = msg.data
            if (speed_ratio < 0.0) or (speed_ratio > 1.0):
                self.msg_if.pub_warn("Invalid speed ratio requested " + "%.2f" % speed_ratio)
            elif speed_cur != speed_ratio and self.setSpeedRatioCb is not None:
                self.speed_ratio = speed_ratio
                self.publish_status()
                self.setSpeedRatioCb(speed_ratio)
                self.msg_if.pub_info("Updated speed ratio to " + str(speed_ratio))
        

    def setHomePositionHandler(self, msg):
        if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
            self.msg_if.pub_warn("Requested home position is invalid... ignoring", log_name_list = self.log_name_list)
            return

        if self.setHomePositionCb is not None:
            # Driver supports absolute positioning, so just let it operate
            self.home_yaw_deg = msg.yaw_deg
            self.home_pitch_deg = msg.pitch_deg
            self.setHomePositionCb(self.home_yaw_deg, self.home_pitch_deg)
        else:
            self.msg_if.pub_warn("Absolution position home setpoints not available... ignoring", log_name_list = self.log_name_list)
            return
        
        self.msg_if.pub_info("Updated home position to " + "%.2f" % self.home_yaw_deg + " " + "%.2f" %  self.home_pitch_deg)
            

    def goHomeHandler(self, _):
        if self.auto_pan == False and self.auto_tilt == False:
            if self.goHomeCb is not None:
                self.yaw_goal_deg = self.home_yaw_deg
                self.pitch_goal_deg = self.home_pitch_deg
                self.goHomeCb()
        

    def gotoPositionHandler(self, msg):
        if self.auto_pan == False and self.auto_tilt == False:
            if self.positionWithinSoftLimits is not None:
                if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
                    self.msg_if.pub_warn("Requested goto position is invalid... ignoring", log_name_list = self.log_name_list)
                    return
                self.yaw_goal_deg = msg.yaw_deg
                self.pitch_goal_deg = msg.pitch_deg
                self.msg_if.pub_info("Driving to  " + "%.2f" % (self.yaw_goal_deg * self.ryi) + " " + "%.2f" % (self.pitch_goal_deg * self.rpi))
                self.self.gotoPanPositionCb(yaw_deg = (self.yaw_goal_deg * self.ryi), pitch_deg = (self.pitch_goal_deg * self.rpi))

    def gotoPanPositionHandler(self, msg):
        yaw_deg = msg.data
        pitch_deg = self.status_msg.pitch_now_deg
        if self.auto_pan == False:
            if self.positionWithinSoftLimits is not None:
                if not self.positionWithinSoftLimits(yaw_deg, pitch_deg):
                    self.msg_if.pub_warn("Requested goto pan position is invalid... ignoring", log_name_list = self.log_name_list)
                    return
                self.yaw_goal_deg = msg.yaw_deg
                self.pitch_goal_deg = msg.pitch_deg
                if self.gotoPanPositionCb is not None:
                    self.msg_if.pub_info("Driving to yaw " + "%.2f" % (self.yaw_goal_deg * self.ryi))
                    self.self.gotoPanPositionCb(yaw_deg = (self.yaw_goal_deg * self.ryi))
                elif self.gotoPanPositionCb is not None:
                    self.msg_if.pub_info("Driving to  " + "%.2f" % (self.yaw_goal_deg * self.ryi) + " " + "%.2f" % (self.pitch_goal_deg * self.rpi))
                    self.self.gotoPanPositionCb(yaw_deg = (self.yaw_goal_deg * self.ryi), pitch_deg = (self.pitch_goal_deg * self.rpi))                    

    def gotoTiltPositionHandler(self, msg):
        yaw_deg = self.status_msg.yaw_now_deg 
        pitch_deg = msg.data
        if self.auto_pan == False:
            if self.positionWithinSoftLimits is not None:
                if not self.positionWithinSoftLimits(yaw_deg, pitch_deg):
                    self.msg_if.pub_warn("Requested goto tilt position is invalid... ignoring", log_name_list = self.log_name_list)
                    return
                self.yaw_goal_deg = msg.yaw_deg
                self.pitch_goal_deg = msg.pitch_deg
                if self.gotoTiltPositionCb is not None:
                    self.msg_if.pub_info("Driving to  " + "%.2f" % (self.pitch_goal_deg * self.rpi))
                    self.self.gotoTiltPositionCb(pitch_deg = (self.pitch_goal_deg * self.rpi))    
                elif self.gotoPanPositionCb is not None:
                    self.msg_if.pub_info("Driving to  " + "%.2f" % (self.yaw_goal_deg * self.ryi) + " " + "%.2f" % (self.pitch_goal_deg * self.rpi))
                    self.self.gotoPanPositionCb(yaw_deg = (self.yaw_goal_deg * self.ryi), pitch_deg = (self.pitch_goal_deg * self.rpi))      

    def jogToYawRatioHandler(self, msg):
        if self.auto_pan == False:
            ratio = msg.data
            if (ratio < 0.0 or ratio > 1.0):
                self.msg_if.pub_warn("Invalid yaw position ratio " + "%.2f" % ratio)
                return
            if self.gotoPositionCb is not None:
                self.yaw_goal_deg = self.yawRatioToDeg(ratio)
                pitch_now_deg = self.status_msg.pitch_now_deg
                self.msg_if.pub_info("Driving to  " + "%.2f" % self.yaw_goal_deg + " " + "%.2f" % pitch_now_deg)
                self.gotoPositionCb(yaw_deg = self.yaw_goal_deg, pitch_deg = pitch_now_deg)
        

    def jogToPitchRatioHandler(self, msg):
        if self.auto_tilt == False:
            ratio = msg.data
            if (ratio < 0.0 or ratio > 1.0):
                self.msg_if.pub_warn("Invalid pitch position ratio " + "%.2f" % ratio)
                return
            if self.gotoPositionCb is not None:
                self.pitch_goal_deg = self.pitchRatioToDeg(ratio)
                yaw_now_deg = self.status_msg.yaw_now_deg
                self.msg_if.pub_info("Driving to  " + "%.2f" % yaw_now_deg + " " + "%.2f" % self.pitch_goal_deg)
                self.gotoPositionCb(yaw_deg = yaw_now_deg, pitch_deg = self.pitch_goal_deg)
        

    def stopMovingHandler(self, _):
        self.auto_pan = False
        self.auto_tilt = False
        if self.stopMovingCb is not None:
            self.stopMovingCb()
            self.yaw_goal_deg = self.status_msg.yaw_now_deg
            self.pitch_goal_deg = self.status_msg.pitch_now_deg
            self.msg_if.pub_info("Stopping motion by request", log_name_list = self.log_name_list)
        

    def jogTimedYawHandler(self, msg):
        if self.auto_pan == False:
            self.msg_if.pub_warn("Got job yaw msg: " + str(msg))
            if self.moveYawCb is not None:
                direction = msg.direction if self.reverse_yaw_control is False else (-1 * msg.direction)
                time_s = msg.duration_s
                self.moveYawCb(direction,  time_s)
                self.msg_if.pub_info("Jogging yaw", log_name_list = self.log_name_list)
        

    def jogTimedPitchHandler(self, msg):
        if self.auto_tilt == False:
            self.msg_if.pub_warn("Got job pitch msg: " + str(msg))
            if self.movePitchCb is not None:
                direction = msg.direction if self.reverse_pitch_control is False else (-1 * msg.direction)
                time_s = msg.duration_s
                self.movePitchCb(direction, time_s)
                self.msg_if.pub_info("Jogging pitch", log_name_list = self.log_name_list)
        

    def setReverseYawControl(self, msg):
        self.reverse_yaw_control = msg.data
        ryi = 1
        if msg.data == True:
            ryi = -1
        self.ryi = ryi
        self.msg_if.pub_info("Set yaw control to reverse=" + str(self.reverse_yaw_control))
        

    def setReversePitchControl(self, msg):
        self.reverse_pitch_control = msg.data
        rpi = 1
        if msg.data == True:
            rpi = -1
        self.rpi = rpi
        self.msg_if.pub_info("Set pitch control to reverse=" + str(self.reverse_pitch_control))
        

    def setHomePositionHereHandler(self, _):
        if self.setHomePositionHereCb is not None:
            # Driver supports it directly
            # Capture home position if possible
            if self.capabilities_report.has_absolute_positioning and self.npx_if is not None:
                navpose_dict = npx_if.get_navpose_dict()
                self.home_yaw_deg = navpose_dict['yaw_deg']
                self.home_pitch_deg = navpose_dict['pitch_deg']
            self.setHomePositionHereCb()
        else:
            self.msg_if.pub_warn("Instant home position not available for this device", log_name_list = self.log_name_list)
            return
        self.msg_if.pub_info("Updated home position to current position", log_name_list = self.log_name_list)
        

    def setWaypointHandler(self, msg):
        if self.positionWithinSoftLimits is not None:
            yaw_deg = msg.yaw_deg
            pitch_deg = msg.pitch_deg
            waypoint_index = msg.waypoint_index
            if not self.positionWithinSoftLimits(msg.yaw_deg, msg.pitch_deg):
                self.msg_if.pub_warn("Requested waypoint position is invalid... ignoring", log_name_list = self.log_name_list)
                return
            if self.setWaypointCb is not None:
                self.setWaypointCb(waypoint_index, yaw_deg, pitch_deg)
        

    def setWaypointHereHandler(self, msg):
        waypoint_index = msg.data
        if self.setWaypointHereCb is not None:
            self.setWaypointHereCb(waypoint_index)
        
    
    def gotoWaypointHandler(self, msg):
        waypoint_index = msg.data
        if self.gotoWaypointCb is not None:
            self.gotoWaypointCb(waypoint_index)
            self.msg_if.pub_info("Going to waypoint by command " + str(waypoint_index))
        
    

    def setAutoPanHandler(self, msg):
        self.auto_pan = msg.data
        if self.auto_pan == False:
            self.is_auto_pan = False
        self.msg_if.pub_info("Setting auto pan: " + str(self.auto_pan))
        self.publish_status()
        self.node_if.set_param('auto_pan_enabled', self.auto_pan)
        
    def setAutoPanWindowHandler(self, msg):
        self.min_deg = msg.start_range
        self.max_deg = msg.stop_range
        if max_deg > min_deg:
            if max_deg > self.max_yaw_softstop_deg:
                max_deg = self.max_yaw_softstop_deg
            if min_deg < self.min_yaw_softstop_deg:
                min_deg = self.min_yaw_softstop_deg
            self.msg_if.pub_info("Setting auto pan limits: " + str([min_deg,max_deg]))
            self.publish_status()
            self.node_if.set_param('min_yaw_softstop_deg', min_deg)
            self.node_if.set_param('max_yaw_softstop_deg', max_deg)


    def setAutoTiltHandler(self, msg):
        self.auto_tilt = msg.data
        if self.auto_tilt == False:
            self.is_auto_tilt = False
        self.msg_if.pub_info("Setting auto tilt: " + str(self.auto_tilt))
        self.publish_status()
        self.node_if.set_param('auto_tilt_enabled', self.auto_tilt)

    def setAutoTiltWindowHandler(self, msg):
        self.min_deg = msg.start_range
        self.max_deg = msg.stop_range
        if max_deg > min_deg:
            if max_deg > self.max_pitch_softstop_deg:
                max_deg = self.max_pitch_softstop_deg
            if min_deg < self.min_pitch_softstop_deg:
                min_deg = self.min_pitch_softstop_deg
            self.msg_if.pub_info("Setting auto tilt limits: " + str([min_deg,max_deg]))
            self.publish_status()
            self.node_if.set_param('min_pitch_softstop_deg', min_deg)
            self.node_if.set_param('max_pitch_softstop_deg', max_deg)



    def provideCapabilities(self, _):
        return self.capabilities_report
    

    def initConfig(self):
        self.initCb(do_updates = True)

    def initCb(self, do_updates = False):
        if do_updates == True:
            self.resetCb()



    def resetCb(self, do_updates = True):
        self.msg_if.pub_warn("Reseting System to Current Values")
        self.max_yaw_hardstop_deg = self.node_if.get_param('max_yaw_hardstop_deg')
        self.min_yaw_hardstop_deg = self.node_if.get_param('min_yaw_hardstop_deg')
        self.max_yaw_softstop_deg = self.node_if.get_param('max_yaw_softstop_deg')
        self.min_yaw_softstop_deg = self.node_if.get_param('min_yaw_softstop_deg')

        self.max_pitch_hardstop_deg = self.node_if.get_param('max_pitch_hardstop_deg')
        self.min_pitch_hardstop_deg = self.node_if.get_param('min_pitch_hardstop_deg')
        self.max_pitch_softstop_deg = self.node_if.get_param('max_pitch_softstop_deg')
        self.min_pitch_softstop_deg = self.node_if.get_param('min_pitch_softstop_deg')

        #**********************
        # This one comes from the parent
        if self.getSoftLimitsCb is not None:
                [min_yaw,max_yaw,min_pitch,max_pitch] = self.getSoftLimitsCb()
                if min_yaw != -999:
                    self.min_yaw_softstop_deg = min_yaw
                    self.node_if.set_param('min_yaw_softstop_deg', self.min_yaw_softstop_deg)
                if max_yaw != -999:
                    self.max_yaw_softstop_deg = max_yaw
                    self.node_if.set_param('max_yaw_softstop_deg',self.max_yaw_softstop_deg)

                if min_pitch != -999:
                    self.min_pitch_softstop_deg = min_pitch
                    self.node_if.set_param('min_pitch_softstop_deg', self.min_pitch_softstop_deg)
                if max_pitch != -999:
                    self.max_pitch_softstop_deg = max_pitch
                    self.node_if.set_param('max_pitch_softstop_deg',self.max_pitch_softstop_deg)

        if self.getSpeedRatioCb is not None:
            self.node_if.set_param('speed_ratio', self.getSpeedRatioCb()) 
        #**********************
        



    def factoryResetCb(self, do_updates = True):
        if self.deviceResetCb is not None:
            self.deviceResetCb()
            nepi_ros.sleep(2)
        self.resetCb()

    def passFunction(self):
        return 0

