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
import math
import numpy as np


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from nepi_sdk_interfaces.msg import RangeWindow
from nepi_sdk_interfaces.msg import PTXStatus, PanTiltLimits, PanTiltOffsets, PanTiltPosition, SingleAxisTimedMove
from nepi_sdk_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryRequest, PTXCapabilitiesQueryResponse

from tf.transformations import quaternion_from_euler

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF
from nepi_api.device_if_npx import NPXDeviceIF

#from nepi_api.device_if_npx import NPXDeviceIF



class PTXActuatorIF:
    MAX_STATUS_UPDATE_RATE = 3

    # Backup Factory Control Values 
    FACTORY_CONTROLS_DICT = {
                'frame_id' : 'ptx_frame',
                'pan_joint_name' : 'ptx_pan_joint',
                'tilt_joint_name' : 'ptx_tilt_joint',
                'reverse_pan_enabled' : False,
                'reverse_tilt_enabled' : False,
                'speed_ratio' : 0.5
    }

    AUTO_SCAN_SWITCH_DEG = 5 # If angle withing this bound, switch dir
    AUTO_SCAN_UPDATE_INTERVAL = 1

    data_products_list = ['orientation']

    orientation_dict = {
        'time_orientation': nepi_utils.get_time(),
        # Orientation should be provided in Degrees ENU
        'roll_deg': 0.0,
        'tilt_deg': 0.0,
        'pan_deg': 0.0,
    }

    ready = False

    status_msg = PTXStatus()
    joint_state_msg = JointState()    

    has_position_feedback = False
    has_absolute_positioning = False
    has_timed_positioning = False
    has_seperate_pan_tilt_control = False
    has_adjustable_limits = False
    has_adjustable_speed = False
    has_auto_pan = False
    has_auto_tilt = False
    has_homing = False
    has_set_home = False


    # Define some member variables
    pan_now_deg = 0.0
    pan_goal_deg = 0.0
    pan_home_pos_deg = 0.0
    min_pan_softstop_deg = 0.0
    max_pan_softstop_deg = 0.0
    tilt_now_deg = 0.0
    tilt_goal_deg = 0.0
    tilt_home_pos_deg = 0.0
    min_tilt_softstop_deg = 0.0
    max_tilt_softstop_deg = 0.0

    home_pan_deg = 0.0
    home_tilt_deg = 0.0

    reverse_pan_enabled = False
    rpi = 1
    reverse_tilt_enabled = False
    rti = 1

    last_pan = 0
    last_tilt = 0

    max_pan_hardstop_deg = 0
    min_pan_hardstop_deg = 0

    max_tilt_hardstop_deg = 0
    min_tilt_hardstop_deg = 0

    max_pan_softstop_deg = 0
    min_pan_softstop_deg = 0

    max_tilt_softstop_deg = 0
    min_tilt_softstop_deg = 0

    current_position = [0.0,0.0]
    last_position = current_position
    speed_ratio = 0.5


    is_auto_pan = False
    start_auto_pan = False
    auto_pan_enabled = False
    auto_pan_min = -10
    auto_pan_max = 10
    auto_pan_sec = 5

    is_auto_tilt = False
    start_auto_tilt = False
    auto_tilt_enabled = False
    auto_tilt_min = -10
    auto_tilt_max = 10
    auto_tilt_sec = 5


    offsets_dict = dict()
    offsets_dict['x'] = 0
    offsets_dict['y'] = 0
    offsets_dict['z'] = 0
    
    is_moving = False
    ### IF Initialization
    def __init__(self,  device_info, 
                 capSettings, factorySettings, 
                 settingUpdateFunction, getSettingsFunction,
                 factoryControls , # Dictionary to be supplied by parent, specific key set is required
                 factoryLimits = None,
                 factoryOffsets = None,
                 stopMovingCb = None, # Required; no args
                 movePanCb = None, # Required; direction and time args
                 moveTiltCb = None, # Required; direction and time args
                 setSoftLimitsCb=None,
                 getSoftLimitsCb=None,
                 setSpeedRatioCb=None, # None ==> No speed adjustment capability; Speed ratio arg
                 getSpeedRatioCb=None, # None ==> No speed adjustment capabilitiy; Returns speed ratio
                 getPositionCb=None,
                 gotoPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 gotoPanPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 gotoTiltPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 goHomeCb=None, # None ==> No native driver homing capability, can still use homing if absolute positioning is supported
                 setHomePositionCb=None, # None ==> No native driver home absolute setting capability, can still use it if absolute positioning is supported
                 setHomePositionHereCb=None, # None ==> No native driver home instant capture capability, can still use it if absolute positioning is supported
                 capSettingsNavPose=None, factorySettingsNavPose=None, 
                 settingUpdateFunctionNavPose=None, getSettingsFunctionNavPose=None,
                 getNpHeadingCb = None, getNpPositionCb = None, getNpOrientationCb = None,
                 getNpLocationCb = None, getNpAltitudeCb = None, getNpDepthCb = None,
                 max_navpose_update_rate = 10,
                 deviceResetCb = None,
                 log_name = None,
                 log_name_list = [],
                 msg_if = None
                ):
        ####  IF INIT SETUP ####
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

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

        # update offset controls dictionary
        if factoryOffsets is not None:
            for key in factoryOffsets.keys():
                if key in self.offsets_dict.keys():
                    self.offsets_dict[key] = factoryOffsets[key]


        self.deviceResetCb = deviceResetCb
        self.factory_device_name = device_info["device_name"] + "_" + device_info["identifier"]


       # Configure PTX Capabilities

        # STOP MOVE #############
        self.stopMovingCb = stopMovingCb


        # GET POSITION #############
        self.getPositionCb = getPositionCb
        if self.getPositionCb is not None:
            self.has_position_feedback = True
            self.has_limit_controls = True

        # Soft Limits are handled by PTX IF at top level, 
        # these are for updating the hardware if required
        self.setSoftLimitsCb = setSoftLimitsCb
        self.getSoftLimitsCb = getSoftLimitsCb

        # POSITION MOVE ############
        self.gotoPositionCb = gotoPositionCb
        self.gotoPanPositionCb = gotoPanPositionCb
        self.gotoTiltPositionCb = gotoTiltPositionCb


        if self.gotoPositionCb is not None or self.gotoPanPositionCb is not None:
            self.has_absolute_positioning = True
    
        if gotoPanPositionCb is not None and gotoTiltPositionCb is not None:
            self.has_seperate_pan_tilt_control = True

        # JOG MOVE ############
        self.movePanCb = movePanCb
        self.moveTiltCb = moveTiltCb
        if self.movePanCb is not None and self.moveTiltCb is not None:
            self.has_timed_positioning = True

        # AUTO SCANNING ##############
        # timed auto scanning is not supported yet
        if self.has_absolute_positioning:
            self.has_auto_pan = True
            self.has_auto_tilt = True

        # SPEED SETTINGS  #############
        if setSpeedRatioCb is None:
            self.getSpeedRatioCb = self.getZeroCb
        else:
            self.setSpeedRatioCb = setSpeedRatioCb
        self.getSpeedRatioCb = getSpeedRatioCb
        if self.getSpeedRatioCb is not None:
            self.has_adjustable_speed = True


        # Homing  #############
        self.goHomeCb = goHomeCb
        self.setHomePositionCb = setHomePositionCb
        self.setHomePositionHereCb = setHomePositionHereCb

        if self.goHomeCb is not None:
            self.has_homing = True
        if self.setHomePositionHereCb is not None:
            self.has_set_home = True
       

        # Create Capabilities Report
        self.capabilities_report = PTXCapabilitiesQueryResponse()
        self.capabilities_report.has_absolute_positioning = self.has_absolute_positioning
        self.capabilities_report.has_timed_positioning = self.has_timed_positioning
        self.capabilities_report.has_seperate_pan_tilt_control = self.has_seperate_pan_tilt_control
        self.capabilities_report.has_position_feedback = self.has_position_feedback
        self.capabilities_report.has_adjustable_speed = self.has_adjustable_speed
        self.capabilities_report.has_limit_controls = self.has_limit_controls
        self.capabilities_report.has_auto_pan = self.has_auto_pan
        self.capabilities_report.has_auto_tilt = self.has_auto_tilt
        self.capabilities_report.has_homing = self.has_homing
        self.capabilities_report.has_set_home = self.has_set_home



        #######################################
        # Set up factory limits

        if factoryLimits is not None:
            self.factoryLimits = factoryLimits  
        else:
            # Hard limits
            self.factoryLimits['min_pan_hardstop_deg'] = 0
            self.factoryLimits['max_pan_hardstop_deg'] = 0
            self.factoryLimits['min_tilt_hardstop_deg'] = 0
            self.factoryLimits['max_tilt_hardstop_deg'] = 0
  
            # Soft limits
            self.factoryLimits['min_pan_softstop_deg'] = 0
            self.factoryLimits['max_pan_softstop_deg'] = 0
            self.factoryLimits['min_tilt_softstop_deg'] = 0
            self.factoryLimits['max_tilt_softstop_deg'] = 0

        self.min_pan_hardstop_deg =  self.factoryLimits['min_pan_hardstop_deg']
        self.max_pan_hardstop_deg = self.factoryLimits['max_pan_hardstop_deg']

        self.min_tilt_hardstop_deg = self.factoryLimits['min_tilt_hardstop_deg']
        self.max_tilt_hardstop_deg = self.factoryLimits['max_tilt_hardstop_deg']


        self.min_pan_softstop_deg =  self.factoryLimits['min_pan_softstop_deg']
        self.max_pan_softstop_deg = self.factoryLimits['max_pan_softstop_deg']


        self.min_tilt_softstop_deg = self.factoryLimits['min_tilt_softstop_deg']
        self.max_tilt_softstop_deg = self.factoryLimits['max_tilt_softstop_deg']


        self.auto_pan_min = self.factoryLimits['min_pan_softstop_deg']
        self.auto_pan_max = self.factoryLimits['max_pan_softstop_deg']

        self.auto_tilt_min = self.factoryLimits['min_tilt_softstop_deg']
        self.auto_tilt_max = self.factoryLimits['max_tilt_softstop_deg']



        ########################
        # Set up status message static values
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version
        self.status_msg.has_absolute_positioning = self.has_absolute_positioning
        self.status_msg.has_timed_positioning = self.has_timed_positioning
        self.status_msg.has_seperate_pan_tilt_control = self.has_seperate_pan_tilt_control
        self.status_msg.has_position_feedback = self.has_position_feedback
        self.status_msg.has_adjustable_speed = self.has_adjustable_speed
        self.status_msg.has_limit_controls = self.has_limit_controls
        self.status_msg.has_auto_pan = self.has_auto_pan
        self.status_msg.has_auto_tilt = self.has_auto_tilt
        self.status_msg.has_homing = self.has_homing
        self.status_msg.has_set_home = self.has_set_home

        # And joint state status static values
        # Skip the header -- we just copy it from the status message each time
        self.joint_state_msg.position.append(0.0) # pan
        self.joint_state_msg.position.append(0.0) # Tilt

        # And odom static values
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.device_name + '_fixed_frame'
        self.odom_msg.child_frame_id = self.device_name + '_rotating_frame'

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
            'speed_ratio': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['speed_ratio']
            },
            'home_position/pan_deg': {
                'namespace': self.node_namespace,
                'factory_val': 0.0
            },
            'home_position/tilt_deg': {
                'namespace': self.node_namespace,
                'factory_val': 0.0
            }, 
            'frame_id': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['frame_id']
            },            
            'pan_joint_name': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['pan_joint_name']
            },            
            'tilt_joint_name': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['tilt_joint_name']
            },            
            'reverse_pan_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['reverse_pan_enabled']
            },            
            'reverse_tilt_enabled': {
                'namespace': self.node_namespace,
                'factory_val': self.factory_controls_dict['reverse_tilt_enabled']
            },            
            'max_pan_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_pan_hardstop_deg']
            },            
            'min_pan_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_pan_hardstop_deg']
            },            
            'max_tilt_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_tilt_hardstop_deg']
            },            
            'min_tilt_hardstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_tilt_hardstop_deg']
            },            
            'max_pan_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_pan_softstop_deg']
            },            
            'min_pan_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_pan_softstop_deg']
            },            
            'max_tilt_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_tilt_softstop_deg']
            },           
            'min_tilt_softstop_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_tilt_softstop_deg']
            },           
            'auto_pan_enabled': {
                'namespace': self.node_namespace,
                'factory_val': False
            },           
            'min_auto_pan_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_pan_softstop_deg']
            },           
            'max_auto_pan_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_pan_softstop_deg']
            },           
            'auto_tilt_enabled': {
                'namespace': self.node_namespace,
                'factory_val': False
            },           
            'min_auto_tilt_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['min_tilt_softstop_deg']
            },           
            'max_auto_tilt_deg': {
                'namespace': self.node_namespace,
                'factory_val': self.factoryLimits['max_tilt_softstop_deg']
            },            
            'offsets': {
                'namespace': self.node_namespace,
                'factory_val': self.offsets_dict
            }
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
                'topic': 'ptx/goto_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self.gotoPositionHandler, 
                'callback_args': ()
            },
            'goto_to_pan_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_pan_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoPanPositionHandler, 
                'callback_args': ()
            },
            'goto_to_tilt_position': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_tilt_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoTiltPositionHandler, 
                'callback_args': ()
            },
            'goto_pan_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_pan_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoToPanRatioHandler, 
                'callback_args': ()
            },
            'goto_tilt_ratio': {
                'namespace': self.node_namespace,
                'topic': 'ptx/goto_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoToTiltRatioHandler, 
                'callback_args': ()
            },
            'jog_timed_pan': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_timed_pan',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self.jogTimedPanHandler, 
                'callback_args': ()
            },
            'jog_timed_tilt': {
                'namespace': self.node_namespace,
                'topic': 'ptx/jog_timed_tilt',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self.jogTimedTiltHandler, 
                'callback_args': ()
            },
            'reverse_pan_enabled': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_reverse_pan_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setReversePanEnable, 
                'callback_args': ()
            },
            'reverse_tilt_enabled': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_reverse_tilt_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setReverseTiltEnable, 
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
            },
            'set_pt_offsets': {
                'namespace': self.node_namespace,
                'topic': 'ptx/set_pt_offsets',
                'msg': PanTiltOffsets,
                'qsize': 1,
                'callback': self.setOffsetsHandler, 
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
        settings_ns = nepi_sdk.create_namespace(self.node_namespace,'ptx')

        self.SETTINGS_DICT = {
                    'capSettings': capSettings, 
                    'factorySettings': factorySettings,
                    'setSettingFunction': settingUpdateFunction, 
                    'getSettingsFunction': getSettingsFunction

        }
        self.settings_if = SettingsIF(namespace = settings_ns, 
                        settings_dict = self.SETTINGS_DICT,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )


        # Setup System IF Classes ####################
        ''' # PT saving handled by navpose IF class
        if getNpOrientationCb is not None:
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
                
            sd_namespace = nepi_sdk.create_namespace(self.node_namespace,'ptx')
            self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict,
                                    namespace = sd_namespace,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )
            time.sleep(1)
        '''

        if getNpOrientationCb is not None:
            # Create a NavPose Device IF
            self.capSettingsNavPose = capSettingsNavPose
            self.factorySettingsNavPose=factorySettingsNavPose
            self.settingUpdateFunctionNavPose=settingUpdateFunctionNavPose 
            self.getSettingsFunctionNavPose=getSettingsFunctionNavPose

            self.getNpHeadingCb = getNpHeadingCb  
            self.getNpPositionCb = getNpPositionCb
            self.getNpOrientationCb = getNpOrientationCb
            self.getNpLocationCb = getNpLocationCb
            self.getNpAltitudeCb = getNpAltitudeCb
            self.getNpDepthCb = getNpDepthCb
            self.max_navpose_update_rate = max_navpose_update_rate

            has_navpose = ( getNpHeadingCb is not None or \
            getNpPositionCb is not None or \
            getNpOrientationCb is not None or \
            getNpLocationCb is not None or \
            getNpAltitudeCb is not None or \
            getNpDepthCb is not None )
            
            if has_navpose == True:
                self.msg_if.pub_warn("Starting NPX Device IF Initialization", log_name_list = self.log_name_list)
                self.npx_if = NPXDeviceIF(device_info, 
                    capSettings = self.capSettingsNavPose,
                    factorySettings = self.factorySettingsNavPose,
                    settingUpdateFunction = self.settingUpdateFunctionNavPose, 
                    getSettingsFunction = self.getSettingsFunctionNavPose,
                    getHeadingCb = self.getNpHeadingCb, 
                    getPositionCb = self.getNpPositionAdjustedCb, 
                    getOrientationCb = self.getNpOrientationAdjustedCb,
                    getLocationCb = self.getNpLocationCb, 
                    getAltitudeCb = self.getNpAltitudeCb, 
                    getDepthCb = self.getNpDepthCb,
                    max_navpose_update_rate = self.max_navpose_update_rate,
                        log_name_list = self.log_name_list,
                        msg_if = self.msg_if
                        )


        time.sleep(1)
   
        ###############################
        # Finish Initialization
        self.min_pan_softstop_deg =  self.node_if.get_param('min_pan_softstop_deg')
        self.max_pan_softstop_deg = self.node_if.get_param('max_pan_softstop_deg')

        self.min_tilt_softstop_deg = self.node_if.get_param('min_tilt_softstop_deg')
        self.max_tilt_softstop_deg = self.node_if.get_param('max_tilt_softstop_deg')


        if self.setSoftLimitsCb is not None:
            self.setSoftLimitsCb(self.min_pan_softstop_deg,
                                self.max_pan_softstop_deg,
                                self.min_tilt_softstop_deg,
                                self.max_tilt_softstop_deg)

        if self.has_adjustable_speed == False:
            if self.setSpeedRatioCb is not None and self.getSpeedRatioCb is not None:
                speed_ratio = self.node_if.get_param('speed_ratio')
                self.msg_if.pub_warn("Initializing speed_ratio: " + str(self.speed_ratio))
                self.setSpeedRatioCb(speed_ratio)
                nepi_sdk.sleep(1)
                self.speed_ratio = self.getSpeedRatioCb()
                self.msg_if.pub_warn("Got Init speed ratio: " + str(self.speed_ratio))

                
        self.home_pan_deg = self.node_if.get_param('home_position/pan_deg')
        self.home_tilt_deg = self.node_if.get_param('home_position/tilt_deg')

        self.goHome()

        # Auto Scan Settings Update
        self.auto_pan_enabled = self.node_if.get_param('auto_pan_enabled')
        self.auto_pan_min = self.node_if.get_param('min_pan_softstop_deg')
        self.auto_pan_max = self.node_if.get_param('max_pan_softstop_deg')

        self.auto_tilt_enabled = self.node_if.get_param('auto_tilt_enabled')
        self.auto_tilt_min = self.node_if.get_param('min_tilt_softstop_deg')
        self.auto_tilt_max = self.node_if.get_param('max_tilt_softstop_deg')
       
        self.offsets_dict = self.node_if.get_param('offsets')
   
        # Set reverse int values
        rpi = 1
        if self.reverse_pan_enabled:
            rpi = -1
        self.rpi = rpi
        rti = 1
        if self.reverse_tilt_enabled:
            rti = -1
        self.rti = rti



        # Setup Joint Info
        self.frame_id = self.node_if.get_param('frame_id')
        self.pan_joint_name = self.node_if.get_param('pan_joint_name')
        self.tilt_joint_name = self.node_if.get_param('tilt_joint_name')
        self.reverse_pan_enabled = self.node_if.get_param('reverse_pan_enabled')
        self.reverse_tilt_enabled = self.node_if.get_param('reverse_tilt_enabled')
        self.msg_if.pub_debug("Factory Controls Dict: " + str(self.factory_controls_dict))
        self.msg_if.pub_debug("reverse_pan_enabled: " + str(self.reverse_pan_enabled))

        self.status_msg.header.frame_id = self.frame_id

        self.joint_state_msg.name = (self.pan_joint_name, self.tilt_joint_name)

        

        # Init everything
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
        nepi_sdk.start_timer_process(status_pub_delay, self.publishJointStateAndStatus, oneshot = True)

        if self.has_auto_pan:
            # Start Auto Pan Process
            self.msg_if.pub_info("Starting auto pan scanning process")
            nepi_sdk.start_timer_process(self.AUTO_SCAN_UPDATE_INTERVAL, self.autoPanProcess)

        if self.has_auto_tilt:
            # Start Auto Pan Process
            self.msg_if.pub_info("Starting auto tilt scanning process")
            nepi_sdk.start_timer_process(self.AUTO_SCAN_UPDATE_INTERVAL, self.autoTiltProcess)
        self.publish_status()
        self.ready = True
        self.msg_if.pub_info("Initialization Complete", log_name_list = self.log_name_list)



    ###############################
    # Class Methods

    def getPanAdj(self,pan_deg):
        return pan_deg * self.rpi

    def getPanRatioAdj(self,ratio):
        if self.reverse_pan_enabled == True:
            ratio = 1 - ratio
        return ratio

    def getTiltAdj(self,tilt_deg):
        return tilt_deg * self.rti

    def getPanTiltAdj(self,pan_deg,tilt_deg):
        adj_pan = self.getPanAdj(pan_deg)
        adj_tilt = self.getTiltAdj(tilt_deg)
        return adj_pan,adj_tilt

    def getTiltRatioAdj(self,ratio):
        if self.reverse_tilt_enabled == True:
            ratio = 1 - ratio
        return ratio

    def getPanMinMaxAdj(self,min_deg,max_deg):
        if self.reverse_pan_enabled == True:
            adj_min = copy.deepcopy(max_deg) * -1
            adj_max = copy.deepcopy(min_deg) * -1
        else:
            adj_min = min_deg
            adj_max = max_deg
        return adj_min,adj_max

    def getTiltMinMaxAdj(self,min_deg,max_deg):
        if self.reverse_tilt_enabled == True:
            adj_min = copy.deepcopy(max_deg) * -1
            adj_max = copy.deepcopy(min_deg) * -1
        else:
            adj_min = min_deg
            adj_max = max_deg
        return adj_min,adj_max

    def getLimitsAdj(self,pan_min,pan_max,tilt_min,tilt_max):
        [adj_pan_min,adj_pan_max] = self.getPanMinMaxAdj(pan_min,pan_max)
        [adj_tilt_min,adj_tilt_max] = self.getTiltMinMaxAdj(tilt_min,tilt_max)
        return adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max

    def getLimitsHardstopAdj(self):
        pan_min = self.min_pan_hardstop_deg
        pan_max = self.max_pan_hardstop_deg
        [adj_pan_min,adj_pan_max] = self.getPanMinMaxAdj(pan_min,pan_max)

        tilt_min = self.min_tilt_hardstop_deg
        tilt_max = self.max_tilt_hardstop_deg
        [adj_tilt_min,adj_tilt_max] = self.getTiltMinMaxAdj(tilt_min,tilt_max)

        return adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max

    def getLimitsSoftstopAdj(self):
        pan_min = self.min_pan_softstop_deg
        pan_max = self.max_pan_softstop_deg
        [adj_pan_min,adj_pan_max] = self.getPanMinMaxAdj(pan_min,pan_max)

        tilt_min = self.min_tilt_softstop_deg
        tilt_max = self.max_tilt_softstop_deg
        [adj_tilt_min,adj_tilt_max] = self.getTiltMinMaxAdj(tilt_min,tilt_max)

        return adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max




    def getPtPosition(self, orien_dict):
        x = 0.0
        y = 0.0
        z = 0.0
        if orien_dict is not None:
            
            ho = self.offsets_dict['z']
            xo = self.offsets_dict['x']
            yo = self.offsets_dict['y']
            zo = 0

            # calculate pos from tilt axis
            tilt_rad = -1 * math.radians(orien_dict['pitch_deg'])
            x = (xo * np.cos(tilt_rad) - zo * np.sin(tilt_rad))
            z = (zo * np.cos(tilt_rad) + xo * np.sin(tilt_rad)) * self.rpi

            # Add tilt axis height
            z += (ho * self.rpi)

            # Add left right offset
            y += yo * self.rpi

            # Rotate about center pan axis
            pan_deg = orien_dict['yaw_deg'] * self.rpi
            [x,y,z] = nepi_utils.rotate_3d([x,y,z], 'z', pan_deg) 

        return x,y,z


    def getNpPositionAdjustedCb(self):
        pos_dict = dict()
        pos_dict['time_position'] = nepi_utils.get_time()
        pos_dict['x_m'] = 0.0
        pos_dict['y_m'] = 0.0     
        pos_dict['z_m'] = 0.0

        if self.getNpOrientationCb is not None:
            orien_dict = self.getNpOrientationCb()
            [x,y,z] = self.getPtPosition(orien_dict)
            pos_dict['x_m'] = round(x,5)
            pos_dict['y_m'] = round(y,5)   
            pos_dict['z_m'] = round(z,5)
            self.msg_if.pub_debug("Calculate navpose x,y,z: " + str([x,y,z]), log_name_list = self.log_name_list, throttle_s = 3.0)
        return pos_dict

    def getNpOrientationAdjustedCb(self):
        orien_dict = dict()
        orien_dict['time_orientation'] = nepi_utils.get_time()
        orien_dict['yaw_deg'] = self.current_position[0]
        orien_dict['pitch_deg'] = self.current_position[1]      

        if self.getNpOrientationCb is not None:
            orien_dict = self.getNpOrientationCb()
            pan_deg = orien_dict['yaw_deg']
            orien_dict['yaw_deg'] = self.getPanAdj(pan_deg)
            tilt_deg = orien_dict['pitch_deg']
            orien_dict['pitch_deg'] = self.getTiltAdj(tilt_deg)
        return orien_dict
           

    def autoPanProcess(self,timer):
        #self.msg_if.pub_warn("Starting Pan Scan Process") 
        if self.auto_pan_enabled == False:
            self.is_auto_pan = False
        else:
            if self.has_seperate_pan_tilt_control == False:
                self.setAutoTilt(False)
            start_auto_pan = False
            if self.is_auto_pan == False:
                start_auto_pan = True     
            self.is_auto_pan = True    
            pan_cur = self.current_position[0]
            if start_auto_pan == True:
                self.gotoPanPosition(self.auto_pan_min)  
            elif (pan_cur < (self.auto_pan_min + self.AUTO_SCAN_SWITCH_DEG)):
                self.gotoPanPosition(self.auto_pan_max)
            elif (pan_cur > (self.auto_pan_max - self.AUTO_SCAN_SWITCH_DEG)):
                self.gotoPanPosition(self.auto_pan_min)

    def autoTiltProcess(self,timer):
        #self.msg_if.pub_warn("Starting Tilt Scan Process") 
        if self.auto_tilt_enabled == False:
            self.is_auto_tilt = False
        else:
            if self.has_seperate_pan_tilt_control == False:
                self.setAutoPan(False)
            start_auto_tilt = False
            if self.is_auto_tilt == False:
                start_auto_tilt = True     
            self.is_auto_tilt = True    
            tilt_cur = self.current_position[1]
            if start_auto_tilt == True:
                self.gotoTiltPosition(self.auto_tilt_min)  
            elif (tilt_cur < (self.auto_tilt_min + self.AUTO_SCAN_SWITCH_DEG)):
                self.gotoTiltPosition(self.auto_tilt_max)
            elif (tilt_cur > (self.auto_tilt_max - self.AUTO_SCAN_SWITCH_DEG)):
                self.gotoTiltPosition(self.auto_tilt_min)



    def check_ready(self):
        return self.ready  

    def wait_for_ready(self, timeout = float('inf') ):
        success = False
        if self.ready is not None:
            self.msg_if.pub_info("Waiting for connection", log_name_list = self.log_name_list)
            timer = 0
            time_start = nepi_sdk.get_time()
            while self.ready == False and timer < timeout and not nepi_sdk.is_shutdown():
                nepi_sdk.sleep(.1)
                timer = nepi_sdk.get_time() - time_start
            if self.ready == False:
                self.msg_if.pub_info("Failed to Connect", log_name_list = self.log_name_list)
            else:
                self.msg_if.pub_info("Connected", log_name_list = self.log_name_list)
        return self.ready   


    def panRatioToDeg(self, ratio):
        pan_deg = 0
        if self.reverse_pan_enabled == False:
           pan_deg =  self.min_pan_softstop_deg + (1-ratio) * (self.max_pan_softstop_deg - self.min_pan_softstop_deg)
        else:
           pan_deg =  self.max_pan_softstop_deg - (1-ratio)  * (self.max_pan_softstop_deg - self.min_pan_softstop_deg)
        return  pan_deg
    
    def panDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse_pan_enabled == False:
          ratio = 1 - (deg - self.min_pan_softstop_deg) / (self.max_pan_softstop_deg - self.min_pan_softstop_deg)
        else:
          ratio = (deg - self.min_pan_softstop_deg) / (self.max_pan_softstop_deg - self.min_pan_softstop_deg)
        return (ratio)
     
    def tiltDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse_tilt_enabled == False:
          ratio = 1 - (deg - self.min_tilt_softstop_deg) / (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)
        else:
          ratio = (deg - self.min_tilt_softstop_deg) / (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)
        return ratio
    
    def tiltRatioToDeg(self, ratio):
        tilt_deg = 0
        if self.reverse_tilt_enabled == False:
           tilt_deg =  self.min_tilt_softstop_deg + (1-ratio) * (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)
        else:
           tilt_deg =  (self.max_tilt_softstop_deg) - (1-ratio) * (self.max_tilt_softstop_deg - self.min_tilt_softstop_deg)
        return  tilt_deg



    def positionWithinSoftLimits(self, pan_deg, tilt_deg):
        valid = False
        if (pan_deg <= self.max_pan_softstop_deg) or (pan_deg >= self.min_pan_softstop_deg) or \
           (tilt_deg <= self.max_tilt_softstop_deg) or (tilt_deg >= self.min_tilt_softstop_deg):
            valid = True
        
        return valid


 
    def setSoftstopHandler(self, msg):
        min_pan = msg.min_pan_deg
        max_pan = msg.max_pan_deg
        min_tilt = msg.min_tilt_deg
        max_tilt = msg.max_tilt_deg

        [min_pan_adj,max_pan_adj,min_tilt_adj,max_tilt_adj] = self.getLimitsAdj(min_pan,max_pan,min_tilt,max_tilt)

        valid = False
        if min_pan_adj >= self.min_pan_hardstop_deg and max_pan_adj <= self.max_pan_hardstop_deg and min_pan_adj < max_pan_adj:  
            if min_tilt_adj >= self.min_tilt_hardstop_deg and max_tilt_adj <= self.max_tilt_hardstop_deg and min_tilt_adj < max_tilt_adj:
                valid = True

                self.min_pan_softstop_deg =  min_pan_adj
                self.max_pan_softstop_deg = max_pan_adj
                self.min_tilt_softstop_deg = min_tilt_adj
                self.max_tilt_softstop_deg = max_tilt_adj

                #self.msg_if.pub_info("Calling set auto pan limits with: " + str([min_pan_adj, max_pan_adj]))
                self.setAutoPanWindow( min_pan_adj, max_pan_adj)
                #self.msg_if.pub_info("Calling set auto tilt limits with: " + str([min_tilt_adj, max_tilt_adj]))
                self.setAutoTiltWindow( min_tilt_adj, max_tilt_adj)

                if self.setSoftLimitsCb is not None:
                    self.setSoftLimitsCb(min_pan_adj,max_pan_adj,min_tilt_adj,max_tilt_adj)

                self.node_if.set_param('max_pan_softstop_deg', max_pan_adj)
                self.node_if.set_param('min_pan_softstop_deg', min_pan_adj)
                self.node_if.set_param('max_tilt_softstop_deg', max_tilt_adj)
                self.node_if.set_param('min_tilt_softstop_deg', min_tilt_adj)


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
                self.node_if.set_param('speed_ratio',speed_ratio)
                self.msg_if.pub_warn("Updated speed ratio to " + str(speed_ratio))
        

    def setHomePositionHandler(self, msg):
        [pan_deg_adj,tilt_deg_adj] = self.getPanTiltAdj(msg.pan_deg, msg.tilt_deg)
        if not self.positionWithinSoftLimits(pan_deg_adj, tilt_deg_adj):
            self.msg_if.pub_warn("Requested home position is invalid... ignoring", log_name_list = self.log_name_list)
            return

        self.home_pan_deg = pan_deg_adj
        self.home_tilt_deg = tilt_deg_adj
        self.msg_if.pub_info("Updated home position to " + "%.2f" % self.home_pan_deg + " " + "%.2f" %  self.home_tilt_deg)
        self.publish_status()
        if self.setHomePositionCb is not None:
            # Driver supports absolute positioning, so just let it operate
            self.setHomePositionCb(self.home_pan_deg, self.home_tilt_deg)
        else:
            self.msg_if.pub_warn("Absolution position home setpoints not available... ignoring", log_name_list = self.log_name_list)
            return
        
        
            

    def goHomeHandler(self, _):
        self.goHome()

    def goHome(self):
        self.setAutoPan(False)
        self.setAutoTilt(False)
        if self.gotoPositionCb is not None:
            self.pan_goal_deg = self.home_pan_deg
            self.tilt_goal_deg = self.home_tilt_deg
            self.gotoPositionCb(self.pan_goal_deg,self.tilt_goal_deg)
            self.publish_status()
        elif self.goHomeCb is not None:
            self.goHomeCb()
        

    def gotoPositionHandler(self, msg):
        self.setAutoPan(False)
        self.setAutoTilt(False)
        if self.positionWithinSoftLimits is not None:
            [pan_deg_adj,tilt_deg_adj] = self.getPanTiltAdj(msg.pan_deg,msg.tilt_deg)
        self.gotPosition(pan_deg_adj,tilt_deg_adj)

    def gotPosition(self,pan_deg,tilt_deg):
            if not self.positionWithinSoftLimits(pan_deg, tilt_deg):
                self.msg_if.pub_warn("Requested goto position is invalid... ignoring", log_name_list = self.log_name_list)
                return
            self.pan_goal_deg = pan_deg
            self.tilt_goal_deg = tilt_deg
            self.publish_status()
            self.msg_if.pub_info("Driving to  " + "%.2f" % (self.pan_goal_deg * self.rpi) + " " + "%.2f" % (self.tilt_goal_deg * self.rti))
            self.gotoPositionCb(self.pan_goal_deg, self.tilt_goal_deg)

    def gotoPanPositionHandler(self, msg):
        self.setAutoPan(False)
        pan_deg_adj = self.getPanAdj(msg.data)
        self.gotoPanPosition(pan_deg_adj)

    def gotoPanPosition(self,pan_deg):
        tilt_deg = self.getTiltAdj(self.status_msg.tilt_now_deg)
        if self.positionWithinSoftLimits is not None:
            if not self.positionWithinSoftLimits(pan_deg, tilt_deg):
                self.msg_if.pub_warn("Requested goto pan position is invalid... ignoring", log_name_list = self.log_name_list)
                return
            self.pan_goal_deg = pan_deg
            if self.gotoPanPositionCb is not None:
                self.msg_if.pub_info("Driving to pan " + "%.2f" % (self.pan_goal_deg * self.rpi))
                self.gotoPanPositionCb(self.pan_goal_deg)
            elif self.gotoPositionCb is not None:
                self.setAutoTilt(False)
                self.tilt_goal_deg = tilt_deg
                self.msg_if.pub_info("Driving to  " + "%.2f" % self.pan_goal_deg * self.rpi + " " + "%.2f" % self.tilt_goal_deg * self.rti)
                self.gotoPositionCb(self.pan_goal_deg, self.tilt_goal_deg)
            self.publish_status()
                 

    def gotoTiltPositionHandler(self, msg):
        self.setAutoTilt(False)
        tilt_deg_adj = self.getTiltAdj(msg.data)
        self.gotoPanPosition(tilt_deg_adj)

    def gotoTiltPosition(self,tilt_deg):
        pan_deg = self.getPanAdj(self.status_msg.pan_now_deg)
        if self.positionWithinSoftLimits is not None:
            if not self.positionWithinSoftLimits(pan_deg, tilt_deg):
                self.msg_if.pub_warn("Requested goto tilt position is invalid... ignoring", log_name_list = self.log_name_list)
                return
            self.tilt_goal_deg = tilt_deg
            if self.gotoTiltPositionCb is not None:
                self.msg_if.pub_info("Driving to  " + "%.2f" % (self.tilt_goal_deg * self.rti))
                self.gotoTiltPositionCb(self.tilt_goal_deg)    
            elif self.gotoPositionCb is not None:
                self.setAutoPan(False)
                self.pan_goal_deg = pan_deg
                self.msg_if.pub_info("Driving to  " + "%.2f" % self.pan_goal_deg * self.rpi + " " + "%.2f" % self.tilt_goal_deg * self.rti)
                self.gotoPositionCb(self.pan_goal_deg, self.tilt_goal_deg)
            self.publish_status()
    

    def gotoToPanRatioHandler(self, msg):
        self.setAutoPan(False)
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid pan position ratio " + "%.2f" % ratio)
            return
        self.pan_goal_deg = self.panRatioToDeg(ratio) # Function takes care of reverse conversion
        if self.gotoPanPositionCb is not None:
            self.msg_if.pub_info("Driving to  " + "%.2f" % self.pan_goal_deg * self.rpi)
            self.gotoPanPositionCb(self.pan_goal_deg)
        elif self.gotoPositionCb is not None:
            self.setAutoTilt(False)
            self.tilt_goal_deg = self.getTiltAdj(self.status_msg.tilt_now_deg)
            self.msg_if.pub_info("Driving to  " + "%.2f" % self.pan_goal_deg * self.rpi + " " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoPositionCb(self.pan_goal_deg, self.tilt_goal_deg)
        self.publish_status()
        

    def gotoToTiltRatioHandler(self, msg):
        self.setAutoTilt(False)
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid tilt position ratio " + "%.2f" % ratio)
            return
        self.tilt_goal_deg = self.tiltRatioToDeg(ratio) # Function takes care of reverse conversion
        if self.gotoTiltPositionCb is not None:
            self.msg_if.pub_info("Driving to  " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoTiltPositionCb(self.tilt_goal_deg)
        elif self.gotoPositionCb is not None:
            self.setAutoPan(False)
            self.pan_goal_deg = self.getPanAdj(self.status_msg.pan_now_deg)
            self.msg_if.pub_info("Driving to  " + "%.2f" % self.pan_goal_deg * self.rpi + " " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoPositionCb(self.pan_goal_deg, self.tilt_goal_deg)
        self.publish_status()
        

    def stopMovingHandler(self, _):
        self.setAutoPan(False)
        self.setAutoTilt(False)
        self.pan_goal_deg = self.status_msg.pan_now_deg * self.rpi
        self.tilt_goal_deg = self.status_msg.tilt_now_deg * self.rti
        self.msg_if.pub_info("Stopping motion by request", log_name_list = self.log_name_list)
        if self.stopMovingCb is not None:
            self.stopMovingCb()
        elif self.gotoPositionCb is not None:
            self.gotoPositionCb(self.pan_goal_deg,self.tilt_goal_deg)
        self.publish_status()
        

    def jogTimedPanHandler(self, msg):
        self.setAutoPan(False)
        self.msg_if.pub_warn("Got job pan msg: " + str(msg))
        if self.movePanCb is not None:
            direction = msg.direction * self.rpi
            time_s = msg.duration_s
            self.movePanCb(direction,  time_s)
            self.msg_if.pub_info("Jogging pan", log_name_list = self.log_name_list)
        self.publish_status()
        

    def jogTimedTiltHandler(self, msg):
        self.setAutoTilt(False)
        self.msg_if.pub_warn("Got job tilt msg: " + str(msg))
        if self.moveTiltCb is not None:
            direction = msg.direction * self.rti
            time_s = msg.duration_s
            self.moveTiltCb(direction, time_s)
            self.msg_if.pub_info("Jogging tilt", log_name_list = self.log_name_list)
        self.publish_status()
        

    def setReversePanEnable(self, msg):
        self.reverse_pan_enabled = msg.data
        rpi = 1
        if msg.data == True:
            rpi = -1
        self.rpi = rpi
        self.msg_if.pub_info("Set pan control to reverse=" + str(self.reverse_pan_enabled))
        self.publish_status()
        

    def setReverseTiltEnable(self, msg):
        self.reverse_tilt_enabled = msg.data
        rti = 1
        if msg.data == True:
            rti = -1
        self.rti = rti
        self.msg_if.pub_info("Set tilt control to reverse=" + str(self.reverse_tilt_enabled))
        self.publish_status()
        

    def setHomePositionHereHandler(self, _):
        self.home_pan_deg = self.status_msg.pan_now_deg * self.rpi
        self.home_tilt_deg = self.status_msg.tilt_now_deg * self.rti
        self.msg_if.pub_info("Home positon set to: " + "%.2f" % self.home_pan_deg * self.rpi + " " + "%.2f" % self.home_tilt_deg * self.rti)
        self.publish_status()
        if self.setHomePositionHereCb is not None:
            self.setHomePositionHereCb()
          

    def setAutoPanHandler(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting auto pan: " + str(enabled))
        self.setAutoPan(enabled)
        self.publish_status()

    def setAutoPan(self,enabled):
        self.auto_pan_enabled = enabled
        self.publish_status()
        self.node_if.set_param('auto_pan_enabled', self.auto_pan_enabled)
        
    def setAutoPanWindowHandler(self, msg):
        if self.reverse_pan_enabled == True:
            adj_min_deg = msg.stop_range * self.rpi
            adj_max_deg = msg.start_range * self.rpi
        else:            
            adj_min_deg = msg.start_range
            adj_max_deg = msg.stop_range
        self.msg_if.pub_info("Setting auto pan limits to: " + "%.2f" % adj_min_deg * self.rpi + " " + "%.2f" % adj_max_deg * self.rpi)
        self.setAutoPanWindow(adj_min_deg,adj_max_deg)


    def setAutoPanWindow(self, min_deg, max_deg):
        if max_deg > min_deg:
            if max_deg > self.max_pan_softstop_deg:
                max_deg = self.max_pan_softstop_deg
            if min_deg < self.min_pan_softstop_deg:
                min_deg = self.min_pan_softstop_deg
            self.auto_pan_min = min_deg
            self.auto_pan_max = max_deg
            self.msg_if.pub_info("Auto Pan limits set to: " + "%.2f" % min_deg * self.rpi + " " + "%.2f" % max_deg * self.rpi)
            self.publish_status()
            self.node_if.set_param('min_pan_softstop_deg', min_deg)
            self.node_if.set_param('max_pan_softstop_deg', max_deg)



    def setAutoTiltHandler(self, msg):
        enabled = msg.data
        self.msg_if.pub_info("Setting auto tilt: " + str(enabled))
        self.setAutoTilt(enabled)

    def setAutoTilt(self,enabled):
        self.auto_tilt_enabled = enabled
        self.publish_status()
        self.node_if.set_param('auto_tilt_enabled', self.auto_tilt_enabled)


    def setAutoTiltWindowHandler(self, msg):
        if self.reverse_tilt_enabled == True:
            adj_min_deg = msg.stop_range * self.rti
            adj_max_deg = msg.start_range * self.rti
        else:            
            adj_min_deg = msg.start_range
            adj_max_deg = msg.stop_range
        self.msg_if.pub_info("Setting auto pan limits to: " + "%.2f" % adj_min_deg * self.rti + " " + "%.2f" % adj_max_deg * self.rti)
        self.setAutoTiltWindow(adj_min_deg,adj_max_deg)


    def setOffsetsHandler(self, msg):
        self.msg_if.pub_info("Setting offsets to: " + str(msg))
        self.offsets_dict['x'] = msg.x_from_base_center
        self.offsets_dict['y'] = msg.y_from_base_center
        self.offsets_dict['z'] = msg.z_from_base_center
        self.publish_status()
        self.node_if.set_param('offsets', self.offsets_dict)



    def setAutoTiltWindow(self, min_deg, max_deg):
        if max_deg > min_deg:
            if max_deg > self.max_tilt_softstop_deg:
                max_deg = self.max_tilt_softstop_deg
            if min_deg < self.min_tilt_softstop_deg:
                min_deg = self.min_tilt_softstop_deg
            self.auto_tilt_min = min_deg
            self.auto_tilt_max = max_deg
            self.msg_if.pub_info("Auto Tilt limits set to: " + "%.2f" % min_deg * self.rti + " " + "%.2f" % max_deg * self.rti)
            self.publish_status()
            self.node_if.set_param('min_tilt_softstop_deg', min_deg)
            self.node_if.set_param('max_tilt_softstop_deg', max_deg)




    def provideCapabilities(self, _):
        return self.capabilities_report
    

    def initConfig(self):
        self.initCb(do_updates = True)

    def initCb(self, do_updates = False):
        if do_updates == True:
            self.resetCb()



    def resetCb(self, do_updates = True):
        self.msg_if.pub_warn("Reseting System to Current Values")
        self.max_pan_hardstop_deg = self.node_if.get_param('max_pan_hardstop_deg')
        self.min_pan_hardstop_deg = self.node_if.get_param('min_pan_hardstop_deg')
        self.max_pan_softstop_deg = self.node_if.get_param('max_pan_softstop_deg')
        self.min_pan_softstop_deg = self.node_if.get_param('min_pan_softstop_deg')

        self.max_tilt_hardstop_deg = self.node_if.get_param('max_tilt_hardstop_deg')
        self.min_tilt_hardstop_deg = self.node_if.get_param('min_tilt_hardstop_deg')
        self.max_tilt_softstop_deg = self.node_if.get_param('max_tilt_softstop_deg')
        self.min_tilt_softstop_deg = self.node_if.get_param('min_tilt_softstop_deg')

        #**********************
        # This one comes from the parent
        if self.getSoftLimitsCb is not None:
                [min_pan,max_pan,min_tilt,max_tilt] = self.getSoftLimitsCb()
                if min_pan != -999:
                    self.min_pan_softstop_deg = min_pan
                    self.node_if.set_param('min_pan_softstop_deg', self.min_pan_softstop_deg)
                if max_pan != -999:
                    self.max_pan_softstop_deg = max_pan
                    self.node_if.set_param('max_pan_softstop_deg',self.max_pan_softstop_deg)

                if min_tilt != -999:
                    self.min_tilt_softstop_deg = min_tilt
                    self.node_if.set_param('min_tilt_softstop_deg', self.min_tilt_softstop_deg)
                if max_tilt != -999:
                    self.max_tilt_softstop_deg = max_tilt
                    self.node_if.set_param('max_tilt_softstop_deg',self.max_tilt_softstop_deg)

        if self.getSpeedRatioCb is not None:
            self.speed_ratio = self.getSpeedRatioCb()
            self.msg_if.pub_warn("ResetCb got speed_ratio: " + str(self.speed_ratio))
            self.node_if.set_param('speed_ratio', self.speed_ratio) 
        #**********************
        



    def factoryResetCb(self, do_updates = True):
        if self.deviceResetCb is not None:
            self.deviceResetCb()
            nepi_sdk.sleep(2)
        self.resetCb()


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
        nepi_sdk.start_timer_process(status_pub_delay, self.publishJointStateAndStatus, oneshot = True)

    def publish_status(self, do_updates = False):
        start_time = nepi_utils.get_time()
        self.status_msg.header.stamp = nepi_sdk.get_msg_stamp()
        #self.msg_if.pub_info("Entering Publish Status", log_name_list = self.log_name_list)
        if self.has_absolute_positioning == True and self.getPositionCb is not None:
            [pan_deg,tilt_deg] = self.getPositionCb()
            #self.msg_if.pub_warn("Got PT position: " + str(pan_deg) + " : " + str(tilt_deg))

            if pan_deg != -999 and tilt_deg != -999:
                self.current_position = [pan_deg,tilt_deg]
            
        # Update Status Info
        got_time = nepi_utils.get_time() - start_time
        got_time = round(got_time, 3)
        #self.msg_if.pub_debug("Got PT status with time: " + str(got_time))
    
        #self.msg_if.pub_warn("Using PT position: " + str(self.current_position[0]) + " : " + str(self.current_position[1]))
        [pan_deg,tilt_deg] = self.current_position


        pan_now_deg_adj = self.getPanAdj(pan_deg)
        self.status_msg.pan_now_deg = pan_now_deg_adj
        pan_goal_deg_adj = self.getPanAdj(self.pan_goal_deg)
        self.status_msg.pan_goal_deg = pan_goal_deg_adj
        self.status_msg.pan_home_pos_deg = self.getPanAdj(self.home_pan_deg)

        tilt_now_deg_adj = self.getTiltAdj(tilt_deg)
        self.status_msg.tilt_now_deg = tilt_now_deg_adj
        tilt_goal_deg_adj = self.getTiltAdj(self.tilt_goal_deg)
        self.status_msg.tilt_goal_deg = tilt_goal_deg_adj
        self.status_msg.tilt_home_pos_deg = self.getTiltAdj(self.home_tilt_deg)


        [adj_pan_hs_min,adj_pan_hs_max,adj_tilt_hs_min,adj_tilt_hs_max] = self.getLimitsHardstopAdj()
        [adj_pan_ss_min,adj_pan_ss_max,adj_tilt_ss_min,adj_tilt_ss_max] = self.getLimitsSoftstopAdj()

        self.status_msg.pan_min_hardstop_deg = adj_pan_hs_min
        self.status_msg.pan_max_hardstop_deg = adj_pan_hs_max
        self.status_msg.pan_min_softstop_deg = adj_pan_ss_min
        self.status_msg.pan_max_softstop_deg = adj_pan_ss_max

        self.status_msg.tilt_min_hardstop_deg = adj_tilt_hs_min
        self.status_msg.tilt_max_hardstop_deg = adj_tilt_hs_max
        self.status_msg.tilt_min_softstop_deg = adj_tilt_ss_min
        self.status_msg.tilt_max_softstop_deg = adj_tilt_ss_max
        

        axis_info = [pan_now_deg_adj,pan_goal_deg_adj,adj_pan_ss_min,adj_pan_ss_max]
        #self.msg_if.pub_warn("Using Pan now,goal,min,max: " + str(axis_info), log_name_list = self.log_name_list, throttle_s = 2.0)        
        pan_now_ratio_adj =  1 - (pan_now_deg_adj - adj_pan_ss_min) / (adj_pan_ss_max - adj_pan_ss_min) 
        self.status_msg.pan_now_ratio = pan_now_ratio_adj
        pan_goal_ratio_adj =  1 - (pan_goal_deg_adj - adj_pan_ss_min) / (adj_pan_ss_max - adj_pan_ss_min)
        self.status_msg.pan_goal_ratio = pan_goal_ratio_adj

        axis_info = [tilt_now_deg_adj,tilt_goal_deg_adj,adj_tilt_ss_min,adj_tilt_ss_max]
        #self.msg_if.pub_warn("Using Tilt now,goal,min,max: " + str(axis_info), log_name_list = self.log_name_list, throttle_s = 2.0)    
        tilt_now_ratio_adj =  1 - (tilt_now_deg_adj - adj_tilt_ss_min) / (adj_tilt_ss_max - adj_tilt_ss_min) 
        self.status_msg.tilt_now_ratio = tilt_now_ratio_adj
        tilt_goal_ratio_adj =  1 - (tilt_goal_deg_adj - adj_tilt_ss_min) / (adj_tilt_ss_max - adj_tilt_ss_min)
        self.status_msg.tilt_goal_ratio = tilt_goal_ratio_adj


        self.status_msg.reverse_pan_enabled = self.reverse_pan_enabled
        self.status_msg.reverse_tilt_enabled = self.reverse_tilt_enabled

        self.status_msg.speed_ratio = self.speed_ratio

        self.status_msg.auto_pan_enabled = self.auto_pan_enabled
        self.status_msg.auto_pan_range_window.start_range = self.getPanAdj(self.auto_pan_min)
        self.status_msg.auto_pan_range_window.stop_range = self.getPanAdj(self.auto_pan_max)
        

        self.status_msg.auto_tilt_enabled = self.auto_tilt_enabled
        self.status_msg.auto_tilt_range_window.start_range = self.getTiltAdj(self.auto_tilt_min)
        self.status_msg.auto_tilt_range_window.stop_range = self.getTiltAdj(self.auto_tilt_max)

        
        pan_changed = self.last_position[0] != self.current_position[0]
        tilt_changed = self.last_position[1] != self.current_position[1]
        self.last_position = copy.deepcopy(self.current_position)
        self.status_msg.is_moving = pan_changed or tilt_changed

        self.status_msg.x_from_base_center = self.offsets_dict['x']
        self.status_msg.y_from_base_center = self.offsets_dict['y']
        self.status_msg.z_from_base_center = self.offsets_dict['z']

        if do_updates == True:
            if self.getSoftLimitsCb is not None:
                [min_pan,max_pan,min_tilt,max_tilt] = self.getSoftLimitsCb()
                if min_pan != -999:
                    self.min_pan_softstop_deg = min_pan
                    self.node_if.set_param('min_pan_softstop_deg', self.min_pan_softstop_deg)
                if max_pan != -999:
                    self.max_pan_softstop_deg = max_pan
                    self.node_if.set_param('max_pan_softstop_deg',self.max_pan_softstop_deg)

                if min_tilt != -999:
                    self.min_tilt_softstop_deg = min_tilt
                    self.node_if.set_param('min_tilt_softstop_deg', self.min_tilt_softstop_deg)
                if max_tilt != -999:
                    self.max_tilt_softstop_deg = max_tilt
                    self.node_if.set_param('max_tilt_softstop_deg',self.max_tilt_softstop_deg)

            if self.getSpeedRatioCb is not None:
                self.status_msg.speed_ratio = self.getSpeedRatioCb()
                self.speed_ratio = self.status_msg.speed_ratio
                self.msg_if.pub_warn("Status updated speed ratio: " + str(self.speed_ratio))
                self.node_if.set_param('speed_ratio', self.speed_ratio) 



    
        self.msg_if.pub_debug("Publishing Status", log_name_list = self.log_name_list)

        self.node_if.publish_pub('status_pub',self.status_msg)


        pan_rad = 0.01745329 * self.status_msg.pan_now_deg
        tilt_rad = 0.01745329 * self.status_msg.tilt_now_deg

        # And joint state if appropriate
        self.joint_state_msg.header.stamp = self.status_msg.header.stamp
        self.joint_state_msg.position[0] = pan_rad
        self.joint_state_msg.position[1] = tilt_rad
        self.msg_if.pub_debug("Publishing Joint", log_name_list = self.log_name_list)
        self.node_if.publish_pub('joint_pub',self.joint_state_msg)

        pub_time = nepi_utils.get_time() - start_time


        return pub_time



    def passFunction(self):
        return 0

