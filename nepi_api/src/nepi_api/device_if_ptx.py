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
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header
from nav_msgs.msg import Odometry
from nepi_interfaces.msg import RangeWindow
from nepi_interfaces.msg import DevicePTXStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove
from nepi_interfaces.srv import PTXCapabilitiesQuery, PTXCapabilitiesQueryRequest, PTXCapabilitiesQueryResponse

from nepi_interfaces.msg import Frame3DTransform
from nepi_interfaces.msg import NavPose

from tf.transformations import quaternion_from_euler

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.system_if import SaveDataIF, SettingsIF, Transform3DIF
from nepi_api.device_if_npx import NPXDeviceIF




class PTXActuatorIF:
    MAX_STATUS_UPDATE_RATE = 3

    # Backup Factory Control Values 
    FACTORY_CONTROLS_DICT = {
                'reverse_pan_enabled' : False,
                'reverse_tilt_enabled' : False,
                'speed_ratio' : 0.5
    }

    AUTO_SCAN_SWITCH_DEG = 5 # If angle withing this bound, switch dir
    AUTO_SCAN_UPDATE_INTERVAL = 1

    orientation_dict = {
        'time_orientation': nepi_utils.get_time(),
        # Orientation should be provided in Degrees ENU
        'roll_deg': 0.0,
        'tilt_deg': 0.0,
        'pan_deg': 0.0,
    }

    ready = False

    node_if = None
    settings_if = None
    save_data_if = None
    transform_if = None
    npx_if = None
    navpose_if = None

    status_msg = DevicePTXStatus()

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

    frame_3d = 'nepi_frame'
    tr_source_ref_description = 'tilt_axis_center'
    tr_end_ref_description = 'nepi_frame'


    data_source_description = 'pan_tilt'
    data_ref_description = 'tilt_axis_center'
    data_end_description = 'nepi_frame'
    device_mount_description = 'fixed'
    mount_desc = 'None'
    
    is_moving = False

    navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    sys_navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT

    ### IF Initialization
    def __init__(self,  device_info, 
                 capSettings, factorySettings, 
                 settingUpdateFunction, getSettingsFunction,
                 factoryControls , # Dictionary to be supplied by parent, specific key set is required
                 factoryLimits = None,
                 data_source_description = 'pan_tilt',
                 data_ref_description = 'tilt_axis_center',
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
                 getNavPoseCb=None,
                 navpose_update_rate = 10,
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
        self.namespace = nepi_sdk.create_namespace(self.node_namespace,'ptx')
        self.data_products_list = []


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
        self.device_id = device_info["device_name"]
        self.identifier = device_info["identifier"]
        self.serial_num = device_info["serial_number"]
        self.hw_version = device_info["hw_version"]
        self.sw_version = device_info["sw_version"]

        self.device_name = self.device_id + "_" + self.identifier

        self.data_source_description = data_source_description
        self.data_ref_description = data_ref_description

        # Create and update factory controls dictionary
        self.factory_controls_dict = self.FACTORY_CONTROLS_DICT
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]

        self.deviceResetCb = deviceResetCb
        self.device_name = device_info["device_name"] + "_" + device_info["identifier"]


       # Configure PTX Capabilities

        # STOP MOVE #############
        self.stopMovingCb = stopMovingCb


        # GET POSITION #############
        self.getPositionCb = getPositionCb
        if self.getPositionCb is not None:
            self.has_position_feedback = True
            self.has_limit_controls = True

        self.getNavPoseCb = getNavPoseCb
        if navpose_update_rate < 1:
           navpose_update_rate = 1
        if navpose_update_rate > 10:
            navpose_update_rate = 10
        self.navpose_update_rate = navpose_update_rate



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
        if getSpeedRatioCb is not None:
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
        self.status_msg.device_id = self.device_id
        self.status_msg.identifier = self.identifier
        self.status_msg.serial_num = self.serial_num
        self.status_msg.hw_version = self.hw_version
        self.status_msg.sw_version = self.sw_version

        self.status_msg.data_source_description = self.data_source_description
        self.status_msg.data_ref_description = self.data_ref_description
        self.status_msg.device_mount_description = self.get_mount_description() 

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

        ##################################################
        ### Node Class Setup

        self.msg_if.pub_info("Starting Node IF Initialization", log_name_list = self.log_name_list)
        # Configs Config Dict ####################
        self.CONFIGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace':  self.namespace
        }


        # Params Config Dict ####################

        self.PARAMS_DICT = {
            'device_name': {
                'namespace': self.namespace,
                'factory_val': self.device_name
            },
            'speed_ratio': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['speed_ratio']
            },
            'home_position/pan_deg': {
                'namespace': self.namespace,
                'factory_val': 0.0
            },
            'home_position/tilt_deg': {
                'namespace': self.namespace,
                'factory_val': 0.0
            },
            'reverse_pan_enabled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['reverse_pan_enabled']
            },            
            'reverse_tilt_enabled': {
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['reverse_tilt_enabled']
            },
            'max_pan_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_pan_softstop_deg']
            },            
            'min_pan_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_pan_softstop_deg']
            },            
            'max_tilt_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_tilt_softstop_deg']
            },           
            'min_tilt_softstop_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_tilt_softstop_deg']
            },           
            'auto_pan_enabled': {
                'namespace': self.namespace,
                'factory_val': False
            },           
            'min_auto_pan_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_pan_softstop_deg']
            },           
            'max_auto_pan_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_pan_softstop_deg']
            },           
            'auto_tilt_enabled': {
                'namespace': self.namespace,
                'factory_val': False
            },           
            'min_auto_tilt_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_tilt_softstop_deg']
            },           
            'max_auto_tilt_deg': {
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_tilt_softstop_deg']
            },
            'mount_desc': {
                'namespace': self.namespace,
                'factory_val': 'None'
            }
        }
        

        # Services Config Dict ####################

        self.SRVS_DICT = {
            'capabilities_query': {
                'namespace': self.namespace,
                'topic': 'capabilities_query',
                'srv': PTXCapabilitiesQuery,
                'req': PTXCapabilitiesQueryRequest(),
                'resp': PTXCapabilitiesQueryResponse(),
                'callback': self.provideCapabilities
            }
        }


        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': DevicePTXStatus,
                'qsize': 10,
                'latch': False
            },
            'navpose_pub': {
                'msg': NavPose,
                'namespace': self.namespace,
                'topic': 'navpose',
                'qsize': 1,
                'latch': False
            }
        }



        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'set_device_name': {
                'namespace': self.namespace,
                'topic': 'set_device_name',
                'msg': String,
                'qsize': 1,
                'callback': self.updateDeviceNameCb, 
                'callback_args': ()
            },
            'reset_device_name': {
                'namespace': self.namespace,
                'topic': 'reset_device_name',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetDeviceNameCb, 
                'callback_args': ()
            },
            'speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.setSpeedRatioHandler, 
                'callback_args': ()
            },
            'stop_moving': {
                'namespace': self.namespace,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
                'callback': self.stopMovingHandler, 
                'callback_args': ()
            },
            'goto_to_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self.gotoPositionHandler, 
                'callback_args': ()
            },
            'goto_to_pan_position': {
                'namespace': self.namespace,
                'topic': 'goto_pan_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoPanPositionHandler, 
                'callback_args': ()
            },
            'goto_to_tilt_position': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoTiltPositionHandler, 
                'callback_args': ()
            },
            'goto_pan_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_pan_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoToPanRatioHandler, 
                'callback_args': ()
            },
            'goto_tilt_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self.gotoToTiltRatioHandler, 
                'callback_args': ()
            },
            'jog_timed_pan': {
                'namespace': self.namespace,
                'topic': 'jog_timed_pan',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self.jogTimedPanHandler, 
                'callback_args': ()
            },
            'jog_timed_tilt': {
                'namespace': self.namespace,
                'topic': 'jog_timed_tilt',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self.jogTimedTiltHandler, 
                'callback_args': ()
            },
            'reverse_pan_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_pan_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setReversePanEnable, 
                'callback_args': ()
            },
            'reverse_tilt_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_tilt_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setReverseTiltEnable, 
                'callback_args': ()
            },
            'set_soft_limits': {
                'namespace': self.namespace,
                'topic': 'set_soft_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
                'callback': self.setSoftstopHandler, 
                'callback_args': ()
            },
            'go_home': {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
                'callback': self.goHomeHandler, 
                'callback_args': ()
            },
            'set_home_position': {
                'namespace': self.namespace,
                'topic': 'set_home_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self.setHomePositionHandler, 
                'callback_args': ()
            },
            'set_home_position_here': {
                'namespace': self.namespace,
                'topic': 'set_home_position_here',
                'msg': Empty,
                'qsize': 1,
                'callback': self.setHomePositionHereHandler, 
                'callback_args': ()
            },
            'set_auto_pan': {
                'namespace': self.namespace,
                'topic': 'set_auto_pan_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setAutoPanHandler, 
                'callback_args': ()
            },
            'set_auto_pan_window': {
                'namespace': self.namespace,
                'topic': 'set_auto_pan_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self.setAutoPanWindowHandler, 
                'callback_args': ()
            },
            'set_auto_tilt': {
                'namespace': self.namespace,
                'topic': 'set_auto_tilt_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self.setAutoTiltHandler, 
                'callback_args': ()
            },
            'set_auto_tilt_window': {
                'namespace': self.namespace,
                'topic': 'set_auto_tilt_window',
                'msg': RangeWindow,
                'qsize': 1,
                'callback': self.setAutoTiltWindowHandler, 
                'callback_args': ()
            },
            'set_mount_desc': {
                'namespace': self.namespace,
                'topic': 'set_mount_description',
                'msg': String,
                'qsize': 1,
                'callback': self.setMountDescCb, 
                'callback_args': ()
            },
            'reset_mount_desc': {
                'namespace': self.namespace,
                'topic': 'reset_mount_description',
                'msg': Empty,
                'qsize': 1,
                'callback': self.resetMountDescCb, 
                'callback_args': ()
            },
            'sys_navpose_sub': {
                'namespace': self.base_namespace,
                'topic': 'navpose',
                'msg': NavPose,
                'qsize': 1,
                'callback': self.navposeSysCb, 
                'callback_args': ()
            }
        }
        
        


        # Create Node Class ####################
        self.node_if = NodeClassIF(
                        configs_dict = self.CONFIGS_DICT,
                        params_dict = self.PARAMS_DICT,
                        services_dict = self.SRVS_DICT,
                        pubs_dict = self.PUBS_DICT,
                        subs_dict = self.SUBS_DICT,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )

        ready = self.node_if.wait_for_ready()


        # Init everything
        self.initCb(do_updates = True)

        ##############################
        # Start Node Processes

        # Periodic publishing
        status_update_rate = self.navpose_update_rate  
        if status_update_rate < 1:
            status_update_rate = 1
        if status_update_rate > 10:
            status_update_rate = 10
        self.status_update_rate = status_update_rate
        self.msg_if.pub_warn("Starting pt status publisher at hz: " + str(self.status_update_rate))    
        status_pub_delay = float(1.0) / self.status_update_rate
        nepi_sdk.start_timer_process(status_pub_delay, self._publishStatusCb)

        if self.has_auto_pan:
            # Start Auto Pan Process
            self.msg_if.pub_info("Starting auto pan scanning process")
            nepi_sdk.start_timer_process(self.AUTO_SCAN_UPDATE_INTERVAL, self.autoPanProcess)

        if self.has_auto_tilt:
            # Start Auto Pan Process
            self.msg_if.pub_info("Starting auto tilt scanning process")
            nepi_sdk.start_timer_process(self.AUTO_SCAN_UPDATE_INTERVAL, self.autoTiltProcess)

        ##############################
        # Start Additional System Processes

        ################################
        # Setup Settings IF Class ####################
        self.msg_if.pub_info("Starting Settings IF Initialization", log_name_list = self.log_name_list)
        settings_ns = self.namespace

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

        ##############################
        # Setup Save Data IF Classes ####################
        if self.getNavPoseCb is not None:
            self.data_products_list.append('navpose')
            self.msg_if.pub_info("Starting Save Data IF Initialization", log_name_list = self.log_name_list)
            factory_data_rates = {}
            for d in self.data_products_list:
                factory_data_rates[d] = [0.0, 0.0, 100.0] # Default to 10Hz save rate, set last save = 0.0, max rate = 100.0Hz

            factory_filename_dict = {
                'prefix': "", 
                'add_timestamp': True, 
                'add_ms': True,
                'add_us': False,
                'suffix': "ptx",
                'add_node_name': True
                }
                
            sd_namespace = self.namespace
            self.save_data_if = SaveDataIF(data_products = self.data_products_list,
                                    factory_rate_dict = factory_data_rates,
                                    factory_filename_dict = factory_filename_dict,
                                    namespace = sd_namespace,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )


        '''
        ############################
        # Setup 3D Transform IF Class ####################
        self.msg_if.pub_debug("Starting 3D Transform IF Initialization", log_name_list = self.log_name_list)
        transform_ns = self.namespace
        self.transform_if = Transform3DIF(namespace = transform_ns,
                        source_ref_description = self.tr_source_ref_description,
                        end_ref_description = self.tr_end_ref_description,
                        get_3d_transform_function = self.get_3d_transform,
                        log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                        )
        '''

        ##################################
        # Start Node Processes
        #nepi_sdk.start_timer_process(1, self._updaterCb, oneshot = True)
        nepi_sdk.start_timer_process(1, self.navPoseUpdaterCb, oneshot = True) 

        '''

        ###############################
        # Create a NPX Device IF
        if self.getNavPoseCb is not None:
            self.msg_if.pub_warn("Starting NPX Device IF Initialization", log_name_list = self.log_name_list)
            self.npx_if = NPXDeviceIF(device_info, 
                data_source_description = self.data_source_description,
                data_ref_description = self.data_ref_description,
                getNavPoseCb = self.getNavPoseCb,
                get3DTransformCb = self.transform_if.get_3d_transform,
                navpose_update_rate = self.navpose_update_rate,
                log_name_list = self.log_name_list,
                            msg_if = self.msg_if
                )


        '''
        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)
        ####################################

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
            
            ho = 0
            xo = 0
            yo = 0
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

        if self.getNpPositionCb is not None:
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
        


    def updateDeviceNameCb(self, msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        new_device_name = msg.data
        self.updateDeviceName(new_device_name)

    def updateDeviceName(self, new_device_name):
        valid_name = True
        for char in self.BAD_NAME_CHAR_LIST:
            if new_device_name.find(char) != -1:
                valid_name = False
        if valid_name is False:
            self.msg_if.pub_info("Received invalid device name update: " + new_device_name)
        else:
            self.status_msg.device_name = new_device_name
            self.publish_status(do_updates=False) # Updated inline here 
            self.node_if.set_param('device_name', new_device_name)
   
 
    def resetDeviceNameCb(self,msg):
        self.msg_if.pub_info("Recived update message: " + str(msg))
        self.resetDeviceName()

    def resetDeviceName(self):
        self.status_msg.device_name = self.device_name
        self.publish_status(do_updates=False) # Updated inline here
        self.node_if.set_param('device_name', self.device_name)

  
    def setSpeedRatioHandler(self, msg):
        if self.capabilities_report.has_adjustable_speed == True:
            speed_cur = self.getSpeedRatioCb()
            speed_ratio = msg.data
            self.msg_if.pub_warn("new speed ratio " + "%.2f" % speed_ratio)     
            self.msg_if.pub_warn("cur speed ratio " + "%.2f" % speed_cur)
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
        self.gotoTiltPosition(tilt_deg_adj)

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

    def setMountDescCb(self,msg):
        self.msg_if.pub_info("Recived set mount description message: " + str(msg))
        self.mount_desc = msg.data
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('mount_desc', self.mount_desc)

    def resetMountDescCb(self,msg):
        self.msg_if.pub_info("Recived reset mount description message: " + str(msg))
        self.mount_desc = 'None'
        self.publish_status(do_updates=False) # Updated inline here 
        self.node_if.set_param('mount_desc', self.mount_desc)



    def setFrame3dTransformCb(self, msg):
        self.msg_if.pub_info("Recived Frame Transform update message: " + str(msg))
        transform_msg = msg
        x = transform_msg.translate_vector.x
        y = transform_msg.translate_vector.y
        z = transform_msg.translate_vector.z
        roll = transform_msg.rotate_vector.x
        pitch = transform_msg.rotate_vector.y
        yaw = transform_msg.rotate_vector.z
        heading = transform_msg.heading_offset
        transform = [x,y,z,roll,pitch,yaw,heading]
        self.setFrame3dTransform(transform)


    def get_3d_transform(self):
        transform = nepi_nav.ZERO_TRANSFORM
        if self.transform_if is not None:
            transform = self.transform_if.get_3d_transform()
        return transform



    def get_navpose_dict(self):
        np_dict = copy.deepcopy(self.navpose_dict)
        return np_dict

        
    def publish_navpose(self):
        np_dict = self.get_navpose_dict()
        timestamp = nepi_utils.get_time()
        navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
        if self.node_if is not None and navpose_msg is not None:
            self.node_if.publish_pub('navpose_pub', navpose_msg)

    def navPoseUpdaterCb(self,timer):
        navpose_dict = None
        if navpose_dict is None:
            navpose_dict = copy.deepcopy(self.sys_navpose_dict)
        if navpose_dict is not None:
            output_frame_3d = 'nepi_frame'
        else:
            navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
            output_frame_3d = 'sensor_frame'
        transform = self.get_3d_transform()
        navpose_dict = nepi_nav.transform_navpose_dict(navpose_dict, transform, output_frame_3d = output_frame_3d)
        self.publish_navpose()
        self.frame_3d = output_frame_3d
        timestamp = nepi_utils.get_time()
        self.save_data_if.save('navpose',navpose_dict,timestamp = timestamp,save_check=True)
        self.navpose_dict = navpose_dict

    def get_mount_description(self):
        desc = self.device_mount_description
        if self.mount_desc != 'None':
            desc = self.mount_desc
        return desc

        

    def provideCapabilities(self, _):
        return self.capabilities_report
    

    def initConfig(self):
        self.initCb()

    def initCb(self, do_updates = False):
        if do_updates == True and self.node_if is not None:
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


        if self.node_if is not None:
            self.device_name = self.node_if.get_param('device_name')
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
            self.reverse_pan_enabled = self.node_if.get_param('reverse_pan_enabled')
            self.reverse_tilt_enabled = self.node_if.get_param('reverse_tilt_enabled')
        if do_updates == True:
            pass
        self.publish_status()

    def resetCb(self,do_updates = True):
        if self.node_if is not None:
            self.node_if.reset_params()
        if self.save_data_if is not None:
            self.save_data_if.reset()
        if self.settings_if is not None:
            self.settings_if.reset()
        if self.transform_if is not None:
            self.transform_if.reset()
        if self.navpose_if is not None:
            self.navpose_if.reset()
        if do_updates == True:
            pass
        self.initCb(do_updates = True)

    def factoryResetCb(self,do_updates = True):
        if self.node_if is not None:
            self.node_if.factory_reset_params()
        if self.save_data_if is not None:
            self.save_data_if.factory_reset()
        if self.settings_if is not None:
            self.settings_if.factory_reset()
        if self.transform_if is not None:
            self.transform_if.factory_reset()
        if self.navpose_if is not None:
            self.navpose_if.factory_reset()
        if do_updates == True:
            pass
        self.initCb(do_updates = True)



    def _publishStatusCb(self,timer):
        self.msg_if.pub_warn("will call publisher status msg ", throttle_s = 5.0)
        self.publish_status()
        # Publish navpose at same rate
        np_dict = copy.deepcopy(self.navpose_dict)
        timestamp = nepi_utils.get_time()
        navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
        if self.node_if is not None and navpose_msg is not None:
            self.node_if.publish_pub('navpose_pub', navpose_msg)
        self.save_data_if.save('navpose',np_dict,timestamp = timestamp,save_check=True)


    def publish_status(self, do_updates = False):
        self.msg_if.pub_warn("entering Pub_stat msg", throttle_s = 5.0)
        start_time = nepi_utils.get_time()
        self.status_msg.device_name = self.device_name
        self.status_msg.device_mount_description = self.get_mount_description()
        #self.msg_if.pub_info("Entering Publish Status", log_name_list = self.log_name_list)
        if self.has_absolute_positioning == True and self.getPositionCb is not None:
            [pan_deg,tilt_deg] = self.getPositionCb()
            #self.msg_if.pub_warn("Got PT position: " + str(pan_deg) + " : " + str(tilt_deg))

            if pan_deg != -999 and tilt_deg != -999:
                self.current_position = [pan_deg,tilt_deg]
    
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

        #self.status_msg.frame_3d = self.frame_3d
        transform = self.get_3d_transform()
        transform_msg = nepi_nav.convert_transform_list2msg(transform)
        transform_msg.source_ref_description = self.tr_source_ref_description
        transform_msg.end_ref_description = self.tr_end_ref_description
        self.status_msg.frame_3d_transform = transform_msg
        #self.msg_if.pub_debug("Created status msg: " + str(self.status_msg), throttle_s = 5.0)
        #self.msg_if.pub_debug("Publishing Status", log_name_list = self.log_name_list)
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub',self.status_msg)
        pub_time = nepi_utils.get_time() - start_time
        return pub_time


    def _publishStatusCb(self,timer):
        self.publish_status()

    def passFunction(self):
        return 0

