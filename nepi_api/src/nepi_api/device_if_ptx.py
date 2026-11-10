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
from nepi_sdk import nepi_nav


from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header


from nepi_interfaces.msg import DevicePTXStatus, PanTiltLimits, PanTiltPosition, SingleAxisTimedMove, SingleAxisTimedSpeedMove
from nepi_interfaces.srv import DevicePTXStatusQuery, DevicePTXStatusQueryRequest, DevicePTXStatusQueryResponse

from nepi_api.device_if import DeviceIF




class PTXActuatorIF(DeviceIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = DevicePTXStatus()
    DATA_MSG = None

    ID = 'ptx'
    DESCRIPTION = 'Pan and Tilt Device'

    DATA_SOURCE_DESCRIPTION = 'Device'
    DATA_REF_DESCRIPTION = 'Pan and Tilt Center Axis'

    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict( )

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

    MAX_STATUS_UPDATE_RATE = 3
    MIN_LIMIT_ANGLE = 10
    # Backup Factory Control Values 
    FACTORY_CONTROLS_DICT = {
                'reverse_pan_enabled' : False,
                'reverse_tilt_enabled' : False,
                'speed_ratio' : 0.5
    }


    orientation_dict = {
        'time_orientation': nepi_utils.get_time(),
        # Orientation should be provided in Degrees ENU
        'roll_deg': 0.0,
        'tilt_deg': 0.0,
        'pan_deg': 0.0,
    }

    data_products_list = []




    has_position_feedback = False
    has_absolute_positioning = False
    has_timed_positioning = False
    has_timed_speed_positioning =False
    has_seperate_pan_tilt_control = False
    has_adjustable_limits = False
    has_adjustable_speed = False
    has_seperate_pan_tilt_speed = False
    has_homing = False
    has_set_home = False
    has_limit_controls = False
    has_calibration = False
    

    move_decimal_place = 0
    report_decimal_place = 2


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

    has_calibration : False


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
    position_times = [0.0,0.0]
    speed_ratio = 0.5
    speed_pan_ratio = speed_ratio
    speed_tilt_ratio = speed_ratio
    set_max_dps_disabled = False
    speed_max_dps = 10

    # Intended continuous-motion direction per motor for the standard motor
    # command interface. 1 = clockwise (toward the max soft-limit angle),
    # -1 = counter-clockwise (toward the min soft-limit angle). Set by the
    # standard 'set_direction' command and used by 'go_direction'.
    cmd_dir_pan = 1
    cmd_dir_tilt = 1


    tr_source_ref_description = 'tilt_axis_center'
    tr_end_ref_description = 'nepi_frame'


    data_source_description = 'pan_tilt'
    data_ref_description = 'tilt_axis_center'
    data_end_description = 'nepi_frame'

    
    is_moving = False

    speed_pan_dps = 0
    pan_last_time = 0

    speed_tilt_dps = 0
    tilt_last_time = 0

    last_time = 0

    pan_pos_history = []
    tilt_pos_history = []
    pan_time_history = [0]
    tilt_time_history = [0]
    SPEED_SMOOTHING_WINDOW = 3
    delay = False




    ### IF Initialization
    def __init__(self,  device_info,
                 factoryControls , # Dictionary to be supplied by parent, specific key set is required
                 getSettingsFunction=None, setSettingFunction=None, 
                 factoryLimits = None,
                 data_source_description = 'pan_tilt',
                 data_ref_description = 'tilt_axis_center',

                 stopMovingCb = None, # Required; no args
                 movePanCb = None, # Required; direction and optional time args
                 moveTiltCb = None, # Required; direction and optional time args
                 movePanSpeedRatioCb = None, # Required; direction, speed ratio, and optional time args
                 moveTiltSpeedRatioCb = None, # Required; direction, speed ratio, and optional time args
                 setSoftLimitsCb=None,
                 getSoftLimitsCb=None,
                 getSpeedMaxCb = None,
                 setSpeedMaxCb = None,
                 setSpeedRatioCb=None, # None ==> No speed adjustment capability; Speed ratio arg
                 getSpeedRatioCb=None, # None ==> No speed adjustment capabilitiy; Returns speed ratio
                 setPanSpeedRatioCb=None,
                 setTiltSpeedRatioCb=None,
                 getPanSpeedRatioCb=None,
                 getTiltSpeedRatioCb=None,
                 getPositionCb=None,
                 getPositionTimesCb=None,
                 gotoPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 gotoPanPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 gotoTiltPositionCb=None, # None ==> No absolute positioning capability (pan_deg, tilt_deg, speed, float move_timeout_s) 
                 goHomeCb=None, # None ==> No native driver homing capability, can still use homing if absolute positioning is supported
                 setHomePositionCb=None, # None ==> No native driver home absolute setting capability, can still use it if absolute positioning is supported
                 setHomePositionHereCb=None, # None ==> No native driver home instant capture capability, can still use it if absolute positioning is supported
                 getNavPoseCb=None,
                 navpose_update_rate = 10,
                 deviceResetCb = None,
                 calibrateCenterCB = None,

                msg_if = None,
                node_if = None,
                settings_if = None,
                save_data_if = None,
                connect_navpose_if = None,
                ):

        ##############################
        # Instance Copies of Class State
        ##############################
        self.pan_pos_history = copy.deepcopy(self.pan_pos_history)
        self.tilt_pos_history = copy.deepcopy(self.tilt_pos_history)
        self.pan_time_history = copy.deepcopy(self.pan_time_history)
        self.tilt_time_history = copy.deepcopy(self.tilt_time_history)

     
        ##############################
        # Update Class Variables
        ##############################

        self.start_time = nepi_utils.get_time()
    

        # Create and update factory controls dictionary
        self.factory_controls_dict = copy.deepcopy(self.FACTORY_CONTROLS_DICT)
        if factoryControls is not None:
            controls = list(factoryControls.keys())
            for control in controls:
                if self.factory_controls_dict.get(control) != None and factoryControls.get(control) != None:
                    self.factory_controls_dict[control] = factoryControls[control]

        self.deviceResetCb = deviceResetCb


       # Configure PTX Capabilities

        # STOP MOVE #############
        self.stopMovingCb = stopMovingCb


        # GET POSITION #############
        self.getPositionCb = getPositionCb
        if self.getPositionCb is not None:
            self.has_position_feedback = True
            self.has_limit_controls = True
        self.getPositionTimesCb = getPositionTimesCb

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
        if self.movePanCb is not None or self.moveTiltCb is not None:
            self.has_timed_positioning = True

        # JOG MOVE at Speed Ratio ############
        self.movePanSpeedRatioCb = movePanSpeedRatioCb
        self.moveTiltSpeedRatioCb = moveTiltSpeedRatioCb
        if self.movePanSpeedRatioCb is not None or self.moveTiltSpeedRatioCb is not None:
            self.has_timed_speed_positioning = True

        # SPEED SETTINGS  #############
        self.setSpeedMaxCb = setSpeedMaxCb
        self.getSpeedMaxCb = getSpeedMaxCb
        if self.getSpeedMaxCb is not None:
            self.set_max_dps_disabled = True
            self.speed_max_dps = self.getSpeedMaxCb()


        self.setSpeedRatioCb = setSpeedRatioCb
        if setSpeedRatioCb is None:
            self.getSpeedRatioCb = self.getZeroCb #None
        else:
            self.setSpeedRatioCb = setSpeedRatioCb
        self.getSpeedRatioCb = getSpeedRatioCb
        if getSpeedRatioCb is not None and setSpeedRatioCb is not None:
            self.has_adjustable_speed = True

        # PER-AXIS SPEED SETTINGS  #############
        self.setPanSpeedRatioCb = setPanSpeedRatioCb
        self.setTiltSpeedRatioCb = setTiltSpeedRatioCb
        self.getPanSpeedRatioCb = getPanSpeedRatioCb
        self.getTiltSpeedRatioCb = getTiltSpeedRatioCb
        if (setPanSpeedRatioCb is not None and setTiltSpeedRatioCb is not None and
                getPanSpeedRatioCb is not None and getTiltSpeedRatioCb is not None):
            self.has_seperate_pan_tilt_speed = True


        # Homing  #############
        self.goHomeCb = goHomeCb
        self.setHomePositionCb = setHomePositionCb
        self.setHomePositionHereCb = setHomePositionHereCb

        if self.goHomeCb is not None:
            self.has_homing = True
        if self.setHomePositionHereCb is not None:
            self.has_set_home = True
               
        # Calibration  #############
        self.calibrateCenterCB = calibrateCenterCB
        if self.calibrateCenterCB is not None:
            self.has_calibration = True



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


        ########################       



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
                factoryControls = factoryControls, 
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
           'speed_max_dps': {
                'name': 'speed_max_dps',
                'namespace': self.namespace,
                'factory_val': self.speed_max_dps
            },
           'speed_ratios': {
                'name': 'speed_ratios',
                'namespace': self.namespace,
                'factory_val': [self.speed_pan_ratio,self.speed_tilt_ratio]
            },
           'home_position/pan_deg': {
                'name': 'home_position/pan_deg',
                'namespace': self.namespace,
                'factory_val': 0.0
            },
           'home_position/tilt_deg': {
                'name': 'home_position/tilt_deg',
                'namespace': self.namespace,
                'factory_val': 0.0
            },
           'reverse_pan_enabled': {
                'name': 'reverse_pan_enabled',
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['reverse_pan_enabled']
            },            
           'reverse_tilt_enabled': {
                'name': 'reverse_tilt_enabled',
                'namespace': self.namespace,
                'factory_val': self.factory_controls_dict['reverse_tilt_enabled']
            },
           'max_pan_softstop_deg': {
                'name': 'max_pan_softstop_deg',
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_pan_softstop_deg']
            },            
           'min_pan_softstop_deg': {
                'name': 'min_pan_softstop_deg',
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_pan_softstop_deg']
            },            
           'max_tilt_softstop_deg': {
                'name': 'max_tilt_softstop_deg',
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_tilt_softstop_deg']
            },           
           'min_tilt_softstop_deg': {
                'name': 'min_tilt_softstop_deg',
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_tilt_softstop_deg']
            },
           'move_decimal_place': {
                'name': 'move_decimal_place',
                'namespace': self.namespace,
                'factory_val': 0
            },
           'report_decimal_place': {
                'name': 'report_decimal_place',
                'namespace': self.namespace,
                'factory_val': 2
            }
        }

        if self.has_seperate_pan_tilt_speed:
            params_dict[self.node_if_prefix + 'speed_pan_ratio'] = {
                'name': 'speed_pan_ratio',
                'namespace': self.namespace,
                'factory_val': 0.5
            }
            params_dict[self.node_if_prefix + 'speed_tilt_ratio'] = {
                'name': 'speed_tilt_ratio',
                'namespace': self.namespace,
                'factory_val': 0.5
            }

    
        # Services Config Dict ####################
        services_dict = {
           'status_query': {
                'namespace': self.namespace,
                'topic': 'status_query',
                'srv': DevicePTXStatusQuery,
                'req': DevicePTXStatusQueryRequest(),
                'resp': DevicePTXStatusQueryResponse(),
                'callback': self.status_query_callback
            }
        }

        # Publishers Config Dict ####################
        pubs_dict = {}
                            

        # Subscribers Config Dict ####################
        subs_dict = {
           'set_speed_max_dps': {
                'namespace': self.namespace,
                'topic': 'set_speed_max_dps',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setMaxSpeedCb,
                'callback_args': ()
            },
           'speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setSpeedRatioCb,
                'callback_args': ()
            },
           'set_pan_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_pan_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setPanSpeedRatioCb,
                'callback_args': ()
            },
           'set_tilt_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_tilt_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setTiltSpeedRatioCb,
                'callback_args': ()
            },
           'stop_moving': {
                'namespace': self.namespace,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
                'callback': self._stopMovingCb, 
                'callback_args': ()
            },
           'stop_pan': {
                'namespace': self.namespace,
                'topic': 'stop_pan',
                'msg': Empty,
                'qsize': 1,
                'callback': self._stopPanCb, 
                'callback_args': ()
            },
           'stop_tilt': {
                'namespace': self.namespace,
                'topic': 'stop_tilt',
                'msg': Empty,
                'qsize': 1,
                'callback': self._stopTiltCb, 
                'callback_args': ()
            },
           'goto_to_position': {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self._gotoPositionCb, 
                'callback_args': ()
            },
           'goto_to_pan_position': {
                'namespace': self.namespace,
                'topic': 'goto_pan_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoPanPositionCb, 
                'callback_args': ()
            },
           'goto_to_tilt_position': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoTiltPositionCb, 
                'callback_args': ()
            },
           'goto_pan_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_pan_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoToPanRatioCb, 
                'callback_args': ()
            },
           'goto_tilt_ratio': {
                'namespace': self.namespace,
                'topic': 'goto_tilt_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoToTiltRatioCb, 
                'callback_args': ()
            },
           'jog_timed_pan': {
                'namespace': self.namespace,
                'topic': 'jog_timed_pan',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self._jogTimedPanCb, 
                'callback_args': ()
            },
           'jog_timed_tilt': {
                'namespace': self.namespace,
                'topic': 'jog_timed_tilt',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self._jogTimedTiltCb, 
                'callback_args': ()
            },
           'jog_timed_pan_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'jog_timed_pan_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
                'callback': self._jogTimedPanSpeedRatioCb, 
                'callback_args': ()
            },
           'jog_timed_tilt_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'jog_timed_tilt_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
                'callback': self._jogTimedTiltSpeedRatioCb, 
                'callback_args': ()
            },
           'reverse_pan_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_pan_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setReversePanEnableCb, 
                'callback_args': ()
            },
           'reverse_tilt_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_tilt_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setReverseTiltEnableCb, 
                'callback_args': ()
            },
           'set_soft_limits': {
                'namespace': self.namespace,
                'topic': 'set_soft_limits',
                'msg': PanTiltLimits,
                'qsize': 1,
                'callback': self._setSoftstopCb, 
                'callback_args': ()
            },
           'go_home': {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
                'callback': self._goHomeCb, 
                'callback_args': ()
            },
           'set_home_position': {
                'namespace': self.namespace,
                'topic': 'set_home_position',
                'msg': PanTiltPosition,
                'qsize': 1,
                'callback': self._setHomePositionCb, 
                'callback_args': ()
            },
           'set_home_position_here': {
                'namespace': self.namespace,
                'topic': 'set_home_position_here',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setHomePositionHereCb, 
                'callback_args': ()
            },
           'calibrate_center': {
                'namespace': self.namespace,
                'topic': 'calibrate_center',
                'msg': Empty,
                'qsize': 1,
                'callback': self._calibrateCenterCB,
                'callback_args': ()
            },
           'set_move_decimal_place': {
                'namespace': self.namespace,
                'topic': 'set_move_decimal_place',
                'msg': Int32,
                'qsize': 1,
                'callback': self._setMoveDecimalPlaceCb,
                'callback_args': ()
            },
           'set_report_decimal_place': {
                'namespace': self.namespace,
                'topic': 'set_report_decimal_place',
                'msg': Int32,
                'qsize': 1,
                'callback': self._setReportDecimalPlaceCb,
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

        self.status_msg.has_absolute_positioning = self.has_absolute_positioning
        self.status_msg.has_timed_positioning = self.has_timed_positioning
        self.status_msg.has_timed_speed_positioning = self.has_timed_speed_positioning
        self.status_msg.has_seperate_pan_tilt_control = self.has_seperate_pan_tilt_control
        self.status_msg.has_position_feedback = self.has_position_feedback
        self.status_msg.has_adjustable_speed = self.has_adjustable_speed
        self.status_msg.has_seperate_pan_tilt_speed = self.has_seperate_pan_tilt_speed
        self.status_msg.has_limit_controls = self.has_limit_controls

        self.status_msg.has_homing = self.has_homing
        self.status_msg.has_set_home = self.has_set_home
        self.status_msg.has_calibration = self.has_calibration

        ##############################
        # Start Node Processes

        # Periodic publishing
        status_update_rate = self.navpose_update_rate  
        if status_update_rate < 1:
            status_update_rate = 1
        if status_update_rate > 10:
            status_update_rate = 10
        self.status_update_rate = status_update_rate
        self.msg_if.pub_info("Starting pt status publisher at hz: " + str(self.status_update_rate))    
        status_pub_delay = float(1.0) / self.status_update_rate
        nepi_sdk.start_timer_process(status_pub_delay, self._publishPTXStatusCb)
        ####################################
        self.ready = True
        self.msg_if.pub_info("Initialization Complete" + ": " + self.node_name, color = 'GREEN')
        ####################################

        ##############################
        # Start NodeIF Processes
        if type(self) is PTXActuatorIF:
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
   
        if self.has_absolute_positioning == True and self.getPositionCb is not None:
            
            
            cur_time = nepi_utils.get_time()
            last_time = copy.deepcopy(self.pan_last_time)

            status_msg.move_decimal_place = self.move_decimal_place
            status_msg.report_decimal_place = self.report_decimal_place

            [pan_deg ,tilt_deg] = self.getPositionCb()
            #self.msg_if.pub_warn("Got PT position: " + str(pan_deg) + " : " + str(tilt_deg))
            
            if self.getPositionTimesCb is not None:
                self.position_times = self.getPositionTimesCb()
                #self.msg_if.pub_warn("Got Position Times: " + str(self.position_times))
            else:
                self.position_times = [cur_time,cur_time]
            #self.msg_if.pub_warn("Using PT position times: " + str(self.position_times))
            place = self.report_decimal_place
            pan_deg = nepi_utils.get_sign(pan_deg) * (math.floor( abs(pan_deg * 10**(place)) ) / 10**(place))
            tilt_deg = nepi_utils.get_sign(tilt_deg) * (math.floor( abs(tilt_deg * 10**(place)) ) / 10**(place))
            self.current_position = [pan_deg ,tilt_deg]
            #self.msg_if.pub_warn("Checking for time change: " + str([self.position_times[0],self.pan_time_history[-1]]))
            if self.position_times[0] != self.pan_time_history[-1]:
                self.pan_pos_history.append(pan_deg)
                self.pan_time_history.append(self.position_times[0])
                if len(self.pan_pos_history) > self.SPEED_SMOOTHING_WINDOW:
                    self.pan_pos_history.pop(0)
                    self.pan_time_history.pop(0)
                if len(self.pan_pos_history) >= 2:
                    #self.msg_if.pub_warn("Getting Pan Speed from pos: " + str(self.pan_pos_history))
                    #self.msg_if.pub_warn("Getting Pan Speed from times: " + str(self.pan_time_history))
                    speeds = []
                    for i in range(len(self.pan_pos_history) - 1):
                        delta_time = self.pan_time_history[i+1] - self.pan_time_history[i]
                        speed = (self.pan_pos_history[i+1] - self.pan_pos_history[i]) / delta_time
                        speeds.append(speed)
                    self.speed_pan_dps = sum(speeds) / len(speeds)
                    #self.msg_if.pub_warn("Got Pan Speed: " + str(self.speed_pan_dps))

            if self.position_times[1] != self.tilt_time_history[-1]:
                self.tilt_pos_history.append(tilt_deg)
                self.tilt_time_history.append(self.position_times[1])
                if len(self.tilt_pos_history) > self.SPEED_SMOOTHING_WINDOW:
                    self.tilt_pos_history.pop(0)
                    self.tilt_time_history.pop(0)
                if len(self.tilt_pos_history) >= 2:
                    speeds = []
                    for i in range(len(self.tilt_pos_history) - 1):
                        delta_time = self.tilt_time_history[i+1] - self.tilt_time_history[i]
                        speed = (self.tilt_pos_history[i+1] - self.tilt_pos_history[i]) / delta_time
                        speeds.append(speed)
                    self.speed_tilt_dps = sum(speeds) / len(speeds)

            #self.msg_if.pub_warn("Using PT position: " + str(self.current_position[0]) + " : " + str(self.current_position[1]))

            status_msg.speed_pan_dps = abs(self.speed_pan_dps)
            status_msg.speed_tilt_dps = abs(self.speed_tilt_dps)


            pan_now_deg_adj = self.getPanAdj(pan_deg)
            status_msg.pan_now_deg = pan_now_deg_adj

            if self.pan_goal_deg == -999:
                pan_goal = pan_now_deg_adj
            else:
                pan_goal = self.getPanAdj(self.pan_goal_deg)
            pan_goal_deg_adj = pan_goal
            status_msg.pan_goal_deg = pan_goal_deg_adj
            status_msg.pan_home_pos_deg = self.getPanAdj(self.home_pan_deg)

            tilt_now_deg_adj = self.getTiltAdj(tilt_deg)
            status_msg.tilt_now_deg = tilt_now_deg_adj

            if self.tilt_goal_deg == -999:
                tilt_goal = tilt_now_deg_adj
            else:
                tilt_goal = self.getTiltAdj(self.tilt_goal_deg)
            tilt_goal_deg_adj = tilt_goal
            status_msg.tilt_goal_deg = tilt_goal_deg_adj
            status_msg.tilt_home_pos_deg = self.getTiltAdj(self.home_tilt_deg)


            [adj_pan_hs_min,adj_pan_hs_max,adj_tilt_hs_min,adj_tilt_hs_max] = self.getLimitsHardstopAdj()
            [adj_pan_ss_min,adj_pan_ss_max,adj_tilt_ss_min,adj_tilt_ss_max] = self.getLimitsSoftstopAdj()

            status_msg.pan_min_hardstop_deg = adj_pan_hs_min
            status_msg.pan_max_hardstop_deg = adj_pan_hs_max
            status_msg.pan_min_softstop_deg = adj_pan_ss_min
            status_msg.pan_max_softstop_deg = adj_pan_ss_max

            status_msg.tilt_min_hardstop_deg = adj_tilt_hs_min
            status_msg.tilt_max_hardstop_deg = adj_tilt_hs_max
            status_msg.tilt_min_softstop_deg = adj_tilt_ss_min
            status_msg.tilt_max_softstop_deg = adj_tilt_ss_max
            

            axis_info = [pan_now_deg_adj,pan_goal_deg_adj,adj_pan_ss_min,adj_pan_ss_max]
            #self.msg_if.pub_warn("Using Pan now,goal,min,max: " + str(axis_info) , throttle_s = 2.0)        
            pan_now_ratio_adj =  1 - (pan_now_deg_adj - adj_pan_ss_min) / (adj_pan_ss_max - adj_pan_ss_min) 
            status_msg.pan_now_ratio = pan_now_ratio_adj
            pan_goal_ratio_adj =  1 - (pan_goal_deg_adj - adj_pan_ss_min) / (adj_pan_ss_max - adj_pan_ss_min)
            status_msg.pan_goal_ratio = pan_goal_ratio_adj

            axis_info = [tilt_now_deg_adj,tilt_goal_deg_adj,adj_tilt_ss_min,adj_tilt_ss_max]
            #self.msg_if.pub_warn("Using Tilt now,goal,min,max: " + str(axis_info) , throttle_s = 2.0)    
            tilt_now_ratio_adj =  1 - (tilt_now_deg_adj - adj_tilt_ss_min) / (adj_tilt_ss_max - adj_tilt_ss_min) 
            status_msg.tilt_now_ratio = tilt_now_ratio_adj
            tilt_goal_ratio_adj =  1 - (tilt_goal_deg_adj - adj_tilt_ss_min) / (adj_tilt_ss_max - adj_tilt_ss_min)
            status_msg.tilt_goal_ratio = tilt_goal_ratio_adj

            pan_changed = self.last_position[0] != self.current_position[0]
            tilt_changed = self.last_position[1] != self.current_position[1]

            self.pan_last_time = copy.deepcopy(cur_time)
            self.last_position = copy.deepcopy(self.current_position)

            status_msg.is_moving = pan_changed or tilt_changed


        status_msg.reverse_pan_enabled = self.reverse_pan_enabled
        status_msg.reverse_tilt_enabled = self.reverse_tilt_enabled

        # Re-read the max here rather than trusting the single sample taken in __init__.
        # getSpeedMaxCb can legitimately report a placeholder before the driver's own
        # devices have connected -- ptx_servos_node returns a fallback until its SVX
        # interfaces come up -- and sampling once at construction latched that placeholder
        # permanently. Because supplying getSpeedMaxCb also sets set_max_dps_disabled,
        # nothing downstream could correct it: the reported max was both wrong and locked.
        # Only a positive value is accepted, so a transient None or 0 cannot poison it.
        if self.getSpeedMaxCb is not None:
            speed_max_dps = self.getSpeedMaxCb()
            if speed_max_dps is not None and speed_max_dps > 0:
                self.speed_max_dps = speed_max_dps

        status_msg.set_max_dps_disabled = self.set_max_dps_disabled
        status_msg.speed_max_dps = self.speed_max_dps
        status_msg.speed_ratio = self.speed_ratio
        status_msg.has_seperate_pan_tilt_speed = self.has_seperate_pan_tilt_speed
        status_msg.speed_pan_ratio = self.speed_pan_ratio
        status_msg.speed_tilt_ratio = self.speed_tilt_ratio

        status_msg.data_products = self.data_products_list

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



        return status_msg




  

    def initCb(self, do_updates = False):
        if do_updates == True and self.node_if is not None:
            # This one comes from the parent
            if self.getSoftLimitsCb is not None:
                    [min_pan,max_pan,min_tilt,max_tilt] = self.getSoftLimitsCb()
                    if min_pan != -999:
                        self.min_pan_softstop_deg = min_pan
                        self.node_if.set_param(self.node_if_prefix + 'min_pan_softstop_deg', self.min_pan_softstop_deg)
                    if max_pan != -999:
                        self.max_pan_softstop_deg = max_pan
                        self.node_if.set_param(self.node_if_prefix + 'max_pan_softstop_deg',self.max_pan_softstop_deg)

                    if min_tilt != -999:
                        self.min_tilt_softstop_deg = min_tilt
                        self.node_if.set_param(self.node_if_prefix + 'min_tilt_softstop_deg', self.min_tilt_softstop_deg)
                    if max_tilt != -999:
                        self.max_tilt_softstop_deg = max_tilt
                        self.node_if.set_param(self.node_if_prefix + 'max_tilt_softstop_deg',self.max_tilt_softstop_deg)




        if self.node_if is not None:
            self.min_pan_softstop_deg =  self.node_if.get_param(self.node_if_prefix + 'min_pan_softstop_deg')
            self.max_pan_softstop_deg = self.node_if.get_param(self.node_if_prefix + 'max_pan_softstop_deg')

            self.min_tilt_softstop_deg = self.node_if.get_param(self.node_if_prefix + 'min_tilt_softstop_deg')
            self.max_tilt_softstop_deg = self.node_if.get_param(self.node_if_prefix + 'max_tilt_softstop_deg')

            if None not in [self.min_pan_softstop_deg,self.max_pan_softstop_deg,self.min_tilt_softstop_deg,self.max_tilt_softstop_deg]:
                if self.setSoftLimitsCb is not None:
                    self.setSoftLimitsCb(self.min_pan_softstop_deg,
                                        self.max_pan_softstop_deg,
                                        self.min_tilt_softstop_deg,
                                        self.max_tilt_softstop_deg)
            if self.getSpeedMaxCb is None:
                self.speed_max_dps = self.node_if.get_param(self.node_if_prefix + 'speed_max_dps')
            if self.setSpeedMaxCb is not None:
                self.setSpeedMaxCb(self.speed_max_dps) 

            # Restore on the same predicates the three write paths are gated on:
            # _setSpeedRatio writes when has_adjustable_speed is True, and
            # _setPanSpeedRatio/_setTiltSpeedRatio write when
            # has_seperate_pan_tilt_speed is True. Gating the restore on
            # has_adjustable_speed == False was the exact inverse, so on the units
            # where the operator can move the speed it was saved and never read back.
            # It also left speed_ratios assigned under one guard and consumed under a
            # sibling guard -- a NameError on any driver with per-axis speed callbacks.
            if self.has_adjustable_speed == True or self.has_seperate_pan_tilt_speed == True:
                speed_ratios = self.node_if.get_param(self.node_if_prefix + 'speed_ratios')
                if speed_ratios is not None and len(speed_ratios) == 2:
                    self.speed_pan_ratio = speed_ratios[0]
                    self.speed_tilt_ratio = speed_ratios[1]
                    self.speed_ratio = max(speed_ratios)
                    self.msg_if.pub_debug("Initializing speed_ratios: " + str(speed_ratios))
                    if self.has_seperate_pan_tilt_speed == True:
                        self.setPanSpeedRatioCb(self.speed_pan_ratio)
                        self.setTiltSpeedRatioCb(self.speed_tilt_ratio)
                    elif self.setSpeedRatioCb is not None:
                        self.setSpeedRatioCb(self.speed_ratio)

            self.home_pan_deg = self.node_if.get_param(self.node_if_prefix + 'home_position/pan_deg')
            self.home_tilt_deg = self.node_if.get_param(self.node_if_prefix + 'home_position/tilt_deg')

            move_decimal_place = self.node_if.get_param(self.node_if_prefix + 'move_decimal_place')
            if move_decimal_place is not None:
                self.move_decimal_place = move_decimal_place
            report_decimal_place = self.node_if.get_param(self.node_if_prefix + 'report_decimal_place')
            if report_decimal_place is not None:
                self.report_decimal_place = report_decimal_place

            # self.goHome()

        
    
            # Restore the reverse flags BEFORE deriving the direction multipliers from
            # them. Reading the params after this derivation left rpi/rti computed from
            # the pre-restore value while status_msg reported the restored one, so a
            # restored reverse axis read correct on the page and still moved the wrong
            # way.
            reverse_pan_enabled = self.node_if.get_param(self.node_if_prefix + 'reverse_pan_enabled')
            if reverse_pan_enabled is not None:
                self.reverse_pan_enabled = reverse_pan_enabled
            reverse_tilt_enabled = self.node_if.get_param(self.node_if_prefix + 'reverse_tilt_enabled')
            if reverse_tilt_enabled is not None:
                self.reverse_tilt_enabled = reverse_tilt_enabled

            # Set reverse int values
            rpi = 1
            if self.reverse_pan_enabled:
                rpi = -1
            self.rpi = rpi
            rti = 1
            if self.reverse_tilt_enabled:
                rti = -1
            self.rti = rti
        if do_updates == True:
            pass
        self.publish_status()


    ###############################
    # Class Methods
    ###############################


    def getPanAdj(self,pan_deg):
        if pan_deg is None:
            pan_deg = 0.0
        return pan_deg * self.rpi

    def getPanRatioAdj(self,ratio):
        if self.reverse_pan_enabled == True:
            ratio = 1 - ratio
        return ratio

    def getTiltAdj(self,tilt_deg):
        if tilt_deg is None:
            tilt_deg = 0.0
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
            [x,y,z] = nepi_nav.rotate_enu_point([x,y,z], pan_deg, axis = 'z') 

        return x,y,z



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
        [adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max] = self.getLimitsSoftstopAdj()
        if (pan_deg <= adj_pan_max) or (pan_deg >= adj_pan_min) or \
           (tilt_deg <= adj_tilt_max) or (tilt_deg >= adj_tilt_min):
            valid = True
        
        return valid


    def getPositionConditioned(self, pan_deg, tilt_deg):
        [adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max] = self.getLimitsSoftstopAdj()
        #self.msg_if.pub_info("Checking Positions against soft limits: " + str([pan_deg, tilt_deg]) )
        #self.msg_if.pub_info("Got Soft Limits: " + str([adj_pan_min,adj_pan_max,adj_tilt_min,adj_tilt_max]) )
        if (pan_deg > adj_pan_max):
            pan_deg = adj_pan_max
        if (pan_deg < adj_pan_min):
            pan_deg = adj_pan_min
        if (tilt_deg > adj_tilt_max):
            tilt_deg = adj_tilt_max
        if (tilt_deg < adj_tilt_min):
            tilt_deg = adj_tilt_min

        place = self.move_decimal_place
        pan_deg = nepi_utils.get_sign(pan_deg) * (math.floor( abs(pan_deg * 10**(place)) ) / 10**(place))
        tilt_deg = nepi_utils.get_sign(tilt_deg) * (math.floor( abs(tilt_deg * 10**(place)) ) / 10**(place))
        return pan_deg,tilt_deg



 
    def _setSoftstopCb(self, msg):
        min_pan = msg.min_pan_deg
        max_pan = msg.max_pan_deg
        min_tilt = msg.min_tilt_deg
        max_tilt = msg.max_tilt_deg

        [min_pan_adj,max_pan_adj,min_tilt_adj,max_tilt_adj] = self.getLimitsAdj(min_pan,max_pan,min_tilt,max_tilt)

        valid = False
        if min_pan_adj >= self.min_pan_hardstop_deg and max_pan_adj <= self.max_pan_hardstop_deg and min_pan_adj < max_pan_adj:  
            if min_tilt_adj >= self.min_tilt_hardstop_deg and max_tilt_adj <= self.max_tilt_hardstop_deg and min_tilt_adj < max_tilt_adj:
                if abs(max_pan_adj - min_pan_adj) >= self.MIN_LIMIT_ANGLE and abs(max_tilt_adj - min_tilt_adj) >= self.MIN_LIMIT_ANGLE:
                    valid = True

                    self.min_pan_softstop_deg =  min_pan_adj
                    self.max_pan_softstop_deg = max_pan_adj
                    self.min_tilt_softstop_deg = min_tilt_adj
                    self.max_tilt_softstop_deg = max_tilt_adj

                    if None not in [min_pan_adj,max_pan_adj,min_tilt_adj,max_tilt_adj]:
                        if self.setSoftLimitsCb is not None:
                            self.setSoftLimitsCb(min_pan_adj,max_pan_adj,min_tilt_adj,max_tilt_adj)

                        self.node_if.set_param(self.node_if_prefix + 'max_pan_softstop_deg', max_pan_adj)
                        self.node_if.set_param(self.node_if_prefix + 'min_pan_softstop_deg', min_pan_adj)
                        self.node_if.set_param(self.node_if_prefix + 'max_tilt_softstop_deg', max_tilt_adj)
                        self.node_if.set_param(self.node_if_prefix + 'min_tilt_softstop_deg', min_tilt_adj)


        if valid == False:
            self.msg_if.pub_warn("Invalid softstop requested " + str(msg))
        
  
    def _stopMovingCb(self, _):
        self.msg_if.pub_warn("Got Stop Moving All msg")
        self.stopPanTilt('All')

    def _stopPanCb(self, _):
        self.msg_if.pub_warn("Got Stop Moving Pan msg")
        self.stopPanTilt('pan')

    def _stopTiltCb(self, _):
        self.msg_if.pub_warn("Got Stop Moving Tilt msg")
        self.stopPanTilt('tilt')

    def stopPanTilt(self,axis = 'All'):
        self.msg_if.pub_warn("Stopping Move for axis: " + str(axis))
        if axis == 'pan' or axis == 'All':
            pan_stop_deg = self.status_msg.pan_now_deg * self.rpi
        else:
            pan_stop_deg = self.status_msg.pan_goal_deg * self.rpi
        if axis == 'tilt' or axis == 'All':
            tilt_stop_deg = self.status_msg.tilt_now_deg * self.rpi
        else:
            tilt_stop_deg = self.status_msg.tilt_goal_deg * self.rpi

        self.msg_if.pub_info("Stopping motion by request" )
        if self.stopMovingCb is not None:
            self.stopMovingCb()
        elif self.gotoPositionCb is not None:
            self.gotoPositionCb(pan_stop_deg,tilt_stop_deg)
        self.publish_status()



    def _setMaxSpeedCb(self, msg):
            max_speed = msg.data
            if (max_speed > 0.0):
                
                if self.setSpeedMaxCb is not None:
                    self.setSpeedMaxCb(max_speed) 
                    self.speed_max_dps = max_speed
                self.publish_status()
                self.node_if.set_param(self.node_if_prefix + 'speed_max_dps',max_speed)
                self.msg_if.pub_warn("Updated max speed to " + str(max_speed))

    def _setSpeedRatioCb(self, msg):
        ratio = nepi_utils.check_ratio(msg.data)
        self._setSpeedRatio(ratio)

    def _setSpeedRatio(self,speed_ratio):
        if self.has_adjustable_speed == True:
            speed_cur = math.floor(self.getSpeedRatioCb())
            
            self.msg_if.pub_warn("new speed ratio " + "%.2f" % speed_ratio)
            self.msg_if.pub_warn("cur speed ratio " + "%.2f" % speed_cur)

            self.speed_ratio = speed_ratio
            self.speed_pan_ratio = speed_ratio
            self.speed_tilt_ratio = speed_ratio
            self.publish_status()
            
            self.setSpeedRatioCb(speed_ratio)
            self.node_if.set_param(self.node_if_prefix + 'speed_ratios',[speed_ratio,speed_ratio])
    
        

    def _setPanSpeedRatioCb(self, msg):
        ratio = nepi_utils.check_ratio(msg.data)
        self._setPanSpeedRatio(ratio)

    def _setPanSpeedRatio(self,speed_ratio):
        if self.has_seperate_pan_tilt_speed == True:
            speed_cur = math.floor(self.getPanSpeedRatioCb())
            if speed_cur != speed_ratio and self.setPanSpeedRatioCb is not None:
                self.speed_pan_ratio = speed_ratio
                self.setPanSpeedRatioCb(speed_ratio)
                self.speed_ratio = max([speed_ratio,self.speed_tilt_ratio])
                self.node_if.set_param(self.node_if_prefix + 'speed_ratios', [speed_ratio,self.speed_tilt_ratio])
                self.publish_status()
           

    def _setTiltSpeedRatioCb(self, msg):
        ratio = nepi_utils.check_ratio(msg.data)
        self._setTiltSpeedRatio(ratio)

    def _setTiltSpeedRatio(self,speed_ratio):
        if self.has_seperate_pan_tilt_speed == True:
            speed_cur = math.floor(self.getTiltSpeedRatioCb())
            if speed_cur != speed_ratio and self.setTiltSpeedRatioCb is not None:
                self.speed_tilt_ratio = speed_ratio
                self.setTiltSpeedRatioCb(speed_ratio)
                self.speed_ratio = max([self.speed_pan_ratio,speed_ratio])
                self.node_if.set_param(self.node_if_prefix + 'speed_ratios', [self.speed_pan_ratio,speed_ratio])
                self.publish_status()

    def _setHomePositionCb(self, msg):
        [pan_deg_adj,tilt_deg_adj] = self.getPanTiltAdj(msg.pan_deg, msg.tilt_deg)
        if not self.positionWithinSoftLimits(pan_deg_adj, tilt_deg_adj):
            self.msg_if.pub_warn("Requested home position is invalid... ignoring" )
            return

        self.home_pan_deg = pan_deg_adj
        self.home_tilt_deg = tilt_deg_adj
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'home_position/pan_deg', self.home_pan_deg)
            self.node_if.set_param(self.node_if_prefix + 'home_position/tilt_deg', self.home_tilt_deg)
        self.msg_if.pub_info("Updated home position to " + "%.2f" % self.home_pan_deg + " " + "%.2f" %  self.home_tilt_deg)
        self.publish_status()
        if self.setHomePositionCb is not None:
            # Driver supports absolute positioning, so just let it operate
            self.setHomePositionCb(self.home_pan_deg, self.home_tilt_deg)
        else:
            self.msg_if.pub_warn("Absolution position home setpoints not available... ignoring" )
            return
        
        
            

    def _goHomeCb(self, _):
        self.msg_if.pub_warn("Got Go Home msg")
        self.goHome()

    def goHome(self):
        if self.gotoPositionCb is not None:
            self.pan_goal_deg = self.home_pan_deg
            self.tilt_goal_deg = self.home_tilt_deg
            self.gotoPositionCb(self.pan_goal_deg,self.tilt_goal_deg)
            self.publish_status()
        elif self.goHomeCb is not None:
            self.goHomeCb()
        

    def _gotoPositionCb(self, msg):
        #self.msg_if.pub_warn("Got goto position msg: " + str(msg))
        [pan_deg_adj,tilt_deg_adj] = self.getPanTiltAdj(msg.pan_deg,msg.tilt_deg)
        self.gotPosition(pan_deg_adj,tilt_deg_adj)

    def gotPosition(self,pan_deg,tilt_deg):
            [pan_deg,tilt_deg] = self.getPositionConditioned(pan_deg, tilt_deg)
            self.pan_goal_deg = pan_deg
            self.tilt_goal_deg = tilt_deg
            #self.msg_if.pub_info("Driving to Pan/Tilt deg position  " + "%.2f" % (pan_deg * self.rpi) + " " + "%.2f" % (tilt_deg * self.rti))
            self.gotoPositionCb(pan_deg, tilt_deg)
            self.publish_status()

    def _gotoPanPositionCb(self, msg):
        #self.msg_if.pub_warn("Got goto pan position msg: " + str(msg))
        pan_deg_adj = self.getPanAdj(msg.data)
        self.gotoPanPosition(pan_deg_adj)

    def gotoPanPosition(self,pan_deg):
        [pan_deg,tilt_deg] = self.getPositionConditioned(pan_deg, self.tilt_goal_deg)
        self.pan_goal_deg = pan_deg
        if self.gotoPanPositionCb is not None:
            #self.msg_if.pub_info("Driving to pan deg" + "%.2f" % (pan_deg * self.rpi))
            self.gotoPanPositionCb(self.pan_goal_deg)
        elif self.gotoPositionCb is not None:
            #self.msg_if.pub_info("Driving to pan/tilt deg " + "%.2f" % pan_deg * self.rpi + " " + "%.2f" % tilt_deg * self.rti)
            self.gotoPositionCb(pan_deg, tilt_deg)
        self.publish_status()
                 

    def _gotoTiltPositionCb(self, msg):
        #self.msg_if.pub_warn("Got goto tilt position msg: " + str(msg))
        tilt_deg_adj = self.getTiltAdj(msg.data)
        #self.msg_if.pub_warn("Got adj goto tilt position msg: " + str(tilt_deg_adj))
        self.gotoTiltPosition(tilt_deg_adj)

    def gotoTiltPosition(self,tilt_deg):
        [pan_deg,tilt_deg] = self.getPositionConditioned(self.pan_goal_deg, tilt_deg)
        self.tilt_goal_deg = tilt_deg
        if self.gotoTiltPositionCb is not None:
            #self.msg_if.pub_info("Driving to tilt deg " + "%.2f" % (tilt_deg * self.rti))
            self.gotoTiltPositionCb(self.tilt_goal_deg)    
        elif self.gotoPositionCb is not None:
            #self.msg_if.pub_info("Driving to pan/tilt deg " + "%.2f" % pan_deg * self.rpi + " " + "%.2f" % tilt_deg * self.rti)
            self.gotoPositionCb(pan_deg, tilt_deg)
        self.publish_status()
    

    def _gotoToPanRatioCb(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid pan position ratio " + "%.2f" % ratio)
            return
        self.pan_goal_deg = self.panRatioToDeg(ratio) # Function takes care of reverse conversion
        if self.gotoPanPositionCb is not None:
            #self.msg_if.pub_info("Driving to pan ratio - pan deg:  " + str(self.pan_goal_deg * self.rpi))
            self.msg_if.pub_info("Calling gotoPanPositionCb with deg :" + str(self.pan_goal_deg))
            self.gotoPanPositionCb(self.pan_goal_deg)
        elif self.gotoPositionCb is not None:
            tilt_goal_deg = self.getTiltAdj(self.tilt_goal_deg)
            #self.msg_if.pub_info("Driving to pan ratio - pan/tilt deg  " + "%.2f" % self.pan_goal_deg * self.rpi + " " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoPositionCb(self.pan_goal_deg, tilt_goal_deg)
        self.publish_status()
        

    def _gotoToTiltRatioCb(self, msg):
        #self.msg_if.pub_warn("Got go to tilt ratio - tilt deg  " + "%.2f" % self.tilt_goal_deg * self.rti)
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid tilt position ratio " + "%.2f" % ratio)
            return
        self.tilt_goal_deg = self.tiltRatioToDeg(ratio) # Function takes care of reverse conversion
        if self.gotoTiltPositionCb is not None:
            #self.msg_if.pub_info("Driving to tilt ratio - tilt deg  " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoTiltPositionCb(self.tilt_goal_deg)
        elif self.gotoPositionCb is not None:
            
            self.pan_goal_deg = self.getPanAdj(self.status_msg.pan_now_deg)
            #self.msg_if.pub_info("Driving to tilt ratio - pan/tilt deg  " + "%.2f" % self.pan_goal_deg * self.rpi + " " + "%.2f" % self.tilt_goal_deg * self.rti)
            self.gotoPositionCb(self.pan_goal_deg, self.tilt_goal_deg)
        self.publish_status()
        

    def _jogTimedPanCb(self, msg):
        #self.msg_if.pub_warn("Got job pan msg: " + str(msg))
        if self.movePanCb is not None:
            self.pan_goal_deg = -999
            direction = msg.direction * self.rpi
            time_s = msg.duration_s
            self.movePanCb(direction,  time_s)
            #self.msg_if.pub_info("Jogging pan" )
        self.publish_status()
        

    def _jogTimedTiltCb(self, msg):
        #self.msg_if.pub_warn("Got job tilt msg: " + str(msg))
        if self.moveTiltCb is not None:
            self.tilt_goal_deg = -999
            direction = msg.direction * self.rti
            time_s = msg.duration_s
            self.moveTiltCb(direction, time_s)
            #self.msg_if.pub_info("Jogging tilt" )
        self.publish_status()

    def _jogTimedPanSpeedRatioCb(self, msg):
        #self.msg_if.pub_warn("Got job pan msg: " + str(msg))
        if self.movePanSpeedRatioCb is not None:
            self.pan_goal_deg = -999
            direction = msg.direction * self.rpi
            speed_ratio = nepi_utils.check_ratio(msg.speed_ratio)
            time_s = msg.duration_s
            self.movePanSpeedRatioCb(direction, speed_ratio, time_s)
            #self.msg_if.pub_info("Jogging pan" )
        self.publish_status()
        

    def _jogTimedTiltSpeedRatioCb(self, msg):
        #self.msg_if.pub_warn("Got job tilt msg: " + str(msg))
        if self.moveTiltSpeedRatioCb is not None:
            self.tilt_goal_deg = -999
            direction = msg.direction * self.rti
            speed_ratio = nepi_utils.check_ratio(msg.speed_ratio)
            time_s = msg.duration_s
            self.moveTiltSpeedRatioCb(direction, speed_ratio, time_s)
            #self.msg_if.pub_info("Jogging tilt at ratio " )
        self.publish_status()
        

    def _setReversePanEnableCb(self, msg):
        self.reverse_pan_enabled = msg.data
        rpi = 1
        if msg.data == True:
            rpi = -1
        self.rpi = rpi
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'reverse_pan_enabled', self.reverse_pan_enabled)
        self.msg_if.pub_info("Set pan control to reverse=" + str(self.reverse_pan_enabled))
        self.publish_status()


    def _setReverseTiltEnableCb(self, msg):
        self.reverse_tilt_enabled = msg.data
        rti = 1
        if msg.data == True:
            rti = -1
        self.rti = rti
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'reverse_tilt_enabled', self.reverse_tilt_enabled)
        self.msg_if.pub_info("Set tilt control to reverse=" + str(self.reverse_tilt_enabled))
        self.publish_status()
        

    def _setHomePositionHereCb(self, _):
        self.home_pan_deg = self.status_msg.pan_now_deg * self.rpi
        self.home_tilt_deg = self.status_msg.tilt_now_deg * self.rti
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'home_position/pan_deg', self.home_pan_deg)
            self.node_if.set_param(self.node_if_prefix + 'home_position/tilt_deg', self.home_tilt_deg)
        self.msg_if.pub_info("Home positon set to: " + "%.2f" % self.home_pan_deg * self.rpi + " " + "%.2f" % self.home_tilt_deg * self.rti)
        self.publish_status()
        if self.setHomePositionHereCb is not None:
            self.setHomePositionHereCb()
          
    def _calibrateCenterCB(self,msg):
        if self.calibrateCenterCB is not None:
            self.calibrateCenterCB()

    def _setMoveDecimalPlaceCb(self, msg):
        val = msg.data
        if val < 0:
            self.msg_if.pub_warn("Invalid move_decimal_place requested: " + str(val))
            return
        self.move_decimal_place = val
        self.node_if.set_param(self.node_if_prefix + 'move_decimal_place', val)
        self.msg_if.pub_info("Updated move_decimal_place to " + str(val))

    def _setReportDecimalPlaceCb(self, msg):
        val = msg.data
        if val < 0:
            self.msg_if.pub_warn("Invalid report_decimal_place requested: " + str(val))
            return
        self.report_decimal_place = val
        self.node_if.set_param(self.node_if_prefix + 'report_decimal_place', val)
        self.msg_if.pub_info("Updated report_decimal_place to " + str(val))


    def _goPanDirection(self, direction):
        target_deg = self.max_pan_softstop_deg if direction >= 0 else self.min_pan_softstop_deg
        self.msg_if.pub_info("Driving pan continuously toward soft-limit deg " + str(target_deg))
        self.gotoPanPosition(target_deg)

    def _goTiltDirection(self, direction):
        target_deg = self.max_tilt_softstop_deg if direction >= 0 else self.min_tilt_softstop_deg
        self.msg_if.pub_info("Driving tilt continuously toward soft-limit deg " + str(target_deg))
        self.gotoTiltPosition(target_deg)

    

    def status_query_callback(self, _):
        """Service handler that returns the PTX capabilities report.

        Returns:
            DevicePTXStatusQueryResponse: The current DevicePTXStatus in device_status,
                describing which PTX capabilities are supported, including absolute
                positioning, timed positioning, separate pan/tilt control, position
                feedback, adjustable speed, limit controls, homing, set-home, and
                calibration.
        """
        return DevicePTXStatusQueryResponse(device_status = self.status_msg)
    
  


    def getZeroCb(self):
        return 0




    def passFunction(self):
        return 0

    def _publishPTXStatusCb(self, timer):
        self.publish_status()