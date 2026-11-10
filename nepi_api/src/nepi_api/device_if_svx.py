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

# SVX (devicesvx) device interface.
# The generic single-devicesvx interface: one SVX instance drives exactly one devicesvx.
# Coordinating multiple devicesvxs into a higher-level rig is the application's job,
# not this interface's.
# Open loop: position is reported as the pos_last commanded value; nothing is measured.
# The API speaksrees. Conversion to devicesvx pulse width is a driver concern.

import os
import time
import copy
import math

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64, Header




from nepi_interfaces.msg import DeviceSVXStatus
from nepi_interfaces.srv import DeviceSVXStatusQuery, DeviceSVXStatusQueryRequest, DeviceSVXStatusQueryResponse

from nepi_interfaces.msg import SingleAxisLimits, SingleAxisTimedMove, SingleAxisTimedSpeedMove


from nepi_api.device_if import DeviceIF




class DeviceSVXIF(DeviceIF):

    #########################
    ## DATA PRODUCT IF Required
    ## Class Variables Updated by DataProductIF Init Class
    #########################

    STATUS_MSG = DeviceSVXStatus()
    DATA_MSG = None

    ID = 'svx'
    DESCRIPTION = 'Servo Device'

    DATA_SOURCE_DESCRIPTION = 'Servo Device'
    DATA_REF_DESCRIPTION = 'Device Ref Point'

    #Factory Control Values 
    FACTORY_CONTROLS_DICT = dict(

    )


####################
## Single DeviceSVX  Status msg

#################
# Capabilities
#################


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

    data_products_list = []

    is_spin = True
    is_rotate = False
    is_linear = False
    is_continous = False

    mode_options = []
    getModeOption = None
    setModeOption  = None
    mode_name = ''
    mode_index = 0

    connected = False

    ##########################
    # Capabilities

    has_modes = False

    has_direction_control = False

    has_speed_feedback = False
    has_speed_control = False     
    has_set_max_speed = False

    has_position_feedback = False        
    has_position_control = False  

    has_jog_control = False
    has_jog_speed_control = False

    has_goto_control = False          
    has_stop_control = False 
    has_go_home_control = False           
    has_set_home_control = False      

    has_limit_controls = False

    has_calibration = False

    has_pan_control = False
    has_tilt_control = False

    has_connected_control = True

    caps_dict = dict()
    ##########################

    #################
    # DeviceSVX Data
    #################
    move_decimal_place = 0
    report_decimal_place = 2

    ##### Move Direction #####
    ### Direction for Rotational Axis is 1/-1 (+/-) right-hand rule around ENU axis
    ### Direction for Linear Axis are 1/-1 (+/-) along ENU axis
    reverse = False
    reverse_int = 1

    dir_now = 1

    pos_home = 0.0
    pos_now = 0.0
    pos_last = 0
    pos_goal = 0.0

    max_hardstop = 0
    min_hardstop = 0

    max_softstop = 0
    min_softstop = 0

    current_position = 0.0
    last_position = current_position
    position_times = 0.0
    pos_last_time = 0

    speed_max = 10
    set_max_disabled = False
    speed_now = 0
    speed_ratio = 0.5

    last_time = 0

    pos_history = []

    pos_time_history = [0]
   
    SPEED_SMOOTHING_WINDOW = 3
    MIN_LIMIT_ANGLE = 10
    delay = False



    #######################
    ### IF Initialization    
    def __init__(self, 
                 device_info,                  
                 data_source_description = None,
                 data_ref_description = None,

                 getSettingsFunction=None, setSettingFunction=None, 

                 factoryControls = None, 
                 factoryLimits = None,


                mode_options = [],
                getModeOption = None,
                setModeOption  = None,

                is_spin = True,
                is_rotate = False,
                is_linear = False,
                is_continous = False,

                has_pan_control = False,
                has_tilt_control = False,
                always_connected = False,

                 get_direction = None,
                 set_direction = None,

                 stopMovingCb = None, # Required; no args

                 moveCb = None, # Required; direction and optional time args
                 moveSpeedRatioCb = None, # Required; direction, speed_now ratio, and optional time args

                 setSoftLimitsCb=None,
                 getSoftLimitsCb=None,

                 getSpeedMaxCb = None,
                 setSpeedMaxCb = None,

                 setSpeedRatioCb=None, # None ==> No speed_now adjustment capability; Speed ratio arg
                 getSpeedRatioCb=None, # None ==> No speed_now adjustment capabilitiy; Returns speed_now ratio

                 getPositionCb=None,
                 getPositionTimesCb=None,

                 gotoPositionCb=None, # None ==> No absolute positioning capability (pos,  speed_now, float move_timeout_s) 
            
                 goHomeCb=None, # None ==> No native driver homing capability, can still use homing if absolute positioning is supported
                 setHomePositionCb=None, # None ==> No native driver pos_home absolute setting capability, can still use it if absolute positioning is supported
                 setHomePositionHereCb=None, # None ==> No native driver pos_home instant capture capability, can still use it if absolute positioning is supported

                 getNavPoseCb=None,
                 navpose_update_rate = 10,
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
        self.caps_dict = copy.deepcopy(self.caps_dict)
        self.pos_history = copy.deepcopy(self.pos_history)
        self.pos_time_history = copy.deepcopy(self.pos_time_history)


        ##############################
        # Update Class Variables
        ##############################
        ##############################
        # Initialize Class Variables
        self.start_time = nepi_utils.get_time()

    
        self.is_spin = False
        self.is_rotate = False
        self.is_linear = False
        if is_linear == True:
            self.is_linear = True
        elif is_rotate == True:
            self.is_rotate = True
        elif is_spin == True:
            self.is_spin = True
        else:
            self.is_rotate = True

        self.is_continous = is_continous


        self.getModeOption = getModeOption
        self.setModeOption = setModeOption
        if self.getModeOption is not None and self.setModeOption is not None and len(mode_options) > 0:
            self.mode_options = mode_options
            self.has_modes = True        

       # Configure SVX  Capabilities





        self.get_direction = get_direction
        self.set_direction = set_direction
        if set_direction is not None:
            self.has_direction_control = True

        # STOP MOVE #############
        self.stopMovingCb = stopMovingCb
        if stopMovingCb is not None:
            self.has_stop_control = True


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



        # Soft Limits are handled by SVX  IF at top level, 
        # these are for updating the hardware if required
        self.setSoftLimitsCb = setSoftLimitsCb
        self.getSoftLimitsCb = getSoftLimitsCb


        # POSITION MOVE ############
        self.gotoPositionCb = gotoPositionCb
        if self.gotoPositionCb is not None:
            self.has_position_control = True
            self.has_goto_control = True

        # SPEED FEEDBACK ############
        # speed_now is measured from the position history in the status update,
        # which runs only with both position control and position feedback
        if self.has_position_control == True and self.getPositionCb is not None:
            self.has_speed_feedback = True
    

        # JOG MOVE ############
        self.moveCb = moveCb
        if self.moveCb is not None:
            self.has_jog_control = True

        # JOG MOVE at Speed Ratio ############
        self.moveSpeedRatioCb = moveSpeedRatioCb
        if self.moveSpeedRatioCb is not None:
            self.has_jog_speed_control = True

        # SPEED SETTINGS  #############
        self.setSpeedMaxCb = setSpeedMaxCb
        if self.setSpeedMaxCb is not None:
            self.has_set_max_speed = True
        self.getSpeedMaxCb = getSpeedMaxCb
        if self.getSpeedMaxCb is not None:
            self.set_max_disabled = True
            self.speed_max = self.getSpeedMaxCb()


        self.setSpeedRatioCb = setSpeedRatioCb
        if setSpeedRatioCb is None:
            self.getSpeedRatioCb = self.passFunction #None
        else:
            self.setSpeedRatioCb = setSpeedRatioCb
        self.getSpeedRatioCb = getSpeedRatioCb
        if getSpeedRatioCb is not None and setSpeedRatioCb is not None:
            self.has_speed_control = True




        # Homing  #############
        self.goHomeCb = goHomeCb
        self.setHomePositionCb = setHomePositionCb
        self.setHomePositionHereCb = setHomePositionHereCb

        if self.goHomeCb is not None:
            self.has_go_home_control = True
        if self.setHomePositionHereCb is not None:
            self.has_set_home_control = True
               
        # Calibration  #############
        self.calibrateCenterCB = calibrateCenterCB
        if self.calibrateCenterCB is not None:
            self.has_calibration = True



        #######################################
        # Set up factory limits

        if factoryLimits is not None:
            self.factoryLimits = factoryLimits  
        else:
            self.factoryLimits = dict()
            # Hard limits
            self.factoryLimits['min_hardstop'] = 0
            self.factoryLimits['max_hardstop'] = 0
  
            # Soft limits
            self.factoryLimits['min_softstop'] = 0
            self.factoryLimits['max_softstop'] = 0

        self.min_hardstop =  self.factoryLimits['min_hardstop']
        self.max_hardstop = self.factoryLimits['max_hardstop']

        self.min_softstop =  self.factoryLimits['min_softstop']
        self.max_softstop = self.factoryLimits['max_softstop']

        self.has_pan_control = has_pan_control
        self.has_tilt_control = has_tilt_control

        # On a multi-channel controller the operator marks which channels have a servo
        # in use (saved). A driver whose servos are always present, such as a fixed
        # pan-tilt, passes always_connected: it reports connected and has no toggle.
        self.has_connected_control = (always_connected == False)
        if always_connected == True:
            self.connected = True

        ###########################
        # Update Capabilities
        self.caps_dict['has_modes'] = self.has_modes

        self.caps_dict['has_direction_control'] = self.has_direction_control

        self.caps_dict['has_speed_feedback'] = self.has_speed_feedback
        self.caps_dict['has_speed_control'] = self.has_speed_control
        self.caps_dict['has_set_max_speed'] = self.has_set_max_speed

        self.caps_dict['has_position_feedback'] = self.has_position_feedback
        self.caps_dict['has_position_control'] = self.has_position_control

        self.caps_dict['has_goto_control'] = self.has_goto_control   
        self.caps_dict['has_stop_control'] = self.has_stop_control  
        self.caps_dict['has_go_home_control'] = self.has_go_home_control  
        self.caps_dict['has_set_home_control'] = self.has_set_home_control  
        self.caps_dict['has_limit_controls'] = self.has_limit_controls  

        self.caps_dict['has_calibration'] = self.has_calibration  

        self.caps_dict['has_pan_control'] = self.has_pan_control
        self.caps_dict['has_tilt_control'] = self.has_tilt_control

        self.caps_dict['has_connected_control'] = self.has_connected_control


        ######################################



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
           'mode_name': {
                'name': 'mode_name',
                'namespace': self.namespace,
                'factory_val': self.mode_name
            },
           'reverse': {
                'name': 'reverse',
                'namespace': self.namespace,
                'factory_val': self.reverse
            },
           'speed_max': {
                'name': 'speed_max',
                'namespace': self.namespace,
                'factory_val': self.speed_max
            },
           'speed_max': {
                'name': 'speed_max',
                'namespace': self.namespace,
                'factory_val': self.speed_max
            },
           'speed_ratio': {
                'name': 'speed_ratio',
                'namespace': self.namespace,
                'factory_val': self.speed_ratio
            },
           'home_position': {
                'name': 'home_position',
                'namespace': self.namespace,
                'factory_val': 0.0
            },
           'max_softstop': {
                'name': 'max_softstop',
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['max_softstop']
            },            
           'min_softstop': {
                'name': 'min_softstop',
                'namespace': self.namespace,
                'factory_val': self.factoryLimits['min_softstop']
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
        if self.has_connected_control == True:
            params_dict['servo_connected'] = {
                'name': 'servo_connected',
                'namespace': self.namespace,
                'factory_val': False
            }

    
        # Services Config Dict ####################
        services_dict = {
            'status_query': {
                'namespace': self.namespace,
                'topic': 'status_query',
                'srv': DeviceSVXStatusQuery,
                'req': DeviceSVXStatusQueryRequest(),
                'resp': DeviceSVXStatusQueryResponse(),
                'callback': self.status_query_callback
            }
        }

        # Publishers Config Dict ####################
        pubs_dict = {}
                            

        # Subscribers Config Dict ####################
        subs_dict =  {           
            'stop_moving': {
                'namespace': self.namespace,
                'topic': 'stop_moving',
                'msg': Empty,
                'qsize': 1,
                'callback': self._stopMovingCb, 
                'callback_args': ()
            },
           'set_reverse_enabled': {
                'namespace': self.namespace,
                'topic': 'set_reverse_enable',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setReverseEnableCb, 
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

        if self.has_direction_control == True:
           subs_dict['set_direction'] = {
                'namespace': self.namespace,
                'topic': 'set_direction',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setDirectionCb,
                'callback_args': ()
            }


        if self.has_set_max_speed == True:
           subs_dict['set_speed_max'] = {
                'namespace': self.namespace,
                'topic': 'set_speed_max',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setMaxSpeedCb,
                'callback_args': ()
            }
           
        if self.has_speed_control == True:
           subs_dict['set_speed_ratio'] = {
                'namespace': self.namespace,
                'topic': 'set_speed_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setSpeedRatioCb,
                'callback_args': ()
            }
           
        if self.has_position_control == True:
           subs_dict['goto_to_position'] = {
                'namespace': self.namespace,
                'topic': 'goto_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoPositionCb, 
                'callback_args': ()
            }
           subs_dict['goto_ratio'] = {
                'namespace': self.namespace,
                'topic': 'goto_ratio',
                'msg': Float32,
                'qsize': 1,
                'callback': self._gotoPosRatioCb, 
                'callback_args': ()
            }
           
        if self.has_jog_control == True:
           subs_dict['jog_timed'] = {
                'namespace': self.namespace,
                'topic': 'jog_timed',
                'msg': SingleAxisTimedMove,
                'qsize': 1,
                'callback': self._jogTimedPosCb, 
                'callback_args': ()
            }
           
        if self.has_jog_speed_control == True:
           subs_dict['jog_timed_speed_ratio'] = {
                'namespace': self.namespace,
                'topic': 'jog_timed_speed_ratio',
                'msg': SingleAxisTimedSpeedMove,
                'qsize': 1,
                'callback': self._jogTimedSpeedRatioCb, 
                'callback_args': ()
            }


        if self.has_limit_controls == True:    
           subs_dict['set_soft_limits'] = {
                'namespace': self.namespace,
                'topic': 'set_soft_limits',
                'msg': SingleAxisLimits,
                'qsize': 1,
                'callback': self._setSoftstopCb, 
                'callback_args': ()
            }

        if self.has_go_home_control == True:    
           subs_dict['go_home'] = {
                'namespace': self.namespace,
                'topic': 'go_home',
                'msg': Empty,
                'qsize': 1,
                'callback': self._goHomeCb, 
                'callback_args': ()
            }

        
        if self.has_set_home_control == True:               
           subs_dict['set_home_position'] = {
                'namespace': self.namespace,
                'topic': 'set_home_position',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setHomePositionCb, 
                'callback_args': ()
            }

           subs_dict['set_home_position_here'] = {
                'namespace': self.namespace,
                'topic': 'set_home_position_here',
                'msg': Empty,
                'qsize': 1,
                'callback': self._setHomePositionHereCb, 
                'callback_args': ()
            }

        if self.has_calibration == True:  
           subs_dict['calibrate_center'] = {
                'namespace': self.namespace,
                'topic': 'calibrate_center',
                'msg': Empty,
                'qsize': 1,
                'callback': self._calibrateCenterCB,
                'callback_args': ()
            }
           
        if self.has_pan_control == True:               
           subs_dict['set_pan_angle'] = {
                'namespace': self.namespace,
                'topic': 'set_pan_angle',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setPanAngleCb, 
                'callback_args': ()
            }
        if self.has_tilt_control == True:
           subs_dict['set_tilt_angle'] = {
                'namespace': self.namespace,
                'topic': 'set_tilt_angle',
                'msg': Float32,
                'qsize': 1,
                'callback': self._setTiltAngleCb,
                'callback_args': ()
            }
        if self.has_connected_control == True:
           subs_dict['set_servo_connected'] = {
                'namespace': self.namespace,
                'topic': 'set_servo_connected',
                'msg': Bool,
                'qsize': 1,
                'callback': self._setServoConnectedCb,
                'callback_args': ()
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

        self.data_source_description = self.data_source_description
        self.data_ref_description = self.data_ref_description


        self.status_msg.is_spin = self.is_spin
        self.status_msg.is_rotate = self.is_rotate
        self.status_msg.is_linear = self.is_linear
        self.status_msg.is_continous = self.is_continous

        self.status_msg.mode_options = self.mode_options
        self.status_msg.mode_name = self.mode_name



        self.status_msg.has_direction_control = self.has_direction_control

        self.status_msg.has_speed_feedback = self.has_speed_feedback
        self.status_msg.has_speed_control = self.has_speed_control
        self.status_msg.has_set_max_speed = self.has_set_max_speed
        self.status_msg.has_position_feedback = self.has_position_feedback
        self.status_msg.has_position_control = self.has_position_control
        self.status_msg.has_goto_control = self.has_goto_control   
        self.status_msg.has_stop_control = self.has_stop_control  
        self.status_msg.has_go_home_control = self.has_go_home_control  
        self.status_msg.has_set_home_control = self.has_set_home_control  
        self.status_msg.has_limit_controls = self.has_limit_controls  


        self.status_msg.has_calibration = self.has_calibration  

        self.status_msg.has_pan_control = self.has_pan_control  
        self.status_msg.has_tilt_control = self.has_tilt_control  

        self.status_msg.min_hard_limit = self.min_hardstop
        self.status_msg.max_hard_limit = self.max_hardstop

        ##############################
        # Start Node Processes
        ##############################
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
        nepi_sdk.start_timer_process(status_pub_delay, self._publishSVXStatusCb)


        ####################################
        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete", color = 'GREEN')
        ####################################

        ##############################
        # Start NodeIF Processes
        if type(self) is DeviceSVXIF:
            self.start()


    ###############################
    # NodeIF Class Override Methods
    ###############################


    def update_status_msg(self, status_msg):
        """Builds and publishes the DeviceSVXStatus message to the status topic.


        Args:
            do_updates (bool, optional): Reserved for future use. Defaults to False.

        Returns:
            float: Elapsed time in seconds spent building and publishing the status.
        """
   
        if self.has_position_control == True and self.getPositionCb is not None:
            
            
            cur_time = nepi_utils.get_time()
            last_time = copy.deepcopy(self.pos_last_time)

            status_msg.move_decimal_place = self.move_decimal_place
            status_msg.report_decimal_place = self.report_decimal_place

            
            try:
                self.dir_now = self.get_direction() * self.reverse_int
            except:
                self.dir_now = 0
            status_msg.dir_now = self.dir_now

            pos = self.getPositionCb()
            
            if self.getPositionTimesCb is not None:
                self.position_times = self.getPositionTimesCb()
                #self.msg_if.pub_warn("Got Position Times: " + str(self.position_times))
            else:
                self.position_times = cur_time
            #self.msg_if.pub_warn("Using position times: " + str(self.position_times))
            place = self.report_decimal_place
            pos = nepi_utils.get_sign(pos) * (math.floor( abs(pos * 10**(place)) ) / 10**(place))
            self.current_position = pos
            #self.msg_if.pub_warn("Checking for time change: " + str([self.position_times,self.pos_time_history[-1]]))
            if self.position_times != self.pos_time_history[-1]:
                self.pos_history.append(pos)
                self.pos_time_history.append(self.position_times)
                if len(self.pos_history) > self.SPEED_SMOOTHING_WINDOW:
                    self.pos_history.pop(0)
                    self.pos_time_history.pop(0)
                if len(self.pos_history) >= 2:
                    #self.msg_if.pub_warn("Getting Pos Speed from pos: " + str(self.pos_history))
                    #self.msg_if.pub_warn("Getting Pos Speed from times: " + str(self.pos_time_history))
                    speeds = []
                    for i in range(len(self.pos_history) - 1):
                        delta_time = self.pos_time_history[i+1] - self.pos_time_history[i]
                        speed_now = (self.pos_history[i+1] - self.pos_history[i]) / delta_time
                        speeds.append(speed_now)
                    self.speed_now = sum(speeds) / len(speeds)
                    #self.msg_if.pub_warn("Got Pos Speed: " + str(self.speed_now))


            #self.msg_if.pub_warn("Using position: " + str(self.current_position)))

            status_msg.speed_now = abs(self.speed_now)
        

            pos_now_adj = self.getPosAdj(pos)
            status_msg.pos_now = pos_now_adj

            if self.pos_goal == -999:
                pos_goal = pos_now_adj
            else:
                pos_goal = self.getPosAdj(self.pos_goal)
            pos_goal_adj = pos_goal
            status_msg.pos_goal = pos_goal_adj
            status_msg.pos_home = self.getPosAdj(self.pos_home)


            [adj_pos_hs_min,adj_pos_hs_max] = self.getLimitsHardstopAdj()
            [adj_pos_ss_min,adj_pos_ss_max] = self.getLimitsSoftstopAdj()

            status_msg.min_hard_limit = adj_pos_hs_min
            status_msg.max_hard_limit = adj_pos_hs_max
            status_msg.min_soft_limit = adj_pos_ss_min
            status_msg.max_soft_limit = adj_pos_ss_max

     
            pos_now_ratio_adj =  1 - (pos_now_adj - adj_pos_ss_min) / (adj_pos_ss_max - adj_pos_ss_min) 
            status_msg.pos_now_ratio = pos_now_ratio_adj
            pos_goal_ratio_adj =  1 - (pos_goal_adj - adj_pos_ss_min) / (adj_pos_ss_max - adj_pos_ss_min)
            status_msg.pos_goal_ratio = pos_goal_ratio_adj

            pos_changed = self.last_position != self.current_position
            status_msg.is_moving = pos_changed

            self.pos_last_time = copy.deepcopy(cur_time)
            self.last_position = copy.deepcopy(self.current_position)

        status_msg.reverse_enabled = self.reverse

        status_msg.connected = self.connected
        status_msg.has_connected_control = self.has_connected_control


        # Re-read the max here rather than trusting the single sample taken in __init__.
        # getSpeedMaxCb can legitimately report a placeholder before the driver's own
        # devices have connected -- SVX_servos_node returns a fallback until its SVX
        # interfaces come up -- and sampling once at construction latched that placeholder
        # permanently. Because supplying getSpeedMaxCb also sets set_max_disabled,
        # nothing downstream could correct it: the reported max was both wrong and locked.
        # Only a positive value is accepted, so a transient None or 0 cannot poison it.
        if self.getSpeedMaxCb is not None:
            speed_max = self.getSpeedMaxCb()
            if speed_max is not None and speed_max > 0:
                self.speed_max = speed_max

        status_msg.set_max_disabled = self.set_max_disabled
        status_msg.speed_max = self.speed_max
        status_msg.speed_ratio = self.speed_ratio
        status_msg.speed_ratio = self.speed_ratio

        status_msg.data_products = self.data_products_list

        ###################
        # TOPIC UPDATES
        
        ########
        # TRANSFORM (Pan Tilt Offsets) DATA
        try:
           transform_dict = self.transform_if.get_transform_dict()
           pan_angle = transform_dict['yaw_deg_offset']
           tilt_angle = transform_dict['pitch_deg_offset']
        except:
           pan_angle = 0
           tilt_angle = 0
        status_msg.pan_deg = pan_angle
        status_msg.tilt_deg = tilt_angle


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
                    [min,max] = self.getSoftLimitsCb()
                    if min != -999:
                        self.min_softstop = min
                        self.node_if.set_param(self.node_if_prefix + 'min_softstop', self.min_softstop)
                    if max != -999:
                        self.max_softstop = max
                        self.node_if.set_param(self.node_if_prefix + 'max_softstop',self.max_softstop)




        if self.node_if is not None:
            self.min_softstop =  self.node_if.get_param(self.node_if_prefix + 'min_softstop')
            self.max_softstop = self.node_if.get_param(self.node_if_prefix + 'max_softstop')

            if None not in [self.min_softstop,self.max_softstop]:
                if self.setSoftLimitsCb is not None:
                    self.setSoftLimitsCb(self.min_softstop,
                                        self.max_softstop,
                                  )
            if self.getSpeedMaxCb is None:
                self.speed_max = self.node_if.get_param(self.node_if_prefix + 'speed_max')
            if self.setSpeedMaxCb is not None:
                self.setSpeedMaxCb(self.speed_max) 

        
            if self.has_speed_control == True:
                speed_ratio = self.node_if.get_param(self.node_if_prefix + 'speed_ratio')
                if speed_ratio is not None:
                    self.speed_ratio = speed_ratio
                    self.msg_if.pub_debug("Initializing speed_ratio: " + str(speed_ratio))
                    # A spin servo's speed ratio is its run command: restore it into the
                    # instance only, so a config init/reset never starts the motor.
                    if self.is_spin == False:
                        self.setSpeedRatioCb(self.speed_ratio)

            self.pos_home = self.node_if.get_param(self.node_if_prefix + 'home_position')

            if self.has_connected_control == True:
                servo_connected = self.node_if.get_param(self.node_if_prefix + 'servo_connected')
                if servo_connected is not None:
                    self.connected = servo_connected


            move_decimal_place = self.node_if.get_param(self.node_if_prefix + 'move_decimal_place')
            if move_decimal_place is not None:
                self.move_decimal_place = move_decimal_place
            report_decimal_place = self.node_if.get_param(self.node_if_prefix + 'report_decimal_place')
            if report_decimal_place is not None:
                self.report_decimal_place = report_decimal_place



        
    
            # Restore the reverse flags BEFORE deriving the direction multipliers from
            # them. Reading the params after this derivation left reverse_int/rti computed from
            # the pre-restore value while status_msg reported the restored one, so a
            # restored reverse axis read correct on the page and still moved the wrong
            # way.
            reverse = self.node_if.get_param(self.node_if_prefix + 'reverse')
            if reverse is not None:
                self.reverse = reverse
    
            # Set reverse int values
            reverse_int = 1
            if self.reverse:
                reverse_int = -1
            self.reverse_int = reverse_int
          
        if do_updates == True:
            pass
        self.publish_status()


    ###############################
    # Class Methods
    ###############################


    def getPosAdj(self,pos):
        if pos is None:
            pos = 0.0
        return pos * self.reverse_int

    def getPosRatioAdj(self,ratio):
        if self.reverse == True:
            ratio = 1 - ratio
        return ratio

    def getPosMinMaxAdj(self,min,max):
        if self.reverse == True:
            adj_min = copy.deepcopy(max) * -1
            adj_max = copy.deepcopy(min) * -1
        else:
            adj_min = min
            adj_max = max
        return adj_min,adj_max

    def getLimitsAdj(self,pos_min,pos_max):
        [adj_min,adj_max] = self.getPosMinMaxAdj(pos_min,pos_max)
        return adj_min,adj_max
    
    def getLimitsHardstopAdj(self):
        pos_min = self.min_hardstop
        pos_max = self.max_hardstop
        [adj_min,adj_max] = self.getPosMinMaxAdj(pos_min,pos_max)
        return adj_min,adj_max

    def getLimitsSoftstopAdj(self):
        pos_min = self.min_softstop
        pos_max = self.max_softstop
        [adj_min,adj_max] = self.getPosMinMaxAdj(pos_min,pos_max)


        return adj_min,adj_max




    def posRatioToDeg(self, ratio):
        pos = 0
        if self.reverse == False:
           pos =  self.min_softstop + (1-ratio) * (self.max_softstop - self.min_softstop)
        else:
           pos =  self.max_softstop - (1-ratio)  * (self.max_softstop - self.min_softstop)
        return  pos
    
    def posDegToRatio(self, deg):
        ratio = 0.5
        if self.reverse == False:
          ratio = 1 - (deg - self.min_softstop) / (self.max_softstop - self.min_softstop)
        else:
          ratio = (deg - self.min_softstop) / (self.max_softstop - self.min_softstop)
        return (ratio)


    def positionWithinSoftLimits(self, pos):
        valid = False
        [adj_min,adj_max] = self.getLimitsSoftstopAdj()
        if (pos <= adj_max) and (pos >= adj_min):
            valid = True
        
        return valid


    def getPositionConditioned(self, pos):
        [adj_min,adj_max] = self.getLimitsSoftstopAdj()
        #self.msg_if.pub_info("Checking Positions against soft limits: " + str(pos) )
        #self.msg_if.pub_info("Got Soft Limits: " + str([adj_min,adj_max]) )
        if (pos > adj_max):
            pos = adj_max
        if (pos < adj_min):
            pos = adj_min
    

        place = self.move_decimal_place
        pos = nepi_utils.get_sign(pos) * (math.floor( abs(pos * 10**(place)) ) / 10**(place))
    
        return pos



 
    def _setSoftstopCb(self, msg):
        min = msg.min
        max = msg.max


        [min_adj,max_adj] = self.getLimitsAdj(min,max)

        valid = False
        if min_adj >= self.min_hardstop and max_adj <= self.max_hardstop and min_adj < max_adj:  
                if abs(max_adj - min_adj) >= self.MIN_LIMIT_ANGLE >= self.MIN_LIMIT_ANGLE:
                    valid = True

                    self.min_softstop =  min_adj
                    self.max_softstop = max_adj

                    if None not in [min_adj,max_adj]:
                        if self.setSoftLimitsCb is not None:
                            self.setSoftLimitsCb(min_adj,max_adj)

                        self.node_if.set_param(self.node_if_prefix + 'max_softstop', max_adj)
                        self.node_if.set_param(self.node_if_prefix + 'min_softstop', min_adj)


        if valid == False:
            self.msg_if.pub_warn("Invalid softstop requested " + str(msg))
        
  
    def _stopMovingCb(self, _):
        self.stop_moving()


    def stop_moving(self):
        pos_stop = self.status_msg.pos_now * self.reverse_int
    
        self.msg_if.pub_info("Stopping motion by request" )
        if self.stopMovingCb is not None:
            self.stopMovingCb()
        elif self.gotoPositionCb is not None:
            self.gotoPositionCb(pos_stop)
        self.publish_status()



    def _setDirectionCb(self, msg):
            dir = 1
            if msg.data < 0:
                dir = -1
            if self.set_direction is not None:
                self.dir_now = dir
                self.set_direction(dir) 
            self.publish_status()
            self.node_if.set_param(self.node_if_prefix + 'dir_now',self.dir_now)




    def _setMaxSpeedCb(self, msg):
            max_speed = msg.data
            if (max_speed > 0.0):
                
                if self.setSpeedMaxCb is not None:
                    self.setSpeedMaxCb(max_speed) 
                    self.speed_max = max_speed
                self.publish_status()
                self.node_if.set_param(self.node_if_prefix + 'speed_max',max_speed)
                self.msg_if.pub_warn("Updated max speed_now to " + str(max_speed))

    def _setSpeedRatioCb(self, msg):
        ratio = nepi_utils.check_ratio(msg.data)
        self._setSpeedRatio(ratio)

    def _setSpeedRatio(self,speed_ratio):
        speed_cur = self.getSpeedRatioCb()
        if speed_cur != speed_ratio and self.setSpeedRatioCb is not None:
            self.speed_ratio = speed_ratio
            self.setSpeedRatioCb(speed_ratio)
            self.speed_ratio = speed_ratio
            self.node_if.set_param(self.node_if_prefix + 'speed_ratio', speed_ratio)
            self.publish_status()
           


    def _setHomePositionCb(self, msg):
        pos_adj = self.getPosAdj(msg.data)
        if not self.positionWithinSoftLimits(pos_adj):
            self.msg_if.pub_warn("Requested pos_home position is invalid... ignoring" )
            return

        self.pos_home = pos_adj
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'home_position', self.pos_home)
        self.msg_if.pub_info("Updated pos_home position to " + "%.2f" % self.pos_home)
        self.publish_status()
        if self.setHomePositionCb is not None:
            # Driver supports absolute positioning, so just let it operate
            self.setHomePositionCb(self.pos_home)
        else:
            self.msg_if.pub_warn("Absolution position pos_home setpoints not available... ignoring" )
            return
        
        
            

    def _goHomeCb(self, _):
        self.msg_if.pub_warn("Got Go Home msg")
        self.go_home()

    def go_home(self):
        if self.gotoPositionCb is not None:
            self.pos_goal = self.pos_home
            self.gotoPositionCb(self.pos_goal)
            self.publish_status()
        elif self.goHomeCb is not None:
            self.goHomeCb()
        

    def _gotoPositionCb(self, msg):
        #self.msg_if.pub_warn("Got goto position msg: " + str(msg))
        pos_adj = self.getPosAdj(msg.data)
        self.gotoPosition(pos_adj)

    def gotoPosition(self,pos):
            pos = self.getPositionConditioned(pos)
            self.pos_goal = pos
            #self.msg_if.pub_info("Driving to Pos/Tilt position  " + "%.2f" % (pos * self.reverse_int))
            self.gotoPositionCb(pos)
            self.publish_status()                 

    

    def _gotoPosRatioCb(self, msg):
        ratio = msg.data
        if (ratio < 0.0 or ratio > 1.0):
            self.msg_if.pub_warn("Invalid pos position ratio " + "%.2f" % ratio)
            return
        self.pos_goal = self.posRatioToDeg(ratio) # Function takes care of reverse conversion
        if self.gotoPositionCb is not None:
            #self.msg_if.pub_info("Driving to pos ratio - pos:  " + str(self.pos_goal * self.reverse_int))
            self.msg_if.pub_info("Calling gotoPositionCb with :" + str(self.pos_goal))
            self.gotoPositionCb(self.pos_goal)
        self.publish_status()
        


    def _jogTimedPosCb(self, msg):
        #self.msg_if.pub_warn("Got job pos msg: " + str(msg))
        if self.moveCb is not None:
            self.pos_goal = -999
            direction = msg.direction * self.reverse_int
            time_s = msg.duration_s
            self.moveCb(direction,  time_s)
            #self.msg_if.pub_info("Jogging pos" )
        self.publish_status()
        


    def _jogTimedSpeedRatioCb(self, msg):
        #self.msg_if.pub_warn("Got job pos msg: " + str(msg))
        if self.moveSpeedRatioCb is not None:
            self.pos_goal = -999
            direction = msg.direction * self.reverse_int
            speed_ratio = nepi_utils.check_ratio(msg.speed_ratio)
            time_s = msg.duration_s
            self.moveSpeedRatioCb(direction, speed_ratio, time_s)
            #self.msg_if.pub_info("Jogging pos" )
        self.publish_status()
        
        

    def _setReverseEnableCb(self, msg):
        enabled = msg.data
        self.set_reverse_enable(enabled)

    def set_reverse_enable(self, enabled):
        self.reverse = enabled
        reverse_int = 1
        if enabled == True:
            reverse_int = -1
        self.reverse_int = reverse_int
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'reverse', self.reverse)
        self.msg_if.pub_info("Set pos control to reverse=" + str(self.reverse))
        self.publish_status()

        

    def _setServoConnectedCb(self, msg):
        # Purely a declaration -- nothing is commanded to the hardware. Saved right
        # away, so the channels the operator marked in use survive a restart.
        self.connected = msg.data
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'servo_connected', self.connected)
            self.node_if.save_config()
        self.msg_if.pub_info("Set servo connected to " + str(self.connected))
        self.publish_status()

    def _setHomePositionHereCb(self, _):
        self.set_home_position_here()

    def set_home_position_here(self):
        self.pos_home = self.status_msg.pos_now * self.reverse_int
        if self.node_if is not None:
            self.node_if.set_param(self.node_if_prefix + 'home_position', self.pos_home)
        self.msg_if.pub_info("Home positon set to: " + "%.2f" % (self.pos_home * self.reverse_int))
        self.publish_status()
        if self.setHomePositionHereCb is not None:
            self.setHomePositionHereCb()
          
    def _calibrateCenterCB(self,msg):
        self.calibrate_center()

    def calibrate_center(self):
        if self.calibrateCenterCB is not None:
            self.calibrateCenterCB()

    def set_hardstop_limits(self, min_hardstop, max_hardstop):
        # Update the mechanical range after construction. Hardstops are otherwise
        # write-once from factoryLimits, but an open-loop servo cannot report its own
        # travel: the operator finds it and declares it in the driver's settings, so
        # the driver needs to hand the corrected range back. Values are in the
        # driver's own (non-reversed) frame, the same frame factoryLimits uses.
        min_hardstop = float(min_hardstop)
        max_hardstop = float(max_hardstop)
        if min_hardstop >= max_hardstop or abs(max_hardstop - min_hardstop) < self.MIN_LIMIT_ANGLE:
            self.msg_if.pub_warn("Rejecting invalid hardstop range " + str(min_hardstop) + " to " + str(max_hardstop))
            return False

        # Softstops still spanning the old hardstops were never narrowed, so they follow
        # the new range; narrowed ones are kept where they still fit inside it
        if [self.min_softstop, self.max_softstop] == [self.min_hardstop, self.max_hardstop]:
            min_softstop = min_hardstop
            max_softstop = max_hardstop
        else:
            min_softstop = max(self.min_softstop, min_hardstop)
            max_softstop = min(self.max_softstop, max_hardstop)
            if max_softstop - min_softstop < self.MIN_LIMIT_ANGLE:
                min_softstop = min_hardstop
                max_softstop = max_hardstop

        self.min_hardstop = min_hardstop
        self.max_hardstop = max_hardstop

        if [min_softstop, max_softstop] != [self.min_softstop, self.max_softstop]:
            self.min_softstop = min_softstop
            self.max_softstop = max_softstop
            if self.setSoftLimitsCb is not None:
                self.setSoftLimitsCb(min_softstop, max_softstop)
            if self.node_if is not None:
                self.node_if.set_param(self.node_if_prefix + 'min_softstop', min_softstop)
                self.node_if.set_param(self.node_if_prefix + 'max_softstop', max_softstop)

        self.msg_if.pub_info("Set hardstop range to " + str(min_hardstop) + " to " + str(max_hardstop))
        self.publish_status()
        return True

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


    def _goPosDirection(self, direction):
        target = self.max_softstop if direction >= 0 else self.min_softstop
        self.msg_if.pub_info("Driving pos continuously toward soft-limit " + str(target))
        self.gotoPosition(target)
    
 
    def _setPanAngleCb(self, msg):
        angle = msg.data
        if angle >= -360 and angle <= 360:
            try:
                self.transform_if.set_transform_value('yaw_deg_offset', angle)
            except:
                pass

    def _setTiltAngleCb(self, msg):
        angle = msg.data
        if angle >= -360 and angle <= 360:
            try:
                self.transform_if.set_transform_value('pitch_deg_offset', angle)
            except:
                pass

    def status_query_callback(self, _):
        """Service handler that returns the SVX  capabilities report.

        Returns:
            DeviceSVXStatusQueryResponse: The current DeviceSVXStatus in device_status,
                describing which SVX  capabilities are supported, including absolute
                positioning, timed positioning, separate pos control, position
                feedback, adjustable speed_now, limit controls, homing, set-pos_home, and
                calibration.
        """
        return DeviceSVXStatusQueryResponse(device_status = self.status_msg)
    
  



    def passFunction(self):
        return 0

    def _publishSVXStatusCb(self, timer):
        self.publish_status()

      
