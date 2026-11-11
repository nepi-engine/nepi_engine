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

import copy
import math

import numpy as np

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_process
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img

from nepi_interfaces.msg import NavPose
from nepi_interfaces.msg import ProcessStatus


logger = nepi_sdk.sdk_logger


########################
## REQUIRED Process IF Utilities
ID = 'rbx_process'
NAME = 'rbx_process'
DESCRIPTION = 'RBX Device Process'

STATUS_MSG = ProcessStatus()

RESULTS_PUB_MSG = None
RESULTS_PUB_TOPIC = None


DEFAULT_PROCESS = 'rbx_auto_1'


########################
## Shared Process Utility Functions
##########################


########################
## Process Data Dictionary
##########################


##########################   
DATA_DICT = dict(
    ###############
    # Provided By Caller
    rbx_status_dict = dict(),
    rbx_navpose_dict = None,

    servo_names = [],
    servo_status_dict = dict(),

    cv2_img = None,
    cv2_img_status = None,
    cv2_img_navpose_dict = None,

    ###############
    # Updated By Caller, Then Cleared
    set_home_updated =   None,
    go_home_updated = None,
    go_stop_updated = None,
    goto_pose_updated = None,
    goto_position_updated = None,
    goto_location_updated = None,
    goto_velocity_updated = None,
    mode_updated = None,
    action_updated = None,


    ###############
    # Capabilities Set by Process
    has_x_control = False,
    has_y_control = False,
    has_z_control = False,

    has_roll_control = False,
    has_pitch_control = False,
    has_yaw_control = False,

    has_stop = False,
    has_go_home = False,    
    has_set_home = False,

    has_goto_pose = False,
    has_goto_position = False,
    has_goto_location = False,
    has_go_velocity = False,

    mode_options = [],
    action_options = [],

    ###############
    # Updated by Process, Cleared by Caller
    set_home_success = None,
    go_home_success = None, # Caller Waits/Blocks for True/False Response
    go_stop_success = None, 
    goto_pose_success = None,
    goto_position_success = None,
    goto_location_success = None,
    goto_velocity_success = None,
    set_mode_success = None,
    set_action_success = None, # Caller Waits/Blocks for True/False Response   

    ###############
    # Custom Data Used and Managed by Process


    )



##########################
## Process Result Dictionaries
##########################

RESULTS_DICT = dict(
    result_1 = 'Result'
)

# Controls which and how results are displayed to the User
RESULTS_DISPLAY_DICT = dict(

    result_1 = {"type":"String", "default":'None',
                # OPTIONAL
                'display_name':'Result 1', 'description':'Result 1 Display', 'display_hidden':False},

),


##########################
## Process Functions   
##########################
processes_dict = dict()
functions_dict = dict()



rbx_auto_1_controls = dict(
        example_slider = {'type': 'FloatSlider', 'default': 0, 'bounds': [0,1],
                      'display_name': 'Example', 'description': 'Example Controls.', 'display_hidden': False}, 
)


def rbx_auto_1_process(controls_dict, data_dict, results_dict, if_dict):
    start_time = nepi_utils.get_time()
    last_data_dict = copy.deepcopy(data_dict)
    last_results_dict = copy.deepcopy(results_dict)
    if last_results_dict is None:
        last_results_dict = copy.deepcopy(RESULTS_DICT)
    controls_values_dict = nepi_controls.get_values_dict(controls_dict)
    #logger.log_warn("Got  Data: " + str(data_dict), throttle_s = 10)
    #logger.log_warn("Got  Data,Controls: " + str([data_dict,controls_values_dict]), throttle_s = 10)


    #logger.log_warn("Got Data and Controls: " + str([data_dict, controls_dict]), throttle_s = 5)

    ##############
    # Get Controls
    move_cont = controls_values_dict.get('move_cont', False)


    ##############
    # Get Data
    ##############

    #####
    rbx_status_dict = data_dict.get('rbx_status_dict',dict())

    rbx_navpose_dict = data_dict.get('rbx_navpose_dict',None)
    if rbx_navpose_dict is None:
        rbx_navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    if rbx_navpose_dict['has_pan_tilt'] == True:
        if rbx_navpose_dict['has_heading'] == True:
            rbx_heading_deg = rbx_navpose_dict.get('pan_tilt_heading_deg',-999)
        else:
            rbx_heading_deg = rbx_navpose_dict.get('pan_tilt_yaw_deg',-999)
        rbx_pitch_deg = rbx_navpose_dict.get('pan_tilt_pitch_deg',-999) 
        rbx_roll_deg = rbx_navpose_dict.get('pan_tilt_roll_deg',-999)
    else:
        if rbx_navpose_dict['has_heading'] == True:
            rbx_heading_deg = rbx_navpose_dict.get('heading_deg',-999)
        else:
            rbx_heading_deg = rbx_navpose_dict.get('yaw_deg',-999)
        rbx_pitch_deg = rbx_navpose_dict.get('pitch_deg',-999) 
        rbx_roll_deg = rbx_navpose_dict.get('roll_deg',-999)

    if int(rbx_heading_deg) == -999:
        rbx_heading_deg = 0
    
    if int(rbx_pitch_deg) == -999:
        rbx_pitch_deg = 0
      
    if int(rbx_roll_deg) == -999:
        rbx_roll_deg = 0


    #####
    servo_names = data_dict.get('servo_names',[])
    servo_status_dict = data_dict.get('servo_status_dict',dict())

    ##################
    # Get Servo IFS
    servo_if_keys_dict = data_dict.get('servo_if_keys_dict',dict())
    servo_if_dict = if_dict.get('servo_if_dict',dict())
    for servo_name in servo_names:
        if servo_name not in servo_if_dict.keys():
            servo_if_dict[servo_name] = None


    #####
    cv2_img = data_dict.get('cv2_img',None)
    cv2_img_status = data_dict.get('cv2_img_status',dict())
    
    cv2_img_navpose_dict = data_dict.get('cv2_img_navpose_dict',None)
    if cv2_img_navpose_dict is None:
        cv2_img_navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    if cv2_img_navpose_dict['has_pan_tilt'] == True:
        if cv2_img_navpose_dict['has_heading'] == True:
            cv2_img_heading_deg = cv2_img_navpose_dict.get('pan_tilt_heading_deg',-999)
        else:
            cv2_img_heading_deg = cv2_img_navpose_dict.get('pan_tilt_yaw_deg',-999)
        cv2_img_pitch_deg = cv2_img_navpose_dict.get('pan_tilt_pitch_deg',-999) 
        cv2_img_roll_deg = cv2_img_navpose_dict.get('pan_tilt_roll_deg',-999)
    else:
        if cv2_img_navpose_dict['has_heading'] == True:
            cv2_img_heading_deg = cv2_img_navpose_dict.get('heading_deg',-999)
        else:
            cv2_img_heading_deg = cv2_img_navpose_dict.get('yaw_deg',-999)
        cv2_img_pitch_deg = cv2_img_navpose_dict.get('pitch_deg',-999) 
        cv2_img_roll_deg = cv2_img_navpose_dict.get('roll_deg',-999)

    if int(cv2_img_heading_deg) == -999:
        cv2_img_heading_deg = 0
    
    if int(cv2_img_pitch_deg) == -999:
        cv2_img_pitch_deg = 0
      
    if int(cv2_img_roll_deg) == -999:
        cv2_img_roll_deg = 0

    #####
    last_results_time  = data_dict.get('last_results_time',0)
 
    ##############
    # Apply Updates
    #############


    ##############
    # Run Process
    #############

    
    ############################
    ## Control Servos in the if_dict
    ############################

    ##############
    # Process Results
    #############

  
    ##################
    # Clean Up
    ##################
    data_dict['last_results_time'] = nepi_utils.get_time()
    #logger.log_warn("Process Axisleted: " + str([auto_dict,results_dict,len(filtered_targets),len(targets_dict_list),class_filters]), throttle_s = 5)
    return data_dict, results_dict

    #########################################################

###########
# Create Initial Dictionaries
############

rbx_auto_1_update_dict = dict()
rbx_auto_1_update_dict['data_dict'] = copy.deepcopy(REQUIRED_DATA_DICT)
rbx_auto_1_update_dict['data_dict'].update(rbx_auto_1_dict['data_dict'])

rbx_auto_1_update_dict['controls_dict'] = copy.deepcopy(REQUIRED_CONTROLS_DICT)
rbx_auto_1_update_dict['controls_dict'].update(rbx_auto_1_dict['controls_dict'])

rbx_auto_1_update_dict['results_display_dict'] = copy.deepcopy(REQUIRED_RESULTS_DISPLAY_DICT)
rbx_auto_1_update_dict['results_display_dict'].update(rbx_auto_1_dict['results_display_dict'])

rbx_auto_1_update_dict['states_dict'] = copy.deepcopy(REQUIRED_STATES_DICT)
rbx_auto_1_update_dict['states_dict'].update(rbx_auto_1_dict['states_dict'])

processes_dict = nepi_process.update_processes_dict(processes_dict, process_name = 'rbx_auto_1', process_dict = rbx_auto_1_update_dict)
#logger.log_warn("Updated processes dict: " + str(processes_dict))
functions_dict['rbx_auto_1'] = rbx_auto_1_process


########################
## Processes Init Dict  
PROCESSES_DICT = copy.deepcopy(processes_dict)
FUNCTIONS_DICT = functions_dict





########################
## Process Image Functions   
#######################



IMAGE_CONTROLS_DICT =  dict(
    overlay_results =  {
            'type': 'Toggle', 'value': True,
            # Optional
            'display_name': 'Show Overlay', 'description': 'Overlay Result Image', 'display_hidden': False},

)


IMAGE_CONFIG_DICT = dict(
    config_dict = dict(),
    overlay_color = (0,0,127),
    overlay_colors = [],
    overlay_font = nepi_img.OVERLAY_FONT,
    overlay_font_color = nepi_img.OVERLAY_FONT_COLOR,
    overlay_line_type = nepi_img.OVERLAY_LINE_TYPE,
    overlay_line_color = nepi_img.OVERLAY_LINE_COLOR,
    overlay_labels = True,
    overlay_range_bearing = True,
)



def pre_process_results_image(cv2_img,  
                              data_dict,
                              results_dict,                              
                              image_controls_dict = nepi_controls.get_values_dict(IMAGE_CONTROLS_DICT),
                              image_config_dict = IMAGE_CONFIG_DICT,):
        ##################
        # Get Image Data
        try:
            cv2_img_results = copy.deepcopy(cv2_img)

        except:
            return cv2_img

        ################
        # Setup
        ################

        ##################
        # Get Image Data
        if cv2_img_status is None:
            cv2_img_status = dict()
        width_deg = cv2_img_status.get('width_deg', 100)
        height_deg = cv2_img_status.get('height_deg', 70)

        if cv2_img_navpose_dict is None:
            cv2_img_navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
        if cv2_img_navpose_dict['has_pan_tilt'] == True:
            if cv2_img_navpose_dict['has_heading'] == True:
                cv2_img_heading_deg = cv2_img_navpose_dict.get('pan_tilt_heading_deg',-999)
            else:
                cv2_img_heading_deg = cv2_img_navpose_dict.get('pan_tilt_yaw_deg',-999)
            cv2_img_pitch_deg = cv2_img_navpose_dict.get('pan_tilt_pitch_deg',-999) 
            cv2_img_roll_deg = cv2_img_navpose_dict.get('pan_tilt_roll_deg',-999)
        else:
            if cv2_img_navpose_dict['has_heading'] == True:
                cv2_img_heading_deg = cv2_img_navpose_dict.get('heading_deg',-999)
            else:
                cv2_img_heading_deg = cv2_img_navpose_dict.get('yaw_deg',-999)
            cv2_img_pitch_deg = cv2_img_navpose_dict.get('pitch_deg',-999) 
            cv2_img_roll_deg = cv2_img_navpose_dict.get('roll_deg',-999)

        if int(cv2_img_heading_deg) == -999:
            cv2_img_heading_deg = 0
        
        if int(cv2_img_pitch_deg) == -999:
            cv2_img_pitch_deg = 0
        
        if int(cv2_img_roll_deg) == -999:
            cv2_img_roll_deg = 0

        ##################
        # Get IFs
        pub_image_if = if_dict.get('image_if', None)
                        
        ##################
        # Get Process Controls
        controls_dict = process_image_dict.get('controls_dict', dict())
        overlay_results = controls_dict.get('overlay_results', True)

        ##################
        # Get Process Data
        data_dict = process_image_dict.get('data_dict', dict())       
        last_image_time = copy.deepcopy(data_dict.get('last_image_time', 0))
        rbx_status_dict = copy.deepcopy(data_dict.get('rbx_status_dict', dict()))
                                        
        ##################
        # Get Process Config
        config_dict = process_image_dict.get('config_dict', dict())
        overlay_color = config_dict.get('overlay_color',(0,0,127))
        overlay_colors = config_dict.get('overlay_colors',[])
        overlay_font = config_dict.get('overlay_color',nepi_img.OVERLAY_FONT)
        overlay_font_color = config_dict.get('overlay_color',nepi_img.OVERLAY_FONT_COLOR)
        overlay_line_type = config_dict.get('overlay_color',nepi_img.OVERLAY_LINE_TYPE)
        overlay_line_color = config_dict.get('overlay_color',nepi_img.OVERLAY_LINE_COLOR)
        overlay_labels = config_dict.get('overlay_labels',True)
        overlay_range_bearing = config_dict.get('overlay_range_bearing',True)

        ##################
        # Get Results Data
        if results_dict is None:
            results_dict = dict()       
        # targets_list = results_dict.get('targets', [])

        ##################
        # Pre Process Results Image
        ################

        

        ##############################################################
        # Pre Process Results Overlay
        if overlay_results == True:
            ## MODIFY cv2_img
            pass
        ##############################################################




        ################
        # Clean Up
        ################
        process_image_dict['controls_dict'] = controls_dict
        process_image_dict['data_dict'] = data_dict
        process_image_dict['config_dict'] = config_dict
        return cv2_img_results, process_image_dict


    
def post_process_results_image(cv2_img, cv2_img_status, cv2_img_navpose_dict, process_image_dict, results_dict, if_dict):
        ##################
        # Get Image Data
        try:
            cv2_img_results = copy.deepcopy(cv2_img)

        except:
            return cv2_img

        ################
        # Setup
        ################

        ##################
        # Get Image Data
        if cv2_img_status is None:
            cv2_img_status = dict()
        width_deg = cv2_img_status.get('width_deg', 100)
        height_deg = cv2_img_status.get('height_deg', 70)
         
        if cv2_img_navpose_dict is None:
            cv2_img_navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
        if cv2_img_navpose_dict['has_pan_tilt'] == True:
            if cv2_img_navpose_dict['has_heading'] == True:
                cv2_img_heading_deg = cv2_img_navpose_dict.get('pan_tilt_heading_deg',-999)
            else:
                cv2_img_heading_deg = cv2_img_navpose_dict.get('pan_tilt_yaw_deg',-999)
            cv2_img_pitch_deg = cv2_img_navpose_dict.get('pan_tilt_pitch_deg',-999) 
            cv2_img_roll_deg = cv2_img_navpose_dict.get('pan_tilt_roll_deg',-999)
        else:
            if cv2_img_navpose_dict['has_heading'] == True:
                cv2_img_heading_deg = cv2_img_navpose_dict.get('heading_deg',-999)
            else:
                cv2_img_heading_deg = cv2_img_navpose_dict.get('yaw_deg',-999)
            cv2_img_pitch_deg = cv2_img_navpose_dict.get('pitch_deg',-999) 
            cv2_img_roll_deg = cv2_img_navpose_dict.get('roll_deg',-999)

        if int(cv2_img_heading_deg) == -999:
            cv2_img_heading_deg = 0
        
        if int(cv2_img_pitch_deg) == -999:
            cv2_img_pitch_deg = 0
        
        if int(cv2_img_roll_deg) == -999:
            cv2_img_roll_deg = 0
                      
        ##################
        # Get IFs
        pub_image_if = if_dict.get('image_if', None)

        ##################
        # Get Process Controls
        controls_dict = process_image_dict.get('controls_dict', dict())
        overlay_results = controls_dict.get('overlay_results', True)

        ##################
        # Get Process Data
        data_dict = process_image_dict.get('data_dict', dict())       
        last_image_time = copy.deepcopy(data_dict.get('last_image_time', 0))
        rbx_status_dict = copy.deepcopy(data_dict.get('rbx_status_dict', dict()))

        ##################
        # Get Process Config
        config_dict = process_image_dict.get('config_dict', dict())
        overlay_color = config_dict.get('overlay_color',(0,0,127))
        overlay_colors = config_dict.get('overlay_colors',[])
        overlay_font = config_dict.get('overlay_color',nepi_img.OVERLAY_FONT)
        overlay_font_color = config_dict.get('overlay_color',nepi_img.OVERLAY_FONT_COLOR)
        overlay_line_type = config_dict.get('overlay_color',nepi_img.OVERLAY_LINE_TYPE)
        overlay_line_color = config_dict.get('overlay_color',nepi_img.OVERLAY_LINE_COLOR)
        overlay_labels = config_dict.get('overlay_labels',True)
        overlay_range_bearing = config_dict.get('overlay_range_bearing',True)

        ##################
        # Get Results Data
        if results_dict is None:
            results_dict = dict()       
        # targets_list = results_dict.get('targets', [])

        ##################
        # Post Process Results Image
        ################

        

        ##############################################################
        # Post Process Results Overlay
        if overlay_results == True:
            ## MODIFY cv2_img
            pass
        ##############################################################




        ################
        # Clean Up
        ################
        process_image_dict['controls_dict'] = controls_dict
        data_dict['last_image_time'] = nepi_utils.get_time()
        process_image_dict['data_dict'] = data_dict
        process_image_dict['config_dict'] = config_dict
        return cv2_img_results, process_image_dict
    
