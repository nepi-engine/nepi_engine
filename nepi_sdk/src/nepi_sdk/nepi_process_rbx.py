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
NAME = 'rbx_auto'
DESCRIPTION = 'Light and Strobe Device'

STATUS_MSG = ProcessStatus()

RESULTS_PUB_MSG = None
RESULTS_PUB_TOPIC = None

IMAGE_PUB_TOPIC = None

DEFAULT_PROCESS = 'rbx_auto_1'




########################
## Shared Process Utility Functions
##########################


##########################
REQUIRED_RESULTS_DICT = dict()

def get_blank_results_dict():
  results_dict = copy.deepcopy(REQUIRED_RESULTS_DICT)
  return results_dict


def update_results(results_dict, navpose_dict = None):
    if results_dict is not None:        
        timestamp = results_dict.get('timestamp',-999)
        if timestamp == -999:
            timestamp = nepi_utils.get_time()
        age_sec =  nepi_utils.get_time() - timestamp
        results_dict['timestamp'] = timestamp
        results_dict['age_sec'] = age_sec
        #logger.log_warn("Updated Results Dict: " + str([results_dict]), throttle_s = 5)
    return results_dict






########################
## Process Functions   
#######################
processes_dict = dict()
functions_dict = dict()

# REQUIRED_IF_DICT = dict(
#     servo_name = servo_name_if,
#     ...
# )
   
REQUIRED_DATA_DICT = dict(
    ###############
    # Provided By Caller
    servo_names = [],
    servo_status_dict = dict(),

    cv2_img = None,
    cv2_img_status = None,
    cv2_navpose_dict = None,

    navpose_dict = None,

    last_results_time = 0,

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
    # Capabilities Set by Process and Overide Any Driver Settings
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

    go_action_options = [],

    ###############
    # Updated by Process
    set_home_success = None,
    go_home_success = None, # Caller Waits/Blocks for True/False Response
    go_stop_success = None, 
    goto_pose_success = None,
    goto_position_success = None,
    goto_location_success = None,
    goto_velocity_success = None,
    go_action_success = None # Caller Waits/Blocks for True/False Response   
    )


REQUIRED_CONTROLS_DICT = dict()


REQUIRED_RESULTS_DISPLAY_DICT = dict( )


REQUIRED_STATES_DICT = dict(

    )


########################
## Process 1   



rbx_auto_1_dict = {

   
    'data_dict': dict(
        has_set_home = False,
        has_home = False,
        has_stop = False,
        has_pose = False,
        has_position = False,
        has_location = False,
        has_velocity = False,
        go_action_options = [],
    ),



    'controls_dict': dict(
        select_motor = {'type': 'Menu', 'default': 0, 'options': ['None'],
                      'display_name': 'Display Motor Data', 'description': 'Pick Motor Data to Display.', 'display_hidden': False}, 
            
    ),


    'results_display_dict': dict(

        motor_name = {"type":"String", "value":'None',
                    # OPTIONAL
                    'display_name':'Motor Name', 'description':'Motor Name', 'display_hidden':False},

        speed_ratio = {"type":"Float", "value":-999, 'round_value': 1,
                    # OPTIONAL
                    'display_name':'Motor Speed Ratio', 'description':'Motor Speed Ratio', 'display_hidden':False, 'round_display': 1},

        direction = {"type":"Int", "value":1,
                    # OPTIONAL
                    'display_name':'Motor Dir', 'description':'Motor Dir', 'display_hidden':False},
    ),
       

    'states_dict': {
       
    },
}


def rbx_auto_1_process(data_dict, controls_dict, states_dict, results_dict, if_dict):
    start_time = nepi_utils.get_time()
    last_data_dict = copy.deepcopy(data_dict)
    last_results_dict = copy.deepcopy(results_dict)
    if last_results_dict is None:
        last_results_dict = get_blank_results_dict()
    controls_values_dict = nepi_controls.get_values_dict(controls_dict)
    #logger.log_warn("Got  Data: " + str(data_dict), throttle_s = 10)
    #logger.log_warn("Got  Data,Controls: " + str([data_dict,controls_values_dict]), throttle_s = 10)


    #logger.log_warn("Got Data and Controls: " + str([data_dict, controls_dict]), throttle_s = 5)

    ##############
    # Get Controls
    move_cont = controls_values_dict.get('move_cont', False)


    ##############
    # Get Data
   

    navpose_dict = data_dict.get('navpose_dict',None)
    if navpose_dict is None:
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose())


    
    if navpose_dict is None:
        navpose_dict = nepi_nav.BLANK_NAVPOSE_DICT
    if navpose_dict['has_pan_tilt'] == True:
        if navpose_dict['has_heading'] == True:
            heading_deg = navpose_dict.get('pan_tilt_heading_deg',-999)
        else:
            heading_deg = navpose_dict.get('pan_tilt_yaw_deg',-999)
        pitch_deg = navpose_dict.get('pan_tilt_pitch_deg',-999) 
        roll_deg = navpose_dict.get('pan_tilt_roll_deg',-999)
    else:
        if navpose_dict['has_heading'] == True:
            heading_deg = navpose_dict.get('heading_deg',-999)
        else:
            heading_deg = navpose_dict.get('yaw_deg',-999)
        pitch_deg = navpose_dict.get('pitch_deg',-999) 
        roll_deg = navpose_dict.get('roll_deg',-999)

    if int(heading_deg) == -999:
        heading_deg = 0
    
    if int(pitch_deg) == -999:
        pitch_deg = 0
      
    if int(roll_deg) == -999:
        roll_deg = 0


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
    results_dict = get_blank_results_dict()
    ######
    # Update Results Dict As Needed
    #####
    results_dict = update_results(results_dict, navpose_dict)
  

    ##################
    #logger.log_warn("Process Axisleted: " + str([auto_dict,results_dict,len(filtered_targets),len(targets_dict_list),class_filters]), throttle_s = 5)
    return data_dict, controls_dict, states_dict, results_dict



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


process_image_dict = {

    'controls_dict': dict(
        stabilize =  {
                'type': 'Toggle', 'value': True,
                # Optional
                'display_name': 'Stabilize', 'description': 'Stabilize Image.', 'display_hidden': False},

        overlay_results =  {
                'type': 'Toggle', 'value': True,
                # Optional
                'display_name': 'Show Overlay', 'description': 'Overlay Result Image', 'display_hidden': False},

    ),
    
    'data_dict': dict(
        last_image_time = 0
    ),

    'config_dict': dict(
        config_dict = dict(),
        overlay_color = (0,0,127),
        overlay_colors = [],
        overlay_font = nepi_img.OVERLAY_FONT,
        overlay_font_color = nepi_img.OVERLAY_FONT_COLOR,
        overlay_line_type = nepi_img.OVERLAY_LINE_TYPE,
        overlay_line_color = nepi_img.OVERLAY_LINE_COLOR,
        overlay_labels = True,
        overlay_range_bearing = True,
    ),

}


def process_results_image(cv2_img, cv2_img_status, process_image_dict, results_dict, image_if):
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
        navpose_dict = cv2_img_status.get('navpose_dict', nepi_nav.BLANK_NAVPOSE_DICT)

        ##################
        # Get Process Controls
        controls_dict = process_image_dict.get('controls_dict', dict())
        stabilize = controls_dict.get('stabilize', True)
        overlay_results = controls_dict.get('overlay_results', True)

        ##################
        # Get Process Data
        data_dict = process_image_dict.get('data_dict', dict())       
        last_image_time = copy.deepcopy(data_dict.get('last_image_time', 0))
        data_dict['last_image_time'] = nepi_utils.get_time()

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
        # Process Results Image
        ################

        #################
        # Process Image Stabilization
        if stabilize == True and image_if is not None:
            rotate_deg = -1 * navpose_dict.get('roll_deg', 0)
            pt_rotate_deg = navpose_dict.get('pan_tilt_roll_deg', 0)
            if pt_rotate_deg != 0:
                rotate_deg = -1 * pt_rotate_deg
            image_if.set_live_adjust_rotate_deg(rotate_deg)



        ##############################################################
        # Process Results Overlay
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
    
