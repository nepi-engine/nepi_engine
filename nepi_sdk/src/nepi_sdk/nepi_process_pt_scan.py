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

from nepi_interfaces.msg import DevicePTXStatus
from nepi_interfaces.msg import NavPose

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_process_scan"
logger = Logger(log_name = log_name)


########################
## REQUIRED Process IF Utilities
DEFAULT_PROCESS_NAME = 'scan'
DEFAULT_PROCESS = 'scan_1'


RESULTS_PUB_MSG = None
RESULTS_PUB_TYPE = None
RESULTS_PUB_DICT = None
RESULTS_PUB_TOPIC = None


IMAGE_PUB_TOPIC = None

########################
## Process Utility Functions


BASE_RESULTS_DICT = dict()

def get_blank_results_dict():
  results_dict = copy.deepcopy(BASE_RESULTS_DICT)
  results_dict['timestamp'] = -999
  results_dict['pan_deg'] = -999
  results_dict['pan_dps'] = -999
  results_dict['tilt_deg'] = -999
  results_dict['tilt_dps'] = -999
  return results_dict


def update_results(results_dict, navpose_dict = None):
    if results_dict is not None:        
        timestamp = results_dict.get('timestamp',-999)
        if timestamp == -999:
            timestamp == nepi_utils.get_time()
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


   
BASE_DATA_DICT = dict(

        pan_deg = 0,
        tilt_deg = 0,
        pantilt_dict = nepi_sdk.convert_msg2dict(DevicePTXStatus()),
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose()),

        last_results_time = 0,
    )


BASE_CONTROLS_DICT = dict(

        pan_scan = {"type":"Toggle", "value":True,
                   # OPTIONAL
                   'display_name':'Pan Enable', 'description':'Scan in Pan Axis', 'display_hidden':False}, 

        pan_speed = {
            'type': 'FloatSlider', 'default': 0.5, 'bounds': [0.0, 1.0], 'round_value': 1,
            'display_name': 'Pan Speed',
            'description': 'Pan Speed', 'hidden': False}, 

        pan_deg = {"type":"Float", "value":350, 'bounds': [60, 350], 'round_value': 1,
                    # OPTIONAL
                    'display_name':'Pan Scan (Deg)', 'description':'Pan Scan Deg', 'display_hidden':False, 'round_display': 0,},


        tilt_scan = {"type":"Toggle", "value":False,
                   # OPTIONAL
                   'display_name':'Tilt Enable', 'description':'Scan in Tilt Axis', 'display_hidden':False}, 

        tilt_speed = {
            'type': 'FloatSlider', 'default': 0.5, 'bounds': [0.0, 1.0], 'round_value': 1,
            'display_name': 'Tilt Speed',
            'description': 'Tilt Speed', 'hidden': False}, 

        tilt_deg = {"type":"Float", "value":120, 'bounds': [30, 120], 'round_value': 1,
                    # OPTIONAL
                    'display_name':'Tilt Scan (Deg)', 'description':'Tilt Scan Deg', 'display_hidden':False, 'round_display': 0,},


    )


BASE_RESULTS_DISPLAY_DICT = dict(


        pan_goal = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Pan (Deg)', 'description':'Pan Goal Degrees', 'display_hidden':False, 'round_display': 1,},

        tilt_goal = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Tilt (Deg)', 'description':'Tilt Goal Degrees', 'display_hidden':False, 'round_display': 1,},


    )

BASE_STATES_DICT = dict(

    )


########################
## Process 1   



scan_1_dict = {

   
    'data_dict': copy.deepcopy(BASE_DATA_DICT),


    'controls_dict': copy.deepcopy(BASE_CONTROLS_DICT),


    'results_display_dict':  copy.deepcopy(BASE_RESULTS_DISPLAY_DICT),

    'states_dict':  copy.deepcopy(BASE_STATES_DICT),
}


def scan_1_process(data_dict, controls_dict, states_dict, results_dict, if_dict):
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
    pan_scan = controls_values_dict.get('pan_scan', True)
    controls_dict = nepi_controls.set_disabled('pan_speed',pan_scan)
    controls_dict = nepi_controls.set_disabled('pan_deg',pan_scan)
    if pan_scan == True:
        pan_speed = controls_values_dict.get('pan_speed', 0.5)
    else:
        pan_speed = -999


    tilt_scan = controls_values_dict.get('tilt_scan', True)
    controls_dict = nepi_controls.set_disabled('tilt_speed',tilt_scan)
    controls_dict = nepi_controls.set_disabled('tilt_deg',tilt_scan)
    if tilt_scan == True:
        tilt_speed = controls_values_dict.get('tilt_speed', 0.5)
    else:
        tilt_speed = -999

    ##############
    # Get Data
    blank_data_dict = copy.deepcopy(BASE_DATA_DICT)

    pantilt_dict = data_dict.get('pantilt_dict',blank_data_dict['pantilt_dict'])
    if pantilt_dict is None:
        pantilt_dict = nepi_sdk.convert_msg2dict(DevicePTXStatus())
        
    navpose_dict = data_dict.get('navpose_dict',blank_data_dict['navpose_dict'])
    if navpose_dict is None:
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose())

    pan_now = pantilt_dict['pan_now_deg']
    tilt_now = pantilt_dict['tilt_now_deg']

    pan_goal = pantilt_dict['pan_goal_deg']
    tilt_goal = pantilt_dict['tilt_goal_deg']

 
    # has_limit_controls = pantilt_dict['has_limit_controls']
    # pan_min_hardstop_deg = pantilt_dict['pan_min_hardstop_deg']
    # pan_max_hardstop_deg = pantilt_dict['pan_max_hardstop_deg']
    # tilt_min_hardstop_deg = pantilt_dict['tilt_min_hardstop_deg']
    # tilt_max_hardstop_deg = pantilt_dict['tilt_max_hardstop_deg']

    # pan_min_softstop_deg = pantilt_dict['pan_min_softstop_deg']
    # pan_max_softstop_deg = pantilt_dict['pan_max_softstop_deg']
    # tilt_min_softstop_deg = pantilt_dict['tilt_min_softstop_deg']
    # tilt_max_softstop_deg = pantilt_dict['tilt_max_softstop_deg']

    ##############
    # Run Process
    #############

    if pan_scan == True:
      
      pan_pos_disabled  = True
      pan_jog_disabled = True
      pan_speed_disabled = True

      tilt_pos_disabled  = True
      tilt_jog_disabled = True
      tilt_speed_disabled = True

      track_heading_deg = track_results_dict['heading_deg']
      track_pitch_deg = track_results_dict['pitch_deg']
      track_roll_deg = track_results_dict['roll_deg']

      scan_pan_error = -1 * (heading_deg - track_heading_deg)
      scan_tilt_error = -1 * (pitch_deg - track_pitch_deg)

    else:
      pan_pos_disabled = False
      pan_jog_disabled = False
      pan_speed_disabled = False

      tilt_pos_disabled  = False
      tilt_jog_disabled = False
      tilt_speed_disabled = False
      
      scan_pan_error = -1 * (pan_now - pan_goal)
      scan_tilt_error = -1 * (tilt_now - tilt_goal)

    #######
    # Process Data
    scan_pan_goal = round(pan_now + scan_pan_error,1)
    scan_pan_error = round(scan_pan_error,1)

    scan_tilt_goal = round(tilt_now + scan_tilt_error,1)
    scan_tilt_error = round(scan_tilt_error,1)

    # scan_heading_goal = round(heading_deg + scan_pan_error, 1) 
    # scan_heading_error = round(scan_pan_error,1)

    ##############
    # Process Results
    #############
    results_dict = get_blank_results_dict()

    results_dict['pan_goal'] = scan_pan_goal
    results_dict['pan_error'] = scan_pan_error
    results_dict['pan_dps'] = -999

    results_dict['tilt_goal'] = scan_tilt_goal
    results_dict['tilt_error'] = scan_tilt_error
    results_dict['tilt_dps'] = -999

    results_dict['heading_goal'] = -999
    results_dict['heading_error'] = -999
    results_dict['roll_goal'] = -999
    results_dict['roll_error'] = -999
    results_dict['pitch_goal'] = -999
    results_dict['pitch_error'] = -999

    results_dict = update_results(results_dict, navpose_dict)

    ###########################
    # Update PanTilt Axis Data

    ###############
    # Pan
    pan_axis_dict['pos_disabled'] = pan_pos_disabled
    pan_axis_dict['jog_disabled'] = pan_jog_disabled
    pan_axis_dict['speed_disabled'] = pan_speed_disabled

    pan_axis_dict['now_deg'] = pan_now
    # pan_axis_dict['lock_deg'] = pan_lock_deg

    # pan_axis_dict['goal_ratio'] = pan_goal_ratio
    pan_axis_dict['goal_deg'] = scan_pan_goal
    pan_axis_dict['error_deg'] = scan_pan_error

    # pan_axis_dict['speed_lock'] = pan_speed_lock
    # pan_axis_dict['speed_ratio'] = pan_speed_ratio
    # pan_axis_dict['speed_dps'] = pan_speed_dps

    # ###############
    # # Tilt

    tilt_axis_dict['pos_disabled'] = tilt_pos_disabled
    tilt_axis_dict['jog_disabled'] = tilt_jog_disabled
    tilt_axis_dict['speed_disabled'] = tilt_speed_disabled

    tilt_axis_dict['now_deg'] = tilt_now
    # tilt_axis_dict['lock_deg'] = tilt_lock_deg

    # tilt_axis_dict['goal_ratio'] = tilt_goal_ratio
    tilt_axis_dict['goal_deg'] = scan_tilt_goal
    tilt_axis_dict['error_deg'] = scan_tilt_error

    # tilt_axis_dict['speed_lock'] = tilt_speed_lock
    # tilt_axis_dict['speed_ratio'] = tilt_speed_ratio
    # tilt_axis_dict['speed_dps'] = tilt_speed_dps


    # ###########################
    # # Apply PanTilt Control Updates
    # pantilt_connect_if = if_dict.get('pantilt_connect_if', None)
    # if pantilt_connect_if is not None:
    #     if pantilt_connect_if.get_ready() == True:
    #         pass
    
    #logger.log_warn("Process Completed: " + str([scan_dict,results_dict,len(filtered_targets),len(targets_dict_list),class_filters]), throttle_s = 5)
    return data_dict, controls_dict, states_dict, results_dict


processes_dict = nepi_process.update_processes_dict(processes_dict, process_name = 'scan_1', process_dict = scan_1_dict)
#logger.log_warn("Updated processes dict: " + str(processes_dict))
functions_dict['scan_1'] = scan_1_process




########################
## Processes Init Dict  
PROCESSES_DICT = copy.deepcopy(processes_dict)
FUNCTIONS_DICT = functions_dict



########################
## Process Image Functions   
#######################

