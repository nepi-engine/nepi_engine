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

logger = nepi_sdk.sdk_logger


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
SCAN_SWITCH_DEG = 5

BASE_RESULTS_DICT = dict()

def get_blank_results_dict():
  results_dict = copy.deepcopy(BASE_RESULTS_DICT)
  results_dict['timestamp'] = -999
  results_dict['scanning'] = [False,False]
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
        scanning_enabled = False,
        pantilt_dict = nepi_sdk.convert_msg2dict(DevicePTXStatus()),
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose()),

        last_results_time = 0,
    )


BASE_CONTROLS_DICT = dict(

        pan_scan = {"type":"Toggle", "value":True,
                   # OPTIONAL
                   'display_name':'Pan Enable', 'description':'Scan in Pan Axis', 'display_hidden':False}, 

        pan_speed = {
            'type': 'FloatSlider', 'default': 0.5, 'bounds': [0.1, 1.0], 'round_value': 1,
            'display_name': 'Pan Speed',
            'description': 'Pan Speed', 'hidden': False}, 


        pan_limits = {
            'type': 'Floats', 'default': [-175, 175], 'bounds': [-175, 175, 20,1], 'round_value': 0,
            # OPTIONAL
            'display_name':'Pan Limits (Deg)', 'display_labels':['Min','Max'], 'description':' Scan Limits in Degs', 'display_bounds': False, 'display_hidden':False, 'round_display': 0, 'display_row': True},

        tilt_scan = {"type":"Toggle", "value": False,
                   # OPTIONAL
                   'display_name':'Tilt Enable', 'description':'Scan in Tilt Axis', 'display_hidden':False}, 

        tilt_speed = {
            'type': 'FloatSlider', 'default': 0.5, 'bounds': [0.1, 1.0], 'round_value': 1,
            'display_name': 'Tilt Speed',
            'description': 'Tilt Speed', 'hidden': False}, 


        tilt_limits = {
            'type': 'Floats', 'default': [-60, 60], 'bounds': [-60, 60, 20, 1], 'round_value': 0,
            # OPTIONAL
            'display_name':'Tilt Limits (Deg)', 'display_labels':['Min','Max'], 'description':'Tilt Scan Limits in Degs', 'display_bounds': False, 'display_hidden':False, 'round_display': 0, 'display_row': True},


    )


BASE_RESULTS_DISPLAY_DICT = dict(

        scanning = {
                    'type': 'Bools', 'value': [False, False],
                    'display_name': 'Scanning', 'display_labels':['Pan','Tilt'], 'description': 'Two booleans.', 'display_hidden': False, 'display_row': True},

        pan_deg = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Pan (Deg)', 'description':'Pan Goal Degrees', 'display_hidden':False,'round_display': 1,},

        tilt_deg = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Tilt (Deg)', 'description':'Tilt Goal Degrees','display_hidden': False, 'round_display': 1,},

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
    scanning_enabled = controls_values_dict.get('scanning_enabled', False)

    pan_scan = controls_values_dict.get('pan_scan', True)
    controls_dict = nepi_controls.set_hidden(controls_dict,'pan_speed',pan_scan == False)
    controls_dict = nepi_controls.set_hidden(controls_dict,'pan_limits',pan_scan == False)
    pan_enabled = nepi_controls.get_value(controls_dict,'pan_scan')
    pan_speed_ratio = nepi_controls.get_value(controls_dict,'pan_speed')


    tilt_scan = controls_values_dict.get('tilt_scan', True)
    controls_dict = nepi_controls.set_hidden(controls_dict,'tilt_speed',tilt_scan == False)
    controls_dict = nepi_controls.set_hidden(controls_dict,'tilt_limits',tilt_scan == False)
    tilt_enabled = nepi_controls.get_value(controls_dict,'tilt_scan')
    tilt_speed_ratio = nepi_controls.get_value(controls_dict,'tilt_speed')

    #logger.log_warn("Got Pan Tilt Speed Ratios: " + str([pan_speed_ratio,tilt_speed_ratio]), throttle_s = 5)
    ##############
    # Get Data
    blank_data_dict = copy.deepcopy(BASE_DATA_DICT)
    navpose_dict = data_dict.get('navpose_dict',blank_data_dict['navpose_dict'])
    if navpose_dict is None:
        navpose_dict = nepi_sdk.convert_msg2dict(NavPose())

    pantilt_dict = data_dict.get('pantilt_dict',None)

    if pantilt_dict is not None:
        if pantilt_dict == nepi_sdk.convert_msg2dict(DevicePTXStatus()):
            pantilt_dict = None

    if pantilt_dict is not None:
        pan_bounds = [pantilt_dict['pan_min_softstop_deg'],pantilt_dict['pan_max_softstop_deg']]
        controls_dict = nepi_controls.set_bounds(controls_dict,'pan_limits',pan_bounds)
        pan_limits = nepi_controls.get_value(controls_dict,'pan_limits')
        tilt_bounds = [pantilt_dict['tilt_min_softstop_deg'],pantilt_dict['tilt_max_softstop_deg']]
        controls_dict = nepi_controls.set_bounds(controls_dict,'tilt_limits',tilt_bounds)
        tilt_limits = nepi_controls.get_value(controls_dict,'tilt_limits')

        
        is_moving = pantilt_dict['is_moving']

        pan_now_deg = pantilt_dict['pan_now_deg']
        pan_goal_deg = pantilt_dict['pan_goal_deg']
        pan_speed_dps = pantilt_dict['speed_max_dps'] * pan_speed_ratio

        tilt_now_deg = pantilt_dict['tilt_now_deg']
        tilt_goal_deg = pantilt_dict['tilt_goal_deg']
        tilt_speed_dps = pantilt_dict['speed_max_dps'] * tilt_speed_ratio
  



    ##############
    # Run Process
    #############
    if pantilt_dict is not None:
        scanning = [False,False]

        pan_deg = -999
        pan_dps = -999
        if pan_enabled == True and scanning_enabled == True:
            scanning[0] = True
            pan_dps = pan_speed_dps
            if (pan_now_deg < (pan_limits[0] + SCAN_SWITCH_DEG)):
                pan_deg = pan_limits[1]

            elif (pan_now_deg > (pan_limits[1] - SCAN_SWITCH_DEG)):
                pan_deg = pan_limits[0]
              
            elif pan_goal_deg != pan_limits[0] or pan_goal_deg != pan_limits[1]:
                pan_deg = pan_limits[0]

        tilt_deg = -999
        tilt_dps = -999
        if tilt_enabled == True and scanning_enabled == True:
            scanning[1] = True
            tilt_dps = tilt_speed_dps
            if (tilt_now_deg < (tilt_limits[0] + SCAN_SWITCH_DEG)):
                tilt_deg = tilt_limits[1]

            elif (tilt_now_deg > (tilt_limits[1] - SCAN_SWITCH_DEG)):
                tilt_deg = tilt_limits[0]
              
            elif tilt_goal_deg != tilt_limits[0] or tilt_goal_deg != tilt_limits[1]:
                tilt_deg = tilt_limits[0]               

        
    ##############
    # Process Results
    #############    
    if pantilt_dict is not None:
        results_dict = get_blank_results_dict()
        results_dict['scanning'] = scanning

        results_dict['pan_deg'] = pan_deg
        results_dict['pan_dps'] = pan_dps

        results_dict['tilt_deg'] = tilt_deg
        results_dict['tilt_dps'] = tilt_dps

        results_dict = update_results(results_dict, navpose_dict)
        
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

