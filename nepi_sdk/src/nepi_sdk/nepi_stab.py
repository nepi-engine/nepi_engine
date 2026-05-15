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


#import os
import copy
import numpy as np
# import cv2
# import math
# import pandas as pd
# from scipy.stats import linregress
# from scipy.signal import medfilt
# from scipy.interpolate import UnivariateSpline
# from scipy.interpolate import CubicSpline


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_stab"
logger = Logger(log_name = log_name)


PAN_TILT_SOURCE_MESSAGES = ['NavPose','NavPoseOrientation']

PAN_TILT_STAB_DATA_DICT = {
    'data_time': 0.0,
    'process_time': 0.0,

    'roll_deg': -999,
    'roll_dps': -999,

    'pitch_deg': -999,
    'pitch_dps': -999,

    'heading_deg': -999,
    'heading_dps': -999,

    'pan_deg': 0.0,
    'pan_adjs': [],
    'pan_adj': 0.0,
    'pan_goal': 0.0,
    'pan_dps': 0.0,


    'tilt_deg': 0.0,
    'tilt_adjs': [],
    'tilt_adj': 0.0,
    'tilt_goal': 0.0,
    'tilt_dps': 0.0,

    'last_stab_time': None
}


PAN_TILT_STAB_PROCESSES_DICT = dict()

DEFAULT_PAN_TILT_STAB_PROCESS = 'pt_stab_1'

#########################
# Stab Process Functions
#########################


def pt_stab_1(pt_connect_if, 
                        pan_min_max_degs,
                        til_min_max_degs,
                        stab_data_dict, 
                        stab_data_dict_last,
                        stab_settings_dict, 
                        goto_position = [0,0],
                        stab_pan_enabled = 1, 
                        stab_tilt_enabled = 1
                        ):

    logger.log_info("******", throttle_s=1)
    logger.log_info("*** Stabs Solution Update Starting ***", throttle_s=1)
    logger.log_info("******", throttle_s=1)
    
    last_time = copy.deepcopy(stab_data_dict['last_stab_time'])
                
    if last_time is not None and stab_data_dict_last is not None:
        delta_time = nepi_utils.get_time() - last_time
        stab_data_dict['roll_dps'] = (stab_data_dict['roll_deg'] - stab_data_dict_last['roll_deg']) / delta_time
        stab_data_dict['pitch_dps'] = (stab_data_dict['pitch_deg'] - stab_data_dict_last['pitch_deg']) / delta_time
        stab_data_dict['heading_dps'] = (stab_data_dict['heading_deg'] - stab_data_dict_last['heading_deg']) / delta_time


        stab_pos_deg = stab_settings_dict['stab_pos_deg']
        stab_vel_deg = stab_settings_dict['stab_vel_deg']
            

        ##########################
        # APPLY PT UPDATES IF NEEDED
        [pan_goal, tilt_goal] = goto_position

        pan_deg = stab_data_dict['pan_deg']
        pan_dps = stab_data_dict['pan_dps']
        tilt_deg = stab_data_dict['tilt_deg']
        tilt_dps = stab_data_dict['tilt_dps']
        pan_tilt_max_speed_dps = stab_settings_dict['pan_tilt_max_speed_dps']
        stab_min_dps = stab_settings_dict['stab_pt_min_speed_ratio'] * pan_tilt_max_speed_dps 
        stab_max_dps = stab_settings_dict['stab_pt_max_speed_ratio'] * pan_tilt_max_speed_dps
    
    
        # pan_adj = stab_data_dict['pan_adj']
        # pan_adj_last = stab_data_dict_last['pan_adj']
        # adj_pan_delta = abs(pan_adj - pan_adj_last)
        # if stab_pan_enabled == True:
        #     if adj_pan_delta > stab_pos_deg:
        #         adj_pan_goal = pan_goal + pan_adj
        #         stab_data_dict['pan_goal'] = adj_pan_goal
        #         stab_speed_ratio = stab_settings_dict['stab_pt_max_speed_ratio']
        #         pt_connect_if.set_pan_speed_ratio(stab_speed_ratio)
        #         pt_connect_if.goto_to_pan_position(adj_pan_goal)
        

    
        tilt_adj = stab_data_dict['tilt_adj']
        tilt_adj_last = stab_data_dict_last['tilt_adj']
        adj_tilt_delta = abs(tilt_adj - tilt_adj_last)
        adj_tilt_goal = tilt_goal + tilt_adj
        adj_tilt_goal_delta = (adj_tilt_goal - tilt_deg)
        logger.log_info("Stab checking tilt conditions: " + str([adj_tilt_delta, adj_tilt_goal_delta, stab_pos_deg, stab_vel_deg]), throttle_s=1)
        if stab_tilt_enabled == True:
            if adj_tilt_delta > stab_pos_deg: # or abs(adj_tilt_goal_delta)  > stab_pos_deg :
                stab_data_dict['tilt_goal'] = adj_tilt_goal
                
                tilt_dps = stab_min_dps + (stab_max_dps - stab_min_dps) * adj_tilt_goal_delta / stab_pos_deg
                stab_speed_ratio = tilt_dps / pan_tilt_max_speed_dps
                pt_connect_if.set_tilt_speed_ratio(stab_speed_ratio)
                pt_connect_if.goto_to_tilt_position(adj_tilt_goal)
                logger.log_info("Stab Control updated: " + str([adj_tilt_delta, stab_speed_ratio, adj_tilt_goal]), throttle_s=1)
    return stab_data_dict, stab_settings_dict

pt_stab_1_settings = {
    
    'stab_update_rate': 10,
    'stab_pos_deg': 5.0,
    'stab_vel_deg': 2.0,
    'stab_move_ratio': 1.0,
    'stab_reset_time_sec': 3.0,

    'stab_num_avg': 3,

    'pan_tilt_max_speed_dps': 10,
    'stab_pt_min_speed_ratio': 0.1,
    'stab_pt_max_speed_ratio': 0.9,
}

PAN_TILT_STAB_PROCESSES_DICT['pt_stab_1'] = {'process_function': pt_stab_1, 
                                             'default_settings_dict': pt_stab_1_settings}


############################



def pt_stab_2(pt_connect_if, 
                        pan_min_max_degs,
                        til_min_max_degs,
                        stab_data_dict, 
                        stab_data_dict_last,
                        stab_settings_dict, 
                        goto_position = [0,0],
                        stab_pan_enabled = 1, 
                        stab_tilt_enabled = 1
                        ):

    logger.log_info("******", throttle_s=1)
    logger.log_info("*** Stabs Solution Update Starting ***", throttle_s=1)
    logger.log_info("******", throttle_s=1)
    
    last_time = copy.deepcopy(stab_data_dict['last_stab_time'])
                
    if last_time is not None and stab_data_dict_last is not None:
        delta_time = nepi_utils.get_time() - last_time
        stab_data_dict['roll_dps'] = (stab_data_dict['roll_deg'] - stab_data_dict_last['roll_deg']) / delta_time
        stab_data_dict['pitch_dps'] = (stab_data_dict['pitch_deg'] - stab_data_dict_last['pitch_deg']) / delta_time
        stab_data_dict['heading_dps'] = (stab_data_dict['heading_deg'] - stab_data_dict_last['heading_deg']) / delta_time


        stab_pos_deg = stab_settings_dict['stab_pos_deg']
        stab_vel_deg = stab_settings_dict['stab_vel_deg']
            

        ##########################
        # APPLY PT UPDATES IF NEEDED
        [pan_goal, tilt_goal] = goto_position

        pan_deg = stab_data_dict['pan_deg']
        pan_dps = stab_data_dict['pan_dps']
        tilt_deg = stab_data_dict['tilt_deg']
        tilt_dps = stab_data_dict['tilt_dps']
        pan_tilt_max_speed_dps = stab_settings_dict['pan_tilt_max_speed_dps']
        stab_min_dps = stab_settings_dict['stab_pt_min_speed_ratio'] * pan_tilt_max_speed_dps 
        stab_max_dps = stab_settings_dict['stab_pt_max_speed_ratio'] * pan_tilt_max_speed_dps
    
    
        # pan_adj = stab_data_dict['pan_adj']
        # pan_adj_last = stab_data_dict_last['pan_adj']
        # adj_pan_delta = abs(pan_adj - pan_adj_last)
        # if stab_pan_enabled == True:
        #     if adj_pan_delta > stab_pos_deg:
        #         adj_pan_goal = pan_goal + pan_adj
        #         stab_data_dict['pan_goal'] = adj_pan_goal
        #         stab_speed_ratio = stab_settings_dict['stab_pt_max_speed_ratio']
        #         pt_connect_if.set_pan_speed_ratio(stab_speed_ratio)
        #         pt_connect_if.goto_to_pan_position(adj_pan_goal)
        

    
        tilt_adj = stab_data_dict['tilt_adj']
        tilt_adj_last = stab_data_dict_last['tilt_adj']
        adj_tilt_delta = abs(tilt_adj - tilt_adj_last)
        adj_tilt_goal = tilt_goal + tilt_adj
        adj_tilt_goal_delta = (adj_tilt_goal - tilt_deg)
        logger.log_info("Stab checking tilt conditions: " + str([adj_tilt_delta, adj_tilt_goal_delta, stab_pos_deg, stab_vel_deg]), throttle_s=1)
        if stab_tilt_enabled == True:
            if adj_tilt_delta > stab_pos_deg: # or abs(adj_tilt_goal_delta)  > stab_pos_deg :
                stab_data_dict['tilt_goal'] = adj_tilt_goal
                
                tilt_dps = stab_min_dps + (stab_max_dps - stab_min_dps) * adj_tilt_goal_delta / stab_pos_deg
                stab_speed_ratio = tilt_dps / pan_tilt_max_speed_dps
                pt_connect_if.set_tilt_speed_ratio(stab_speed_ratio)
                pt_connect_if.goto_to_tilt_position(adj_tilt_goal)
                logger.log_info("Stab Control updated: " + str([adj_tilt_delta, stab_speed_ratio, adj_tilt_goal]), throttle_s=1)
    return stab_data_dict, stab_settings_dict


pt_stab_2_settings = {
    
    'stab_update_rate': 10,
    'stab_pos_deg': 5.0,
    'stab_vel_deg': 2.0,
    'stab_move_ratio': 1.0,
    'stab_reset_time_sec': 3.0,

    'stab_num_avg': 3,
    'pan_tilt_max_speed_dps': 10,
    'stab_pt_min_speed_ratio': 0.1,
    'stab_pt_max_speed_ratio': 0.9,
}

PAN_TILT_STAB_PROCESSES_DICT['pt_stab_2'] = {'process_function': pt_stab_2, 
                                             'default_settings_dict': pt_stab_2_settings}



#########################
# Stab Utility Functions
#########################

def create_pan_tilt_processes_dict():
    processes_dict = dict()
    for process_name in PAN_TILT_STAB_PROCESSES_DICT.keys():
        processes_dict[process_name] = PAN_TILT_STAB_PROCESSES_DICT[process_name]['default_settings_dict']
    return processes_dict

def update_pan_tilt_processes_dict(stab_processes_dict):
    clean_stab_dict = create_pan_tilt_processes_dict()
    for stab_process in clean_stab_dict.keys():
        if stab_process in stab_processes_dict.keys():
            for key in clean_stab_dict[stab_process].keys():
                if key in stab_processes_dict[stab_process].keys():
                    clean_stab_dict[stab_process][key] = stab_processes_dict[stab_process][key]
    return clean_stab_dict

def get_blank_pan_tilt_data_dict():
    return copy.deepcopy(PAN_TILT_STAB_DATA_DICT)