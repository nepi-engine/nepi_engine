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


from nepi_interfaces.msg import NavPose, NavPoseOrientation

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_stab"
logger = Logger(log_name = log_name)


PAN_TILT_SOURCE_MESSAGE_DICT = {'NavPose' : NavPose, 'NavPoseOrientation': NavPoseOrientation}

PAN_TILT_STAB_DATA_DICT = {
    # Required Fields
    'data_time': 0.0,
    'process_time': 0.0,

    'roll_deg': -999,
    'rolls_dps': [],
    'roll_dps': -999,

    'pitch_deg': -999,
    'pitchs_dps': [],
    'pitch_dps': -999,

    'heading_deg': -999,
    'headings_dps': [],
    'heading_dps': -999,

    'pan_tilt_max_speed_dps': 10,
    'pan_deg': 0.0,
    'pan_dps': 0.0,
    'pan_speed_start_ratio': 1.0,
    'pan_min_deg': -170,
    'pan_max_deg': 170,


    'tilt_deg': 0.0,
    'tilt_dps': 0.0,
    'tilt_speed_start_ratio': 1.0,
    'tilt_min_deg': -50,
    'tilt_max_deg': 50,


    'stab_pan_deg': 0.0,
    'stab_pan_adjs': [],
    'stab_pan_adj': 0.0,
    'stab_pan_goal': 0.0,
    'stab_pan_dir': 0,
    'stab_pan_dps': 0.0,


    'stab_tilt_deg': 0.0,
    'stab_tilt_adjs': [],
    'stab_tilt_adj': 0.0,
    'stab_tilt_goal': 0.0,
    'stab_tilt_dir': 0,
    'stab_tilt_dps': 0.0,

    'last_stab_time': None,

     # Add Custom Fields Here

}


PAN_TILT_STAB_PROCESSES_DICT = dict()

DEFAULT_PAN_TILT_STAB_PROCESS = 'pt_stab_1'

#########################
# Stab Process Functions
#########################



pt_stab_1_settings = {
    # Required Fields
    'stab_update_rate': 10,
    'stab_reset_time_sec': 3.0,
    'stab_num_avg': 3,
    'stab_pt_min_speed_ratio': 0.1,
    'stab_pt_max_speed_ratio': 0.9,

    # Custom Fields. Automatically Populated in RUI
    'stab_controls_dict': {
        'nav_deg': 1.0,
        'pos_deg': 1.0,
        'vel_deg': 10.0,
        'move_deg': 2.0
    },
}

def pt_stab_1(pt_connect_if, 
                        stab_data_dict, 
                        stab_settings_dict, 
                        goto_position = [0,0],
                        stab_pan_enabled = 1, 
                        stab_tilt_enabled = 1
                        ):

    #logger.log_info("******")
    #logger.log_info("*** Stabs Solution Update Starting ***")
    #logger.log_info("******")
    
    ##########################
    # APPLY PT UPDATES IF NEEDED
    [pan_goal, tilt_goal] = goto_position

    pan_deg = stab_data_dict['pan_deg']
    pan_dps = stab_data_dict['pan_dps']
    pan_speed_start_ratio = stab_data_dict['pan_speed_start_ratio']
    tilt_deg = stab_data_dict['tilt_deg']
    tilt_dps = stab_data_dict['tilt_dps']
    tilt_speed_start_ratio = stab_data_dict['tilt_speed_start_ratio']
    pan_tilt_max_speed_dps = stab_data_dict['pan_tilt_max_speed_dps']

    stab_min_dps = stab_settings_dict['stab_pt_min_speed_ratio'] * pan_tilt_max_speed_dps 
    stab_max_dps = stab_settings_dict['stab_pt_max_speed_ratio'] * pan_tilt_max_speed_dps
    controls_dict = stab_settings_dict['stab_controls_dict']
    nav_deg = controls_dict['nav_deg']
    pos_deg = controls_dict['pos_deg']
    vel_deg = controls_dict['vel_deg']
    move_deg = controls_dict['move_deg']
    


    last_data_dict = copy.deepcopy(stab_data_dict)
    last_tilt_dps = last_data_dict['stab_tilt_dps']
    last_tilt_goal = last_data_dict['stab_tilt_goal']
    last_tilt_dir = last_data_dict['stab_tilt_dir']


    tilt_adj = stab_data_dict['stab_tilt_adj']
    adj_tilt_goal = round(tilt_goal + tilt_adj,1)
    adj_tilt_goal_delta = (adj_tilt_goal - tilt_deg)
    adj_tilt_dir = 1 if adj_tilt_goal_delta > 0 else -1
    adj_tilt_change = (last_tilt_goal - adj_tilt_goal)

    
    if stab_tilt_enabled == True:
        if abs(adj_tilt_goal_delta) > vel_deg or abs(adj_tilt_goal_delta) < pos_deg:
            stab_speed_ratio = tilt_speed_start_ratio
            stab_tilt_dps = round(stab_speed_ratio * pan_tilt_max_speed_dps,1)
        else:
            stab_tilt_dps = round(stab_min_dps + (stab_max_dps - stab_min_dps) * abs(adj_tilt_goal_delta) / vel_deg, 1)
            stab_speed_ratio = stab_tilt_dps / pan_tilt_max_speed_dps

        if abs(stab_tilt_dps - last_tilt_dps) > 1:     
            stab_data_dict['stab_tilt_dps'] = stab_tilt_dps     
            pt_connect_if.set_tilt_speed_ratio(stab_speed_ratio)

        pos_needs_update = False
        if (last_tilt_dir != adj_tilt_dir):
            pos_needs_update = True
        else:
            pos_needs_update = abs(adj_tilt_goal_delta) > pos_deg and abs(adj_tilt_change) > move_deg
            if adj_tilt_dir == 1:
                pos_needs_update = pos_needs_update and adj_tilt_goal > last_tilt_goal and abs(adj_tilt_goal - last_tilt_goal) > move_deg
            else:
                pos_needs_update = pos_needs_update and adj_tilt_goal < last_tilt_goal and abs(adj_tilt_goal - last_tilt_goal) > move_deg
             

        if pos_needs_update == True:
            stab_data_dict['stab_tilt_goal'] = adj_tilt_goal
            stab_data_dict['stab_tilt_dir'] = adj_tilt_dir
            pt_connect_if.goto_to_tilt_position(adj_tilt_goal)
            logger.log_info("Stab Tilt Position updated: " + str([adj_tilt_goal_delta, stab_speed_ratio, adj_tilt_goal]))
    return stab_data_dict, stab_settings_dict


PAN_TILT_STAB_PROCESSES_DICT['pt_stab_1'] = {'process_function': pt_stab_1, 
                                             'default_settings_dict': pt_stab_1_settings}


############################

# pt_stab_2_settings = {
#     # Required Fields
#     'stab_update_rate': 10,
#     'stab_reset_time_sec': 3.0,
#     'stab_num_avg': 3,
#     'stab_pt_min_speed_ratio': 0.1,
#     'stab_pt_max_speed_ratio': 0.9,

#     # Custom Fields. Automatically Populated in RUI
#     'stab_controls_dict': {
#         'nav_deg': 1.0,
#         'pos_deg': 1.0,
#         'vel_deg': 10.0,
#         'move_deg': 2.0
#     }
# }

# def pt_stab_2(pt_connect_if, 
#                         stab_data_dict, 
#                         stab_settings_dict, 
#                         goto_position = [0,0],
#                         stab_pan_enabled = 1, 
#                         stab_tilt_enabled = 1
#                         ):

#     #logger.log_info("******")
#     #logger.log_info("*** Stabs Solution Update Starting ***")
#     #logger.log_info("******")
    

#     pos_deg = stab_settings_dict['pos_deg']
#     vel_deg = stab_settings_dict['vel_deg']
        

#     ##########################
#     # APPLY PT UPDATES IF NEEDED
#     [pan_goal, tilt_goal] = goto_position

#     pan_deg = stab_data_dict['pan_deg']
#     pan_dps = stab_data_dict['pan_dps']
#     pan_speed_start_ratio = stab_data_dict['pan_speed_start_ratio']
#     tilt_deg = stab_data_dict['tilt_deg']
#     tilt_dps = stab_data_dict['tilt_dps']
#     tilt_speed_start_ratio = stab_data_dict['tilt_speed_start_ratio']
#     pan_tilt_max_speed_dps = stab_data_dict['pan_tilt_max_speed_dps']
#     stab_min_dps = stab_settings_dict['stab_pt_min_speed_ratio'] * pan_tilt_max_speed_dps 
#     stab_max_dps = stab_settings_dict['stab_pt_max_speed_ratio'] * pan_tilt_max_speed_dps


#     # pan_adj = stab_data_dict['stab_pan_adj']
#     # pan_adj_last = stab_data_dict_last['stab_pan_adj']
#     # adj_pan_delta = abs(pan_adj - pan_adj_last)
#     # adj_pan_goal = pan_goal + pan_adj
#     # adj_pan_goal_delta = (adj_pan_goal - pan_deg)
#     # #logger.log_info("Stab checking pan conditions: " + str([stab_pan_enabled, adj_pan_delta, adj_pan_goal_delta, pos_deg, vel_deg]))
#     # if stab_pan_enabled == True:
#     #     if adj_pan_delta > pos_deg: # or abs(adj_pan_goal_delta)  > pos_deg :
#     #         stab_data_dict['stab_pan_goal'] = adj_pan_goal
            
#     #         stab_pan_dps = stab_min_dps + (stab_max_dps - stab_min_dps) * abs(adj_pan_goal_delta) / vel_deg
#     #         stab_data_dict['stab_pan_dps'] = stab_pan_dps
#     #         stab_speed_ratio = stab_pan_dps / pan_tilt_max_speed_dps
#     #         pt_connect_if.set_pan_speed_ratio(stab_speed_ratio)
#     #         pt_connect_if.goto_to_pan_position(adj_pan_goal)
#     #         #logger.log_info("Stab Control updated: " + str([adj_pan_delta, stab_speed_ratio, adj_pan_goal]))
    

#     last_tilt_dps = copy.deepcopy(stab_data_dict['stab_tilt_dps'])
#     last_tilt_goal = copy.deepcopy(stab_data_dict['stab_tilt_goal'])
#     tilt_adj = stab_data_dict['stab_tilt_adj']
#     adj_tilt_goal = round(tilt_goal + tilt_adj,1)
#     adj_tilt_goal_delta = (adj_tilt_goal - tilt_deg)
    
#     #logger.log_info("Stab checking tilt conditions: " + str([stab_tilt_enabled, adj_tilt_goal_delta, pos_deg, vel_deg]))
#     if stab_tilt_enabled == True:
#         if abs(adj_tilt_goal_delta) > vel_deg:
#             stab_speed_ratio = tilt_speed_start_ratio
#             stab_tilt_dps = round(stab_speed_ratio * pan_tilt_max_speed_dps,1)
#             stab_data_dict['stab_tilt_dps'] = stab_tilt_dps
#         else:
#             stab_tilt_dps = round(stab_min_dps + (stab_max_dps - stab_min_dps) * abs(adj_tilt_goal_delta) / vel_deg, 1)
#             stab_data_dict['stab_tilt_dps'] = stab_tilt_dps
#             stab_speed_ratio = stab_tilt_dps / pan_tilt_max_speed_dps

#         if stab_tilt_dps !=  last_tilt_dps:          
#             pt_connect_if.set_tilt_speed_ratio(stab_speed_ratio)

#         if abs(adj_tilt_goal_delta)  > pos_deg or abs(last_tilt_goal - adj_tilt_goal) > pos_deg:
#             stab_data_dict['stab_tilt_goal'] = adj_tilt_goal
#             pt_connect_if.goto_to_tilt_position(adj_tilt_goal)
#             #logger.log_info("Stab Control updated: " + str([adj_tilt_delta, stab_speed_ratio, adj_tilt_goal]))
#     return stab_data_dict, stab_settings_dict


# PAN_TILT_STAB_PROCESSES_DICT['pt_stab_2'] = {'process_function': pt_stab_2, 
#                                              'default_settings_dict': pt_stab_2_settings}



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