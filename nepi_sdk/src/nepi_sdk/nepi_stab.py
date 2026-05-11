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
from nepi_sdk import nepi_predict



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_stab"
logger = Logger(log_name = log_name)


STAB_DEFAULT_SOURCE = nepi_sdk.get_base_namespace() + '/navposes/base_frame/navpose'

STAB_DATA_NAMES = ['heading_deg','roll_deg','pitch_deg',]


BLANK_STAB_SETTINGS_DICT = {
    
    'stab_update_rate': 0.3,
    'stab_goal_deg': 1.0,
    'stab_move_deg': 2.0,
    'stab_move_ratio': 1.0,
    'stab_reset_time_sec': 3.0,

    'max_pan_dps': 10.0,
    'max_tilt_dps': 10.0,
    'avg_move_delay': 1.0,

    'num_adj_avg': 1
}


BLANK_STAB_DATA_DICT = {
    'data_time': 0.0,
    'process_time': 0.0,
    'predict_time': 0.0,

    'roll_deg': -999,
    'roll_est': -999,

    'pitch_deg': -999,
    'pitch_est': -999,

    'heading_deg': -999,
    'heading_est': -999,

    'pan_dps': 0.0,
    'pan_deg': 0.0,
    'pan_adjs': [],
    'pan_adj': 0.0,

    'tilt_dps': 0.0,
    'tilt_deg': 0.0,
    'tilt_adjs': [],
    'tilt_adj': 0.0    
}
   
def get_blank_stab_settings_dict():
    return copy.deepcopy(BLANK_STAB_SETTINGS_DICT)

def get_blank_stab_data_dict():
    return copy.deepcopy(BLANK_STAB_DATA_DICT)

#########################
# Stab Process Function
#########################

PREDICT_DEFAULT_LOG_TIME = 5
PREDICT_DEFAULT_LOG_RATE = 2
PREDICT_DEFAULT_PREDICT_TIME = 1
PREDICT_DEFAULT_QUALITY_FILTER = 0.5

PREDICT_DICT = {
    'enabled': True,
    'source_topic': 'None',
    'data_names_list': [],
    'max_log_sec': PREDICT_DEFAULT_LOG_TIME,
    'max_log_hz': PREDICT_DEFAULT_LOG_RATE,
    'predict_time_sec': PREDICT_DEFAULT_PREDICT_TIME,
    'quality_filter': PREDICT_DEFAULT_QUALITY_FILTER,
    'process_dict': dict(),

}

PROCESS_DEFAULT_LOG_TIME = 5
PROCESS_DEFAULT_LOG_RATE = 2
PROCESS_DEFAULT_MIN_SAMPLES = 10
PROCESS_DEFAULT_SENSITIVITY = 0.5

BLANK_PREDICT_PROCESS_DICT = {
    'process_name': '',
    'enabled': True,
    'min_samples': PROCESS_DEFAULT_MIN_SAMPLES,
    'max_process_sec': PROCESS_DEFAULT_LOG_TIME,
    'max_process_hz': PROCESS_DEFAULT_LOG_RATE,
    'sensitivity': PROCESS_DEFAULT_SENSITIVITY,
    'weight': 1,
    'arg_names': ['None'],
    'arg_values': [0.0]
}


BLANK_PREDICT_DATAS_DICT = {
    'data_names_list': [],
    'data_dict': dict(),
    'num_samples': 0,
    'latest_time_sec': 0.0,
    'latest_data': [],
    'oldest_time_sec': 0.0
}


def rotate_enu_angles(rpy_vector, angle_deg, axis='z'):
    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    
    if axis == 'x': # East
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == 'y': # North
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == 'z': # Up
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")
    new_vector = np.dot(R, rpy_vector)
    return new_vector

def update_data_from_msg(msg, predict_data_dict, predict_settings_dict, verbose = False):
    navpose_dict = nepi_nav.convert_navpose_msg2dict(msg)
    #######################
    # Update predict_data_dict 
    new_data = []
    data_names = STAB_DATA_NAMES

    data_len = len(data_names)
    for data_name in data_names:
        if data_name in navpose_dict.keys():
            data = navpose_dict[data_name]
        else:
            data = -999
        new_data.append(data)

    timestamp = nepi_utils.get_time()
    if verbose == True:
        logger.log_info("Stabs Got first update inputs: " + str([timestamp,new_data,predict_data_dict,predict_settings_dict]) )
    predict_data_dict = nepi_predict.update_datas_dict(timestamp,new_data,predict_data_dict,predict_settings_dict)

    return predict_data_dict


BLANK_PREDICT_SOLUTION_DICT = {
    'data_names_list': [],
    'active_processes': [],
    'results_dict': dict(),
    'data_time_sec': 0.0,
    'process_time_sec': 0.0,
    'predict_time_sec': 0.0,
    'predict_list': [],
    'quality_list': []
}

def process_data_from_dict(predict_data_dict, predict_settings_dict, stab_settings_dict, stab_data_dict = None):
    data_names = STAB_DATA_NAMES
    data_len = len(data_names)

    latest_data = predict_data_dict['latest_data']
    if len(latest_data) == 0:
        latest_data = [0] * data_len

    blank_stab = get_blank_stab_data_dict()
    if stab_data_dict is None:
        stab_data_dict = blank_stab
    else:
        for key in blank_stab.keys():
            if key not in stab_data_dict.keys():
                stab_data_dict[key] = blank_stab[key]

    pan_dps = stab_data_dict['pan_dps'] 
    pan_deg = stab_data_dict['pan_deg'] 
    tilt_dps = stab_data_dict['tilt_dps'] 
    tilt_deg = stab_data_dict['tilt_deg'] 


    start_time = nepi_utils.get_time()
    #######################
    # predict solution
    #### NEED TO ADD AUTO ADJUST BASED ON CURRENT PAN TILT STATES
    predict_time_sec = predict_settings_dict['predict_time_sec']
    ###################################################
    if predict_time_sec > .1:
        solution_dict = nepi_predict.predict(predict_data_dict,predict_settings_dict)
        start_time = nepi_utils.get_time()

    else:        

        solution_dict = {
            'data_names_list': data_names,
            'active_processes': [],
            'results_dict': dict(),
            'data_time_sec': predict_data_dict['latest_time_sec'],
            'process_time_sec': start_time,
            'predict_time_sec': start_time,
            'predict_list': latest_data,
            'quality_list': [1] * data_len
            }

    ######################
    # Update stab_data_dict
    if solution_dict is not None:
      
        results_dict = dict()
        for i,data_name in enumerate(data_names):
            cur_deg = latest_data[i]
            cur_est = solution_dict['predict_list'][i]
            results_dict[data_name] = [cur_deg,cur_est]


        stab_data_dict['process_time'] = solution_dict['process_time_sec']
        stab_data_dict['predict_time'] = solution_dict['predict_time_sec']

        if results_dict['heading_deg'][0] == -999:
            results_dict['heading_deg'] = [0,0]
        stab_data_dict['heading_deg'] = results_dict['heading_deg'][0]
        stab_data_dict['heading_est'] = results_dict['heading_deg'][1]

        stab_data_dict['roll_deg'] = results_dict['roll_deg'][0]
        stab_data_dict['roll_est'] = results_dict['roll_deg'][1]

        stab_data_dict['pitch_deg'] = results_dict['pitch_deg'][0]
        stab_data_dict['pitch_est'] = results_dict['pitch_deg'][1]

        ##########################
        # Calculate pan tilt adjustments
        pan_radians = np.radians(pan_deg)
        tilt_radians = np.radians(tilt_deg)
        rpy_vector = [stab_data_dict['roll_est'], stab_data_dict['pitch_est'], stab_data_dict['heading_est'] ]
        if -999 not in rpy_vector:
            [ar,ap,ay] = rpy_vector
            [art,apt,ayt]  = rotate_enu_angles([ar,ap,ay],tilt_deg,'y')
            [ar,ap,ay] = [art,apt,ayt]
            [arp,app,ayp]  = rotate_enu_angles([ar,ap,ay],pan_deg,'z')
            [ar,ap,ay] = [arp,app,ayp]

            num_avg = p_adjs = stab_settings_dict['num_adj_avg']
            ####
            p_adj = 0 #####
            pan_adjs = stab_data_dict['pan_adjs']
            if (len(pan_adjs) >= num_avg):
                pan_adjs.pop(0)
            pan_adjs.append(p_adj)
            # pan_adj =  sum(pan_adjs) / len(pan_adjs)
            pan_adj = 0
            
            ####

            t_adj = -1 * (-1 * ar * np.sin(pan_radians) + ap * np.cos(pan_radians))
            tilt_adjs = stab_data_dict['tilt_adjs']
            if (len(tilt_adjs) >= num_avg):
               tilt_adjs.pop(0)
            tilt_adjs.append(t_adj)
            # tilt_adj =  sum(tilt_adjs) / len(tilt_adjs)
            tilt_adj = t_adj

            stab_data_dict['pan_adjs'] = pan_adjs
            stab_data_dict['pan_adj'] = pan_adj
            stab_data_dict['tilt_adjs'] = tilt_adjs 
            stab_data_dict['tilt_adj'] = tilt_adj  


    return stab_data_dict
    