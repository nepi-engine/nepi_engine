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

from nepi_sdk import nepi_utils



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_lines"
logger = Logger(log_name = log_name)



PREDICT_DEFAULT_LOG_TIME = 5
PREDICT_DEFAULT_LOG_RATE = 2
PREDICT_DEFAULT_PREDICT_TIME = 1
PREDICT_DEFAULT_QUALITY_FILTER = 0.5

##################


BLANK_PREDICT_DICT = {
    'enabled': True,
    'source_topic': 'None',
    'data_names_list': [],
    'max_log_sec': PREDICT_DEFAULT_LOG_TIME,
    'max_log_hz': PREDICT_DEFAULT_LOG_RATE,
    'predict_time_sec': PREDICT_DEFAULT_PREDICT_TIME,
    'quality_filter': PREDICT_DEFAULT_QUALITY_FILTER,
    'process_dict': dict(),

}



##################
EXAMPLE_DATA_DICT = {
    'FLOAT_TIME_MSEC_1': ['VAR_VALUE_1', 'VAR_VALUE_2'],
    'FLOAT_TIME_MSEC_2': ['VAR_VALUE_1', 'VAR_VALUE_2']
}

EXAMPLE_DATAS_DICT = {
    'data_names_list': ['VAR_NAME_1','VAR_NAME_1'],
    'data_dict': EXAMPLE_DATA_DICT,
    'num_samples': 2,
    'latest_time_sec': 0.0,
    'oldest_time_sec': 0.0
}

BLANK_DATAS_DICT = {
    'data_names_list': [],
    'data_dict': dict(),
    'num_samples': 0,
    'latest_time_sec': 0.0,
    'oldest_time_sec': 0.0
}

##################

# -999 used for None and no result values

BLANK_RESULT_DICT = {
    'process_name': '',
    'weight'
    'has_result': False,
    'predict_list': [],
    'quality_list': [],
}

BLANK_SOLUTION_DICT = {
    'data_names_list': [],
    'active_processes': [],
    'results_dict': dict(),
    'data_time_sec': 0.0,
    'process_time_sec': 0.0,
    'predict_time_sec': 0.0,
    'predict_list': [],
    'quality_list': []
}


##########################
# Misc Util Functions
#########################

def create_predict_dict(data_names_list):
  predict_dict = copy.deepcopy(BLANK_PREDICT_DICT)
  return predict_dict

def update_predict_dict(key_name, key_value, predict_dict):
  if key_name != 'data_names_list' and key_name != 'process_dict':
    if key_name in predict_dict.keys():
        predict_dict[key_name] = key_value
  return predict_dict

def update_process_dict(process_name, key_name, key_value, predict_dict):
  success = False
  if process_name in predict_dict['process_dict'].keys():
    process_dict = predict_dict['process_dict'][process_name]
    if key_name in process_dict.keys():
        process_dict[key_name] = key_value
        predict_dict['process_dict'][process_name] = process_dict
        success = True
  return predict_dict


def create_datas_dict(predict_dict):
    data_names = predict_dict['data_names_list']
    datas_dict = copy.deepcopy(BLANK_DATAS_DICT)
    datas_dict['data_names_list'] = data_names
    return datas_dict
    

def update_datas_dict(data_time_sec, data_list, datas_dict, predict_dict):
    data_time_msec = int(round(data_time_sec * 1000),0)
    data_names = datas_dict['data_names_list']
    num_vars = len(data_names)
    data_dict = datas_dict['data_dict']
    data_times = list(data_dict.keys())
    num_data = len(data_times)

    if len(data_list) != num_vars:
       return datas_dict

    if num_data == 0:
       datas_dict['data_dict'][data_time_msec] = data_list
       return datas_dict
    
    oldest_time = min(data_times)
    latest_time = max(data_times)

    max_log_sec = predict_dict['max_log_sec']
    if (latest_time - oldest_time) > max_log_sec:
        filter_time = latest_time - max_log_sec
        purge_times = [item for sublist in data_times for item in sublist if item < filter_time]
        for purge_time in purge_times:
            data_dict.pop(purge_time, None)  
    
    max_log_hz = predict_dict['max_log_hz']
    time_step = float(1) / max_log_hz
    if (data_time_msec - latest_time) >= time_step:
       data_dict[data_time_msec] = data_list
    
    datas_dict['data_dict'] = data_dict
    data_times = list(data_dict.keys())
    num_data = len(data_times)
    datas_dict['num_samples'] = num_data
    datas_dict['latest_time_sec'] = max(data_times)
    datas_dict['oldest_time_sec'] = min(data_times)

    return datas_dict



def update_datas_dict_from_dict(data_time_sec, data_dict, datas_dict):
    data_names_list = datas_dict['data_names_list']
    data_list = []
    for data_name in data_names_list:
        if data_name in data_dict.keys():
            data = data_dict[data_name]
        else:
            data = -999
        data_list.append(data)
    datas_dict = update_datas_dict(data_time_sec, data_list, datas_dict)
    return datas_dict


def filter_datas_dict(datas_dict, process_dict):
    data_dict = datas_dict['data_dict']
    data_times = list(data_dict.keys())
    num_data = len(data_times)

    if num_data < 2:
       return datas_dict
    
    oldest_time = min(data_times)
    latest_time = max(data_times)

    max_process_sec = process_dict['max_process_sec']
    if (latest_time - oldest_time) > max_process_sec:
        filter_time = latest_time - max_process_sec
        purge_times = [item for sublist in data_times for item in sublist if item < filter_time]
        for purge_time in purge_times:
            data_dict.pop(purge_time, None)  
    
    data_times = list(data_dict.keys())
    sorted_times = sorted(data_times, reverse=True)
    filtered_dict = dict()
    last_time = None
    max_process_hz = process_dict['max_process_hz']
    time_step = float(1) / max_process_hz
    for data_time in sorted_times:
        valid_time = False
        if last_time is None:
            valid_time = True
        elif (data_time - last_time) >= time_step:
            valid_time = True
        if valid_time == True:
            try:
                filtered_dict[data_time] = data_dict[data_time]
                last_time = data_time
            except:
                pass
    
    datas_dict['data_dict'] = filtered_dict
    data_times = list(filtered_dict.keys())
    num_data = len(data_times)
    datas_dict['num_samples'] = num_data
    datas_dict['latest_time_sec'] = max(data_times)
    datas_dict['oldest_time_sec'] = min(data_times)

    return datas_dict
   


#########################
# Predict Process Functions
#########################

BLANK_PROCESS_DICT = {
    'process_name': '',
    'enabled': True,
    'max_process_sec': PREDICT_DEFAULT_LOG_TIME,
    'max_process_hz': PREDICT_DEFAULT_LOG_RATE,
    'sensitivity': PREDICT_DEFAULT_PREDICT_TIME,
    'weight': 1,
    'arg_names': ['None'],
    'arg_values': [0.0]
}


PREDICT_PROCESS_OPTIONS_DICT = dict()

PREDICT_PROCESS_ARGS_DICT = dict()

def set_process_setting(process_name, key_name, key_value, predict_dict):
  success = False
  if process_name in predict_dict['process_dict'].keys():
    process_dict = predict_dict['process_dict'][process_name]
    if key_name in process_dict.keys():
        process_dict[key_name] = key_value
        predict_dict['process_dict'][process_name] = process_dict
        success = True
  return predict_dict


def get_process_arg(process_name, arg_name, predict_dict):
  success = False
  arg_values = None
  if process_name in predict_dict['process_dict'].keys():
    process_dict = predict_dict['process_dict'][process_name]
    if 'arg_names' in process_dict:
        arg_names = process_dict['arg_names']
        arg_ind = arg_names.index(arg_name)
        if arg_ind != -1:
            arg_values = process_dict['arg_values'][arg_ind]
            success = True
  return arg_values

def set_process_arg(process_name, arg_name, arg_values, predict_dict):
  success = False
  if process_name in predict_dict['process_dict'].keys():
    process_dict = predict_dict['process_dict'][process_name]
    if 'arg_names' in process_dict:
        arg_names = process_dict['arg_names']
        arg_ind = arg_names.index(arg_name)
        if arg_ind != -1:
            process_dict['arg_values'][arg_ind] = arg_values
            predict_dict['process_dict'][process_name] = process_dict
            success = True
  return predict_dict



#########################

def process_1(datas_dict, process_dict, predict_time_sec, min_samples=10):
    enabled = process_dict['enabled']
    data_names = datas_dict['data_names_list']
    num_vars = len(data_names)
    datas_dict = filter_datas_dict(datas_dict,process_dict)
    data_dict = datas_dict['data_dict']
    data_times = list(data_dict.keys())
    num_data = len(data_times)

    result_dict = copy.deepcopy(BLANK_RESULT_DICT)
    result_dict['process_name'] =  process_dict['process_name']

    if enabled == False or len(data_times) < min_samples:
        result_dict['weight'] = 0.0
        result_dict['predict_list'] = [-999] * num_vars
        result_dict['quality_list'] = [0] * num_vars
    else:
        sensitivity = process_dict['sensitivity']

        ###########################################
        ### RUN PREDICTION PROCESS

        ### FILTER OUT -999 values

        predict_step = predict_time_sec
        predict_list = data_dict[data_times[-1]]
        quality_list = [1] * num_vars
        

        ###########################################
        result_dict['weight'] =  process_dict['weight']
        result_dict['predict_list'] = predict_list
        result_dict['quality_list'] = quality_list
        result_dict['has_results'] = True
    return result_dict

PREDICT_PROCESS_OPTIONS_DICT['Process1'] = process_1

PREDICT_PROCESS_ARGS_DICT['Process1'] = {
    'arg_names': ['arg1', 'arg2'],
    'arg_values': [0.0, 0.0]
}



#########################

def process_sin_wave(datas_dict, process_dict, predict_time_sec, min_samples=10):
    enabled = process_dict['enabled']
    data_names = datas_dict['data_names_list']
    num_vars = len(data_names)
    datas_dict = filter_datas_dict(datas_dict,process_dict)
    data_dict = datas_dict['data_dict']
    data_times = list(data_dict.keys())
    num_data = len(data_times)

    result_dict = copy.deepcopy(BLANK_RESULT_DICT)
    result_dict['process_name'] =  process_dict['process_name']

    if enabled == False or len(data_times) < min_samples:
        result_dict['weight'] = 0.0
        result_dict['predict_list'] = [-999] * num_vars
        result_dict['quality_list'] = [0] * num_vars
    else:
        sensitivity = process_dict['sensitivity']

        ###########################################
        ### RUN PREDICTION PROCESS

        ### FILTER OUT -999 values

        predict_step = predict_time_sec
        predict_list = data_dict[data_times[-1]]
        quality_list = [1] * num_vars
        

        ###########################################
        result_dict['weight'] =  process_dict['weight']
        result_dict['predict_list'] = predict_list
        result_dict['quality_list'] = quality_list
        result_dict['has_results'] = True
    return result_dict

PREDICT_PROCESS_OPTIONS_DICT['Sin Wave'] = process_sin_wave

PREDICT_PROCESS_ARGS_DICT['Sin Wave'] = {
    'arg_names': ['arg1', 'arg2'],
    'arg_values': [0.0, 0.0]
}



#########################


def process_spline(data_dict, process_dict, min_samples=10):
    enabled = process_dict['enabled']
    data_names = datas_dict['data_names_list']
    num_vars = len(data_names)
    datas_dict = filter_datas_dict(datas_dict,process_dict)
    data_dict = datas_dict['data_dict']
    data_times = list(data_dict.keys())
    num_data = len(data_times)

    result_dict = copy.deepcopy(BLANK_RESULT_DICT)
    result_dict['process_name'] =  process_dict['process_name']

    if enabled == False or len(data_times) < min_samples:
        result_dict['weight'] = 0.0
        result_dict['predict_list'] = [-999] * num_vars
        result_dict['quality_list'] = [0] * num_vars
    else:
        sensitivity = process_dict['sensitivity']

        ###########################################
        ### RUN PREDICTION PROCESS

        ### FILTER OUT -999 values
        
        predict_list = data_dict[data_times[-1]]
        quality_list = [1] * num_vars

        ###########################################
        result_dict['weight'] =  process_dict['weight']
        result_dict['predict_list'] = predict_list
        result_dict['quality_list'] = quality_list
        result_dict['has_results'] = True
    return result_dict


PREDICT_PROCESS_OPTIONS_DICT['Spline'] = process_spline

PREDICT_PROCESS_ARGS_DICT['Spline'] = {
    'arg_names': ['arg1', 'arg2'],
    'arg_values': [0.0, 0.0]
}

#########################


PREDICT_PROCESS_OPTIONS = list(PREDICT_PROCESS_OPTIONS_DICT.keys())
for process in PREDICT_PROCESS_OPTIONS:
   BLANK_PREDICT_DICT['process_dict'][process] = copy.deepcopy(BLANK_PROCESS_DICT)
   BLANK_PREDICT_DICT['process_dict'][process]['arg_names'] = PREDICT_PROCESS_ARGS_DICT[process]['arg_names']
   BLANK_PREDICT_DICT['process_dict'][process]['arg_values'] = PREDICT_PROCESS_ARGS_DICT[process]['arg_values']




#########################
# Predict Function
#########################

def predict(datas_dict, predict_dict):
    result_dict = None
    enabled = predict_dict['enabled']
    if enabled == False:
        return result_dict
    
    data_names = datas_dict['data_names_list']
    num_vars = len(data_names)
    data_times = list(data_dict.keys())
    latest_time = max(data_times)
    num_data = len(data_times)
    
    ### Run Predict Processes
    results_dict = dict()
    active_processes = []
    predict_time_sec = predict_dict['predict_time_sec']
    process_names = list(PREDICT_PROCESS_OPTIONS_DICT.keys())
    for process_name in process_names:
        process_function = PREDICT_PROCESS_OPTIONS_DICT[process_name]
        if process_name in predict_dict['process_dict'].keys():
            process_dict = predict_dict['process_dict'][process_name]
            data_dict = datas_dict['data_dict']
            result_dict = process_function(data_dict, process_dict, predict_time_sec)
            if result_dict is not None:
                process_enabled = process_dict['enabled']
                has_results = result_dict['has_results']
                if process_enabled and has_results:
                    active_processes.append(process_name)
                results_dict[process_name] = result_dict

    ###########################################
    ### Merge Process Results

    results = []
    qaulities = []
    weights = []
    for process_name in active_processes:
        results.append(results_dict[process_name]['predict_list'])
        qaulities.append(results_dict[process_name]['quality_list'])
        weights.append(results_dict[process_name]['weight'])

    predict_list = [-999] * num_vars
    quality_list = [0] * num_vars
    quality_filter = predict_dict['quality_filter']
    for i, data_name in enumerate(data_names):
        dlist = []
        wlist = []
        qlist = []
        for i2, result in enumerate(results):
            if results[i2][i] != -999 and weights[i2] != 0 and qaulities[i2] >= quality_filter:
                dlist.append(results[i2][i])
                wlist.append(weights[i2])
                qlist.append(qaulities[i2])
        if len(dlist) > 0:
            predict_list[i] = np.average(dlist, weights=wlist)
            quality_list[i] = np.average(qlist, weights=wlist)
    
    ###########################################

    solution_dict = copy.deepcopy(BLANK_SOLUTION_DICT)
    solution_dict['results_dict'] = results_dict
    solution_dict['data_time_sec'] = latest_time
    solution_dict['predict_time_sec'] = latest_time + predict_time_sec
    solution_dict['process_time_sec'] = nepi_utils.get_time()
    solution_dict['active_processes'] = active_processes
    solution_dict['results_dict'] = results_dict
    solution_dict['predict_list'] = predict_list
    solution_dict['quality_list'] = quality_list
    return solution_dict


            