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
import cv2

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_process
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data as nepi_data

from nepi_interfaces.msg import ProcessResultsTrack


from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_obstacles"
logger = Logger(log_name = log_name)


########################

RESULTS_PUB_MSG = ProcessResultsTrack
RESULTS_PUB_DICT = nepi_sdk.convert_msg2dict(RESULTS_PUB_MSG())
RESULTS_PUB_TOPIC = 'track'

DEFAULT_PROCESS = 'track_1'

########################
## Process Utility Functions

BEST_FILTER_OPTIONS = ['SMALLEST','LARGEST']

def filter_by_classes(targets_dict_list, class_filter_list):
    #print(targets_dict_list)

    if len(class_filter_list) == 0:
      filtered_targets = targets_dict_list
    else:
      filtered_targets = []
      for name in class_filter_list:
          for target_dict in targets_dict_list:
              if target_dict['name'] == name:
                  filtered_targets.append(target_dict)
                  #logger.log_info("Added target with name: " + str(name))

    # for target_dict in filtered_targets:   
    #     logger.log_info("Returning target with name: " + str(name))
    return filtered_targets
    


def filter_by_area(targets_dict_list, size_min_filter = .01, size_max_filter = .99):
    #print(targets_dict_list)

    filtered_targets = []

    for target_dict in targets_dict_list:
        target_area = target_dict['area_ratio']
        if target_area >= size_min_filter and target_area <= size_max_filter:
            filtered_targets.append(target_dict)
    #logger.log_info("Got Area filtered_targets: " + str(filtered_targets))
    return filtered_targets



# def filter_by_range(self,targets_dict_list, size_min_filter = .01, size_max_filter = .99):
#     ################
#     # Filter by min max range and angles
#     filtered_dict_list = []
#     cur_position = copy.deepcopy(self.current_position)
#     if cur_position is not None:
#       [cur_pan,cur_tilt] = [cur_position[0],cur_position[1]]
#       range_min = self.track_range_min_m
#       range_max = self.track_range_max_m
#       pan_min = self.track_pan_min_deg
#       pan_max = self.track_pan_max_deg
#       tilt_min = self.track_tilt_min_deg
#       tilt_max = self.track_tilt_max_deg

#       for target_dict in targets_dict_list:
#           target_valid = True
#           range_m = target_dict['range_m']
#           if (range_m < range_min or range_m > range_max) and range_m != -999:
#             target_valid = False
#           target_pan_angle = target_dict['azimuth_deg']
#           pan_angle =  cur_pan + target_pan_angle
#           if (pan_angle < pan_min or pan_angle > pan_max) and target_pan_angle != -999:
#             target_valid = False
#           target_tilt_angle = cur_pan + target_dict['elevation_deg']
#           tilt_angle =  cur_tilt + target_tilt_angle
#           if (tilt_angle < tilt_min or tilt_angle > tilt_max) and target_tilt_angle != -999:
#             target_valid = False
#           if target_valid == True:
#             filtered_dict_list.append(target_dict)
#           #self.msg_if.pub_warn("Range Angle Filter returned: " + str(target_dict['target_name']) + " : " + str(target_valid) )
#           #self.msg_if.pub_warn(str([range_m,cur_pan,cur_tilt]))
#           #self.msg_if.pub_warn(str([range_m,target_pan_angle,target_tilt_angle]))
#           #self.msg_if.pub_warn(str([range_m,pan_angle,tilt_angle]))
#     return filtered_dict_list

# def filter_by_bearings(self,targets_dict_list):
#     ################
#     # Filter by min max range and angles
#     filtered_dict_list = []
#     cur_position = copy.deepcopy(self.current_position)
#     if cur_position is not None:
#       [cur_pan,cur_tilt] = [cur_position[0],cur_position[1]]
#       range_min = self.track_range_min_m
#       range_max = self.track_range_max_m
#       pan_min = self.track_pan_min_deg
#       pan_max = self.track_pan_max_deg
#       tilt_min = self.track_tilt_min_deg
#       tilt_max = self.track_tilt_max_deg

#       for target_dict in targets_dict_list:
#           target_valid = True
#           range_m = target_dict['range_m']
#           if (range_m < range_min or range_m > range_max) and range_m != -999:
#             target_valid = False
#           target_pan_angle = target_dict['azimuth_deg']
#           pan_angle =  cur_pan + target_pan_angle
#           if (pan_angle < pan_min or pan_angle > pan_max) and target_pan_angle != -999:
#             target_valid = False
#           target_tilt_angle = cur_pan + target_dict['elevation_deg']
#           tilt_angle =  cur_tilt + target_tilt_angle
#           if (tilt_angle < tilt_min or tilt_angle > tilt_max) and target_tilt_angle != -999:
#             target_valid = False
#           if target_valid == True:
#             filtered_dict_list.append(target_dict)
#           #self.msg_if.pub_warn("Range Angle Filter returned: " + str(target_dict['target_name']) + " : " + str(target_valid) )
#           #self.msg_if.pub_warn(str([range_m,cur_pan,cur_tilt]))
#           #self.msg_if.pub_warn(str([range_m,target_pan_angle,target_tilt_angle]))
#           #self.msg_if.pub_warn(str([range_m,pan_angle,tilt_angle]))
#     return filtered_dict_list


def filter_by_threshold(targets_dict_list, threshold_filter):
    #print(targets_dict_list)

    filtered_targets = []

    for target_dict in targets_dict_list:
        prob = target_dict['confidence']
        if prob >= threshold_filter:
            filtered_targets.append(target_dict)
    #logger.log_info("Got Area filtered_targets: " + str(filtered_targets))
    return filtered_targets

def find_best(targets_dict_list, best_filter = 'LARGEST'):
    #print(tracks_dict_list)
    best_target = None
    for target_dict in targets_dict_list:
        
        best = True

        if best_target is not None:
            bsize = best_target['area_ratio']
            tsize = target_dict['area_ratio']
            bprob = best_target['confidence']
            tprob = target_dict['confidence']
            if best_filter == 'LARGEST' and tsize < bsize:
                best = False
            elif best_filter == 'SMALLEST' and tsize > bsize:
                best = False
            elif best_filter == 'PROPABILITY' and tprob < bprob:
                best = False

        if best == True:
            best_target = target_dict
    #logger.log_info("Got filtered_dict " + str(filtered_track))
    return best_target

def update_results(results_dict, track_dict):
    results_pub_dict = None
    if track_dict is not None:
        results_dict = nepi_data.set_data_values(results_dict, track_dict)
    
        results_pub_dict = copy.deepcopy(RESULTS_PUB_DICT)
        #print([results_dict,results_pub_dict])
        for key in track_dict.keys():
            if key in results_pub_dict.keys():
                results_pub_dict[key] = track_dict[key]

    timestamp = nepi_data.get_datum_value(results_dict, 'timestamp')
    if timestamp == -999:
        age_sec = -999
    else:
        age_sec =  nepi_utils.get_time() - timestamp
    results_dict = nepi_data.set_datum_value(results_dict, 'age_sec', age_sec)
    return results_dict, results_pub_dict


########################
## Process Functions   
#######################
processes_dict = dict()
functions_dict = dict()



########################
## Process 1   



track_1_dict = {
   
    'data_dict': dict(
        targets_dict_list = [], 
        last_track_time = 0,
        last_track_dict = None
    ),


    'controls_dict': dict(

        class_filters = {"type":"Selections", "default":['ALL'], "options":[], 
                   # OPTIONAL
                   'display_name':'Select Classes', 'description':'Set Class Filters', 'hidden':False}, 

        size_min_filter = {
            'type': 'FloatSlider', 'default': 0.001, 'bounds': [0.0, 1.0], 'round_value': 3,
            'display_name': 'Max Range (m)',
            'description': 'Ignore targets with pixel areas less than min.', 'hidden': False},

        size_max_filter = {
            'type': 'FloatSlider', 'default': 0.99, 'bounds': [0.0, 1.0], 'round_value': 3,
            'display_name': 'Max Range (m)',
            'description': 'Ignore targets with pixel areas larger than max.', 'hidden': False},

        threshold_filter = {
            'type': 'FloatSlider', 'default': 0.3, 'bounds': [0.0, 1.0], 'round_value': 1,
            'display_name': 'Max Range (m)',
            'description': 'Ignore targets with confidance lower than threshold.', 'hidden': False},

        best_filter = {"type":"Selection", "default":['LARGEST'], "options":BEST_FILTER_OPTIONS, 
                   # OPTIONAL
                   'display_name':'Best Filter', 'description':'Set Best Filte', 'hidden':False}, 

    ),


    'results_dict': dict(

        timestamp = {"type":"Float", "value":-999,
                    # OPTIONAL
                    'display_name':'Timestamp', 'description':'Timestamp', 'hidden':True},

        age_sec = {"type":"Float", "value":-999, 'round_value': 3,
                    # OPTIONAL
                    'display_name':'Age (Sec)', 'description':'Age in seconds', 'hidden':False, 'round_display': 3,},

        azimuth_deg = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Azimuth (Deg)', 'description':'Degrees in horizontal axis to tracked target', 'hidden':False, 'round_display': 1,},

        elevation_deg = {"type":"Float", "value":-999, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Elevation (Deg)', 'description':'Degrees in vertical axis to tracked target', 'hidden':False, 'round_display': 1,},

        range_m = {"type":"Float", "value":2.0, 'round_value': 2,
                    # OPTIONAL
                    'display_name':'Range (M)', 'description':'Range in meters to tracked target', 'hidden':False, 'round_display': 1,},
    ),

}


def track_1_process(data_dict, controls_dict, results_dict):
    start_time = nepi_utils.get_time()
    last_data_dict = copy.deepcopy(data_dict)
    last_results_dict = copy.deepcopy(results_dict)
    #logger.log_warn("Process Results Got Data and Controls: " + str([data_dict, controls_dict]), throttle_s = 5)
    results_pub_dict = None
    
    track_dict = None
    filtered_targets = data_dict.get('targets_dict_list', [])
    if filtered_targets is None:
        filtered_targets = []

    class_filters = nepi_controls.get_control_value(controls_dict, 'class_filters')
    filtered_targets = filter_by_classes(filtered_targets, class_filters)

    size_max_filter = nepi_controls.get_control_value(controls_dict, 'size_max_filter')
    size_min_filter = nepi_controls.get_control_value(controls_dict, 'size_min_filter')
    filtered_targets = filter_by_area(filtered_targets, size_min_filter = size_min_filter, size_max_filter = size_max_filter)

    threshold_filter = nepi_controls.get_control_value(controls_dict, 'threshold_filter')
    filtered_targets = filter_by_threshold(filtered_targets, threshold_filter)

    if len(filtered_targets) > 0:
        best_filter = nepi_controls.get_control_value(controls_dict, 'best_filter')
        track_dict = find_best(filtered_targets, best_filter = best_filter)

    [results_dict, results_pub_dict] = update_results(results_dict, track_dict)
    #logger.log_warn("Process Completed: " + str([data_dict, controls_dict, results_dict, results_pub_dict]), throttle_s = 5)
    return data_dict, controls_dict, results_dict, results_pub_dict


processes_dict = nepi_process.update_processes_dict(processes_dict, process_name = 'track_1', process_dict = track_1_dict)
#logger.log_warn("Updated processes dict: " + str(processes_dict))
functions_dict['track_1'] = track_1_process

########################
## Processes Init Dict  
PROCESSES_DICT = copy.deepcopy(processes_dict)
FUNCTIONS_DICT = functions_dict