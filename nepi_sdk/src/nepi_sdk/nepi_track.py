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

import os
import copy


from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_targets

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header

from nepi_interfaces.msg import Targets

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_track"
logger = Logger(log_name = log_name)






#######################
## Misc Tracking Helper Functions
SOURCE_MESSAGE_DICT = {'Targets' : Targets}

BEST_FILTER_OPTIONS = nepi_targets.BEST_FILTER_OPTIONS

BLANK_DATA_DICT = {
    'targets_topic': 'None',
    'source_topic': 'None',
    'class_filter': 'None',
    'size_min_filter': 0.01,
    'size_max_filter': 0.99,
    'range_min_filter': 0.01,
    'range_max_filter': 1000,
    'threshold_filter': 0.01,
    'best_filter': 'LARGEST'
}


def get_source_namespaces( topics_list = None, types_list = None):
    namespace = []
    msg_type = 'nepi_interfaces/TargetingStatus'
    namespaces = nepi_sdk.find_topics_by_msg(msg_type, topics_list = topics_list, types_list = types_list)
    for i, namespace in enumerate(namespaces):
        namespaces[i] = os.path.dirname(namespaces[i])
    return namespaces 

'''
def get_classes_colors_list(classes_str_list):
    rgb_list = []
    if len(classes_str_list) > 0:
        cmap = plt.get_cmap('viridis')
        color_list = cmap(np.linspace(0, 1, len(classes_str_list))).tolist()
        for color in color_list:
            rgb = []
            for i in range(3):
                rgb.append(int(color[i]*255))
            rgb_list.append(rgb)
    return rgb_list
'''


# def convert_tacking_dict2msg(track_data_dict, log_name_list = []):
#   track_msg = None
#   if track_data_dict is None:
#     logger.log_info("Got None track dict", throttle_s = 5.0)
#   else:
#     try:
#         track_msg = Target()
#         track_msg.track_name = track_data_dict['track_name']
#         track_msg.track_uid = track_data_dict['track_uid']
#         track_msg.track_confidence = track_data_dict['track_confidence']

#         # 2D Data ENU Reference Frame
#         track_msg.xmin_pixel = track_data_dict['xmin_pixel']
#         track_msg.xmax_pixel = track_data_dict['xmax_pixel']

#         track_msg.ymin_pixel = track_data_dict['ymin_pixel']
#         track_msg.ymax_pixel = track_data_dict['ymax_pixel']

#         track_msg.width_pixels = track_data_dict['width_pixels']
#         track_msg.height_pixels = track_data_dict['height_pixels']

      
#         track_msg.area_pixels = track_data_dict['area_pixels']
#         track_msg.area_ratio = track_data_dict['area_ratio']
#         #track_msg.vel_pixels

#         # 3D Data in ENU Reference Frame
#         # track_msg.width_meters = track_data_dict['width_meters']
#         # track_msg.height_meters = track_data_dict['height_meters']
#         # track_msg.depth_meters = track_data_dict['depth_meters']
#         # track_msg.area_meters = track_data_dict['area_meters']

#         #track_msg.center_xyz_meters = track_data_dict['center_xyz_meters']

#         # Range, Bearing, Nav, and Pose Data ENU Reference Frame
#         track_msg.range_m = track_data_dict['range_m']
#         track_msg.azimuth_deg = track_data_dict['azimuth_deg']
#         track_msg.elevation_deg = track_data_dict['elevation_deg']
#         navpose_dict = track_data_dict['navpose_dict']
#         if len(navpose_dict.keys()) == 0:
#             navpose_msg = nepi_nav.convert_navpose_dict2msg(navpose_dict)
#             track_msg.source_nav_pose = navpose_msg       
#     except Exception as e:
#       track_msg = None
#       logger.log_warn("Failed to convert Target Data dict: " + str(e), throttle_s = 5.0)
#   return track_msg

def convert_track_msg2dict(track_msg, log_name_list = []):
  track_data_dict = None
  try:
    track_data_dict = nepi_sdk.convert_msg2dict(track_msg)
  except Exception as e:
    logger.log_warn("Failed to convert Target Data msg: " + str(e), throttle_s = 5.0)
  return track_data_dict



def get_best_from_targets(targets_dict_list,tracking_dict = BLANK_DATA_DICT):
   filtered_targets = targets_dict_list
   best_target = None
   for entry in BLANK_DATA_DICT.keys():
    if entry not in tracking_dict.keys():
       tracking_dict[entry] = BLANK_DATA_DICT[entry]
    
    class_filters = tracking_dict['class_filter']
    filtered_targets = nepi_targets.filter_by_classes(filtered_targets, [class_filters])

    size_max_filter = tracking_dict['size_max_filter']
    size_min_filter = tracking_dict['size_min_filter']
    filtered_targets = nepi_targets.filter_by_area(filtered_targets, size_min_filter = size_min_filter, size_max_filter = size_max_filter)

    threshold_filter = tracking_dict['threshold_filter']
    filtered_targets = nepi_targets.filter_by_threshold(filtered_targets, threshold_filter)
    
    if len(filtered_targets) > 0:
      best_filter = tracking_dict['best_filter']
      best_target = nepi_targets.find_best(filtered_targets, best_filter = best_filter)

            
    return best_target,tracking_dict    






# #########################
# # Stab Process Functions
# #########################


# PAN_TILT_STAB_DATA_DICT = {
#     # Required Fields
#     'data_time': 0.0,
#     'process_time': 0.0,

#     'roll_deg': -999,
#     'rolls_dps': [],
#     'roll_dps': -999,

#     'pitch_deg': -999,
#     'pitchs_dps': [],
#     'pitch_dps': -999,

#     'heading_deg': -999,
#     'headings_dps': [],
#     'heading_dps': -999,

#     'pan_tilt_max_speed_dps': 10,
#     'pan_deg': 0.0,
#     'pan_dps': 0.0,
#     'pan_speed_start_ratio': 1.0,
#     'pan_min_deg': -170,
#     'pan_max_deg': 170,


#     'tilt_deg': 0.0,
#     'tilt_dps': 0.0,
#     'tilt_speed_start_ratio': 1.0,
#     'tilt_min_deg': -50,
#     'tilt_max_deg': 50,


#     'track_pan_deg': 0.0,
#     'track_pan_adjs': [],
#     'track_pan_adj': 0.0,
#     'track_pan_goal': 0.0,
#     'track_pan_dir': 0,
#     'track_pan_dps': 0.0,
#     'track_pan_vel_rate': 0.0,
#     'track_pan_pos_rate': 0.0,  

#     'track_tilt_deg': 0.0,
#     'track_tilt_adjs': [],
#     'track_tilt_adj': 0.0,
#     'track_tilt_goal': 0.0,
#     'track_tilt_dir': 0,
#     'track_tilt_dps': 0.0,
#     'track_tilt_vel_rate': 0.0,
#     'track_tilt_pos_rate': 0.0,    

#     'last_track_time': None,
#     'last_pan_vel_time': 0,
#     'last_pan_pos_time': 0,
#     'last_tilt_vel_time': 0,
#     'last_tilt_pos_time': 0,
#      # Add Custom Fields Here

# }



# PAN_TILT_STAB_PROCESSES_DICT = dict()

# DEFAULT_PAN_TILT_STAB_PROCESS = 'pt_track_1'

# pt_track_1_settings = {
#     # Required Fields
#     'track_update_rate': 5,
#     'track_reset_time_sec': 3.0,
#     'track_num_avg': 2,
#     'track_pt_min_speed_ratio': 0.1,
#     'track_pt_max_speed_ratio': 0.9,

#     # Custom Fields. Automatically Populated in RUI
#     'track_controls_dict': {
#         'pos_deg': 2.0,
#         'vel_deg': 20.0,
#     }
# }

# def pt_track_1(pt_connect_if, 
#                         track_data_dict, 
#                         track_settings_dict, 
#                         goto_position,
#                         track_pan_enabled, 
#                         track_tilt_enabled
#                         ):

#     #logger.log_info("******")
#     #logger.log_info("*** Stabs Solution Update Starting ***")
#     #logger.log_info("******")
#     start_time = nepi_utils.get_time()
#     ##########################
#     # Gather Stab Settings
#     ##########################
#     [pan_goal, tilt_goal] = goto_position

#     pan_deg = track_data_dict['pan_deg']
#     pan_dps = track_data_dict['pan_dps']
#     pan_speed_start_ratio = track_data_dict['pan_speed_start_ratio']
#     tilt_deg = track_data_dict['tilt_deg']
#     tilt_dps = track_data_dict['tilt_dps']
#     tilt_speed_start_ratio = track_data_dict['tilt_speed_start_ratio']
#     pan_tilt_max_speed_dps = track_data_dict['pan_tilt_max_speed_dps']

#     track_min_dps = track_settings_dict['track_pt_min_speed_ratio'] * pan_tilt_max_speed_dps 
#     track_max_dps = track_settings_dict['track_pt_max_speed_ratio'] * pan_tilt_max_speed_dps
#     controls_dict = track_settings_dict['track_controls_dict']

  
#     ######################
#     ## Process Stab Tilt Adjustments
#     ######################
#     pos_deg = controls_dict['pos_deg']
#     vel_deg = controls_dict['vel_deg']

#     last_data_dict = copy.deepcopy(track_data_dict)
#     last_tilt_dps = last_data_dict['track_tilt_dps']
#     last_tilt_goal = last_data_dict['track_tilt_goal']
#     last_tilt_goal_delta = (last_tilt_goal - tilt_deg)
#     last_tilt_dir = last_data_dict['track_tilt_dir']


#     tilt_adj = track_data_dict['track_tilt_adj']
#     adj_tilt_goal = round(tilt_goal + tilt_adj,1)
#     adj_tilt_goal_delta = (adj_tilt_goal - tilt_deg)
#     adj_tilt_dir = 1 if adj_tilt_goal_delta > 0 else -1
#     adj_tilt_change = (last_tilt_goal - adj_tilt_goal)

    
#     if track_tilt_enabled == True:
#         # Process Velocity Adjustment
#         if abs(adj_tilt_goal_delta) > vel_deg : # or abs(adj_tilt_goal_delta) < pos_deg:
#             track_speed_ratio = tilt_speed_start_ratio
#             track_tilt_dps = round(track_speed_ratio * pan_tilt_max_speed_dps,1)
#         else:
#             track_tilt_dps = round(track_min_dps + (track_max_dps - track_min_dps) * abs(adj_tilt_goal_delta) / vel_deg, 1)
#             track_speed_ratio = track_tilt_dps / pan_tilt_max_speed_dps

#         if abs(track_tilt_dps - last_tilt_dps) > 1:   
#             last_time = track_data_dict['last_tilt_vel_time']
#             update_rate = float(1) / (start_time - last_time)
#             track_data_dict['last_tilt_vel_time'] = start_time
#             track_data_dict['track_tilt_vel_rate'] = round(update_rate,2) 
#             track_data_dict['track_tilt_dps'] = track_tilt_dps     
#             pt_connect_if.set_tilt_speed_ratio(track_speed_ratio)

#         # Process Position Adjustment
#         pos_needs_update = False
#         if (last_tilt_dir != adj_tilt_dir):
#             pos_needs_update = True
#         else:
#             #logger.log_warn("Stab Tilt Pos Check: " + str([abs(adj_tilt_goal_delta), pos_deg, abs(adj_tilt_change), pos_deg]) ) 
#             #logger.log_warn("Stab Tilt Pos Check: " + str([abs(adj_tilt_goal_delta) > pos_deg, abs(adj_tilt_change) > pos_deg]) ) 
#             pos_needs_update = abs(adj_tilt_goal_delta) > pos_deg or abs(adj_tilt_change) > pos_deg
#             if adj_tilt_dir == 1:
#                 pos_needs_update = pos_needs_update and \
#                                     adj_tilt_goal > last_tilt_goal and \
#                                     abs(adj_tilt_goal - last_tilt_goal) > pos_deg 
#             else:
#                 pos_needs_update = pos_needs_update and \
#                                     adj_tilt_goal < last_tilt_goal and \
#                                     abs(adj_tilt_goal - last_tilt_goal) > pos_deg 
#         if pos_needs_update == True:
#             last_time = track_data_dict['last_tilt_pos_time']
#             update_rate = float(1) / (start_time - last_time)
#             track_data_dict['last_tilt_pos_time'] = start_time
#             track_data_dict['track_tilt_pos_rate'] = round(update_rate,2)
#             track_data_dict['track_tilt_goal'] = adj_tilt_goal
#             track_data_dict['track_tilt_dir'] = adj_tilt_dir
#             pt_connect_if.goto_to_tilt_position(adj_tilt_goal)
#             #logger.log_info("Stab Tilt Position updated: " + str([adj_tilt_goal_delta, track_speed_ratio, adj_tilt_goal]))

            
#     return track_data_dict, track_settings_dict



# PAN_TILT_STAB_PROCESSES_DICT['pt_track_1'] = {'process_function': pt_track_1, 
#                                              'default_settings_dict': pt_track_1_settings}


# ############################

# pt_track_2_settings = {
#     # Required Fields
#     'track_update_rate': 5,
#     'track_reset_time_sec': 3.0,
#     'track_num_avg': 2,
#     'track_pt_min_speed_ratio': 0.1,
#     'track_pt_max_speed_ratio': 0.9,

#     # Custom Fields. Automatically Populated in RUI
#     'track_controls_dict': {
#         'pos_deg': 2.0,
#         'vel_deg': 20.0,
#     }
# }

# def pt_track_2(pt_connect_if, 
#                         track_data_dict, 
#                         track_settings_dict, 
#                         goto_position,
#                         track_pan_enabled, 
#                         track_tilt_enabled
#                         ):

#     #logger.log_info("******")
#     #logger.log_info("*** Stabs Solution Update Starting ***")
#     #logger.log_info("******")
#     start_time = nepi_utils.get_time()
#     ##########################
#     # Gather Stab Settings
#     ##########################
#     [pan_goal, tilt_goal] = goto_position

#     pan_deg = track_data_dict['pan_deg']
#     pan_dps = track_data_dict['pan_dps']
#     pan_speed_start_ratio = track_data_dict['pan_speed_start_ratio']
#     tilt_deg = track_data_dict['tilt_deg']
#     tilt_dps = track_data_dict['tilt_dps']
#     tilt_speed_start_ratio = track_data_dict['tilt_speed_start_ratio']
#     pan_tilt_max_speed_dps = track_data_dict['pan_tilt_max_speed_dps']

#     track_min_dps = track_settings_dict['track_pt_min_speed_ratio'] * pan_tilt_max_speed_dps 
#     track_max_dps = track_settings_dict['track_pt_max_speed_ratio'] * pan_tilt_max_speed_dps
#     controls_dict = track_settings_dict['track_controls_dict']

  
#     ######################
#     ## Process Stab Tilt Adjustments
#     ######################
#     pos_deg = controls_dict['pos_deg']
#     vel_deg = controls_dict['vel_deg']

#     last_data_dict = copy.deepcopy(track_data_dict)
#     last_tilt_dps = last_data_dict['track_tilt_dps']
#     last_tilt_goal = last_data_dict['track_tilt_goal']
#     last_tilt_goal_delta = (last_tilt_goal - tilt_deg)
#     last_tilt_dir = last_data_dict['track_tilt_dir']


#     tilt_adj = track_data_dict['track_tilt_adj']
#     adj_tilt_goal = round(tilt_goal + tilt_adj,1)
#     adj_tilt_goal_delta = (adj_tilt_goal - tilt_deg)
#     adj_tilt_dir = 1 if adj_tilt_goal_delta > 0 else -1
#     adj_tilt_change = (last_tilt_goal - adj_tilt_goal)

    
#     if track_tilt_enabled == True:
#         # Process Velocity Adjustment
#         if abs(adj_tilt_goal_delta) > vel_deg : # or abs(adj_tilt_goal_delta) < pos_deg:
#             track_speed_ratio = tilt_speed_start_ratio
#             track_tilt_dps = round(track_speed_ratio * pan_tilt_max_speed_dps,1)
#         else:
#             track_tilt_dps = round(track_min_dps + (track_max_dps - track_min_dps) * abs(adj_tilt_goal_delta) / vel_deg, 1)
#             track_speed_ratio = track_tilt_dps / pan_tilt_max_speed_dps

#         if abs(track_tilt_dps - last_tilt_dps) > 1:   
#             last_time = track_data_dict['last_tilt_vel_time']
#             update_rate = float(1) / (start_time - last_time)
#             track_data_dict['last_tilt_vel_time'] = start_time
#             track_data_dict['track_tilt_vel_rate'] = round(update_rate,2) 
#             track_data_dict['track_tilt_dps'] = track_tilt_dps     
#             pt_connect_if.set_tilt_speed_ratio(track_speed_ratio)

#         # Process Position Adjustment
#         pos_needs_update = False
#         if (last_tilt_dir != adj_tilt_dir):
#             pos_needs_update = True
#         else:
#             #logger.log_warn("Stab Tilt Pos Check: " + str([abs(adj_tilt_goal_delta), pos_deg, abs(adj_tilt_change), pos_deg]) ) 
#             #logger.log_warn("Stab Tilt Pos Check: " + str([abs(adj_tilt_goal_delta) > pos_deg, abs(adj_tilt_change) > pos_deg]) ) 
#             pos_needs_update = abs(adj_tilt_goal_delta) > pos_deg or abs(adj_tilt_change) > pos_deg
#             if adj_tilt_dir == 1:
#                 pos_needs_update = pos_needs_update and \
#                                     adj_tilt_goal > last_tilt_goal and \
#                                     abs(adj_tilt_goal - last_tilt_goal) > pos_deg 
#             else:
#                 pos_needs_update = pos_needs_update and \
#                                     adj_tilt_goal < last_tilt_goal and \
#                                     abs(adj_tilt_goal - last_tilt_goal) > pos_deg 
#         if pos_needs_update == True:
#             last_time = track_data_dict['last_tilt_pos_time']
#             update_rate = float(1) / (start_time - last_time)
#             track_data_dict['last_tilt_pos_time'] = start_time
#             track_data_dict['track_tilt_pos_rate'] = round(update_rate,2)
#             track_data_dict['track_tilt_goal'] = adj_tilt_goal
#             track_data_dict['track_tilt_dir'] = adj_tilt_dir
#             pt_connect_if.goto_to_tilt_position(adj_tilt_goal)
#             #logger.log_info("Stab Tilt Position updated: " + str([adj_tilt_goal_delta, track_speed_ratio, adj_tilt_goal]))

            
#     return track_data_dict, track_settings_dict


# PAN_TILT_STAB_PROCESSES_DICT['pt_track_2'] = {'process_function': pt_track_2, 
#                                              'default_settings_dict': pt_track_2_settings}



# #########################
# # Stab Utility Functions
# #########################

# def create_pan_tilt_processes_dict():
#     processes_dict = dict()
#     for process_name in PAN_TILT_STAB_PROCESSES_DICT.keys():
#         processes_dict[process_name] = PAN_TILT_STAB_PROCESSES_DICT[process_name]['default_settings_dict']
#     return processes_dict

# def update_pan_tilt_processes_dict(track_processes_dict):
#     clean_track_dict = create_pan_tilt_processes_dict()
#     for track_process in clean_track_dict.keys():
#         if track_process in track_processes_dict.keys():
#             for key in clean_track_dict[track_process].keys():
#                 if key in track_processes_dict[track_process].keys() and key != 'track_controls_dict':
#                     clean_track_dict[track_process][key] = track_processes_dict[track_process][key]
#             for key in clean_track_dict[track_process]['track_controls_dict'].keys():
#                 if key in track_processes_dict[track_process]['track_controls_dict'].keys():
#                     clean_track_dict[track_process]['track_controls_dict'][key] = track_processes_dict[track_process]['track_controls_dict'][key]
#     return clean_track_dict

# def get_blank_pan_tilt_data_dict():
#     return copy.deepcopy(PAN_TILT_STAB_DATA_DICT)






