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

BEST_FILTER_OPTIONS = ['SMALLEST','LARGEST','PROBABILITY']

BLANK_SETTINGS_DICT = {
    'targets_topic': 'None',
    'source_topic': 'None',
    'class_filters': [],
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
            

def get_best_from_targets(targets_dict_list,tracking_dict = BLANK_SETTINGS_DICT):
   filtered_targets = targets_dict_list
   best_target = None
   for entry in BLANK_SETTINGS_DICT.keys():
    if entry not in tracking_dict.keys():
       tracking_dict[entry] = BLANK_SETTINGS_DICT[entry]
    
    class_filters = tracking_dict['class_filters']
    filtered_targets = filter_by_classes(filtered_targets, class_filters)

    size_max_filter = tracking_dict['size_max_filter']
    size_min_filter = tracking_dict['size_min_filter']
    filtered_targets = filter_by_area(filtered_targets, size_min_filter = size_min_filter, size_max_filter = size_max_filter)

    threshold_filter = tracking_dict['threshold_filter']
    filtered_targets = filter_by_threshold(filtered_targets, threshold_filter)
    
    if len(filtered_targets) > 0:
      best_filter = tracking_dict['best_filter']
      best_target = find_best(filtered_targets, best_filter = best_filter)

            
    return best_target,tracking_dict    







