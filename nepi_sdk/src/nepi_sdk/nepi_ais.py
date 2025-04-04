#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import numpy as np

from std_msgs.msg import UInt8, Float32, Bool, Empty, String, Header

from nepi_sdk.nepi_ros import logger as Logger
log_name = "nepi_ais"
logger = Logger(log_name = log_name)

########################
## Misc AI Helper Functions

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

EXAMPLE_AI_MGR_STATUS_DICT = {
    'ai_frameworks': [],

    'ai_models': [],
    'ai_models_frameworks': [],
    'ai_models_types': [],

    'active_ai_framework': 'framework_name',

    'active_ai_models': [],
    'active_ai_models_frameworks': [],
    'active_ai_models_types': [],
    'active_ai_models_nodes': [],
    'active_ai_models_namespaces': [],

    'all_namespace': '/base_namespace/ai/all'
}



EXAMPLE_BOXES_INFO_DICT_ENTRY = {
    'model_name': 'test_model',
    'image_header': Header(),
    'image_topic': '/test_topic',
    'src_height': 600,
    'src_width': 1000,
    'prc_height': 300,
    'prc_width': 500,
}


EXAMPLE_BOX_DICT_ENTRY = {
    'name': 'chair', # Class String Name
    'id': 1, # Class Index from Classes List
    'uid': '', # Reserved for unique tracking by downstream applications
    'prob': .3, # Probability of detection
    'xmin': 10,
    'ymin': 10,
    'xmax': 100,
    'ymax': 100,
    'area_ratio': 0.054,
    'area_pixels': 8100
}




def get_boxes_info_from_msg(bboxes_msg):
    boxes_info_dict  = {
        'model_name': bboxes_msg.model_name ,
        'image_header': bboxes_msg.image_header ,
        'image_topic': bboxes_msg.image_topic ,
        'src_height': bboxes_msg.src_height ,
        'src_width': bboxes_msg.src_width ,
        'prc_height': bboxes_msg.prc_height ,
        'prc_width': bboxes_msg.prc_width ,
    }
    return boxes_info_dict


def get_boxes_list_from_msg(bboxes_msg):
    bboxes_list = bboxes_msg.bounding_boxes
    boxes_list = []
    for bbox in bboxes_list:
        box_dict = {
            'name': bbox.Class ,
            'id': bbox.id ,
            'uid': bbox.uid ,
            'prob': bbox.probability ,
            'xmin': bbox.xmin ,
            'ymin': bbox.ymin ,
            'xmax': bbox.xmax ,
            'ymax': bbox.ymax ,
            'area_ratio': bbox.area_ratio ,
            'area_pixels': bbox.area_pixels
        }
        boxes_list.append(box_dict)
    return boxes_list



def scale_boxes_list(boxes_list, width_scaler, height_scaler):
    sboxes_list = []
    for box in boxes_list:
        box['xmin'] = box['xmin'] * width_scaler
        box['ymin'] = box['ymin'] * height_scaler
        box['xmax'] = box['xmax'] * width_scaler
        box['ymax'] = box['ymax'] * height_scaler
        box['area_pixels'] = box['area_pixels'] * width_scaler * height_scaler
        boxes_list.append(box)
    return sboxes_list


