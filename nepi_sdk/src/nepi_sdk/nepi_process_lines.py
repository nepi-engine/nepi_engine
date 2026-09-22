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
import cv2
import numpy as np
from collections import defaultdict

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_process
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_data
from nepi_sdk import nepi_img

from sensor_msgs.msg import Image

from nepi_interfaces.msg import ImageStatus


from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_process_line"
logger = Logger(log_name = log_name)


########################
## REQUIRED Process IF Utilities
DEFAULT_PROCESS_NAME = 'line'
DEFAULT_PROCESS = 'line_brightness'


SOURCE_MSG = Image
SOURCE_STATUS_MSG = ImageStatus
SOURCE_STATUS_TYPE = 'nepi_interfaces/ImageStatus'
SOURCE_NAME_FILTERS = None

RESULTS_PUB_MSG = None
RESULTS_PUB_TYPE = None
RESULTS_PUB_DICT = None
RESULTS_PUB_TOPIC = None

IMAGE_PUB_TOPIC = 'lines_image'



########################
## Process Utility Functions

DEFAULT_COLOR_BGR = (147, 175, 35)
COLOR_AVG_LEN=20

BLANK_LINE_DICT = dict()
BLANK_LINE_DICT['x'] = []
BLANK_LINE_DICT['y'] = []


BLANK_RESULTS_DICT = dict()
BLANK_RESULTS_DICT['line_dict'] = copy.deepcopy(BLANK_LINE_DICT)
BLANK_RESULTS_DICT['quality'] = 0
BLANK_RESULTS_DICT['line_color_bgr'] = BLANK_LINE_DICT 

BASE_DATA_DICT = dict(
    detect_quality = 0,
    cv2_img = None,
    x_offset = 0,
    y_offset = 0,
    line_color_bgr = copy.deepcopy(DEFAULT_COLOR_BGR),
    line_colors_bgr = []
)


BASE_CONTROLS_DICT = dict(

    detect_threshold = {
        'type': 'FloatSlider', 'value': 0.3, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Detect Threshold',
        'description': 'Detect Threshold', 'display_hidden': False},


    denoise_level = {
        'type': 'FloatSlider', 'value': 0.2, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Denoise Level',
        'description': 'Denoise Image Filter Level', 'display_hidden': False},

    color_sensitivity = {
        'type': 'FloatSlider', 'value': 0.2, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Color Sensitivity',
        'description': 'Line Point Picking Color Sensitivity', 'display_hidden': False},

    filter_level = {
        'type': 'FloatSlider', 'value': 0.2, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Filter Level',
        'description': 'Line Points Filter Level', 'display_hidden': False},

    quality_threshold = {
        'type': 'FloatSlider', 'value': 0.3, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Quality Threshold',
        'description': 'Quality Threshold', 'display_hidden': False},

)






def get_blank_line_dict():
  return copy.deepcopy(BLANK_LINE_DICT)


def get_blank_results_dict():
  return copy.deepcopy(BLANK_RESULTS_DICT)

def get_point_count(line_dict):
    num_points = len(line_dict['x'])
    return num_points

def filter_image_denoise(cv2_img, sensitivity = 0.5 ):
    cv2_shape = cv2_img.shape
    img_width = cv2_shape[1] 
    img_height = cv2_shape[0] 
    kernel_float = int( (10 + min(img_width,img_height) * 0.01) * (sensitivity))
    kernel_size = nepi_utils.get_closest_odd_integer(kernel_float)
    if kernel_size < 1:
        kernel_size = 1
    #logger.log_warn("Applying Denoise Filter with K size of " + str(kernel_size))
    cv2_img_filtered = nepi_img.denoise_filter(cv2_img, filter_type='gaussian', kernel_size=kernel_size)
    return cv2_img_filtered



# def sanitize_points(points_dict: dict) -> dict:
#     """Converts 'x' and 'y' values in a dictionary to integers."""
#     return {k: int(v) for k, v in points_dict.items() if k in ("x", "y")}

# def average_by_index(data: list[dict[str, float]], grid_size: float = 1.0) -> list[dict[str, float]]:
#     """
#     Groups a list of x, y points by their grid index and returns the average 
#     x and y coordinates for each unique index.
    
#     :param data: List of dictionaries, e.g., [{'x': 1.2, 'y': 3.4}, ...]
#     :param grid_size: The size of the index grid slot (default is 1.0 for integer grouping)
#     :return: List of averaged x, y dictionaries
#     """
#     grid_size = 
#     groups = defaultdict(list)
    
#     # 1. Group points by their spatial grid index
#     for point in data:
#         x, y = point['x'], point['y']
#         # Using round() or floor (//) determines how you define the "index" boundary
#         grid_index = (round(x / grid_size), round(y / grid_size))
#         groups[grid_index].append((x, y))
        
#     # 2. Calculate the average for each index group
#     averaged_points = []
#     for coords in groups.values():
#         total_x = sum(pt[0] for pt in coords)
#         total_y = sum(pt[1] for pt in coords)
#         count = len(coords)
        
#         averaged_points.append({
#             'x': total_x / count,
#             'y': total_y / count
#         })
        
#     return sanitize_points(averaged_points)




def find_brightest_pixels_per_row(cv2_img):

    # Convert the image to grayscale for single-channel intensity analysis
    # This simplifies finding "brightness"
    if nepi_img.is_gray(cv2_img) == True:
        cv2_img_gray = cv2_img
    else:
        cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

    # Find the x-coordinate (column index) of the maximum intensity in each row
    # np.argmax with axis=1 returns the index of the max value in each row
    x_coords = np.argmax(cv2_img_gray, axis=1)

    # Get the total number of rows (height) of the image
    height = cv2_img_gray.shape[0]

    # Generate the corresponding y-coordinates (row indices)
    y_coords = np.arange(height)

    # Combine x and y coordinates into a list of (x, y) tuples
    # The format in OpenCV generally uses (x, y) coordinates for location, 
    # where x is the column and y is the row
    brightest_pixel_positions = list(zip(x_coords, y_coords))

    return brightest_pixel_positions

def find_brightest_pixels_per_column(cv2_img):

    # Convert the image to grayscale for single-channel intensity analysis
    # This simplifies finding "brightness"

    if nepi_img.is_gray(cv2_img) == True:
        cv2_img_gray = cv2_img
    else:
        cv2_img_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

    # Find the y-coordinate (row index) of the maximum intensity in each row
    # np.argmax with axis=1 returns the index of the max value in each row
    y_coords = np.argmax(cv2_img_gray, axis=0)

    # Get the total number of rows (height) of the image
    width = cv2_img_gray.shape[1]

    # Generate the corresponding x-coordinates (columns indices)
    x_coords = np.arange(width)


    # Combine x and y coordinates into a list of (x, y) tuples
    # The format in OpenCV generally uses (x, y) coordinates for location, 
    # where x is the column and y is the row
    brightest_pixel_positions = list(zip(x_coords, y_coords))

    return brightest_pixel_positions





def process_line_brightest(cv2_img, line_color_bgr = DEFAULT_COLOR_BGR, sensitivity = 0.5 , x_offset = 0, y_offset = 0):
    line_dict = dict()
    line_dict['x'] = []
    line_dict['y'] = []

        
    c_mask = nepi_img.create_color_mask(cv2_img, color_bgr = line_color_bgr, sensitivity = sensitivity,  hscalers = [2,2], sscalers = [1,1], vscalers = [2,1])

    mask_img = cv2.bitwise_and(cv2_img,cv2_img,mask = c_mask)

    # Process brightest for each row and column
  
    b_pixels = []
    b_pixels = b_pixels + find_brightest_pixels_per_row(mask_img)
    b_pixels = b_pixels + find_brightest_pixels_per_column(mask_img)

    if len(b_pixels) == 0:
        filtered_points = b_pixels
    else:
        # Filter out Edge pixels
        filtered_points = [(x, y) for x, y in b_pixels if x != 0 and y != mask_img.shape[1] and y != 0 and y != mask_img.shape[0] and not np.isnan(x) and not np.isnan(y)]
    
    if len(filtered_points) > 0:
        cols1, cols2 = zip(*filtered_points)
        line_dict['x'] = [item + x_offset for item in list(cols1)]
        line_dict['y'] = [item + y_offset for item in list(cols2)]
    
    #line_dict = average_by_index(line_dict)
    return line_dict



def update_color_from_lines(data_dict,results_dict_list,reset_color_bgr = None):
    if reset_color_bgr is not None:
        data_dict['line_color_bgr'] = reset_color_bgr
        data_dict['line_colors_bgr'] = []
        return data_dict
    if len(results_dict_list) == 0:
        return data_dict
    color_b_list = []
    color_g_list = []
    color_r_list = []
    for results_dict in results_dict_list:
        color_b_list.append(results_dict['line_color_bgr'][0])
        color_g_list.append(results_dict['line_color_bgr'][1])
        color_r_list.append(results_dict['line_color_bgr'][2])
    color_b = min(255,int(sum(color_b_list) / len(color_b_list)))
    color_g = min(255,int(sum(color_g_list) / len(color_g_list)))
    color_r = min(255,int(sum(color_r_list) / len(color_r_list)))
    color_bgr = (color_b,color_g,color_r)

    colors_bgr = data_dict['line_colors_bgr']
    colors_bgr.append(color_bgr)
    if len(colors_bgr) > COLOR_AVG_LEN:
        colors_bgr.pop(0)
    color_b_list = []
    color_g_list = []
    color_r_list = []
    for color in colors_bgr:
        color_b_list.append(color[0])
        color_g_list.append(color[1])
        color_r_list.append(color[2])
    color_b = min(255,int(sum(color_b_list) / len(color_b_list)))
    color_g = min(255,int(sum(color_g_list) / len(color_g_list)))
    color_r = min(255,int(sum(color_r_list) / len(color_r_list)))
    data_dict['line_color_bgr'] = (color_b,color_g,color_r)
    data_dict['line_colors_bgr'] = colors_bgr

    return data_dict



def get_line_bounds(line_dict):
        xmin = min(line_dict['x'])
        xmax = max(line_dict['x'])
        ymin = min(line_dict['y'])
        ymax = max(line_dict['y'])
        return [xmin,xmax,ymin,ymax]

def check_lines_overlap(bounds_1,bounds_2):
    """
    Checks if two bounding boxes overlap.
    Boxes are in the format [xmin, xmax, ymin, ymax]
    """
    # Unpack the coordinates for clarity
    xmin1, xmax1, ymin1, ymax1 = bounds_1
    xmin2, xmax2, ymin2, ymax2 = bounds_2

    # Check for overlap along both axes
    x_overlap = xmin1 <= xmax2 and xmax1 >= xmin2
    y_overlap = ymin1 <= ymax2 and ymax1 >= ymin2

    return x_overlap and y_overlap

def merge_lines(results_dict_list):
    line_dict = get_blank_line_dict

    ####################
    # Get Line Bounds
    lines_bounds_list = []
    for results_dict in results_dict_list:
        line_dict = results_dict['line_dict']
        lines_bounds_list.append(get_line_bounds(line_dict))

    ####################
    # Merge Overap Lines
    lines_overlap_list = []
    for i, bounds in enumerate(lines_bounds_list):
        new_line = results_dict_list[i]['line_dict']
        new_bounds = get_line_bounds(new_line)
        if len(lines_overlap_list) == 0:
            lines_overlap_list.append(new_line)
        else:
            line_overlaped = False
            for i2, overlap_list in enumerate(lines_overlap_list):    
                overlap_bounds = get_line_bounds(overlap_list[i2])
                lines_overlap = check_lines_overlap(new_bounds,overlap_bounds)
                if lines_overlap == True:
                    line_overlaped = True
                    overlap_list[i2]['x'].append(new_line['x'])
                    overlap_list[i2]['y'].append(new_line['y'])
                    break
            if line_overlaped == False:
                lines_overlap_list.append(new_line)

    ####################
    # Clean Overlap Lines



    ####################
    # Merge Lines
    for overlap_list in lines_overlap_list:   
        #overlap_list = average_by_index(overlap_list)     
        line_dict['x'].append(overlap_list['x'])
        line_dict['y'].append(overlap_list['y'])

    
    return line_dict


#########################
# Line Filter Functions

def filter_line_IQR(line_dict, line_color_bgr = DEFAULT_COLOR_BGR, sensitivity = 0.5 ):

    lower_q_value = 0.4 - (0.4 * (1 - sensitivity))
    upper_q_value = 0.6 + (0.4 * (1 - sensitivity))
    filtered_line_dict = {
        'x': [],
        'y': []
    }
    # Apply to y column
    df = pd.DataFrame(line_dict)
    column = 'y'
    Q1 = df[column].quantile(lower_q_value)
    Q3 = df[column].quantile(upper_q_value)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    dfx = df[(df[column] >= lower_bound) & (df[column] <= upper_bound)]
    #logger.log_warn("IQR FILTER X got line data size " + str(dfx.shape))
    # Apply to y column
    column = 'y'
    Q1 = dfx[column].quantile(lower_q_value)
    Q3 = dfx[column].quantile(upper_q_value)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    dfy = dfx[(dfx[column] >= lower_bound) & (dfx[column] <= upper_bound)]
    #logger.log_warn("IQR FILTER Y got line data size " + str(dfy.shape))
  
    line_dict = dfy.to_dict('list')

    line_quality = 1.0

    return filtered_line_dict

def get_line_quality(line_dict):
    quality = 1
    if 'x' not in line_dict.keys():
        quality = 0
    elif len(line_dict['x']) < 10:
        quality = 0
    return quality

########################
## Process Functions   
#######################
processes_dict = dict()
functions_dict = dict()



########################
## Process 1   



line_brightness_dict = {

   
    'data_dict': copy.deepcopy(BASE_DATA_DICT),


    'controls_dict': copy.deepcopy(BASE_CONTROLS_DICT),


    'results_display_dict': dict(
    ),

    'states_dict': dict(
    )
}


def line_brightness_process(data_dict, controls_dict, states_dict, results_dict):
    start_time = nepi_utils.get_time()
    last_data_dict = copy.deepcopy(data_dict)
    last_results_dict = copy.deepcopy(results_dict)
    controls_values_dict = nepi_controls.get_values_dict(controls_dict)
    #logger.log_warn("Got  Data: " + str(data_dict), throttle_s = 10)
    #logger.log_warn("Got  Data,Controls: " + str([data_dict,controls_values_dict]), throttle_s = 10)
    line_dict = get_blank_line_dict()
    cv2_img = data_dict['cv2_img']
    line_color_bgr = data_dict['line_color_bgr']
    line_color_list = data_dict['line_color_list']

    results_dict = get_blank_results_dict()
    results_dict['line_color_bgr'] = line_color_bgr

    if cv2_img is not None:

        detect_quality = data_dict['x_offset']
        detect_threshold = controls_values_dict['detect_threshold']

        if detect_quality > detect_threshold:

            x_offset = data_dict['x_offset']
            y_offset = data_dict['y_offset']
            
            
            denoise_level = controls_values_dict['denoise_level']
            cv2_img = filter_image_denoise(cv2_img, denoise_level )

            color_sensitivity = controls_values_dict['color_sensitivity']
            line_dict = points_dict = process_line_brightest(cv2_img, line_color_bgr , color_sensitivity , x_offset, y_offset)

            # filter_level = controls_values_dict['filter_level']
            # line_dict = filter_line_IQR(line_dict, line_color_bgr, filter_level)



            quality = get_line_quality(line_dict)
            quality_threshold = controls_values_dict['quality_threshold']

            if quality > quality_threshold:
                results_dict['line_dict'] = line_dict
                results_dict['quality'] = quality
                data_dict['cv2_img'] = cv2_img

    return data_dict, controls_dict, states_dict, results_dict


processes_dict = nepi_process.update_processes_dict(processes_dict, process_name = 'line_brightness', process_dict = line_brightness_dict)
#logger.log_warn("Updated processes dict: " + str(processes_dict))
functions_dict['line_brightness'] = line_brightness_process


########################
## Process 2  


########################
## Processes Init Dict  
PROCESSES_DICT = copy.deepcopy(processes_dict)
FUNCTIONS_DICT = functions_dict



# ########################
# ## Process Image Functions   
# #######################


# process_image_dict = {
    
#     'data_dict': dict(
#         last_image_time = 0,
#         image_status = nepi_sdk.convert_msg2dict(ImageStatus()),
#     ),


#     'controls_dict': dict(
#         options_dict = dict(),
#         overlay_color = (0,0,127),
#         overlay_colors = [],
#         overlay_font = nepi_img.OVERLAY_FONT,
#         overlay_font_color = nepi_img.OVERLAY_FONT_COLOR,
#         overlay_line_type = nepi_img.OVERLAY_LINE_TYPE,
#         overlay_line_color = nepi_img.OVERLAY_LINE_COLOR,
#         overlay_labels = True,
#         overlay_range_bearing = True,
#     ),

# }


# def process_results_image(cv2_img, data_dict, controls_dict, results_dict):
#         ##################
#         # Get Image Data
#         try:
#             cv2_img_results = copy.deepcopy(cv2_img)

#         except:
#             return cv2_img

#         last_image_time = copy.deepcopy(data_dict.get('last_image_time', 0))
#         data_dict.get('last_image_time') = nepi_utils.get_time()
#         width_deg = data_dict['image_status'].get('width_deg', 100)
#         height_deg = data_dict['image_status'].get('height_deg', 70)

#         ##################
#         # Get Image Controls
#         if controls_dict is None:
#             controls_dict = dict()
#         overlay_color = controls_dict.get('overlay_color',(0,0,127))
#         overlay_colors = controls_dict.get('overlay_colors',[])
#         overlay_font = controls_dict.get('overlay_color',nepi_img.OVERLAY_FONT)
#         overlay_font_color = controls_dict.get('overlay_color',nepi_img.OVERLAY_FONT_COLOR)
#         overlay_line_type = controls_dict.get('overlay_color',nepi_img.OVERLAY_LINE_TYPE)
#         overlay_line_color = controls_dict.get('overlay_color',nepi_img.OVERLAY_LINE_COLOR)
#         overlay_labels = controls_dict.get('overlay_labels',True)
#         overlay_range_bearing = controls_dict.get('overlay_range_bearing',True)


#         ##################
#         # Get Image Options
#         options_dict = controls_dict.get('options_dict',None)
#         if options_dict is None:
#             options_dict = dict()


#         ##################
#         # Get Results Data
#         if results_dict is None:
#             results_dict = dict()       
#         image_list = results_dict.get('image', [])

#         ##################
#         # Process Results Image

#         for i, target_dict in enumerate(image_list):
#             try:
#                 cv2_shape = cv2_img.shape
#                 img_width = cv2_shape[1] 
#                 img_height = cv2_shape[0] 


#                 ###### Apply Image Overlays and Publish Image ROS Message
#                 # Overlay adjusted detection boxes on image 
#                 class_name = target_dict['name']
#                 xmin = target_dict['xmin_pixel']
#                 ymin = target_dict['ymin_pixel']
#                 xmax = target_dict['xmax_pixel']
#                 ymax = target_dict['ymax_pixel']

#                 if xmin <= 0:
#                     xmin = 5
#                 if ymin <= 0:
#                     ymin = 5
#                 if xmax >= img_width:
#                     xmax = img_width - 5
#                 if ymax >= img_height:
#                     ymax = img_height - 5


#                 bot_left_px = (xmin, ymin)
#                 top_right_px = (xmax, ymax)


#                 class_color = overlay_color
            
#                 #logger.log_warn("Got Class Color: " + str(class_color) + ' type: ' + str(type(class_color)) + " type: " + str(type(class_color[0])) )
#                 line_thickness = max(1, math.ceil(max([img_height, img_width])/2000))
                

#                 success = False
#                 try:
#                     cv2_img_results = nepi_img.overlay_bounding_box(cv2_img_results,bot_left_px, top_right_px, line_color=class_color, line_thickness=line_thickness)
#                     success = True
#                 except Exception as e:
#                     logger.log_warn("Failed to create bounding box rectangle: " + str(e))

#                 # Overlay text data on OpenCV image
#                 if success == True:

#                     overlay_text = ""

#                     if overlay_labels:
#                         overlay_text = overlay_text + class_name + " "
                        
#                     if overlay_range_bearing:
#                         rb_text = ''
#                         if target_dict['range_m'] != -999 and target_dict['range_m'] != '':
#                             rb_text = rb_text + str(round(target_dict['range_m'],1)) + 'm :'
#                         if target_dict['azimuth_deg'] != -999 and target_dict['elevation_deg'] != -999:
#                             rb_text = rb_text + str(round(target_dict['azimuth_deg'],1)) + 'deg '
#                             rb_text = rb_text + str(round(target_dict['elevation_deg'],1)) + 'deg '
#                         if len(rb_text) > 0:
#                             overlay_text = overlay_text + rb_text


#                     if len(overlay_text) > 0:

#                         text_size = nepi_img.optimal_text_size
#                         #logger.log_warn("Text Size: " + str(text_size))
#                         line_height = text_size[0][1]
#                         line_width = text_size[0][0]
#                         x_padding = int(line_height*0.4)
#                         y_padding = int(line_height*0.4)
                        
#                         center = bot_left_box[0] + int(( top_right_box[0] - bot_left_box[0]) / 2 )
#                         #bot_left_text = (xmin + (line_thickness * 2) + x_padding , ymin + line_height + (line_thickness * 2) + y_padding)
#                         bot_left_text = (center + x_padding , ymin - (line_thickness * 2) - y_padding)
#                         # Create Text Background Box
#                         #bot_left_box =  (bot_left_text[0] - x_padding , bot_left_text[1] + y_padding)
#                         bot_left_box =  ( center - x_padding, bot_left_text[1] + y_padding)
#                         top_right_box = (center + line_width + x_padding, bot_left_text[1] - line_height - y_padding )

#                         cv2_img_results = overlay_text(cv2_img_results, overlay_text, x_px = 10 , y_px = 10, color_rgb = class_color, scale = None, thickness = None, background_rgb = None, apply_shadow = True)

#             except:
#                 pass

#         return cv2_img_results, data_dict, controls_dict
    