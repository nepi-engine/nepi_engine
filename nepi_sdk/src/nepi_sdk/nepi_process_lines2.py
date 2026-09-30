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
from scipy.optimize import linear_sum_assignment
from scipy.interpolate import splprep, splev

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
DEFAULT_PROCESS_NAME = 'lines'
DEFAULT_PROCESS = 'lines_1'


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
DEFAULT_COLOR_RGB = DEFAULT_COLOR_BGR[::-1]


BLANK_LINE_DICT = dict()
BLANK_LINE_DICT['x'] = []
BLANK_LINE_DICT['y'] = []


BLANK_RESULTS_DICT = dict()
BLANK_RESULTS_DICT['line_dict'] = copy.deepcopy(BLANK_LINE_DICT)
BLANK_RESULTS_DICT['quality'] = 0
BLANK_RESULTS_DICT['color_bgr'] = DEFAULT_COLOR_BGR 

BASE_DATA_DICT = dict(
    detect_quality = 0,
    cv2_img = None,
    x_offset = 0,
    y_offset = 0,
    color_bgr = copy.deepcopy(DEFAULT_COLOR_BGR),
)


BASE_CONTROLS_DICT = dict(


    denoise_level = {
        'type': 'FloatSlider', 'value': 0.2, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Denoise Level',
        'description': 'Denoise Image Filter Level', 'display_hidden': False},

    color_sensitivity = {
        'type': 'FloatSlider', 'value': 0.0, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Color Sensitivity',
        'description': 'Line Point Picking Color Sensitivity', 'display_hidden': False},

    dist_filter = {
        'type': 'FloatSlider', 'value': 1.0, 'bounds': [0.0, 1.0], 'round_value': 1,
        'display_name': 'Dist Filter Level',
        'description': 'Line Points Distance Filter Level', 'display_hidden': False},

    quality_threshold = {
        'type': 'FloatSlider', 'value': 0.3, 'bounds': [0.0, 1.0], 'round_value': 3,
        'display_name': 'Quality Threshold',
        'description': 'Quality Threshold', 'display_hidden': True},

)

BASE_DISPLAY_RESULTS_DICT = dict(

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



def sanitize_points(line_dict: dict) -> dict:
    """Converts 'x' and 'y' values in a dictionary to integers."""
    return {k: int(v) for k, v in line_dict.items() if k in ("x", "y")}

def remove_isolated_entries_fast(numbers, max_distance):
    """
    An optimized O(n log n) approach using sorting for large datasets.
    """
    if len(numbers) <= 1:
        return numbers

    # Pair each number with its original index to reconstruct the order later
    indexed_nums = sorted(enumerate(numbers), key=lambda x: x[1])
    keep_indices = set()
    n = len(indexed_nums)

    for i in range(n):
        orig_idx, val = indexed_nums[i]
        
        # Check the neighbor to the left
        if i > 0 and abs(val - indexed_nums[i - 1][1]) <= max_distance:
            keep_indices.add(orig_idx)
            continue
            
        # Check the neighbor to the right
        if i < n - 1 and abs(val - indexed_nums[i + 1][1]) <= max_distance:
            keep_indices.add(orig_idx)

    # Filter the original list based on the valid indices collected
    return [val for i, val in enumerate(numbers) if i in keep_indices]


def find_avg_pixels_per_row(line_dict, max_distance = 10):
    """
    Filters a list of x and y coordinates, returning only the coordinates 
    that represent the avg pixel for each unique y-axis row.
    """
    filtered_line_dict = get_blank_line_dict()
    [x_data, y_data] = [line_dict['x'],line_dict['y']]
    if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
        return line_dict
    
    if len(x_data) != len(y_data):
        raise ValueError("The x and y coordinate lists must be of equal length.")
        
    # Group x-coordinates by their y-coordinate index
    grouped_coords = defaultdict(list)
    for x, y in zip(x_data, y_data):
        grouped_coords[y].append(x)
        
    # Calculate the average x for each unique y, sorted by y axis index
    filtered_x = []
    filtered_y = []
    
    for y in sorted(grouped_coords.keys()):
        x_list = grouped_coords[y]
        if len(x_list) > 1:
            x_list = remove_isolated_entries_fast(x_list, max_distance)
        if len(x_list) > 0:
            avg_x = int(sum(x_list) / len(x_list))
            
            filtered_x.append(avg_x)   # Keeps float for sub-pixel accuracy
            filtered_y.append(y)

    filtered_line_dict['x'] = filtered_x
    filtered_line_dict['y'] = filtered_y
    return filtered_line_dict

def find_avg_pixels_per_column(line_dict, max_distance = 10):
    """
    Filters a list of x and y coordinates, returning only the coordinates 
    that represent the avg pixel for each unique x-axis row.
    """
    filtered_line_dict = get_blank_line_dict()
    [x_data, y_data] = [line_dict['x'],line_dict['y']]
    if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
        return line_dict
    
    if len(x_data) != len(y_data):
        raise ValueError("The x and y coordinate lists must be of equal length.")
        
    # Group y-coordinates by their x-coordinate index
    grouped_coords = defaultdict(list)
    for x, y in zip(x_data, y_data):
        grouped_coords[x].append(y)
        
    # Calculate the average y for each unique x, sorted by x axis index
    filtered_x = []
    filtered_y = []
    
    for x in sorted(grouped_coords.keys()):
        y_list = grouped_coords[x]
        if len(y_list) > 1:
            y_list = remove_isolated_entries_fast(y_list, max_distance)
        if len(y_list) > 0:
            avg_y = int(sum(y_list) / len(y_list))
            
            filtered_x.append(x)
            filtered_y.append(avg_y)   # Keeps float for sub-pixel accuracy, cast to int() if pixel alignment is required
        
    filtered_line_dict['x'] = filtered_x
    filtered_line_dict['y'] = filtered_y
    return filtered_line_dict




# def calculate_average_points(line_dict):


#     """
#     Fits a line minimizing perpendicular distances (Total Least Squares)
#     and returns the average (x, y) coordinates of the perpendicular projection points.
    
#     """
#     avg_line_dict = get_blank_line_dict()
#     [x_data, y_data] =[line_dict['x'],line_dict['y']]
#     if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
#         return line_dict
#     x = np.array(x_data, dtype=float)
#     y = np.array(y_data, dtype=float)
    
#     # 2. Fit a 1st-degree polynomial (line): y = mx + c
#     # Using numpy.polyfit to retrieve slope (m) and intercept (c)
#     m, c = np.polyfit(x, y, 1)
    
#     # 3. Define the direction vector of the line and its perpendicular
#     # Line vector v = (1, m). Perpendicular vector u = (-m, 1)
#     # We normalize 'u' so distances are scaled correctly
#     u = np.array([-m, 1.0])
#     u /= np.linalg.norm(u)
    
#     # 4. Project each point onto the fitted line
#     # The closest point on y = mx + c to (x_i, y_i) has a known geometric formula
#     x_proj = (x + m * y - m * c) / (m**2 + 1)
#     y_proj = m * x_proj + c
    
#     # 5. Map the projected points into integer bins to find "averages"
#     # We round the projection points to the nearest integer coordinates 
#     # to group adjacent points perpendicular to the line.
#     unique_bins = {}
#     for xp, yp, xi, yi in zip(x_proj, y_proj, x, y):
#         # Round the line anchor point to create a discrete bucket key
#         bin_key = (int(np.round(xp)), int(np.round(yp)))
        
#         if bin_key not in unique_bins:
#             unique_bins[bin_key] = []
#         unique_bins[bin_key].append((xi, yi))
        
#     # 6. Compute the average (x, y) integer point for each perpendicular slice
#     avg_perp_points = []
#     for bin_key, original_points in unique_bins.items():
#         pts_array = np.array(original_points)
#         # Average the original coordinates clustered in this slice
#         avg_line_dict['x'].append(int(np.round(np.mean(pts_array[:, 0]))))
#         avg_line_dict['y'].append(int(np.round(np.mean(pts_array[:, 1]))))

#     #logger.log_warn("Avg Line got avg line data size " + str([len(avg_line_dict['x']),len(avg_line_dict['y'])]))
    
#     return avg_line_dict



def find_brightest_pixels_per_row(cv2_img, line_dict):
    """
    Filters a list of x and y coordinates, returning only the coordinates 
    that represent the brightest pixel for each unique y-axis row.
    """
    filtered_line_dict = get_blank_line_dict()
    [x_data, y_data] = [line_dict['x'],line_dict['y']]
    if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
        return line_dict
    
    # Convert image to grayscale if it is in BGR format
    if len(cv2_img.shape) == 3:
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    else:
        gray = cv2_img

    # Group x coordinates by their corresponding y coordinates
    y_to_x_map = {}
    for x, y in zip(x_data, y_data):
        if y not in y_to_x_map:
            y_to_x_map[y] = []
        y_to_x_map[y].append(x)

    filtered_x = []
    filtered_y = []

    # Find the brightest x for each unique y
    for y, x_candidates in y_to_x_map.items():
        # Get pixel intensities for all x candidates in this row
        intensities = [gray[y, x] for x in x_candidates]
        
        # Find the index of the maximum intensity
        max_idx = np.argmax(intensities)
        
        # Append the brightest candidate to our filtered lists
        filtered_x.append(x_candidates[max_idx])
        filtered_y.append(y)


    filtered_line_dict['x'] = filtered_x
    filtered_line_dict['y'] = filtered_y
    return filtered_line_dict

def find_brightest_pixels_per_column(cv2_img, line_dict):
    """
    Filters a list of x and y coordinates, returning only the coordinates 
    that represent the brightest pixel for each unique x-axis row.
    """
    filtered_line_dict = get_blank_line_dict()
    [x_data, y_data] = [line_dict['x'],line_dict['y']]
    if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
        return line_dict
    
 # Convert image to grayscale if it is in BGR format
    if len(cv2_img.shape) == 3:
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    else:
        gray = cv2_img

    # Group y coordinates by their corresponding x coordinates
    x_to_y_map = {}
    for x, y in zip(x_data, y_data):
        if x not in x_to_y_map:
            x_to_y_map[x] = []
        x_to_y_map[x].append(y)

    filtered_x = []
    filtered_y = []

    # Find the brightest y for each unique x
    for x, y_candidates in x_to_y_map.items():
        # Get pixel intensities for all y candidates in this column
        # Note: OpenCV indexing is gray[y, x] (row, column)
        intensities = [gray[y, x] for y in y_candidates]
        
        # Find the index of the maximum intensity
        max_idx = np.argmax(intensities)
        
        # Append the brightest candidate to our filtered lists
        filtered_x.append(x)
        filtered_y.append(y_candidates[max_idx])

    filtered_line_dict['x'] = filtered_x
    filtered_line_dict['y'] = filtered_y
    return filtered_line_dict





def get_line_avg_color(cv2_img, line_dict, color_bgr = DEFAULT_COLOR_BGR):

    x_points = [item for item in list(line_dict['x'])]
    y_points = [item for item in list(line_dict['y'])]
    color_b_list = []
    color_g_list = []
    color_r_list = []
    for i, x in enumerate(x_points):
        color_bgr = None
        try:
            color_bgr = cv2_img[y_points[i],x_points[i]]
            color_b_list.append(color_bgr[0])
            color_g_list.append(color_bgr[1])
            color_r_list.append(color_bgr[2])
        except:
            pass

    if len(color_b_list) > 0:
        try:
            color_b = int(sum(color_b_list)/len(color_b_list))
            color_g = int(sum(color_g_list)/len(color_g_list))
            color_r = int(sum(color_r_list)/len(color_r_list))
            color_bgr = (color_b,color_g,color_r)
        except:
            pass
        
    return color_bgr

def get_color_from_colors(colors_bgr_list):
    color_bgr = None
    color_b_list = []
    color_g_list = []
    color_r_list = []
    if len(colors_bgr_list) > 0:
        for color_bgr in colors_bgr_list:
            if color_bgr is not None:
                color_b_list.append(color_bgr[0])
                color_g_list.append(color_bgr[1])
                color_r_list.append(color_bgr[2])
        if len(color_b_list) > 0:
            color_b = min(255,int(sum(color_b_list) / len(color_b_list)))
            color_g = min(255,int(sum(color_g_list) / len(color_g_list)))
            color_r = min(255,int(sum(color_r_list) / len(color_r_list)))
            color_bgr = (color_b,color_g,color_r)
    return color_bgr

def get_color_from_results(results_dict_list):
    color_bgr = None
    color_b_list = []
    color_g_list = []
    color_r_list = []
    if len(results_dict_list) > 0:
        for results_dict in results_dict_list:
            color_bgr = results_dict['color_bgr']
            if color_bgr is not None:
                color_b_list.append(color_bgr[0])
                color_g_list.append(color_bgr[1])
                color_r_list.append(color_bgr[2])
        if len(color_b_list) > 0:
            color_b = min(255,int(sum(color_b_list) / len(color_b_list)))
            color_g = min(255,int(sum(color_g_list) / len(color_g_list)))
            color_r = min(255,int(sum(color_r_list) / len(color_r_list)))
            color_bgr = (color_b,color_g,color_r)
    return color_bgr


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




#########################
# Line Filter Functions


def filter_line_color(cv2_img, color_bgr = DEFAULT_COLOR_BGR, sensitivity = 0.5):


    """
    Finds (x, y) coordinates matching a target BGR color within a sensitivity range.
    
    Parameters:
    - img: cv2 image (NumPy array in BGR format)
    - color_bgr: tuple/list of 3 integers representing (Blue, Green, Red)
    - sensitivity: float from 0 to 1 (0 = strict match, 1 = matches everything)
    
    Returns:
    - list of (x, y) coordinate tuples matching the filtered criteria.
    """

    line_dict = get_blank_line_dict()
    #logger.log_warn("Color Filter got image shape " + str(cv2_img.shape))

    # 1. Convert sensitivity fraction to a channel variation range (0 to 255)
    tolerance = int(sensitivity * 255)
    
    # 2. Extract BGR components from the target color
    target_b, target_g, target_r = color_bgr
    
    # 3. Calculate lower and upper bounds, safely clamping them between 0 and 255
    lower_bound = np.array([
        max(0, target_b - int(sensitivity * color_bgr[0])),
        max(0, target_g - int(sensitivity * color_bgr[1])),
        max(0, target_r - int(sensitivity * color_bgr[2]))
    ], dtype=np.uint8)
    
    upper_bound = np.array([
        min(255, target_b + int(sensitivity * color_bgr[0])),
        min(255, target_g + int(sensitivity * color_bgr[1])),
        min(255, target_r + int(sensitivity * color_bgr[2]))
    ], dtype=np.uint8)
    
    # 4. Create a binary mask where matching pixels are 255 (white) and others are 0 (black)
    mask = cv2.inRange(cv2_img, lower_bound, upper_bound)
    # c_mask = nepi_img.create_color_mask(cv2_img, color_bgr = color_bgr, sensitivity = sensitivity, hscalers = [2,2], sscalers = [1,1], vscalers = [2,1])

    
    # 5. Extract indices where the mask is active. 
    # np.where returns (row_indices, col_indices), which correspond to (y, x)
    y_indices, x_indices = np.where(mask > 0)
    #y_indices = [(x, y) for x, y in mask_img if x != 0 and y != cv2_img.shape[1] and y != 0 and y != cv2_img.shape[0] and not np.isnan(x) and not np.isnan(y)]
    
    # 6. Pair them up into a list of (x, y) tuples
    line_dict['x'] = x_indices
    line_dict['y'] = y_indices


    ############################
    c_mask = nepi_img.create_color_mask(cv2_img, color_bgr = color_bgr, sensitivity = sensitivity,  hscalers = [1,1], sscalers = [1,1], vscalers = [1,1])
    mask_img = cv2.bitwise_and(cv2_img,cv2_img,mask = c_mask)
    gray_output = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)

    # 3. Get all pixel coordinates where value > 0
    # This returns an array of coordinates in [[y1, x1], [y2, x2], ...] format
    pixel_points = np.argwhere(gray_output > 0)
    
    filtered_points = [(y, x) for y, x in pixel_points if x != 0 and y != mask_img.shape[0] and y != 0 and y != mask_img.shape[1] and not np.isnan(x) and not np.isnan(y)]
    #logger.log_warn("Color Filter got points len " + str(len(filtered_points)))
    #logger.log_warn("Color Filter got points " + str(filtered_points))
    if len(filtered_points) > 0:
        y_points, x_points  = zip(*filtered_points)
        line_dict['x'] = line_dict['x'] + list(x_points)
        line_dict['y'] = line_dict['y'] + list (y_points)
    #logger.log_warn("Color Filter got line data size " + str([len(line_dict['x']),len(line_dict['y'])]))



    return mask_img, line_dict

def filter_isolated_points(line_dict, max_distance = 5):
    """
    Removes points that do not have at least one other point within max_distance.
    """
    # Pair the coordinates into a list of tuples
    filtered_line_dict = get_blank_line_dict()
    [x_data, y_data] = [line_dict['x'],line_dict['y']]
    if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data) or max_distance < 1:
        return line_dict
    points = list(zip(x_data, y_data))
    n = len(points)
    
    keep_x = []
    keep_y = []
    
    # Check each point against all other points
    for i, (x1, y1) in enumerate(points):
        has_neighbor = False
        for j, (x2, y2) in enumerate(points):
            if i == j:
                continue  # Skip comparing the point to itself
                
            # Calculate Euclidean distance
            if math.hypot(x1 - x2, y1 - y2) <= max_distance:
                has_neighbor = True
                break  # Stop searching early if a neighbor is found
                
        if has_neighbor:
            filtered_line_dict['x'].append(x1)
            filtered_line_dict['y'].append(y1)
    #logger.log_warn("Isolated Filter got line data size " + str([len(filtered_line_dict['x']),len(filtered_line_dict['y'])]))
    return filtered_line_dict




def filter_line_brightest(cv2_img, line_dict):
    filtered_line_dict = get_blank_line_dict()
    [x_data, y_data] = [line_dict['x'],line_dict['y']]
    if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
        return line_dict

    line_dict_x = find_brightest_pixels_per_row(cv2_img,line_dict)
    filtered_line_dict['x'] = filtered_line_dict['x'] + line_dict_x['x']
    filtered_line_dict['y'] = filtered_line_dict['y'] + line_dict_x['y']
    line_dict_y = find_brightest_pixels_per_column(cv2_img,line_dict)
    filtered_line_dict['x'] = filtered_line_dict['x'] + line_dict_y['x']
    filtered_line_dict['y'] = filtered_line_dict['y'] + line_dict_y['y']
    #logger.log_warn("Brightness Filter got line data size " + str([len(filtered_line_dict['x']),len(filtered_line_dict['y'])]))
    return filtered_line_dict


def filter_line_avg(line_dict, max_distance = 5):
    filtered_line_dict = get_blank_line_dict()
    [x_data, y_data] = [line_dict['x'],line_dict['y']]
    if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
        return line_dict

    line_dict_x = find_avg_pixels_per_row(line_dict, max_distance)
    filtered_line_dict['x'] = filtered_line_dict['x'] + line_dict_x['x']
    filtered_line_dict['y'] = filtered_line_dict['y'] + line_dict_x['y']
    line_dict_y = find_avg_pixels_per_column(line_dict, max_distance)
    filtered_line_dict['x'] = filtered_line_dict['x'] + line_dict_y['x']
    filtered_line_dict['y'] = filtered_line_dict['y'] + line_dict_y['y']
    #logger.log_warn("Avg Filter got line data size " + str([len(filtered_line_dict['x']),len(filtered_line_dict['y'])]))
    return filtered_line_dict

def get_line_quality(line_dict):
    quality = 1
    if 'x' not in line_dict.keys():
        quality = 0
    elif len(line_dict['x']) < 10:
        quality = 0
    return quality





def remove_points_in_window(line_dict, bounds):
    """
    Removes points (x, y) that fall inside the specified window boundaries (inclusive).
    """
    # Filter out points where both x is in [x_min, x_max] AND y is in [y_min, y_max]
    filtered_line_dict = get_blank_line_dict()
    [x_data, y_data] = [line_dict['x'],line_dict['y']]
    if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
        return line_dict

    [x_min, x_max, y_min, y_max] = bounds
    filtered_points = [
        (x, y) for x, y in zip(x_data, y_data)
        if not (x_min <= x <= x_max and y_min <= y <= y_max)
    ]
    
    # If all points were removed, return two empty lists
    if not filtered_points:
        return [], []
        
    # Unzip the filtered pairs back into two separate lists
    new_x, new_y = map(list, zip(*filtered_points))
    filtered_line_dict['x'] = new_x
    filtered_line_dict['y'] = new_y
    #logger.log_warn("Line remove points data size " + str([len(filtered_line_dict['x']),len(filtered_line_dict['y'])]))
    return filtered_line_dict


def merge_results(cv2_img, results_dict_list):
    line_dict = get_blank_line_dict
    all_results = get_blank_results_dict()
    #logger.log_warn("Merge Line got results list len " + str(len(results_dict_list)))
    ####################)
    # Get Line Bounds
    lines_dict_list = []
    lines_bounds_list = []

    for i, results_dict in enumerate(results_dict_list):
        line_dict = results_dict['line_dict']        
        [x_data, y_data] = [line_dict['x'],line_dict['y']]
        if len(x_data) == 0 or len(y_data) == 0 or len(x_data) != len(y_data):
            pass
        else:
            lines_dict_list.append(line_dict)
            lines_bounds_list.append(get_line_bounds(line_dict))
            #logger.log_warn("Merge Line updated line bounds list len " + str(lines_bounds_list))

    #logger.log_warn("Merge Line updated results list len " + str([len(lines_dict_list),len(lines_bounds_list)]))
    #logger.log_warn("Merge Line got bounds list len " + str(lines_bounds_list))
    ####################
    # Merge Overap Lines
    lines_overlap_list = []
    for i, new_line in enumerate(lines_dict_list):
            #logger.log_warn("Processing Line overlap check with line len " + str([len(new_line['x']),len(new_line['y'])]))
            new_bounds = lines_bounds_list[i]
            if len(lines_overlap_list) == 0:
                lines_overlap_list.append(new_line)
            else:
                lines_overlap = False
                for i2, overlap_list in enumerate(lines_overlap_list):    
                    overlap_bounds = get_line_bounds(overlap_list)
                    #logger.log_warn("Merge Line overlap check " + str([new_bounds,overlap_bounds]))
                    lines_overlap = check_lines_overlap(new_bounds,overlap_bounds)
                    if lines_overlap == True:
                        #logger.log_warn("Merge Line Found Overlap " + str([new_bounds,overlap_bounds]))
                        #lines_overlap_list[i2] = remove_points_in_window(lines_overlap_list[i2], overlap_bounds)
                        lines_overlap_list[i2]['x'] = lines_overlap_list[i2]['x'] + new_line['x']
                        lines_overlap_list[i2]['y'] = lines_overlap_list[i2]['y'] + new_line['y']
                        break
                if lines_overlap == False:
                    lines_overlap_list.append(new_line)
            #logger.log_warn("Merge got Overlap List len " + str([len(lines_overlap_list)]))
              

    ####################
    # Merge Lines
    #logger.log_warn("Merge Line has n lines " + str(len(lines_overlap_list)))
    for overlap_list in lines_overlap_list:   
        overlap_list = filter_line_avg(overlap_list) 
        #overlap_list = filter_line_brightest(cv2_img, overlap_list)
        line_dict['x'] = line_dict['x'] + overlap_list['x']
        line_dict['y'] = line_dict['y'] + overlap_list['y']

    #logger.log_warn("Merge Line got line data size " + str([len(line_dict['x']),len(line_dict['y'])]))
    all_results['line_dict'] = line_dict
    return [all_results]


########################
## Process Functions   
#######################
processes_dict = dict()
functions_dict = dict()



########################
## Process 1   



lines_1_dict = {

   
    'data_dict': copy.deepcopy(BASE_DATA_DICT),


    'controls_dict': copy.deepcopy(BASE_CONTROLS_DICT),


    'results_display_dict': copy.deepcopy(BASE_DISPLAY_RESULTS_DICT),

    'states_dict': dict(
    )
}


def lines_1_process(data_dict, controls_dict, states_dict, results_dict):
    start_time = nepi_utils.get_time()
    last_data_dict = copy.deepcopy(data_dict)
    last_results_dict = copy.deepcopy(results_dict)
    controls_values_dict = nepi_controls.get_values_dict(controls_dict)
    #logger.log_warn("Got  Data: " + str(data_dict), throttle_s = 10)
    #logger.log_warn("Got  Data,Controls: " + str([data_dict,controls_values_dict]), throttle_s = 10)

    cv2_img = data_dict['cv2_img']
    color_bgr = data_dict['color_bgr']
    dist_filter = controls_values_dict['dist_filter']
    results_dict = get_blank_results_dict()
    results_dict['color_bgr'] = color_bgr

    if cv2_img is not None:

        line_dict = get_blank_line_dict()

        x_offset = data_dict['x_offset']
        y_offset = data_dict['y_offset']
        
        


        color_sensitivity = controls_values_dict['color_sensitivity']
        [cv2_img, line_dict] = filter_line_color(cv2_img, color_bgr , color_sensitivity)

        denoise_level = controls_values_dict['denoise_level']
        cv2_img = filter_image_denoise(cv2_img, denoise_level )

        line_dict = filter_line_brightest(cv2_img, line_dict)


        max_dist = 1 + dist_filter * 9
        line_dict = filter_isolated_points(line_dict, max_distance = max_dist)

        max_dist = 1 + (1 - dist_filter) * 9
        line_dict = filter_line_avg(line_dict, max_distance = max_dist)








        process_line_dict = get_blank_line_dict()
        process_line_dict['x'] = [item + x_offset for item in line_dict['x']]
        process_line_dict['y'] = [item + y_offset for item in line_dict['y']]

        quality = get_line_quality(line_dict)
        quality_threshold = controls_values_dict['quality_threshold']

        line_color = get_line_avg_color(cv2_img, line_dict, color_bgr)
        if line_color is None:
            line_color = color_bgr
        if quality > quality_threshold:
            results_dict['line_dict'] = process_line_dict
            results_dict['quality'] = quality
            results_dict['color_bgr'] = line_color
            data_dict['cv2_img'] = cv2_img

    return data_dict, controls_dict, states_dict, results_dict


processes_dict = nepi_process.update_processes_dict(processes_dict, process_name = 'lines_1', process_dict = lines_1_dict)
#logger.log_warn("Updated processes dict: " + str(processes_dict))
functions_dict['lines_1'] = lines_1_process


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
    