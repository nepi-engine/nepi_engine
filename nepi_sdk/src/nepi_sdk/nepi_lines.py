#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
#os.environ['EGL_PLATFORM'] = 'surfaceless'   # Ubuntu 20.04+
import copy
import numpy as np
import cv2
import math
import pandas as pd
from scipy.stats import linregress
from scipy.signal import medfilt
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import CubicSpline

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img 


from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_lines"
logger = Logger(log_name = log_name)

BLANK_LINE_DICT = dict()
BLANK_LINE_DICT['x'] = []
BLANK_LINE_DICT['y'] = []

##########################
# Misc Util Functions

def get_blank_line_dict():
  return copy.deepcopy(BLANK_LINE_DICT)


def get_point_count(line_dict):
    num_points = len(line_dict['x'])
    return num_points


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


#########################
# Image process Functions
def process_image_none(cv2_img, sensitivity = 0.5 ):
    img_quality = 1.0
    return cv2_img, img_quality


def process_image_enhance(cv2_img, sensitivity = 0.5 ):

    cv2_img_processed = nepi_img.adjust_auto(cv2_img, sensitivity_ratio = sensitivity)
    img_quality = 1.0

    return cv2_img_processed, img_quality


#########################
# Image Filter Functions
def filter_image_none(cv2_img, sensitivity = 0.5 ):
    img_quality = 1.0
    return cv2_img, img_quality

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
    img_quality = 1.0
    return cv2_img_filtered, img_quality



#########################
# Line Point Functions

def process_line_brightest(cv2_img, line_color_bgr = (147, 175, 35), sensitivity = 0.5 , x_offset = 0, y_offset = 0):
    line_dict = dict()
    line_dict['x'] = []
    line_dict['y'] = []

    line_quality = 1.0

    # Apply color mask filter
        
    c_mask = nepi_img.create_color_mask(cv2_img, color_bgr = line_color_bgr, sensitivity = sensitivity,  hscalers = [2,2], sscalers = [2,1], vscalers = [2,1])

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
    

    return line_dict, line_quality



#########################
# Line Filter Functions

def filter_line_none(line_dict, line_color_bgr = (147, 175, 35), sensitivity = 0.5 ):
    line_quality = 1.0
    return line_dict, line_quality


def filter_line_IQR(line_dict, line_color_bgr = (147, 175, 35), sensitivity = 0.5 ):

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

    return filtered_line_dict, line_quality



#########################
# Merge Line Functions

def get_line_extent(line_dict,x_filter = None, y_filter = None):
    xmin = min(line_dict['x'])
    xmax = max(line_dict['x'])
    ymin = min(line_dict['y'])
    ymax = max(line_dict['y'])

    return [xmin,xmax,ymin,ymax]



def merge_lines_replace(base_line, merge_line, sensitivity):
    
    merge_quality = 1.0
    [xmin,xmax,ymin,ymax] = get_line_extent(merge_line)
    
    merged_line = dict()

    merged_line['x'] = base_line['x'] + merge_line['x'] #[x for x in base_line['x'] if x < xmin or x > xmax]
    #merged_line['x'].extends = merge_line[x]

    merged_line['y']  = base_line['y'] + merge_line['y'] # [y for y in base_line['y'] if y < ymin or y > ymax]
    #merged_line['y'].extends = merge_line[y]

    return merged_line, merge_quality

# def merge_lines_mean(base_line, merge_line, sensitivity):
#     merge_point_sensitivities = 

# def extract_laser_line_points(image_path, K, dist, laser_plane_equation=None, R_cam2world=None, t_cam2world=None):
#     imgimg = cv2.imread(image_path)


#     # Now, convert undistorted_pixels to 3D XYZ points
#     # This step depends on your setup: structured light with known laser plane or stereo vision
#     # Example for structured light with a known laser plane:
#     xyz_points = []
#     if laser_plane_equation is not None:
#         A, B, C, D = laser_plane_equation
#         for u, v in undistorted_pixels.reshape(-1, 2):
#             # Assuming camera coordinate system where Z is along the optical axis
#             # and u, v are normalized image coordinates
#             # This is a simplified example and might require more complex calculations
#             # based on your specific camera model and laser plane orientation.
#             # You'd typically solve for the intersection of the ray (from camera origin through (u,v,1))
#             # and the laser plane.
#             # For a basic approximation, you might assume Z=1 in camera coordinates
#             # and then transform to world coordinates if R_cam2world and t_cam2world are available.
#             # More accurate methods involve solving for intersection of ray and plane.
#             # For demonstration, let's assume a simple case where we need to find Z
#             # based on the laser plane equation and then transform to world coordinates.
#             # This part is highly specific to your calibration and setup.
#             # Placeholder for actual 3D reconstruction logic:
#             x_cam = u
#             y_cam = v
#             # ... calculate z_cam and then transform to world coordinates if needed
#             # For simplicity, let's assume we can get a Z value for each (u,v)
#             # based on the laser plane and camera pose.
#             # This is a complex geometric problem requiring detailed setup knowledge.
#             # For now, let's just use placeholder values for demonstration.
#             z_cam = 1.0 # Placeholder
#             if R_cam2world is not None and t_cam2world is not None:
#                 point_cam = np.array([x_cam, y_cam, z_cam])
#                 point_world = np.dot(R_cam2world, point_cam) + t_cam2world
#                 xyz_points.append(point_world)
#             else:
#                 xyz_points.append(np.array([x_cam, y_cam, z_cam]))

#     return np.array(xyz_points)

# # Example usage (placeholders for K, dist, etc.)
# # K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
# # dist = np.array([k1, k2, p1, p2, k3])
# # laser_plane = (A, B, C, D) # coefficients of the laser plane equation
# # R_cam2world = np.eye(3) # Rotation matrix from camera to world
# # t_cam2world = np.zeros(3) # Translation vector from camera to world

# # xyz_points = extract_laser_line_points("laser_image.jpg", K, dist, laser_plane, R_cam2world, t_cam2world)
# # logger.log_warn(xyz_points)



IMAGE_PROCESS_OPTIONS_DICT = {
    'None': process_image_none,
    'Enhance': process_image_enhance
}

IMAGE_FILTER_OPTIONS_DICT = {
    'None': filter_image_none,
    'Denoise': filter_image_denoise
}

LINE_PROCESS_OPTIONS_DICT = {
    'Brightest': process_line_brightest

}

LINE_FILTER_OPTIONS_DICT = {
    'None': filter_line_none
}

LINE_MERGE_OPTIONS_DICT = {
    'Replace': merge_lines_replace
}