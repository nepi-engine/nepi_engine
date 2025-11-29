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
from scipy.stats import linregress

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_img 


BLANK_LINE_DICT = dict()
BLANK_LINE_DICT['x'] = []
BLANK_LINE_DICT['y'] = []


LINE_PROCESS_OPTIONS_DICT = {
    Contours: get_line_contours
}

LINE_MERGE_OPTIONS_DICT = {
    Replace: merge_lines_replace
}



##########################
# Misc Util Functions

def get_blank_line_dict():
  return copy.deepcopy(BLANK_LINE_DICT)


def get_point_count(line_dict):
    num_points = len(line_dict['x'])

#########################
# Get Line Point Functions

def get_line_contours(cv2_img, line_color_bgr = (147, 175, 35), sensitivity = 0.5 ):
    line_dict = dict()
    line_dict['x'] = []
    line_dict['y'] = []

    line_quality = 1.0

    [lower_bound_bgr,upper_bound_bgr] = nepi_img.get_bgr_filter(line_color_bgr, sensitivity)

    # Convert to HSV and isolate laser color (example for red laser)
    hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
    lower_bound_bgr = np.array([0, 100, 100])
    upper_bound_bgr = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_bound_bgr, upper_bound_bgr)

    # Further processing to refine the line (e.g., morphological operations, line detection)
    # ... (e.g., using cv2.findContours and fitting a line or extracting the center line)

    # Example: Simple centroid extraction for illustration
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    line_pixels = []
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            line_pixels.append((cx, cy))

    # Undistort points
    undistorted_pixels = cv2.undistortPoints(np.array(line_pixels, dtype=np.float32).reshape(-1, 1, 2), K, dist)
    for point in undistorted_pixels:
        line_dict['x'].append(point[0])
        line_dict['y'].append(point[1])
    return line_dict, line_quality


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

    merged_line['x'] = [x for x in base_line if x < xmin or x > xmax]
    merged_line['x'].extends = merge_line[x]

    merged_line['y'] = [y for y in base_line if y < ymin or y > yma]
    merged_line['y'].extends = merge_line[y]

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
# # print(xyz_points)