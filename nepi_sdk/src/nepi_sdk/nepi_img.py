#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI image utility functions include
# 1) Image conversion functions
# 2) Image filter functions
# 3) Image manipulation functions
# 4) Image process functions
# 5) Image rendering functions
# 6) Image saving functions


import time
import sys
import rospy
import numpy as np
import ros_numpy
import cv2
import math
import copy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt8, Empty, String, Bool

from nepi_sdk import nepi_ros



###########################################
### Image conversion functions

def rosimg_to_cv2img(ros_img_msg, encoding = 'passthrough'):
  """ Convert image from ROS to OpenCV

  Args: 
      ros_img_msg (sensor.msg.Image): ROS Image message

  Returns: 
      cv2_img (cv2.mat): OpenCV Mat Image
  """
  bridge = CvBridge()
  cv2_img = bridge.imgmsg_to_cv2(ros_img_msg, desired_encoding = encoding)
  return cv2_img
    
    
def cv2img_to_rosimg(cv2_img, encoding="bgr8"): # "bgr8", "rgb8", or "mono8"
  """ Convert image from OpenCV to ROS

  Args: 
       cv2_img (cv2.mat): OpenCV Mat Image

  Returns: 
      ros_img_msg (sensor.msg.Image): ROS Image message
  """

  bridge = CvBridge()
  ros_img_msg = bridge.cv2_to_imgmsg(cv2_img, encoding = encoding)
  return ros_img_msg


def grayscale_to_rgb(gray_image):
    """
    Converts a grayscale image to an RGB image.

    Args:
        gray_image (numpy.ndarray): A grayscale image with shape (H, W) or (H, W, 1).

    Returns:
        numpy.ndarray: An RGB image with shape (H, W, 3).
    """

    if len(gray_image.shape) == 2:
        height, width = gray_image.shape
    elif len(gray_image.shape) == 3 and gray_image.shape[2] == 1:
        height, width, _ = gray_image.shape
    else:
        raise ValueError("Input image must be grayscale with shape (H, W) or (H, W, 1)")

    rgb_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2RGB)
    return rgb_image
    
###########################################
### Image filter functions    


    
###########################################
### Image manipulation functions


###########################################
### Image process functions
def is_gray(cv2_img):
    cv_shape = cv2_img.shape
    if len(cv_shape) == 2:
      return True
    elif len(cv_shape) == 3 and cv_shape[2] == 1:
      return True
    else:
      return False

def adjust_auto(cv2_img, sensitivity_ratio = 0.5):    
  # Apply Image Enhancment
  cv2_img.setflags(write=1)
  # Apply threshold filter
  cv2_img=adjust_sharpness(cv2_img, sensitivity_ratio = 0.2)
  #rospy.loginfo("input image shape and type")
  #rospy.loginfo(cv2_img.shape)
  if is_gray(cv2_img) is True:
    gray = cv2_img
  else:
    # Color Correction optimization
    Max=[0,0,0]
    for k in range(0, 3):
      Max[k] = np.max(cv2_img[:,:,k])
    Min_Max_channel  = np.min(Max)
    for k in range(0, 3):
      Max_channel  = np.max(cv2_img[:,:,k])
      Min_channel  = np.min(cv2_img[:,:,k])
      Mean_channel = np.mean(cv2_img[:,:,k])
      Chan_scale = (255 - Mean_channel) / (Max_channel - Min_channel)
      if Chan_scale < 1:
        Chan_scale = 1 - (1-Chan_scale)*(255-Min_Max_channel)/170
      elif Chan_scale > 1:
        Chan_scale = 1 + (Chan_scale-1)*(255-Min_Max_channel)/170
      if Chan_scale > 1*(1+sensitivity_ratio):
        Chan_scale = 1 *(1+sensitivity_ratio)
      if Chan_scale < -1*(1+sensitivity_ratio):
        Chan_scale = -1 *(1+sensitivity_ratio)
      Chan_offset = -1*Min_channel
      if Chan_offset < -10 * (1+9*sensitivity_ratio):
        Chan_offset = -10 * (1+9*sensitivity_ratio)
    cv2_img[:,:,k] = (cv2_img[:,:,k] + Chan_offset) * Chan_scale   
    # Contrast and Brightness optimization
    gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
  #rospy.loginfo("gray image shape and type")
  #rospy.loginfo(gray.shape)
  # Calculate grayscale histogram
  hist = cv2.calcHist([gray],[0],None,[256],[0,256])
  hist_size = len(hist)
  # Calculate cumulative distribution from the histogram
  accumulator = []
  accumulator.append(float(hist[0]))
  for index in range(1, hist_size):
    accumulator.append(accumulator[index -1] + float(hist[index]))
  # Locate points to clip
  maximum = accumulator[-1]
  clip_hist_percent = (maximum/100.0)
  clip_hist_percent /= 2.0
  # Locate left cut
  minimum_gray = 0
  while accumulator[minimum_gray] < clip_hist_percent:
    minimum_gray += 5
  # Locate right cut
  maximum_gray = hist_size -1
  while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
    maximum_gray -= 1
  # Calculate alpha and beta values
  alpha = 255 / (maximum_gray - minimum_gray) * (sensitivity_ratio*4)
  if alpha>2: ##
    alpha=2 ##
  beta = (-minimum_gray * alpha + 10) * (sensitivity_ratio*4)
  if beta<-50: ##
    beta=-50 ##
  cv2_img = cv2.convertScaleAbs(cv2_img, alpha=alpha, beta=beta)
  return cv2_img
    
def adjust_brightness(cv2_img, sensitivity_ratio = 0.5):
  if sensitivity_ratio != 0.5:
    max_value = 200
    brightness = int((sensitivity_ratio - 0.5) * 2 * max_value) # +- max_value
    if abs(brightness) > max_value:
      brightness = np.sign(brightness) * max_value
    if brightness != 0:
      if brightness > 0:
        shadow = brightness
        highlight = 255
      else:
        shadow = 0
        highlight = 255 + brightness
    alpha = (highlight - shadow)/255
    gamma = shadow
    cv2_img = cv2.addWeighted(cv2_img, alpha, cv2_img, 0, gamma)
  return cv2_img

def adjust_contrast(cv2_img, sensitivity_ratio = 0.5):
  if sensitivity_ratio != 0.5:
    max_value = 90
    contrast = int((sensitivity_ratio - 0.5) * 2 * max_value) # +- max_value
    if abs(contrast) > max_value:
      contrast = np.sign(contrast) * max_value
    f = 131*(contrast + 127)/(127*(131-contrast))
    alpha = f
    gamma = 127*(1-f)
    
    #cv2.convertScaleAbs(cv2_img, cv2_img, alpha_c, gamma_c)
    cv2_img = cv2.addWeighted(cv2_img, alpha, cv2_img, 0, gamma)
  return cv2_img

def adjust_sharpness(cv2_img, sensitivity_ratio = 0.0):
  if sensitivity_ratio != 0.0:
    # gaussian kernel for sharpening
    gaussian_blur = cv2.GaussianBlur(cv2_img,(7,7),sigmaX=2)
    # sharpening using addWeighted()
    sharpness_min = 2
    sharpness_max = 10
    #sharpness = int(sharpness_min + sensitivity_ratio * (sharpness_max-sharpness_min))
    sharpness = int(sharpness_min + sensitivity_ratio**2 * (sharpness_max-sharpness_min))
    cv2_img = cv2.addWeighted(cv2_img,sharpness,gaussian_blur,-(sharpness-1),0)
    #cv2_img * sensitivity_ratio
  return cv2_img

def adjust_resolution(cv2_img, res_mode):
  res_modes = [25,50,75,100]
  if res_mode != 3:
    res_scale = float(res_modes[res_mode]) / 100.0
    new_resolution = (int(cv2_img.shape[1] * res_scale),int(cv2_img.shape[0] * res_scale))
    cv2_img = cv2.resize(cv2_img,(new_resolution), 0, 0, interpolation = cv2.INTER_NEAREST)
  return cv2_img,cv2_img.shape

def adjust_framerate(current_fps,fr_mode):
  adj_fr = current_fps
  fr_modes = [25,50,75,100]
  if fr_mode != 3:
    frame_scale = float(fr_modes[fr_mode]) / 100.0
    adj_fr = frame_scale *  current_fps
  return adj_fr

def get_contours(cv2_img):
  """ Calculate image contours

  Args: 
       cv2_img (cv2.mat): OpenCV Mat Image

  Returns: 
      contrours3 : OpenCV contours list
      hierarchy3 : 
  """
  #cv2_img.setflags(write=1)
  cv2_mat_gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
  ret, thresh2 = cv2.threshold(cv2_mat_gray, 150, 255, cv2.THRESH_BINARY)
  contours3, hierarchy3 = cv2.findContours(thresh2, cv2.RETR_LIST, 
                                       cv2.CHAIN_APPROX_NONE)
  return contours3, hierarchy3

###########################################
### Image overlay functions

def overlay_contours(cv2_img,contours3, color_rgb = (0, 255, 0)):
  cv2_img_out = copy.deepcopy(cv2_img)
  cv2.drawContours(cv2_img_out, contours3, -1, color_rgb, 2, cv2.LINE_AA)
  return cv2_img_out
  
def overlay_text(cv2_img, text, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0), scale = 0.5,thickness = 1):
  # Add text overlay
  bottomLeftCornerOfText = (x_px,y_px)
  font                   = cv2.FONT_HERSHEY_SIMPLEX
  lineType               = 1
  cv2.putText(cv2_img,text, 
    bottomLeftCornerOfText, 
    font, 
    scale,
    color_rgb,
    thickness,
    lineType)
  return cv2_img
  
def overlay_text_list(cv2_img, text, x_px = 10 , y_px = 10, line_space_px = 20, color_rgb = (0, 255, 0), scale = 0.5,thickness = 1):
  # Add text overlay
  for text in text_list:
    cv2_overlay_text(cv2_img, text, x_px , y_px, color_rgb, scale, thickness)
    y_px = y_px + line_space_px
    
def optimal_font_dims(img, font_scale = 2e-3, thickness_scale = 1.5e-3):
    shape = img.shape
    h=shape[0]
    w=shape[1]
    font_scale = min(w, h) * font_scale
    thickness = math.ceil(min(w, h) * thickness_scale)
    return font_scale, thickness
    
def overlay_text_autoscale(cv2_img, text, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0)):
  # Add text overlay
  bottomLeftCornerOfText = (x_px,y_px)
  font                   = cv2.FONT_HERSHEY_SIMPLEX
  lineType               = 1
  fontScale, thickness  = optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
  cv2.putText(cv2_img,text, 
    bottomLeftCornerOfText, 
    font, 
    scale,
    color_rgb,
    thickness,
    lineType)
  return cv2_img
    
def overlay_box(cv2_img, color_rgb = (255,255,255), x_px = 10, y_px = 10, w_px = 20, h_px = 20):
      # Add status box overlay
      cv2_img_out = copy.deepcopy(cv2_img)
      cv2.rectangle(cv2_img_out, (x_px, y_px), (w_px, h_px), color_rgb, -1)
      
def create_message_image(message, image_size = (350, 700, 3),font_color = (0, 255, 0) ):
    # Create a blank img for when not running
    cv2_img = np.zeros(image_size, dtype = np.uint8) # Empty Black Image
    # Overlay text data on OpenCV image
    font = cv2.FONT_HERSHEY_DUPLEX
    fontScale              = 0.5
    fontColor              = font_color
    thickness              = 1
    lineType               = 1
    text2overlay=message
    bottomLeftCornerOfText = (50,50)
    cv2.putText(cv2_img,text2overlay, 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
    return cv2_img

    
###########################################
### Image saving functions


    


