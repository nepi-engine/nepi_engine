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
import cv2
import math
import copy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nepi_sdk import nepi_ros

from nepi_sdk.nepi_ros import logger as Logger
log_name = "nepi_img"
logger = Logger(log_name = log_name)

#########################
### Misc Functions

def create_cv2_blank_img(width,height, color = (255, 255, 255) ):
  # Create a black image (all zeros)
  img = np.zeros((height, width, 3), dtype=np.uint8)

  # Draw a white circle in the center
  center_coordinates = (width // 2, height // 2)
  radius = 100
    # White color in BGR
  thickness = -1  # Fill the circle
  cv2.circle(img, center_coordinates, radius, color, thickness)

  # Draw a blue rectangle
  start_point = (width // 4, height // 4)
  end_point = (3 * width // 4, 3 * height // 4)
  color = (255, 0, 0)  # Blue color in BGR
  thickness = 5
  img_out = copy.deepcopy(img)
  cv2.rectangle(img_out, start_point, end_point, color, -1)
  return img_out


def getImgShortName(det_namespace):
    base_namespace = nepi_ros.get_base_namespace()
    short_name = det_namespace.replace(base_namespace,"")
    if short_name.find("idx") != -1:
        short_name = short_name.replace("/idx","")
    return short_name


def get_img_depth_map_topic(img_topic):
  topic = img_topic.rsplit('/',1)[0] + "/depth_map"
  topic = nepi_ros.find_topic(topic)
  if topic == "":
    topic = None
  return topic
      

def get_img_pointcloud_topic(img_topic):
  topic = img_topic.rsplit('/',1)[0] + "/pointcloud"
  topic = nepi_ros.find_topic(topic)
  if topic == "":
    topic = None
  return topic

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


def resize_proportionally(image, max_width, max_height, interp = cv2.INTER_NEAREST):
    height, width = image.shape[:2]

    if max_width is None and max_height is None:
        return image
    
    if max_width is None:
        ratio = max_height / height
        max_width = int(width * ratio)
    elif max_height is None:
        ratio = max_width / width
        max_height = int(height * ratio)
    else:
        ratio = min(max_width / width, max_height / height)
        new_width = int(width * ratio)
        new_height = int(height * ratio)

    resized_image = cv2.resize(image, (new_width, new_height), interpolation=interp)
    return resized_image, ratio, new_width, new_height

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

def adjust_auto(cv2_img, sensitivity_ratio = 0.0):    
  # Apply Image Enhancment
  cv2_img.setflags(write=1)
  # Apply threshold filter
  cv2_img=adjust_sharpness(cv2_img, sensitivity_ratio = 0.2)
  #logger.log_info("input image shape and type")
  #logger.log_info(cv2_img.shape)
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
  #logger.log_info("gray image shape and type")
  #logger.log_info(gray.shape)
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
  if sensitivity_ratio < 0.49 or sensitivity_ratio > 0.51:
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
  if sensitivity_ratio < 0.49 or sensitivity_ratio > 0.51:
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
  if sensitivity_ratio > 0.05:
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

def adjust_resolution_ratio(cv2_img, ratio):
  if ratio < 0.1:
    ratio = 0.1
  if ratio > .99:
    ratio = 1
  if ratio != 1:
    new_resolution = (int(cv2_img.shape[1] * ratio),int(cv2_img.shape[0] * ratio))
    cv2_img = cv2.resize(cv2_img,(new_resolution), 0, 0, interpolation = cv2.INTER_NEAREST)
  return cv2_img,cv2_img.shape

def adjust_framerate_ratio(current_fps,ratio):
  if ratio < 0.1:
    ratio = 0.1
  if ratio > .99:
    ratio = 1
  if ratio != 1:
    adj_fr = ratio * current_fps
  else:
    adj_fr = current_fps
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
  
def overlay_text(cv2_img, text, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0), scale = None, thickness = None, background_rgb = None, apply_shadow = False):
    # Add text overlay
    if scale is None or thickness is None:
        scale, thickness  = optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontColor              = color_rgb
    lineType               = 1
    bottomLeftCornerOfText = (x_px,y_px)

    fontColorBlk = (0,0,0)

    # Add Background Box box if requested
    if background_rgb is not None:
      text_size = cv2.getTextSize(text, 
          font, 
          scale,
          thickness)    
      line_height = text_size[0][1]
      line_width = text_size[0][0]
      x_padding = int(line_height*0.4)
      y_padding = int(line_height*0.4)
      # Create Text Background Box

      bot_left_box =  (x_px - x_padding , y_px + y_padding)
      top_right_box = (x_px + line_width + x_padding, y_px - line_height - y_padding )

      try:
          cv2.rectangle(cv2_det_img, bot_left_box, top_right_box, background_rgb , -1)
      except Exception as e:
           logger.log_warn("Failed to add text background box: " + str(e), throttle_s = 5)

    # Add Text Shadow if requested
    if apply_shadow == True:
      try:
          cv2.putText(cv2_img,text, 
              bottomLeftCornerOfText, 
              font, 
              scale,
              fontColorBlk,
              thickness*2,
              lineType)
      except Exception as e:
           logger.log_warn("Failed to apply text shadow: " + str(e), throttle_s = 5)

  
    # Overlay Text
    try:
        cv2.putText(cv2_img,text, 
            bottomLeftCornerOfText, 
            font, 
            scale,
            fontColor,
            thickness,
            lineType)
    except Exception as e:
         logger.log_warn("Failed to apply overlay text: " + str(e), throttle_s = 5)
    return cv2_img
 
def overlay_text_list(cv2_img, text_list, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0), scale = None,thickness = None, background_rgb = None, apply_shadow = False):
    # Add text overlay
    font = cv2.FONT_HERSHEY_SIMPLEX
    lineType = 1
    for text in text_list:
      if scale is None or thickness is None:
        scale, thickness  = optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
      cv2_img = overlay_text(cv2_img, text, x_px , y_px, color_rgb, scale, thickness,background_rgb = background_rgb, apply_shadow = apply_shadow)
      text_size = cv2.getTextSize(text, 
          font, 
          scale,
          thickness)
      # logger.log_warn("Text Size: " + str(text_size))
      line_height = text_size[0][1]
      line_width = text_size[0][0]
      y_px = y_px + line_height * 1.5
    return cv2_img

    
def optimal_font_dims(cv2_img, font_scale = 2e-3, thickness_scale = 1.5e-3):
    shape = cv2_img.shape
    h=shape[0]
    w=shape[1]
    font_scale = min(w, h) * font_scale
    thickness = math.ceil(min(w, h) * thickness_scale)
    return font_scale, thickness
    

    
def overlay_box(cv2_img, color_rgb = (255,255,255), x_px = 10, y_px = 10, w_px = 20, h_px = 20):
    # Add status box overlay
    cv2_img_out = copy.deepcopy(cv2_img)
    cv2.rectangle(cv2_img_out, (x_px, y_px), (w_px, h_px), color_rgb, -1)

def overlay_bounding_box(cv2_img,bot_left_px, top_right_px, line_color=(255,0,0), line_thickness=2):
    try:
        cv2.rectangle(cv2_img, bot_left_px, top_right_px, line_color, thickness=line_thickness)
        success = True
    except Exception as e:
         logger.log_warn("Failed to create bounding box rectangle: " + str(e), throttle_s = 5)
    return cv2_img

def overlay_bounding_box_text_list(cv2_img, text_list, bot_left_px ,top_right_px, color_rgb = (0, 255, 0), scale = None,thickness = None, background_rgb = None, apply_shadow = False):
    font = cv2.FONT_HERSHEY_SIMPLEX
    lineType = 1
    if scale is None or thickness is None:
      scale, thickness  = optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
    text_size = cv2.getTextSize(text, 
      font, 
      scale,
      thickness)
    line_height = text_size[0][1]
    line_width = text_size[0][0]
    x_padding = int(line_height*0.4)
    y_padding = int(line_height*0.4)
    bot_left_text = (xmin + (line_thickness * 2) + x_padding , ymin + line_height + (line_thickness * 2) + y_padding)
    overlay_text_list(cv2_img, text_list, x_px = bot_left_text[0] , y_px = bot_left_text[1], color_rgb = color_rgb, scale = scale, thickness = thickness, background_rgb = background_rgb, apply_shadow = apply_shadow)
      
def create_blank_image(image_size = (350, 700, 3) ):
    # Create a blank img for when not running
    cv2_img = np.zeros(image_size, dtype = np.uint8) # Empty Black Image
    return cv2_img


def create_message_image(message, image_size = (350, 700, 3),color_rgb = (0, 255, 0) ):
    # Create a blank img for when not running
    cv2_img = create_blank_image(image_size) # Empty Black Image
    # Overlay text data on OpenCV image

    scale, thickness  = optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)

    font = cv2.FONT_HERSHEY_DUPLEX
    fontColor              = color_rgb
    lineType               = 1
    text2overlay=message
    bottomLeftCornerOfText = (50,50)
    cv2.putText(cv2_img,text2overlay, 
        bottomLeftCornerOfText, 
        font, 
        scale,
        fontColor,
        thickness,
        lineType)
    return cv2_img

    
###########################################
### Image saving functions


    


