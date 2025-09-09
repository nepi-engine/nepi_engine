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

'''
from scipy.spatial import distance
from scipy.ndimage.filters import convolve
from scipy.sparse import diags, csr_matrix
from scipy.sparse.linalg import spsolve
'''


from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nepi_sdk import nepi_sdk

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_img"
logger = Logger(log_name = log_name)




STANDARD_IMAGE_SIZES = ['630 x 900','720 x 1080','955 x 600','1080 x 1440','1024 x 768 ','1980 x 2520','2048 x 1536','2580 x 2048','3648 x 2736']

def get_image_publisher_namespaces(name):
    msg_type = 'sensor_msgs/Image'
    return nepi_sdk.find_topics_by_msg(msg_type)


#########################
### Misc Functions

def create_cv2_blank_img(width=900, height=630, color = (255, 255, 255) ):
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
    base_namespace = nepi_sdk.get_base_namespace()
    short_name = det_namespace.replace(base_namespace,"")
    if short_name.find("idx") != -1:
        short_name = short_name.replace("/idx","")
    return short_name


def get_img_depth_map_topic(img_topic):
  topic = img_topic.rsplit('/',1)[0] + "/depth_map"
  topic = nepi_sdk.find_topic(topic)
  if topic == "":
    topic = None
  return topic
      

def get_img_pointcloud_topic(img_topic):
  topic = img_topic.rsplit('/',1)[0] + "/pointcloud"
  topic = nepi_sdk.find_topic(topic)
  if topic == "":
    topic = None
  return topic

def create_bgr_jet_color(x): # x -> 0-1
    x = np.clip(x, 0, 1)
    
    c1 = np.array([0.267004, 0.004874, 0.329415])
    c2 = np.array([0.232985, 0.298771, 0.538668])
    c3 = np.array([0.128768, 0.568284, 0.505529])
    c4 = np.array([0.529777, 0.780797, 0.307166])
    c5 = np.array([0.993248, 0.906157, 0.143936])

    if 0 <= x <= 0.25:
        c_list = c1 + (c2 - c1) * (x / 0.25)
    elif 0.25 < x <= 0.5:
        c_list = c2 + (c3 - c2) * ((x - 0.25) / 0.25)
    elif 0.5 < x <= 0.75:
        c_list = c3 + (c4 - c3) * ((x - 0.5) / 0.25)
    elif 0.75 < x <= 1:
        c_list = c4 + (c5 - c4) * ((x - 0.75) / 0.25)
    else:
        c_list = np.array([0.0, 0.0, 0.0])
    c_list = c_list * 255
    c_list = c_list.astype(int)
    return (c_list[0],c_list[1],c_list[2])

def create_bgr_jet_colormap_list(num_colors=256):
  colors_list = []
  for i in range(num_colors):
    color = create_bgr_jet_color(i/num_colors)
    colors_list.append(color)
  return colors_list






'''
def create_jet_colormap_list(num_colors=256):
    """
    Generates a jet colormap as a color tuple list.

    Args:
        num_colors: The number of colors in the colormap.

    Returns:
        A NumPy array of shape (num_colors, 3) representing the RGB colormap.
    """

    # Initialize the colormap array
    colormap_list = []
    color = [0,0,0]

    # Define the color segments
    positions = [0.0, 0.125, 0.375, 0.625, 0.875, 1.0]
    red_values = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
    green_values = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    blue_values = [0.5, 1.0, 1.0, 0.0, 0.0, 0.5]

    # Interpolate colors for each segment
    for i in range(num_colors):
        # Normalize the index to the range [0, 1]
        x = i / (num_colors - 1)

        # Find the segment where x belongs
        for j in range(len(positions) - 1):
            if positions[j] <= x <= positions[j + 1]:
                x0 = positions[j]
                x1 = positions[j + 1]
                y_red0 = red_values[j]
                y_red1 = red_values[j + 1]
                y_green0 = green_values[j]
                y_green1 = green_values[j + 1]
                y_blue0 = blue_values[j]
                y_blue1 = blue_values[j + 1]
                break

        # Linear interpolation
        red = y_red0 + (y_red1 - y_red0) * ((x - x0) / (x1 - x0))
        green = y_green0 + (y_green1 - y_green0) * ((x - x0) / (x1 - x0))
        blue = y_blue0 + (y_blue1 - y_blue0) * ((x - x0) / (x1 - x0))

        # Assign RGB values to the colormap array
        color[0] = int(red*255)
        color[1] = int(green*255)
        color[2] = int(blue*255)

        colormap = []
        for i2 in range(3):
            colormap.append(int(color[i2])) #*255))
        colormap_list.append(colormap)

    return colormap_list
  '''

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
  clip_hist_percent = (maximum/100)
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
 
def overlay_text_list(cv2_img, text_list, x_px = 10 , y_px = 10, color_rgb = (0, 255, 0),
    scale = None, thickness = None, size_ratio = 0.5, background_rgb = None, apply_shadow = False):
    # Add text overlay
    x_px = int(x_px)
    y_px = int(y_px)
    font = cv2.FONT_HERSHEY_SIMPLEX
    lineType = 1
    if scale is None or thickness is None:
        scale, thickness  = optimal_font_dims(cv2_img,font_scale = 2e-3, thickness_scale = 1.5e-3)
    scale = scale * (0.5 + size_ratio)
    for i in range(len(text_list)):
      text = text_list[i]
      text_size = cv2.getTextSize(text, 
          font, 
          scale,
          thickness)
      # logger.log_warn("Text Size: " + str(text_size))
      line_height = text_size[0][1]
      line_width = text_size[0][0]
      y_px = int(y_px + line_height * 1.5)
      #logger.log_warn("overlaying text: " + str(text) + ' at: ' + str([x_px,y_px]))
      cv2_img = overlay_text(cv2_img, text, x_px , y_px, color_rgb, scale, thickness, background_rgb = background_rgb, apply_shadow = apply_shadow)

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

def read_image_file(file_path, log_name_list = []):
    cv2_img = None
    if os.path.exists(file_path):
        try:
            cv2_img = cv2.imread(file_path)
            if cv2_img is not None:
                success = True
        except:
            logger.log_warn("Failed to get cv2_img from file: " + file_path + " " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
    else:
        logger.log_warn("Failed to find image file: " + file_path, log_name_list = log_name_list, throttle_s = 5.0)
    return cv2_img

def write_image_file(cv2_img,file_path, log_name_list = []):
    success = False
    path = os.path.dirname(file_path)
    if os.path.exists(path):
        try:
            success = cv2.imwrite(file_path, cv2_img)
        except:
            logger.log_warn("Failed to write image to file: " + file_path + " " + str(e), log_name_list = log_name_list, throttle_s = 5.0)
    else:
        logger.log_warn("Failed to find file path: " + path, log_name_list = log_name_list, throttle_s = 5.0)
    return success


    #########################
    # Image Analysis Functions

    '''
  def getShapeDistribution(cv2_img,x_px = None, y_px = None, w_px = None, h_px = None):
    https://www.geeksforgeeks.org/how-to-detect-shapes-in-images-in-python-using-opencv/
    # converting image into grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    i = 0

    # list for storing names of shapes
    for contour in contours:

        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue

        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.01 * cv2.arcLength(contour, True), True)
        
        # using drawContours() function
        cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

        # finding center point of shape
        M = cv2.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

        # putting shape name at center of each shape
        if len(approx) == 3:
            cv2.putText(img, 'Triangle', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        elif len(approx) == 4:
            cv2.putText(img, 'Quadrilateral', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        elif len(approx) == 5:
            cv2.putText(img, 'Pentagon', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        elif len(approx) == 6:
            cv2.putText(img, 'Hexagon', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        else:
            cv2.putText(img, 'circle', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
    '''

    '''
    def getColorDistribution(cv2_img,x_px = None, y_px = None, w_px = None, h_px = None):
      https://www.kaggle.com/code/caralosal/distribution-of-color-names-in-an-image
      # We specify each color name along with its values in the RGB channels.
      colors = ['black', 'white', 'red', 'green', 'blue', 'yellow', 'magenta', 'cyan']
      rgb_color = [[0,0,0], [255,255,255],[255,0,0], [0,255,0], [0,0,255],[255,255,0],[255,0,255],[0,255,255]]

      # CV2 reads the image as BGR instead of RGB, that's why we need to convert it after
      image = cv2.imread('/kaggle/input/color-example/image_example.jpg')

      # Convert from BGR to RGB
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

      # Matplotlib uses the RGB channel
      plt.imshow(image)
      plt.axis('off')
      plt.show()

    def distance(x,y):
        
        """Computes the distance between two points in a 3d space"""
        
        return np.sqrt(np.sum(np.square(np.array(x) - np.array(y))))

    def get_color_name_rgb(rgb, colors = colors, rgb_color = rgb_color):
        
        """Receives a pixel in RGB format and returns the name of the color of that pixel"""
        
        # Computes the distance between the pixel and each corner of the 3d cube
        each_distance = [distance(rgb, channel) for channel in rgb_color]
        
        # We get the index of the nearest color name to that pixel
        near_color = np.argmin(each_distance)
        
        # Return the color
        return colors[near_color]

    # Example of an specific pixel (Yellow is (255, 255,0) so (220, 220,0) is yellow)
    r = 220
    g = 220
    b = 0
    print(get_color_name_rgb((r, g, b)))



    def get_color_proportions_rgb(image):
        
        """Receieves an image in RGB format and returns the color proportions of the image"""
        
        red = image[:,:,0].ravel()
        green = image[:,:,1].ravel()
        blue = image[:,:,2].ravel()
        
        list_colors = [get_color_name_rgb((r, g, b)) for r, g, b in zip(red, green, blue)]
        
        # Show the image:
        plt.imshow(image)
        plt.axis('off')
        plt.show()

        return pd.DataFrame({'Percentage': round(pd.DataFrame(list_colors).value_counts() / len(list_colors) * 100,2)})

    get_color_proportions_rgb(image)


    
    '''
    
    
################################
# Low Light Enhancment Dual Illumination
# github.com/pvnieo/Low-light-Image-Enhancement

'''
def enhance_low_light_dual_illumination(cv2_img, gamma = 0.6, lambda_ = 0.15, sigma = 3,
                           bc = 1, bs = 1, be = 1):
    """Enhance input image, using either DUAL method, or LIME method. For more info, please see original papers.

    Arguments:
        cv2_img {np.ndarray} -- input image to be corrected.
        gamma {float} -- gamma correction factor.
        lambda_ {float} -- coefficient to balance the terms in the optimization problem (in DUAL and LIME).

    Keyword Arguments:
        sigma {int} -- Spatial standard deviation for spatial affinity based Gaussian weights. (default: {3})
        bc {float} -- parameter for controlling the influence of Mertens's contrast measure. (default: {1})
        bs {float} -- parameter for controlling the influence of Mertens's saturation measure. (default: {1})
        be {float} -- parameter for controlling the influence of Mertens's well exposedness measure. (default: {1})

    Returns:
        np.ndarray -- image exposure enhanced. same shape as `cv2_img`.
    """
    # create spacial affinity kernel
    kernel = create_spacial_affinity_kernel(sigma)

    # correct underexposudness
    eps = 1e-3
    im_normalized = cv2_img.astype(float) / 255.
    under_corrected = correct_underexposure(im_normalized, gamma, lambda_, kernel, eps)

    dual = True
    if dual:
        # correct overexposure and merge if DUAL method is selected
        inv_im_normalized = 1 - im_normalized
        over_corrected = 1 - correct_underexposure(inv_im_normalized, gamma, lambda_, kernel, eps)
        # fuse images
        im_corrected = fuse_multi_exposure_images(im_normalized, under_corrected, over_corrected, bc, bs, be)
    else:
        im_corrected = under_corrected

    # convert to 8 bits and returns
    return np.clip(im_corrected * 255, 0, 255).astype("uint8")

def create_spacial_affinity_kernel(spatial_sigma: float, size: int = 15):
    """Create a kernel (`size` * `size` matrix) that will be used to compute the he spatial affinity based Gaussian weights.

    Arguments:
        spatial_sigma {float} -- Spatial standard deviation.

    Keyword Arguments:
        size {int} -- size of the kernel. (default: {15})

    Returns:
        np.ndarray - `size` * `size` kernel
    """
    kernel = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            kernel[i, j] = np.exp(-0.5 * (distance.euclidean((i, j), (size // 2, size // 2)) ** 2) / (spatial_sigma ** 2))

    return kernel


def compute_smoothness_weights(L: np.ndarray, x: int, kernel: np.ndarray, eps: float = 1e-3):
    """Compute the smoothness weights used in refining the illumination map optimization problem.

    Arguments:
        L {np.ndarray} -- the initial illumination map to be refined.
        x {int} -- the direction of the weights. Can either be x=1 for horizontal or x=0 for vertical.
        kernel {np.ndarray} -- spatial affinity matrix

    Keyword Arguments:
        eps {float} -- small constant to avoid computation instability. (default: {1e-3})

    Returns:
        np.ndarray - smoothness weights according to direction x. same dimension as `L`.
    """
    Lp = cv2.Sobel(L, cv2.CV_64F, int(x == 1), int(x == 0), ksize=1)
    T = convolve(np.ones_like(L), kernel, mode='constant')
    T = T / (np.abs(convolve(Lp, kernel, mode='constant')) + eps)
    return T / (np.abs(Lp) + eps)


def fuse_multi_exposure_images(cv2_img: np.ndarray, under_ex: np.ndarray, over_ex: np.ndarray,
                               bc: float = 1, bs: float = 1, be: float = 1):
    """perform the exposure fusion method used in the DUAL paper.

    Arguments:
        cv2_img {np.ndarray} -- input image to be enhanced.
        under_ex {np.ndarray} -- under-exposure corrected image. same dimension as `cv2_img`.
        over_ex {np.ndarray} -- over-exposure corrected image. same dimension as `cv2_img`.

    Keyword Arguments:
        bc {float} -- parameter for controlling the influence of Mertens's contrast measure. (default: {1})
        bs {float} -- parameter for controlling the influence of Mertens's saturation measure. (default: {1})
        be {float} -- parameter for controlling the influence of Mertens's well exposedness measure. (default: {1})

    Returns:
        np.ndarray -- the fused image. same dimension as `cv2_img`.
    """
    merge_mertens = cv2.createMergeMertens(bc, bs, be)
    images = [np.clip(x * 255, 0, 255).astype("uint8") for x in [cv2_img, under_ex, over_ex]]
    fused_images = merge_mertens.process(images)
    return fused_images


def refine_illumination_map_linear(L: np.ndarray, gamma: float, lambda_: float, kernel: np.ndarray, eps: float = 1e-3):
    """Refine the illumination map based on the optimization problem described in the two papers.
       This function use the sped-up solver presented in the LIME paper.

    Arguments:
        L {np.ndarray} -- the illumination map to be refined.
        gamma {float} -- gamma correction factor.
        lambda_ {float} -- coefficient to balance the terms in the optimization problem.
        kernel {np.ndarray} -- spatial affinity matrix.

    Keyword Arguments:
        eps {float} -- small constant to avoid computation instability (default: {1e-3}).

    Returns:
        np.ndarray -- refined illumination map. same shape as `L`.
    """
    # compute smoothness weights
    wx = compute_smoothness_weights(L, x=1, kernel=kernel, eps=eps)
    wy = compute_smoothness_weights(L, x=0, kernel=kernel, eps=eps)

    n, m = L.shape
    L_1d = L.copy().flatten()

    # compute the five-point spatially inhomogeneous Laplacian matrix
    row, column, data = [], [], []
    for p in range(n * m):
        diag = 0
        for q, (k, l, x) in get_sparse_neighbor(p, n, m).items():
            weight = wx[k, l] if x else wy[k, l]
            row.append(p)
            column.append(q)
            data.append(-weight)
            diag += weight
        row.append(p)
        column.append(p)
        data.append(diag)
    F = csr_matrix((data, (row, column)), shape=(n * m, n * m))

    # solve the linear system
    Id = diags([np.ones(n * m)], [0])
    A = Id + lambda_ * F
    L_refined = spsolve(csr_matrix(A), L_1d, permc_spec=None, use_umfpack=True).reshape((n, m))

    # gamma correction
    L_refined = np.clip(L_refined, eps, 1) ** gamma

    return L_refined


def correct_underexposure(cv2_img: np.ndarray, gamma: float, lambda_: float, kernel: np.ndarray, eps: float = 1e-3):
    """correct underexposudness using the retinex based algorithm presented in DUAL and LIME paper.

    Arguments:
        cv2_img {np.ndarray} -- input image to be corrected.
        gamma {float} -- gamma correction factor.
        lambda_ {float} -- coefficient to balance the terms in the optimization problem.
        kernel {np.ndarray} -- spatial affinity matrix.

    Keyword Arguments:
        eps {float} -- small constant to avoid computation instability (default: {1e-3})

    Returns:
        np.ndarray -- image underexposudness corrected. same shape as `cv2_img`.
    """

    # first estimation of the illumination map
    L = np.max(cv2_img, axis=-1)
    # illumination refinement
    L_refined = refine_illumination_map_linear(L, gamma, lambda_, kernel, eps)

    # correct image underexposure
    L_refined_3d = np.repeat(L_refined[..., None], 3, axis=-1)
    im_corrected = cv2_img / L_refined_3d
    return im_corrected

# TODO: resize image if too large, optimization take too much time



def get_sparse_neighbor(p: int, n: int, m: int):
    """Returns a dictionnary, where the keys are index of 4-neighbor of `p` in the sparse matrix,
       and values are tuples (i, j, x), where `i`, `j` are index of neighbor in the normal matrix,
       and x is the direction of neighbor.

    Arguments:
        p {int} -- index in the sparse matrix.
        n {int} -- number of rows in the original matrix (non sparse).
        m {int} -- number of columns in the original matrix.

    Returns:
        dict -- dictionnary containing indices of 4-neighbors of `p`.
    """
    i, j = p // m, p % m
    d = {}
    if i - 1 >= 0:
        d[(i - 1) * m + j] = (i - 1, j, 0)
    if i + 1 < n:
        d[(i + 1) * m + j] = (i + 1, j, 0)
    if j - 1 >= 0:
        d[i * m + j - 1] = (i, j - 1, 1)
    if j + 1 < m:
        d[i * m + j + 1] = (i, j + 1, 1)
    return d   
    
    
'''

