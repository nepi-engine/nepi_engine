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


# Sample AI training data partitioning script. 
# Uses onboard python libraries to
# 1. Search all of the subfolders in the same folder as the script
# for image files
# 2) Tries to open the file. If opening the file fails, it deletes the file
# along with its bounding box .xml file if it exists

import os
from os.path import exists
from PIL import Image

import subprocess
import sys
sys.tracebacklimit = None
import glob
import fileinput
import random

IMAGE_FILE_TYPES = ['jpg','JPG','jpeg','png','PNG']

##########################################
# SETUP - Edit as Necessary 
##########################################
TEST_DATA_PERCENTAGE = 20

##########################################
# Methods
##########################################

def check_data_set(image_dir):
  path, dirs, files = next(os.walk(image_dir))
  data_size = len(files)
  ind = 0
  for f in os.listdir(image_dir):
    f_ext = os.path.splitext(f)[1]
    f_ext = f_ext.replace(".","")
    if f_ext in IMAGE_FILE_TYPES:
      #print('Found image file')
      image_file = (image_dir + '/' + f)
      #print(image_file)
      # Open and verify the image file
      try:
        img = Image.open(image_file) # open the image file
        img.verify()
        #print('Good file')
      except:
        print('')
        print('Found bad image file:')
        print(image_file) # print out the names of corrupt files
        print('Deleting file')
        os.remove(image_file)
        print('Looking for label file')
        label_file = (image_dir + '/' + f.split(f_ext)[0]+'xml')
        #print(label_file)
        if exists(label_file):
          print('Found xml label file for bad image:')
          print(label_file) # print out the names of corrupt files
          print('Deleting file')
          os.remove(label_file)
        else:
          print('No label file found for bad image')
            
def get_folder_list(script_folder_path):
  filelist=os.listdir(script_folder_path + '/')
  folder_list=[]
  #print('')
  #print('Files and Folders in Path:')
  #print(script_folder_path)
  #print(filelist)
  for file in enumerate(filelist):
    foldername = (script_folder_path + '/' + file[1])
    #print('Checking file: ')
    #print(foldername)
    if os.path.isdir(foldername): # file is a folder
       folder_list.append(foldername)
  return folder_list


###############################################
# Main
###############################################

if __name__ == '__main__':
  abs_path = os.path.realpath(__file__)
  script_path = os.path.dirname(abs_path)
  ### Check for corrupt image files in folders
  print(script_path)
  folders_to_process=get_folder_list(script_path)
  print('')
  print('Found folders in script directory:')
  print(folders_to_process)
  for folder in folders_to_process:
    print('Evaluting data in folder:')
    print(folder)
    check_data_set(folder)
  ### Wrap Up
  print('')
  print('All done')




