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
from os.path import exists
import subprocess
import sys
import glob
import fileinput
import random

import ai_utils

##########################################
# SETUP - Edit as Necessary 
##########################################
data_folder_path = os.path.join(ai_utils.current_folder,'data_labeling')
train_folder_path = os.path.join(ai_utils.current_folder,'model_training')
train_file_path = os.path.join(train_folder_path,'data_custom.yaml')

unlabeled_file_path = os.path.join(data_folder_path, 'unlabeled_data_list.txt')

##########################################
# Methods
##########################################

def process_folder(folder_path):
  train_data_files = []
  train_label_files = []
  test_data_file = []
  test_label_files = []
  ulab_files = []

  ### Walk through folder folders
  print("Processing folders in: " + folder_path)
  folders_to_process=ai_utils.get_folder_list(folder_path)
  print('')
  print('Found folders: ' + str(folders_to_process))
  for folder in folders_to_process:
    print('Processing folder: ' + folder)
    path, dirs, files = next(os.walk(folder))
    data_size = len(files)
    ind = 0
    data_test_size = int(float(1)/float(ai_utils.TEST_DATA_PERCENTAGE) * data_size)
    test_array = random.sample(range(data_size), k=data_test_size)
    files = os.listdir(folder)
    print("Found " + str(len(files)) + " files in folder")
    #print('Found image files: ' + str(files))
    for f in files:
      f_ext = os.path.splitext(f)[1]
      f_ext = f_ext.replace(".","")
      try:
        if f_ext in ai_utils.IMAGE_FILE_TYPES:
          image_file = (folder_path + '/' + f)
          #print('Found image file: ' + image_file)
          #print(image_file)
          label_file = (folder_path + '/' + f.split(f_ext)[0]+'txt')
          #print('Looking for label file: ' + label_file)
          if os.path.exists(label_file):
            #print('Found label file')
            ind += 1
            if ind in test_array: 
              #print('Adding image to test file list')
              test_data_file.append(image_file)
              test_label_files.append(label_file)
            if ind not in test_array and ai_utils.MAKE_TRAIN_TEST_UNIQUE == True: 
              #print('Adding image to train file list')
              train_data_files.append(image_file)
              train_label_files.append(label_file)
          else:
            print("Warning: No label file for image: " + image_file)
            ulab_files.append(image_file)
      except Exception as e:
        print("Excepton on file write: " + str(e))

  print("Found " + str(len(ulab_files)) + " unlabeled files")
  ### Create return data
  data = ulab_files
  return data
            

def write_data_to_file(file_path, data):

    ### Create save Data String 
    data_str = ''
    for file in data:
      data_str = data_str + file + '\n'

    ### Write Data String file
    try:
      file = ai_utils.open_new_file(file_path)
    except Exception as e:
      print('Failed to create file: ' + file_path + " " + str(e))
      return False
  
    try:
      file.write(data_str)
      return True
    except Exception as e:
      print("Excepton on file write: " + str(e))
      return False

###############################################
# Main
###############################################

if __name__ == '__main__':


  ### Process Data
  data = process_folder(data_folder_path)
  ### Write File
  success = write_data_to_file(unlabeled_file_path,data)
  ### Wrap Up
  print('')
  print('All done')




