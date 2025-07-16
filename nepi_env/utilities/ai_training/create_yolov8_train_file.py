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
MAX_NUM_IMAGES = -1 #100 # set to -1 to use all
VAL_DATA_PERCENTAGE = 10
TEST_DATA_PERCENTAGE = 10
MAKE_TRAIN_TEST_UNIQUE = True

data_folder_path = os.path.join(ai_utils.current_folder,'data_labeling')
train_folder_path = os.path.join(ai_utils.current_folder,'model_training')

classes_file_path = os.path.join(data_folder_path,'classes.txt')
train_file_path = os.path.join(train_folder_path,'train_data.txt')
val_file_path = os.path.join(train_folder_path,'val_data.txt')
test_file_path = os.path.join(train_folder_path,'test_data.txt')

custom_data_file_path = os.path.join(train_folder_path,'data_custom.yaml')
##########################################
# Methods
##########################################

def process_folder():


  train_files = []
  val_files = []
  test_files = []
  ulab_files = []

  ### Walk through folder folders
  print("Processing folders in: " + data_folder_path)
  folders_to_process=ai_utils.get_folder_list(data_folder_path)
  print('')
  print('Found folders: ' + str(folders_to_process))
  for folder in folders_to_process:
    print('Processing folder: ' + folder)
    path, dirs, files = next(os.walk(folder))
    data_size = len(files)
    ind = 0
    data_val_size = int(float(1)/float(VAL_DATA_PERCENTAGE) * data_size)
    val_indexes = random.sample(range(data_size), k=data_val_size)
    data_test_size = int(float(1)/float(TEST_DATA_PERCENTAGE) * data_size)
    test_indexes = random.sample(range(data_size), k=data_test_size)
    files = os.listdir(folder)
    #print("Found " + str(len(files)) + " files in folder")
    #print('Found image files: ' + str(files))
    for f in files:
      f_ext = os.path.splitext(f)[1]
      f_ext = f_ext.replace(".","")
      try:
        if f_ext in ai_utils.IMAGE_FILE_TYPES:
          image_file = (folder + '/' + f)
          #print('Found image file: ' + image_file)
          #print(image_file)
          label_file = (folder + '/' + f.split(f_ext)[0]+'txt')
          #print('Looking for label file: ' + label_file)
          if os.path.exists(label_file):
            #print('Found label file' + label_file)
            ind += 1
            if ind in val_indexes: 
              #print('Adding image to val file list')
              val_files.append(image_file)
            elif ind in test_indexes: 
              #print('Adding image to test file list')
              test_files.append(image_file)
            else: 
              #print('Adding image to train file list')
              train_files.append(image_file)
          else:
            # print("Warning: No label file for image: " + image_file)
            ulab_files.append(image_file)
          max_num_images = len(files)
          if MAX_NUM_IMAGES > 0:
            max_num_images = MAX_NUM_IMAGES
          if ind > max_num_images:
            break
      except Exception as e:
        print("Excepton on file write: " + str(e))

  print("Found " + str(len(ulab_files)) + " unlabeled files")
  ### Create return data
  # data = ulab_files

  ### Create train/test data set file
  ai_utils.write_list_to_file(train_files, train_file_path)
  if os.path.exists(val_file_path) == False:
    ai_utils.write_list_to_file(val_files, val_file_path)
  if os.path.exists(test_file_path) == False:
    ai_utils.write_list_to_file(test_files, test_file_path)
    
  

  ### Create dictionary

  classes_list = ai_utils.read_list_from_file(classes_file_path)
  number_of_classes = len(classes_list)
  #print(test_files)
  #print(test_label_files)
  #print(train_files)
  #print(val_files)


  #data : dict[str, any] = {
  data = {
    'path' : train_folder_path,
    'train' : os.path.basename(train_file_path),
    'val' : os.path.basename(val_file_path),
    'test' : os.path.basename(test_file_path),
    'nc' : number_of_classes,
    'names' : classes_list
  }
  return data
            


def write_data_to_file( data):
  
  ### Create data
  success = ai_utils.write_dict_2_yaml(data, custom_data_file_path)
  return success
   
  
    



  

  

###############################################
# Main
###############################################

if __name__ == '__main__':

  # output_file_path = os.path.join(current_folder, 'unlabeled_data_list.txt')

  ### Process Data
  # data = ai_utils.read_yaml_2_dict(data_folder_path)
  data = process_folder()
  ### Write File
  success = write_data_to_file(data)
  ### Wrap Up
  print('')
  print('All done')




