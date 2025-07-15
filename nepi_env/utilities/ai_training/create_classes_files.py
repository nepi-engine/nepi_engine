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

classes_file_path = os.path.join(data_folder_path,'classes.txt')

##########################################
# Methods
##########################################

def process_folder():
  classes_list = ai_utils.read_list_from_file(classes_file_path)
  if len(classes_list) > 0:

    ### Walk through folder folders
    print("Processing folders in: " + data_folder_path)
    folders_to_process=ai_utils.get_folder_list(data_folder_path)
    print('')
    print('Found folders: ' + str(folders_to_process))
    for folder in folders_to_process:
      success = True
      file_path = os.path.join(folder,'classes.txt')
      if os.path.exists(file_path):
        try:
            os.remove(file_path)
            print(f"File '{file_path}' deleted successfully.")
        except OSError as e:
            success = False
            print(f"Error deleting file '{file_path}': {e}")
      if success == True:
        print('saving classes.text to: ' + file_path)
        ai_utils.write_list_to_file(classes_list,file_path)
  return classes_list
   

###############################################
# Main
###############################################

if __name__ == '__main__':

  # output_file_path = os.path.join(current_folder, 'unlabeled_data_list.txt')

  ### Process Data
  # data = ai_utils.read_yaml_2_dict(data_folder_path)
  data = process_folder()
  ### Write File
  ### Wrap Up
  print('')
  print('All done')




