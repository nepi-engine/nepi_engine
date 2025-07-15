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
import yaml

##########################################
# SETUP - Edit as Necessary 
##########################################

IMAGE_FILE_TYPES = ['jpg','JPG','jpeg','png','PNG']


##########################################
# Variables
##########################################

abs_path = os.path.realpath(__file__)
current_folder = os.path.dirname(abs_path)


##########################################
# Methods
##########################################



def get_folder_list(folder_path):
  filelist=os.listdir(folder_path + '/')
  folder_list=[]
  #print('')
  #print('Files and Folders in Path:')
  #print(folder_path)
  #print(filelist)
  for file in enumerate(filelist):
    foldername = (folder_path + '/' + file[1])
    #print('Checking file: ')
    #print(foldername)
    if os.path.isdir(foldername): # file is a folder
       folder_list.append(foldername)
  return folder_list

def open_new_file(file_path):
  print('')
  if os.path.isfile(file_path):
    print('Deleting existing file:')
    print(file_path)
    os.remove(file_path)
  print('Creating new file: ' + file_path)
  fnew = open(file_path, 'w')
  return fnew

def read_list_from_file(file_path):
    lines = []
    with open(file_path) as f:
        lines = [line.rstrip() for line in f] 
    return lines

def write_list_to_file(data_list, file_path):
    success = True
    try:
        with open(file_path, 'w') as file:
            for data in data_list:
                file.write(data + '\n')
    except Exception as e:
        print("Failed to write list to file " + file_path + " " + str(e))
        success = False
    return success



def read_yaml_2_dict(file_path):
    dict_from_file = dict()
    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                dict_from_file = yaml.load(f, Loader=yaml.FullLoader)
        except Exception as e:
            print("Failed to get dict from file: " + file_path + " " + str(e))
    else:
        print("Failed to find dict file: " + file_path)
    return dict_from_file


def write_dict_2_yaml(dict_2_save,file_path,defaultFlowStyle=False,sortKeys=False):
    success = False
    try:
        with open(file_path, "w") as f:
            yaml.dump(dict_2_save, stream=f, default_flow_style=defaultFlowStyle, sort_keys=sortKeys)
        success = True
    except Exception as e:
        print("Failed to write dict: "  + " to file: " + file_path + " " + str(e))
    return success