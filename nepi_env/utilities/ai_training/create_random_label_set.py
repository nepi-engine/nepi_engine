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
import shutil

import create_deploy_model
import ai_utils

##########################################
# SETUP - Edit as Necessary 
##########################################

NUM_IMAGES = 100


data_folder_path = os.path.join(ai_utils.current_folder,'data_labeling')
random_label_set = os.path.join(data_folder_path, 'random_label_set')


##########################################
# Methods
##########################################

def create_random_label_set(folder_path):
    #print('')
    if os.path.exists(random_label_set) == True:
        try:
            shutil.rmtree(random_label_set)
            #print("Successfully deleted random set folder")
        except OSError as e:
            print("Error deleting folder " + str(e))

    #image_files = os.listdir(folder_path)

    image_files = []
    
    print("Processing folders in: " + data_folder_path)
    folders_to_process=ai_utils.get_folder_list(data_folder_path)
    print('')
    print('Found folders: ' + str(folders_to_process))
    for folder in folders_to_process:
        print('Processing folder: ' + folder)
        path, dirs, files = next(os.walk(folder))
        for f in files:
            f_ext = os.path.splitext(f)[1]
            f_ext = f_ext.replace(".","")
            if f_ext in ai_utils.IMAGE_FILE_TYPES:
                file_path = os.path.join(folder, f)
                #print("Found: " + file_path)
                image_files.append(file_path)
    #image_files = random.sample(image_files, len(image_files))
    
    random.shuffle(image_files)

    if NUM_IMAGES > len(image_files):
        raise ValueError("Number of images cannot be larger than data set")

    os.mkdir(random_label_set)

    for i in range(NUM_IMAGES):
        image_file_path = image_files[i]
        #print("Image file path: " + image_file_path)
        try:
            shutil.copy(image_file_path, random_label_set)
            #print("File: " + image_file_path + " Copied to: " + random_label_set)
        except FileNotFoundError:
            print("Error file " + image_file_path + "not found") 
        except Exception as e:
            print("Excepton: " + str(e))


    

###############################################
# Main
###############################################

if __name__ == '__main__':

    create_random_label_set(data_folder_path)