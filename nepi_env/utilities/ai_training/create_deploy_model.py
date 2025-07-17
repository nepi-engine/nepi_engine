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
import shutil

import train_yolov8_model
import ai_utils


##########################################
# SETUP - Edit as Necessary 
##########################################

data_folder_path = os.path.join(ai_utils.current_folder,'data_labeling')

model_deploy_path = os.path.join(ai_utils.current_folder,'model_deploy')
model_training = os.path.join(ai_utils.current_folder,'model_training')
classes_file_path = os.path.join(data_folder_path,'classes.txt')

pt_file_path = os.path.join(model_deploy_path, train_yolov8_model.model_name + '.pt')
yaml_file_path = os.path.join(model_deploy_path, train_yolov8_model.model_name + '.yaml')

FRAME_WORK_NAME = 'yolov8'
TYPE_NAME = 'detection'
DESCRIPTION_NAME = 'light bulb object detector'


##########################################
# Methods
##########################################

def copy_file_to_new_destination(file_path, destination_path):
    print('')
    if os.path.exists(destination_path) == False:
        with open(pt_file_path, 'x') as f:
            pass
    try:
        shutil.copy(file_path, destination_path)
        print("File: " + file_path + " Copied to: " + destination_path)
    except FileNotFoundError:
        print("Error file " + file_path + "not found") 
    except Exception as e:
        print("Excepton: " + str(e))

def get_weight_path(target_weight):
    weight = None
    print('')
    for dirpath, dirnames, files in os.walk('model_training'):
        for f in files:
            if target_weight in f:
                file_path = os.path.join(dirpath, f)
                #print("Found: " + file_path)
                weight = file_path
    return weight

def create_yaml_file():
    ### Create dictionary
    classes_list = ai_utils.read_list_from_file(classes_file_path)
    data = {
        'ai_model' : {
            'framework' : {
                'name' : FRAME_WORK_NAME
            },
            'type' : {
                'name' : TYPE_NAME
            },
            'description' : {
                'name' : DESCRIPTION_NAME
            },
            'weight_file' : {
                'name' : train_yolov8_model.model_name + '.pt'
            },
            'image_size' : {
                'image_width' : {
                    'value' : train_yolov8_model.CURRENT_IMGSZ
                },
                'image_height' : {
                    'value' : train_yolov8_model.CURRENT_IMGSZ
                }
            },
            'classes' : {
                'names' : classes_list
            }
        }
    }

    success = ai_utils.write_dict_2_yaml(data, yaml_file_path)
    # print("Yaml created: " + success)
    return success


###############################################
# Main
###############################################

if __name__ == '__main__':

    # Get latest weight

    weight_path = get_weight_path("best.pt")

    # Move weight to model_deploy

    if weight_path is not None:
        copy_file_to_new_destination(weight_path, pt_file_path)
    else:
        print("Weight path is None")

    # Create yaml file in model deploy
    create_yaml_file()

