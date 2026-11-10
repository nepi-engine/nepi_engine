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


# Sample NEPI Helper Script. 
# 1. Cycles through all or list of scripts and replaces line based line starting with key_word_string
# NOTE: Must be run from command line with sudo
# NOTE: You should make a copy of your scipts in a "temp" folder and run there

import os
import sys
import glob
import fileinput


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################
SCRIPT_FOLDER = "/mnt/nepi_storage/temp/"
SCRIPT_LIST = ["image_enhance_process_script.py"] # Leave empty to update all files in folder. Add files to limit which files are updated

KEY_WORD_STRING = "IMAGE_INPUT_TOPIC"
KEY_WORD_VALUE_OR_STRING = "/nepi/s2x/testcam2/idx/color_2d_image"

#KEY_WORD_STRING = "SOME_KEY_WORD"
#KEY_WORD_VALUE_OR_STRING = 5.333


#####################################################################################
# Methods
#####################################################################################

### Function for updating values in automation script files (All or From List)
def update_script_key_word_value(script_folder_path="",key_word="",key_value_or_string="",optional_script_list=[]):
  filelist=os.listdir(script_folder_path)
  if len(optional_script_list) != 0: # Just These Scripts
    scriptlist=[]
    for ind, file in enumerate(filelist):
      for script in optional_script_list:
        if file.find(script)!= -1:
          scriptlist.append([script_folder_path + file])
  else: # All Scripts
    scriptlist=[]
    for ind, file in enumerate(filelist):
     scriptlist.append([script_folder_path + file])
  print("Checking and updating the following files")
  print(scriptlist)
  search_str_len=len(key_word)
  for file in scriptlist:
    print("")
    print(file)
    lnum = []
    for ind, line in enumerate(fileinput.input(file, inplace=1)):
      line = line.strip()
      if line[0:search_str_len] == key_word:
        lnum.append(ind)
        if type(key_value_or_string) == str:
          new_line = '{} = "{}"'.format(key_word, key_value_or_string)
        else:
          new_line = '{} = {}'.format(key_word, key_value_or_string)
      else:
        new_line = line
      print(new_line)
    print('Replaced line numbers')
    print(lnum)    

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  update_script_key_word_value(SCRIPT_FOLDER,KEY_WORD_STRING,KEY_WORD_VALUE_OR_STRING,SCRIPT_LIST)

