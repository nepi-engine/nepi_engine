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
# 1. Cycles through all or list of scripts and replaces line based line starting with org_word_string
# NOTE: Must be run from command line with sudo
# NOTE: You should make a copy of your scipts in a "temp" folder and run there

import os
import sys
import glob
import fileinput


#####################################################################################
# SETUP - Edit as Necessary ##################################
##########################################
SCRIPT_FOLDER = "/mnt/nepi_storage/automation_scripts/"
SCRIPT_LIST = [] # Leave empty to update all files in folder. Add files to limit which files are updated

FIND_REPLACE_LIST = [["resources import nepi","nepi_edge_sdk_base import nepi_sdk "],
                     ["resources import nepi_navpose","nepi_edge_sdk_base import nepi_nav "],
                     ["nepi.","nepi_sdk."]]


#####################################################################################
# Methods
#####################################################################################

### Function for updating values in automation script files (All or From List)
def update_script_strings(script_folder_path,find_replace_list,optional_script_list):
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
     if file.find(".py") != -1:
       scriptlist.append([script_folder_path + file])
  print("Checking and updating the following files")
  print(scriptlist)
  for file in scriptlist:
    print("")
    print(file)
    lnum = []
    for line_ind, line in enumerate(fileinput.input(file, inplace=1)):
      for pair in find_replace_list:
        if line.find(pair[0]) != -1:
          line = line.replace(pair[0],pair[1])
          lnum.append(line_ind+1)
      print(line[0:-1])
    print('Replaced line numbers')
    print(lnum)

#####################################################################################
# Main
#####################################################################################

if __name__ == '__main__':
  update_script_strings(SCRIPT_FOLDER,FIND_REPLACE_LIST,SCRIPT_LIST)

