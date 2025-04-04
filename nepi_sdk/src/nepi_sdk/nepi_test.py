#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# NEPI test utility functions include

import os
import sys
import importlib

#######################
### Import Test Functions

def file_import_test(file_path):
    msg = ""
    print("")
    if os.path.exists(file_path):
      file = os.path.basename(file_path)
      if file.endswith(".py"): 
        print("Testing import of file: " + file)
        module_name = file.replace('.py','')
        print("Importing module: " + module_name)
        msg = ""
        try:
          module = importlib.import_module(module_name)
          msg = file + " : Import Succeeded"
        except Exception as e:
          msg = file + " : Import Failed: " + str(e)
        print( msg )
    return msg


'''
# RUN TEST COMMAND LINE FROM ANY FOLDER
python -c "from nepi_sdk import nepi_test; nepi_test.folder_import_test()"
'''
def folder_import_test(folder = os.getcwd()):
    msg_dict = dict()
    if os.path.exists(folder):
        if folder[-1] == "/":
            folder = folder[:-1]
        sys.path.append(folder)
        print("Searching for python files in path: " + folder)
        for file in os.listdir(folder):
          if file.endswith(".py"): 
            file_path = os.path.join(folder,file)
            msg = file_import_test(file_path)
            msg_dict[file] = msg
    return msg_dict
     

