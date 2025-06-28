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

from nepi_sdk import nepi_serial

from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_drvs"
logger = Logger(log_name = log_name)


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
     

'''
# RUN TEST COMMAND LINE FROM ANY FOLDER
python -c "from nepi_sdk import nepi_test; nepi_test.test_cv2_cuda()"
'''

def test_cv2_cuda():    
  import cv2
  cuda_available = False
  try:
      count = cv2.cuda.getCudaEnabledDeviceCount()
      if count > 0:
          cuda_available = True
          print(f"OpenCV is using CUDA. {count} CUDA-enabled device(s) found.")
      else:
          print("OpenCV is not using CUDA.")
  except:
      print("OpenCV was likely not built with CUDA support.")

  if cuda_available:
      print(f"Device name: {cv2.cuda.getDevice(0).getName()}")
  return cuda_available


'''
# RUN TEST COMMAND LINE FROM ANY FOLDER
python -c "from nepi_sdk import nepi_test; nepi_test.test_o3d_cuda()"
'''
def test_o3d_cuda():     
  import open3d as o3d
  cuda_available = False
  if o3d.core.cuda.is_available():
      cuda_available = True
      print("Open3D with CUDA is available.")
  else:
      print("Open3D with CUDA is not available.")
  return cuda_available



'''
# RUN TEST COMMAND LINE FROM ANY FOLDER
python -c "import torch; from nepi_sdk import nepi_test; nepi_test.test_torch_cuda()"
'''
def test_torch_cuda():      
  cuda_available = False
  if torch.cuda.is_available():
      cuda_available = True
      print("CUDA is available!")
      device = torch.device('cuda')
  else:
      print("CUDA is not available.")
      device = torch.device('cpu')
  return cuda_available


