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

import os
from nepi_edge_sdk_base import nepi_sdk
from nepi_sdk import nepi_utils

     
if __name__ == '__main__':
  CAL_SRC_PATH = "/usr/local/zed/settings"
  USER_CFG_PATH = "/mnt/nepi_storage/user_cfg"
  CAL_BACKUP_PATH = USER_CFG_PATH + "/zed_cals"
  # Try to backup camera calibration files
  [success,files_copied,files_not_copied] = nepi_utils.copy_files_from_folder(CAL_SRC_PATH,CAL_BACKUP_PATH)
  if success:
    #print("Backed up zed cal files")
    if len(files_copied) > 0:
      strList = str(files_copied)
      print("Backed up zed cal files: " + strList)
  else:
    print("Failed to back up up zed cal files")


    # Try to restore camera calibration files from
  [success,files_copied,files_not_copied] = nepi_utils.copy_files_from_folder(CAL_BACKUP_PATH,CAL_SRC_PATH)
  if success:
    if len(files_copied) > 0:
      strList = str(files_copied)
      print("Restored zed cal files: " + strList)
  else:
    print("Failed to restore zed cal files")


