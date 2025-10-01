#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# These utility functions work in conjunction with the NEPI First-Stage Filesystem. You
# must ensure that the NEPI First-Stage Filesystem is installed before using any
# utilities here.

# Each function in this utility set returns a status (True/False for success/failure) and a
# status string indicating failure cause if any. Some functions also return a data object.

import glob
import os.path
import os
import subprocess
import shlex
#import psutil

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system

# Local rootfs definitions - These can be freely changed
###################################################################################################


NEPI_CONFIG_FILE='/opt/nepi/etc/nepi_system_config.yaml'
config_dict=nepi_utils.read_dict_from_file(NEPI_CONFIG_FILE)

def CheckPartitionBusy(partition_path):
    return False

def mountPartition(part_device_pathname, part_mountpoint):
    return True, "Success"


def unmountPartition(part_mountpoint):
    return True, "Success"


def getFWVersionStringForPartition(partition_device_name):
    return True, "Success", "Uknown"


def checkForNewImagesAvailable(new_img_staging_device, staging_device_is_removable):
    return True, "New image file identified", ["nepi-3p2p2-jetson-orin-5d.tar"], ["3p2p2-jetson-orin-5d"] , [100]

def getRootfsABStatus():
    rootfs_ab_status_dict = {}

    rootfs_ab_status_dict["active_part_device"] = "nepi_fs_a"
    rootfs_ab_status_dict["inactive_part_device"] = "nepi_fs_b"
    rootfs_ab_status_dict["max_boot_fail_count"] = 1
    rootfs_ab_status_dict["running_boot_fail_count"] = 0
    rootfs_ab_status_dict["inactive_part_fw_version"] = "uknown"
    return True, "Success", rootfs_ab_status_dict

def identifyRootfsABScheme():
    return 'container'

# Get how much availible space
def getPartitionByteCount(partition_device):
    return 100000000000 

# How much pace is free
def getPartitionFreeByteCount(partition_device):
    return 100000000000

# Export
def writeImage(new_img_staging_device, uncompressed_img_filename, inactive_partition_device, do_slow_transfer, progress_cb=None):
    path_to_sh = '/mnt/nepi_config/docker_cfg/switch_nepi_docker.sh'
    try:
        # Execute the shell script and wait for it to complete
        result = subprocess.run([path_to_sh], check=True, capture_output=True, text=True)     
        # Print the output and exit code
        print("STDOUT:", result.stdout)
        print("STDERR:", result.stderr)
        print("Exit Code:", result.returncode)
    except FileNotFoundError:
        print("Error: The 'sh' command or the script file was not found.")
    except subprocess.CalledProcessError as e:
        # The `check=True` flag causes this exception to be raised on non-zero exit codes
        print("Error: The script returned a non-zero exit code.")
        print("STDOUT:", e.stdout)
        print("STDERR:", e.stderr)
        print("Exit Code:", e.returncode)
    return True, "Success"

def checkAndRepairPartition(partition_device):
    return True, "Success"

def resetBootFailCounter(first_stage_rootfs_device):
    nepi_system.update_nepi_docker_config("NEPI_FAIL_COUNT",0)
    nepi_system.update_nepi_docker_config("NEPI_TEST",0)
    # print("<<<<<<<<<<<<<<<<<<<<<<<<<<")
    # print("Updating Fail Boot Count")
    # print(">>>>>>>>>>>>>>>>>>>>>>>>>>")
    return True, "Success"

def switchActiveAndInactivePartitions():
    path_to_sh = '/mnt/nepi_config/docker_cfg/switch_nepi_docker.sh'
    try:
        # Execute the shell script and wait for it to complete
        result = subprocess.run([path_to_sh], check=True, capture_output=True, text=True)
        # Print the output and exit code
        print("STDOUT:", result.stdout)
        print("STDERR:", result.stderr)
        print("Exit Code:", result.returncode)
    except FileNotFoundError:
        print("Error: The 'sh' command or the script file was not found.")
    except subprocess.CalledProcessError as e:
        # The `check=True` flag causes this exception to be raised on non-zero exit codes
        print("Error: The script returned a non-zero exit code.")
        print("STDOUT:", e.stdout)
        print("STDERR:", e.stderr)
        print("Exit Code:", e.returncode)
    return True, "Success"

def switchActiveAndInactivePartitionsJetson():
    return True, "Success"

def archiveInactiveToStaging(inactive_partition_device, staging_device, archive_file_basename, do_slow_transfer, progress_cb=None):
    return True, "Success"


def restart():
    success = False
    return success
# Restart fs from that call start script

#############################################
# TEMP
#############################################

if __name__ == "__main__":
    print(DOCKER_CONFIG_FILE_PATH)
    switchActiveAndInactivePartitions()