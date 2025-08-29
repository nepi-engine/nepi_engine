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

#from nepi_sdk import nepi_utils

# Local rootfs definitions - These can be freely changed
###################################################################################################

DOCKER_CONFIG_FILE = "nepi_docker_config.yaml"
DOCKER_CONFIG_PATH = "/mnt/nepi_config/docker_cfg"
DOCKER_CONFIG_FILE_PATH=DOCKER_CONFIG_PATH + "/" + DOCKER_CONFIG_FILE

NEPI_FULL_IMG_SUBDIR = "nepi_images"
NEPI_BACKUP_IMG_SUBDIR = "nepi_images"
NEPI_FULL_IMG_SEARCH_STRING = "nepi*.tar"

NEPI_FULL_IMG_FW_VERSION_PATH = "opt/nepi/nepi_engine/etc/fw_version.txt"

MAX_BOOT_FAILURE_ENV_VAR_NAME = "MAX_BOOT_FAILURE_COUNT"

SUPPORTS_AB_FS = True
###################################################################################################

# First-stage rootfs definitions - Do not change these unless also updating the first-stage
# rootfs structure, file contents, etc.
###################################################################################################
FLASH_ROOTFS_BOOT_FAIL_COUNT_FILE = "opt/nepi/nepi_boot_failure_count.txt"
FLASH_ROOTFS_CUSTOM_ENV_PATHNAME = "opt/nepi/nepi_rootfs_ab_custom_env.sh"

INACTIVE_PARTITION_ENV_VAR_NAME = "INACTIVE_PARTITION"
ACTIVE_PARTITION_ENV_VAR_NAME = "ACTIVE_PARTITION"
TMP_PARTITION_ENV_VAR_NAME = "TMP_PARTITION"

###################################################################################################

# Alternative Jetson+NEPI A/B Scheme: Used for Orin-NX and probably others going forward
###################################################################################################
JETSON_ROOTFS_AB_DEVICES = {'A': '/dev/nvme0n1p1', 'B': '/dev/nvme0n1p2'}
#NEPI_DOCKER_CONFIG=nepi_utils.read_yaml_2_dict(DOCKER_CONFIG_FILE_PATH)
NEPI_DOCKER_CONFIG=dict()

def CheckPartitionBusy(partition_path):
    return False

def mountPartition(part_device_pathname, part_mountpoint):
    return True, "Success"


def unmountPartition(part_mountpoint):
    return True, "Success"


def getFWVersionStringForPartition(partition_device_name):
    return True, "Success", "Uknown"


def checkForNewImagesAvailable(new_img_staging_device, staging_device_is_removable):
    return True, "New image file identified", ["FILE NAMES"], ["FILE_VERSIONS"] , ["FILE SIZES"]

def getRootfsABStatus(first_stage_rootfs_device):
    return True, "Success", dict()

def getRootfsABStatusJetson():
    return True, "Success", dict()

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
    path_to_sh = '/home/nepidev/nepi_engine_ws/setup/resources/docker/switch_nepi_docker.sh'
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
    NEPI_DOCKER_CONFIG["FAIL_COUNT"] = 0
    return True, "Success"

def switchActiveAndInactivePartitions():
    path_to_sh = '/home/nepidev/nepi_engine_ws/setup/resources/docker/switch_nepi_docker.sh'
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

# Restart fs from that call start script

#############################################
# TEMP
#############################################

if __name__ == "__main__":
    print(DOCKER_CONFIG_FILE_PATH)
    switchActiveAndInactivePartitions()