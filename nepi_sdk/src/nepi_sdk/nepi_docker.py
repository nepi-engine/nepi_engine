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
import copy

from nepi_sdk import nepi_utils
from nepi_sdk import nepi_system

BLANK_IMAGE_DICT={
    'name': 'uknown',
    'version': 'uknown',
    'size_mb': 0,
    'hw_type': 'uknown',
    'sw_desc': 'uknown',
    'date': 'uknown',
}

def getContainerInfo(which_container = 'Active'):
    info_dict = copy.deepcopy(BLANK_IMAGE_DICT)
    return info_dict

def getImageFileInfo(image_file):
    info_dict = copy.deepcopy(BLANK_IMAGE_DICT)
    return info_dict


def mountPartition(part_device_pathname, part_mountpoint):
    # Might already be mounted
    if os.path.ismount(part_mountpoint):
        return True, "Already mounted"

    if not os.path.exists(part_device_pathname):
        return False, "Partition device does not exist"

    if not os.path.isdir(part_mountpoint):
        os.mkdir(part_mountpoint)

    mount_rc = subprocess.call(
        ["mount", part_device_pathname, part_mountpoint])
    if mount_rc == 0:
        return True, "Success"
    else:
        return False, "Failed to mount"


def unmountPartition(part_mountpoint):
    # Might already be unmounted
    if not os.path.ismount(part_mountpoint):
        return True, "Partition Already unmounted"

    if CheckPartitionBusy(part_mountpoint) == True:
        return False, "Partition Busy"

    unmount_rc = subprocess.call(["umount", part_mountpoint])
    if unmount_rc == 0:
        return True, "Success"
    else:
        return False, "Failed to unmount"



def checkForNewImagesAvailable(image_install_path, install_device_is_removable):
    new_img_files=[]
    new_img_versions=[]
    new_img_filesizes=[]
    files_dict=nepi_utils.get_folder_files(image_install_path)
    print(files_dict)
    if "tar" in files_dict.keys():
        tar_files=files_dict["tar"]
        key='-'
        split_tar_files = [item.split(key) for item in tar_files]
        new_img_files = [inner_array[0] for inner_array in split_tar_files]
        new_img_versions = [inner_array[1] for inner_array in split_tar_files]
        new_img_filesizes = [inner_array[4] for inner_array in split_tar_files]
        # print("New Image Files: ", new_img_files)
        # print("New Image Versions: ", new_img_versions)
        # print("New Image Filesizes: ", new_img_filesizes)
        # print("Tar Files Array: " + str(split_tar_files))
    return True, "New image file identified", new_img_files, new_img_versions, new_img_filesizes


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

# Import
def installImage(new_img_staging_device, uncompressed_img_filename, inactive_partition_device, do_slow_transfer, progress_cb=None):
    nepi_system.update_nepi_docker_config("NEPI_FS_IMPORT", 1)
    return True, "Success"
    

def checkAndRepairPartition(partition_device):
    return True, "Success"

def resetBootFailCounter(first_stage_rootfs_device):
    nepi_system.update_nepi_docker_config("NEPI_FAIL_COUNT",0)
    # print("<<<<<<<<<<<<<<<<<<<<<<<<<<")
    # print("Updating Fail Boot Count")
    # print(">>>>>>>>>>>>>>>>>>>>>>>>>>")
    return True, "Success"

def switchActiveAndInactiveContainers():
    nepi_system.update_nepi_docker_config("NEPI_FS_SWITCH", 1)
    return True, "Success"

# Export
def saveImage(inactive_partition_device, staging_device, archive_file_basename, do_slow_transfer, progress_cb=None , info_dict = BLANK_IMAGE_DICT):
    nepi_system.update_nepi_docker_config("NEPI_FS_EXPORT", 1)
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