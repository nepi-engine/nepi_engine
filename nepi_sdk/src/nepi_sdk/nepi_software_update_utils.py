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

import rospy

# Local rootfs definitions - These can be freely changed
###################################################################################################
FLASH_ROOTFS_MOUNTPOINT = "/mnt/flash"
INACTIVE_PARTITION_MOUNTPOINT = "/mnt/inactive"
STAGING_MOUNTPOINT = "/mnt/staging"
TMP_MOUNTPOINT = "/mnt/tmp"

NEPI_FULL_IMG_SUBDIR = "nepi_full_img"
NEPI_BACKUP_IMG_SUBDIR = "nepi_full_img_archive"
NEPI_FULL_IMG_SEARCH_STRING = "nepi*img.raw"
NEPI_FULL_IMG_FW_VERSION_PATH = "opt/nepi/ros/etc/fw_version.txt"
###################################################################################################

# First-stage rootfs definitions - Do not change these unless also updating the first-stage
# rootfs structure, file contents, etc.
###################################################################################################
FLASH_ROOTFS_BOOT_FAIL_COUNT_FILE = "opt/nepi/nepi_boot_failure_count.txt"
FLASH_ROOTFS_CUSTOM_ENV_PATHNAME = "opt/nepi/nepi_rootfs_ab_custom_env.sh"

INACTIVE_PARTITION_ENV_VAR_NAME = "INACTIVE_PARTITION"
ACTIVE_PARTITION_ENV_VAR_NAME = "ACTIVE_PARTITION"
TMP_PARTITION_ENV_VAR_NAME = "TMP_PARTITION"
MAX_BOOT_FAILURE_ENV_VAR_NAME = "MAX_BOOT_FAILURE_COUNT"
###################################################################################################

# Alternative Jetson+NEPI A/B Scheme: Used for Orin-NX and probably others going forward
###################################################################################################
JETSON_ROOTFS_AB_DEVICES = {'A': '/dev/nvme0n1p1', 'B': '/dev/nvme0n1p2'}


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
        return True, "Already unmounted"

    unmount_rc = subprocess.call(["umount", part_mountpoint])
    if unmount_rc == 0:
        return True, "Success"
    else:
        return False, "Failed to unmount"


def getFWVersionStringForPartition(partition_device_name):
    status, err_msg = mountPartition(partition_device_name, TMP_MOUNTPOINT)
    if status is False:
        return status, err_msg, None

    fw_version_pathname = os.path.join(
        TMP_MOUNTPOINT, NEPI_FULL_IMG_FW_VERSION_PATH)
    if not os.path.isfile(fw_version_pathname):
        unmountPartition(TMP_MOUNTPOINT)
        return False, "No firmware version file", None

    with open(fw_version_pathname) as f:
        fw_version = f.read()

    unmountPartition(TMP_MOUNTPOINT)
    return True, "Success", fw_version.strip()


def checkForNewImageAvailable(new_img_staging_device, staging_device_is_removable):
    # First, check that the new image staging device actually exists. If not, probably just
    # removable
    if staging_device_is_removable and not os.path.exists(new_img_staging_device):
        return True, "Removable staging device not detected", None, None, 0

    # Now mount the staging partition
    status, err_string = mountPartition(
        new_img_staging_device, STAGING_MOUNTPOINT)
    if status is False:
        return status, err_string, None, None, 0

    new_img_search_string = os.path.join(
        STAGING_MOUNTPOINT, NEPI_FULL_IMG_SUBDIR, NEPI_FULL_IMG_SEARCH_STRING)

    img_files = glob.glob(new_img_search_string)

    if (len(img_files) == 0):
        unmountPartition(STAGING_MOUNTPOINT)
        return True, "No new image file identified", None, None, 0
    elif (len(img_files) > 1):
        unmountPartition(STAGING_MOUNTPOINT)
        # TODO: Maybe we should allow more than one detectable image so users can select desired one?
        return False, "More than one image file identified", None, None, 0

    # Detected a single image, so grab the version string from inside it (with the staging parition still mounted, of course)... 
    # this validates that it the image file mountable and is (probably) a nepi rootfs image
    new_img_pathname = img_files[0]
    status, err_msg, new_img_version = getFWVersionStringForPartition(
        new_img_pathname)
    
    if os.path.exists(new_img_pathname):
        new_img_filesize = os.path.getsize(new_img_pathname)
    else:
        status = False
        new_img_filesize = 0

    unmountPartition(STAGING_MOUNTPOINT)

    if status == False:
        return False, err_msg, None, None, 0

    return True, "New image file identified", os.path.basename(new_img_pathname), new_img_version, new_img_filesize

def getRootfsABStatus(first_stage_rootfs_device):
    rootfs_ab_status_dict = {}

    # First, mount the FLASH partition where the current ACTIVE/INACTIVE device info is stored
    status, err_msg = mountPartition(
        first_stage_rootfs_device, FLASH_ROOTFS_MOUNTPOINT)
    if status is False:
        return status, err_msg, rootfs_ab_status_dict

    custom_env_pathname = os.path.join(
        FLASH_ROOTFS_MOUNTPOINT, FLASH_ROOTFS_CUSTOM_ENV_PATHNAME)
    if not os.path.isfile(custom_env_pathname):
        return False, "Env. file does not exist", rootfs_ab_status_dict

    boot_fail_count_file_pathname = os.path.join(
        FLASH_ROOTFS_MOUNTPOINT, FLASH_ROOTFS_BOOT_FAIL_COUNT_FILE)
    if not os.path.isfile(boot_fail_count_file_pathname):
        return False, "Boot failure count file does not exist", rootfs_ab_status_dict

    command = shlex.split("env -i bash -c 'source " +
                          custom_env_pathname + " && env'")
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, text=True)
    for line in proc.stdout:
        (key, _, value) = line.partition("=")
        if key == ACTIVE_PARTITION_ENV_VAR_NAME:
            rootfs_ab_status_dict["active_part_device"] = value.strip()
        elif key == INACTIVE_PARTITION_ENV_VAR_NAME:
            rootfs_ab_status_dict["inactive_part_device"] = value.strip()
        elif key == MAX_BOOT_FAILURE_ENV_VAR_NAME:
            rootfs_ab_status_dict["max_boot_fail_count"] = int(value.strip())

    proc.communicate()

    # Gather the running boot failure count separately
    with open(boot_fail_count_file_pathname, 'r') as f:
        running_fail_count = int(f.read())
        rootfs_ab_status_dict["running_boot_fail_count"] = int(running_fail_count)

    unmountPartition(FLASH_ROOTFS_MOUNTPOINT)

    # Now grab the fw_version for the inactive partition
    inactive_part_fw_version_pathname = os.path.join(INACTIVE_PARTITION_MOUNTPOINT, NEPI_FULL_IMG_FW_VERSION_PATH)
    status, err_msg = mountPartition(
        rootfs_ab_status_dict["inactive_part_device"], INACTIVE_PARTITION_MOUNTPOINT)
    if status is False or not os.path.isfile(inactive_part_fw_version_pathname):
        rootfs_ab_status_dict["inactive_part_fw_version"] = "unknown"
    else:
        with open(inactive_part_fw_version_pathname, 'r') as f:
           rootfs_ab_status_dict["inactive_part_fw_version"] = f.read().strip() 
        unmountPartition(INACTIVE_PARTITION_MOUNTPOINT)

    return True, "Success", rootfs_ab_status_dict

def getRootfsABStatusJetson():
    rootfs_ab_status_dict = {}

    command = ['nvbootctrl', '-t', 'rootfs', 'dump-slots-info']
    slots_info = subprocess.check_output(command, text=True)
    current_rootfs_slot = None
    for line in slots_info.splitlines():
        tokens = line.split(':')
        if tokens[0] == 'Current rootfs slot': # NEPI defines "active" to be the current, Jetson defines "current" to be the current
            current_rootfs_slot = tokens[1].strip()
            inactive_rootfs_slot = 'B' if current_rootfs_slot == 'A' else 'A'
            rootfs_ab_status_dict["active_part_device"] = JETSON_ROOTFS_AB_DEVICES[current_rootfs_slot]
            rootfs_ab_status_dict["inactive_part_device"] = JETSON_ROOTFS_AB_DEVICES[inactive_rootfs_slot]
        elif tokens[0] == 'Active rootfs slot': # NEPI defines "next active" to be the next one, Jetson defines that as "active"
            next_rootfs_slot = tokens[1].strip()
            rootfs_ab_status_dict["next_active_part_device"] = JETSON_ROOTFS_AB_DEVICES[next_rootfs_slot]
        elif tokens[0] == 'slot':
            slot_val = tokens[1].strip()[0] # First character of the comma-delineated line
            if (current_rootfs_slot == 'A' and slot_val == '0') or (current_rootfs_slot == 'B' and slot_val == '1'):
                rootfs_ab_status_dict["max_boot_fail_count"] = int(tokens[2].strip()[0])

    # Validate
    if ('active_part_device' not in rootfs_ab_status_dict) or \
       ('inactive_part_device' not in rootfs_ab_status_dict) or \
       ('max_boot_fail_count' not in rootfs_ab_status_dict):
        return False, "nvbootctrl call or parse failure", rootfs_ab_status_dict
    
    # Now grab the fw_version for the inactive partition
    inactive_part_fw_version_pathname = os.path.join(INACTIVE_PARTITION_MOUNTPOINT, NEPI_FULL_IMG_FW_VERSION_PATH)
    status, err_msg = mountPartition(
        rootfs_ab_status_dict["inactive_part_device"], INACTIVE_PARTITION_MOUNTPOINT)
    if status is False or not os.path.isfile(inactive_part_fw_version_pathname):
        rootfs_ab_status_dict["inactive_part_fw_version"] = "unknown"
    else:
        with open(inactive_part_fw_version_pathname, 'r') as f:
           rootfs_ab_status_dict["inactive_part_fw_version"] = f.read().strip() 
    unmountPartition(INACTIVE_PARTITION_MOUNTPOINT)

    return True, "Success", rootfs_ab_status_dict

def identifyRootfsABScheme():
    try:
        slot_count = int(subprocess.check_output(['nvbootctrl', '-t', 'rootfs', 'get-number-slots']))
        if slot_count == 2:
            return 'jetson'
    except:
        pass

    # Default to 'nepi'
    return 'nepi'

def getPartitionByteCount(partition_device):
    return int(subprocess.check_output(["blockdev", "--getsize64", partition_device], text=True))

def getPartitionFreeByteCount(partition_device):
    # Have to mount it to compute free size
    status, err_msg = mountPartition(
        partition_device, STAGING_MOUNTPOINT)
    if status is False:
        return -1

    statvfs = os.statvfs(STAGING_MOUNTPOINT)
    return (float(statvfs.f_frsize) * statvfs.f_bavail)

def writeImage(new_img_staging_device, uncompressed_img_filename, inactive_partition_device, do_slow_transfer, progress_cb=None):
    if not os.path.exists(new_img_staging_device):
        return False, "Staging partition device does not exist"

    if not os.path.exists(inactive_partition_device):
        return False, "Inactive partition device does not exist"

    status, err_msg = mountPartition(
        new_img_staging_device, STAGING_MOUNTPOINT)
    if status is False:
        return False, err_msg

    uncompressed_img_pathname = os.path.join(
        STAGING_MOUNTPOINT, NEPI_FULL_IMG_SUBDIR, uncompressed_img_filename)
    if not os.path.exists(uncompressed_img_pathname):
        return False, "Image file does not exist"

    bytes_to_write = os.path.getsize(uncompressed_img_pathname)
    dest_bytes_available = getPartitionByteCount(inactive_partition_device)

    # Check that the destination can fit the image.
    # TODO: Should there be some additional buffer space required here?
    if (bytes_to_write > dest_bytes_available):
        unmountPartition(STAGING_MOUNTPOINT)
        return False, "Not enough space on inactive partition"

    dd_param_array = ["dd", "if=" + uncompressed_img_pathname,
                            "of=" + inactive_partition_device, "status=progress"]
    if do_slow_transfer: 
        dd_param_array.append("bs=64K")
    else:
        dd_param_array.append("bs=32M")
    #print("DEBUG: dd cmd: " + str(dd_param_array))

    dd_proc = subprocess.Popen(dd_param_array, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, text=True)
    if progress_cb is not None:
        while dd_proc.poll() is None:
            line = dd_proc.stdout.readline()
            #print("DEBUG: dd output: " + line)
            # Parse dd output to figure out how much has been written
            if "copied" in line:
                bytes_written = line.split()[0]
                progress_cb(float(bytes_written)/float(bytes_to_write))
    dd_final_out = dd_proc.communicate()[0] 
    
    unmountPartition(STAGING_MOUNTPOINT)

    if dd_proc.returncode == 0:
        return True, "Success"
    else:
        return False, "Failed (dd error: " + str(dd_proc.returncode) + ": " + dd_final_out + ")"

def checkAndRepairPartition(partition_device):
    fsck_rc = subprocess.call(["fsck", "-a", partition_device])

    if int(fsck_rc) < 4: # No errors or errors were repaired
        return True, "Success"
    else:
        return False, "Failed to check and repair partition"

def resetBootFailCounter(first_stage_rootfs_device):
    # First, mount the FLASH partition where the boot fail counter file is stored
    status, err_msg = mountPartition(
        first_stage_rootfs_device, FLASH_ROOTFS_MOUNTPOINT)
    if status is False:
        return status, err_msg

    # Now reset the boot fail counter to zero
    boot_fail_count_file_pathname = os.path.join(
        FLASH_ROOTFS_MOUNTPOINT, FLASH_ROOTFS_BOOT_FAIL_COUNT_FILE)
    with open(boot_fail_count_file_pathname, 'w') as boot_fail_count_file:
        boot_fail_count_file.write('0')

    # And unmount before we exit
    unmountPartition(FLASH_ROOTFS_MOUNTPOINT)
    return True, "Success"

def switchActiveAndInactivePartitions(first_stage_rootfs_device):
    # First, mount the FLASH partition where the current ACTIVE/INACTIVE device info is stored
    status, err_msg = mountPartition(
        first_stage_rootfs_device, FLASH_ROOTFS_MOUNTPOINT)
    if status is False:
        return status, err_msg

    # Open the environment file
    custom_env_pathname = os.path.join(
        FLASH_ROOTFS_MOUNTPOINT, FLASH_ROOTFS_CUSTOM_ENV_PATHNAME)
    if not os.path.isfile(custom_env_pathname):
        return False, "Env. file does not exist"

    # Read in current file and make swap as appropriate
    with open(custom_env_pathname, 'r') as f:
        data = f.read()
        # Must use 3 slots to swap two entries
        data = data.replace(INACTIVE_PARTITION_ENV_VAR_NAME,
                            TMP_PARTITION_ENV_VAR_NAME)
        data = data.replace(ACTIVE_PARTITION_ENV_VAR_NAME,
                            INACTIVE_PARTITION_ENV_VAR_NAME)
        data = data.replace(TMP_PARTITION_ENV_VAR_NAME,
                            ACTIVE_PARTITION_ENV_VAR_NAME)

    with open(custom_env_pathname, 'w') as f:
        f.write(data)

    unmountPartition(FLASH_ROOTFS_MOUNTPOINT)
    return True, "Success"

def switchActiveAndInactivePartitionsJetson():
    current_slot_num = int(subprocess.check_output(['nvbootctrl', '-t', 'rootfs', 'get-current-slot'], text=True))
    new_slot_num = '1' if current_slot_num == 0 else '0'

    subprocess.call(['nvbootctrl', '-t', 'rootfs', 'set-active-boot-slot', new_slot_num], text=True)
    return True, "Success"

def archiveInactiveToStaging(inactive_partition_device, staging_device, archive_file_basename, do_slow_transfer, progress_cb=None):
    # First, check that the devices exist and are mounted/unmounted as necessary
    if not os.path.exists(staging_device):
        return False, "Staging partition device does not exist"

    if not os.path.exists(inactive_partition_device):
        return False, "Inactive partition device does not exist"

    # Now make sure things are mounted/unmounted properly for the copy
    status, err_msg = mountPartition(
        staging_device, STAGING_MOUNTPOINT)
    if status is False:
        return False, err_msg

    statvfs = os.statvfs(STAGING_MOUNTPOINT)
    dest_bytes_available = statvfs.f_frsize * statvfs.f_blocks

    # Ensure the source partition is unmounted
    unmountPartition(inactive_partition_device)
    bytes_to_write = getPartitionByteCount(inactive_partition_device)

    #print("DEBUG: Archive requires " + str(bytes_to_write) + " bytes and have " + str(dest_bytes_available))

    # Check that the destination can fit the image.
    # TODO: Should there be some additional buffer space required here?
    if (bytes_to_write > dest_bytes_available):
        unmountPartition(STAGING_MOUNTPOINT)
        return False, "Not enough space on staging device"
        
    backup_staging_dir = os.path.join(STAGING_MOUNTPOINT, NEPI_BACKUP_IMG_SUBDIR)
    if not os.path.isdir(backup_staging_dir):
        os.mkdir(backup_staging_dir)

    if not archive_file_basename.endswith('.img.raw'):
        archive_file_basename = archive_file_basename + '.img.raw'

    backup_img_pathname = os.path.join(backup_staging_dir, archive_file_basename)

    dd_param_array = ["dd", "if=" + inactive_partition_device,
                            "of=" + backup_img_pathname, "status=progress"]
    if do_slow_transfer: 
        dd_param_array.append("bs=64K")
    else:
        dd_param_array.append("bs=32M")
    #print("DEBUG: dd cmd: " + str(dd_param_array))

    dd_proc = subprocess.Popen(dd_param_array, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, text=True)
    if progress_cb is not None:
        while dd_proc.poll() is None:
            line = dd_proc.stdout.readline()
            #print("DEBUG: dd output: " + line)
            # Parse dd output to figure out how much has been written
            if "copied" in line:
                bytes_written = line.split()[0]
                progress_cb(float(bytes_written)/float(bytes_to_write))
    
    dd_final_out = dd_proc.communicate()[0] 
    
    unmountPartition(STAGING_MOUNTPOINT)

    if dd_proc.returncode == 0:
        return True, "Success"
    else:
        return False, "Failed (dd error: " + str(dd_proc.returncode) + ": " + dd_final_out + ")"


'''
DEPRECATED: Images must already be decompressed on staging media
def decompressImageIfNecessary(new_img_staging_device, new_img_filename):
    # First, mount the staging partition
    status, err_string = mountPartition(
        new_img_staging_device, STAGING_MOUNTPOINT)
    if status is False:
        return status, err_string, None    

    new_img_pathname = os.path.join(STAGING_MOUNTPOINT, NEPI_FULL_IMG_SUBDIR, new_img_filename)
    if not os.path.isfile(new_img_pathname):
        return False, "Invalid image pathname", None

    if new_img_pathname.endswith('.gz'):
        gzip_rc = subprocess.call(["gunzip", "-v", new_img_pathname])
        if gzip_rc is 0:
            return True, "Decompression successful", new_img_filename[:-3]
        else:
            return False, "Failed to decompress", None

    return True, "Image already decompressed", new_img_filename
'''
