#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##


# This interactive script installs an entire local copy of the /opt/nepi/ros directory
# to a remote machine, overwriting the existing folder if it exists.
# Before overwriting, the existing /opt/nepi/ros directory on the remote target
# is archived locally.

# The following environment variables can be set ahead of time outside this script for
# faster running:
# SSH_KEY_PATH ==> full path to the private key for the remote system. Defaults to $HOME/.ssh/numurus/numurus_3dx_jetson_sshkey
# REMOTE_HOST ==> IP address or resolvable hostname for the remote system. Defaults to 192.168.179.102
# SRC_PATH ==> Path to the "ros" folder to be written to /opt/nepi/ros on the remote system. Defaults to ../../
# SERIAL_NUM ==> Device serial number. Defaults to 0000

#echo "THIS SCRIPT NEEDS REWORK TO INSTALL ENTIRE /opt/nepi FOLDER"
#exit 1
echo "Warning -- this script will overwrite existing settings on the device."

if [ -z "$SSH_KEY_PATH" ]; then
	SSH_KEY_PATH=$HOME/.ssh/numurus_3dx_jetson_sshkey
fi

if [ ! -f "$SSH_KEY_PATH" ]; then
	echo "Enter the path to the SSH key for the remote system"
	read -e -p "Enter the path to the SSH key for the remote system
	> " -i $SSH_KEY_PATH SSH_KEY_PATH
	if [ ! -f "$SSH_KEY_PATH" ]; then
		echo "Error: $SSH_KEY_PATH does not exist... exiting"
		exit 1
	fi
fi

if [ -z "$REMOTE_HOST" ]; then
	REMOTE_HOST=192.168.179.102
	read -e -p "Enter the remote hostname or IP address
	> " -i $REMOTE_HOST REMOTE_HOST
fi

# Set the source path. Assumes this is running from ros/share
if [ -z "$SRC_PATH" ]; then
	SRC_PATH=../..
	read -e -p "Enter the path to the installation folder (parent of 'ros' subfolder)
	> " -i $SRC_PATH SRC_PATH
fi

cd $SRC_PATH
#echo "Navigated to `pwd`"

if [ ! -d "./ros" ]; then
	echo "Error: Installation folder `pwd`/ros does not exist... exiting"
	exit 1
fi

if [ -z "$SERIAL_NUM" ]; then
	SERIAL_NUM=000000
	read -e -p "Enter the serial number for the device
	> " -i $SERIAL_NUM SERIAL_NUM
fi

echo "Check these selections carefully. Press ctl+C to abort the install or press enter to continue"
echo "   Remote Host   = $REMOTE_HOST"
echo "   Source Path   = `pwd`/ros"
echo "   Serial Number = $SERIAL_NUM"
read CONTINUE

# This no longer works because serial number is at /opt/nepi/sys_env.bash on the remote system
#cp ros/etc/sys_env_base.bash ros/etc/sys_env.bash
#sed -i 's/DEVICE_TYPE=TBD/DEVICE_TYPE=3dsc/' ros/etc/sys_env.bash
#sed -i 's/DEVICE_SN=.*/DEVICE_SN='$SERIAL_NUM'/' ros/etc/sys_env.bash
#sed -i 's/SDK_PROJECT=TBD/SDK_PROJECT=num_sdk_jetson/' ros/etc/sys_env.bash

# Stop the running SDK
echo numurus | ssh -tt -i $SSH_KEY_PATH numurus@$REMOTE_HOST "sudo systemctl stop roslaunch; sudo systemctl stop numurus_rui"

# Copy the entire existing folder for archive purposes
NOW=`date +"%F_%H%M%S"`
ARCHIVE_FOLDER=`pwd`/pre_install_archive_$REMOTE_HOST_$NOW
echo "Archiving the existing installation to $ARCHIVE_FOLDER.tar.gz... this may take a moment"
sleep 3
rsync -avzhe "ssh -i $SSH_KEY_PATH" numurus@$REMOTE_HOST:/opt/nepi/ros $ARCHIVE_FOLDER
tar -czf $ARCHIVE_FOLDER.tar.gz $ARCHIVE_FOLDER
rm -rf $ARCHIVE_FOLDER

# Delete the remote folder
echo "Removing the old installation on the remote system"
sleep 3
echo numurus | ssh -tt -i $SSH_KEY_PATH numurus@$REMOTE_HOST "rm -rf /opt/nepi/ros/*"

# Write the new folder to the remote system
echo "Uploading the new installation"
sleep 3
rsync -avzhe "ssh -i $SSH_KEY_PATH" ros/* numurus@192.168.179.102:/opt/nepi/ros/

# All done
echo "Installation Complete. Prior installation is archived at $ARCHIVE_FOLDER.tar.gz"
echo "Reboot the remote device to complete the update"
