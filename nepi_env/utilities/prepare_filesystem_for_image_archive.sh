#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##


# This script cleans up a working system ahead of a release archive
# It is intended to run locally on a NEPI device (S2X, etc)
# It must be run under sudo or as root
TMP_FOLDER=/home/nepi/tmp
NEPI_ROS_FOLDER=/opt/nepi/ros
NUID_FOLDER=/opt/nepi/nepi_link/nepi-bot/devinfo

NEPI_BOT_FOLDER=/opt/nepi/nepi_link/nepi-bot
NEPI_BOT_LOG_CONTENTS=${NEPI_BOT_FOLDER}/log/*
NEPI_BOT_DB_CONTENTS=${NEPI_BOT_FOLDER}/db/nepibot.db
NEPI_BOT_HB_CONTENTS=${NEPI_BOT_FOLDER}/hb/*
NEPI_BOT_LB_CONTENTS=${NEPI_BOT_FOLDER}/lb/*
NEPI_BOT_NUID_FILE=${NEPI_BOT_FOLDER}/devinfo/devnuid.txt

if [ "$EUID" -ne 0 ]
then
  echo "Must run this script as root (or under sudo)"
  exit
fi

# Safety check
echo 'This script will make permanent changes to your NEPI filesystem'
echo 'It is intended to be run only in order to prepare a generic archive of a NEPI filesystem'
read -p 'Are you sure you want to continue (y/n)?' response
if [ "$response" != "y" ]; then
  echo "Exiting due to response $response"
  exit 1
fi

# Another safety check
FW_REV=`cat $NEPI_ROS_FOLDER/etc/fw_version.txt`
if grep -q "dirty" <<< "$FW_REV"; then
  echo "This version of software, $FW_REV, does not appear to be release-ready"
  read -p 'Are you sure you want to continue (y,n)?' response
  if [ "$response" != "y" ]; then
    echo "Exiting"
    exit 1
  fi
fi

echo "Cleaning out the tmp folder"
rm -rf $TMP_FOLDER/*
echo "... done"

echo "Reverting to factory settings"
source NEPI_ROS_FOLDER/setup.bash
rostopic pub -1 /nepi/s2x/reset nepi_ros_interfaces/Reset "reset_type: 1" 

echo "Clearing out NEPI-Bot temporary files"
rm -rf $NEPI_BOT_LOG_CONTENTS $NEPI_BOT_DB_CONTENTS $NEPI_BOT_HB_CONTENTS $NEPI_BOT_LB_CONTENTS
echo "... done"

echo "Resetting NUID and Connect SSH key to UNSET"
cd /opt/nepi/nepi_link/nepi-bot/devinfo
python ./change_identity.py -n UNSET

echo "Filesystem prep complete... can proceed to archive this filesystem for release"
