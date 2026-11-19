#!/bin/bash
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


# This script cleans up a working system ahead of a release archive
# It is intended to run locally on a NEPI device (S2X, etc)
# It must be run under sudo or as root
TMP_FOLDER=/home/nepi/tmp
NEPI_ROS_FOLDER=/opt/nepi/nepi_engine
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
rostopic pub -1 /nepi/device1/reset nepi_interfaces/Reset "reset_type: 1" 

echo "Clearing out NEPI-Bot temporary files"
rm -rf $NEPI_BOT_LOG_CONTENTS $NEPI_BOT_DB_CONTENTS $NEPI_BOT_HB_CONTENTS $NEPI_BOT_LB_CONTENTS
echo "... done"

echo "Resetting NUID and Connect SSH key to UNSET"
cd /opt/nepi/nepi_link/nepi-bot/devinfo
python ./change_identity.py -n UNSET

echo "Filesystem prep complete... can proceed to archive this filesystem for release"
