#! /bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

cur_dir=$(pwd)


ETC_FOLDER=/opt/nepi/etc
nepi_etc_scripts=${ETC_FOLDER}/scripts

if [[ -f "$nepi_etc_scripts/update_sys_config.sh" ]]; then
	cd $nepi_etc_scripts
	echo "Updating NEPI System Cofiguration files from ${nepi_etc}"
	source update_sys_config.sh
	wait
fi
if [[ -f "$nepi_etc_scripts/sync_from_configs.sh" ]]; then
	echo "Syncing NEPI Cofiguration files to ${nepi_etc}"
	cd $nepi_etc_scripts
	source sync_from_configs.sh
	wait
fi

cd $cur_dir
echo "Running nepi setup script"
source /opt/nepi/nepi_engine/setup.sh
if [[ "$?" -ne 0 ]]; then
	echo "ERROR! Failed to call nepi_engine setup script from /opt/nepi/nepi_engine/setup.sh"
	return 1
fi

echo "Loading updated nepi config"
source ${ETC_FOLDER}/load_system_config.sh
if [[ "$?" -ne 0 ]]; then
	echo "ERROR! Failed to load system configuration values from ${ETC_FOLDER}/load_system_config.sh"
	return 1
fi

# Update and source nepi sys_env 
SYS_ENV_FILE=/opt/nepi/etc/sys_env.bash

if [ ! -f ${SYS_ENV_FILE} ]; then
	echo "ERROR! Could not find ${SYS_ENV_FILE}"
	return 1
fi

function update_value(){
  FILE=$1
  KEY=$2
  UPDATE=$3
  if [ -f "$FILE" ]; then
    if grep -q "$KEY" "$FILE"; then
      sed -i "/^$KEY/c\\$UPDATE" "$FILE"
    else
      echo "$UPDATE" | sudo tee -a $FILE
    fi
  else
    echo "File not found ${FILE}"
  fi
}

echo ""
echo "Checking for Valid Config Settings"

# CHECK FOR VALID DEVICE ID
# Check for empty string
if [ -z "$NEPI_DEVICE_ID" ]; then
	echo "ERROR! NEPI ID's can not be blank string."
	return 1
fi
# Check that first char is a letter
if [[ ! "$NEPI_DEVICE_ID" =~ ^[a-zA-Z] ]]; then
	echo "ERROR! The first character or NEPI ID must be a letter."
	return 1
fi
# Check if input is only letters numbers and underscores with no spaces
if [[ ! "$NEPI_DEVICE_ID" =~ ^[a-zA-Z0-9_]+$ ]]; then
	echo "ERROR! NEPI ID's must be only letters, numbers, and underscores with no spaces."
	return 1
fi

# CHECK FOR VALID DEVICE MODEL NAME
# Check for empty string
if [ -z "$NEPI_DEVICE_MD" ]; then
	echo "ERROR! NEPI Device Model Name can not be blank string."
	return 1
fi

# CHECK FOR VALID DEVICE SN
# Check for empty string
if [ -z "$NEPI_DEVICE_SN" ]; then
	echo "ERROR! NEPI Serial Numbers can not be blank string."
	return 1
fi
# Check if serial number is valid 6 digit number
if [[ ! "$NEPI_DEVICE_SN" =~ ^[0-9]{6}$ ]]; then
	echo "'ERROR! $NEPI_DEVICE_SN' is not a valid 6-digit number."
	return 1
fi

echo ""
echo "Updating nepi system bash file"
echo "Using Device ID: ${NEPI_DEVICE_ID}"
update_value ${SYS_ENV_FILE} "export DEVICE_ID" "export DEVICE_ID=${NEPI_DEVICE_ID}"
echo "Using Device Model Name: ${NEPI_DEVICE_MD}"
update_value ${SYS_ENV_FILE} "export DEVICE_TYPE" "export DEVICE_TYPE=${NEPI_DEVICE_MD}"
echo "Using Device Serial Number: ${NEPI_DEVICE_SN}"
update_value ${SYS_ENV_FILE} "export DEVICE_SN" "export DEVICE_SN=${NEPI_DEVICE_SN}"



# Check if system hostname has changed
if [[ "${HOSTNAME}" != "${NEPI_DEVICE_ID}" ]]; then
	echo "System Hostname has changed, Running ETC hostname update script"
	. /opt/nepi/etc/scripts/update_etc_hostname.sh
fi


echo ""
echo "Sourcing nepi system bash file"
source ${SYS_ENV_FILE}

# CHECK FOR VALID ROS Package
if [ "$ROS1_PACKAGE" = "TBD" ] || [ "$ROS1_LAUNCH_FILE" = "TBD" ]; then
	echo "ERROR! No ROS defs in ${SYS_ENV_FILE}... nothing to launch"
	return 0
fi

# This is a good place to monitor and purge large log sets
ROS_LOG_SIZE=($(rosclean check))
echo ROS Log Size: $ROS_LOG_SIZE
# A "G" in the size indicates logs are over 1GB, so we purge
if grep -q "G" <<< "$ROS_LOG_SIZE"; then
	echo "Purging ROS logs because they exceed 1GB"
	yes | rosclean purge
fi


# Check for and restore any broken config files, since that will cause roslaunch to fail
echo "Running pre-launch config file checks"
python /opt/nepi/nepi_engine/etc/nepi_env/fix_broken_cfg_file_links.py

# Tune ethernet interfaces for fast sensor throughput (especially important for genicam)
echo "Running pre-launch ethernet interface tuning"
python /opt/nepi/nepi_engine/etc/nepi_env/tune_ethernet_interfaces.py

roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}
