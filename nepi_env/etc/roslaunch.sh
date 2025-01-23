#! /bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

source /opt/nepi/ros/setup.sh

SYS_ENV_FILE=/opt/nepi/sys_env.bash

# The sys_env script must exist and be valid. The committed base file is
# (intentionally) not valid because TBD fields are not populated
if [ ! -f ${SYS_ENV_FILE} ]; then
	echo "ERROR! Could not find ${SYS_ENV_FILE}"
	exit 1
fi

source ${SYS_ENV_FILE}
if [ "$DEVICE_TYPE" = "TBD" ]; then
	echo "ERROR! DEVICE_TYPE must be set in ${SYS_ENV_FILE}"
	exit 1
fi

if [ "$DEVICE_SN" = "TBD" ]; then
	echo "ERROR! DEVICE_SN must be set in ${SYS_ENV_FILE}"
	exit 1
fi

if [ -z "$DEVICE_ID" ]; then
	export DEVICE_ID=$DEVICE_SN
fi

if [ "$ROS1_PACKAGE" = "TBD" ] || [ "$ROS1_LAUNCH_FILE" = "TBD" ]; then
	echo "No ROS1 defs in ${SYS_ENV_FILE}... nothing to launch"
	exit 0
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
python /opt/nepi/ros/etc/fix_broken_cfg_file_links.py

# Tune ethernet interfaces for fast sensor throughput (especially important for genicam)
echo "Running pre-launch ethernet interface tuning"
python /opt/nepi/ros/etc/tune_ethernet_interfaces.py

roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}
