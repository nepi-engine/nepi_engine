#! /bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##


# This file is in the nepi_sdk project because nepi-edge-sdk is intended to become
# open-source and general purpose, and the following is reliant on other Numurus infrastructure
# that isn't necessarily available on a generic target.

# Must source the ROS setup first, as it overwrites important messages
source /opt/nepi/engine/setup.sh
source /opt/nepi/nepi_edge_sdk_link/setup.bash

SYS_ENV_FILE=/opt/nepi/sys_env.bash

# The sys_env script must exist and be valid. The committed base file is
# (intentionally) not valid because TBD fields are not populated
if [ ! -f ${SYS_ENV_FILE} ]; then
	echo "ERROR! Could not find ${SYS_ENV_FILE}"
	exit 1
fi

source ${SYS_ENV_FILE}
if [ "$ROOTNAME" = "TBD" ]; then
	echo "ERROR! ROOTNAME must be set in ${SYS_ENV_FILE}"
	exit 1
fi

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

# Set the required env. variables
export NEPI_EDGE_ROS_BRIDGE_NS=/numurus/$DEVICE_TYPE/$DEVICE_ID
export NEPI_EDGE_ROS_BRIDGE_PARAM_FILE=/opt/nepi/engine/etc/nepi_edge_ros_bridge/nepi_edge_ros_bridge.yaml

roslaunch nepi_edge_ros_bridge nepi_edge_ros_bridge.launch --wait # Wait for main ROS master to come up
