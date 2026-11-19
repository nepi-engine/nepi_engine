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

# Copy the entire existing folder for archive purposes
NOW=`date +"%F_%H%M%S"`
ARCHIVE_FOLDER=numurus_ros_sdk_archive_${REMOTE_HOST}_$NOW
echo "Archiving the existing installation to $ARCHIVE_FOLDER.tar.gz... this may take a moment"
sleep 3
rsync -avzhe "ssh -i $SSH_KEY_PATH" numurus@192.168.179.102:/opt/nepi/nepi_engine $ARCHIVE_FOLDER
tar -czf $ARCHIVE_FOLDER.tar.gz $ARCHIVE_FOLDER
rm -rf $ARCHIVE_FOLDER
