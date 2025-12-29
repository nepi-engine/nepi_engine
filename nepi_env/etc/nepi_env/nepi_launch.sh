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
NEPI_ETC_SCRIPTS=${ETC_FOLDER}/scripts
SYS_BASH_FILE=/opt/nepi/etc/sys_env.bash


nepi_count=$(process_count "nepi_engine")
if [[ "$nepi_count" -gt 1 ]]; then
	echo ""
	echo "NEPI Engine Still Running!"
	echo ""
	echo "Run 'nepistop' and try again."
else


	#####################################

	ros_log_path=/mnt/nepi_storage/logs/ros_log
	if [[ -d "$ros_log_path" ]]; then
		echo "Clearing old logs in: ${ros_log_path}"
		sudo rm -r ${ros_log_path}/* 2>/dev/null
	fi

	lost_found_path='/mnt/nepi_storage/lost+found'
	if [[ -d "$lost_found_path" ]]; then
		echo "Clearing lost and found in: ${lost_found_path}"
		sudo rm -r ${lost_found_path}/* 2>/dev/null
	fi

	function run_script() {
		run_script=$1
		arg_1=""
		if [[ -n "$2" ]]; then
			arg_1=$2
		fi
		if [[ -f "${run_script}" ]]; then
			cd $NEPI_ETC_SCRIPTS 
			echo "Running ${run_script}"
			bash ${run_script} $arg_1
			wait
		else
			echo "Script file not found ${run_script}"
		fi
	}


	#####################################

	script_file=nepi_system_sync.sh
	script_path=${NEPI_ETC_SCRIPTS}/${script_file}
	echo "Running script ${script_path}"
	run_script $script_path


	echo ""
	echo "Loading Updated NEPI System Config"
	source ${ETC_FOLDER}/load_system_config.sh
	if [[ "$?" -ne 0 ]]; then
		echo "ERROR! Failed to load system configuration values from ${ETC_FOLDER}/load_system_config.sh"
		return 1
	fi

	LOAD_CONFIG=0

	script_file=update_sys_config.sh
	script_path=${NEPI_ETC_SCRIPTS}/${script_file}
	run_script $script_path $LOAD_CONFIG


	script_file=update_sys_bash.sh
	script_path=${NEPI_ETC_SCRIPTS}/${script_file}
	run_script $script_path $LOAD_CONFIG


	script_file=update_etc_users.sh
	script_path=${NEPI_ETC_SCRIPTS}/${script_file}
	run_script $script_path $LOAD_CONFIG

	script_file=update_etc_ssh_keys.sh
	script_path=${NEPI_ETC_SCRIPTS}/${script_file}
	run_script $script_path $LOAD_CONFIG

	#########################################

	echo "Running NEPI Setup Script"
	source /opt/nepi/nepi_engine/setup.sh
	if [[ "$?" -ne 0 ]]; then
		echo "ERROR! Failed to call nepi_engine setup script from /opt/nepi/nepi_engine/setup.sh"
		return 1
	fi

	if [ ! -f ${SYS_BASH_FILE} ]; then
		echo "ERROR! Could not find ${SYS_BASH_FILE}"
		return 1
	fi

	echo ""
	echo "Sourcing nepi sys bash file ${SYS_BASH_FILE}"
	source ${SYS_BASH_FILE}

	# CHECK FOR VALID ROS Package
	if [ "$ROS1_PACKAGE" = "TBD" ] || [ "$ROS1_LAUNCH_FILE" = "TBD" ]; then
		echo "ERROR! No ROS defs in ${SYS_BASH_FILE}... nothing to launch"
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


	# # Check for and restore any broken config files, since that will cause roslaunch to fail
	# echo "Running pre-launch config file checks"
	# python /opt/nepi/nepi_engine/etc/nepi_env/fix_broken_cfg_file_links.py

	########################

	echo "Running roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}"
	roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}
fi
