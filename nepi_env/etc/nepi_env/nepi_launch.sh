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

nepi_count=$(process_count "nepi_engine")
if [[ "$nepi_count" -gt 1 ]]; then
	echo ""
	echo "NEPI Engine Still Running!"
	echo ""
	echo "Run 'nepistop' and try again."
else

	fi
	function run_script() {
		run_script=$1
		if [[ -f "${run_script}" ]]; then
			cd $nepi_etc_scripts
			echo "Running ${run_script}"
			bash ${run_script}
			wait
		else
			echo "Script file not found ${run_script}"
		fi
	}

	ETC_FOLDER=/opt/nepi/etc
	nepi_etc_scripts=${ETC_FOLDER}/scripts

	script_file=sync_to_sys_config.sh
	script_path=${nepi_etc_scripts}/${script_file}
	run_script $script_path

	script_file=update_sys_config.sh
	script_path=${nepi_etc_scripts}/${script_file}
	run_script $script_path

	script_file=sync_from_configs.sh
	script_path=${nepi_etc_scripts}/${script_file}
	run_script $script_path

	script_file=update_etc_users.sh
	script_path=${nepi_etc_scripts}/${script_file}
	run_script $script_path

	script_file=update_etc_ssh_keys.sh
	script_path=${nepi_etc_scripts}/${script_file}
	run_script $script_path

	echo "Running nepi setup script"
	source /opt/nepi/nepi_engine/setup.sh
	if [[ "$?" -ne 0 ]]; then
		echo "ERROR! Failed to call nepi_engine setup script from /opt/nepi/nepi_engine/setup.sh"
		return 1
	fi

	echo ""
	echo "Loading updated nepi config"
	source ${ETC_FOLDER}/load_system_config.sh
	if [[ "$?" -ne 0 ]]; then
		echo "ERROR! Failed to load system configuration values from ${ETC_FOLDER}/load_system_config.sh"
		return 1
	fi


	script_file=update_sys_config.sh
	script_path=${nepi_etc_scripts}/${script_file}
	run_script $script_path



	script_file=update_sys_bash.sh
	script_path=${nepi_etc_scripts}/${script_file}
	run_script $script_path


	SYS_BASH_FILE=/opt/nepi/etc/sys_env.bash

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

	echo "Running roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}"
	roslaunch ${ROS1_PACKAGE} ${ROS1_LAUNCH_FILE}
fi
