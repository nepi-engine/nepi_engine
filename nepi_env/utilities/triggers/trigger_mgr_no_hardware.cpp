/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include "trigger_mgr_no_hardware.h"

#define NODE_NAME	"trigger_mgr"

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ROS_INFO("Starting the %s node", NODE_NAME);
		
	Numurus::TriggerMgrNoHardware trig_mgr;
	trig_mgr.run();

	return 0;
}
