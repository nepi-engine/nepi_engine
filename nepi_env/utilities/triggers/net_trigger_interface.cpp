/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */

#include <net_trigger_interface.h>

namespace Numurus
{
NetTriggerInterface::NetTriggerInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh, 
				   						 uint32_t parent_trig_index, std::function<void(SDKNode&)> parent_trig_callback) :
	TriggerInterface{parent, parent_pub_nh, parent_priv_nh, parent_trig_index},
	_parent_trig_callback{parent_trig_callback}
{}

NetTriggerInterface::~NetTriggerInterface(){}

void NetTriggerInterface::initSubscribers()
{
	// Call the base method
	TriggerInterface::initSubscribers();

	// Register directly for the s/w trig topic
	subscribers.push_back(_parent_pub_nh->subscribe("sw_trigger", 5, &Numurus::NetTriggerInterface::executeSwTrig, this));
}

void NetTriggerInterface::executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_mask)
{
	// First, check if triggering is enabled - if not (e.g., Node3DX is paused, just return without doing the callback)
	if (false == _trig_enabled) return; 

	// Check if the mask includes our value (index). If not, just ignore and return
	const bool mask_match = (trig_mask->data & (1 << _trig_index)) != 0;
	if (false == mask_match) return;

	const int delay_usecs = _trig_delay;
	const float delay_secs = ((float)delay_usecs) / 1000000.0f;
	ros::Duration(delay_secs).sleep();

	// Execute the callback
	_parent_trig_callback(*_parent_node);
}

} // namespace Numurus