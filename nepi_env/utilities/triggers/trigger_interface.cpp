/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include "nepi_ros_interfaces/TriggerIndexSettings.h"

#include "trigger_interface.h"

namespace Numurus
{
TriggerInterface::TriggerInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh, uint32_t parent_trig_index):
	SDKInterface{parent, parent_pub_nh, parent_priv_nh},
	_trig_enabled{true},
	_trig_index{parent_trig_index},
	_trig_delay{"trig_delay", 0, parent}
{}

TriggerInterface::~TriggerInterface(){}

void TriggerInterface::retrieveParams()
{
	// Call the base class method
	SDKInterface::retrieveParams();

	_trig_delay.retrieve();
}

void TriggerInterface::initPublishers()
{
	// Call the base class method
	SDKInterface::initPublishers();

	// Advertise and publish (with latch) the trigger index so trig mgr. can populate trig defs appropriately
	_trig_index_pub = _parent_pub_nh->advertise<nepi_ros_interfaces::TriggerIndexSettings>("trigger_index_settings", 5, true); // true to latch it; ensure it is sent on each new subscriber connection
	
	// Publish TriggerIndexSettings once to latch automatic response to future subscribers
	nepi_ros_interfaces::TriggerIndexSettings index_settings;
	index_settings.trigger_name = _parent_node->getDisplayName(); // Use display name for configurability
	index_settings.index = _trig_index;
	_trig_index_pub.publish(index_settings);
}

void TriggerInterface::initSubscribers()
{
	// Call the base class method
	SDKInterface::initSubscribers();

	subscribers.push_back(_parent_priv_nh->subscribe("set_trig_delay", 1, &TriggerInterface::setTrigDelay, this));
}

void TriggerInterface::setTrigDelay(const std_msgs::UInt32::ConstPtr& trig_delay_val_usecs)
{
	_trig_delay = trig_delay_val_usecs->data;
}
	
} // namespace Numurus
