/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <vector>
#include <thread>
#include <algorithm>

#include <ros/ros.h>

#include "trigger_mgr_base.h"

#define MAX_TRIG_IDX				31 // Largest bit offset possible for uint32 register
#define HW_TRIG_OUT_IDX 			MAX_TRIG_IDX
#define HW_TRIG_OUT_FIXED_MASK 		(1 << MAX_TRIG_IDX) // Highest bit set.

#define DEFAULT_TRIG_MASK_VAL		0x40000000			// Everything but the output trigger -- can be changed easily in the config file
#define DEFAULT_TRIG_RATE_VAL		2.0f				// 2 Hz

namespace Numurus
{

TriggerMgrBase::TriggerMgrBase() :
	default_periodic_trig_mask_{"default_periodic_trig_mask", DEFAULT_TRIG_MASK_VAL, this},
	default_periodic_trig_rate_{"default_periodic_trig_rate", DEFAULT_TRIG_RATE_VAL, this}
{
	trig_indices[HW_TRIG_OUT_IDX] = "H/W Out";
}

TriggerMgrBase::~TriggerMgrBase()
{
	// Clear the periodic_trigger_map to signal to all (detached) worker threads that they should exit
	periodic_trig_map.clear();
	ros::Duration(1.0).sleep();
	// TODO: Consider keeping references to the worker threads and join() them here
	// instead of sleeping.
}

void TriggerMgrBase::initSubscribers()
{
	// Call the base method
	SDKNode::initSubscribers();

	// Subscribe to trigger-mgr-specific topics... these are in the public namespace because there is only one trigger manager
	subscribers.push_back(n.subscribe("sw_trigger", 5, &Numurus::TriggerMgrBase::executeSwTrig, this));
	subscribers.push_back(n.subscribe("hw_trigger_in_enab", 5, &Numurus::TriggerMgrBase::setHwTrigInEnab, this));
	subscribers.push_back(n.subscribe("hw_trigger_in_cfg", 1, &Numurus::TriggerMgrBase::configureHwTrigIn, this));
	subscribers.push_back(n.subscribe("hw_trigger_out_enab", 5, &Numurus::TriggerMgrBase::setHwTrigOutEnab, this));
	subscribers.push_back(n.subscribe("hw_trigger_out_cfg", 1, &Numurus::TriggerMgrBase::configureHwTrigOut, this));
	subscribers.push_back(n.subscribe("hw_trigger_out_dly", 1, &Numurus::TriggerMgrBase::setHwTrigOutDly, this));
	subscribers.push_back(n.subscribe("set_periodic_sw_trig", 5, &Numurus::TriggerMgrBase::setPeriodicSwTrig, this));
	// Provide a big queue for trigger_index_settings as all of these might be coming in at once
	subscribers.push_back(n.subscribe("trigger_index_settings", 31, &Numurus::TriggerMgrBase::updateTriggerIndexSet, this));
}

void TriggerMgrBase::initPublishers()
{
	// Call the base method
	SDKNode::initPublishers();

	_sw_trig_pub = n.advertise<std_msgs::UInt32>("sw_trigger", 3);
}

void TriggerMgrBase::initServices()
{
	// Call the base method
	SDKNode::initServices();

	// Advertise trigger status query service
	servicers.push_back(n.advertiseService("trigger_status_query", &Numurus::TriggerMgrBase::provideTriggerStatus, this));
	servicers.push_back(n.advertiseService("trigger_defs", &Numurus::TriggerMgrBase::provideTriggerDefs, this));
}

void TriggerMgrBase::retrieveParams()
{
	// Call the base method
	SDKNode::retrieveParams();

	default_periodic_trig_mask_.retrieve();
	default_periodic_trig_rate_.retrieve();

	if ((0 != default_periodic_trig_mask_) && (0.0f != default_periodic_trig_rate_))
	{
		setPeriodicSwTrigImpl(true, (uint32_t)default_periodic_trig_mask_, default_periodic_trig_rate_);
	}
}

void TriggerMgrBase::executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_val)
{
	// Just set the map of trigger times to now - this doesn't account for whether the trigger was successful or not
	last_trig_time_map[trig_val->data] = ros::Time::now();
}

void TriggerMgrBase::setPeriodicSwTrig(const nepi_ros_interfaces::PeriodicSwTrig::ConstPtr& trig_cfg)
{
	setPeriodicSwTrigImpl(trig_cfg->enabled, trig_cfg->sw_trig_mask, trig_cfg->rate_hz);
}

void TriggerMgrBase::setPeriodicSwTrigImpl(bool enabled, uint32_t sw_trig_mask, float rate_hz)
{
	// Bounds checking
	if ((true == enabled) && (0.0f >= rate_hz))
	{
		ROS_WARN("Invalid periodic trigger rate (%f Hz)... ignoring", rate_hz);
		return;
	}

	// If this is the default mask, make sure to update default trigger rate for param server saving
	const uint32_t default_mask = default_periodic_trig_mask_;
	if (default_mask == sw_trig_mask)
	{
		default_periodic_trig_rate_ = (enabled == false)? 0.0 : rate_hz;
	}

	// If disabling, just signal the worker thread by removing the map entry
	if (false == enabled)
	{
		auto map_entry = periodic_trig_map.find(sw_trig_mask);
		if (map_entry != periodic_trig_map.end())
		{
			ROS_INFO("Cancelling periodic sw trigger with mask 0x%x", sw_trig_mask);
			// Remove it from the map. The timer threads will (eventually) check this map for updates and
			// cancel themselves accordingly
			trig_map_mutex.lock();
			periodic_trig_map.erase(map_entry);
			trig_map_mutex.unlock();
		}
		else
		{
			ROS_WARN("No periodic trigger with mask 0x%x... ignoring cancellation request", sw_trig_mask);
		}
	}
	else // Setting up a new trigger or modifying an existing one
	{
		trig_map_mutex.lock();
		const bool periodic_trig_exists = (periodic_trig_map.find(sw_trig_mask) != periodic_trig_map.end());
		periodic_trig_map[sw_trig_mask] = rate_hz;
		trig_map_mutex.unlock();

		if (false == periodic_trig_exists)
		{
			ROS_INFO("Starting periodic sw trigger (mask=0x%x, rate=%f)", sw_trig_mask, rate_hz);
				std::thread t(&TriggerMgrBase::runPeriodicTrig, this, sw_trig_mask);
				t.detach();
		}
	}
}

void TriggerMgrBase::runPeriodicTrig(uint32_t trig_mask)
{
	const uint32_t my_trig_mask = trig_mask;
	double elapsed_sleep = 0.0;

	while (true)
	{
		float my_rate = -1.0f;
		trig_map_mutex.lock();
		const auto entry = periodic_trig_map.find(my_trig_mask);
		if (periodic_trig_map.end() != entry)
		{
			my_rate = entry->second;
		}
		trig_map_mutex.unlock();

		// If this mask is no longer one of the periodic timer masks, just terminate
		// the thread.
		if (-1.0f == my_rate)
		{
			break;
		}

		// Must maintain a reasonably low loop rate to ensure that rate changes
		// are picked up in a timely fashion (rather than at next rate clock expiration)
		// This complicates things a bit
		static constexpr double MAX_SLEEP_DURATION = 1.0; // Guaranteed response within 1 second.
		const double rate_based_sleep_duration = 1.0 / my_rate;

		const double remaining_rate_based_sleep = rate_based_sleep_duration - elapsed_sleep;
		const double sleep_duration = std::min(remaining_rate_based_sleep, MAX_SLEEP_DURATION);

		ros::Duration(sleep_duration).sleep();
		elapsed_sleep += sleep_duration;

		// If we've slept long enough, send the trigger and reset values
		if (elapsed_sleep >= rate_based_sleep_duration)
		{
			// Finally, publish a trigger message to inform this node and all others that subscribe
			// of the trigger
			std_msgs::UInt32 trig_msg;
			trig_msg.data = my_trig_mask;
			_sw_trig_pub.publish(trig_msg);

			elapsed_sleep = 0.0; // Reset for next period
		}
	}
}

void TriggerMgrBase::updateTriggerIndexSet(const nepi_ros_interfaces::TriggerIndexSettings::ConstPtr& trig_idx_settings)
{
	// Copy the settings out of the message
	const uint32_t index = trig_idx_settings->index;

	const std::string trig_name = trig_idx_settings->trigger_name;

	// Check for invalid index
	if (index == HW_TRIG_OUT_IDX)
	{
		ROS_ERROR("Trig Mgr: Rejecting invalid index setting (%u:%s) because it would overwrite fixed hardware trigger index", index, trig_name.c_str());
		return;
	}
	if (index > MAX_TRIG_IDX)
	{
		ROS_ERROR("Trig Mgr: Rejecting invalid index setting (%u:%s) because it exceeds max index (%u)", index, trig_name.c_str(), MAX_TRIG_IDX);
		return;
	}

	// Log some info
	auto entry = trig_indices.find(index);
	if (trig_indices.end() == entry)
	{
		// Restore: ROS_INFO("Trig Mgr: Received new trigger index (%u:%s)", index, trig_name.c_str());
		ROS_INFO("Trig Mgr: Received new trigger index (%u:%s)", index, trig_name.c_str());
	}
	else if (trig_name != entry->second)
	{
		ROS_WARN("Trig Mgr: Received updated trigger index (%u:%s-->%s)", index, entry->second.c_str(), trig_name.c_str());
	}

	// In all cases, update
	trig_indices[index] = trig_name;
}

bool TriggerMgrBase::provideTriggerStatus(nepi_ros_interfaces::TriggerStatusQuery::Request &req, nepi_ros_interfaces::TriggerStatusQuery::Response &resp)
{
	resp.status.trig_val = req.trig_val;

	// Check if the requested value has a periodic trigger established
	trig_map_mutex.lock();
	auto map_entry = periodic_trig_map.find(req.trig_val);
	trig_map_mutex.unlock();
	if (map_entry == periodic_trig_map.end())
	{
		resp.status.auto_rate = 0.0f;
	}
	else
	{
		resp.status.auto_rate = map_entry->second;
	}

	// TODO: No good way for s/w to determine the achieved_rate yet. Trigger manager FPGA module should
	// implement a trigger success signal routed back from each sensor node to the trigger manager,
	// and trigger manager can keep count of all of these, providing them in per-output-trigger
	// registers so s/w can derive the achieved rates. Then we probably want to report the lowest
	// rate for any output s/w trigger covered by this req.trig_val???
	// For now, spoof it:
	resp.status.achieved_rate = resp.status.auto_rate;

	// No H/W In Trig Here
	resp.status.hw_in_trig_val = 0.0f;

	// TODO: Need a way to get the h/w input trigger rate from FPGA
	resp.status.hw_in_trig_rate = 0.0f;

	// Determine the most recent time for this trigger value
	static ros::Time never(0,0);
	auto time_map_entry = last_trig_time_map.find(req.trig_val);
	if (time_map_entry == last_trig_time_map.end())
	{
		resp.status.last_execution_time = never;
	}
	else
	{
		resp.status.last_execution_time = time_map_entry->second;
	}
	return true;
}

bool TriggerMgrBase::provideTriggerDefs(nepi_ros_interfaces::TriggerDefs::Request &req, nepi_ros_interfaces::TriggerDefs::Response &resp)
{
	for (size_t i = 0; i <= MAX_TRIG_IDX; ++i)
	{
		const auto &entry = trig_indices.find(i);
		resp.trig_names[i] = (trig_indices.end() == entry)? resp.NO_TRIGGER : entry->second;
	}
	return true;
}

} // namespace Numurus
