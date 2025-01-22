/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <vector>
#include <boost/algorithm/string.hpp>

#include "std_msgs/String.h"
#include "nepi_ros_interfaces/FileReset.h"

#include "sdk_node.h"
#include "sdk_utils.h"

namespace Numurus
{
static std::vector<std::string> splitNamespace()
{
	const std::string ns_string = ros::this_node::getName();
	std::vector<std::string> tokens;

	// Use a lambda function to provide the delimiter identifier
	boost::split(tokens, ns_string, [](char c){return c == '/';});

	return tokens;
}

static const std::string extractDeviceNamespace(const std::vector<std::string> &ns_tokens)
{
	if (ns_tokens.size() < 3)
	{
		ROS_ERROR("Invalid namespace token length");
		return "";
	}
	// Trial and error determined these token indices
	const std::string device_ns = "/" + ns_tokens[1] + "/" + ns_tokens[2];
	return device_ns;
}

SDKNode::SDKNode() :
	ns_tokens{splitNamespace()},
	NODE_NAME_INDEX{ns_tokens.size() - 1},
  current_rate_hz{0},
	n{extractDeviceNamespace(ns_tokens)},
	n_priv{"~"}, // Create a private namespace for this node - just use the fully qualified node name
	_param_list{}, // Call default constructor explicityly here -- must come before any parameters are initialized
	_display_name{"display_name", ns_tokens[NODE_NAME_INDEX], this} // Default to the fixed node name
{
	if (false == validateNamespace())
	// Validate the namespace
	{
		// Kill the node - this is a critical failure
		ROS_FATAL("Invalid namespace for an SDKNode (%s)", ros::this_node::getName().c_str());
		ros::shutdown();
	}
}

SDKNode::~SDKNode()
{}

void SDKNode::init()
{
	// initPublishers() first to ensure that any messages published by the other inits() are valid
	initPublishers();

	retrieveParams();

	initSubscribers();
	initServices();
	initServiceClients();

	// Do the interfaces, too
	for (auto interface : sdk_interfaces)
	{
		interface->initPublishers();
		interface->retrieveParams();
		interface->initSubscribers();
		interface->initServices();
		interface->initServiceClients();
	}

	// Now that all retrieveParams() have been called, check that all defined parameters were
	// actually retrieved
	warnUnretrievedParams();

  current_rate_hz = max_rate_hz;

	_initialized = true;
}

void SDKNode::run()
{
	// Do init() if it hasn't already been done
	if (false == _initialized)
	{
		init();
	}

  // Spin at the current rate
  while (ros::ok())
  {
    ros::Rate r(current_rate_hz);
	  ros::spinOnce();
    r.sleep();
  }
}

void SDKNode::initPublishers()
{
	// Advertise the save_cfg coordination topics
	_store_params_pub = n.advertise<std_msgs::String>("store_params", 5);
	_submit_sys_err_msg_pub = n.advertise<std_msgs::String>("submit_system_error_msg", 5);
}

void SDKNode::retrieveParams()
{
	// Clear parameter retrieval flags to make subsequent calls to warnUnretrievedParams() work properly
	for (NodeParamBase *p : _param_list) p->clearRetrievalFlag();

	// To appease the rosparam API, all nodes should have at least one parameter (to ensure a non-empty namespace
	// when dumping the param server contents to various config files). We use the displayName for that purpose,
	// though it is not modifiable at this generic SDKNode level
	_display_name.retrieve();
}

void SDKNode::initSubscribers()
{
	// These versions are in the public namespace so that we can support param reinit and update
	// messages to ALL of the SDK nodes simultaneously
	subscribers.push_back(n.subscribe("save_config", 3, &SDKNode::saveCfgHandler, this));
	subscribers.push_back(n.subscribe("save_config_rt", 3, &SDKNode::saveCfgRtHandler, this));
	subscribers.push_back(n.subscribe("reset", 3, &SDKNode::resetHandler, this));
  subscribers.push_back(n.subscribe("apply_throttle", 3, &SDKNode::applyThrottleHandler, this));

	// These versions are in this nodes private namespace so that just this node can be reinit'd and/or updated
	subscribers.push_back(n_priv.subscribe("save_config", 3, &SDKNode::saveCfgHandler, this));
	subscribers.push_back(n_priv.subscribe("save_config_rt", 3, &SDKNode::saveCfgRtHandler, this));
	subscribers.push_back(n_priv.subscribe("reset", 3, &SDKNode::resetHandler, this));
  subscribers.push_back(n_priv.subscribe("apply_throttle", 3, &SDKNode::applyThrottleHandler, this));
}

void SDKNode::initServices()
{
	// No services - just a placeholder for subclasses
}

void SDKNode::initServiceClients()
{
	// No clients - just a placeholder for subclasses
}

void SDKNode::warnUnretrievedParams()
{
	for (NodeParamBase *p : _param_list)
	{
		if (false == p->getRetrievalFlag())
		{
			ROS_WARN("%s: did not retrieve %s in retrieveParams()", getUnqualifiedName().c_str(), p->getName().c_str());
		}
	}
}

void SDKNode::saveCfgHandler(const std_msgs::Empty::ConstPtr &msg)
{
	ROS_DEBUG("%s: Initiating config save by request", getUnqualifiedName().c_str());
	saveCfg();
}

void SDKNode::saveCfgRtHandler(const std_msgs::Bool::ConstPtr &msg)
{
	const bool save_data_updated = (msg->data != _save_cfg_rt);
	if (true == save_data_updated)
	{
		_save_cfg_rt = msg->data;
		ROS_DEBUG("%s realtime configuration saving is now %s", getUnqualifiedName().c_str(), BOOL_TO_ENABLED(_save_cfg_rt));

		// If we're enabling RT saving, save the current configuration right now
		if (true == _save_cfg_rt)
		{
			saveCfg();
		}
	}
}

void SDKNode::resetHandler(const nepi_ros_interfaces::Reset::ConstPtr &msg)
{
	const uint8_t reset_type = msg->reset_type;

	switch (reset_type)
	{
	case nepi_ros_interfaces::Reset::USER_RESET:
		ROS_INFO("%s: Executing user-level reset by request", getUnqualifiedName().c_str());
		userReset();
		break;
	case nepi_ros_interfaces::Reset::FACTORY_RESET:
	{
		ROS_INFO("%s: Executing factory reset by request", getUnqualifiedName().c_str());
		factoryReset();
	}
		break;
	case nepi_ros_interfaces::Reset::SOFTWARE_RESET:
		ROS_INFO("%s: Executing software reset by request", getUnqualifiedName().c_str());
		softwareReset();
		break;
	// No implentation for HARDWARE_RESET in this base class
	case nepi_ros_interfaces::Reset::HARDWARE_RESET:
		ROS_INFO("%s: Executing hardware reset by request", getUnqualifiedName().c_str());
		hardwareReset();
	default:
		ROS_WARN("%s: No available hardware reset. Request ignored", getUnqualifiedName().c_str());
		// TODO:
	}
}

void SDKNode::applyThrottleHandler(const std_msgs::Float32::ConstPtr &msg)
{
  setThrottleRatio(msg->data);
}

void SDKNode::submitSysErrorMsg(const std::string &error_msg)
{
	std_msgs::String error_msg_ros;
	error_msg_ros.data = error_msg;
	_submit_sys_err_msg_pub.publish(error_msg_ros);
}

void SDKNode::userReset()
{
	ros::ServiceClient client = n.serviceClient<nepi_ros_interfaces::FileReset>("user_reset");
	nepi_ros_interfaces::FileReset srv;
	srv.request.node_name = getQualifiedName();

	if (false == client.call(srv) || false == srv.response.success)
	{
		ROS_ERROR("%s: User reset request failed", getUnqualifiedName().c_str());
	}
	else
	{
		// Regather params from the param server now that it's been updated by config mgr
		retrieveParams();
	}
}

void SDKNode::factoryReset()
{
	ros::ServiceClient client = n.serviceClient<nepi_ros_interfaces::FileReset>("factory_reset");
	nepi_ros_interfaces::FileReset srv;
	srv.request.node_name = getQualifiedName();
	if (false == client.call(srv) || false == srv.response.success)
	{
		ROS_ERROR("%s: Factory reset request failed", getUnqualifiedName().c_str());
	}
	else
	{
		// Regather params from the param server now that it's been updated by config mgr
		retrieveParams();
	}
}

void SDKNode::softwareReset()
{
	// First, ensure the config file is reloaded on param server before restarting... userReset
	// is the most natural way.
	userReset();

	// Simply shutdown the node... it will be automatically restarted per launch file specification
	ros::shutdown();
}

void SDKNode::hardwareReset()
{
	// This generic version simply does a software reset.
	softwareReset();
}

void SDKNode::saveCfg()
{
	ROS_INFO("%s: Saving current config", getUnqualifiedName().c_str());

	// Inform the config_mgr so it can store the file. We don't do this directly here
	// because ROS has no C++ API for rosparam
	std_msgs::String node_name;
	node_name.data = getQualifiedName(); // config_mgr expects a fully namespaced name
	_store_params_pub.publish(node_name);
}

void SDKNode::setThrottleRatio(float throttle_ratio)
{
  // Bound the inputs to [0.0 - 1.0]
  if (throttle_ratio > 1.0f) throttle_ratio = 1.0f;
  else if (throttle_ratio < 0.0f) throttle_ratio = 0.0f;

  const double rate_spread = max_rate_hz - min_rate_hz;
  current_rate_hz = min_rate_hz + (rate_spread * throttle_ratio);
	//ROS_INFO("%s: Updating current rate to %.2fHz for throttle ratio %.2f", getUnqualifiedName().c_str(), current_rate_hz, throttle_ratio);
}

} // namespace Numurus
