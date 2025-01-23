/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef _TRIGGER_INTERFACE_H
#define _TRIGGER_INTERFACE_H

#include "std_msgs/UInt32.h"
#include "sdk_interface.h"
#include "sdk_node.h"

namespace Numurus
{
/**
 * @brief      Abstract base (interface) class for nodes that can be triggered
 * 
 * 			   This class provides the basic ROS interface for nodes that support
 * 			   hardware triggering. 
 */	
class TriggerInterface : public SDKInterface
{
public:
	/**
	 * @brief      Standard constructor
	 *
	 * @param      parent             Pointer to parent SDKNode
	 * @param      parent_pub_nh      Pointer to parent's public namespace node handle
	 * @param      parent_priv_nh     Pointer to parent's private namespace node handle
	 * @param[in]  parent_trig_index  Parent node's trig index (software or hardware implemented)
	 */
	TriggerInterface(SDKNode *parent, ros::NodeHandle *parent_pub_nh, ros::NodeHandle *parent_priv_nh, 
					 uint32_t parent_trig_index);

	TriggerInterface() = delete; // No default constructor available

	virtual ~TriggerInterface(); 

	// Inherited from SDKInterface
	virtual void retrieveParams() override;
	virtual void initPublishers() override;
	virtual void initSubscribers() override;

	/**
	 * @brief      Enables or disables the triggering.
	 * 
	 * @param[in]	enabled		True to enable triggering, false otherwise
	 */
	virtual inline void setTrigEnabled(bool enabled){_trig_enabled = enabled;}
protected:
	/**
	 * True if trigger is enabled for the parent node, false otherwise
	 */
	bool _trig_enabled;
	
	/**
	 * Fixed trigger index. Specified in constructor.
	 */
	const uint32_t _trig_index;
	
	/**
	 * Param-server-backed trigger delay (in usecs)
	 */
	SDKNode::NodeParam<int> _trig_delay;

	/**
	 * Publisher for the TriggerIndexInfo topic
	 */
	ros::Publisher _trig_index_pub;

	/**
	 * @brief      Sets the input trigger delay value (in usecs).
	 * 
	 * 			   This method serves as the callback for the (private namespace) set_trig_delay topic
	 *
	 * @param[in]  trig_delay_val_usecs  The trigger delay value to set
	 */	
	virtual void setTrigDelay(const std_msgs::UInt32::ConstPtr& trig_delay_val_usecs);

};
} // namespace Numurus
#endif //_TRIGGER_INTERFACE_H