/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef _TRIGGER_MGR_BASE_H
#define _TRIGGER_MGR_BASE_H

#include <string>
#include <unordered_map>
#include <mutex>

#include "ros/console.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Bool.h"

#include "nepi_ros_interfaces/HwTrigInCfg.h"
#include "nepi_ros_interfaces/HwTrigOutCfg.h"
#include "nepi_ros_interfaces/PeriodicSwTrig.h"
#include "nepi_ros_interfaces/TriggerIndexSettings.h"
#include "nepi_ros_interfaces/TriggerStatusQuery.h"
#include "nepi_ros_interfaces/TriggerDefs.h"
#include "sdk_node.h"

namespace Numurus
{

/**
 * @brief      ROS node interface to the FPGA-agnostic trigger manager base module
 * 
 * 			   This class provides a base for the configuration and control of the non-FPGA trigger manager functionality.
 * 			   It was separated from the FPGA-dependent trigger manager functionality to support running a subset of the 
 * 			   full trigger interface on an FPGA-free processor, specifically the Jetson TX2 in the 3DSC (for an initial
 * 			   ship where Zynq network interface was broken hence unsuitable to serve in the ROS subsystem).
 */	
class TriggerMgrBase : public SDKNode
{
public:
	TriggerMgrBase();
	~TriggerMgrBase();

protected:
	// Overrides from SDKNode
	virtual void initSubscribers() override;
	virtual void initPublishers() override;
	virtual void initServices() override;
	virtual void retrieveParams() override;

	virtual void executeSwTrig(const std_msgs::UInt32::ConstPtr& trig_mask);
	// Pure virtual h/w capability functions
	virtual void setHwTrigInEnab(const std_msgs::UInt32::ConstPtr& enab_mask) = 0;
	virtual void configureHwTrigIn(const nepi_ros_interfaces::HwTrigInCfg::ConstPtr& cfg) = 0;
	virtual void setHwTrigOutEnab(const std_msgs::Bool::ConstPtr& enable) = 0;
	virtual void configureHwTrigOut(const nepi_ros_interfaces::HwTrigOutCfg::ConstPtr& cfg) = 0;
	virtual void setHwTrigOutDly(const std_msgs::UInt32::ConstPtr& dly_usecs) = 0;

private:
	/**
	 * Map of current periodic triggers <mask, rate>
	 */
	std::unordered_map<uint32_t, float> periodic_trig_map;

	/**
	 * Map of most recent trigger execution times for each trig mask
	 */
	std::unordered_map<uint32_t, ros::Time> last_trig_time_map;
	
	/**
	 * Mutex to protect periodic_trig_map, which is accessed by main and all periodic worker threads
	 */
	std::mutex trig_map_mutex;

	/**
	 * Handle to the sw_trig publisher called by periodic tasks
	 */
	ros::Publisher _sw_trig_pub;

	/**
	 * Map of the active trigger indices (first=index, second=trigger interface name)
	 */
	std::map<uint32_t, std::string> trig_indices;

	/**
	 * Configurable param for a default periodic trigger mask to execute on start-up (in concert)
	 */
	SDKNode::NodeParam<int> default_periodic_trig_mask_;

	/**
	 * Configurable param for the rate for the default_trig_mask_
	 */
	SDKNode::NodeParam<float> default_periodic_trig_rate_;

	/**
	 * @brief      Sets a periodic software trig. configuration
	 * 
	 * 			   This method serves as a callback for the set_periodic_sw_trig topic
	 * 			   It starts or stops a periodic trigger.
	 *
	 * @param[in]  trig_cfg  The trig configuration message
	 */
	void setPeriodicSwTrig(const nepi_ros_interfaces::PeriodicSwTrig::ConstPtr& trig_cfg);

	void setPeriodicSwTrigImpl(bool enabled, uint32_t sw_trig_mask, float rate_hz);

	/**
	 * @brief      Worker method for periodic trigger threads
	 *
	 * @param[in]  trig_mask  The trig mask value for this thread
	 */
	void runPeriodicTrig(uint32_t trig_mask);

	/**
	 * @brief      Adds an entry to the trigger index set
	 * 
	 *			   This method serves as a callback for the trigger_index_settings message received from triggerable nodes 
	 *
	 * @param[in]  trig_idx_settings  The trig index settings message
	 */
	void updateTriggerIndexSet(const nepi_ros_interfaces::TriggerIndexSettings::ConstPtr& trig_idx_settings);

	/**
	 * @brief      Provide the trigger status
	 * 
	 * 			   This method serves as a callback for the trigger_status_query service.
	 *	
	 * @param      req   The request, includes the specific trigger value for the request
	 * @param      resp  The response
	 *
	 * @return     true if successful, false otherwise
	 */
	bool provideTriggerStatus(nepi_ros_interfaces::TriggerStatusQuery::Request &req, nepi_ros_interfaces::TriggerStatusQuery::Response &resp);

	/**
	 * @brief      Provide the trigger defs
	 * 
	 *             This method serves as a callback for the trigger_defs service
	 *
	 * @param      req   The request (empty)
	 * @param      resp  The response, includes a fixed array of trigger indices
	 *
	 * @return     true
	 */
	bool provideTriggerDefs(nepi_ros_interfaces::TriggerDefs::Request &req, nepi_ros_interfaces::TriggerDefs::Response &resp);

};

} // namespace Numurus

#endif //_TRIGGER_MGR_H