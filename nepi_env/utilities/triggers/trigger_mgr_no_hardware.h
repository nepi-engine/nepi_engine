/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef _TRIGGER_MGR_NO_FPGA_H
#define _TRIGGER_MGR_NO_FPGA_H

#include "trigger_mgr_base.h"

namespace Numurus
{

class TriggerMgrNoHardware : public TriggerMgrBase
{
	/**
	 * @brief      This method does nothing but warn of unsupported functionality.
	 *
	 * @param[in]  enab_mask  The s/w mask message, which specifies the value to generate on receipt of hardware trigger input
	 */
	virtual void setHwTrigInEnab(const std_msgs::UInt32::ConstPtr& enab_mask) override
	{ 
		ROS_WARN("Unsupported HW function (setHwTrigInEnab)"); 
	}
	
	/**
	 * @brief      This method does nothing but warn of unsupported functionality.
	 *			   
	 * @param[in]  cfg   The configuration message
	 */
	virtual void configureHwTrigIn(const nepi_ros_interfaces::HwTrigInCfg::ConstPtr& cfg) override
	{
		ROS_WARN("Unsupported HW function (configureHwTrigIn)"); 
	}
	
	/**
	 * @brief      Enable the hardware trigger output and set the mask of s/w inputs that can generate a h/w trigger out
	 * 
	 *			   This method does nothing but warn of unsupported functionality.
	 *
	 * @param[in]  enable  The enable mask message
	 */
	virtual void setHwTrigOutEnab(const std_msgs::Bool::ConstPtr& enable) override
	{
		ROS_WARN("Unsupported HW function (setHwTrigOutEnab)"); 
	}

	/**
	 * @brief      This method does nothing but warn of unsupported functionality.
	 *
	 * @param[in]  cfg   The configuration message
	 */
	virtual void configureHwTrigOut(const nepi_ros_interfaces::HwTrigOutCfg::ConstPtr& cfg) override
	{
		ROS_WARN("Unsupported HW function (configureHwTrigOut)"); 
	}

	/**
	 * @brief      This method does nothing but warn of unsupported functionality.
	 *
	 * @param[in]  dly_usecs  The delay message
	 */	
	virtual void setHwTrigOutDly(const std_msgs::UInt32::ConstPtr& dly_usecs) override
	{
		ROS_WARN("Unsupported HW function (setHwTrigOutDly)"); 
	}
};

} // namespace Numurus

#endif //_TRIGGER_MGR_NO_FPGA_H
