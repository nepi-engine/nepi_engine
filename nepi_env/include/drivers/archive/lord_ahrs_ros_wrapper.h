/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
 */
#ifndef __LORD_AHRS_ROS_WRAPPER_H
#define __LORD_AHRS_ROS_WRAPPER_H

#include "sdk_node.h"
#include "lord_ahrs_driver.h"

namespace Numurus
{
class LORDAHRSRosWrapper : public SDKNode
{
public:
  LORDAHRSRosWrapper();
  virtual ~LORDAHRSRosWrapper();

  // SDK Node Overrides
  void init() override;
  void run() override;
  void retrieveParams() override;
  void initPublishers() override;

private:
  NodeParam<std::string> ahrs_serial_port;
  NodeParam<int> publish_rate_hz;
  NodeParam<float> ahrs_x_rot_offset_deg;
  NodeParam<float> ahrs_y_rot_offset_deg;
  NodeParam<float> ahrs_z_rot_offset_deg;

  ros::Publisher imu_pub;
  ros::Publisher odom_pub;
  ros::Publisher heading_pub;

  LORDAHRSDriver driver;

  void publishIMUandOdom();
};

} //namespace Numurus
#endif
