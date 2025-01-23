/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
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
