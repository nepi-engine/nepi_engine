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
#include <ctime>
#include <unistd.h>
#include <lord_ahrs_driver.h>

#define TTY_DEVICE  "/dev/ttyUSB0"
#define DATA_RATE_HZ   0.2f

int main(int argc, char **argv)
{
  Numurus::LORDAHRSDriver drv;
  const std::time_t init_posix_s = std::time(0);

  Numurus::LORDAHRSRollPitchYaw sensor_to_vehicle_transform; // Use zero-initialized

  if (true == drv.init(TTY_DEVICE, DATA_RATE_HZ, init_posix_s,
                       sensor_to_vehicle_transform))
  {
    printf("Driver initialization successful\n");
  }
  else
  {
    printf("Driver initialization failed\n");
    return -1;
  }

#if 0
  Numurus::LORDAHRSReferencePosition initial_position;
  initial_position.latitude_deg = 36.281537;
  initial_position.longitude_deg = -82.812528;
  initial_position.altitude_m = 391.3632;
  printf("Setting the initial reference position\n");
  drv.updateReferencePosition(initial_position);
  sleep(2);
#endif

  while(1)
  {
    Numurus::AHRSDataSet dummy_set;
    if (false == drv.receiveLatestData(dummy_set))
    {
      printf("Failed to retrieve latest data\n");
      //return -1;
    }

    const std::time_t delta_t = std::time(0) - init_posix_s;
    if (delta_t == 30)
    {
      printf("\n\nApplying Mag AutoCal\n\n");
      drv.magnetometerCaptureAutoCal();
      sleep(2);
    }

    /*
    if (delta_t % 10 == 9)
    {
      printf("\n\nUpdating Ref Position");
      drv.updateReferencePosition(initial_position);
      ++initial_position.longitude_deg;
      sleep(2);
    }
    */
  }

  return 0;
}
