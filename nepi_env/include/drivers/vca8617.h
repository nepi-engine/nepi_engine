/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef __VCA8617_H
#define __VCA8617_H

// Linux spidev defaults to MSB first on the wire. That can be changed by ioctl(SPI_IOC_RD_LSB_FIRST),
// but instead, we just organize the data words that way
const int VCA8617_START_BIT = 0x1 << 7;
const int VCA8617_READ_BIT = 0x0 << 6;
const int VCA8617_WRITE_BIT = 0x1 << 6;

enum VCA8617_PWR
{
  VCA8617_PWR_ENABLED   = 0x0 << 5,
  VCA8617_PWR_DISABLED  = 0x1 << 5
};

enum VCA8617_ATTEN
{
  VCA8617_ATTEN_29dB    = 0x0 << 3,
  VCA8617_ATTEN_33dB    = 0x2 << 3,
  VCA8617_ATTEN_36p5dB  = 0x1 << 3,
  VCA8617_ATTEN_40dB    = 0x3 << 3
};

enum VCA8617_MODE
{
  VCA8617_MODE_TGC      = 0x1 << 2,
  VCA8617_MODE_DOPPLER  = 0x0 << 2
};

enum VCA8617_GAIN
{
  VCA8617_GAIN_25dB     = 0x0 << 0,
  VCA8617_GAIN_30dB     = 0x2 << 0,
  VCA8617_GAIN_35dB     = 0x1 << 0,
  VCA8617_GAIN_40dB     = 0x3 << 0
};

constexpr uint32_t VCA8617_PWR_UP_DELAY_US = 100;

// TODO: CW Coding enums and values in the struct if ever necessary

struct VCA8617_Setup
{
  VCA8617_PWR pwr_state = VCA8617_PWR_ENABLED;
  VCA8617_ATTEN attenuation = VCA8617_ATTEN_40dB;
  VCA8617_MODE mode = VCA8617_MODE_TGC;
  VCA8617_GAIN gain = VCA8617_GAIN_40dB;
};

int vca8617_setup(const char *spidev_device_path, const VCA8617_Setup &setup);
//int vca8617_read_setup(const char *spidev_device_path, VCA8617_Setup &setup_out);

#endif
