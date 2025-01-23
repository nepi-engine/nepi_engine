/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef __AD9249_ADC_H
#define __AD9249_ADC_H

#include "stdint.h"

namespace Numurus{

enum AD9249_REGISTER_ADDR
{
  AD9249_REGISTER_ADDR_CHIP_ID        = 0x001,
  AD9249_REGISTER_ADDR_DEVICE_INDEX_2 = 0x004,
  AD9249_REGISTER_ADDR_DEVICE_INDEX_1 = 0x005,
  AD9249_REGISTER_ADDR_POWER_MODES    = 0x008,
  AD9249_REGISTER_ADDR_CLOCK_DIVIDE   = 0x00B,
  AD9249_REGISTER_ADDR_TEST_MODE      = 0x00D,
  AD9249_REGISTER_ADDR_USER_PAT_1_LSB = 0x019,
  AD9249_REGISTER_ADDR_USER_PAT_1_MSB = 0x01A,
  AD9249_REGISTER_ADDR_USER_PAT_2_LSB = 0x01B,
  AD9249_REGISTER_ADDR_USER_PAT_2_MSB = 0x01C,
  AD9249_REGISTER_ADDR_SYNC           = 0x109
};

constexpr uint8_t AD9249_DATA_SIZE_1 = 0x0 << 5; // This is the only data size we use

constexpr uint16_t AD9249_CHANNEL_MASK_E = 0x1 << 0;
constexpr uint16_t AD9249_CHANNEL_MASK_F = 0x1 << 1;
constexpr uint16_t AD9249_CHANNEL_MASK_G = 0x1 << 2;
constexpr uint16_t AD9249_CHANNEL_MASK_H = 0x1 << 3;
constexpr uint16_t AD9249_CHANNEL_MASK_A = 0x1 << 4;
constexpr uint16_t AD9249_CHANNEL_MASK_B = 0x1 << 5;
constexpr uint16_t AD9249_CHANNEL_MASK_C = 0x1 << 6;
constexpr uint16_t AD9249_CHANNEL_MASK_D = 0x1 << 7;
constexpr uint16_t AD9249_CHANNEL_MASK_FCO = 0x1 << 8;
constexpr uint16_t AD9249_CHANNEL_MASK_DCO = 0x1 << 9;
constexpr uint16_t AD9249_CHANNEL_MASK_ALL = 0x3FF;

enum AD9249_POWER_DOWN_MODE_INTERNAL
{
  AD9249_POWER_DOWN_MODE_INTERNAL_CHIP_RUN = 0x0,
  AD9249_POWER_DOWN_MODE_INTERNAL_FULL     = 0x1,
  AD9249_POWER_DOWN_MODE_INTERNAL_STANDBY  = 0x2,
  AD9249_POWER_DOWN_MODE_INTERNAL_RESET    = 0x3
};
constexpr uint8_t AD9249_POWER_DOWN_MODE_INTERNAL_MASK = 0x3;

enum AD9249_POWER_DOWN_MODE_EXTERNAL
{
  AD9249_POWER_DOWN_MODE_EXTERNAL_FULL    = 0x0 << 5,
  AD9249_POWER_DOWN_MODE_EXTERNAL_STANDBY = 0x1 << 5
};
constexpr uint8_t AD9249_POWER_DOWN_MODE_EXTERNAL_MASK = 0x20;

enum AD9249_CLOCK_DIVIDE
{
  AD9249_CLOCK_DIVIDE_1 = 0x0,
  AD9249_CLOCK_DIVIDE_2 = 0x1,
  AD9249_CLOCK_DIVIDE_3 = 0x2,
  AD9249_CLOCK_DIVIDE_4 = 0x3,
  AD9249_CLOCK_DIVIDE_5 = 0x4,
  AD9249_CLOCK_DIVIDE_6 = 0x5,
  AD9249_CLOCK_DIVIDE_7 = 0x6,
  AD9249_CLOCK_DIVIDE_8 = 0x7
};

enum AD9249_TEST_MODE
{
  AD9249_TEST_MODE_OFF            = 0x0 << 0,
  AD9249_TEST_MODE_CHECKERBOARD   = 0x4 << 0,
  AD9249_TEST_MODE_PN_LONG        = 0x5 << 0,
  AD9249_TEST_MODE_PN_SHORT       = 0x6 << 0,
  AD9249_TEST_MODE_USER_INPUT     = 0x8 << 0
};

constexpr uint8_t AD9249_TEST_MODE_RESET_PN_LONG_GEN    = 0x1 << 5;
constexpr uint8_t AD9249_TEST_MODE_RESET_PN_SHORT_GEN   = 0x1 << 4;

constexpr uint8_t AD9249_TEST_MODE_USER_MODE_ALTERNATE  = 0x1 << 6;

constexpr uint8_t AD9249_SYNC_ENABLED = 0x1;
constexpr uint8_t AD9249_SYNC_DISABLED = 0x0;

constexpr uint32_t AD9249_FULL_PWR_UP_DELAY_US = 375;

class Ad9249_Adc
{
public:
  Ad9249_Adc();
  ~Ad9249_Adc();

  bool init(const char* spidev_device_path);

  bool getPowerDownMode(AD9249_POWER_DOWN_MODE_INTERNAL &internal_mode_out, AD9249_POWER_DOWN_MODE_EXTERNAL &external_mode_out);
  bool setPowerDownMode(AD9249_POWER_DOWN_MODE_INTERNAL internal_mode, AD9249_POWER_DOWN_MODE_EXTERNAL external_mode);

  bool getClockDivide(AD9249_CLOCK_DIVIDE &clock_divide_out);
  bool setClockDivide(AD9249_CLOCK_DIVIDE clock_divide);

  bool getTestMode(AD9249_TEST_MODE &mode_out, uint16_t channel_mask = AD9249_CHANNEL_MASK_ALL);
  bool setTestMode(AD9249_TEST_MODE mode, uint16_t channel_mask = AD9249_CHANNEL_MASK_ALL);

  bool getUserTestPattern(uint16_t &user_pattern_1_out, uint16_t &user_pattern_2_out);
  bool setUserTestPattern(uint16_t user_pattern_1, uint16_t user_pattern_2);

  bool getSyncMode(bool &sync_enabled);
  bool setSyncMode(bool enable_sync);

private:
  bool read(AD9249_REGISTER_ADDR addr, uint8_t &data_out, uint16_t device_bitmask = AD9249_CHANNEL_MASK_ALL);
  bool write(AD9249_REGISTER_ADDR addr, uint8_t data, uint16_t device_bitmask = AD9249_CHANNEL_MASK_ALL);
  bool setDeviceBitmaskIfNecessary(uint16_t device_bitmask);

  bool checkChipId();
  bool getWriteDeviceIndex(uint16_t &write_index_out);
  int fd = -1;
  uint16_t write_device_index = 0x3FF; // All devices as specified by register defaults for DEVICE_INDEX_1

  static constexpr uint8_t AD9249_READ_BIT = 0x1 << 7;
  static constexpr uint8_t AD9249_WRITE_BIT = 0x0 << 7;
  static constexpr uint8_t AD9249_CHIP_ID = 0x92;
};

} // namespace Numurus

#endif //__AD9249_ADC_H
