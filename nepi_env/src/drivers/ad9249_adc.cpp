/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <cstdlib>

#include "drivers/ad9249_adc.h"

namespace Numurus
{

#ifdef AD9249_TEST_APP
  #define PRINTF_TEST_APP(...) printf(__VA_ARGS__)
#else
  void PRINTF_TEST_APP(const char* fmt...){} // Do nothing
#endif

// Enable the following to readback after every write to make sure the setting took -- as of 12/31/21 the ADC SPI is
// not very reliable when coming up from a cold boot.
#define VERIFY_ALL_WRITES

static const char* EXTERNAL_POWER_MODE_AS_STRING(uint8_t mode)
{
  if (mode == AD9249_POWER_DOWN_MODE_EXTERNAL_FULL) return "Full Power Down";
  if (mode == AD9249_POWER_DOWN_MODE_EXTERNAL_STANDBY) return "Standby";
  return "Unknown";
}

static const char* INTERNAL_POWER_MODE_AS_STRING(uint8_t mode)
{
  if (mode == AD9249_POWER_DOWN_MODE_INTERNAL_CHIP_RUN) return "Power On";
  if (mode == AD9249_POWER_DOWN_MODE_INTERNAL_FULL) return "Full Power Down";
  if (mode == AD9249_POWER_DOWN_MODE_INTERNAL_STANDBY) return "Standby";
  if (mode == AD9249_POWER_DOWN_MODE_INTERNAL_RESET) return "Reset";
  return "Unknown";
}

static const char* TEST_MODE_AS_STRING(uint8_t mode)
{
  if (mode == AD9249_TEST_MODE_OFF) return "Off";
  if (mode == AD9249_TEST_MODE_CHECKERBOARD) return "Checkerboard";
  if (mode == AD9249_TEST_MODE_PN_LONG) return "Pseudo-noise Long";
  if (mode == AD9249_TEST_MODE_PN_SHORT) return "Pseudo-noise Short";
  if (mode == AD9249_TEST_MODE_USER_INPUT) return "User Input";
  return "Unknown";
}

Ad9249_Adc::Ad9249_Adc()
{
}

Ad9249_Adc::~Ad9249_Adc()
{
  close(fd);
}

bool Ad9249_Adc::init(const char* spidev_device_path)
{
  PRINTF_TEST_APP("--- Beginning AD9249 Initialization ---\n");
  fd = open(spidev_device_path, O_RDWR);
  if (fd < 0)
  {
    PRINTF_TEST_APP("Failed to open spi device %s (%s)\n", spidev_device_path, strerror(errno));
    return -1;
  }

  // Check the Chip ID to ensure that the device is up and running
  if (false == checkChipId())
  {
    PRINTF_TEST_APP("Failed to match the Chip ID\n");
    return false;
  }

  if (false == getWriteDeviceIndex(write_device_index))
  {
    PRINTF_TEST_APP("Failed to get the current device index\n");
    return false;
  }

  PRINTF_TEST_APP("--- AD9249 Initialization Complete ---\n");
  return true;
}

bool Ad9249_Adc::getPowerDownMode(AD9249_POWER_DOWN_MODE_INTERNAL &internal_mode_out, AD9249_POWER_DOWN_MODE_EXTERNAL &external_mode_out)
{
  uint8_t power_down_val;
  if (false == read(AD9249_REGISTER_ADDR_POWER_MODES, power_down_val))
  {
    return false;
  }
  internal_mode_out = static_cast<AD9249_POWER_DOWN_MODE_INTERNAL>(power_down_val & static_cast<uint8_t>(AD9249_POWER_DOWN_MODE_INTERNAL_MASK));
  external_mode_out = static_cast<AD9249_POWER_DOWN_MODE_EXTERNAL>(power_down_val & static_cast<uint8_t>(AD9249_POWER_DOWN_MODE_EXTERNAL_MASK));

  PRINTF_TEST_APP("Power Mode: External = %s, Internal = %s\n",
                  EXTERNAL_POWER_MODE_AS_STRING(external_mode_out), INTERNAL_POWER_MODE_AS_STRING(internal_mode_out));

  return true;
}

bool Ad9249_Adc::setPowerDownMode(AD9249_POWER_DOWN_MODE_INTERNAL internal_mode, AD9249_POWER_DOWN_MODE_EXTERNAL external_mode)
{
  const uint8_t power_down_val = internal_mode | external_mode;
  PRINTF_TEST_APP("Writing Power Mode to External = %s, Internal = %s\n",
         EXTERNAL_POWER_MODE_AS_STRING(external_mode),
         INTERNAL_POWER_MODE_AS_STRING(internal_mode));
  return write(AD9249_REGISTER_ADDR_POWER_MODES, power_down_val);
}

bool Ad9249_Adc::getClockDivide(AD9249_CLOCK_DIVIDE &clock_divide_out)
{
  uint8_t clock_divide_val;
  if (false == read(AD9249_REGISTER_ADDR_CLOCK_DIVIDE, clock_divide_val))
  {
    return false;
  }

  clock_divide_out = static_cast<AD9249_CLOCK_DIVIDE>(clock_divide_val + 1);
  PRINTF_TEST_APP("Clock Divide: %u\n", (uint32_t)(clock_divide_val + 1));
  return true;
}

bool Ad9249_Adc::setClockDivide(AD9249_CLOCK_DIVIDE clock_divide)
{
  const uint8_t clock_divide_val = static_cast<uint8_t>(clock_divide);
  PRINTF_TEST_APP("Writing Clock Divide to %d\n", clock_divide_val + 1);
  return write(AD9249_REGISTER_ADDR_CLOCK_DIVIDE, clock_divide_val);
}

bool Ad9249_Adc::getTestMode(AD9249_TEST_MODE &mode_out, uint16_t channel_mask)
{
  uint8_t mode_val;
  if (false == read(AD9249_REGISTER_ADDR_TEST_MODE, mode_val, channel_mask))
  {
    return false;
  }

  mode_out = static_cast<AD9249_TEST_MODE>(mode_val & 0xF);
  PRINTF_TEST_APP("Test Mode: %s\n", TEST_MODE_AS_STRING(mode_out));
  return true;
}

bool Ad9249_Adc::setTestMode(AD9249_TEST_MODE mode, uint16_t channel_mask)
{

  const uint8_t test_mode_val = mode |
    // Always set the PN reset bits -- no harm in doing this, and it users an easy way to execute this
    AD9249_TEST_MODE_RESET_PN_LONG_GEN | AD9249_TEST_MODE_RESET_PN_SHORT_GEN |
    // For now we only use the alternating mode, where we send PATT1, then PATT2, then repeat
    AD9249_TEST_MODE_USER_MODE_ALTERNATE;
  PRINTF_TEST_APP("Writing Test Mode to %s with device bitmask 0x%x\n", TEST_MODE_AS_STRING(mode), channel_mask);
  return write(AD9249_REGISTER_ADDR_TEST_MODE, test_mode_val, channel_mask);
}

bool Ad9249_Adc::getUserTestPattern(uint16_t &user_pattern_1_out, uint16_t &user_pattern_2_out)
{
  uint8_t user_test_pattern_1_lsb;
  uint8_t user_test_pattern_1_msb;

  uint8_t user_test_pattern_2_lsb;
  uint8_t user_test_pattern_2_msb;

  if ((false == read(AD9249_REGISTER_ADDR_USER_PAT_1_LSB, user_test_pattern_1_lsb)) ||
      (false == read(AD9249_REGISTER_ADDR_USER_PAT_1_MSB, user_test_pattern_1_msb)) ||
      (false == read(AD9249_REGISTER_ADDR_USER_PAT_2_LSB, user_test_pattern_2_lsb)) ||
      (false == read(AD9249_REGISTER_ADDR_USER_PAT_2_MSB, user_test_pattern_2_msb)))
  {
    return false;
  }
  user_pattern_1_out = (uint16_t)user_test_pattern_1_lsb | ((uint16_t)(user_test_pattern_1_msb) << 8);
  user_pattern_2_out = (uint16_t)user_test_pattern_2_lsb | ((uint16_t)(user_test_pattern_2_msb) << 8);
  PRINTF_TEST_APP("User Test Pattern: Word 1 = 0x%04x, Word 2 = 0x%04x\n", user_pattern_1_out, user_pattern_2_out);
  return true;
}

bool Ad9249_Adc::setUserTestPattern(uint16_t user_pattern_1, uint16_t user_pattern_2)
{
  user_pattern_1 &= 0x3FFF; // 14 bits
  user_pattern_2 &= 0x3FFF; // 14 bits
  const uint8_t user_pattern_1_lsb = user_pattern_1 & 0xFF;
  const uint8_t user_pattern_1_msb = (user_pattern_1 & 0xFF00) >> 8;
  const uint8_t user_pattern_2_lsb = user_pattern_2 & 0xFF;
  const uint8_t user_pattern_2_msb = (user_pattern_2 & 0xFF00) >> 8;

  PRINTF_TEST_APP("Setting User Test Pattern to Word 1 = 0x%04x, Word 2 = 0x%04x\n", user_pattern_1, user_pattern_2);

  if ((false == write(AD9249_REGISTER_ADDR_USER_PAT_1_LSB, user_pattern_1_lsb)) ||
      (false == write(AD9249_REGISTER_ADDR_USER_PAT_1_MSB, user_pattern_1_msb)) ||
      (false == write(AD9249_REGISTER_ADDR_USER_PAT_2_LSB, user_pattern_2_lsb)) ||
      (false == write(AD9249_REGISTER_ADDR_USER_PAT_2_MSB, user_pattern_2_msb)))
  {
    return false;
  }
  return true;
}

bool Ad9249_Adc::getSyncMode(bool &sync_enabled)
{
  uint8_t sync_mode_val;
  if (false == read(AD9249_REGISTER_ADDR_SYNC, sync_mode_val))
  {
    return false;
  }

  sync_enabled = (sync_mode_val & AD9249_SYNC_ENABLED);
  PRINTF_TEST_APP("Sync: %s\n", sync_enabled? "Enabled" : "Disabled");
  return true;
}

bool Ad9249_Adc::setSyncMode(bool enable_sync)
{
  // This sets sync such that EVERY sync signal after this call will resync. It is also possible to set bits in
  // this register such that only the next sync signal has any impact, but for now we don't expose that functionality
  const uint8_t sync_mode_val = (enable_sync)? AD9249_SYNC_ENABLED : AD9249_SYNC_DISABLED;
  PRINTF_TEST_APP("Writing Sync Mode to %s\n", enable_sync? "Enabled" : "Disabled");
  return write(AD9249_REGISTER_ADDR_SYNC, sync_mode_val);
}

bool Ad9249_Adc::read(AD9249_REGISTER_ADDR addr, uint8_t &data_out, uint16_t device_bitmask)
{
  if (false == setDeviceBitmaskIfNecessary(device_bitmask))
  {
    return false;
  }

  struct spi_ioc_transfer xfer;
  memset(&xfer, 0, sizeof(xfer));

  uint8_t instruction_buf[3]; // 2 bytes instruction, 1 byte response
  memset(instruction_buf, 0, sizeof(instruction_buf));
  uint8_t data_buf[3]; // 2 bytes instruction, 1 byte response
  memset(data_buf, 0, sizeof(data_buf));

  const uint8_t addr_high = (uint8_t)((addr & 0x1f00) >> 8); // Upper 5 bits
  const uint8_t addr_low = (uint8_t)((addr & 0xff)); // Lower 8 bits
  instruction_buf[0] = AD9249_READ_BIT | AD9249_DATA_SIZE_1 | addr_high;
  instruction_buf[1] = addr_low;

  PRINTF_TEST_APP("   (ADC Read: Writing instruction 0x%02x%02x to spi bus)\n", instruction_buf[0], instruction_buf[1]);

  // All other control bytes can remain zero, since CW mode isn't implemented here anyway
  xfer.tx_buf = (uint64_t)instruction_buf;
  xfer.rx_buf = (uint64_t)data_buf;
  xfer.len = 3;

  const int status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
  if (status < 0)
  {
    return false;
  }
  data_out = data_buf[2]; // First two bytes are blank during instruction phase
  PRINTF_TEST_APP("   (ADC Read: Read 0x%02x%02x%02x from spi bus)\n", data_buf[0], data_buf[1], data_buf[2]);
  return true;
}

bool Ad9249_Adc::write(AD9249_REGISTER_ADDR addr, uint8_t data, uint16_t device_bitmask)
{
  if (false == setDeviceBitmaskIfNecessary(device_bitmask))
  {
    return false;
  }

  // Now execute the write
  struct spi_ioc_transfer xfer;
  memset(&xfer, 0, sizeof(xfer));

  uint8_t write_buf[3]; // 2 bytes instruction, 1 byte data
  memset(write_buf, 0, sizeof(write_buf));
  const uint8_t addr_high = (uint8_t)((addr & 0x1f00) >> 8); // Upper 5 bits
  const uint8_t addr_low = (uint8_t)((addr & 0xff)); // Lower 8 bits
  write_buf[0] = AD9249_WRITE_BIT | AD9249_DATA_SIZE_1 | addr_high;
  write_buf[1] = addr_low;
  write_buf[2] = data;

  PRINTF_TEST_APP("   (ADC Write: Writing instruction+data 0x%02x%02x%02x to spi bus)\n", write_buf[0], write_buf[1], write_buf[2]);

  xfer.tx_buf = (uint64_t)write_buf;
  xfer.len = 3;

  const int status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
  if (status < 0)
  {
    return false;
  }

#ifdef VERIFY_ALL_WRITES
  uint8_t data_readback;
  if ((false == read(addr, data_readback, device_bitmask)) || (data_readback != data))
  {
    return false;
  }
#endif

  return true;
}

bool Ad9249_Adc::setDeviceBitmaskIfNecessary(uint16_t device_bitmask)
{
  // First, must determine if we need to set up the bitmask so we only write to specified devices
  const uint8_t device_bitmask_1 = (uint8_t)((device_bitmask & 0x3F0) >> 4);
  const uint8_t device_bitmask_2 = (uint8_t)(device_bitmask & 0xF);
  if (device_bitmask_1 != ((write_device_index & 0x3F0) >> 4))
  {
    // Write the index 1 bitbask
    PRINTF_TEST_APP("Updating Index 1 Device Bitmask from 0x%x to 0x%x\n", (write_device_index & 0x3F0) >> 4, device_bitmask_1);
    if (false == write(AD9249_REGISTER_ADDR_DEVICE_INDEX_1, device_bitmask_1, write_device_index)) // Make sure to avoid infinite recurse
    {
      PRINTF_TEST_APP("Error -- could not set device index 1 for write to 0x%x\n", device_bitmask_1);
      return false;
    }
    write_device_index |= (device_bitmask_1 << 4);
  }
  if (device_bitmask_2 != (write_device_index & 0xF))
  {
    // Write the index 2 bitmask
    PRINTF_TEST_APP("Updating Index 2 Device Bitmask from 0x%x to 0x%x\n", (write_device_index & 0xF), device_bitmask_2);
    if (false == write(AD9249_REGISTER_ADDR_DEVICE_INDEX_2, device_bitmask_2, write_device_index)) // Make sure to avoid infinite recurse
    {
      PRINTF_TEST_APP("Error -- could not set device index 2 for write to 0x%x\n", device_bitmask_2);
      return false;
    }
    write_device_index |= device_bitmask_2;
  }
  return true;
}

bool Ad9249_Adc::checkChipId()
{
  uint8_t chip_id;
  PRINTF_TEST_APP("Reading Chip ID\n");
  if (false == read(AD9249_REGISTER_ADDR_CHIP_ID, chip_id))
  {
    return false;
  }
  if (chip_id != AD9249_CHIP_ID)
  {
    PRINTF_TEST_APP("Got unexpected chip id 0x%x\n", chip_id);
    return false;
  }
  PRINTF_TEST_APP("AD9249 responds with proper chip id 0x%x... ready for configuration\n", chip_id);
  return true;
}

bool Ad9249_Adc::getWriteDeviceIndex(uint16_t &write_index_out)
{
  uint8_t write_device_index_1;
  PRINTF_TEST_APP("Reading Device Index 1\n");
  if (false == read(AD9249_REGISTER_ADDR_DEVICE_INDEX_1, write_device_index_1))
  {
    return false;
  }
  write_device_index_1 &= 0x3f;
  uint8_t write_device_index_2;
  PRINTF_TEST_APP("Reading Device Index 2\n");
  if (false == read(AD9249_REGISTER_ADDR_DEVICE_INDEX_2, write_device_index_2))
  {
    return false;
  }
  write_device_index_2 &= 0xf;
  write_index_out = (static_cast<uint16_t>(write_device_index_1) << 4) | static_cast<uint16_t>(write_device_index_2);
  PRINTF_TEST_APP("Current Device Index is 0x%03x\n", write_index_out);
  return true;
}

} //namespace Numurus

void printUsage()
{
  printf("\nUsage:\n");
  printf("  ad9249_test_app <spidev_device_path> <config_id> <config_args...>\n");
  printf("  where <config_id> is one of:\n");
  printf("    -p (power settings)\n");
  printf("       args: <ext_full | ext_stdby> <int_on | int_off | int_stdby | int_rst>\n");
  printf("    -c (clock divisor)\n");
  printf("       args: <divisor value (1...8)>\n");
  printf("    -s (sync mode)\n");
  printf("       args: <enable|disable>\n");
  printf("    -t (test mode)\n");
  printf("       args: <off|checkerboard|pn_short|pn_long|user_pat> <device_mask>\n");
  printf("         where device_mask is a bitmask specifying which devices\n");
  printf("         config applies to (bit order:MSB = ChD, ChC, ChB, ChA, ChH, ChG, ChF, ChE = LSB)\n");
  printf("         This arg can be blank, in which case case all channels get configured identically\n");
  printf("         (as if mask was 0xff)\n");
  printf("    -u (user pattern setup for test mode \"user_pat\")\n");
  printf("       args: <word 1> <word 2>\n");
  printf("         where each word is a 14-bit value and the output pattern will alternate between the two\n");
  printf("\n  If no config_args are specified, the device will be read and that current configuration reported\n\n");
}

#ifdef AD9249_TEST_APP
using namespace Numurus;
int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printUsage();
    return -1;
  }
  const char* spi_device_path = argv[1];
  Ad9249_Adc ad9249;
  if (false == ad9249.init(spi_device_path))
  {
    printf("Error: Failed to contact/initialize chip. Did you set the correct spidev_device_path?\n");
    return -1;
  }

  if (argc == 2)
  {
    printf("No configuration specified... all done\n");
    return 0;
  }

  if (argv[2][0] != '-')
  {
    printf("Error: Invalid configuration specified: %s\n", argv[2]);
    printUsage();
    return -1;
  }

  const char config_id = argv[2][1];
  const bool read_only = (argc == 3);

  switch (config_id)
  {
    case 'p':
    {
      AD9249_POWER_DOWN_MODE_INTERNAL internal;
      AD9249_POWER_DOWN_MODE_EXTERNAL external;
      if (read_only)
      {
        printf("Reading Power Mode\n");
        ad9249.getPowerDownMode(internal, external);
      }
      else
      {
        if (argc != 5)
        {
          printf("Error: Invalid arg. count for power configuration\n");
          printUsage();
          return -1;
        }

        if (0 == strcmp(argv[3], "ext_full")) external = AD9249_POWER_DOWN_MODE_EXTERNAL_FULL;
        else if (0 == strcmp(argv[3], "ext_stdby")) external = AD9249_POWER_DOWN_MODE_EXTERNAL_STANDBY;
        else
        {
          printf("Error: %s not a valid value for external power mode\n", argv[4]);
          printUsage();
          return -1;
        }
        if (0 == strcmp(argv[4], "int_on")) internal = AD9249_POWER_DOWN_MODE_INTERNAL_CHIP_RUN;
        else if (0 == strcmp(argv[4], "int_off")) internal = AD9249_POWER_DOWN_MODE_INTERNAL_FULL;
        else if (0 == strcmp(argv[4], "int_stdby")) internal = AD9249_POWER_DOWN_MODE_INTERNAL_STANDBY;
        else if (0 == strcmp(argv[4], "int_rst")) internal = AD9249_POWER_DOWN_MODE_INTERNAL_RESET;
        else
        {
          printf("Error: \"%s\" not a valid value for internal power mode\n", argv[4]);
          printUsage();
          return -1;
        }
        ad9249.setPowerDownMode(internal, external);
      }
    } break;
    case ('c'):
    {
      AD9249_CLOCK_DIVIDE clock_divide;
      if (read_only)
      {
        printf("Reading Clock Divide\n");
        ad9249.getClockDivide(clock_divide);
      }
      else
      {
        if (argc != 4)
        {
          printf("Error: Invalid arg. count for power configuration\n");
          printUsage();
          return -1;
        }

        const int clock_divisor_val = strtol(argv[3], nullptr, 0);
        if (clock_divisor_val < 1 || clock_divisor_val > 8)
        {
          printf("Error: Invalid clock divisor \"%s\"\n", argv[3]);
          printUsage();
          return -1;
        }
        clock_divide = static_cast<AD9249_CLOCK_DIVIDE>(clock_divisor_val - 1);
        ad9249.setClockDivide(clock_divide);
      }
    } break;
    case ('s'):
    {
      bool sync_enable;
      if (read_only)
      {
        printf("Reading Sync Mode\n");
        ad9249.getSyncMode(sync_enable);
      }
      else
      {
        if (argc != 4)
        {
          printf("Error: Invalid arg. count for sync configuration\n");
          printUsage();
          return -1;
        }
        if (0 == strcmp(argv[3], "enable")) sync_enable = true;
        else if (0 == strcmp(argv[3], "disable")) sync_enable = false;
        else
        {
          printf("Error: \"%s\" not a valid value for sync mode\n", argv[3]);
          printUsage();
          return -1;
        }
        ad9249.setSyncMode(sync_enable);
      }
    } break;
    case 't':
    {
      AD9249_TEST_MODE mode;
      if (read_only)
      {
        printf("Reading Test Mode\n");
        ad9249.getTestMode(mode);
      }
      else
      {
        if (argc < 4 || argc > 5)
        {
          printf("Error: Invalid arg. count for test mode configuration\n");
          printUsage();
          return -1;
        }
        if (0 == strcmp(argv[3], "off")) mode = AD9249_TEST_MODE_OFF;
        else if (0 == strcmp(argv[3], "checkerboard")) mode = AD9249_TEST_MODE_CHECKERBOARD;
        else if (0 == strcmp(argv[3], "pn_short")) mode = AD9249_TEST_MODE_PN_SHORT;
        else if (0 == strcmp(argv[3], "pn_long")) mode = AD9249_TEST_MODE_PN_LONG;
        else if (0 == strcmp(argv[3], "user_pat")) mode = AD9249_TEST_MODE_USER_INPUT;
        else
        {
          printf("Error: \"%s\" is an unrecognized test mode\n", argv[3]);
          printUsage();
          return -1;
        }
        uint16_t device_bitmask = AD9249_CHANNEL_MASK_ALL;
        if (argc == 5)
        {
          device_bitmask = strtol(argv[4], nullptr, 0);
          if (device_bitmask == 0)
          {
            printf("Error: Invalid device bitmask specified for test mode (%s) \n", argv[4]);
            printUsage();
            return -1;
          }
        }
        ad9249.setTestMode(mode, device_bitmask);
      }
    } break;
    case 'u':
    {
      uint16_t word_1;
      uint16_t word_2;
      if (read_only)
      {
        printf("Reading User Test Pattern\n");
        ad9249.getUserTestPattern(word_1, word_2);
      }
      else
      {
        if (argc != 5)
        {
          printf("Error: Invalid arg. count for User Test Pattern configuration\n");
          printUsage();
          return -1;
        }
        word_1 = static_cast<uint16_t>(strtol(argv[3], nullptr, 0) & 0x3FFF);
        word_2 = static_cast<uint16_t>(strtol(argv[4], nullptr, 0) & 0x3FFF);
        ad9249.setUserTestPattern(word_1, word_2);
      }
    } break;
    default:
      printf("Error: Unknown config_id %s\n", argv[2]);
      printUsage();
      return -1;
  }

  return 0;
}
#endif // AD9249_TEST_APP
