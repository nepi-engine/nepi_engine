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

#include "drivers/vca8617.h"

#ifdef VCA8617_TEST_APP
void printSetup(const VCA8617_Setup setup);
#endif

int vca8617_setup(const char *spidev_device_path, const VCA8617_Setup &setup)
{
  int fd = open(spidev_device_path, O_RDWR);
  if (fd < 0) return -1;

  struct spi_ioc_transfer xfer;
  memset(&xfer, 0, sizeof(xfer));

  // We write the 5-byte control and then read back as the following 5 bytes, all as a single transfer (CS asserted the entire time)
  uint8_t write_buf[10];
  memset(write_buf, 0, sizeof(write_buf));
  uint64_t read_buf[10];
  memset(read_buf, 0, sizeof(read_buf));

  write_buf[0] = VCA8617_START_BIT | VCA8617_WRITE_BIT | setup.pwr_state | setup.attenuation | setup.mode | setup.gain;
  write_buf[1] = 0x01;
  write_buf[2] = 0x23;
  write_buf[3] = 0x45;
  write_buf[4] = 0x67;
  // All other control bytes can remain zero, since CW mode isn't implemented here anyway
  xfer.tx_buf = (uint64_t)write_buf;
  xfer.rx_buf = (uint64_t)read_buf;
  xfer.len = 10;

  const int status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
  if (status < 0)
  {
    close(fd);
    return status;
  }

  // Now verify that the readback was as expected
  // TODO: Programatic check
  const uint8_t readback_control_word = *(((uint8_t*)xfer.rx_buf) + 5);
  #ifdef VCA8617_TEST_APP
    printf("\nvca8617_setup: Wrote control word 0x%x and read back 0x%x\n\n", write_buf[0], readback_control_word);
    // Now decode it
    VCA8617_Setup setup_out;
    setup_out.pwr_state = (VCA8617_PWR)(readback_control_word & 0x20); // bit 5
    setup_out.attenuation = (VCA8617_ATTEN)(readback_control_word & 0x18); // bits 4:3
    setup_out.mode = (VCA8617_MODE)(readback_control_word & 0x4); // bit 2
    setup_out.gain = (VCA8617_GAIN)(readback_control_word & 0x3); // bits 1:0
    printf("Decoded read back:\n");
    printSetup(setup_out);
  #endif
  int retval = 0;
  if (readback_control_word != write_buf[0])
  {
    retval = -1;
  }
  close(fd);
  return retval;
}

/* This function is not working
int vca8617_read_setup(const char *spidev_device_path, VCA8617_Setup &setup_out)
{
  int fd = open(spidev_device_path, O_RDWR);
  if (fd < 0) return -1;

  struct spi_ioc_transfer xfer;
  memset(&xfer, 0, sizeof(xfer));

  // We write a 5-byte dummy control and then read back as the following 5 bytes, all as a single transfer (CS asserted the entire time)
  uint8_t write_buf[10];
  memset(write_buf, 0, sizeof(write_buf));
  uint64_t read_buf[10];
  memset(read_buf, 0, sizeof(read_buf));

  write_buf[0] = VCA8617_START_BIT | VCA8617_READ_BIT; // Only these two bits matter
  xfer.tx_buf = (uint64_t)write_buf;
  xfer.rx_buf = (uint64_t)read_buf;
  xfer.len = 10;

  const int status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
  if (status < 0)
  {
    close(fd);
    return status;
  }

  // Now verify that the readback was as expected
  // TODO: Programatic check
  const uint8_t readback_control_word = *(((uint8_t*)xfer.rx_buf) + 5);
  close(fd);

  // Now decode it
  setup_out.pwr_state = (VCA8617_PWR)(readback_control_word & 0x20); // bit 5
  setup_out.attenuation = (VCA8617_ATTEN)(readback_control_word & 0x18); // bits 4:3
  setup_out.mode = (VCA8617_MODE)(readback_control_word & 0x4); // bit 2
  setup_out.gain = (VCA8617_GAIN)(readback_control_word & 0x3); // bits 1:0

  return 0;
}
*/

#ifdef VCA8617_TEST_APP
void printSetup(const VCA8617_Setup setup)
{
  printf("Power State = %s\n", (setup.pwr_state & 0x20)? "Disabled" : "Enabled");

  printf("Attenuation = ");
  switch (setup.attenuation)
  {
    case VCA8617_ATTEN_29dB:
      printf("29dB\n"); break;
    case VCA8617_ATTEN_33dB:
      printf("33dB\n"); break;
    case VCA8617_ATTEN_36p5dB:
      printf("36.5dB\n"); break;
    case VCA8617_ATTEN_40dB:
      printf("40dB\n"); break;
    default:
      printf("Unknown!!!\n");
  }

  printf("Mode = %s\n", (setup.mode & VCA8617_MODE_TGC)? "TGC" : "Doppler");

  printf("Gain = ");
  switch (setup.gain)
  {
    case VCA8617_GAIN_25dB:
      printf("25dB\n"); break;
    case VCA8617_GAIN_30dB:
      printf("30dB\n"); break;
    case VCA8617_GAIN_35dB:
      printf("35dB\n"); break;
    case VCA8617_GAIN_40dB:
      printf("40dB\n"); break;
    default:
      printf("Unknown!!!");
  }
}

void printUsage()
{
  printf("\nUsage:\n");
  printf("vca8617_test_app /dev/spidevX.Y <p|P> <a29|a33|a36.5|a40> <tgc|doppler> <g25|g30|g35|g40>\n");
  printf("\tp = power off, P = power on\n");
  printf("\taXX = attenuation in dB\n");
  printf("\ttgc = TGC mode, dopper = Doppler mode\n");
  printf("\tgXX = gain in dB\n");
  printf("For default settings (P a40 tgc g40) no argument is required... otherwise all arguments must be present\n\n");
}

int argsToSetup(char **argv, VCA8617_Setup &setup)
{
  if (0 == strcmp(argv[2], "p"))
  {
    setup.pwr_state = VCA8617_PWR_DISABLED;
  }
  else if (0 == strcmp(argv[2], "P"))
  {
    setup.pwr_state = VCA8617_PWR_ENABLED;
  }
  else
  {
    printf("Error: Invalid power state setting: %s\n", argv[2]);
    return -1;
  }

  if (0 == strcmp(argv[3], "a29"))
  {
    setup.attenuation = VCA8617_ATTEN_29dB;
  }
  else if (0 == strcmp(argv[3], "a33"))
  {
    setup.attenuation = VCA8617_ATTEN_33dB;
  }
  else if (0 == strcmp(argv[3], "a36.5"))
  {
    setup.attenuation = VCA8617_ATTEN_36p5dB;
  }
  else if (0 == strcmp(argv[3], "a40"))
  {
    setup.attenuation = VCA8617_ATTEN_40dB;
  }
  else
  {
    printf("Error: Invalid attenuation setting: %s\n", argv[3]);
    return -1;
  }

  if (0 == strcmp(argv[4], "tgc"))
  {
    setup.mode = VCA8617_MODE_TGC;
  }
  else if (0 == strcmp(argv[4], "doppler"))
  {
    setup.mode = VCA8617_MODE_DOPPLER;
  }
  else
  {
    printf("Error: Invalid mode setting: %s\n", argv[4]);
    return -1;
  }

  if (0 == strcmp(argv[5], "g25"))
  {
    setup.gain = VCA8617_GAIN_25dB;
  }
  else if (0 == strcmp(argv[5], "g30"))
  {
    setup.gain = VCA8617_GAIN_30dB;
  }
  else if (0 == strcmp(argv[5], "g35"))
  {
    setup.gain = VCA8617_GAIN_35dB;
  }
  else if (0 == strcmp(argv[5], "g40"))
  {
    setup.gain = VCA8617_GAIN_40dB;
  }
  else
  {
    printf("Error: Invalid gain setting: %s\n", argv[5]);
    return -1;
  }

  return 0;
}

int main(int argc, char **argv)
{
  VCA8617_Setup setup; // Just use defaults here
  if (argc == 6)
  {
    if (0 != argsToSetup(argv, setup))
    {
      // error logged upstream
      printUsage();
      return -1;
    }
  }
  else if (argc != 2)
  {
    printf("Error: Invalid argument count (%d) --  must be exactly 1 or 5 arguments\n", argc - 1);
    printUsage();
    return -1;
  }

  printf("Setting VCA8617 for\n");
  printSetup(setup);

  if (0 != vca8617_setup(argv[1], setup))
  {
    printf("\nFailed to write setup: %s\n", (errno == 0)? "Readback Mismatch\n\tIf CS bitmask configured for multiple VGAs, this is probably okay" : strerror(errno));
    return -1;
  }

  /*
  printf("Writing complete... press any key to commence readback\n");
  getchar();


  if (0 != vca8617_read_setup(argv[1], setup))
  {
    printf("Failed to read back setup (%s)\n", strerror(errno));
    return -1;
  }
  printf("Read back setup:\n");
  printSetup(setup);
  */
}
#endif //VCA8617_TEST_APP
