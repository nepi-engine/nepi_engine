/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "drivers/i2c.h"

// Add some delays so the current driver doesn't cause the kernel to crash...
// This problem is fixed in 19.2, but 18.2 seems quite fragile with much
// traffic on the bus.
// https://forums.xilinx.com/t5/Embedded-Linux/Cadence-I2C-driver-bug/td-p/854621
// Try 5ms and see if it works reliably for now...
#define I2C_BUS_DELAY_HACK() usleep(5000)

int i2c_init( i2c_struct_t *i2c, uint32_t bus_id )
{
  char bus_device[64];
  snprintf(bus_device, 64, "/dev/i2c-%u", bus_id);
  i2c->fd = open(bus_device, O_RDWR);
  if (i2c->fd < 0) return -1;

  return 0;
}


// Generic I2C access.
// These return the ioctl return value on success (0 or positive on success)
// On failure, return the errno, as negative value.
int i2c_write( const i2c_struct_t* const i2c, uint8_t addr, uint8_t *data, int len )
{
    i2c_msg              msg;
    i2c_rdwr_ioctl_data  msg_set;

    // Populate the I2C message struct, passing in data ptr
    msg.addr  = addr;
    msg.flags = 0;  // no flags means Write
    msg.len   = len;
    msg.buf   = data;

    // Wrap the messages up for ioctl call (only one in this case)
    msg_set.msgs  = &msg;
    msg_set.nmsgs = 1;

    I2C_BUS_DELAY_HACK();
    int err = ioctl (i2c->fd, I2C_RDWR, &msg_set );
    if (err == -1)
    {
      return -errno;
    }

    return 0;
}

int i2c_read( const i2c_struct_t* const i2c, uint8_t addr, uint8_t *data, int len )
{
    i2c_msg              msg;
    i2c_rdwr_ioctl_data  msg_set;

    // Populate the I2C message struct, passing in data ptr
    msg.addr  = addr;
    msg.flags = I2C_M_RD;
    msg.len   = len;
    msg.buf   = data;

    // Wrap the messages up for ioctl call (only one in this case)
    msg_set.msgs  = &msg;
    msg_set.nmsgs = 1;

    I2C_BUS_DELAY_HACK();
    int err = ioctl (i2c->fd, I2C_RDWR, &msg_set );
    if (err == -1)
    {
      return -errno;
    }

    return 0;
}
