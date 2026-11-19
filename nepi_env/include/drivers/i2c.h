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
#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>

typedef struct
{
  int fd = 0;
} i2c_struct_t;

int i2c_init( i2c_struct_t *i2c, uint32_t bus_id );
int i2c_write( const i2c_struct_t* const i2c, uint8_t addr, uint8_t *data, int len );
int i2c_read( const i2c_struct_t* const i2c, uint8_t addr, uint8_t *data, int len );

#endif
