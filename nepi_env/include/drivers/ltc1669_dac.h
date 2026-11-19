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
#ifndef __LTC1669_DAC_H
#define __LTC1669_DAC_H

#include <stdint.h>
#include <drivers/i2c.h>

typedef struct
{
  i2c_struct_t i2c;
  uint8_t addr_7_bit = 0x0;
  uint16_t curr_dac_val = 0x0;
  uint8_t initialized = 0;
} ltc1669_dac_struct_t;

int ltc1669_dac_init(ltc1669_dac_struct_t *dac, uint32_t i2c_bus_id, uint8_t addr_7_bit);
int ltc1669_dac_write(ltc1669_dac_struct_t* dac, uint16_t val);
inline uint16_t ltc1669_dac_get_current_val(const ltc1669_dac_struct_t* const dac){return dac->curr_dac_val;}

#endif
