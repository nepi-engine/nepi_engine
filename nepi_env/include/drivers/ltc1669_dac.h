/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
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
