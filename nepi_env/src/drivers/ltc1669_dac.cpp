/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include "drivers/i2c.h"
#include "drivers/ltc1669_dac.h"

int ltc1669_dac_init(ltc1669_dac_struct_t *dac, uint32_t i2c_bus_id, uint8_t addr_7_bit)
{
  const int i2c_init_ret =  i2c_init( &(dac->i2c), i2c_bus_id );
  if (0 != i2c_init_ret)
  {
    return -1;
  }
  dac->addr_7_bit = addr_7_bit;
  dac->initialized = 1;

  // Zero the DAC
  if (0 != ltc1669_dac_write(dac, 0))
  {
    dac->initialized = 0;
    return -2;
  }


  return 0;
}

int ltc1669_dac_write(ltc1669_dac_struct_t* dac, uint16_t val)
{
  if (1 != dac->initialized) return -1;

  uint8_t data[3];
  data[0] = 0x00;                             // "command" is always zero
  data[1] = (uint8_t)(val & 0x00ff);          // low byte first
  data[2] = (uint8_t)((val & 0xff00) >> 8);   // High byte next
  if (0 != i2c_write( &(dac->i2c), dac->addr_7_bit, data, 3 ))
  {
    return -2;
  }

  dac->curr_dac_val = val;
  return 0;
}

#ifdef LTC1669_TEST_APP
  #include <stdio.h>
  #include <stdlib.h>
  #include <errno.h>
  int main(int argc, char** argv)
  {
    if (argc != 4)
    {
      printf("Usage: ltc1669_test_app <i2c_bus_id> <dac_addr_7_bit> <dac_val>\n");
      return -1;
    }

    const uint32_t i2c_bus_id = strtol(argv[1], NULL, 0);
    const uint8_t addr_7_bit = strtol(argv[2], NULL, 0);
    const uint16_t dac_val = strtol(argv[3], NULL, 0);
    printf("ltc1669_test_app starting:\n");
    printf("\tI2C Bus ID = %u\n", i2c_bus_id);
    printf("\t7-bit Addr = 0x%02x\n", addr_7_bit);
    printf("\tDAC Value  = 0x%03x\n", dac_val);

    ltc1669_dac_struct_t dac;
    const int init_ret = ltc1669_dac_init(&dac, i2c_bus_id, addr_7_bit);
    if (0 != init_ret)
    {
      printf("Unable to initialize device (ec=%d, errno=%d)\n", init_ret, errno);
      return -1;
    }

    const int write_ret = ltc1669_dac_write(&dac, dac_val);
    if (0 != write_ret)
    {
      printf("Failed to write value to DAC (ec=%d, errno=%d)\n", write_ret, errno);
    }

    printf("Success!\n");
    return 0;
  }
#endif
