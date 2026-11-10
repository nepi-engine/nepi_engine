/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#pragma once

#include <chrono>

namespace Numurus
{

class ApproxMovingAvg
{
public:
  ApproxMovingAvg(size_t window_size);
  virtual ~ApproxMovingAvg();

  double calculateNext(double new_val);
  void updateWindowSize(size_t window_size);
  void reset();

protected:
  size_t val_count;
  double mean;
  double gamma;
};

}
