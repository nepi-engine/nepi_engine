/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <math.h>

#include <algorithms/approx_moving_avg.h>

#define TIME_AVG_WINDOW_SIZE    5

namespace Numurus
{

ApproxMovingAvg::ApproxMovingAvg(size_t window_size) :
  val_count{0},
  mean{0.0}
{
  updateWindowSize(window_size);
}

ApproxMovingAvg::~ApproxMovingAvg()
{}

double ApproxMovingAvg::calculateNext(double new_val)
{
  if (gamma <= 0.0) return new_val; // Means window_size was <= 1

  ++val_count;

  mean = gamma*mean + (1.0 - gamma)*new_val;
  const double bias_corrected_mean = mean / (1.0 - pow(gamma, val_count));

  return bias_corrected_mean;
}

void ApproxMovingAvg::updateWindowSize(size_t window_size)
{
  if (window_size < 1) window_size = 1; // Don't let gamma go negative
  gamma = static_cast<double>(window_size - 1) / window_size;
}

void ApproxMovingAvg::reset()
{
  val_count = 0;
  mean = 0;
}

} // namespace Numurus
