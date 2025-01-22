/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#include <math.h>

#include "sdk_utils.h"

namespace Numurus
{

// RunningStat class
RunningStat::RunningStat()
{};

void RunningStat::clear()
{
  count = 0; // Everything else gets cleared in push()
};

void RunningStat::push(double x)
{
  ++count;

  // See Knuth TAOCP vol 2, 3rd edition, page 232
  if (count== 1)
  {
      oldM = newM = maxVal = minVal = x;
      oldS = 0.0;
  }
  else
  {
      newM = oldM + (x - oldM)/count;
      newS = oldS + (x - oldM)*(x - newM);

      if (x > maxVal) maxVal = x;
      if (x < minVal) minVal = x;

      // set up for next iteration
      oldM = newM;
      oldS = newS;
  }
}

size_t RunningStat::getCount() const
{
  return count;
}

double RunningStat::mean() const
{
  return (count> 0) ? newM : 0.0;
}

double RunningStat::variance() const
{
  return ( (count> 1) ? newS/(count- 1) : 0.0 );
}

double RunningStat::stdDeviation() const
{
  return sqrt( variance() );
}

double RunningStat::max() const
{
  return maxVal;
}

double RunningStat::min() const
{
  return minVal;
}

} // namespace Numurus
