/*
 * Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
 *
 * This file is part of nepi-engine
 * (see https://github.com/nepi-engine).
 *
 * License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
 */
#ifndef _SDK_UTILS_H
#define _SDK_UTILS_H

#define BOOL_TO_ENABLED(x)	((x)==true)? "enabled" : "disabled"
#define DEG_TO_RAD 0.01745329251f
#define RAD_TO_DEG 57.2958

namespace Numurus
{
// Following is lifted from John Cook's blog, in turn lifted from Knuth's TAOCP
class RunningStat
{
public:
  RunningStat();

  void clear();
  void push(double x);
  size_t getCount() const;
  double mean() const;
  double variance() const;
  double stdDeviation() const;
  double max() const;
  double min() const;

private:
  size_t count = 0;
  double oldM = 0.0;
  double newM = 0.0;
  double oldS = 0.0;
  double newS = 0.0;
  double maxVal = 0.0;
  double minVal = 0.0;
};

} // namespace Numurus

#endif
