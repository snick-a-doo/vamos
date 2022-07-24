// Numeric.h - numerical utility functions.
//
//  Copyright (C) 2005 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _NUMERIC_H_
#define _NUMERIC_H_

#include "constants.h"

#include <algorithm>
#include <cstdlib>
#include <ctime>

// Windows' normal build config defines these as macros.
// Undefine these to get real std::min() and std::max() usuable.
#ifdef WIN32
# undef min
# undef max
#endif

namespace Vamos_Geometry
{
  // Return -1, 0, 1 for VALUE < 0, == 0, > 0, respectively.
  template <typename T> T sign (T value)
  { return value == 0 ? 0 : (value > 0 ? 1 : -1); }

  // Clip VALUE to the range LOW--HIGH.
  template <typename T> T clip (T value, T low, T high)
  { return std::max (std::min (value, high), low); }

  // True if VALUE is in the range LOW--HIGH.
  template <typename T> bool is_in_range (T value, T low, T high)
  { return (value >= low) && (value <= high); }

  // Return LOW or HIGH, whichever is closer to VALUE.
  template <typename T> T closer (T value, T low, T high)
  { return (std::abs (value - low) < std::abs (value - high)) ? low : high; }

  // Return the average of LOW and HIGH.
  template <typename T> T average (T low, T high)
  { return 0.5 * (low + high); }

  // Return number in the interval [0.0, maximum).
  template <typename T> T wrap (T number, T maximum)
  {
    while (number >= maximum)
      number -= maximum;
    while (number < 0.0)
      number += maximum;
    return number;
  }

  // Return number in the interval [minimum, maximum).
  template <typename T> T wrap (T number, T minimum, T maximum)
  {
    return minimum + wrap (number - minimum, maximum - minimum);
  }

  // Return angle in the interval [minimum, minimum + 2pi).
  template <typename T> T branch (T angle, T minimum)
  {
    return wrap (angle, minimum, minimum + 2*pi);
  }

  // Square a number
  template <typename T> T square (T value)
  { return value * value; }

  // Cube a number
  template <typename T> T cube (T value)
  { return value * value * value; }

  // Return f(X0) for a line with SLOPE through (X1, Y1).
  template <typename T> T intercept (T x0, T x1, T y1, T slope)
  { return y1 - slope * (x1 - x0); } 

  template <typename T> T interpolate (T x, T x1, T y1, T x2, T y2)
  { return y1 + (y2 - y1) * (x - x1) / (x2 - x1); }

  // Return the argument with the larger absolute value.
  template <typename T> T abs_max (T a, T b)
  { return (std::abs (a) > std::abs (b)) ? a : b; }

  // Return the argument with the largest absolute value.
  template <typename T> T abs_max (T a, T b, T c, T d)
  { return abs_max (abs_max (abs_max (a, b), c), d); }

  // Return the argument with the smaller absolute value.
  template <typename T> T abs_min (T a, T b)
  { return (std::abs (a) < std::abs (b)) ? a : b; }

  // Return the argument with the smallest absolute value.
  template <typename T> T abs_min (T a, T b, T c, T d)
  { return abs_min (abs_min (abs_min (a, b), c), d); }

  /// Return the real part of the square root of x.
  inline double real_sqrt (double x)
  { return x > 0.0 ? std::sqrt (x) : 0.0; }

  /// Return a random number in the given range.
  inline double random_in_range (double low, double high)
  {
    static bool seeded = false;
    if (!seeded)
      {
        srand (time (0));
        seeded = true;
      }
    return low + (high - low) * (double (rand ()) / RAND_MAX);
  }
}

#endif
