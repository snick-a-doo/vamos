//  Copyright (C) 2005-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

// @file Numerical utility functions.
#ifndef _NUMERIC_H_
#define _NUMERIC_H_

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace Vamos_Geometry
{
    /// Exception thrown if the computation can't be performed for the given arguments.
    template<typename T> class Bad_Argument : public std::runtime_error
    {
    public:
        Bad_Argument(T x, std::string const& message)
        : std::runtime_error("Argument is " + std::to_string(x) + ". " + message)
        {}
    };

    /// @return -1, 0, 1 for value < 0, == 0, > 0, respectively.
    template<typename T> T sign(T value) noexcept
    {
        return value == 0 ? 0 : (value > 0 ? 1 : -1);
    }

    /// Clip value to the range low--high.
    template<typename T> T clip(T value, T low, T high)
    {
        return std::max(std::min(value, high), low);
    }

  // True if VALUE is in the range LOW--HIGH.
  template <typename T> bool is_in_range (T value, T low, T high)
  { return (value >= low) && (value <= high); }

  // Return number in the interval [0.0, maximum).
  template <typename T> T wrap (T number, T maximum)
  {
      if (maximum < std::numeric_limits<T>::min())
          throw(Bad_Argument(maximum, "Maximum must be positive"));
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
      using namespace std::numbers;
      return wrap (angle, minimum, minimum + 2*pi);
  }

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

    /// @return a random number in the given range.
    double random_in_range(double low, double high);
}

#endif
