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
#ifndef VAMOS_GEOMETRY_NUMERIC_H_INCLUDED
#define VAMOS_GEOMETRY_NUMERIC_H_INCLUDED

#include <array>
#include <cmath>
#include <numbers>
#include <numeric>

namespace Vamos_Geometry
{
/// @return -1, 0, 1 for value < 0, == 0, > 0, respectively.
template <typename T> T sign(T value) noexcept
{
    return value == 0 ? 0 : (value > 0 ? 1 : -1);
}

/// @return @p value if between @p low and @p high, else return @p min if <= @p min, @p
/// max if >= @p max.
template <typename T> T clip(T value, T low, T high)
{
    return std::max(std::min(value, high), low);
}

/// @return True if value is in [low, high]
template <typename T> bool is_in_range(T value, T low, T high)
{
    return value >= low && value <= high;
}

/// @return @p along modulo @p length. Force a positive result so it represents the
/// distance from the start of a closed path after traveling in either direction.
template <typename T> T wrap(T along, T length)
{
    auto mod{std::fmod(along, length)};
    return mod + (mod < 0.0 ? length : 0.0);
}

/// @return @p along in the cyclic interval [start, end).
template <typename T> T wrap(T along, T start, T end)
{
    return start + wrap(along - start, end - start);
}

// Return angle in the interval [minimum, minimum + 2pi).
template <typename T> T branch(T angle, T minimum)
{
    using namespace std::numbers;
    return wrap(angle, minimum, minimum + 2 * pi);
}

/// @return f(x) for a line with a give slope through the point (x1, y1).
template <typename T> T intercept(T x, T x1, T y1, T slope)
{
    return y1 + (x - x1) * slope;
}

/// @return f(x) for a line with a give slope through (x1, y1) and (x2, y2).
template <typename T> T interpolate(T x, T x1, T y1, T x2, T y2)
{
    return intercept(x, x1, y1, (y2 - y1) / (x2 - x1));
}

/// @return The argument with the largest absolute value. At least two arguments are
/// required. All arguments must convert to the same type.
template <typename T, typename... U> double abs_max(T x1, U... xs)
{
    std::array values{xs...};
    return std::reduce(values.begin(), values.end(), std::abs(x1),
                       [](T max, T next) { return std::max(max, std::abs(next)); });
}

/// @return A random number in the given range.
double random_in_range(double low, double high);
} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_NUMERIC_H_INCLUDED
