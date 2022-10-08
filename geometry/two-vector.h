//  Copyright (C) 2001-2022 Sam Varner
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

#ifndef VAMOS_GEOMETRY_TWO_VECTOR_H_INCLUDED
#define VAMOS_GEOMETRY_TWO_VECTOR_H_INCLUDED

#include <iostream>

namespace Vamos_Geometry
{
template <typename T> struct Point
{
    T x{0};
    T y{0};
    /// Test for equality.
    friend bool operator==(Point<T> const& p1, Point<T> const& p2) = default;
};

struct Two_Vector : public Point<double>
{
    Two_Vector(double x, double y);
    Two_Vector(Point<double> const& p);
    Two_Vector();
    /// @return The magnitude of this vector.
    double magnitude() const;
    /// @return The unit vector that points along this vector.
    Two_Vector unit() const;
};

/// Arithmetic operators
/// @{
template <typename T> Point<T> operator+(Point<T> const& p1, Point<T> const& p2)
{
    return {p1.x + p2.x, p1.y + p2.y};
}
template <typename T> Point<T> operator-(Point<T> const& p1, Point<T> const& p2)
{
    return {p1.x - p2.x, p1.y - p2.y};
}
template <typename T> Point<T> operator-(Point<T> const& p)
{
    return {-p.x, -p.y};
}
template <typename T> Point<T> operator*(Point<T> const& p, double scalar)
{
    return {p.x * scalar, p.y * scalar};
}
template <typename T> Point<T> operator*(double scalar, Point<T> const& p)
{
    return p * scalar;
}
template <typename T> Point<T> operator/(Point<T> const& p, double scalar)
{
    return {p.x / scalar, p.y / scalar};
}
/// @}


template <typename T>
std::istream& operator>>(std::istream& is, Vamos_Geometry::Point<T>& point)
{
    char delim;
    return is >> delim >> point.x >> delim >> point.y >> delim;
}
template <typename T>
std::ostream& operator<<(std::ostream& os, Vamos_Geometry::Point<T> const& point)
{
    return os << "[" << point.x << ", " << point.y << "]";
}
} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_TWO_VECTOR_H_INCLUDED
