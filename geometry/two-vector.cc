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

#include "two-vector.h"

#include <cmath>
#include <iostream>

using namespace Vamos_Geometry;

Two_Vector::Two_Vector(double x, double y)
    : Point{x, y}
{
}

Two_Vector::Two_Vector()
    : Two_Vector(0.0, 0.0)
{
}

double Two_Vector::magnitude() const
{
    return std::sqrt(x * x + y * y);
}

Two_Vector Two_Vector::unit() const
{
    auto length{magnitude()};
    return length == 0.0 ? Two_Vector{0.0, 1.0} : *this / length;
}

namespace Vamos_Geometry
{
Two_Vector operator+(Two_Vector const& p1, Two_Vector const& p2)
{
    return {p1.x + p2.x, p1.y + p2.y};
}

Two_Vector operator-(Two_Vector const& p1, Two_Vector const& p2)
{
    return {p1.x - p2.x, p1.y - p2.y};
}

Two_Vector operator*(Two_Vector const& p, double scalar)
{
    return {p.x * scalar, p.y * scalar};
}

Two_Vector operator*(double scalar, Two_Vector const& p)
{
    return p * scalar;
}

Two_Vector operator/(Two_Vector const& p, double scalar)
{
    return {p.x / scalar, p.y / scalar};
}

std::istream& operator>>(std::istream& is, Two_Vector& vector)
{
    char delim;
    return is >> delim >> vector.x >> delim >> vector.y >> delim;
}

std::ostream& operator<<(std::ostream& os, Two_Vector const& vector)
{
    return os << "[" << vector.x << ", " << vector.y << "]";
}
} // namespace Vamos_Geometry
