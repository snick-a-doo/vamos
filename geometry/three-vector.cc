//  Copyright (C) 2001--2022 Sam Varner
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

#include "three-vector.h"
#include "three-matrix.h"
#include "two-vector.h"

#include <cmath>
#include <iostream>
#include <sstream>

using namespace Vamos_Geometry;

Three_Vector const x_hat{1.0, 0.0, 0.0};
Three_Vector const y_hat{0.0, 1.0, 0.0};

Three_Vector::Three_Vector()
{
}

Three_Vector::Three_Vector(double x, double y, double z)
    : x{x}, y{y}, z{z}
{
}

Three_Vector::Three_Vector(double length, double angle)
    : Three_Vector{length * std::cos(angle), length * std::sin(angle), 0.0}
{
}

Three_Vector::Three_Vector(Two_Vector const& xy_vector)
    : Three_Vector{xy_vector.x, xy_vector.y, 0.0}
{
}

double Three_Vector::magnitude() const
{
    return sqrt(this->dot(*this));
}

Three_Vector Three_Vector::unit() const
{
    auto vec_abs{magnitude()};
    return vec_abs == 0.0 ? z_hat : *this / vec_abs;
}

Three_Vector& Three_Vector::zero()
{
    return *this = null_v;
}

bool Three_Vector::is_null() const
{
    return x == 0.0 && y == 0.0 && z == 0.0;
}

double Three_Vector::dot(Three_Vector const& vec) const
{
  return x*vec.x + y*vec.y + z*vec.z;
}

Three_Vector Three_Vector::cross(Three_Vector const& vec) const
{
    return {y * vec.z - z * vec.y,
            z * vec.x - x * vec.z,
            x * vec.y - y * vec.x};
}

Three_Vector Three_Vector::project(Three_Vector const& vec) const
{
    auto vec_abs{vec.magnitude()};
    return vec_abs == 0.0 ? null_v : vec * (dot(vec) / vec_abs / vec_abs);
}

Three_Vector Three_Vector::back_project(Three_Vector const& vec) const
{
    auto dot_prod{dot(vec)};
    auto this_abs{magnitude()};
    return dot_prod == 0.0 ? null_v : this_abs * this_abs * vec / dot_prod;
}

Three_Vector& Three_Vector::operator*=(double factor)
{
    x *= factor;
    y *= factor;
    z *= factor;
    return *this;
}

Three_Vector& Three_Vector::operator/=(double factor)
{
    x /= factor;
    y /= factor;
    z /= factor;
    return *this;
}

Three_Vector& Three_Vector::operator+=(Three_Vector const& vec)
{
    x += vec.x;
    y += vec.y;
    z += vec.z;
    return *this;
}

Three_Vector& Three_Vector::operator-=(Three_Vector const& vec)
{
    x -= vec.x;
    y -= vec.y;
    z -= vec.z;
    return *this;
}

namespace Vamos_Geometry
{
extern Three_Vector const null_v{0.0, 0.0, 0.0};
extern Three_Vector const x_hat{1.0, 0.0, 0.0};
extern Three_Vector const y_hat{0.0, 1.0, 0.0};
extern Three_Vector const z_hat{0.0, 0.0, 1.0};

Three_Vector rotate(Three_Vector const& vec1, Three_Vector const& vec2)
{
    return Three_Matrix{1.0}.rotate(vec2) * vec1;
}

Three_Vector operator + (Three_Vector const& vec1, Three_Vector const& vec2)
{
    return {vec1.x + vec2.x, vec1.y + vec2.y, vec1.z + vec2.z};
}

Three_Vector operator - (Three_Vector const& vec1, Three_Vector const& vec2)
{
    return {vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z};
}

Three_Vector operator - (Three_Vector const& vec)
{
    return vec * -1.0;
}

Three_Vector operator * (Three_Vector const& vec, double factor)
{
    return {vec.x * factor, vec.y * factor, vec.z * factor};
}

Three_Vector operator * (double factor, Three_Vector const& vec)
{
    return operator*(vec, factor);
}

Three_Vector operator / (Three_Vector const& vec, double factor)
{
    if (factor == 0.0)
        throw std::overflow_error("Can't divide a Three_Vector by zero.");
    return vec * (1.0 / factor);
}

std::istream& operator >> (std::istream& is, Three_Vector& vec)
{
    std::string str;
    std::getline(is, str, ']');
    std::istringstream line(str);
    char delim;
    line >> delim >> vec.x >> delim >> vec.y >> delim >> vec.z;
    return is;
}

std::ostream& operator << (std::ostream& os, Three_Vector const& vec)
{
    return os << "[" << vec.x << ", " << vec.y << ", " << vec.z << "]";
}
}
