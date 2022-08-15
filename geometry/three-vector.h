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

#ifndef VAMOS_GEOMETRY_THREE_VECTOR_H_INCLUDED
#define VAMOS_GEOMETRY_THREE_VECTOR_H_INCLUDED

#include <iosfwd>

namespace Vamos_Geometry
{
struct Two_Vector;

/// A three-element vector.  Useful for representing physical quantites like position,
/// veocity or force.
struct Three_Vector
{
    /// Default constructor.  Null vector.
    Three_Vector();
    /// Construct from length and angle in the x-y plane.
    Three_Vector(double length, double angle);
    /// Construct from components.
    Three_Vector(double x_in, double y_in, double z_in);
    /// Construct from 2D vector; z is zero.
    Three_Vector(Two_Vector const& xy_vector);

    /// @return The vector's magnitude.
    double magnitude() const;
    /// @return the unit vector that points along this vector.
    Three_Vector unit() const;

    /// Set each element to zero.
    Three_Vector& zero();
    /// True if all elements are zero.
    bool is_null() const;

    /// @return the dot product with @p vec.
    double dot(Three_Vector const& vec) const;
    /// @return the cross product with @p vec.
    Three_Vector cross(Three_Vector const& vec) const;

    /// @return the projection onto @p vec.
    Three_Vector project(Three_Vector const& vec) const;
    /// @return The vector in the direction of the argument that would yield this vector
    /// when projected. This is the inverse of project().
    Three_Vector back_project(Three_Vector const& vec) const;

    /// Arithmetic operators that modify the vector.
    Three_Vector& operator += (Three_Vector const& vec);
    Three_Vector& operator -= (Three_Vector const& vec);
    Three_Vector& operator *= (double factor);
    Three_Vector& operator /= (double factor);

    /// Test for equality.
    friend bool operator==(Three_Vector const& vec1, Three_Vector const& vec2) = default;

    /// The components of the vector.
    double x{0.0};
    double y{0.0};
    double z{0.0};

    /// Common pre-defined vectors
    static Three_Vector const ZERO; ///< The null vector
    static Three_Vector const X; ///< x unit vector
    static Three_Vector const Y; ///< y unit vector
    static Three_Vector const Z; ///< z unit vector
};

/// @return @p vec1 rotated about @ vec2 by the magnitude of @p vec2.
Three_Vector rotate(const Three_Vector& vec1, const Three_Vector& vec2);

Three_Vector operator + (Three_Vector const& vec1, Three_Vector const& vec2);
Three_Vector operator - (Three_Vector const& vec1, Three_Vector const& vec2);
Three_Vector operator - (Three_Vector const& vec);
Three_Vector operator * (Three_Vector const& vec, double factor);
Three_Vector operator * (double factor, Three_Vector const& vec);
Three_Vector operator / (Three_Vector const& vec, double factor);

std::istream& operator >> (std::istream& is, Vamos_Geometry::Three_Vector& vec);
std::ostream& operator << (std::ostream& os, Vamos_Geometry::Three_Vector const& vec);
}

#endif // VAMOS_GEOMETRY_THREE_VECTOR_H_INCLUDED
