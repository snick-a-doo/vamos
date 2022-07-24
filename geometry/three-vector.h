//  Copyright (C) 2001--2002 Sam Varner
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

#ifndef _THREE_VECTOR_H_
#define _THREE_VECTOR_H_

#include <iostream>
#include <cmath>

namespace Vamos_Geometry
{
  struct Two_Vector;

  /// A three-element vector.  Useful for representing physical
  /// quantites like position, veocity or force.
  struct Three_Vector
  {
    /// Default constructor.  Null vector.
    Three_Vector () : x (0.0), y (0.0), z (0.0) {}
    /// Construct from length and angle in the x-y plane.
    Three_Vector (double length, double angle);
    /// Construct from components.
    Three_Vector (double x_in, double y_in, double z_in)
      : x (x_in), y (y_in), z (z_in) {}
    /// Construct from 2D vector; z is zero.
    Three_Vector (const Two_Vector& xy_vector);

    /// Set each element to zero.
    void zero ();
    /// Return the magnitude of the vector.
    inline double magnitude () const;
    /// True if all elements are zero.
    bool null () const;

    /// Return the dot product with VEC.
    double dot (const Three_Vector& vec) const;
    /// Return the cross product with VEC.
    Three_Vector cross (const Three_Vector& vec) const;

    /// Return the projection onto VEC.
    Three_Vector project (const Three_Vector& vec) const;
    /// Return the vector in the direction of the argument that would
    /// yield this vector when projected.  This is the inverse of
    /// project().
    Three_Vector back_project (const Three_Vector& vec) const;
    /// Return the pependicular distance between vectors parallel to
    /// this one passing through the given points.
    double perp_distance (const Three_Vector& point1,
                          const Three_Vector& point2) const;
    /// Return the pependicular distance between this vector and
    /// the given point.
    double perp_distance (const Three_Vector& point) const;

    /// Return the unit vector that points along this vector.
    inline Three_Vector unit () const;
    /// Return the component of this vector projected onto another
    /// vector.
    double component (const Three_Vector& vec) const;
    /// Return the angle between this vector and another.
    double angle (const Three_Vector& vec) const;

    /// Modifying rotation.  Return a reference to this vector.
    const Three_Vector& rotate (const Three_Vector& vec);
    /// Non-modifying rotation.  Return a fresh vector.
    Three_Vector rotate (const Three_Vector& vec) const;

    /// Arithmetic operators that modify the vector.
    inline Three_Vector& operator += (const Three_Vector& vec);
    inline Three_Vector& operator -= (const Three_Vector& vec);
    inline Three_Vector& operator *= (double factor);
    inline Three_Vector& operator /= (double factor);

    /// The components of the vector.
    double x;
    double y;
    double z;

    /// Common pre-defined vectors
    static const Three_Vector ZERO; ///< The null vector
    static const Three_Vector X; ///< x unit vector
    static const Three_Vector Y; ///< y unit vector
    static const Three_Vector Z; ///< z unit vector
  };

  // Return the magnitude.
  double
  Three_Vector::magnitude () const
  {
    return sqrt (x*x + y*y + z*z);
  }

  Three_Vector& 
  Three_Vector::operator *= (double factor)
  {
    x *= factor;
    y *= factor;
    z *= factor;
    return *this;
  }

  Three_Vector& 
  Three_Vector::operator /= (double factor)
  {
    x /= factor;
    y /= factor;
    z /= factor;
    return *this;
  }

  Three_Vector&
  Three_Vector::operator += (const Three_Vector& vec)
  {
    x += vec.x;
    y += vec.y;
    z += vec.z;
    return *this;
  }

  Three_Vector&
  Three_Vector::operator -= (const Three_Vector& vec)
  {
    x -= vec.x;
    y -= vec.y;
    z -= vec.z;
    return *this;
  }

  // Vector addition
  inline Three_Vector operator + (const Three_Vector& vec1, 
                                  const Three_Vector& vec2)
  {
    return Three_Vector (vec1.x + vec2.x, vec1.y + vec2.y, vec1.z + vec2.z);
  }

  // Vector subtraction
  inline Three_Vector operator - (const Three_Vector& vec1, 
                                  const Three_Vector& vec2)
  {
    return Three_Vector (vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z);
  }

  // Negation
  inline Three_Vector operator - (const Three_Vector& vec)
  {
    return Three_Vector (-vec.x, -vec.y, -vec.z);
  }

  // Scalar multiplication 
  inline Three_Vector operator * (const Three_Vector& vec, double factor)
  {
    return Three_Vector (vec.x * factor, vec.y * factor, vec.z * factor);
  }
  inline Three_Vector operator * (double factor, const Three_Vector& vec)
  {
    return vec * factor;
  }

  // Scalar division
  inline Three_Vector operator / (const Three_Vector& vec, double factor)
  {
    return vec * (1.0 / factor);
  }

  inline bool operator == (const Three_Vector& vec1, const Three_Vector& vec2)
  {
    return ((vec1.x == vec2.x)
            && (vec1.y == vec2.y)
            && (vec1.z == vec2.z));
  }

  // Return the unit vector that points along this vector.
  Three_Vector
  Three_Vector::unit () const
  {
    const double vec_abs = magnitude ();
    if (vec_abs == 0.0)
      return Three_Vector (0.0, 0.0, 1.0);
    return *this / vec_abs;
  }

std::istream& operator>>(std::istream& is, Vamos_Geometry::Three_Vector& vec);
std::ostream& operator<<(std::ostream& os, Vamos_Geometry::Three_Vector const& vec);
}

#endif // not _THREE_VECTOR_H_
