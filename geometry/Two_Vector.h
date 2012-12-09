//	Vamos - a driving simulator
//  Copyright (C) 2001 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef _TWO_VECTOR_H_
#define _TWO_VECTOR_H_

#include <iostream>
#include <cmath>

namespace Vamos_Geometry
{
  struct Two_Vector
  {
	double x;
	double y;
	
	Two_Vector (double x_in, double y_in) : x (x_in), y (y_in) {};
	Two_Vector () : x (0.0), y (0.0) {};

    // Return the magnitude of this vector.
    inline double magnitude () const;

    // Return the unit vector that points along this vector.
    inline Two_Vector unit () const;
  };

  // Return the magnitude of this vector.
  double
  Two_Vector::magnitude () const
  {
    return sqrt (x*x + y*y);
  }

  // Arithmetic operators
  Two_Vector operator + (const Two_Vector& v1, const Two_Vector& v2);
  Two_Vector operator - (const Two_Vector& v1, const Two_Vector& v2);
  Two_Vector operator * (const Two_Vector& v, double scalar);
  Two_Vector operator * (double scalar, const Two_Vector& v);
  Two_Vector operator / (const Two_Vector& v, double scalar);

  inline bool operator == (const Two_Vector& vec1, const Two_Vector& vec2)
  {
    return ((vec1.x == vec2.x) && (vec1.y == vec2.y));
  }

  // Return the unit vector that points along this vector.
  Two_Vector
  Two_Vector::unit () const
  {
    const double length = magnitude ();
    if (length == 0.0)
      return Two_Vector (0.0, 1.0);
    return *this / length;
  }
}

// Stream operators.
std::istream& operator >> (std::istream& is, Vamos_Geometry::Two_Vector& vec);
std::ostream& operator << (std::ostream& os, Vamos_Geometry::Two_Vector const& vec);

#if defined (BOOST_TEST_MODULE)
// For Boost 1.39, need operator << overloaded for Two_Vector in
// namespace std for unit test log messages.  Alternatively, we could
// use BOOST_TEST_DONT_PRINT_LOG_VALUE (Two_Vector), but then the
// messages would be less useful.
namespace std
{
  std::istream& operator >> (std::istream& is, Vamos_Geometry::Two_Vector& vec)
  { return ::operator >> (is, vec); }
  std::ostream& operator << (std::ostream& os, Vamos_Geometry::Two_Vector const& vec)
  { return ::operator << (os, vec); }
}
#endif // BOOST_TEST_MODULE

#endif // not _TWO_VECTOR_H_
