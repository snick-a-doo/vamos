//	Vamos Automotive Simulator
//  Copyright (C) 2001--2005 Sam Varner
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

#include "Three_Vector.h"
#include "Three_Matrix.h"
#include "Two_Vector.h"

using namespace Vamos_Geometry;

const Vamos_Geometry::Three_Vector Vamos_Geometry::
Three_Vector::ZERO = Three_Vector (0.0, 0.0, 0.0);
const Vamos_Geometry::Three_Vector Vamos_Geometry::
Three_Vector::X = Three_Vector (1.0, 0.0, 0.0);
const Vamos_Geometry::Three_Vector Vamos_Geometry::
Three_Vector::Y = Three_Vector (0.0, 1.0, 0.0);
const Vamos_Geometry::Three_Vector Vamos_Geometry::
Three_Vector::Z = Three_Vector (0.0, 0.0, 1.0);

// Constructors.

// Elements are initialized by length and angle in the x-y plane.
Three_Vector::Three_Vector (double length, double angle)
{
  *this = Three_Vector (length, 0.0, 0.0).rotate (Three_Vector (0.0, 0.0, angle));
}

// Initialize with Two_Vector
Three_Vector::Three_Vector (const Two_Vector& xy_vector)
  : x (xy_vector.x),
    y (xy_vector.y),
    z (0.0)
{
}

// Zero all elements.
void
Three_Vector::zero ()
{
  x = y = z = 0.0;
}

// True if all elements are zero.
bool
Three_Vector::null () const
{
  return (x == 0.0) && (y == 0.0) && (z == 0.0);
}

// Return the dot product with the argument.
double
Three_Vector::dot (const Three_Vector& vec) const
{
  return x*vec.x + y*vec.y + z*vec.z;
}

// Return the cross product with the argument.
Three_Vector
Three_Vector::cross (const Three_Vector& vec) const
{
  return Three_Vector (y * vec.z - z * vec.y,
					   z * vec.x - x * vec.z,
					   x * vec.y - y * vec.x);
}

// Return the projection along the argument.
Three_Vector
Three_Vector::project (const Three_Vector& vec) const
{
  double dot_prod = dot (vec);
  double vec_abs = vec.magnitude ();
  if (vec.magnitude () == 0.0)
	return Three_Vector (0.0, 0.0, 0.0);
  return vec.unit () * dot_prod / vec_abs;
}

// Return the vector in the direction of the argument that would yield
// this vector when projected.  This is the inverse of project().
Three_Vector
Three_Vector::back_project (const Three_Vector& vec) const
{
  double dot_prod = dot (vec);
  if (dot_prod == 0.0)
	return Three_Vector (0.0, 0.0, 0.0);
  double this_abs = magnitude ();
  return this_abs * this_abs * vec / dot_prod;
}

// Return the pependicular distance between vectors parallel to this
// one passing through the given points.
double
Three_Vector::perp_distance (const Three_Vector& point1,
							 const Three_Vector& point2) const
{
  Three_Vector vec1 = (point2 - point1);
  Three_Vector vec2 = vec1.project (*this);
  return (vec1 - vec2).magnitude ();
}

// Return the pependicular distance between this vector and the given
// point.
double
Three_Vector::perp_distance (const Three_Vector& point) const
{
  return dot (point) / magnitude ();
}

double
Three_Vector::component (const Three_Vector& vec) const
{
  return dot (vec) / vec.magnitude ();
}

// Return the angle between this vector and `vec'.
double
Three_Vector::angle (const Three_Vector& vec) const
{
  return acos ((*this).dot (vec) / (*this).magnitude () / vec.magnitude ());
}

const Three_Vector&
Three_Vector::rotate (const Three_Vector& vec)
{
  Three_Matrix r;
  r.rotate (vec);
  *this = r * *this;
  return *this;
}

Three_Vector
Three_Vector::rotate (const Three_Vector& vec) const
{
  Three_Matrix r;
  r.rotate (vec);
  return r * *this;
}

std::istream&
Three_Vector::input (std::istream& is)
{
  char delim;
  is >> delim >> x >> delim >> y >> delim >> z >> delim;
  return is;
}

std::ostream&
Three_Vector::output (std::ostream& os) const
{
  os << "[ " << x << ", " << y << ", " << z << " ]";
  return os;
}
