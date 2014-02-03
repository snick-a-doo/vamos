//	Vamos - a driving simulator
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

#include "Two_Vector.h"

Vamos_Geometry::Two_Vector Vamos_Geometry::
operator + (const Two_Vector& p1, const Two_Vector& p2)
{
  return Two_Vector (p1.x + p2.x, p1.y + p2.y);
}

Vamos_Geometry::Two_Vector Vamos_Geometry::
operator - (const Two_Vector& p1, const Two_Vector& p2)
{
  return Two_Vector (p1.x - p2.x, p1.y - p2.y);
}

Vamos_Geometry::Two_Vector Vamos_Geometry::
operator * (const Two_Vector& p, double scalar)
{
  return Two_Vector (p.x * scalar, p.y * scalar);
}

Vamos_Geometry::Two_Vector Vamos_Geometry::
operator * (double scalar, const Two_Vector& p)
{
  return p * scalar;
}

Vamos_Geometry::Two_Vector Vamos_Geometry::
operator / (const Two_Vector& p, double scalar)
{
  return Two_Vector (p.x / scalar, p.y / scalar);
}

// Stream Operators
std::ostream&
operator << (std::ostream& os, Vamos_Geometry::Two_Vector const& vector)
{
  os << "[ " << vector.x << ", " << vector.y << " ]";
  return os;
}

std::istream&
operator >> (std::istream& is,  Vamos_Geometry::Two_Vector& vector)
{
  char delim;
  is >> delim >> vector.x >> delim >> vector.y >> delim;
  return is;
}
