//	Vamos Automotive Simulator
//  Copyright (C) 2001--2002 Sam Varner
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

#include "Inertia_Tensor.h"

// Add a mass to the system at `position'.  The inertia tensor
// components are calculated here.
void Vamos_Geometry::Inertia_Tensor::
add (double mass, const Three_Vector& position)
{
  m_mass += mass;

  // I_xx
  (*this) [0][0] += mass 
	* (position.y * position.y + position.z * position.z);
  // I_xy
  (*this) [0][1] -= mass * (position.x * position.y);
  // I_xz
  (*this) [0][2] -= mass * (position.x * position.z);
  
  // I_yy
  (*this) [1][1] += mass 
	* (position.z * position.z + position.x * position.x);
  // I_yz
  (*this) [1][2] -= mass * (position.y * position.z);
  
  // I_zz
  (*this) [2][2] += mass 
	* (position.x * position.x + position.y * position.y);	  
}

// Zero the components of the inertia tensor and also the mass.
void Vamos_Geometry::Inertia_Tensor::
zero ()
{
  Three_Matrix::zero ();
  m_mass = 0.0;
}

// Calculate the inverse of the inertia tensor.  If the tensor is
// singular, Bad_Inertia_Tensor is thrown.  update() must be called
// after all add()s have been performed.
void Vamos_Geometry::Inertia_Tensor::
update ()
{
  // Fill in the symmetric components of the inertia tensor.
  (*this) [1][0] = (*this) [0][1];
  (*this) [2][0] = (*this) [0][2];
  (*this) [2][1] = (*this) [1][2];

   try
	{
	  m_inverse = invert ();
	}
  catch (Vamos_Geometry::Singular_Matrix)
	{
	  // If the inertia tensor is singular, throw an exception that tells
	  // that the body is ill-formed.
	  throw Bad_Inertia_Tensor ();
	}
}

// Return the moment of inertia for a force applied at `position' in
// the direction `force_direction'.  `force_direction' need not be a
// unit vector.
double Vamos_Geometry::Inertia_Tensor::
inertia (const Three_Vector& position, const Three_Vector& force_direction)
  const
{
  // The axis of rotation for a force applied in the direction of normal.
  // The maxnitude of axis is the rotational inertia about that axis.
  Three_Vector axis = m_inverse * (position.cross (force_direction.unit ()));

  return m_mass / 
	(1.0 
	 + m_mass 
	 * (axis.cross (position).project (force_direction.unit ())).magnitude ());
}

// Return the moment of inertia for TORQUE applied to the center of
// mass.  TORQUE need not be a unit vector.
double Vamos_Geometry::Inertia_Tensor::
inertia (const Three_Vector& torque) const
{
  return (torque.unit() * (*this)).magnitude ();
}
