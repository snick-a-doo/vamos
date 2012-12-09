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

#ifndef _INERTIA_TENSOR_H_
#define _INERTIA_TENSOR_H_

#include "Three_Matrix.h"
#include "Three_Vector.h"

namespace Vamos_Geometry
{
  // Exception class thrown if the inertia tensor is singular.
  class Bad_Inertia_Tensor {};

  // Inertia_Tensor describes the inertia tensor for a rigid body.
  class Inertia_Tensor : public Three_Matrix
  {
	// The total mass of all components.
	double m_mass;
	// The inverse of the inertia tensor.
	Three_Matrix m_inverse;

  public:
	// The constructor is trivial.
	
	// Add a mass to the system at `position'.  The inertia tensor
	// components are calculated here.
	void add (double mass, const Three_Vector& position);

	// Zero the components of the inertia tensor and also the mass.
	void zero ();

	// Calculate the inverse of the inertia tensor.  If the tensor is
	// singular, Bad_Inertia_Tensor is thrown.  update() must be
	// called after all add()s have been performed.
	void update ();
	
	// Return the moment of inertia for a force applied at POSITION in
	// the direction FORCE_DIRECTION.  FORCE_DIRECTION need not be a
	// unit vector.
	double inertia (const Three_Vector& position,
					const Three_Vector& force_direction) const;

	// Return the moment of inertia for TORQUE applied to the center
	// of mass.  TORQUE need not be a unit vector.
	double inertia (const Three_Vector& torque) const;

	// Return the inverse of the inertia tensor.
	const Three_Matrix& inverse () const { return m_inverse; }
  };
}

#endif
