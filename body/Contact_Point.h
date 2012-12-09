//	Contact_Point.h - a particle that responds to collisions.
//
//  Copyright (C) 2002 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
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

#ifndef _CONTACT_POINT_H_
#define _CONTACT_POINT_H_

#include "Particle.h"
#include "../geometry/Material.h"

namespace Vamos_Body
{
  // A Contact_Point is a Particle that responds to collisions.  The
  // impulse is calculated in the contact() method.
  class Contact_Point : public Particle
  {
  public:
	// Specify position and orientation.
	Contact_Point (double mass, 
                   const Vamos_Geometry::Three_Vector& position, 
				   const Vamos_Geometry::Three_Matrix& orientation,
				   Vamos_Geometry::Material::Material_Type type,
				   double friction, 
                   double restitution,
                   const Frame* parent = 0);

	// Take the parent's orientation.
	Contact_Point (double mass, const Vamos_Geometry::Three_Vector& position,
				   Vamos_Geometry::Material::Material_Type type,
				   double friction, double restitution,
                   const Frame* parent = 0);

    Contact_Point (const Particle& particle,
                   const Vamos_Geometry::Material& material);


	// Handle collisions.  The return value is how much the particle
	// has moved as a result of the contact.  For a Particle, this is
	// always 0.  But, for derived classes that model moving parts, a
	// non-zero value may be returned.
	virtual double contact (const Vamos_Geometry::Three_Vector& impulse,
                            const Vamos_Geometry::Three_Vector& velocity, 
                            double distance,
                            const Vamos_Geometry::Three_Vector& normal,
                            const Vamos_Geometry::Three_Vector& angular_velocity,
                            const Vamos_Geometry::Material& material);

  protected:
	// true if a collision has occurred.
	bool m_contact;

	virtual void find_forces ();

	virtual void end_timestep ();
  };
}

#endif // not _CONTACT_POINT_H_
