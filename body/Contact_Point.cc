//	Contact_Point.cc - a particle that responds to collisions.
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

#include "Contact_Point.h"

using namespace Vamos_Geometry;
using namespace Vamos_Body;

//* Constructors

Contact_Point::Contact_Point (double mass, 
                              const Three_Vector& position, 
                              const Three_Matrix& orientation,
                              Material::Material_Type type,
                              double friction, 
                              double restitution,
                              const Frame* parent) 
  : Particle (mass, position, orientation, parent),
	m_contact (false)
{
  set_material (Material (type, friction, restitution));
}

// Take the parent's orientation.
Contact_Point::Contact_Point (double mass, 
               const Three_Vector& position,
			   Material::Material_Type type,
			   double friction, 
               double restitution,
               const Frame* parent) 
  : Particle (mass, position, parent),
	m_contact (false)
{
  set_material (Material (type, friction, restitution));
}

Contact_Point::Contact_Point (const Particle& particle,
                              const Material& material)
  : Particle (particle),
	m_contact (false)
{
  set_material (material);
}


// Handle collisions.  The return value is how much the particle has
// moved as a result of the contact.  For a Particle, this is always
// 0.  But, for derived classes that model moving parts, a non-zero
// value may be returned.
double 
Contact_Point::contact (const Three_Vector& impulse,
                        const Three_Vector& velocity, 
                        double distance,
                        const Three_Vector& normal,
                        const Three_Vector& angular_velocity,
                        const Material& material)
{
  set_impulse (rotate_from_parent (impulse));
  m_contact = true;
  return 0.0;
}


// Find and store the forces and torques for the current
// configuration.
void 
Contact_Point::find_forces ()
{
  if (!m_contact)
    reset ();
}

// Do any neccary cleanup at the end of a time step.
void 
Contact_Point::end_timestep ()
{
  m_contact = false;
}
