//	Contact_Point.cc - a particle that responds to collisions.
//
//  Copyright (C) 2002 Sam Varner
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

#include "contact-point.h"

using namespace Vamos_Geometry;
using namespace Vamos_Body;

//* Constructors

Contact_Point::Contact_Point (double mass, 
                              const Three_Vector& position, 
                              const Three_Matrix& orientation,
                              Material::Composition type,
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
			   Material::Composition type,
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

double Contact_Point::contact(Three_Vector const& impulse,
                              Three_Vector const&, // velocity
                              double, // distance,
                              Three_Vector const&, // normal
                              Three_Vector const&, // angular_velocity
                              Material const&) //material
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
