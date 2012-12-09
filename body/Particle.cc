//	Particle.cc - a massive particle which is part of a rigid body.
//
//  Copyright (C) 2001--2002 Sam Varner
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

#include "Particle.h"

using namespace Vamos_Body;

// Specify position and orientation.
Particle::Particle (double mass, 
                    const Vamos_Geometry::Three_Vector& position, 
                    const Vamos_Geometry::Three_Matrix& orientation,
                    const Frame* parent)
  : Frame (position, orientation, parent),
	m_mass (mass)
{
}

// Take the parent's orientation.
Particle::Particle (double mass, 
                    const Vamos_Geometry::Three_Vector& position,
                    const Frame* parent)
  : Frame (position, parent),
	m_mass (mass)
{
}

// Take the parent's position and orientation.
Particle::Particle (double mass, const Frame* parent)
  : Frame (parent),
	m_mass (mass)
{
}

void 
Particle::reset ()
{
  m_force.zero ();
  m_impulse.zero ();
  m_torque.zero ();
}
