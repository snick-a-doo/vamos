//  Fuel_Tank.cc - a particle that holds fuel for the engine.
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

#include "Fuel_Tank.h"

//** Constructor
Vamos_Body::Fuel_Tank::
Fuel_Tank (const Vamos_Geometry::Three_Vector& position,
		   double capacity,
		   double volume,
		   double density,
           const Frame* parent)
  // The mass of the Particle is initially set to 0.0.  The real mas will be
  // calculated by the call to update_mass () in the constructor body.
  : Particle (0.0, position, parent),
	m_capacity (capacity),
	m_volume (volume),
	m_density (density)
{
  update_mass ();
}

// Put fuel in the tank.  With the default VOLUME of -1.0, the tank is
// filled to capacity.  With any other negative VOLUME, or zero, the
// tank is emptied, i.e. the volume is set to zero.
void Vamos_Body::Fuel_Tank::
fill (double volume)
{
  if (volume == -1.0)
	{
	  m_volume = m_capacity;
	}
  else
	{
	  m_volume = volume;
	}

  update_mass ();
}

// Return the volume of fuel remaining.
double Vamos_Body::Fuel_Tank::
consume (double amount)
{
  m_volume -= amount;
  if (m_volume < 0.0)
	{
	  m_volume = 0.0;
	}

  update_mass ();
  return m_volume;
}
