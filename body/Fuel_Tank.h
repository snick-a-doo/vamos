//  Fuel_Tank.h - a particle that holds fuel for the engine.
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

#ifndef _FUEL_TANK_H_
#define _FUEL_TANK_H_

#include "Particle.h"
#include "../geometry/Three_Vector.h"

namespace Vamos_Body
{
  //* A fuel tank particle.
  class Fuel_Tank : public Particle
  {
	// The capacity of the tank.
	double m_capacity;

	// The remaining volume of fuel.
	double m_volume;

	// The denisty of the fuel.
	double m_density;

	// update_mass () is called to re-calculate the mass of fuel
	// remaining. 
	void update_mass () { set_mass (m_density * m_volume); }

  public:
	Fuel_Tank (const Vamos_Geometry::Three_Vector& position,
			   double capacity,
			   double volume,
			   double density,
               const Frame* parent = 0);

	// Put fuel in the tank.  With the default VOLUME of -1.0, the
	// tank is filled to capacity.  With any other negative VOLUME, or
	// zero, the tank is emptied, i.e. the volume is set to zero.
	void fill (double volume = -1.0);

	// Decrease the volume of fuel by AMMOUNT.  The volume remaining
	// is returned.
	double consume (double amount);

	// Return the volume of fuel remaining.
	double fuel () const { return m_volume; }

	// Return true if the tank is empty, false otherwise.
	bool empty () const { return m_volume == 0.0; }
  };
}

#endif // !_FUEL_TANK_H_
