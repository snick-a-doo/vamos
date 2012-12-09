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

#ifndef _ATMOSPHERE_H_
#define _ATMOSPHERE_H_

#include "../geometry/Three_Vector.h"

namespace Vamos_World
{
  class Atmosphere
  {
	double m_density;
	Vamos_Geometry::Three_Vector m_velocity;

  public:
	Atmosphere (double density, const Vamos_Geometry::Three_Vector& velocity);

	void velocity (const Vamos_Geometry::Three_Vector& vel) 
	{ m_velocity = vel; }
	Vamos_Geometry::Three_Vector velocity () const { return m_velocity; }

	void density (double rho) { m_density = rho; }
	double density () const { return m_density; }
  };
}

#endif
