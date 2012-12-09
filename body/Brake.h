//  Brake.h - a brake for a wheel.
//
//  Copyright (C) 2001 Sam Varner
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

#ifndef _BRAKE_H_
#define _BRAKE_H_

namespace Vamos_Body
{
  //* A brake for a wheel.
  class Brake
  {
	// The sliding coefficient of friction for the brake pads on the
	// rotor.
	double m_friction; 

	// The effective radius of the rotor.
	double m_radius;

	// The area of the brake pads.
	double m_area;

	// The maximum allowed pressure.
	double m_max_pressure;

	// The fraction of the pressure to be applied to the brake.
	double m_bias;

	// The brake locks when the linear brake velocity divided by pl
	double m_threshold;

	// true if the brake is locked, false otherwise.
	bool m_is_locked;

  public:
	//** Constructor
	Brake (double slide, double radius, double area, double max_pressure, 
		   double bias);

	// Return the torque exerted on the wheel by the brake.
	double torque (double factor, double rotational_velocity);

	// Rerturn true if the brake is locked, false otherwise.
	bool is_locked () const { return m_is_locked; }
  };
}

#endif // !_BRAKE_H_
