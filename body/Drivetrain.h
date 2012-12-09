//  Drivetrain.h - manages the engine, clutch, transmission and differential.
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

#ifndef _DRIVETRAIN_H_
#define _DRIVETRAIN_H_

#include "Engine.h"
#include "Clutch.h"
#include "Transmission.h"
#include "Differential.h"
#include "../geometry/Constants.h"

namespace Vamos_Body
{
  //* The drivetrain manages the engine, clutch, transmission and
  //  differential.
  class Drivetrain
  {
	//** The components that make up the drivetrain.

	Engine* mp_engine;

	Clutch* mp_clutch;

	Transmission* mp_transmission;

	Differential* mp_differential;


	// The throttle position.
	double m_gas;

    // If true, shift to neutral instead of stalling.
    bool m_auto_neutral;

  public:
	//** Constructor
	Drivetrain (Engine* engine, 
				Clutch* clutch, 
				Transmission* transmission,
				Differential* differential);

	//** Destructor
	~Drivetrain ();

	// Process the input parameters.
	void input (double gas, double clutch,
				double left_wheel_speed, double right_wheel_speed);

	void find_forces ();

	// Advance the drivetrain in time by TIME.
	void propagate (double time);
	void rewind ();
	void reset ();

	// Allow access to the components.
	Engine* engine () { return mp_engine; }
	Transmission* transmission () { return mp_transmission; }
	Clutch* clutch () { return mp_clutch; }

	// Return the torque for a driven wheel.
	double torque (Vamos_Geometry::Direction side) const;

    void set_auto_neutral (bool state) { m_auto_neutral = state; }
  };
}

#endif // not _DRIVETRAIN_H_
