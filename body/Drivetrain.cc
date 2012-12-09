//  Drivetrain.cc - manages the engine, clutch, transmission and differential.
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

#include "Drivetrain.h"

#include <cassert>

//* Class Drivetrain

//** Constructor
Vamos_Body::
Drivetrain::Drivetrain (Engine* engine,
						Clutch* clutch,
						Transmission* transmission,
						Differential* differential) 
  : mp_engine (engine),
	mp_clutch (clutch),
	mp_transmission (transmission),
	mp_differential (differential),
	m_gas (0.0),
	m_auto_neutral (false)
{
}

//** Destructor
Vamos_Body::
Drivetrain::~Drivetrain ()
{
  delete mp_clutch;
  delete mp_transmission;
  delete mp_differential;
}

// Process the input parameters.
void Vamos_Body::
Drivetrain::input (double gas, double clutch, 
				   double left_wheel_speed, double right_wheel_speed)
{
  m_gas = gas;
  mp_clutch->position (clutch);
  
  double shaft_speed = mp_differential->
	get_driveshaft_speed (left_wheel_speed, right_wheel_speed);
  mp_transmission->set_driveshaft_speed (shaft_speed);
  if (m_auto_neutral 
      && (mp_engine->rotational_speed () == 0.0)
      && !mp_engine->is_out_of_gas ())
    {
      mp_transmission->shift (0);
      mp_engine->start ();
    }
}

void Vamos_Body::
Drivetrain::find_forces ()
{
  // Find the drag due to friction in the clutch if the clutch is not
  // fully engaged, and the torque on the driveshaft.
  double drag = 0.0;
  double torque = 0.0;
  if (mp_transmission->gear () != 0)
	{
	  // Find the drag due to friction in the clutch.  As a
	  // side-effect, the value of Clutch::engaged() is set.
	  drag = mp_clutch->drag (mp_engine->rotational_speed (), 
							mp_transmission->clutch_speed ());

  	  if (mp_clutch->engaged ())
		{
		  // If the clutch is fully engaged, the engine speed is forced to
		  // match the transmission speed.  There is no clutch drag.
		  mp_engine->input (m_gas, 0.0, mp_transmission->clutch_speed (), 
							true);
		  torque = mp_transmission->torque (mp_engine->drive_torque ());
		}
  	  else
		{
		  // Find out how much torque we get from the drivetrain.
		  torque = mp_transmission->torque (drag);
		  // Update the Engine
		  mp_engine->input (m_gas, drag, 0.0, false);
		}
	}
  else
	{
	  // We're in neutral.  Just update the engine.
	  mp_engine->input (m_gas, drag, 0.0, false);
	}

  // Apply the torque to the differential.
  mp_differential->find_wheel_torques (torque);
  mp_engine->find_forces ();
}

// Advance the drivetrain in time by TIME.
void Vamos_Body::
Drivetrain::propagate (double time)
{
  mp_engine->propagate (time);
}

void Vamos_Body::
Drivetrain::rewind ()
{
  mp_engine->rewind ();
}

void Vamos_Body::
Drivetrain::reset ()
{
  mp_clutch->position (0.0);
  mp_transmission->shift (0);
}

double Vamos_Body::
Drivetrain::torque (Vamos_Geometry::Direction side) const
{
  switch (side)
	{
	case Vamos_Geometry::LEFT:
	  return mp_differential->left_wheel_torque ();
	  break;
	case Vamos_Geometry::RIGHT:
	  return mp_differential->right_wheel_torque ();
	  break;
	default:
	  assert (false);
	}
  return 0.0;
}
