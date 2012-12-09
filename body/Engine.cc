//  Engine.cc - an engine for the drivetrain.
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

#include "Engine.h"
#include "../geometry/Conversions.h"
#include <iostream>

using namespace Vamos_Geometry;

//* Class Engine

//** Constructor
Vamos_Body::
Engine::Engine (double mass, const Three_Vector& position,
				double max_power,
				double peak_engine_rpm,
				double rpm_limit,
				double inertia,
				double idle_throttle,
				double start_speed,
				double stall_speed,
				double fuel_consumption,
                const Frame* parent) 
  : Particle (mass, position, parent),
	m_max_power (max_power),
	m_peak_engine_speed (rpm_to_rad_s (peak_engine_rpm)),
	m_engine_speed_limit (rpm_to_rad_s (rpm_limit)),
	m_inertia (inertia),
	m_idle_throttle (idle_throttle),
	m_start_speed (rpm_to_rad_s (start_speed)),
	m_stall_speed (rpm_to_rad_s (stall_speed)),
	m_fuel_consumption (fuel_consumption),
	m_rotational_speed (0.0),
	m_gas (0.0),
	m_drag (0.0),
	m_transmission_speed (0.0),
	m_out_of_gas (false),
	m_drive_torque (0.0),
	m_drive_impulse (0.0),
	m_engaged (false),
    // See "Motor Vehicle Dynamics" Genta, Section 4.2.2
    m_friction (m_max_power / pow (m_peak_engine_speed, 3))
{
}

void Vamos_Body::
Engine::set_torque_curve (const std::vector <Two_Vector>& torque_points)
{
  m_torque_curve.clear ();
  m_torque_curve.load (torque_points);
  m_torque_curve.scale (rpm_to_rad_s (1.0));
}

// Handle the input parameters.  GAS is the throttle position.
// TRANSMISSION_SPEED is the rotational speed of the transmission side
// of the clutch.  DRAG is the torque due to friction when the clutch
// is not fully engaged.  ENGAGED is true when the clutch is fully
// engaged, false otherwise.
void Vamos_Body::
Engine::input (double gas, double drag, double transmission_speed, 
			   bool engaged)
{
  m_gas = gas;
  m_drag = drag;
  m_transmission_speed = transmission_speed;
  m_engaged = engaged;
}

void Vamos_Body::
Engine::find_forces ()
{
  // Find the engine's torque with the current conditions.
  m_drive_torque = torque_map (m_gas, m_rotational_speed) - m_drag;
  set_torque (Three_Vector (-m_drive_torque, 0.0, 0.0));
}

double Vamos_Body::
Engine::power (double gas, double rotational_speed)
{
  return rotational_speed * torque_map (gas, rotational_speed);
}

void Vamos_Body::
Engine::propagate (double time)
{
  // The engine should change its own speed only when the clutch is 
  // disengaged.  Otherwise, the engine speed is matched to the transmission,
  // which changes speed due to the applied engine torque.
  m_last_rotational_speed = m_rotational_speed;
 
  // If the clutch is engaged, the engine speed is locked to the
  // transmission speed.
  if (m_engaged)
	{
	  m_rotational_speed = m_transmission_speed;
	}
  else
	{
	  m_rotational_speed += time * m_drive_torque / m_inertia;
	}

  // Keep engine speed from going negative when changing from forward to
  // reverse (or vice versa) without using the clutch.
  if (m_rotational_speed < m_stall_speed)
	{
	  m_rotational_speed = 0.0;
	}
}

void Vamos_Body::
Engine::rewind ()
{
  m_rotational_speed = m_last_rotational_speed;
}

// Return the torque for a given throttle setting, GAS, and engine
// speed ROT_SPEED.
double Vamos_Body::
Engine::torque_map (double gas, double rot_speed)
{
  if ((m_out_of_gas) 
      || (m_rotational_speed < m_stall_speed)
      || (m_rotational_speed > m_engine_speed_limit))
    m_gas = 0.0;
  else
    m_gas = std::max (gas, m_idle_throttle);

  if (m_torque_curve.size () == 0)
    {
      // See "Motor Vehicle Dynamics" Genta, Section 4.2.2
      return (m_max_power * m_gas * (1.0 + rot_speed / m_peak_engine_speed) 
                / m_peak_engine_speed)
        - m_friction * rot_speed * rot_speed;
    }
  else
    {
      // Interpolate between the drag curve and the torque curve.
      return m_gas * m_torque_curve.interpolate (rot_speed) 
        - m_friction * rot_speed * rot_speed * (1.0 - m_gas);
    }
}

// Set the engine speed to SPEED_IN and calculate the resulting
// impulse.
void Vamos_Body::
Engine::speed (double speed_in)
{
  if (speed_in > m_stall_speed)
	{
	  m_rotational_speed = speed_in;
	}
  else
	{
	  // The engine stalled.
	  m_rotational_speed = 0.0;
	}
  // Record the change in angular momentum.
  m_drive_impulse = m_inertia * (m_rotational_speed - m_last_rotational_speed);
}
