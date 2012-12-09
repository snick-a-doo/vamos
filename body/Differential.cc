//  Differential.cc - the differential gear system.
//
//  Copyright (C) 2001--2004 Sam Varner
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

#include "Differential.h"
#include "../geometry/Numeric.h"

// class Differential - the differential gear system of the
// drivetrain.

Vamos_Body::
Differential::Differential (double final_drive, double anti_slip) 
  : m_final_drive (final_drive),
	m_anti_slip (anti_slip),
	m_left_wheel_speed (0.0),
	m_right_wheel_speed (0.0),
	m_left_wheel_torque (0.0),
	m_right_wheel_torque (0.0)
{
}

double Vamos_Body::Differential::
get_driveshaft_speed (double left_wheel_speed, double right_wheel_speed)
{
  m_left_wheel_speed = left_wheel_speed;
  m_right_wheel_speed = right_wheel_speed;
  return m_final_drive * (left_wheel_speed + right_wheel_speed) / 2.0;
}

double Vamos_Body::
Differential::get_anti_slip_torque () const
{
  double drag = m_anti_slip * (m_left_wheel_speed - m_right_wheel_speed);
  return Vamos_Geometry::clip (drag, -m_anti_slip, m_anti_slip);
}

void Vamos_Body::
Differential::find_wheel_torques (double driveshaft_torque)
{
  double torque = driveshaft_torque * m_final_drive / 2.0;
  double drag = get_anti_slip_torque ();
  m_left_wheel_torque = torque - drag;
  m_right_wheel_torque = torque + drag;
}
