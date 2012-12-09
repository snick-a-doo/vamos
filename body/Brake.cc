//  Brake.cc - a brake for a wheel.
//
//  Copyright (C) 2001--2002  Sam Varner
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

#include "Brake.h"
#include <cmath>

//* Class Brake

//** Constructor
Vamos_Body::Brake::
Brake (double sliding, double radius, double area, double max_pressure, 
	   double bias) :
  m_friction (sliding),
  m_radius (radius),
  m_area (area),
  m_max_pressure (max_pressure * bias),
  m_bias (bias),
  m_threshold (4.0e-4),
  m_is_locked (false)
{
}

// Return the torque exerted on the wheel by the brake.
double Vamos_Body::Brake::
torque (double factor, double rotational_speed)
{
  // `factor' is the fraction of maximum pressure applied. 
  double pressure = factor * m_bias * m_max_pressure;
  double normal = pressure * m_area;
  double torque = m_friction * normal * m_radius;
  double velocity = m_radius * rotational_speed;
  if (velocity < 0.0)
	torque *= -1;

  // See if the brake is locked.
  if (std::abs (velocity) < (m_threshold * normal))
	{
	  m_is_locked = true;
	  torque = 0.0;
	}
  else
	{
	  m_is_locked = false;
	}

  return torque;  
}
