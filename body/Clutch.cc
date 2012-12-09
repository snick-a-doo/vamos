//  Clutch.cc - a clutch for the drivetrain.
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

#include "Clutch.h"

#include <iostream>
#include <cmath>

//* Class Clutch

//** Constructor
Vamos_Body::Clutch::
Clutch (double sliding, double radius, double area, double max_pressure) :
  m_sliding_friction (sliding),
  m_radius (radius),
  m_area (area),
  m_pressure (0.0),
  m_max_pressure (max_pressure),
  m_threshold (0.01),
  m_engaged (false)
{
}

// Return the drag caused by friction between the clutch plates.
double Vamos_Body::Clutch::
drag (double engine_speed, double drive_speed)
{
  double normal = m_pressure * m_area;

  if (std::abs (engine_speed - drive_speed) < (m_threshold * normal))
	{
	  // Setting m_engaged to true will cause the drivetrain to ignore
	  // the return value and act as though the clutch is fully
	  // engaged.
	  m_engaged = true;
	  return 0.0;
	}

  double force = m_sliding_friction * normal;
  if (engine_speed < drive_speed)
	force *= -1;

  // The pressure is considered to be concentrated at m_radius from
  // the center.
  return force * m_radius;
}

// Set the pressure on the plates to FACTOR * m_max_pressure.
void Vamos_Body::Clutch::
position (double factor)
{
  // If FACTOR >= 1.0, the clutch is fully engaged.  Allow for
  // inaccuracy in FACTOR.
  if ((factor + 0.0001) > 1.0)
	{
	  m_engaged = true;
	  m_pressure = m_max_pressure;
	}
  else 
	{
	  m_engaged = false;
	  m_pressure = factor * m_max_pressure;
	}
}
