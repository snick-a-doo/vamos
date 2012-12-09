//  Transmission.cc - a gearbox.
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

#include "Transmission.h"

#include <cassert>

//* Constructors

Vamos_Body::
Transmission::Transmission (int forward_gears, 
							double first_ratio, double last_ratio) 
  : m_forward_gears (forward_gears),
	m_reverse_gears (1),
	m_gear (0), // neutral
	m_clutch_speed (0.0)
{
  assert (forward_gears > 1);
  double first_inv = 1.0 / first_ratio;
  double last_inv = 1.0 / last_ratio;
  double increment = (first_inv - last_inv) / (forward_gears - 1);
  m_gear_ratios [0] = 0.0;
  for (int i = 0; i <= forward_gears; i++)
	{
	  m_gear_ratios [i + 1] = 1.0 / (first_inv - increment * i);
	}
  // Put the ratio for reverse midway between first and second.
  m_gear_ratios [-1] = -0.5 * (m_gear_ratios [1] + m_gear_ratios [2]);
}

Vamos_Body::
Transmission::Transmission () 
  : m_forward_gears (0),
	m_reverse_gears (0),
	m_gear (0), // neutral
	m_clutch_speed (0.0)
{
  m_gear_ratios [0] = 0.0;
}

// Shift to a particular gear.
void Vamos_Body::
Transmission::shift (int gear)
{
  m_gear = gear;
}

// Set the gear ratio for GEAR.  RATIO is (driveshaft speed) / 
// (engine speed).  Forward gears have positive ratios, usually > 1.0.
// Neutral has a rato of 0.0, unless you change it.
void Vamos_Body::
Transmission::gear_ratio (int gear, double ratio)
{
  // One driveshaft revolution yields `ratio' engine revolutions. 
  // ratio == 0.0 indcates neutral.
  m_gear_ratios [gear] = ratio;

  // Find out how many consecutive forward gears we have.
  m_forward_gears = 0;
  int key = 1;
  while (m_gear_ratios.find (key) != m_gear_ratios.end ())
	{
	  m_forward_gears++;
	  key++;
	}

  // Find out how many consecutive forward gears we have.
  m_reverse_gears = 0;
  key = -1;
  while (m_gear_ratios.find (key) != m_gear_ratios.end ())
	{
	  m_reverse_gears++;
	  key--;
	}
}

// Return the torque on the driveshaft due to friction at the clutch.
double Vamos_Body::
Transmission::torque (double drag)
{
  return drag * m_gear_ratios [m_gear];
} 

// Tell the transmission what the rotational speed of the driveshaft
// is. 
void Vamos_Body::Transmission::
set_driveshaft_speed (double driveshaft_speed)
{
  m_clutch_speed = driveshaft_speed * m_gear_ratios [m_gear];
}
