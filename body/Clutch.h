//  Clutch.h - a clutch for the drivetrain.
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

#ifndef _CLUTCH_H_
#define _CLUTCH_H_

namespace Vamos_Body
{
  //* A clutch for the drivetrain.
  class Clutch
  {
	// The coefficient of friction for the clutch plates.
	double m_sliding_friction;

	// The effective radius of the clutch plates.
	double m_radius;

	// The contact area.
	double m_area;
  
	// The current pressure on the plates.
	double m_pressure;

	// The maximum allowed pressure on the plates.
	double m_max_pressure;

	// The locking threshold.  The clutch pretends to be fully engaged
	// when engine speed - transmission speeds < m_threshold * normal
	// force. 
	double m_threshold;

	// true if the clutch is fully engaged, false otherwise.
	bool m_engaged;

  public:
	//** Constructor
	Clutch (double sliding, double radius, double area, double max_pressure);

	// Return the drag caused by friction between the clutch plates.
	double drag (double engine_speed, double drive_speed);

	// Set the pressure on the plates to FACTOR * m_max_pressure.  If
	// FACTOR >= 1.0, the clutch is fully engaged.
	void position (double factor);

	// Return the current pressure.
	double pressure () const { return m_pressure; }

	// Return the maximum presure.
	double max_pressure () const { return m_max_pressure; }

	// Return true if the clutch is fully engaged, false otherwise.
	bool engaged () const { return m_engaged; }
  };
}

#endif // !_CLUTCH_H_
