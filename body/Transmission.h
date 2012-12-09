//  Transmission.h - a gearbox for the drivetrain.
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

#ifndef _TRANSMISSION_H_
#define _TRANSMISSION_H_

#include <map>

namespace Vamos_Body
{
  //* A gearbox for the drivetrain.
  class Transmission
  {
	// A map of gears to gear ratios.  Neutral has a key of 0.
	// Reverse gears have negative keys.
	std::map <int, double> m_gear_ratios;

	// The number of consecutive forward gears.  Any non-consecutive
	// gears are inaccessible.
	int m_forward_gears;

	// The number of consecutive reverse gears.  Any non-consecutive
	// gears are inaccessible.
	int m_reverse_gears;

	// The current gear.
	int m_gear;

	// The rotational speed at the clutch side of the transmission.
	double m_clutch_speed;

  public:
	//** Constructors

	// Let the program calulate equally-spaced
	Transmission (int forward_gears, double first_ratio, double last_ratio);

	Transmission ();

	// Shift to a particular gear.
	void shift (int gear);

	// Set the gear ratio for GEAR.  RATIO is (driveshaft speed) /
	// (engine speed).  Forward gears have positive ratios, usually >
	// 1.0.  Neutral has a rato of 0.0, unless you change it.
	void gear_ratio (int gear, double ratio);

    double gear_ratio (int gear) { return m_gear_ratios [gear]; }

	// Return the torque on the driveshaft due to friction at the
	// clutch.
	double torque (double drag);

	// Return the current gear.
	int gear () const { return m_gear; }

	// Return the number of consecutive forward gears.
	int forward_gears () const { return m_forward_gears; }

	// Return the number of consecutive reverse gears.
	int reverse_gears () const { return m_reverse_gears; }

	// Tell the transmission what the rotational speed of the
	// driveshaft is.
	void set_driveshaft_speed (double speed);

	// Return the rotational speed at the clutch side of the
	// transmission. 
	double clutch_speed () const { return m_clutch_speed; }
  };
}

#endif // not _TRANSMISSION_H_
