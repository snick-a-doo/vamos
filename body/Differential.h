//  Differential.h - the differential gear system of the drivetrain.
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

#ifndef _DIFFERENTIAL_H_
#define _DIFFERENTIAL_H_

namespace Vamos_Body
{
  // The differential gear system of the drivetrain.
  class Differential
  {
	// The differential's gear ratio.
	double m_final_drive;

	double m_anti_slip;

	// The angular speeds of the wheels.
	double m_left_wheel_speed;
	double m_right_wheel_speed;

	double m_left_wheel_torque;
	double m_right_wheel_torque;

	double get_anti_slip_torque () const;

  public:
	Differential (double final_drive, double anti_slip);

	double get_driveshaft_speed (double left_wheel_speed, 
								 double right_wheel_speed);

	void find_wheel_torques (double driveshaft_torque);
	double left_wheel_torque () const { return m_left_wheel_torque; }
	double right_wheel_torque () const { return m_right_wheel_torque; }
  };
}

#endif // not _DIFFERENTIAL_H_
