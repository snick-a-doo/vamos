//  Engine.h - an engine for the drivetrain.
//
//  Copyright (C) 2001--2003 Sam Varner
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

#ifndef _ENGINE_H_
#define _ENGINE_H_

#include "../geometry/Conversions.h"
#include "../geometry/Spline.h"
#include "../geometry/Two_Vector.h"
#include "Particle.h"

namespace Vamos_Body
{
  //* An engine for the drivetrain.  Although it produces a torque,
  // Engine is not derived from Component because the Engine's torque
  // is a scalar, not a vector.
  class Engine : public Particle
  {
	// Used to calculate torque in torque_map ().
	double m_max_power;

	// Used to calculate torque in torque_map ().
	double m_peak_engine_speed;

	// The highest allowed engine speed (rev limit).
	double m_engine_speed_limit;

	// The rotational inertia of the engine.
	double m_inertia;

	// The fraction of throttle used when idling.
	double m_idle_throttle;

	// The rotational speed that the engine is set to when starting.
	double m_start_speed;

	// The engine shuts off if the rotational speed goes below this
	// value. 
	double m_stall_speed;

	// The rate of fuel consumption.
	double m_fuel_consumption;

	// The rotational speed of the engine.
	double m_rotational_speed;

	double m_last_rotational_speed;

	// The throttle position.
	double m_gas;

	// The load on the engine from the clutch.
	double m_drag;

	double m_transmission_speed;

	// true if the gas tank is empty, false otherwise.
	bool m_out_of_gas;

	// The minimum throttle position.
	double m_idle;

	// The current torque produced by the engine.
	double m_drive_torque;

	// The impulse calculated in the last call to torque ().
	double m_drive_impulse;

	// true if the clutch is fully engaged, false otherwise.
	bool m_engaged;

	// Set the engine speed to SPEED_IN and calculate the resulting
	// impulse.
	void speed (double speed_in);

    Vamos_Geometry::Spline m_torque_curve;

    double m_friction;

  public:
	//** Constructor
	// MAX_POWER is in the correct derived units (Watts in SI), 
	// PEAK_ENGINE_RPM is in rotations per minute.
	Engine (double mass, const Vamos_Geometry::Three_Vector& position, 
			double max_power,
			double peak_engine_rpm,
			double rpm_limit,
			double inertia,
			double idle_throttle,
			double start_rpm,
			double stall_rpm,
			double fuel_consumption,
            const Frame* parent = 0);

    void set_torque_curve (const std::vector <Vamos_Geometry::Two_Vector>& 
                           torque_points);

    void set_friction (double friction) { m_friction = friction; }

	// Handle the input parameters.  GAS is the throttle position.
	// TRANSMISSION_SPEED is the rotational speed of the transmission
	// side of the clutch.  DRAG is the torque due to friction when
	// the clutch is not fully engaged.  ENGAGED is true when the
	// clutch is fully engaged, false otherwise.
	void input (double gas, double drag, double transmission_speed, 
				bool engaged);

	void find_forces ();

	// Advance the engine in time by TIME.
	void propagate (double time);
  
	void rewind ();

	// Return the current rotational speed in radians per second.
	double rotational_speed () const { return m_rotational_speed; }

	// Return the engine speed where the rev limiter kicks in.
	double max_rotational_speed () const { return m_engine_speed_limit; }

    double peak_engine_speed () const { return m_peak_engine_speed; }

    double stall_speed () const { return m_stall_speed; }

	// Return the current torque.
	double drive_torque () const { return m_drive_torque; }

	double drive_impulse () const { return m_drive_impulse; }

	// Return the torque for a given throttle setting, GAS, and engine
	// speed ROTATIONAL_SPEED.
	double torque_map (double gas, double rotational_speed);

    double power (double gas, double rotational_speed);

	double throttle () const { return m_gas; }

	// Return the current rate of fuel consumption. 
	double fuel_rate () const 
	{ return m_fuel_consumption * m_rotational_speed * m_gas; }

	// Tell the engine if we're out of gas.
	void out_of_gas (bool out) { m_out_of_gas = out; }
    bool is_out_of_gas () const { return m_out_of_gas; }

	// Start the engine.
	void start () { speed (m_start_speed); }
  };
}

#endif // !_ENGINE_H_
