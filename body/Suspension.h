//  Suspension.h - the suspension component for a wheel.
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

#ifndef _SUSPENSION_H_
#define _SUSPENSION_H_

#include "Particle.h"
#include "../geometry/Constants.h"
#include "../geometry/Three_Vector.h"

#include <vector>
#include <string>

namespace Vamos_Body
{
  struct Suspension_Model;

  class Hinge : public Particle
  {
  public:
	Hinge (const Vamos_Geometry::Three_Vector& position,
           const Frame* parent = 0);

	void input (const Vamos_Geometry::Three_Vector& torque, 
				const Vamos_Geometry::Three_Vector& radius);
  };

  //* The suspension component for a wheel.
  class Suspension : public Particle
  {
  public:
	Suspension (const Vamos_Geometry::Three_Vector& position,
				const Vamos_Geometry::Three_Vector& center_of_translation,
				Vamos_Geometry::Direction side_of_car, 
                double spring_constant, 
				double bounce, 
                double rebound, 
                double travel,
				double max_compression_velocity,
                const Frame* parent = 0);

	~Suspension ();

	//** Geometry-setting methods.  The arguments are in degrees.

	// Set the steering angle.
	void steer (double degree_angle);

	// Set the camber angle.
	void camber (double degree_angle);

	// Set the caster angle.
	void caster (double degree_angle);

	// Set the toe angle.
	void toe (double degree_angle);

	Hinge* hinge () const { return mp_hinge; }

	Vamos_Geometry::Three_Vector force () const { return Particle::force (); }
	Vamos_Geometry::Three_Vector torque () const 
	{ return Particle::torque (); }

	void input (const Vamos_Geometry::Three_Vector& wheel_force,
				const Vamos_Geometry::Three_Vector& normal);

	void torque (double wheel_torque);

	// Calculate the force exerted by the suspension in its current state.
	void find_forces ();

	// Advance this suspension component forward in time by TIME.
	void propagate (double time);

	// Undo the last propagation.
	void rewind ();

	// Specify the suspension component that is attached to this one
	// with an anti-roll bar.  The anti-roll bar will have a spring
	// constant of SPRING_CONSTANT.
	void anti_roll (Suspension* other, double spring_constant);

	// Displace this suspension component by DISTANCE.  A positive
	// DISTANCE means compression.
	void displace (double distance);

	// Return the current displacement.
	double displacement () const { return m_displacement; }

	// Return true if the suspension is displaced as much as it can
	// be, false otherwise.
	bool bottomed_out () const { return m_bottomed_out; }

	// Return the camber angle in radians for a suspension
	// displacement of DISPLACEMENT.
	double camber_function (double displacement) const;

	double current_camber (double normal_y) const;

	// Return this suspension component to equilibrium.
	void reset ();

	void set_model (std::string file_name, 
					double scale, 
					const Vamos_Geometry::Three_Vector& translation, 
					const Vamos_Geometry::Three_Vector& rotation);

	void draw ();

  private:
    // Return the suspension position for the current displacement.
    Vamos_Geometry::Three_Vector get_position () const;

	// The axis of rotation for steering.
	const static Vamos_Geometry::Three_Vector STEER_AXIS;

	Hinge* mp_hinge;
	Vamos_Geometry::Three_Vector m_radius;
	Vamos_Geometry::Three_Vector m_initial_radius;
	double m_radius_magnitude;
	double m_initial_z;

	// The spring constant.
	double m_spring_constant;

	// Damping for compression.
	double m_bounce;

	// Damping for decompression.
	double m_rebound;

	// How far the suspension can be displaced before bottoming out.
	double m_travel;

	// The current displacement of the suspension.  A positive
	// displacement means compression.
	double m_displacement;

	// The size of the last time step handled by propagate().
	double m_time_step;

	// How fast the suspension is being compressed.
	double m_compression_velocity;

	// How fast the suspension can be compressed before the damper
	// locks up.
	double m_max_compression_velocity;

	// true if the displacement has exceeded `m_travel', false
	// otherwise.
	bool m_bottomed_out;

	// The spring constant for the anti-roll bar that connects this
	// suspension component with another.
	double m_anti_roll_k;

	// The suspension component that this one is connected to with an
	// anti-roll bar.
	Suspension* m_anti_roll_suspension;

	Vamos_Geometry::Three_Vector m_wheel_force;

	// The deflection of the tire in radians due to steering.
	double m_steer_angle;

	// The static camber angle in radians.  Positive camber is a
	// rotation that moves the tops of the tires away from the center
	// of the car.  So the direction of the rotation depends on the
	// value of m_side.
	double m_camber;

	// The caster angle in radians.
	double m_caster;

	// The toe angle in radians.  Positive is toe-in.
	double m_toe;

	// The side, RIGHT or LEFT, that the suspension is on.  This is
	// used to choose the right direction for camber, caster and toe
	// adjustments.
	Vamos_Geometry::Direction m_side;

	// The orientation of the wheel in the absence of steering and
	// displacement.
	Vamos_Geometry::Three_Matrix m_static_orientation;

	Vamos_Geometry::Three_Vector m_normal;

    Vamos_Geometry::Three_Vector m_hinge_axis;

	std::vector <Suspension_Model*> m_models;
  };
}

#endif // !_SUSPENSION_H_
