//  Wheel.h - a wheel.
//
//  Copyright (C) 2001--2004 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _WHEEL_H_
#define _WHEEL_H_

#include "suspension.h"
#include "tire.h"
#include "brake.h"
#include "contact-point.h"
#include "../geometry/constants.h"
#include "../geometry/material.h"
#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"

#include <GL/gl.h>

#include <string>

namespace Vamos_Body
{
  //* A wheel.  
  // Handles wheel geometry.  See Tire, Brake and Suspension for
  // related functionality.
  class Wheel : public Contact_Point
  {
  private:

	// The initial position of the wheel.
	Vamos_Geometry::Three_Vector m_original_position;

	double m_tire_offset;
	// The distance from the steering pivot to the effective center
	// of the contact patch.

	double m_roll_height;
	// How far off the road lateral forces are applied to the chassis.

	// The suspension that the wheel is attached to.
	Suspension* mp_suspension;

	// The tire that is attached to the wheel.
	Tire m_tire;

	// The brake for the wheel.
	Brake m_brake;

	// The velocity of the ground relative to the wheel.
	Vamos_Geometry::Three_Vector m_ground_velocity;

	// The current normal vector for the road.
	Vamos_Geometry::Three_Vector m_normal;

	// The angular velocity of the wheel.
 	Vamos_Geometry::Three_Vector m_angular_velocity;
	
	// The surface we're currently on
	Vamos_Geometry::Material m_surface_material;

	double m_drive_torque;

	double m_braking_torque;

	// True if the wheel responds to steering. 
	bool m_steered;

	// True if the wheel is driven.
	bool m_driven;

	// The side of the car that the wheel is on.
	Vamos_Geometry::Direction m_side;

	// The ID of the display lists for the wheel.
	GLuint m_slow_wheel_list;
	GLuint m_fast_wheel_list;
	GLuint m_stator_list;

	// The speed where we start using m_fast_wheel_list.
	double m_transition_speed;

	// The current rotation angle about the axle.
	double m_rotation;

	// Return the display list for the specified model file.
	GLuint make_model (std::string file, double scale,
					   const Vamos_Geometry::Three_Vector& translation,
					   const Vamos_Geometry::Three_Vector& rotation);

	// Do the transformation of the wheel model.
	void transform ();

    Wheel (const Wheel&);

  public:
	//** Constructor
	Wheel (double mass, 
		   Vamos_Geometry::Three_Vector position, 
		   double tire_offset,
		   double roll_height,
		   double restitution,
		   Suspension* suspension, 
		   const Tire& tire, 
		   const Brake& brake,
		   bool steered,
		   bool driven,
		   Vamos_Geometry::Direction side);

	// Find and store the forces and torques for the current
	// configuration.
	void find_forces ();

	// Advance the wheel forward in time by TIME.
	void propagate (double time);

 	// Handle collisions.  The return value is how much the wheel was
 	// displaced by the collision.
	double contact (const Vamos_Geometry::Three_Vector& impulse,
					const Vamos_Geometry::Three_Vector& velocity, 
					double distance,
					const Vamos_Geometry::Three_Vector& normal,
					const Vamos_Geometry::Three_Vector& angular_velocity,
					const Vamos_Geometry::Material& surface_material);

	// Apply the torque TORQUE_IN to the wheel.  This torque results
	// from acceleration or braking.
	void drive_torque (double torque_in);

	void brake (double factor);

	// Set the steering angle.
	void steer (double degree_angle) { mp_suspension->steer (degree_angle); }
 
	// Return the speed of the contact patch of the tire with respect
	// to the ground.
	double slide () const { return m_tire.slide (); }

	// Return the wheel's rotational speed in radians per second.
	double rotational_speed () const { return m_tire.rotational_speed (); }

	// Return the linear speed of the tire tread.  This is the speed
	// that a speedometer on this wheel would read.
	double speed () const { return m_tire.speed (); }

	// Return the position relative to the body where the wheel exerts
	// its force.
	Vamos_Geometry::Three_Vector force_position () const;

	// Return the position of the wheel relative to the body for the
	// purpose of detecting collisions.
	Vamos_Geometry::Three_Vector contact_position () const;

	// Fill in the slip ratios for the wheel's tire;
    Vamos_Geometry::Two_Vector slip () const;

    double grip () const { return m_tire.grip (); }
    double temperature () const { return m_tire.temperature (); }
    double wear () const { return m_tire.wear (); }

    // Return the slip ratio that gives maximum force.
    double peak_slip_ratio () const { return m_tire.peak_slip_ratio (); }
    // Return the slip angle that gives maximum force.
    double peak_slip_angle () const { return m_tire.peak_slip_angle (); }
    // Return the normal force on this wheel's tire.
    double normal_force () const { return m_tire.normal_force (); }

	// Return the wheel to its initial conditions.
	void reset ();

	bool single_contact () const { return false; }

	bool steered () const { return m_steered; }
	bool driven () const { return m_driven; }

	Vamos_Geometry::Direction side () const { return m_side; }

	void set_models (std::string slow_file, 
					 std::string fast_file, 
					 double transition_speed,
					 std::string stator_file,
					 double stator_offset,
					 double scale, 
					 const Vamos_Geometry::Three_Vector& translation,
					 const Vamos_Geometry::Three_Vector& rotation);
	void draw ();
  };
}

#endif // !_WHEEL_H_
