//  Tire.h - the tire for a wheel.
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

#ifndef _TIRE_H_
#define _TIRE_H_

#include "../geometry/Material.h"
#include "../geometry/Three_Vector.h"
#include "Particle.h"

#include <vector>

namespace Vamos_Body
{
  //* The frictional properties of the tire.
  class Tire_Friction
  {
	// The parameters of the longitudinal magic equation.
	std::vector <double> m_longitudital_parameters;

	// The parameters of the transverse magic equation.
	std::vector <double> m_transverse_parameters;

	// The parameters of the magic equation for aligning torque.
	std::vector <double> m_aligning_parameters;

    // The slip that gives the maximum longitudinal force.
    double m_peak_slip;

    // The slip angle that gives the maximum transverse force.
    double m_peak_slip_angle;

    // The slip angle that gives the maximum aligning torque.
    double m_peak_aligning_angle;

	// A parameter that is used to set the volume of the tire squeal
	// sound. 
	double m_slide;

  public:
	//** Constructor
	Tire_Friction (const std::vector <double>& long_parameters,
				   const std::vector <double>& trans_parameters,
				   const std::vector <double>& align_parameters);

	// Return the friction vector calculated from the magic formula.
	// HUB_VELOCITY is the velocity vector of the wheel's reference
	// frame.  PATCH_SPEED is the rearward speed of the contact pacth
	// with respect to the wheel's frame.
	Vamos_Geometry::Three_Vector 
	friction_forces (double normal_force, double friction_factor, 
					 const Vamos_Geometry::Three_Vector& hub_velocity, 
					 double patch_speed, double current_camber);

	// Return the value of the slide parameter.
	double slide () const { return m_slide; }

    // Fill in the longitudinal (sigma) and transverse (alpha) slip ratios.
    static void slip (double patch_speed, 
                      const Vamos_Geometry::Three_Vector& hub_velocity,
                      double* sigma, 
                      double* alpha);

    // Return the slip ratio that gives maximum force.
    double peak_slip_ratio () const { return m_peak_slip; }
    // Return the slip angle that gives maximum force.
    double peak_slip_angle () const { return m_peak_slip_angle; }
  };

  //* The tire for a wheel.
  class Tire : public Particle
  {
	// The radius of the tire.
	double m_radius;

	// Linear rolling resistance on a hard surface.
	double m_rolling_resistance_1;

	// Quadratic rolling resistance on a hard surface.
	double m_rolling_resistance_2;

	Tire_Friction m_tire_friction;

	// The rotational inertia of the tire.
	double m_inertia;

	// The rotational speed of the tire in radians per second.
	double m_rotational_speed;

	double m_last_rotational_speed;

	// How fast the tire is sliding.
	double m_slide;

	//// Input paremeters provided by the wheel.

	// The velocity of the wheel relative to the road.
	Vamos_Geometry::Three_Vector m_velocity;

	// The angular velocity about the normal to the surface.
	double m_normal_angular_velocity;

	// The force perpendicular to the surface.
	double m_normal_force;

	// The current camber, supplied by the wheel.
	double m_camber;

	// The torque on the wheel due to acceleration or braking.
	double m_applied_torque;

	// True if the brake for this wheel is locked.
	bool m_is_locked;

	// The surface that the tire is currently on.
	Vamos_Geometry::Material m_surface_material;

	// Orient the tire's z-axis with the normal force.
	void orient_frame_with_unit_vector (const Vamos_Geometry::
										Three_Vector& normal_unit_vector);

  public:
	Tire (double radius, 
          double rolling_resistance_1,
		  double rolling_resistance_2, 
          const Tire_Friction& friction, 
		  double inertia,
          const Frame* parent = 0);

	// Called by the wheel to update the tire.
	void input (const Vamos_Geometry::Three_Vector& velocity, 
				double normal_angular_velocity,
				const Vamos_Geometry::Three_Vector& normal_force,
				double camber,
				double torque,
				bool is_locked,
				const Vamos_Geometry::Material& surface_material);

	void find_forces ();

	// Advance the tire in time by TIME.
	void propagate (double time);

	void rewind ();

	// Return the radius of the tire
	double radius () const { return m_radius; }

	// Return the rotational speed of the tire.
	double rotational_speed () const { return m_rotational_speed; }

	// Return the linear speed of the tread.
	double speed () const { return m_rotational_speed * m_radius; }
    
    // Fill in the longitudinal (sigma) and transverse (alpha) slip ratios.
    void slip (double* sigma, double* alpha) const;

	// Return the linear sliding speed. 
	double slide () const { return m_slide; }

    // Return the slip ratio that gives maximum force.
    double peak_slip_ratio () const { return m_tire_friction.peak_slip_ratio (); }
    // Return the slip angle that gives maximum force.
    double peak_slip_angle () const { return m_tire_friction.peak_slip_angle (); }

	// Return the position of the contact patch in the wheel's
	// coordinate system.
	Vamos_Geometry::Three_Vector contact_position () const;

    // Return the normal force on this tire.
    double normal_force () const { return m_normal_force; }

	// Set the tire to its initial conditions.
	void reset ();
  };
}

#endif // !_TIRE_H_
