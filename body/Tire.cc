//  Tire.cc - the tire for a wheel.
//
//  Copyright (C) 2001--2013 Sam Varner
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

#include "Tire.h"
#include "../geometry/Conversions.h"
#include "../geometry/Numeric.h"
#include "../geometry/Parameter.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace Vamos_Body;
using namespace Vamos_Geometry;

const double epsilon = 1.0e-12;
const double ambient_temperature = 300.0;

// The magic equation.  Slip is a percentage for longitudinal force,
// an angle in degrees for transverse force and aligning torque.
static inline
double pacejka_equation (double slip,
                         double B, double C, double D, double E, 
                         double Sh, double Sv)
{
  return D * sin (C * atan (B * (1.0 - E) * (slip + Sh)
                            + E * atan (B * (slip + Sh)))) + Sv;
}

// Return the slip value for which the Pacejka function is maximized.
static
double peak_slip (double B, double C, double E, double Sh, double guess)
{
  struct Slip
  {
    // The Pacejka function is maximized when this function is zero...
    static inline 
    double function (double x, double B, double C, double E, double Sh)
    {
      return B*(1.0 - E)*(x + Sh) + E*atan (B*(x + Sh)) - tan (pi/(2.0*C));
    }
    static inline 
    double derivative (double x, double B, double C, double E, double Sh)
    {
      return B*(1.0 - E) + E*B/(1.0 + B*B*(x + Sh)*(x + Sh));
    }
  };

  // Newton's method.  The shape of the function makes it converge
  // quickly.
  double x = guess;
  double y;
  for (int i = 0; i < 10; i++)
    {
      y = Slip::function (x, B, C, E, Sh);
      if (std::abs (y) < 0.001)
          return x;
      x -= y / Slip::derivative (x, B, C, E, Sh);
    }
  //   std::cerr << "peak_slip() failed x=" << x << " y=" << y << 
  //     " B=" << B << " C=" << C << " E=" << E << " Sh=" << Sh << std::endl;
  return guess;
}

//* Class Tire_Friction

//** Constructor
Tire_Friction::Tire_Friction (const std::vector <double>& long_parameters,
							  const std::vector <double>& trans_parameters,
							  const std::vector <double>& align_parameters) :
  m_longitudital_parameters (long_parameters),
  m_transverse_parameters (trans_parameters),
  m_aligning_parameters (align_parameters),
  m_peak_slip (0.0),
  m_peak_slip_angle (0.0),
  m_peak_aligning_angle (0.0),
  m_slide (0.0)
{
  assert (m_longitudital_parameters.size () == 11);
  assert (m_transverse_parameters.size () == 15);
  assert (m_aligning_parameters.size () == 18);
}

// Return the longitudinal (first) and transverse (second) slip ratios.
void
Tire_Friction::slip (double patch_speed, const Three_Vector& hub_velocity,
                     double* sigma, double* alpha)
{
  // Leave the slip parameters at zero if the difference between the
  // patch speed and the hub velocity is very small.
  *sigma = 0.0;
  *alpha = 0.0;
  if (std::abs (hub_velocity.x - patch_speed) > 1.0e-4)
	{
	  // Put a lower limit on the denominator to keep sigma and
	  // tan_alpha from getting out of hand at low speeds.
	  double denom = std::max (std::abs (hub_velocity.x), 3.0);
	  *sigma = 100.0 * (patch_speed - hub_velocity.x) / denom;
	  *alpha = -rad_to_deg (atan2 (hub_velocity.y, denom));
	}
}

// Return the friction vector calculated from the magic formula.
// HUB_VELOCITY is the velocity vector of the wheel's reference
// frame.  PATCH_SPEED is the rearward speed of the contact pacth with
// respect to the wheel's frame.
Three_Vector
Tire_Friction::friction_forces (double normal_force, double friction_factor,
								const Three_Vector& hub_velocity, 
								double patch_speed, double current_camber)
{
  double Fz = normal_force / 1000.0;
  double Fz_squared = Fz * Fz;

  // Evaluate the longitudinal parameters.
  const std::vector <double>& b = m_longitudital_parameters;
  double Cx = b [0];
  double Dx = friction_factor * (b [1] * Fz_squared + b [2] * Fz);
  double Bx = (b [3] * Fz_squared + b [4] * Fz) * exp (-b [5] * Fz) / (Cx * Dx);
  double Ex = b [6] * Fz_squared + b [7] * Fz + b [8];
  double Shx = b [9] * Fz + b [10];

  // Evaluate the transverse parameters.
  const std::vector <double>& a = m_transverse_parameters;
  double gamma = rad_to_deg (current_camber);
  double Cy = a [0];
  double Dy = friction_factor * (a [1] * Fz_squared + a [2] * Fz);
  double By = a [3] * sin (2.0 * atan (Fz / a [4])) 
    * (1.0 - a [5] * std::abs (gamma)) / (Cy * Dy);
  double Ey = a [6] * Fz + a [7];
  double Shy = a [8] * gamma + a [9] * Fz + a [10];
  double Svy = (a [11] * Fz + a [12]) * gamma * Fz + a [13] * Fz + a [14];

  // Evaluate the aligning parameters.
  const std::vector <double>& c = m_aligning_parameters;
  double Cz = c [0];
  double Dz = friction_factor * (c [1] * Fz_squared + c [2] * Fz);
  double Bz = (c [3] * Fz_squared + c [4] * Fz) 
	* (1.0 - c [6] * std::abs (gamma))
	* exp (-c [5] * Fz) / (Cz * Dz);
  double Ez = (c [7] * Fz_squared + c [8] * Fz + c [9]) 
	* (1.0 - c [10] * std::abs (gamma));
  double Shz = c [11] * gamma + c [12] * Fz + c [13];
  double Svz = (c [14] * Fz_squared + c [15] * Fz) * gamma + c [16] * Fz 
    + c [17];

  double sigma;
  double alpha;
  slip (patch_speed, hub_velocity, &sigma, &alpha);

  m_peak_slip = peak_slip (Bx, Cx, Ex, Shx, m_peak_slip);
  Three_Vector slip;
  if (m_peak_slip > epsilon)
    slip.x = sigma / m_peak_slip;
  m_peak_slip_angle = peak_slip (By, Cy, Ey, Shy, m_peak_slip_angle);
  if (m_peak_slip_angle > epsilon)
    slip.y = alpha / m_peak_slip_angle;
  m_peak_aligning_angle = peak_slip (Bz, Cz, Ez, Shz, m_peak_aligning_angle);
  if (m_peak_aligning_angle > epsilon)
    slip.z = alpha / m_peak_aligning_angle;

  double slip_xy = Three_Vector (slip.x, slip.y, 0.0).magnitude ();
  double Fx = pacejka_equation (sign (slip.x) * slip_xy * m_peak_slip, 
                                Bx, Cx, Dx, Ex, Shx, 0.0);
  double Fy = pacejka_equation (sign (slip.y) * slip_xy * m_peak_slip_angle, 
                                By, Cy, Dy, Ey, Shy, Svy);

  double slip_xz = Three_Vector (slip.x, 0.0, slip.z).magnitude ();
  double Mz = pacejka_equation (slip_xz * m_peak_aligning_angle, 
                                Bz, Cz, Dz, Ez, Shz, Svz);

  if (slip_xy > epsilon)
	{
  	  Fx *= std::abs (slip.x) / slip_xy;
      Fy *= std::abs (slip.y) / slip_xy;
    }
  if (slip_xz > epsilon)
      Mz *= slip.z / slip_xz;

  // Set the volume of the tire squeal sound.
  m_slide = (hub_velocity.magnitude () < 0.1) ? 0.0 : slip_xy;

  // Construct the friction vector.  The z-component is the aligning
  // torque.
  return Three_Vector (Fx, Fy, Mz);
}

//* Class Tire

//** Constructor
Vamos_Body::
Tire::Tire (double radius, 
            double rolling_resistance_1,
			double rolling_resistance_2, 
            const Tire_Friction& friction,
            double hardness,
			double inertia,
            const Frame* parent) 
  : Particle (0.0, parent),
    m_radius (radius),
    m_rolling_resistance_1 (rolling_resistance_1),
    m_rolling_resistance_2 (rolling_resistance_2),
    m_tire_friction (friction),
    m_inertia (inertia),
    m_rotational_speed (0.0),
    m_last_rotational_speed (0.0),
    m_slide (0.0),
    m_hardness (hardness),
    m_temperature (345.0),
    m_wear (0.0),
    m_velocity (0.0, 0.0, 0.0),
    m_normal_angular_velocity (0.0),
    m_normal_force (0.0),
    m_camber (0.0),
    m_applied_torque (0.0),
    m_is_locked (false)
{
}

// Called by the wheel to update the tire.
void
Tire::input (const Three_Vector& velocity, 
			 double normal_angular_velocity,
			 const Three_Vector& normal_force,
			 double camber,
			 double torque,
			 bool is_locked,
			 const Material& surface_material)
{
  orient_frame_with_unit_vector (normal_force.unit ());
  m_velocity = rotate_from_parent (velocity);
  m_normal_angular_velocity = normal_angular_velocity;
  m_normal_force = normal_force.magnitude ();
  m_camber = camber;
  m_applied_torque = torque;
  m_is_locked = is_locked;
  m_surface_material = surface_material;
}

// Orient the tire's z-axis with the normal force.
void
Tire::orient_frame_with_unit_vector (const Three_Vector& normal_unit_vector)
{
  Three_Vector rotation_axis = 
	Three_Vector (-normal_unit_vector.y, normal_unit_vector.x, 0.0);

  double length = sqrt (normal_unit_vector.x * normal_unit_vector.x
						+ normal_unit_vector.y * normal_unit_vector.y);
  double rotation_angle = asin (length);

  set_orientation (Three_Matrix ());
  rotate (rotation_axis.unit () * rotation_angle);
}

void
Tire::find_forces ()
{
  // m_force is the force of the road on the tire.  The force of the
  // tire on the body must be calculated.  The transverse component
  // won't change, but the longitudial component will be affected by
  // the tire's ability to move in that direction, and by applied
  // foces (acceleration and braking).

  // Skip this step if we don't have a surface yet.
  if (m_surface_material.type () == Material::UNKNOWN)
      return;

  m_slide = 0.0;

  if (m_normal_force <= 0.0)
	{
      Particle::reset ();
	  return;
	}

  // Get the friction force (components 0 and 1) and the aligning
  // torque (component 2).
  Three_Vector friction_force =
    m_tire_friction.friction_forces (m_normal_force * grip (),
                                     m_surface_material.friction_factor (),
                                     m_velocity, 
                                     speed(), 
                                     m_camber);

  // the frictional force opposing the motion of the contact patch.
  set_force (Three_Vector (friction_force.x, friction_force.y, 0.0));

  // Apply the reaction torque from acceleration or braking.  In
  // normal conditions this torque comes from friction.  However, the
  // frictional force can sometimes be large when no acceleration or
  // braking is applied.  For instance, when you run into a gravel
  // trap.  In any case, the reaction torque should never be larger
  // than the applied accelerating or braking torque. 
  double reaction = force ().x * m_radius;
  if (((m_applied_torque > 0.0) && (reaction > m_applied_torque))
	  || ((m_applied_torque < 0.0) && (reaction < m_applied_torque)))
	{
	  reaction = m_applied_torque;
	}

  set_torque (Three_Vector (0.0, reaction, friction_force.z));

  if (!m_is_locked)
	{
	  double rolling_1 = m_rolling_resistance_1;
	  if (speed () < 0.0)
		rolling_1 *= -1.0;
	  // Include constant and quadratic rolling resistance.
	  double rolling = m_surface_material.rolling_resistance_factor () 
		* (rolling_1 + m_rolling_resistance_2 * speed () * speed ());
	  m_applied_torque -= (force ().x + rolling) * m_radius;
	}

  // Add the suface drag.
  set_force (force () 
             - m_surface_material.drag_factor () 
             * Three_Vector (m_velocity.x, m_velocity.y, 0.0));

  m_slide = m_tire_friction.slide ();
}

// Advance this suspension component forward in time by TIME.
void
Tire::propagate (double time) 
{
  m_last_rotational_speed = m_rotational_speed;
  if (m_is_locked)
	{
	  // This causes speed() to return 0.0.
      m_rotational_speed = 0.0;
	}
  else
	{
	  m_rotational_speed += m_applied_torque * time / m_inertia;
	}

  // My made-up model of tire heating and wear.
  static const double stress_heating = 2e-4;
  static const double abrasion_heating = 1e-1;
  static const double wear = 1e-8;
  static const double cooling = 5e-3;

  const double dT = m_temperature - ambient_temperature;
  if (m_slide > 0.0)
    {
      // Forces applied through the tire flex, stretch, and compress the rubber
      // heating it up.  Slipping results in heating due to sliding friction.
      const double F = (force () + m_normal_force * Three_Vector::Z).magnitude ();
      const double friction = m_slide * m_surface_material.friction_factor ();
      const double heat = time * (stress_heating * F / m_hardness 
                                  + abrasion_heating * friction);
      m_temperature += heat;

      // Slipping wears the tire through abrasion.  The tire wears more quickly
      // at high temperature.
      m_wear += time * wear * friction * pow (dT, 2);
    }

  // Cooling is proportional to difference from ambient.
  m_temperature -= time * cooling * dT;
}

double Tire::grip () const
{
  return std::max (m_temperature / 380.0 - m_wear, 0.3);
}

void
Tire::rewind ()
{
  m_rotational_speed = m_last_rotational_speed;
}

// Fill in the longitudinal (sigma) and transverse (alpha) slip ratios.
void 
Tire::slip (double* sigma, double* alpha) const
{
  m_tire_friction.slip (speed (), m_velocity, sigma, alpha);
}

// Return the position of the contact patch in the wheel's coordinate
// system.
Three_Vector
Tire::contact_position () const
{
  return Three_Vector (0.0, 0.0, -m_radius);
}

// Set the tire to its initial conditions.
void
Tire::reset ()
{
  Particle::reset ();
  m_rotational_speed = 0.0;
}
