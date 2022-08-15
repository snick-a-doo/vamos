//  Suspension.cc - the suspension component for a wheel.
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

#include "../media/model.h"
#include "../geometry/conversions.h"
#include "../geometry/numeric.h"
#include "suspension.h"

#include <cmath>
#include <cassert>

using namespace Vamos_Body;
using namespace Vamos_Geometry;

//* Static Member

// The axis of rotation for steering and toe adjustments.
const Three_Vector Vamos_Body::Suspension::
STEER_AXIS = Three_Vector (0.0, 0.0, 1.0);

// Note that all angles are stored as right-hand rotations.  As a
// result, m_caster for a wheel on the right side of the car follows
// the common convention that positive camber means that the wheel
// leans away from the centerline.  For the wheel on the left,
// m_caster is contrary to convention.
Hinge::Hinge (const Three_Vector& position,
              const Frame* parent) 
  : Particle (0.0, position, parent)
{
}

void
Hinge::input (const Three_Vector& torque, const Three_Vector& radius)
{
  set_force (torque.magnitude () 
             / radius.magnitude () * (torque.cross (radius).unit ()));
}

//* Struct Suspension_Model
struct Vamos_Body::Suspension_Model
{
  GLuint display_list;
  double x;
  double y;
  double z;

  Suspension_Model (GLuint list_id, const Three_Vector& position)
	: display_list (list_id),
	  x (position.x),
	  y (position.y),
	  z (position.z)
  {
  }
};

//* Class Suspension

//** Constructor
Suspension::Suspension (const Three_Vector& pos,
						const Three_Vector& center_of_translation,
						Direction side_of_car, 
                        double spring_constant, 
						double bounce, 
                        double rebound, 
                        double travel, 
						double max_compression_velocity,
                        const Frame* parent) 
  : Particle (0.0, pos, parent),
    mp_hinge (new Hinge (center_of_translation)),
    m_radius (center_of_translation - pos),
    m_initial_radius (m_radius),
    m_radius_magnitude (m_radius.magnitude ()),
    m_initial_z (pos.z),
    m_spring_constant (spring_constant),
    m_bounce (bounce),
    m_rebound (rebound),
    m_travel (travel),
    m_displacement (0.0),
    m_time_step (0.0),
    m_compression_velocity (0.0),
    m_max_compression_velocity (max_compression_velocity),
    m_bottomed_out (false),
    m_anti_roll_k (0.0),
    m_anti_roll_suspension (0),
    m_steer_angle (0.0),
    m_camber (0.0),
    m_caster (0.0),
    m_toe (0.0),
    m_side (side_of_car),
    m_normal (Three_Vector (0.0, 0.0, 1.0)),
    m_hinge_axis (m_radius.cross (Three_Vector::Z).unit ())
{
}

Suspension::~Suspension ()
{
  for (std::vector <Suspension_Model*>::iterator it = m_models.begin ();
	   it != m_models.end ();
	   it++)
	{
	  delete *it;
	}
}

// Specify the suspension component that is attached to this one with
// an anti-roll bar.  The anti-roll bar will have a spring constant of
// SPRING_CONSTANT.
void
Suspension::anti_roll (Suspension* other, double spring_constant)
{
  m_anti_roll_suspension = other;
  m_anti_roll_k = spring_constant;

  m_anti_roll_suspension->m_anti_roll_suspension = this;
  m_anti_roll_suspension->m_anti_roll_k = m_anti_roll_k;
}

// Displace this suspension component by DISTANCE.  A positive
// DISTANCE means compression.
void
Suspension::displace (double distance)
{
  const double last_displacement = m_displacement;
  m_displacement = distance;
  if (m_displacement > m_travel)
	{
	  m_bottomed_out = true;
	  m_displacement = m_travel;
	}
  else
	{
	  m_bottomed_out = false;
	}

  set_position (get_position ());

  // The radius points from position () to the hinge.
  m_radius = mp_hinge->position () - position ();

  m_compression_velocity = (m_displacement - last_displacement) / m_time_step;
}

// Return the suspension position for the current displacement.
Three_Vector
Suspension::get_position () const
{
  const Three_Vector& hinge_pos = mp_hinge->position ();
  const double z = hinge_pos.z - m_initial_z - m_displacement;
  assert (z <= m_radius_magnitude);
  const double angle = asin (z / m_radius_magnitude);
  return hinge_pos - Vamos_Geometry::rotate(m_initial_radius, angle * m_hinge_axis);
}

void
Suspension::input (const Three_Vector& wheel_force,
	   const Three_Vector& normal)
{
  m_wheel_force = wheel_force;
  m_normal = rotate_to_parent (normal);
}

void
Suspension::torque (double wheel_torque)
{
  mp_hinge->input (Three_Vector (0.0, -wheel_torque, 0.0), m_radius);
}

// Calculate the force exerted by the suspension in its current state.
void
Suspension::find_forces ()
{
  double anti_roll_force = 0.0;
  if (m_anti_roll_suspension)
	{
	  anti_roll_force = m_anti_roll_k *
		(m_displacement - m_anti_roll_suspension->m_displacement);
	}

  // Use `m_bounce' for compression, `m_rebound' for decompression.
  double damp = (m_compression_velocity > 0.0) ? m_bounce : m_rebound;

  if (m_displacement <= 0.0)
	{
	  // Don't exert a force if this suspension is not compressed.
	  reset ();
	}
  else
	{
	  // If the suspension is moving at a speed > m_max_compression_velocity,
	  // the damper 'locks up' due to turbulence in the fluid.  The effect
	  // is the same as bottoming out.
	  if (std::abs (m_compression_velocity) > m_max_compression_velocity)
		{
		  m_bottomed_out = true;
		}

	  double spring_force = m_spring_constant * m_displacement;
	  double damp_force = damp * m_compression_velocity;
	  set_force (rotate_from_parent (m_normal 
                                    * (spring_force + damp_force + anti_roll_force)));
	}
}

// Advance this suspension component forward in time by TIME.
void
Suspension::propagate (double time)
{
  m_time_step = time;

  // Start with the static orientation.
  set_orientation (m_static_orientation);
  rotate (m_steer_angle * STEER_AXIS);
}

// Undo the last propagation.
void
Suspension::rewind ()
{
}

// Set the steering angle.
void
Suspension::steer (double degree_angle)
{
  m_steer_angle = deg_to_rad (degree_angle);
}

// Set the camber angle.
void
Suspension::camber (double degree_angle)
{
  if (m_side == LEFT)
	degree_angle *= -1.0;

  // Undo the current camber setting before applying the new one.
  m_static_orientation.rotate (Three_Vector (-m_camber, 0.0, 0.0));
  m_camber = deg_to_rad (degree_angle);
  m_static_orientation.rotate (Three_Vector (m_camber, 0.0, 0.0));
}

// Set the caster angle.
void
Suspension::caster (double degree_angle)
{
  // The caster rotation is in the same direction for both sides.

  // Undo the current caster setting before applying the new one.
  m_static_orientation.rotate (Three_Vector (0.0, -m_caster, 0.0));
  m_caster = -deg_to_rad (degree_angle);
  m_static_orientation.rotate (Three_Vector (0.0, m_caster, 0.0));
}

// Set the toe angle.
void
Suspension::toe (double degree_angle)
{
  if (m_side == LEFT)
  	degree_angle *= -1.0;

  // Undo the current toe setting before applying the new one.
  m_static_orientation.rotate (-m_toe * STEER_AXIS);
  m_toe = deg_to_rad (degree_angle);
  m_static_orientation.rotate (m_toe * STEER_AXIS);
}

double Suspension::camber_function (double) const
{
  return 0.0;
}

double 
Suspension::current_camber (double normal_y) const
{
  return Vamos_Geometry::clip (normal_y, -0.5, 0.5);;
}

// Return this suspension component to equilibrium.
void
Suspension::reset ()
{
  Particle::reset ();
  m_displacement = 0.0;
}


void
Suspension::set_model (std::string file_name,
					   double scale,
					   const Three_Vector& translation, 
					   const Three_Vector& rotation)
{
  Three_Vector position = translation;
  Three_Vector orientation = rotation;
  if (m_side == LEFT)
	{
	  // Make the right and left sides symmetric.
	  position.y *= -1.0;
	  orientation.x *= -1.0;
	  orientation.y *= -1.0;
	}

  Vamos_Media::Ac3d* model = 
    new Vamos_Media::Ac3d (file_name, scale, Three_Vector (), orientation);
  m_models.push_back (new Suspension_Model (model->build (), position));
  delete model;
}

void
Suspension::draw ()
{
  for (std::vector <Suspension_Model*>::iterator it = m_models.begin ();
	   it != m_models.end ();
	   it++)
	{
	  glPushMatrix ();

	  glTranslatef (position ().x + (*it)->x, 
					position ().y + (*it)->y,
					position ().z + (*it)->z - m_displacement);

	  double angle = rad_to_deg (std::atan2 (-m_displacement, (*it)->y));
 	  glRotatef (angle, 1.0, 0.0, 0.0);

	  glCallList ((*it)->display_list);
	  glPopMatrix ();
	}
}
