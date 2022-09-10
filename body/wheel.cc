//  Wheel.cc - a wheel.
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

#include "wheel.h"
#include "../geometry/conversions.h"
#include "../media/model.h"

using namespace Vamos_Body;
using namespace Vamos_Geometry;

//* Class Wheel

//** Constructor
Wheel::Wheel (double mass, 
			  Three_Vector position, 
			  double tire_offset,
			  double roll_height,
			  double restitution,
			  std::shared_ptr<Suspension> suspension,
			  const Tire& tire, 
			  const Brake& brake,
			  bool steered,
			  bool driven,
			  Side side) :
  // Friction is handled by the tire.
    Particle{mass, position, Material{Material::RUBBER, 0.0, restitution}},
  m_original_position (position),
  m_tire_offset (side == Side::right ? -tire_offset : tire_offset),
  m_roll_height (roll_height),
  mp_suspension (suspension),
  m_tire (tire),
  m_brake (brake),
  m_ground_velocity (0.0, 0.0, 0.0),
  m_normal (Three_Vector (0.0, 0.0, 0.0)),
  m_angular_velocity (Three_Vector (0.0, 0.0, 0.0)),
  m_drive_torque (0.0),
  m_braking_torque (0.0),
  m_steered (steered),
  m_driven (driven),
  m_side (side),
  m_slow_wheel_list (0),
  m_fast_wheel_list (0),
  m_stator_list (0),
  m_transition_speed (10.0),
  m_rotation (0.0)
{
}

void
Wheel::find_forces ()
{
    if (!is_in_contact())
	{
      Particle::reset ();
	  set_position (m_original_position);
	  mp_suspension->reset ();
	}

  m_tire.input (m_ground_velocity, 
				m_angular_velocity.z,
				mp_suspension->force ().project (m_normal),
				mp_suspension->current_camber (m_normal.unit ().y),
				m_drive_torque + m_braking_torque,
				m_brake.is_locked (), 
				m_surface_material);

  m_tire.find_forces ();
  set_force (m_tire.force ());
  set_torque (Three_Vector (m_tire.torque ().x, -m_tire.torque ().y, m_tire.torque ().z));
}

// Advance the wheel forward in time by TIME.
void
Wheel::propagate (double time)
{
  m_tire.propagate (time);
  set_orientation (mp_suspension->orientation ());
  m_rotation += speed () * time / m_tire.radius ();
}

double Wheel::contact(const Three_Vector& impulse, const Three_Vector& velocity, double distance,
                      const Three_Vector& normal, const Three_Vector& angular_velocity,
                      const Material& surface_material)
{
    Particle::contact(impulse, rotate_from_parent(velocity), distance,
                      rotate_from_parent(normal), rotate_from_parent(angular_velocity),
                      surface_material);
    if (!mp_suspension->bottomed_out())
        set_impulse(Three_Vector::ZERO);

    m_normal = rotate_from_parent(normal);
    const Three_Vector v_perp = rotate_from_parent(velocity).project(m_normal);
    m_ground_velocity = rotate_from_parent(velocity) - v_perp;
    m_angular_velocity = angular_velocity;

    mp_suspension->displace(((m_normal * distance).back_project(Three_Vector::Z)).magnitude());
    set_position(m_original_position + mp_suspension->displacement() * Three_Vector::Z);
    // The suspension displacement may be different from the argument to
    // displace() if it bottoms out.  Ask the suspension for the actual
    // displacement.

    mp_suspension->input(force(), m_normal);
    mp_suspension->torque(m_braking_torque);
    m_surface_material = surface_material;
    return -mp_suspension->displacement();
}

void
Wheel::drive_torque (double torque_in)
{
  m_drive_torque = torque_in;
}

// Return the torque exerted on the wheel by the brake when a pressure
// of FACTOR * Brake::m_max_pressure is applied to the brake.
void
Wheel::brake (double factor)
{
  m_braking_torque = -m_brake.torque (factor, m_tire.rotational_speed ());
}

// Return the position relative to the body where the wheel exerts its
// force.
Three_Vector
Wheel::force_position () const
{
  return position () + m_tire.contact_position () 
	+ Three_Vector (0.0, m_tire_offset, m_roll_height);
}

// Return the position of the wheel relative to the body for the
// purpose of detecting collisions.
Three_Vector
Wheel::contact_position () const
{
  return m_original_position + m_tire.contact_position ()
	+ Three_Vector (0.0, m_tire_offset, 0.0);
}

Two_Vector
Wheel::slip () const
{
  Two_Vector slips;
  m_tire.slip (&slips.x, &slips.y);
  return slips;
}

// Return the wheel to its initial conditions.
void
Wheel::reset ()
{
  Particle::reset ();
  m_tire.reset ();
}

GLuint
Wheel::make_model (std::string file, double scale, 
				   const Three_Vector& translation, 
				   const Three_Vector& rotation)
{
  Vamos_Media::Ac3d* model = 
    new Vamos_Media::Ac3d (file, scale, translation, rotation);
  GLuint display_list = model->build ();
  delete model;
  return display_list;
}

void
Wheel::set_models (std::string slow_file, 
				   std::string fast_file, 
				   double transition_speed,
				   std::string stator_file,
				   double stator_offset,
				   double scale,
				   const Three_Vector& translation,
				   const Three_Vector& rotation)
{
  Three_Vector offset;
  if (stator_file != "")
	{
        offset.y += (m_side == Side::right) ? stator_offset : -stator_offset;
	}

  if (m_slow_wheel_list != 0)
	{
	  glDeleteLists (m_slow_wheel_list, 1);
	}
  m_slow_wheel_list = 
	make_model (slow_file, scale, translation + offset, rotation);

  if (m_fast_wheel_list != 0)
	{
	  glDeleteLists (m_fast_wheel_list, 1);
	}
  m_fast_wheel_list = 
	make_model (fast_file, scale, translation + offset, rotation);

  m_transition_speed = transition_speed;

  if (stator_file != "")
	{
	  if (m_stator_list != 0)
		{
		  glDeleteLists (m_stator_list, 1);
		}
	  m_stator_list = make_model (stator_file, scale, translation, rotation);
	}
}

void Wheel::transform()
{
    glTranslatef(position().x, position().y, position().z);
    double angle;
    auto axis{axis_angle(angle)};
    glRotatef(angle, axis.x, axis.y, axis.z);
}

// Draw the wheel.
void
Wheel::draw ()
{
  glPushMatrix ();
  transform ();
  glCallList (m_stator_list);
  if (speed () < m_transition_speed)
	{
	  glRotatef (rad_to_deg (m_rotation), 0.0, 1.0, 0.0);
	  glCallList (m_slow_wheel_list);
	}
  else
	{
	  glCallList (m_fast_wheel_list);
	}
  glPopMatrix ();

  mp_suspension->draw ();
}
