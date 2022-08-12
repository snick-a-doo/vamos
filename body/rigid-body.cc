//  Rigid_Body.cc - a rigid body.
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

#include "rigid-body.h"
#include "contact-point.h"
#include "aerodynamics.h"
#include "../geometry/conversions.h"

#include <cassert>

using namespace Vamos_Geometry;
using namespace Vamos_Body;

/// Add the contribution of a particle with the given mass and center-of-mass position to
/// an inertia tensor.
void add_inertia(double mass, Three_Vector const& pos, Three_Matrix& I)
{
    // I_xx I_xy I_xz
    I[0][0] += mass * (pos.y * pos.y + pos.z * pos.z);
    I[0][1] -= mass * (pos.x * pos.y);
    I[0][2] -= mass * (pos.x * pos.z);

    // I_yx I_yy I_yz
    I[1][0] = I[0][1];
    I[1][1] += mass * (pos.z * pos.z + pos.x * pos.x);
    I[1][2] -= mass * (pos.y * pos.z);

    // I_zx I_zy I_zz
    I[2][0] = I[0][2];
    I[2][1] = I[1][2];
    I[2][2] += mass * (pos.x * pos.x + pos.y * pos.y);
}

//* Struct Contact_Parameters

//** Constructor


Contact_Parameters::Contact_Parameters ()
  : m_distance (0.0)
{
}


Contact_Parameters::Contact_Parameters (Particle* contact_point,
                                        const Three_Vector& impulse,
                                        double distance,
                                        const Three_Vector& normal,
                                        const Material& material)
  : mp_contact_point (contact_point),
    m_impulse (impulse),
    m_distance (distance),
    m_normal (normal),
    m_material (material)
{
}

//* Class Body

//** Constructors

// Specify the position and orientation of the body.

Rigid_Body::Rigid_Body (const Three_Vector& pos, const Three_Matrix& orient) 
  : Frame (pos, orient),
    m_initial_position (pos),
    m_initial_orientation (orient),
    m_delta_time (0.0),
    m_mass (0.0)
{
}


//** Destuctor

Rigid_Body::~Rigid_Body ()
{
  // The body is responsible for deleting the partiles even though
  // they were constructed elsewhere
  m_drag_particles.clear ();
  for (std::vector <Particle*>::iterator it = m_particles.begin ();
       it != m_particles.end ();
       it++)
    {
      delete *it;
    }
  remove_temporary_contact_points ();
}

// Set the state for starting and resetting.
void 
Rigid_Body::set_initial_conditions (const Vamos_Geometry::Three_Vector& position,
                                    const Vamos_Geometry::Three_Vector& orientation,
                                    const Vamos_Geometry::Three_Vector& velocity,
                                    const Vamos_Geometry::Three_Vector& angular_velocity)
{
  m_initial_position = position;
  m_initial_velocity = velocity;
  m_initial_orientation.identity ();
  m_initial_orientation = Three_Matrix(1.0).rotate(orientation * deg_to_rad(1.0));
  m_initial_angular_velocity = angular_velocity * deg_to_rad (1.0);
  reset (0.0);
}

void 
Rigid_Body::add_drag_particle (Drag* const comp)
{ 
  m_drag_particles.push_back (comp); 
  add_particle (comp);
} 

// Return the position of the center of mass of the body with respect
// to the world.
Three_Vector 
Rigid_Body::cm_position () const
{
  return transform_to_parent (m_body_cm);
}

// Return the contact position of the INDEXth particle of
// `m_particles' with respect to the world.
Three_Vector 
Rigid_Body::contact_position (Particle* contact_point)
{
  return transform_to_parent (contact_point->contact_position ());
}
Three_Vector 
Rigid_Body::position (Particle* p)
{
  return transform_to_parent (p->position ());
}

// Return the smallest contact position z-value of the particles.
double 
Rigid_Body::lowest_contact_position () const
{
  std::vector <Particle*>::const_iterator it = m_particles.begin ();
  double pos = transform_to_parent ((*it)->contact_position ()).z;
  double lowest = pos;

  for (it++; it != m_particles.end (); it++)
    {
      pos = transform_to_parent ((*it)->contact_position ()).z;
      if (pos < lowest)
        {
          lowest = pos;
        }
    }

  return lowest;
}

// Calculate the center of mass, the ineritia tensor, and its inverse.
void 
Rigid_Body::update_center_of_mass ()
{
  // Find the center of mass in the body frame.
  m_body_cm = Three_Vector (0.0, 0.0, 0.0);
  m_mass = 0.0;
  for (std::vector <Particle*>::const_iterator it = m_particles.begin ();
       it != m_particles.end ();
       it++)
    {
      m_mass += (*it)->mass ();
      // The particle reports its position in the body frame.
      m_body_cm += (*it)->mass_position () * (*it)->mass ();
    }
  m_body_cm /= m_mass;

  // Inertia tensor for rotations about the center of mass.
  m_inertia.zero();
  for (auto const* p : m_particles)
      add_inertia(p->mass(), p->mass_position() - m_body_cm, m_inertia);
}

void 
Rigid_Body::find_forces ()
{
  for (std::vector <Particle*>::const_iterator it = m_particles.begin ();
       it != m_particles.end ();
       it++)
    {
      (*it)->find_forces ();
    }
}

// Advance the body in time by TIME.
void 
Rigid_Body::propagate (double time)
{
  // Re-calculate the inertia tensor and center of mass.
  update_center_of_mass ();

  // Process single-collision contact.
  if (m_contact_parameters.m_distance > 0.0)
    {
      Particle* point = m_contact_parameters.mp_contact_point;
      Three_Vector world_v = velocity (point->position ());
      Three_Vector world_ang_v = angular_velocity ();

      m_contact_parameters.
        mp_contact_point->contact (rotate_from_parent (m_contact_parameters.m_impulse),
                                   rotate_from_parent (world_v),
                                   m_contact_parameters.m_distance, 
                                   rotate_from_parent (m_contact_parameters.m_normal), 
                                   rotate_from_parent (world_ang_v), 
                                   m_contact_parameters.m_material);

      translate (m_contact_parameters.m_distance * m_contact_parameters.m_normal);
      m_contact_parameters.m_distance = 0.0;
    }

  // Propagate the particles
  for (std::vector <Particle*>::iterator it = m_particles.begin ();
       it != m_particles.end ();
       it++)
    {
      (*it)->propagate (time);
    }
  for (std::vector <Contact_Point*>::iterator it = m_temporary_contact_points.begin ();
       it != m_temporary_contact_points.end ();
       it++)
    {
      (*it)->propagate (time);
    }
 
  // Move the body and the particles in response to forces applied to
  // them and their momenta, while keeping their relative positions
  // fixed.
  m_delta_time = time;
  Three_Vector total_force = m_cm_force;
  Three_Vector total_torque;

  for (std::vector <Particle*>::const_iterator it = m_particles.begin ();
       it != m_particles.end ();
       it++)
    {
      // Find the force that the particle exerts on the rest of the system.
      // The particle reports its force in the Body frame.
      Three_Vector body_force = (*it)->force () + (*it)->impulse () / time;
      total_force += body_force;

      // Find the force and torque that the particle exerts on the Body.
      // Find the vector from the cm to the particle in the world frame.
      Three_Vector torque_dist = m_body_cm - (*it)->torque_position ();
      Three_Vector torque = (*it)->torque ();
      double I = (m_inertia * torque.unit ()).magnitude ();
      torque *= I / (I + m_mass * torque_dist.dot (torque_dist));
      Three_Vector force_dist = m_body_cm - (*it)->force_position ();
      total_torque += torque - force_dist.cross (body_force);
    }

  // Transform the forces to the parent's coordinates so we can find
  // out how the Body moves w.r.t its parent.
  total_force = rotate_to_parent (total_force) + m_gravity * m_mass;

  Three_Vector delta_omega = time * total_torque * invert(m_inertia);
  Three_Vector delta_theta = (angular_velocity () + delta_omega) * time;
  m_last_angular_velocity = angular_velocity ();
  angular_accelerate (delta_omega);

  m_acceleration = total_force / m_mass;
  Three_Vector delta_v = m_acceleration * time;
  Three_Vector delta_r = (m_cm_velocity + delta_v) * time;
  m_last_cm_velocity = m_cm_velocity;
  m_cm_velocity += delta_v;

  // Because the body's origin is not necessarily coincident with the
  // center of mass, the body's translation has a component that
  // depends on the orientation.  Place the Body by translating to the
  // cm, rotating and then translating back.
  m_last_position = position ();
  translate (orientation () * m_body_cm);

  // rotate() acts in the body frame.
  m_last_orientation = orientation ();
  rotate (delta_theta);
  translate (orientation () * -m_body_cm + delta_r);

  // Determine the velocity of the origin.
  m_last_velocity = Frame::velocity ();
  set_velocity ((position () - m_last_position) / time);
}

// Undo the last propagation.
void 
Rigid_Body::rewind ()
{
  set_position (m_last_position);
  set_velocity (m_last_velocity);
  m_cm_velocity = m_last_cm_velocity;

  set_orientation (m_last_orientation);
  set_angular_velocity (m_last_angular_velocity);
}

// Finish the timestep.
void 
Rigid_Body::end_timestep ()
{
  for (std::vector <Particle*>::iterator it = m_particles.begin ();
       it != m_particles.end ();
       it++)
    {
      (*it)->end_timestep ();
    }
  remove_temporary_contact_points ();
  m_cm_force.zero ();
}

void 
Rigid_Body::remove_temporary_contact_points ()
{
  for (std::vector <Contact_Point*>::iterator it = m_temporary_contact_points.begin ();
       it != m_temporary_contact_points.end ();
       it++)
    {
      delete *it;
    }
  m_temporary_contact_points.clear ();
}

// Return the velocity of the particle in the parent frame.
Three_Vector 
Rigid_Body::velocity (Particle* particle)
{
  return velocity (particle->position ());
}

// Return the velocity of a body-frame point.
Three_Vector 
Rigid_Body::velocity (const Three_Vector& r)
{
  return m_cm_velocity + rotate_to_parent (angular_velocity ().cross (moment (r)));
}

Three_Vector Rigid_Body::acceleration () const
{
  return rotate_from_world (m_acceleration);
}

Three_Vector
Rigid_Body::moment (const Vamos_Geometry::Three_Vector& position)
{
  return position - m_body_cm;
}

Three_Vector
Rigid_Body::world_moment (const Vamos_Geometry::Three_Vector& world_position)
{
  return rotate_to_world (moment (transform_from_world (world_position)));
}

Three_Matrix Rigid_Body::inertia() const
{
    return m_inertia;
}

// Handle a collision.
void 
Rigid_Body::contact (Particle* contact_point, 
                     const Three_Vector& impulse,
                     const Three_Vector& velocity,
                     double depth,
                     const Three_Vector& normal,
                     const Vamos_Geometry::Material& material)
{
  if (contact_point->single_contact ())
    {
      if (depth > m_contact_parameters.m_distance)
        {
          m_contact_parameters = 
            Contact_Parameters (contact_point, 
                                impulse,
                                depth, 
                                normal, 
                                material);
        }
    }
  else
    {
      contact_point->contact (rotate_from_parent (impulse),
                              rotate_from_parent (velocity),
                              depth,
                              rotate_from_parent (normal),
                              rotate_from_parent (angular_velocity ().project (normal)),
                              material);
    }
}

void 
Rigid_Body::temporary_contact (const Three_Vector& position,
                               const Three_Vector& impulse,
                               const Three_Vector& velocity,
                               double depth,
                               const Three_Vector& normal,
                               const Vamos_Geometry::Material& material)
{
  Contact_Point* point = new Contact_Point (0.0,
                                            transform_from_parent (position),
                                            material.type (), 0.0, 1.0, 
                                            this);

  // The material, restitution and friction are not used for temporaries.
  // The impulse is calculated externally and passed in.
  point->contact (rotate_from_parent (impulse),
                  rotate_from_parent (velocity),
                  depth,
                  rotate_from_parent (normal),
                  rotate_from_parent (angular_velocity ().project (normal)),
                  material);

  m_temporary_contact_points.push_back (point);
}

// Transform the wind into the body frame and send it to the INDEXth
// particle which must be an Aerodynamic_Device.
void 
Rigid_Body::wind (const Three_Vector& wind_vector, 
                  double density)
{
  for (std::vector <Drag*>::iterator it = m_drag_particles.begin ();
       it != m_drag_particles.end ();
       it++)
    {
      (*it)->wind (rotate_from_parent (wind_vector - velocity (*it)),
                   density);
    }
}

double
Rigid_Body::aerodynamic_drag () const
{
  double drag = 0.0;
  for (std::vector <Drag*>::const_iterator it = m_drag_particles.begin ();
       it != m_drag_particles.end ();
       it++)
    {
      drag += (*it)->drag_factor ();
    }
  return drag;
}

double
Rigid_Body::aerodynamic_lift () const
{
  double lift = 0.0;
  for (std::vector <Drag*>::const_iterator it = m_drag_particles.begin ();
       it != m_drag_particles.end ();
       it++)
    {
      lift += (*it)->lift_factor ();
    }
  return lift;
}

// Return the body to its initial state at its initial position.
void 
Rigid_Body::reset (double direction)
{
  set_position (m_initial_position);
  Three_Matrix orient (m_initial_orientation);
  set_orientation (orient.rotate (Three_Vector (0.0, 0.0, direction)));
  private_reset ();
}

// Return the body to its initial state at a particular position and
// orientation.
void 
Rigid_Body::reset (const Three_Vector& position, 
                   const Three_Matrix& orientation)
{
  set_position (position);
  set_orientation (orientation);

  private_reset ();
}

// Common code for the two reset () methods.
void
Rigid_Body::private_reset ()
{
  m_cm_velocity = m_initial_velocity;
  set_velocity (m_cm_velocity 
                + m_initial_velocity.cross (moment (Three_Vector (0.0, 0.0, 0.0))));
  set_angular_velocity (m_initial_angular_velocity);

  for (std::vector <Particle*>::iterator it = m_particles.begin ();
       it != m_particles.end ();
       it++)
    {
      (*it)->reset ();
    }
}
