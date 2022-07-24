//  Rigid_Body.h - a rigid body.
//
//  Copyright (C) 2001--2002 Sam Varner
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

#ifndef _RIGID_BODY_H_
#define _RIGID_BODY_H_

#include "../geometry/three-vector.h"
#include "../geometry/three-matrix.h"
#include "../geometry/material.h"
#include "../geometry/inertia-tensor.h"
#include "frame.h"
#include "contact-point.h"

#include <vector>

namespace Vamos_Body
{
  class Drag;

  struct Contact_Parameters
  {
    Contact_Parameters ();
    Contact_Parameters (Particle* contact_point,
                        const Vamos_Geometry::Three_Vector& impulse,
                        double distance,
                        const Vamos_Geometry::Three_Vector& normal,
                        const Vamos_Geometry::Material& material);

    Particle* mp_contact_point;
    Vamos_Geometry::Three_Vector m_impulse;
    double m_distance;
    Vamos_Geometry::Three_Vector m_normal;
    Vamos_Geometry::Material m_material;
  };

  //* A rigid body.
  class Rigid_Body : public Frame
  {
    // The body's initial state, used by reset ().
    Vamos_Geometry::Three_Vector m_initial_position;
    Vamos_Geometry::Three_Vector m_initial_velocity;
    Vamos_Geometry::Three_Matrix m_initial_orientation;
    Vamos_Geometry::Three_Vector m_initial_angular_velocity;

    Vamos_Geometry::Three_Vector m_last_position;

    Vamos_Geometry::Three_Vector m_acceleration;

    // The velocity of the center of mass.
    Vamos_Geometry::Three_Vector m_cm_velocity;

    Vamos_Geometry::Three_Vector m_last_cm_velocity;

    Vamos_Geometry::Three_Vector m_last_velocity;

    Vamos_Geometry::Three_Matrix m_last_orientation;

    Vamos_Geometry::Three_Vector m_last_angular_velocity;

    // The acceleration due to gravity, distance/time^2.  The units
    // determine the distance and time units used in the rest of the
    // simulation.
    Vamos_Geometry::Three_Vector m_gravity;

    // The elapsed time since the last time step.
    double m_delta_time;

    // The total mass of the body.
    double m_mass;

    // Common code for the two reset () methods.
    void private_reset ();

    Contact_Parameters m_contact_parameters;

    std::vector <Contact_Point*> m_temporary_contact_points;

    void remove_temporary_contact_points ();

    std::vector <Drag*> m_drag_particles;

    Vamos_Geometry::Three_Vector m_cm_force;

  protected:
    // The inertia tensor for the body.
    Vamos_Geometry::Inertia_Tensor m_inertia;

    // A vector of pointers to the force and torque producers that
    // make up the body
    std::vector <Particle*> m_particles;

    // The position of the center of mass of the body relative to the
    // origin of the body.
    Vamos_Geometry::Three_Vector m_body_cm;

  public:
    //** Constructors

    // Specify the position and orientation of the body.
    Rigid_Body (const Vamos_Geometry::Three_Vector& pos, 
                const Vamos_Geometry::Three_Matrix& orient);

    //** Destructor
    virtual ~Rigid_Body ();

    // Set the state for starting and resetting.
    void set_initial_conditions (const Vamos_Geometry::Three_Vector& position,
                                 const Vamos_Geometry::Three_Vector& orientation,
                                 const Vamos_Geometry::Three_Vector& velocity,
                                 const Vamos_Geometry::Three_Vector& angular_velocity);

    Vamos_Geometry::Three_Vector center_of_mass () const { return m_body_cm; }

    // Return the position of the center of mass of the body with
    // respect to the world.
    Vamos_Geometry::Three_Vector cm_position () const;

    // Provide access to the frame's position method.  Return the
    // origin of the body.
    Vamos_Geometry::Three_Vector position () const 
    { return Frame::position (); }

    // Return the contact position of the particle with respect to the
    // world.
    Vamos_Geometry::Three_Vector contact_position (Particle* contact_point);
    Vamos_Geometry::Three_Vector position (Particle* contact_point);

    // Return the smallest contact position z-value of the particles.
    double lowest_contact_position () const;

    // Add a particle to the body.
    void add_particle (Particle* const comp) 
    { m_particles.push_back (comp); } 
    void add_drag_particle (Drag* const comp);

    // Calculate the center of mass, the ineritia tensor, and its
    // inverse.
    void update_center_of_mass ();

    // Provide access to the particles.
    std::vector <Particle*>& particles () { return m_particles; }

    void find_forces ();

    /// Return the sum of the drag factors.
    double aerodynamic_drag () const;

    /// Return the sum of the lift factors.
    double aerodynamic_lift () const;

    // Advance the body in time by TIME.
    void propagate (double time);

    // Undo the last propagation.
    void rewind ();

    // Finish the timestep.
    void end_timestep ();

    // Called by the world to tell the body what the acceleration due
    // to gravity is.
    void gravity (const Vamos_Geometry::Three_Vector& grav) { m_gravity = grav; }

    // Return the velocity of the particle in the parent frame.
    Vamos_Geometry::Three_Vector velocity (Particle* particle);
    // Return the velocity of a point in the body's frame.
    Vamos_Geometry::Three_Vector velocity (const Vamos_Geometry::Three_Vector& r);
    /// Return the acceleration of the center of mass is the body's frame.
    Vamos_Geometry::Three_Vector acceleration () const;

    // Return the velocity of the center of mass.
    Vamos_Geometry::Three_Vector cm_velocity () const { return m_cm_velocity; }

    // Set the velocity of the center of mass.
    void cm_velocity (const Vamos_Geometry::Three_Vector& vel) { m_cm_velocity = vel; }

    void set_cm_force (const Vamos_Geometry::Three_Vector& force) { m_cm_force = force; }

    Vamos_Geometry::Three_Vector moment (const Vamos_Geometry::Three_Vector& position);
    Vamos_Geometry::Three_Vector 
    world_moment (const Vamos_Geometry::Three_Vector& world_position);

    Vamos_Geometry::Inertia_Tensor inertia () const { return m_inertia; }

    // Handle a collision.
    void contact (Particle* contact_point, 
                  const Vamos_Geometry::Three_Vector& impulse,
                  const Vamos_Geometry::Three_Vector& velocity,
                  double distance,
                  const Vamos_Geometry::Three_Vector& normal,
                  const Vamos_Geometry::Material& material);

    void temporary_contact (const Vamos_Geometry::Three_Vector& position,
                            const Vamos_Geometry::Three_Vector& impulse,
                            const Vamos_Geometry::Three_Vector& velocity,
                            double distance,
                            const Vamos_Geometry::Three_Vector& normal,
                            const Vamos_Geometry::Material& material);

    // Transform the wind into the body frame and send it to the 
    // aerodynamic devices.
    void wind (const Vamos_Geometry::Three_Vector& wind_vector, 
               double density);

    // Return the total mass.
    double mass () const { return m_mass; }

    // This function is defined by subclasses that work with a
    // graphics system.
    virtual void draw () {};

    // Return the body to its initial state at its initial position.
    virtual void reset (double direction);

    // Return the body to its initial state at a particular position and
    // orientation.
    virtual void reset (const Vamos_Geometry::Three_Vector& position, 
                        const Vamos_Geometry::Three_Matrix& orientation);
  };
}

#endif // not _BODY_H_
