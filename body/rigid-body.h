//  Copyright (C) 2001-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_BODY_RIGID_BODY_H_INCLUDED
#define VAMOS_BODY_RIGID_BODY_H_INCLUDED

#include "particle.h"
#include "frame.h"

#include "../geometry/material.h"
#include "../geometry/three-matrix.h"
#include "../geometry/three-vector.h"

#include <memory>
#include <vector>

namespace Vamos_Body
{
class Drag;

struct Contact_Parameters
{
    Particle* particle{nullptr};
    Vamos_Geometry::Three_Vector impulse;
    double m_distance{0.0};
    Vamos_Geometry::Three_Vector normal;
    Vamos_Geometry::Material material;
};

class Rigid_Body : public Frame
{
public:
    /// Specify the position and orientation of the body.
    Rigid_Body(const Vamos_Geometry::Three_Vector& pos, const Vamos_Geometry::Three_Matrix& orient);
    virtual ~Rigid_Body() = default;

    // Set the state for starting and resetting.
    void set_initial_conditions(const Vamos_Geometry::Three_Vector& position,
                                const Vamos_Geometry::Three_Vector& orientation,
                                const Vamos_Geometry::Three_Vector& velocity,
                                const Vamos_Geometry::Three_Vector& angular_velocity);

    Vamos_Geometry::Three_Vector center_of_mass() const { return m_body_cm; }

    // Return the position of the center of mass of the body with
    // respect to the world.
    Vamos_Geometry::Three_Vector cm_position() const;

    // Provide access to the frame's position method.  Return the
    // origin of the body.
    Vamos_Geometry::Three_Vector position() const { return Frame::position(); }

    // Return the contact position of the particle with respect to the
    // world.
    Vamos_Geometry::Three_Vector contact_position(Particle const& p) const;
    Vamos_Geometry::Three_Vector position(Particle const& p) const;

    // Return the smallest contact position z-value of the particles.
    double lowest_contact_position() const;

    // Add a particle to the body.
    void add_particle(std::shared_ptr<Particle> particle);

    // Calculate the center of mass, the ineritia tensor, and its
    // inverse.
    void update_center_of_mass();

    // Provide access to the particles.
    std::vector<std::shared_ptr<Particle>>& particles() { return m_particles; }

    void find_forces();

    /// @return The total aerodynamic drag.
    double drag() const;
    /// @return The total aerodynamic lift.
    double lift() const;

    // Advance the body in time by TIME.
    void propagate(double time);

    // Finish the timestep.
    void end_timestep();

    // Called by the world to tell the body what the acceleration due to gravity is.
    void set_gravity(Vamos_Geometry::Three_Vector const& grav) { m_gravity = grav; }

    // Return the velocity of the particle in the parent frame.
    Vamos_Geometry::Three_Vector velocity(Particle const& p) const;
    // Return the velocity of a point in the body's frame.
    Vamos_Geometry::Three_Vector velocity(const Vamos_Geometry::Three_Vector& r) const;
    /// Return the acceleration of the center of mass is the body's frame.
    Vamos_Geometry::Three_Vector acceleration() const;

    // Return the velocity of the center of mass.
    Vamos_Geometry::Three_Vector cm_velocity() const { return m_cm_velocity; }

    // Set the velocity of the center of mass.
    void cm_velocity(const Vamos_Geometry::Three_Vector& vel) { m_cm_velocity = vel; }

    void set_cm_force(const Vamos_Geometry::Three_Vector& force) { m_cm_force = force; }

    Vamos_Geometry::Three_Vector moment(Vamos_Geometry::Three_Vector const& pos) const;
    Vamos_Geometry::Three_Vector world_moment(Vamos_Geometry::Three_Vector const& world_pos) const;

    /// @return The body's inertia tensor.
    Vamos_Geometry::Three_Matrix inertia() const;

    // Handle a collision.
    void contact(Particle& particle, const Vamos_Geometry::Three_Vector& impulse,
                 const Vamos_Geometry::Three_Vector& velocity, double distance,
                 const Vamos_Geometry::Three_Vector& normal,
                 const Vamos_Geometry::Material& material);

    void temporary_contact(const Vamos_Geometry::Three_Vector& position,
                           const Vamos_Geometry::Three_Vector& impulse,
                           const Vamos_Geometry::Three_Vector& velocity, double distance,
                           const Vamos_Geometry::Three_Vector& normal,
                           const Vamos_Geometry::Material& material);

    // Transform the wind into the body frame and send it to the
    // aerodynamic devices.
    void wind(const Vamos_Geometry::Three_Vector& wind_vector, double density);

    // Return the total mass.
    double mass() const { return m_mass; }

    // This function is defined by subclasses that work with a
    // graphics system.
    virtual void draw(){};

    // Return the body to its initial state at its initial position.
    virtual void reset(double direction);

    // Return the body to its initial state at a particular position and
    // orientation.
    virtual void reset(const Vamos_Geometry::Three_Vector& position,
                       const Vamos_Geometry::Three_Matrix& orientation);

protected:
    /// The inertia tensor for the body.
    Vamos_Geometry::Three_Matrix m_inertia{0.0};

    // A vector of pointers to the force and torque producers that
    // make up the body
    std::vector<std::shared_ptr<Particle>> m_particles;

    // The position of the center of mass of the body relative to the
    // origin of the body.
    Vamos_Geometry::Three_Vector m_body_cm;

private:
    // The body's initial state, used by reset ().
    Vamos_Geometry::Three_Vector m_initial_position;
    Vamos_Geometry::Three_Vector m_initial_velocity;
    Vamos_Geometry::Three_Matrix m_initial_orientation{1.0};
    Vamos_Geometry::Three_Vector m_initial_angular_velocity;

    Vamos_Geometry::Three_Vector m_acceleration;

    // The velocity of the center of mass.
    Vamos_Geometry::Three_Vector m_cm_velocity;

    // The acceleration due to gravity, distance/time^2.  The units
    // determine the distance and time units used in the rest of the
    // simulation.
    Vamos_Geometry::Three_Vector m_gravity;

    // The elapsed time since the last time step.
    double m_delta_time{0.0};

    // The total mass of the body.
    double m_mass{0.0};

    // Common code for the two reset () methods.
    void private_reset();

    Contact_Parameters m_contact_parameters;
    std::vector<Particle> m_temporary_contact_points;
    void remove_temporary_contact_points();
    std::vector<std::shared_ptr<Drag>> m_drag_particles;
    Vamos_Geometry::Three_Vector m_cm_force;
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_RIGID_BODY_H_INCLUDED
