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

#include "../geometry/three-matrix.h"
#include "../geometry/three-vector.h"
#include "../media/material.h"

#include <memory>
#include <vector>

namespace Vamos_Body
{
class Drag;

class Rigid_Body : public Frame
{
    friend class Car_Reader;

public:
    /// Specify the position and orientation of the body.
    Rigid_Body(Vamos_Geometry::Three_Vector const& pos,
               Vamos_Geometry::Three_Matrix const& orient);
    virtual ~Rigid_Body() = default;

    /// Set the state for starting and resetting.
    void set_initial_conditions(Vamos_Geometry::Three_Vector const& position,
                                Vamos_Geometry::Three_Vector const& orientation,
                                Vamos_Geometry::Three_Vector const& velocity,
                                Vamos_Geometry::Three_Vector const& angular_velocity);
    /// Add a particle to the body.
    void add_particle(std::shared_ptr<Particle> particle);

    /// @return the total mass.
    double mass() const { return m_mass; }
    /// @return The body's inertia tensor.
    Vamos_Geometry::Three_Matrix inertia() const { return m_inertia; }
    /// @return The world-frame center of mass.
    Vamos_Geometry::Three_Vector cm_position() const;
    /// @return The world-frame contact position of a particle.
    Vamos_Geometry::Three_Vector contact_position(Particle const& p) const;
    /// @return The world-frame position of a particle.
    Vamos_Geometry::Three_Vector particle_position(Particle const& p) const;
    /// @return The displacement from a point to the center of mass in the world frame.
    Vamos_Geometry::Three_Vector moment(Vamos_Geometry::Three_Vector const& pos) const;
    // Return the velocity of the particle in the parent frame.
    Vamos_Geometry::Three_Vector particle_velocity(Particle const& p) const;
    // Return the velocity of a point in the body's frame.
    Vamos_Geometry::Three_Vector point_velocity(Vamos_Geometry::Three_Vector const& r) const;
    /// @return The body-frame acceleration of the center of mass.
    Vamos_Geometry::Three_Vector acceleration() const;
    // Return the velocity of the center of mass.
    Vamos_Geometry::Three_Vector cm_velocity() const { return m_cm_velocity; }
    /// @return The total aerodynamic drag.
    double drag() const;
    /// @return The total aerodynamic lift.
    double lift() const;
    /// @return The particles.
    std::vector<std::shared_ptr<Particle>> const& particles() const { return m_particles; }

    /// Called by the world to tell the body what the acceleration due to gravity is.
    void set_gravity(Vamos_Geometry::Three_Vector const& grav) { m_gravity = grav; }
    /// Called when a body particle contacts another object.
    void contact(Particle& particle, Vamos_Geometry::Three_Vector const& impulse,
                 Vamos_Geometry::Three_Vector const& velocity, double distance,
                 Vamos_Geometry::Three_Vector const& normal,
                 Vamos_Media::Material const& material);
    /// Called when another object makes contact.
    void temporary_contact(Vamos_Geometry::Three_Vector const& position,
                           Vamos_Geometry::Three_Vector const& impulse,
                           Vamos_Geometry::Three_Vector const& velocity, double distance,
                           Vamos_Geometry::Three_Vector const& normal,
                           Vamos_Media::Material const& material);
    /// Transform the wind into the body frame and send it to the aero particles.
    void wind(Vamos_Geometry::Three_Vector const& wind_vector, double density);

    /// Advance the body in time.
    void propagate(double time);
    /// Set the body to its initial state.
    void reset(double direction);
    /// Set the body to its initial state at a particular position and orientation.
    void reset(Vamos_Geometry::Three_Vector const& position,
               Vamos_Geometry::Three_Matrix const& orientation);

private:
    /// Calculate the center of mass, the ineritia tensor, and its inverse.
    void update_center_of_mass();

    Vamos_Geometry::Three_Vector m_gravity; ///< The acceleration due to gravity vector.
    double m_mass{0.0}; ///< The total mass of the body.
    Vamos_Geometry::Three_Matrix m_inertia{0.0}; ///< The inertia tensor for the body.
    std::vector<std::shared_ptr<Particle>> m_particles; ///< The masses that make up the body.
    /// Massless particles that exert aerodynamic forces.
    std::vector<std::shared_ptr<Drag>> m_aero_particles;
    Vamos_Geometry::Three_Vector m_body_cm; ///< The body-frame position of the center of mass.
    Vamos_Geometry::Three_Vector m_cm_velocity; ///< The velocity of the center of mass.
    Vamos_Geometry::Three_Vector m_acceleration; ///< Acceleration of the center of mass.
    Frame m_initial_frame; ///< The body's initial state.

    /// Information about collision of a particle with something else.
    struct Contact
    {
        Particle* particle{nullptr};
        Vamos_Geometry::Three_Vector impulse;
        Vamos_Media::Contact_Info info;
    };
    Contact m_contact_params; ///< The current deepest contact.
    ///< Particles added to exert collision forces from other objects.
    std::vector<Particle> m_temporary_contact;
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_RIGID_BODY_H_INCLUDED
