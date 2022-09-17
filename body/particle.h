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
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_BODY_PARTICLE_H_INCLUDED
#define VAMOS_BODY_PARTICLE_H_INCLUDED

#include "frame.h"

#include "../geometry/three-matrix.h"
#include "../geometry/three-vector.h"
#include "../media/material.h"

namespace Vamos_Body
{
// Particle is a point mass which may be part of a rigid body. It has position and
// orientation information inherited from Frame. A Particle can exert forces and torques.
class Particle : public Frame
{
public:
    /// Make a particle that takes part in collisions.
    /// @param position Position of the particle in the parent frame.
    /// @param material Material properties used in collisions.
    Particle(double mass,
             Vamos_Geometry::Three_Vector const& position,
             Vamos_Geometry::Material const& moterial);
    /// Make a particle that does not collide.
    Particle(double mass, Vamos_Geometry::Three_Vector const& position);
    /// The particle's frame is coincident with the parent's.
    Particle(double mass = 0.0);

    virtual ~Particle() = default;

    /// @return the force exerted on the rigid body in the body's frame.
    virtual Vamos_Geometry::Three_Vector force() const { return rotate_out(m_force); }
    /// @return the impulse exerted on the rigid body in the body's frame.
    virtual Vamos_Geometry::Three_Vector impulse() const { return rotate_out(m_impulse); }
    /// @return the torque exerted on the rigid body in the body's frame.
    virtual Vamos_Geometry::Three_Vector torque() const { return rotate_out(m_torque); }
    // Derived classes may give a position for detecting collisions that's different from
    // the physical position.
    virtual Vamos_Geometry::Three_Vector contact_position() const { return position(); }
    // Derived classes may give a position for exerting forces and torques that's
    // different from the physical position.
    virtual Vamos_Geometry::Three_Vector force_position() const { return position(); }

    /// Override to respond to contact with another object.
    /// @param impulse The change in momentum to give to the body.
    /// @param velocity This object's velocity relative to the other object.
    /// @param distance How far this object has penetrated the other.
    /// @param normal The normal to surface of the other object at the point of contact.
    /// @param ang_vel This object's angular velocity relative to the other
    /// object.
    /// @param material Material properties of the other object.
    virtual double contact(Vamos_Geometry::Three_Vector const& impulse,
                           Vamos_Geometry::Three_Vector const& velocity, double distance,
                           Vamos_Geometry::Three_Vector const& normal,
                           Vamos_Geometry::Three_Vector const& ang_vel,
                           Vamos_Geometry::Material const& material);

    // True for solid, rigid contact. Soft or deformable objects should return false.
    virtual bool single_contact() const { return true; }
    // Find and store the forces, impulses, and torques for the current configuration.
    virtual void find_forces();
    // Propagate the Particle forward in time by TIME.
    virtual void propagate(double /* time */){};
    // Do any necessary cleanup at the end of a time step.
    virtual void end_timestep();
    // Set the force, impulse and torque to zero;
    virtual void reset();

    /// @return the particle's mass.
    double mass() const { return m_mass; }
    /// @return The material properties.
    Vamos_Geometry::Material const& material() const { return m_material; }
    /// @return True if material properties have been set.
    bool can_contact() const;

protected:
    bool is_in_contact() const { return m_contact; }
    void set_mass(double new_mass) { m_mass = new_mass; }
    void set_force(const Vamos_Geometry::Three_Vector& new_force) { m_force = new_force; }
    void set_impulse(const Vamos_Geometry::Three_Vector& new_impulse) { m_impulse = new_impulse; }
    void set_torque(const Vamos_Geometry::Three_Vector& new_torque) { m_torque = new_torque; }

private:
    double m_mass{0.0}; ///< The mass of the particle.
    bool m_contact{false}; ///< true if a collision has occurred.
    Vamos_Geometry::Material m_material; ///< Material properties for the particle.
    Vamos_Geometry::Three_Vector m_force; ///< The force exerted by the component.
    Vamos_Geometry::Three_Vector m_impulse; ///< The impulse exerted by the component.
    Vamos_Geometry::Three_Vector m_torque; ///< The torque exerted by the component.
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_PARTICLE_H_INCLUDED
