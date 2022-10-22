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

#include "rigid-body.h"
#include "aerodynamics.h"
#include "particle.h"

#include "../geometry/conversions.h"

#include <cassert>
#include <numeric>

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

Rigid_Body::Rigid_Body(const Three_Vector& pos, const Three_Matrix& orient)
    : Frame(pos, orient),
      m_initial_position(pos),
      m_initial_orientation(orient)
{
}

void Rigid_Body::set_initial_conditions(const Vamos_Geometry::Three_Vector& position,
                                        const Vamos_Geometry::Three_Vector& orientation,
                                        const Vamos_Geometry::Three_Vector& velocity,
                                        const Vamos_Geometry::Three_Vector& angular_velocity)
{
    m_initial_position = position;
    m_initial_velocity = velocity;
    m_initial_orientation.identity();
    m_initial_orientation = Three_Matrix(1.0).rotate(orientation * deg_to_rad(1.0));
    m_initial_angular_velocity = angular_velocity * deg_to_rad(1.0);
    reset(0.0);
}

void Rigid_Body::add_particle(std::shared_ptr<Particle> particle)
{
    m_particles.push_back(particle);
    if (auto drag = std::dynamic_pointer_cast<Drag>(particle))
        m_drag_particles.push_back(drag);
    update_center_of_mass();
}

Three_Vector Rigid_Body::cm_position() const
{
    return transform_out(m_body_cm);
}

Three_Vector Rigid_Body::contact_position(Particle const& p) const
{
    return transform_out(p.contact_position());
}

Three_Vector Rigid_Body::position(Particle const& p) const
{
    return transform_out(p.position());
}

double Rigid_Body::lowest_contact_position() const
{
    auto lowest{std::numeric_limits<double>::max()};
    for (auto p : m_particles)
        if (p->can_contact())
            lowest = std::min(lowest, transform_out(p->contact_position()).z);
    return lowest;
}

void Rigid_Body::update_center_of_mass()
{
    // Find the center of mass in the body frame.
    m_body_cm.zero();
    m_mass = 0.0;
    for (auto p : m_particles)
    {
        m_mass += p->mass();
        // The particle reports its position in the body frame.
        m_body_cm += p->position() * p->mass();
    }
    m_body_cm /= m_mass;

    // Inertia tensor for rotations about the center of mass.
    m_inertia.zero();
    for (auto const& p : m_particles)
        add_inertia(p->mass(), p->position() - m_body_cm, m_inertia);
}

void Rigid_Body::find_forces()
{
    for (auto p : m_particles)
        p->find_forces();
}

void Rigid_Body::propagate(double time)
{
    // Re-calculate the inertia tensor and center of mass.
    update_center_of_mass();

    // Process single-collision contact.
    if (m_contact_parameters.m_distance > 0.0)
    {
        auto point{m_contact_parameters.particle};
        auto world_v{velocity(point->position())};
        auto world_omega{angular_velocity()};
        m_contact_parameters.particle->contact(
            rotate_in(m_contact_parameters.impulse), rotate_in(world_v),
            m_contact_parameters.m_distance, rotate_in(m_contact_parameters.normal),
            rotate_in(world_omega), m_contact_parameters.material);

        translate(m_contact_parameters.m_distance * m_contact_parameters.normal);
        m_contact_parameters.m_distance = 0.0;
    }

    // Propagate the particles
    for (auto p : m_particles)
        p->propagate(time);
    for (auto& p : m_temporary_contact_points)
        p.propagate(time);

    // Move the body and the particles in response to forces applied to them and their
    // momenta, while keeping their relative positions fixed.
    m_delta_time = time;
    auto total_force{m_cm_force};
    Three_Vector total_torque;
    for (auto p : m_particles)
    {
        // Find the force that the particle exerts on the rest of the system.  The
        // particle reports its force in the Body frame.
        auto body_force{p->force() + p->impulse() / time};
        total_force += body_force;

        // Find the force and torque that the particle exerts on the Body.  Find the
        // vector from the cm to the particle in the world frame.
        auto torque_dist{m_body_cm - p->position()};
        auto torque{p->torque()};
        auto I{(m_inertia * torque.unit()).magnitude()};
        torque *= I / (I + m_mass * torque_dist.dot(torque_dist));
        auto force_dist{m_body_cm - p->force_position()};
        total_torque += torque - force_dist.cross(body_force);
    }

    // Transform the forces to the parent's coordinates so we can find out how the Body
    // moves w.r.t its parent.
    total_force = rotate_out(total_force) + m_gravity * m_mass;
    auto delta_omega{time * total_torque * invert(m_inertia)};
    auto delta_theta{(angular_velocity() + delta_omega) * time};
    angular_accelerate(delta_omega);

    m_acceleration = total_force / m_mass;
    auto delta_v{m_acceleration * time};
    auto delta_r{(m_cm_velocity + delta_v) * time};
    m_cm_velocity += delta_v;

    // Because the body's origin is not necessarily coincident with the center of mass,
    // the body's translation has a component that depends on the orientation.  Place the
    // Body by translating to the cm, rotating and then translating back.
    auto pos{position()};
    translate(orientation() * m_body_cm);
    // rotate() acts in the body frame.
    rotate(delta_theta);
    translate(orientation() * -m_body_cm + delta_r);

    // Determine the velocity of the origin.
    set_velocity((position() - pos) / time);
}

void Rigid_Body::end_timestep()
{
    for (auto p : m_particles)
        p->end_timestep();
    m_temporary_contact_points.clear();
    m_cm_force.zero();
}

Three_Vector Rigid_Body::velocity(Particle const& p) const
{
    return velocity(p.position());
}

Three_Vector Rigid_Body::velocity(const Three_Vector& r) const
{
    return m_cm_velocity + rotate_out(angular_velocity().cross(moment(r)));
}

Three_Vector Rigid_Body::acceleration() const
{
    return rotate_in(m_acceleration);
}

Three_Vector Rigid_Body::moment(const Vamos_Geometry::Three_Vector& pos) const
{
    return pos - m_body_cm;
}

Three_Vector Rigid_Body::world_moment(const Vamos_Geometry::Three_Vector& world_pos) const
{
    return rotate_out(moment(transform_in(world_pos)));
}

Three_Matrix Rigid_Body::inertia() const
{
    return m_inertia;
}

void Rigid_Body::contact(Particle& particle, const Three_Vector& impulse,
                         const Three_Vector& velocity, double depth, const Three_Vector& normal,
                         const Vamos_Geometry::Material& material)
{
    if (particle.single_contact())
    {
        if (depth > m_contact_parameters.m_distance)
            m_contact_parameters
                = Contact_Parameters(&particle, impulse, depth, normal, material);
        return;
    }
    particle.contact(rotate_in(impulse), rotate_in(velocity), depth, rotate_in(normal),
                     rotate_in(angular_velocity().project(normal)), material);
}

void Rigid_Body::temporary_contact(const Three_Vector& position, const Three_Vector& impulse,
                                   const Three_Vector& velocity, double depth,
                                   const Three_Vector& normal,
                                   const Vamos_Geometry::Material& material)
{
    m_temporary_contact_points.emplace_back(0.0, transform_in(position), material);

    // The material, restitution and friction are not used for temporaries.  The impulse
    // is calculated externally and passed in.
    m_temporary_contact_points.back()
        .contact(rotate_in(impulse), rotate_in(velocity), depth, rotate_in(normal),
                 rotate_in(angular_velocity().project(normal)), material);

}

void Rigid_Body::wind(const Three_Vector& wind_vector, double density)
{
    for (auto drag : m_drag_particles)
        drag->wind(rotate_in(wind_vector - velocity(*drag)), density);
}

double Rigid_Body::drag() const
{
    return std::accumulate(m_drag_particles.begin(), m_drag_particles.end(), 0,
                           [](double lift, auto p) { return lift + p->drag_factor(); } );
}

double Rigid_Body::lift() const
{
    return std::accumulate(m_drag_particles.begin(), m_drag_particles.end(), 0,
                           [](double lift, auto p) { return lift + p->lift_factor(); } );
}

void Rigid_Body::reset(double direction)
{
    set_position(m_initial_position);
    Three_Matrix orient(m_initial_orientation);
    set_orientation(orient.rotate(Three_Vector(0.0, 0.0, direction)));
    private_reset();
}

void Rigid_Body::reset(const Three_Vector& position, const Three_Matrix& orientation)
{
    set_position(position);
    set_orientation(orientation);
    private_reset();
}

void Rigid_Body::private_reset()
{
    m_cm_velocity = m_initial_velocity;
    set_velocity(m_cm_velocity + m_initial_velocity.cross(moment(Three_Vector(0.0, 0.0, 0.0))));
    set_angular_velocity(m_initial_angular_velocity);
    for (auto p : m_particles)
        p->reset();
}
