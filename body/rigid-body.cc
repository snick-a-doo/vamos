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

Rigid_Body::Rigid_Body(Three_Vector const& pos, Three_Matrix const& orient)
    : Frame(pos, orient),
      m_initial_frame(pos, orient)
{
}

void Rigid_Body::set_initial_conditions(Three_Vector const& position,
                                        Three_Vector const& orientation,
                                        Three_Vector const& velocity,
                                        Three_Vector const& angular_velocity)
{
    m_initial_frame
        = Frame(position, Three_Matrix(1.0).rotate(orientation * deg_to_rad(1.0)));
    m_initial_frame.set_velocity(velocity);
    m_initial_frame.set_angular_velocity(angular_velocity * deg_to_rad(1.0));
    reset(0.0);
}

void Rigid_Body::add_particle(std::shared_ptr<Particle> particle)
{
    m_particles.push_back(particle);
    if (auto aero = std::dynamic_pointer_cast<Drag>(particle))
        m_aero_particles.push_back(aero);
}

Three_Vector Rigid_Body::cm_position() const
{
    return transform_out(m_body_cm);
}

Three_Vector Rigid_Body::contact_position(Particle const& p) const
{
    return transform_out(p.contact_position());
}

Three_Vector Rigid_Body::particle_position(Particle const& p) const
{
    return transform_out(p.position());
}

Three_Vector Rigid_Body::moment(Three_Vector const& world_pos) const
{
    return rotate_out(transform_in(world_pos) - m_body_cm);
}

Three_Vector Rigid_Body::particle_velocity(Particle const& p) const
{
    return point_velocity(p.position());
}

Three_Vector Rigid_Body::point_velocity(Three_Vector const& r) const
{
    return m_cm_velocity + rotate_out(angular_velocity().cross(r - m_body_cm));
}

Three_Vector Rigid_Body::acceleration() const
{
    return rotate_in(m_acceleration);
}

double Rigid_Body::drag() const
{
    return std::accumulate(m_aero_particles.begin(), m_aero_particles.end(), 0.0,
                           [](double lift, auto const& p) { return lift + p->drag_factor(); } );
}

double Rigid_Body::lift() const
{
    return std::accumulate(m_aero_particles.begin(), m_aero_particles.end(), 0.0,
                           [](double lift, auto const& p) { return lift + p->lift_factor(); } );
}

void Rigid_Body::contact(Particle& particle, Three_Vector const& impulse,
                         Three_Vector const& velocity, double depth, Three_Vector const& normal,
                         Vamos_Media::Material const& material)
{
    if (particle.single_contact())
    {
        if (depth > m_contact_params.info.depth)
            m_contact_params = {&particle, impulse, {true, depth, normal, material}};
        return;
    }
    particle.contact(rotate_in(impulse), rotate_in(velocity), depth, rotate_in(normal),
                     rotate_in(angular_velocity().project(normal)), material);
}

void Rigid_Body::temporary_contact(Three_Vector const& position, Three_Vector const& impulse,
                                   Three_Vector const& velocity, double depth,
                                   Three_Vector const& normal,
                                   Vamos_Media::Material const& material)
{
    m_temporary_contact.emplace_back(0.0, transform_in(position), material);

    // The material, restitution and friction are not used for temporaries.  The impulse
    // is calculated externally and passed in.
    m_temporary_contact.back()
        .contact(rotate_in(impulse), rotate_in(velocity), depth, rotate_in(normal),
                 rotate_in(angular_velocity().project(normal)), material);

}

void Rigid_Body::wind(Three_Vector const& wind_vector, double density)
{
    for (auto aero : m_aero_particles)
        aero->wind(rotate_in(wind_vector - particle_velocity(*aero)), density);
}

/// Add the contribution of a particle with the given mass and center-of-mass position to
/// an inertia tensor.
static void add_inertia(double mass, Three_Vector const& pos, Three_Matrix& I)
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

void Rigid_Body::update_center_of_mass()
{
    m_body_cm.zero();
    m_mass = 0.0;
    m_inertia.zero();

    for (auto p : m_particles)
    {
        m_mass += p->mass();
        m_body_cm += p->position() * p->mass();
    }
    m_body_cm /= m_mass;
    // Need a 2nd loop after finding the center of mass.
    for (auto p : m_particles)
        add_inertia(p->mass(), p->position() - m_body_cm, m_inertia);
}

void Rigid_Body::propagate(double time)
{
    // Process single-collision contact.
    if (m_contact_params.info.depth > 0.0)
    {
        auto point{*m_contact_params.particle};
        auto v{particle_velocity(point)};
        auto omega{angular_velocity()};
        point.contact(rotate_in(m_contact_params.impulse), rotate_in(v),
                      m_contact_params.info.depth, rotate_in(m_contact_params.info.normal),
                      rotate_in(omega), m_contact_params.info.material);
        translate(m_contact_params.info.depth * m_contact_params.info.normal);
        m_contact_params.info.depth = 0.0;
    }

    // Propagate the particles
    for (auto p : m_particles)
        p->propagate(time);
    for (auto& p : m_temporary_contact)
        p.propagate(time);
    update_center_of_mass();

    // Move the body and the particles in response to forces applied to them and their
    // momenta, while keeping their relative positions fixed.
    Three_Vector total_force;
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

    // Because the body's origin is not generally coincident with the center of mass, the
    // body's translation has a component that depends on the orientation. Place the body
    // by translating to the cm, rotating and then translating back.
    auto const& pos{position()};
    translate(orientation() * m_body_cm);
    // rotate() acts in the body frame.
    rotate(delta_theta);
    translate(orientation() * -m_body_cm + delta_r);

    // Determine the velocity of the origin.
    set_velocity((position() - pos) / time);

    m_temporary_contact.clear();
}

void Rigid_Body::reset(double direction)
{
    auto orient{m_initial_frame.orientation()};
    reset(m_initial_frame.position(), orient.rotate(direction * z_hat));
}

void Rigid_Body::reset(Three_Vector const& position, Three_Matrix const& orientation)
{
    set_position(position);
    set_orientation(orientation);
    m_cm_velocity = m_initial_frame.velocity();
    set_velocity(m_cm_velocity + m_body_cm.cross(m_initial_frame.angular_velocity()));
    set_angular_velocity(m_initial_frame.angular_velocity());
    for (auto p : m_particles)
        p->reset();
}
