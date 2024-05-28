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

#include "suspension.h"

#include "../geometry/conversions.h"
#include "../geometry/numeric.h"
#include "../media/model.h"

#include <cassert>
#include <cmath>

using namespace Vamos_Body;
using namespace Vamos_Geometry;

// The axis of rotation for steering and toe adjustments.
auto const steer_axis{z_hat};

// Note that all angles are stored as right-hand rotations.  As a result, m_caster for a
// wheel on the right side of the car follows the common convention that positive camber
// means that the wheel leans away from the centerline.  For the wheel on the left,
// m_caster is contrary to convention.
Hinge::Hinge(Three_Vector const& position)
    : Particle(0.0, position)
{
}

void Hinge::input(Three_Vector const& torque, Three_Vector const& radius)
{
    set_force(torque.magnitude() / radius.magnitude() * torque.cross(radius).unit());
}

//----------------------------------------------------------------------------------------
struct Vamos_Body::Suspension_Model
{
    GLuint display_list;
    Three_Vector pos;
};

//----------------------------------------------------------------------------------------
Suspension::Suspension(Three_Vector const& pos, Three_Vector const& center_of_translation,
                       Side side_of_car, double spring_constant, double bounce, double rebound,
                       double travel, double max_compression_velocity)
    : Particle{0.0, pos},
      mp_hinge{std::make_shared<Hinge>(center_of_translation)},
      m_radius(center_of_translation - pos),
      m_initial_radius(m_radius),
      m_radius_magnitude(m_radius.magnitude()),
      m_initial_z(pos.z),
      m_spring_constant(spring_constant),
      m_bounce(bounce),
      m_rebound(rebound),
      m_travel(travel),
      m_max_compression_velocity(max_compression_velocity),
      m_side(side_of_car),
      m_normal(Three_Vector(0.0, 0.0, 1.0)),
      m_hinge_axis(m_radius.cross(z_hat).unit())
{
}

Suspension::~Suspension()
{
    // Trivial destructor instead of = default because of managed pointers.
}

void Suspension::anti_roll(std::shared_ptr<Suspension> other, double spring_constant)
{
    m_anti_roll_suspension = other.get();
    m_anti_roll_k = spring_constant;
    m_anti_roll_suspension->m_anti_roll_suspension = this;
    m_anti_roll_suspension->m_anti_roll_k = m_anti_roll_k;
}

void Suspension::displace(double distance)
{
    // Ignore interaction before the first timestep.
    if (m_time_step == 0.0)
        return;
    m_bottomed_out = distance > m_travel;
    auto last_displacement{m_displacement};
    m_displacement = std::min(distance, m_travel);
    set_position(get_position());
    // The radius points from this position to the hinge.
    m_radius = mp_hinge->position() - position();
    m_compression_speed = (m_displacement - last_displacement) / m_time_step;
}

Three_Vector Suspension::get_position() const
{
    const auto& hinge_pos{mp_hinge->position()};
    auto z{hinge_pos.z - m_initial_z - m_displacement};
    assert(z <= m_radius_magnitude);
    auto angle{asin(z / m_radius_magnitude)};
    return hinge_pos - Vamos_Geometry::rotate(m_initial_radius, angle * m_hinge_axis);
}

void Suspension::input(const Three_Vector& normal)
{
    m_normal = rotate_out(normal);
}

void Suspension::set_torque(double wheel_torque)
{
    mp_hinge->input({0.0, -wheel_torque, 0.0}, m_radius);
}

void Suspension::find_forces()
{
    // Don't exert a force if this suspension is not compressed.
    if (m_displacement <= 0.0)
    {
        reset();
        return;
    }

    auto anti_roll_force = m_anti_roll_suspension
        ? m_anti_roll_k * (m_displacement - m_anti_roll_suspension->m_displacement)
        : 0.0;
    // Use m_bounce for compression, m_rebound for decompression.
    auto damp{m_compression_speed > 0.0 ? m_bounce : m_rebound};
    // If the suspension is moving at a speed > m_max_compression_speed, the damper locks
    // up due to turbulence in the fluid.  The effect is the same as bottoming out.
    if (std::abs(m_compression_speed) > m_max_compression_velocity)
        m_bottomed_out = true;
    auto spring_force{m_spring_constant * m_displacement};
    auto damp_force{damp * m_compression_speed};
    set_force(rotate_in(m_normal * (spring_force + damp_force + anti_roll_force)));
}

void Suspension::propagate(double time)
{
    find_forces();
    m_time_step = time;
    set_orientation(m_static_orientation);
    rotate(m_steer_angle * steer_axis);
}

void Suspension::steer(double degree_angle)
{
    m_steer_angle = deg_to_rad(degree_angle);
}

void Suspension::camber(double degree_angle)
{
    // Undo the current camber setting before applying the new one.
    m_static_orientation.rotate({-m_camber, 0.0, 0.0});
    m_camber = deg_to_rad(degree_angle * (m_side == Side::left ? -1.0 : 1.0));
    m_static_orientation.rotate({m_camber, 0.0, 0.0});
}

void Suspension::caster(double degree_angle)
{
    // The caster rotation is in the same direction for both sides.
    // Undo the current caster setting before applying the new one.
    m_static_orientation.rotate({0.0, -m_caster, 0.0});
    m_caster = -deg_to_rad(degree_angle);
    m_static_orientation.rotate({0.0, m_caster, 0.0});
}

void Suspension::toe(double degree_angle)
{
    // Undo the current toe setting before applying the new one.
    m_static_orientation.rotate(-m_toe * steer_axis);
    m_toe = deg_to_rad(degree_angle * (m_side == Side::left ? -1.0 : 1.0));
    m_static_orientation.rotate(m_toe * steer_axis);
}

double Suspension::current_camber(double normal_y) const
{
    return Vamos_Geometry::clip(normal_y, -0.5, 0.5);
}

void Suspension::reset()
{
    Particle::reset();
    m_displacement = 0.0;
}

void Suspension::set_model(std::string file_name, double scale, const Three_Vector& translation,
                           const Three_Vector& rotation)
{
    auto position{translation};
    auto orientation{rotation};
    if (m_side == Side::left)
    {
        // Make the right and left sides symmetric.
        position.y *= -1.0;
        orientation.x *= -1.0;
        orientation.y *= -1.0;
    }
    Vamos_Media::Model model(file_name, scale, Three_Vector(), orientation);
    m_models.emplace_back(std::make_unique<Suspension_Model>(model.build(), position));
}

void Suspension::draw()
{
    for (auto const& model : m_models)
    {
        glPushMatrix();
        glTranslatef(position().x + model->pos.x,
                     position().y + model->pos.y,
                     position().z + model->pos.z - m_displacement);
        auto angle{rad_to_deg(std::atan2(-m_displacement, model->pos.y))};
        glRotatef(angle, 1.0, 0.0, 0.0);
        glCallList(model->display_list);
        glPopMatrix();
    }
}
