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

#include "wheel.h"

#include "../geometry/conversions.h"
#include "../media/model.h"

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

Wheel::Wheel(double mass, Three_Vector const& position, double tire_offset, double roll_height,
             double restitution, std::shared_ptr<Suspension> suspension, Tire const& tire,
             Brake const& brake, bool steered, bool driven, Side side)
    : Particle{mass, position, {Material::rubber, 0.0, restitution}},
      m_original_position{position},
      m_tire_offset{side == Side::right ? -tire_offset : tire_offset},
      m_roll_height{roll_height},
      mp_suspension{suspension},
      m_tire{tire},
      m_brake{brake},
      m_is_steered{steered},
      m_is_driven{driven},
      m_side(side)
{
}

void Wheel::propagate(double time)
{
    if (!is_in_contact())
    {
        Particle::reset();
        set_position(m_original_position);
        mp_suspension->reset();
    }

    m_tire.input(m_ground_velocity, m_angular_velocity.z, mp_suspension->force().project(m_normal),
                 mp_suspension->current_camber(m_normal.unit().y),
                 m_drive_torque + m_braking_torque, m_brake.is_locked(), m_surface_material);

    m_tire.propagate(time);
    set_force(m_tire.force());
    set_torque(Three_Vector(m_tire.torque().x, -m_tire.torque().y, m_tire.torque().z));
    set_orientation(mp_suspension->orientation());
    m_rotation += speed() * time / m_tire.radius();
}

double Wheel::contact(const Three_Vector& impulse, const Three_Vector& velocity, double distance,
                      const Three_Vector& normal, const Three_Vector& angular_velocity,
                      const Material& surface_material)
{
    Particle::contact(impulse, rotate_in(velocity), distance, rotate_in(normal),
                      rotate_in(angular_velocity), surface_material);

    // The collision is soft unless the suspension has bottomed out.
    if (!mp_suspension->is_bottomed_out())
        set_impulse(null_v);

    m_normal = rotate_in(normal);
    auto v_perp{rotate_in(velocity).project(m_normal)};
    m_ground_velocity = rotate_in(velocity) - v_perp;
    m_angular_velocity = angular_velocity;

    mp_suspension->displace(((m_normal * distance).back_project(z_hat)).magnitude());
    set_position(m_original_position + mp_suspension->get_displacement() * z_hat);
    // The suspension displacement may be different from the argument to displace() if it
    // bottoms out.  Ask the suspension for the actual displacement.
    mp_suspension->input(m_normal);
    mp_suspension->set_torque(m_braking_torque);
    m_surface_material = surface_material;
    return -mp_suspension->get_displacement();
}

void Wheel::set_drive_torque(double torque_in)
{
    if (m_is_driven)
        m_drive_torque = torque_in;
}

void Wheel::brake(double factor)
{
    m_braking_torque = -m_brake.torque(factor, m_tire.rotational_speed());
}

void Wheel::steer(double degree_angle)
{
    if (m_is_steered)
        mp_suspension->steer(degree_angle);
}

Three_Vector Wheel::force_position() const
{
    return position() + m_tire.contact_position() + Three_Vector(0.0, m_tire_offset, m_roll_height);
}

Three_Vector Wheel::contact_position() const
{
    return m_original_position + m_tire.contact_position() + Three_Vector(0.0, m_tire_offset, 0.0);
}

void Wheel::reset()
{
    Particle::reset();
    m_tire.reset();
}

GLuint Wheel::make_model(std::string const& file, double scale,
                         Three_Vector const& translation,
                         Three_Vector const& rotation)
{
    return Vamos_Media::Model(file, scale, translation, rotation).build();
}

void Wheel::set_models(std::string const& slow_file, std::string const& fast_file,
                       double transition_speed, std::string const& stator_file,
                       double stator_offset, double scale,
                       Three_Vector const& translation, Three_Vector const& rotation)
{
    Three_Vector offset;
    auto has_stator{!stator_file.empty() && !stator_file.ends_with('/')};
    if (has_stator)
        offset.y += (m_side == Side::right) ? stator_offset : -stator_offset;

    if (m_slow_wheel_list)
        glDeleteLists(m_slow_wheel_list, 1);
    m_slow_wheel_list = make_model(slow_file, scale, translation + offset, rotation);

    if (m_fast_wheel_list)
        glDeleteLists(m_fast_wheel_list, 1);
    m_fast_wheel_list = make_model(fast_file, scale, translation + offset, rotation);

    m_transition_speed = transition_speed;

    if (m_stator_list)
        glDeleteLists(m_stator_list, 1);
    if (has_stator)
        m_stator_list = make_model(stator_file, scale, translation, rotation);
}

void Wheel::draw()
{
    glPushMatrix();
    glTranslatef(position().x, position().y, position().z);
    {
        double angle;
        auto axis{axis_angle(angle)};
        glRotatef(angle, axis.x, axis.y, axis.z);
    }
    glCallList(m_stator_list);
    auto slow{speed() < m_transition_speed};
    if (slow)
        glRotatef(rad_to_deg(m_rotation), 0.0, 1.0, 0.0);
    glCallList(slow ? m_slow_wheel_list : m_fast_wheel_list);
    glPopMatrix();
    mp_suspension->draw();
}
