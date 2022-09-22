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
//  If not, see <http://www.gnu.org/licenses/>.  along with this program; if not, write to
//  the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
//  USA

#include "car.h"
#include "aerodynamics.h"
#include "brake.h"
#include "clutch.h"
#include "dashboard.h"
#include "differential.h"
#include "engine.h"
#include "fuel-tank.h"
#include "particle.h"
#include "suspension.h"
#include "tire.h"
#include "transmission.h"
#include "wheel.h"

#include "../geometry/numeric.h"
#include "../media/texture-image.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

auto constexpr acceleration_average_weight{10.0};
auto constexpr view_shift_factor{0.003};

//----------------------------------------------------------------------------------------
Key_Control::Key_Control(bool block)
    : m_block(block)
{
}

void Key_Control::target(double new_target, double time, double delay)
{
    if (m_block)
    {
        if (m_value == m_target)
            m_target_pending = false;
        else
        {
            m_target_pending = true;
            m_next_target = new_target;
            m_next_time = time;
            m_next_delay = delay;
            return;
        }
    }

    m_target = new_target;
    m_delay = delay;
    m_time = 0.0;

    if (time != 0.0)
    {
        m_rate = 1.0 / time;
        if (m_target < m_value)
            m_rate = -m_rate;
    }
    else
        m_rate = 0.0;
}

double Key_Control::update(double time)
{
    auto new_value{m_value};
    m_time += time;
    if (m_time >= m_delay)
    {
        if (m_rate == 0.0)
            new_value = m_target;
        else
        {
            new_value += m_rate * time;
            if ((m_rate > 0.0 && new_value > m_target)
                || (m_rate < 0.0 && new_value < m_target))
            {
                new_value = m_target;
                m_rate = 0.0;
            }
        }
        if (m_target_pending && new_value == m_target)
            target(m_next_target, m_next_time, m_next_delay);
    }
    m_value = new_value;
    return m_value;
}

void Key_Control::end()
{
    m_value = m_target;
    m_time = m_delay;
    m_rate = 0.0;
}

//----------------------------------------------------------------------------------------
Car::Car(const Three_Vector& position, const Three_Matrix& orientation)
    : m_chassis(position, orientation)
{
}

Car::~Car()
{
    // Trivial destructor instead of = default because of shared_ptrs.
}

void Car::read(std::string const& data_dir, std::string const& car_file)
{
    // Remember the file name for re-reading.
    if (!data_dir.empty() && !car_file.empty())
    {
        m_data_dir = data_dir;
        m_car_file = car_file;
    }

    m_wheels.clear();
    Car_Reader reader(m_data_dir, m_car_file, this);

    // Find the bounding box for the particles.
    auto pos1{m_chassis.particles().front()->position()};
    m_crash_box = {pos1.x, pos1.x, pos1.y, pos1.y, pos1.z, pos1.z};
    for (auto const& p : m_chassis.particles())
    {
        const auto& pos{p->position()};
        m_crash_box.front = std::max(m_crash_box.front, pos.x);
        m_crash_box.back = std::min(m_crash_box.back, pos.x);
        m_crash_box.left = std::max(m_crash_box.left, pos.y);
        m_crash_box.right = std::min(m_crash_box.right, pos.y);
        m_crash_box.top = std::max(m_crash_box.top, pos.z);
        m_crash_box.bottom = std::min(m_crash_box.bottom, pos.z);
    }
}

void Car::set_robot_parameters(double slip_ratio, double deceleration, double lateral_acceleration)
{
    m_robot_parameters.slip_ratio = slip_ratio;
    m_robot_parameters.deceleration = deceleration;
    m_robot_parameters.lateral_acceleration = lateral_acceleration;
}

void Car::adjust_robot_parameters(double slip_ratio_factor, double deceleration_factor,
                                  double lateral_acceleration_factor)
{
    m_robot_parameters.slip_ratio += slip_ratio_factor * m_robot_parameters.slip_ratio;
    m_robot_parameters.deceleration += deceleration_factor * m_robot_parameters.deceleration;
    m_robot_parameters.lateral_acceleration
        += lateral_acceleration_factor * m_robot_parameters.lateral_acceleration;
}

void Car::propagate(double time)
{
    m_steer_key_control.update(time);
    m_gas_key_control.update(time);
    m_brake_key_control.update(time);
    m_clutch_key_control.update(time);
    m_pan_key_control.update(time);

    // Update the transmission.
    auto gas{m_gas_key_control.value()};
    static bool going = false; //!!!!
    if (mp_drivetrain)
    {
        if (m_shift_pending)
        {
            m_shift_timer += time;
            if (m_shift_timer > m_shift_delay)
            {
                mp_drivetrain->transmission().shift(m_new_gear);
                m_shift_pending = false;
            }
        }

        // Update the throttle.
        assert(mp_fuel_tank);
        // Let the engine know if the fuel tank is empty.
        if (mp_fuel_tank->empty())
            gas = 0.0;
        mp_drivetrain->engine().out_of_gas(mp_fuel_tank->empty());

        // Update the fuel tank.
        mp_fuel_tank->consume(mp_drivetrain->engine().fuel_rate() * time);

        if (mp_drivetrain->transmission().gear() != 0
            && mp_drivetrain->clutch().pressure() != 0.0)
            going = true;
    }

    m_slide = 0.0;
    auto [left_torque, right_torque]
        = mp_drivetrain ? mp_drivetrain->get_torque() : std::make_pair(0.0, 0.0);
    auto right_wheel_speed{0.0};
    auto left_wheel_speed{0.0};
    for (auto const& wheel : m_wheels)
    {
        wheel->steer(m_steer_key_control.value());
        wheel->brake(going ? m_brake_key_control.value() : 1.0);
        if (mp_drivetrain)
        {
            // Apply the driving torque.
            auto left{wheel->side() == Side::left};
            wheel->set_drive_torque(left ? left_torque : right_torque);
            (left ? left_wheel_speed : right_wheel_speed) = wheel->rotational_speed();
        }
        // Sum the sliding speeds of the tires.
        m_slide += wheel->get_tire().slide();
    }
    m_slide = std::min(1.0, m_slide / m_wheels.size());

    // Update the drivetrain.
    if (mp_drivetrain)
    {
        mp_drivetrain->input(gas, m_clutch_key_control.value(), left_wheel_speed,
                             right_wheel_speed);
        mp_drivetrain->propagate(time);
    }
    m_chassis.propagate(time);

    m_distance_traveled += m_chassis.rotate_in(m_chassis.cm_velocity()).x * time;

    // Update the smoothed acceleration.
    auto weight{std::min(acceleration_average_weight * time, 1.0)};
    m_smoothed_acceleration
        = (1.0 - weight) * m_smoothed_acceleration + weight * m_chassis.acceleration();
}

void Car::steer(double angle, double time, bool direct)
{
    if (!direct)
    {
        // Apply the non-linearity.
        angle = sign(angle) * std::pow(std::abs(angle), m_steer_exponent);
        // Set the maximum angle and speed sensitivity.
        auto v2{m_chassis.cm_velocity().dot(m_chassis.cm_velocity())};
        auto sens{1.0 / (1.0 + 1.0e-4 * m_steer_speed_sensitivity * v2)};
        angle = m_max_steer_angle * clip(angle * sens, -1.0, 1.0);
    }
    m_steer_key_control.target(angle, time);
}

void Car::gas(double factor, double time)
{
    m_gas_key_control.target(factor, time);
}

void Car::brake(double factor, double time)
{
    m_brake_key_control.target(factor, time);
}

void Car::pan(double factor, double time)
{
    m_pan_key_control.target(factor * m_pan_angle, time / m_pan_angle);
}

int Car::shift_down()
{
    assert(mp_drivetrain);
    return shift(mp_drivetrain->transmission().gear() - 1);
}

int Car::shift_up()
{
    assert(mp_drivetrain);
    return shift(mp_drivetrain->transmission().gear() + 1);
}

int Car::shift(int gear)
{
    if (m_new_gear == gear)
        return gear;
    // Do the shift if GEAR is accessible.
    assert(mp_drivetrain);
    if ((gear <= mp_drivetrain->transmission().forward_gears())
        && (-gear <= mp_drivetrain->transmission().reverse_gears()))
    {
        m_shift_pending = true;
        m_shift_timer = 0.0;
        m_last_gear = mp_drivetrain->transmission().gear();
        m_new_gear = gear;
    }
    return m_new_gear;
}

void Car::clutch(double factor, double time)
{
    m_clutch_key_control.target(factor, time, 0.0);
}

void Car::engage_clutch(double time)
{
    // Wait for the shift timer.
    m_clutch_key_control.target(1.0, time, m_shift_delay - m_shift_timer);
}

void Car::disengage_clutch(double time)
{
    // Wait for the shift timer.
    m_clutch_key_control.target(0.0, time, m_shift_delay - m_shift_timer);
}

Wheel& Car::wheel(size_t wheel_index) const
{
    assert(wheel_index < m_wheels.size());
    return *m_wheels[wheel_index];
}

Three_Vector Car::view_position(bool world, bool bob) const
{
    auto pos{m_driver_view};
    if (bob)
        pos -= view_shift_factor * acceleration(true);
    return world ? m_chassis.transform_out(pos) : pos;
}

Three_Vector Car::draw_rear_view(double, int)
{
    return Vamos_Geometry::Three_Vector::ZERO;
}

void Car::start_engine()
{
    if (mp_drivetrain)
        mp_drivetrain->engine().start();
    m_clutch_key_control.end();
}

void Car::reset()
{
    m_chassis.reset(0.0);
    private_reset();
}

void Car::reset(const Three_Vector& position, const Three_Matrix& orientation)
{
    m_chassis.reset(position, orientation);
    private_reset();
}

void Car::private_reset()
{
    if (mp_drivetrain)
    {
        mp_drivetrain->reset();
        shift(0);
        start_engine();
    }
}

void Car::set_drivetrain(std::unique_ptr<Drivetrain> drivetrain)
{
    mp_drivetrain = std::move(drivetrain);
}

Contact_Info Car::collision(Three_Vector const& position,
                            Three_Vector const& velocity,
                            bool ignore_z) const
{
    auto in{m_crash_box.penetration(m_chassis.transform_in(position),
                                    m_chassis.transform_velocity_in(velocity), ignore_z)};
    return {!in.is_null(), in.magnitude(), m_chassis.rotate_out(in), Material::METAL};
}

void Car::wind(const Vamos_Geometry::Three_Vector& wind_vector, double density)
{
    m_air_density = density;
    m_chassis.wind(wind_vector, density);
}

Three_Vector Car::chase_position() const
{
    auto v1{m_chassis.cm_velocity().unit()};
    auto w1{std::min(m_chassis.cm_velocity().magnitude(), 1.0)};
    auto v2{m_chassis.rotate_out(Three_Vector::X)};
    auto w2{1.0 - w1};
    return m_chassis.transform_out(center() - 0.1 * acceleration(true))
           - (w1 * v1 + w2 * v2) * 3.0 * length() + Three_Vector(0.0, 0.0, length());
}

Three_Vector Car::front_position() const
{
    return m_chassis.transform_out(front());
}

void Car::set_front_position(const Three_Vector& pos)
{
    m_chassis.set_position(pos - m_chassis.rotate_out(front()));
}

Three_Vector Car::target_position() const
{
    return m_chassis.transform_out(center() + Three_Vector(target_distance(), 0.0, 0.0));
}

double Car::target_distance() const
{
    auto speed = m_chassis.cm_velocity().magnitude();
    return length() + 0.2 * speed;
}

double Car::grip() const
{
    return std::accumulate(m_wheels.begin(), m_wheels.end(), 0.0,
                           [](double total, auto wheel) {
                               return total + wheel->get_tire().grip(); })
        / m_wheels.size();
}

Three_Vector Car::acceleration(bool smooth) const
{
    return smooth ? m_smoothed_acceleration : m_chassis.acceleration();
}

Three_Vector Car::Crash_Box::penetration(Three_Vector const& point,
                                         Three_Vector const& velocity,
                                         bool ignore_z) const
{
    if (!within(point, ignore_z))
        return Three_Vector();

    if (velocity.x != 0.0 && is_in_range(point.x, back, front))
    {
        auto x_limit{point.x - back < front - point.x ? back : front};
        if ((point.x - back < front - point.x && velocity.x > 0.0)
            || (point.x - back >= front - point.x && velocity.x < 0.0))
        {
            Three_Vector x_int(x_limit,
                               intercept(x_limit, point.x, point.y, velocity.y / velocity.x),
                               intercept(x_limit, point.x, point.z, velocity.z / velocity.x));
            if (is_in_range(x_int.y, right, left)
                && (ignore_z || is_in_range(x_int.z, bottom, top)))
                return {x_limit - point.x, 0.0, 0.0};
        }
    }

    if (velocity.y != 0.0 && is_in_range(point.y, right, left))
    {
        auto y_limit{point.y - right < left - point.y ? right : left};
        if ((point.y - right < left - point.y && velocity.y > 0.0)
            || (point.y - right >= left - point.y && velocity.y < 0.0))
        {
            Three_Vector y_int(intercept(y_limit, point.y, point.x, velocity.x / velocity.y),
                               y_limit,
                               intercept(y_limit, point.y, point.z, velocity.z / velocity.y));
            if (is_in_range(y_int.x, back, front)
                && (ignore_z || is_in_range(y_int.z, bottom, top)))
                return {0.0, y_limit - point.y, 0.0};
        }
    }

    if (!ignore_z && velocity.z != 0.0 && is_in_range(point.z, bottom, top))
    {
        auto z_limit{point.z - bottom < top - point.z ? bottom : top};
        if ((point.z - bottom < top - point.z && velocity.z > 0.0)
            || (point.z - bottom >= top - point.z && velocity.z < 0.0))
        {
            Three_Vector z_int(intercept(z_limit, point.z, point.x, velocity.x / velocity.z),
                               intercept(z_limit, point.z, point.y, velocity.y / velocity.z),
                               z_limit);
            if (is_in_range(z_int.x, back, front) && is_in_range(z_int.y, right, left))
                return {0.0, 0.0, z_limit - point.z};
        }
    }
    return {0.0, 0.0, 0.0};
}

bool Car::Crash_Box::within(const Three_Vector& pos, bool ignore_z) const
{
    return is_in_range(pos.x, back, front)
        && is_in_range(pos.y, right, left)
        && (ignore_z || is_in_range(pos.z, bottom, top));
}
