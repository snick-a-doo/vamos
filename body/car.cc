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
    : m_chassis(position, orientation),
      m_max_steer_angle(15.0),
      m_steer_exponent(1.0),
      m_slide(0.0),
      m_shift_pending(false),
      m_shift_timer(0.0),
      m_shift_delay(0.2),
      m_new_gear(0),
      m_last_gear(0),
      m_clutch_key_control(true),
      mp_front_particle(0),
      m_distance_traveled(0.0),
      m_field_of_view(60.0),
      m_pan_angle(90.0),
      m_show_dashboard_extras(false),
      m_air_density(0.0)
{
}

void Car::read(std::string data_dir, std::string car_file)
{
    // Remember the file name for re-reading.
    if (!data_dir.empty() && !car_file.empty())
    {
        m_data_dir = data_dir;
        m_car_file = car_file;
    }

    m_wheels.clear();
    m_chassis.particles().clear();
    Car_Reader reader(m_data_dir, m_car_file, this);

    // Find the bounding box for the particles.
    auto pos1{m_chassis.particles().front()->position()};
    m_crash_box = {pos1.x, pos1.x, pos1.y, pos1.y, pos1.z, pos1.z};
    for (auto const& p : m_chassis.particles())
    {
        const auto& pos{p->position()};
        if (pos.x > m_crash_box.front)
            mp_front_particle = p.get();
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

// Advance the car in time by TIME.  This method assumes that the
// first four members of m_particles are the left-front, right-front,
// left-rear, and right-rear wheels, and that the front wheels are
// steered and the rear wheels are driven.  Re-define this virtual
// function if you want to change these conditions.
void Car::propagate(double time)
{
    // Propagate the key controls.
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
    auto [left_torque, right_torque] = mp_drivetrain->get_torque();
    auto right_wheel_speed{0.0};
    auto left_wheel_speed{0.0};
    for (auto const& wheel : m_wheels)
    {
        if (wheel->steered()) //!! put this logic in the wheel.
            wheel->steer(m_steer_key_control.value());
        wheel->brake(going ? m_brake_key_control.value() : 1.0);
        if (mp_drivetrain && wheel->driven())
        {
            // Apply the driving torque.
            auto left{wheel->side() == Side::left};
            wheel->drive_torque(left ? left_torque : right_torque);
            (left ? left_wheel_speed : right_wheel_speed) = wheel->rotational_speed();
        }
        // Sum the sliding speeds of the tires.
        m_slide += wheel->slide();
    }
    m_slide = std::min(1.0, m_slide / m_wheels.size());

    // Update the drivetrain.
    if (mp_drivetrain)
    {
        mp_drivetrain->input(gas, m_clutch_key_control.value(), left_wheel_speed,
                             right_wheel_speed);

        // Propagate the base class.
        mp_drivetrain->find_forces();
    }

    m_chassis.find_forces();
    if (mp_drivetrain)
        mp_drivetrain->propagate(time);
    m_chassis.propagate(time);

    m_chassis.end_timestep();

    m_distance_traveled += m_chassis.rotate_from_parent(m_chassis.cm_velocity()).x * time;

    // Update the smoothed acceleration.
    const double weight = std::min(acceleration_average_weight * time, 1.0);
    m_smoothed_acceleration
        = (1.0 - weight) * m_smoothed_acceleration + weight * m_chassis.acceleration();
}

// Change the steering angle to ANGLE with a time constant of TIME.
void Car::steer(double angle, double time, bool direct /* = false */)
{
    if (!direct)
    {
        double steer_sign = (angle < 0.0) ? -1.0 : 1.0;

        // Apply the non-linearity.
        angle = steer_sign * std::pow(std::abs(angle), m_steer_exponent);

        // Set the maximum angle and speed sensitivity.
        double sens = 1.0
                      / (1.0
                         + 1.0e-4 * m_steer_speed_sensitivity
                               * m_chassis.cm_velocity().dot(m_chassis.cm_velocity()));
        angle = m_max_steer_angle * clip(angle * sens, -1.0, 1.0);
    }
    m_steer_key_control.target(angle, time);
}

// Change the throttle to FACTOR with a time constant of TIME.
void Car::gas(double factor, double time)
{
    m_gas_key_control.target(factor, time);
}

// Change the brakes to FACTOR with a time constant of TIME.
void Car::brake(double factor, double time)
{
    m_brake_key_control.target(factor, time);
}

// Pan the view.
void Car::pan(double factor, double time)
{
    m_pan_key_control.target(factor * m_pan_angle, time / m_pan_angle);
}

// Shift to the next lower gear.  The chosen gear is returned.
int Car::shift_down()
{
    assert(mp_drivetrain);
    return shift(mp_drivetrain->transmission().gear() - 1);
}

// Shift to the next higher gear.  The chosen gear is returned.
int Car::shift_up()
{
    assert(mp_drivetrain);
    return shift(mp_drivetrain->transmission().gear() + 1);
}

// Shift to GEAR.  The chosen gear is returned.
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

// Engage the clutch with a time constant of TIME.
void Car::engage_clutch(double time)
{
    // Wait for the shift timer.
    double delay = m_shift_delay - m_shift_timer;
    m_clutch_key_control.target(1.0, time, delay);
}

// Disengage the clutch with a time constant of TIME.
void Car::disengage_clutch(double time)
{
    // Wait for the shift timer.
    double delay = m_shift_delay - m_shift_timer;
    m_clutch_key_control.target(0.0, time, delay);
}

// Return the pointer to the WHEEL_INDEXth wheel.
Wheel& Car::wheel(size_t wheel_index) const
{
    assert(wheel_index < m_wheels.size());
    return *m_wheels[wheel_index];
}

// Return the position of the viewpont.
Three_Vector Car::view_position(bool world, bool bob) const
{
    Three_Vector pos = m_driver_view;
    if (bob)
        pos -= view_shift_factor * acceleration(true);

    return world ? m_chassis.transform_to_world(pos) : pos;
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

// Restore the initial conditions.
void Car::reset()
{
    m_chassis.reset(0.0);
    private_reset();
}

// Restore the initial conditions and then set the position to
// POSITION and the orientation to ORIENTATION.
void Car::reset(const Three_Vector& position, const Three_Matrix& orientation)
{
    m_chassis.reset(position, orientation);
    private_reset();
}

// Perform operations common to both reset() methods.
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

Contact_Info Car::collision(const Three_Vector& position, const Three_Vector& velocity,
                            bool ignore_z) const
{
    const Three_Vector penetration(
        m_crash_box.penetration(m_chassis.transform_from_world(position),
                                m_chassis.transform_velocity_from_world(velocity), ignore_z));

    return Contact_Info(!penetration.is_null(), penetration.magnitude(),
                        m_chassis.rotate_to_world(penetration), Material::METAL);
}

void Car::wind(const Vamos_Geometry::Three_Vector& wind_vector, double density)
{
    m_air_density = density;
    m_chassis.wind(wind_vector, density);
}

Three_Vector Car::chase_position() const
{
    const Three_Vector v1 = m_chassis.cm_velocity().unit();
    const double w1 = std::min(m_chassis.cm_velocity().magnitude(), 1.0);
    const Three_Vector v2 = m_chassis.rotate_to_world(Three_Vector::X);
    const double w2 = 1.0 - w1;

    return m_chassis.transform_to_world(center() - 0.1 * acceleration(true))
           - (w1 * v1 + w2 * v2) * 3.0 * length() + Three_Vector(0.0, 0.0, length());
}

Three_Vector Car::front_position() const
{
    return m_chassis.transform_to_world(front());
}

void Car::set_front_position(const Three_Vector& pos)
{
    m_chassis.set_position(pos - m_chassis.rotate_to_world(front()));
}

Three_Vector Car::target_position() const
{
    return m_chassis.transform_to_world(center() + Three_Vector(target_distance(), 0.0, 0.0));
}

double Car::target_distance() const
{
    auto speed = m_chassis.cm_velocity().magnitude();
    return length() + 0.2 * speed;
}

double Car::grip() const
{
    return std::accumulate(m_wheels.begin(), m_wheels.end(), 0.0,
                           [](double total, auto wheel) { return total + wheel->grip(); })
        / m_wheels.size();
}

Three_Vector Car::acceleration(bool smooth) const
{
    return smooth ? m_smoothed_acceleration : m_chassis.acceleration();
}

Three_Vector Car::Crash_Box::penetration(const Three_Vector& point, const Three_Vector& velocity,
                                         bool ignore_z) const
{
    if (!within(point, ignore_z))
        return Three_Vector();

    if (velocity.x != 0.0 && is_in_range(point.x, back, front))
    {
        const double x_limit = (point.x - back) < (front - point.x) ? back : front;
        if (((point.x - back < front - point.x) && velocity.x > 0.0)
            || ((point.x - back >= front - point.x) && velocity.x < 0.0))
        {
            const Three_Vector x_intercept(
                x_limit, intercept(x_limit, point.x, point.y, velocity.y / velocity.x),
                intercept(x_limit, point.x, point.z, velocity.z / velocity.x));
            if (is_in_range(x_intercept.y, right, left)
                && (ignore_z || is_in_range(x_intercept.z, bottom, top)))
                return Three_Vector(x_limit - point.x, 0.0, 0.0);
        }
    }

    if (velocity.y != 0.0 && is_in_range(point.y, right, left))
    {
        const double y_limit = (point.y - right) < (left - point.y) ? right : left;
        if (((point.y - right < left - point.y) && velocity.y > 0.0)
            || ((point.y - right >= left - point.y) && velocity.y < 0.0))
        {
            const Three_Vector y_intercept(
                intercept(y_limit, point.y, point.x, velocity.x / velocity.y), y_limit,
                intercept(y_limit, point.y, point.z, velocity.z / velocity.y));
            if (is_in_range(y_intercept.x, back, front)
                && (ignore_z || is_in_range(y_intercept.z, bottom, top)))
                return Three_Vector(0.0, y_limit - point.y, 0.0);
        }
    }

    if (!ignore_z && velocity.z != 0.0 && is_in_range(point.z, bottom, top))
    {
        const double z_limit = (point.z - bottom) < (top - point.z) ? bottom : top;
        if (((point.z - bottom < top - point.z) && velocity.z > 0.0)
            || ((point.z - bottom >= top - point.z) && velocity.z < 0.0))
        {
            const Three_Vector z_intercept(
                intercept(z_limit, point.z, point.x, velocity.x / velocity.z),
                intercept(z_limit, point.z, point.y, velocity.y / velocity.z), z_limit);
            if (is_in_range(z_intercept.x, back, front) && is_in_range(z_intercept.y, right, left))
                return Three_Vector(0.0, 0.0, z_limit - point.z);
        }
    }

    return Three_Vector(0.0, 0.0, 0.0);
}

// Return true if the position is within the crash box.  ignore_z ==
// true will only consider the x and y coordinates -- useful for
// collisions with the edge of the world.
bool Car::Crash_Box::within(const Three_Vector& position, bool ignore_z) const
{
    return (position.x < front) && (position.x > back) && (position.y < left)
           && (position.y > right) && (ignore_z || ((position.z < top) && (position.z > bottom)));
}
