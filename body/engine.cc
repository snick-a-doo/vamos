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

#include "engine.h"
#include "../geometry/conversions.h"

#include <cmath>

using namespace Vamos_Body;
using namespace Vamos_Geometry;

Engine::Engine(double mass,
               Three_Vector const& position,
               double max_power,
               double peak_engine_rpm,
               double rpm_limit,
               double inertia,
               double idle_throttle,
               double start_speed,
               double stall_speed,
               double fuel_consumption)
    : Particle{mass, position},
      m_max_power{max_power},
      m_peak_engine_speed{rpm_to_rad_s(peak_engine_rpm)},
      m_engine_speed_limit{rpm_to_rad_s(rpm_limit)},
      m_inertia{inertia},
      m_idle_throttle{idle_throttle},
      m_start_speed{rpm_to_rad_s(start_speed)},
      m_stall_speed{rpm_to_rad_s(stall_speed)},
      m_fuel_consumption{fuel_consumption},
      // See "Motor Vehicle Dynamics" Genta, Section 4.2.2
      m_friction{m_max_power / std::pow(m_peak_engine_speed, 3)}
{
}

void Engine::set_torque_curve(std::vector<Point<double>> const& torque_points)
{
    m_torque_curve.clear();
    m_torque_curve.load(torque_points);
    m_torque_curve.scale(rpm_to_rad_s(1.0));
}

void Engine::set_friction(double friction)
{
    m_friction = friction;
}

void Engine::input(double gas, double drag, double transmission_speed, bool engaged)
{
    m_gas = (m_out_of_gas
             || m_rotational_speed < m_stall_speed
             || m_rotational_speed > m_engine_speed_limit)
        ? 0.0 : std::max(gas, m_idle_throttle);
    m_drag = drag;
    m_transmission_speed = transmission_speed;
    m_engaged = engaged;
}

double Engine::power(double gas, double rotational_speed)
{
    return rotational_speed * torque_map(gas, rotational_speed);
}

void Engine::propagate(double time)
{
    // Find the engine's torque with the current conditions.
    m_drive_torque = torque_map(m_gas, m_rotational_speed) - m_drag;
    set_torque(Three_Vector{-m_drive_torque, 0.0, 0.0});

    // If the clutch is engaged, the engine speed is locked to the transmission speed.
    m_rotational_speed = m_engaged
        ? m_transmission_speed
        : m_rotational_speed + time * m_drive_torque / m_inertia;

    // Keep engine speed from going negative when changing from forward to reverse (or
    // vice versa) without using the clutch.
    if (m_rotational_speed < m_stall_speed)
        m_rotational_speed = 0.0;
}

double Engine::torque_map(double gas, double rot_speed)
{
    // See "Motor Vehicle Dynamics" Genta, Section 4.2.2
    auto torque{m_torque_curve.empty()
                ? m_max_power * (1.0 + rot_speed / m_peak_engine_speed) / m_peak_engine_speed
                : m_torque_curve.interpolate(rot_speed)};
    auto friction{m_friction * rot_speed * rot_speed};
    return gas * torque - friction;
}

double Engine::fuel_rate() const
{
    return m_fuel_consumption * m_rotational_speed * m_gas;
}

void Engine::start()
{
    m_rotational_speed = m_start_speed;
}
