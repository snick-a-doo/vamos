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

#ifndef VAMOS_BODY_ENGINE_H_INCLUDED
#define VAMOS_BODY_ENGINE_H_INCLUDED

#include "../geometry/spline.h"
#include "../geometry/two-vector.h"
#include "particle.h"

namespace Vamos_Body
{
/// An engine for the drivetrain.
class Engine : public Particle
{
public:
    /// @param max_power Peak engine power in the correct derived units (Watts in SI),
    /// @param peak_engine_rpm Engine speed at which @p max_power is achieved, in
    /// rotations per minute.
    /// @param rpm_limit The engine speed where the rev limiter kicks in.
    /// @param inertia The rotational inertia of the moving parts.
    /// @param idle_throttle The minimum throttle position.
    /// @param start_rpm The engine speed set when starting.
    /// @param stall_rpm The engine shuts off below this speed.
    /// @param fuel_consumption Fuel consumption (in l/s) per engine speed (rad/s) at full
    /// throttle.
    Engine(double mass,
           Vamos_Geometry::Three_Vector const& position,
           double max_power,
           double peak_engine_rpm,
           double rpm_limit,
           double inertia,
           double idle_throttle,
           double start_rpm,
           double stall_rpm,
           double fuel_consumption);

    /// Provide an optional set of (rpm, torque) points for interpolating engine
    /// torque. If a curve is not set, torque is calculated from a polynomial.
    void set_torque_curve(std::vector<Vamos_Geometry::Point<double>> const& torque_points);
    /// Override the default friction parameter.
    void set_friction(double friction);

    /// Handle input parameters.
    /// @param gas Throttle position.
    /// @param drag Torque due to friction when the clutch is not fully engaged.
    /// @param transmission_speed Rotational speed of the transmission side of the clutch.
    /// @param engaged True when the clutch is fully engaged, false otherwise.
    void input(double gas, double drag, double transmission_speed, bool engaged);
    /// Advance the engine in time.
    void propagate(double time);
    // Tell the engine if we're out of gas.
    void out_of_gas(bool out) { m_out_of_gas = out; }
    // Start the engine.
    void start();

    /// @return Rotational speed in radians per second.
    double rotational_speed() const { return m_rotational_speed; }
    /// @return The engine speed at which maximum power is produced.
    double peak_engine_speed() const { return m_peak_engine_speed; }
    /// @return The current torque.
    double drive_torque() const { return m_drive_torque; }
    /// @return The torque for a given throttle setting, and engine speed.
    double power(double gas, double rotational_speed);
    /// @return The throttle position.
    double throttle() const { return m_gas; }
    /// @return The current rate of fuel consumption.
    double fuel_rate() const;

private:
    /// @return The torque produced at the given throttle position and engine speed.
    double torque_map(double gas, double rot_speed);

    const double m_max_power; ///< Peak engine power.
    const double m_peak_engine_speed; ///< Engine speed at maximum power.
    const double m_engine_speed_limit; ///< Rev limit.
    const double m_inertia; ///< The rotational inertia of the moving parts.
    const double m_idle_throttle; ///< The fraction of throttle used when idling.
    const double m_start_speed; ///< The rotational speed of the engine when starting.
    const double m_stall_speed; ///< The slowest operational engine speed.
    const double m_fuel_consumption; ///< The rate of fuel consumption at full throttle.

    Vamos_Geometry::Spline m_torque_curve; ///< An optional spline of torque vs. engine speed.
    double m_friction; ///< Engine friction coefficient.

    double m_rotational_speed{0.0}; ///< Rotational speed of the engine.
    double m_gas{0.0}; ///< Throttle position.
    double m_drag{0.0}; ///< Load on the engine from the clutch.
    double m_transmission_speed{0.0}; ///< Speed of the engine side of the transmission.
    bool m_out_of_gas{false}; ///< True if the gas tank is empty, false otherwise.
    double m_drive_torque{0.0}; ///< Current torque produced by the engine.
    bool m_engaged{false}; ///< True if the clutch is fully engaged, false otherwise.
};
}

#endif // VAMOS_BODY_ENGINE_H_INCLUDED
