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

#ifndef VAMOS_BODY_DRIVETRAIN_H_INCLUDED
#define VAMOS_BODY_DRIVETRAIN_H_INCLUDED

#include <memory>
#include <tuple>

namespace Vamos_Body
{
class Clutch;
class Differential;
class Engine;
class Transmission;

/// The drivetrain manages the engine, clutch, transmission and differential.
class Drivetrain
{
public:
    Drivetrain(std::shared_ptr<Engine> engine,
               std::unique_ptr<Clutch> clutch,
               std::unique_ptr<Transmission> transmission,
               std::unique_ptr<Differential> differential);

    /// Process the input parameters.
    void input(double gas, double clutch, double left_wheel_speed, double right_wheel_speed);
    /// Calculate forces and torques of the components.
    void find_forces();
    /// Return to the starting state.
    void reset();
    /// Advance the drivetrain in time.
    void propagate(double time);

    /// Allow access to the components.
    /// @{
    Engine& engine() { return *mp_engine; }
    Engine const& engine() const { return *mp_engine; }
    Transmission& transmission() { return *mp_transmission; }
    Transmission const& transmission() const { return *mp_transmission; }
    Clutch& clutch() { return *mp_clutch; }
    Clutch const& clutch() const { return *mp_clutch; }
    /// @}
    /// @return The torque for a driven wheel.
    std::tuple<double, double> get_torque() const;

private:
    std::shared_ptr<Engine> mp_engine;
    std::unique_ptr<Clutch> mp_clutch;
    std::unique_ptr<Transmission> mp_transmission;
    std::unique_ptr<Differential> mp_differential;
    double m_gas{0.0}; ///< The throttle position.
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_DRIVETRAIN_H_INCLUDED
