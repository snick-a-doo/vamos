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

#ifndef VAMOS_BODY_DIFFERENTIAL_H_INCLUDED
#define VAMOS_BODY_DIFFERENTIAL_H_INCLUDED

#include <tuple>

namespace Vamos_Body
{
/// The differential gear system of the drivetrain.
class Differential
{
public:
    /// @param final_drive The differential's gear ratio.
    /// @param anti_slip The amount of friction between the wheels.
    Differential(double final_drive, double anti_slip);

    /// @return The rotational speed of the driveshaft given the speed of the wheels.
    double get_driveshaft_speed(double left_wheel_speed, double right_wheel_speed);
    /// Calculate the torques on the wheels.
    void find_torques(double driveshaft_torque);
    /// @return The torque on the wheels.
    std::tuple<double, double> get_torque() const;

private:
    double m_final_drive; ///< The differential's gear ratio.
    double m_anti_slip; ///< The amount of friction between the axles.
    double m_left_speed{0.0}; ///< The angular speeds of the left wheel.
    double m_right_speed{0.0}; ///< The angular speeds of the right wheel.
    double m_left_torque{0.0}; ///< The torques on the left wheel.
    double m_right_torque{0.0}; ///< The torques on the right wheel.
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_DIFFERENTIAL_H_INCLUDED
