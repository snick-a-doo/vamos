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

#ifndef VAMOS_BODY_BRAKE_H_INCLUDED
#define VAMOS_BODY_BRAKE_H_INCLUDED

namespace Vamos_Body
{
/// A brake for a wheel.
class Brake
{
public:
    /// @param sliding The coefficient of sliding friction.
    /// @param radius The effective radius of the rotor.
    /// @param area Surface area of the rotor.
    /// @param max_pressure Pressure when brakes are fully applied.
    /// @param bias The fraction of braking given to this brake.
    Brake(double slideing, double radius, double area, double max_pressure, double bias);

    /// @param factor The fraction of full braking being applied.
    /// @param rotational_velocity The rotational speed of the wheel in rad/s.
    /// @return the torque exerted on the wheel by the brake.
    double torque(double factor, double rotational_velocity);
    // Rerturn true if the brake is locked, false otherwise.
    bool is_locked() const { return m_is_locked; }

private:
    double m_friction; ///< The sliding coefficient of friction for the brake pads on the rotor.
    double m_radius; ///< The effective radius of the rotor.
    double m_area; ///< The area of the brake pads.
    double m_max_pressure; ///< The maximum allowed pressure.
    double m_bias; ///< The fraction of the pressure to be applied to the brake.
    double m_threshold{4.0e-4}; ///< The brake locks when the linear brake velocity divided by pl
    bool m_is_locked{false}; ///< true if the brake is locked, false otherwise.
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_BRAKE_H_INCLUDED
