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

#ifndef VAMOS_BODY_TRANSMISSION_H_INCLUDED
#define VAMOS_BODY_TRANSMISSION_H_INCLUDED

#include <map>

namespace Vamos_Body
{
/// A gearbox for the drivetrain.
class Transmission
{
public:
    /// Let the program calculate equally log-spaced gear ratios.
    Transmission(int forward_gears, double first_ratio, double last_ratio);
    Transmission();

    /// Shift to a particular gear.
    void shift(int gear);
    /// Set the gear ratio for @p gear.
    /// @param ratio (driveshaft speed) / (engine speed). Forward gears have positive
    /// ratios, usually > 1.0.  Neutral has a rato of 0.0 by default.
    void set_gear_ratio(int gear, double ratio);
    /// @return The ratio for a specified gear.
    double gear_ratio(int gear) { return m_gear_ratios[gear]; }
    /// @return The torque on the driveshaft due to friction at the clutch.
    double torque(double drag);
    /// @return The current gear.
    int gear() const { return m_gear; }
    /// @return True if the current gear is neutral.
    bool is_in_neutral() const { return m_gear == 0; }
    /// @return The number of consecutive forward gears.
    int forward_gears() const { return m_forward_gears; }
    /// @return The number of consecutive reverse gears.
    int reverse_gears() const { return m_reverse_gears; }
    /// Tell the transmission what the rotational speed of the driveshaft is.
    void set_driveshaft_speed(double speed);
    /// @return The rotational speed at the clutch side of the transmission.
    double clutch_speed() const { return m_clutch_speed; }

private:
    std::map<int, double> m_gear_ratios; ///< A map of gears to gear ratios.
    int m_forward_gears{0}; ///< Number of consecutive forward gears.
    int m_reverse_gears{0}; ///< Number of consecutive reverse gears.
    int m_gear{0}; ///< Current gear.
    double m_clutch_speed{0.0}; ///< Rotational speed at the clutch side of the transmission.
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_TRANSMISSION_H_INCLUDED
