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

#ifndef _CLUTCH_H_
#define _CLUTCH_H_

namespace Vamos_Body
{
/// A clutch for the drivetrain.
class Clutch
{
public:
    Clutch(double sliding, double radius, double area, double max_pressure);

    /// Set the clutch position as a fraction of fully engaged.
    void set_position(double fraction);

    /// @return The drag caused by friction between the clutch plates.
    double drag(double engine_speed, double drive_speed);
    /// @return The current pressure.
    double pressure() const { return m_pressure; }
    /// @return The maximum pressure.
    double max_pressure() const { return m_max_pressure; }
    /// @return True if the clutch is fully engaged, false otherwise.
    bool is_engaged() const { return m_engaged; }

private:
    const double m_sliding_friction; ///< The coefficient of friction for the clutch plates.
    const double m_radius; ///< The effective radius of the clutch plates.
    const double m_area; ///< The contact area.
    const double m_max_pressure; ///< The maximum allowed pressure on the plates.
    double m_pressure{0.0}; ///< The current pressure on the plates.
    bool m_engaged{false}; ///< True if the clutch is fully engaged, false otherwise.
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_CLUTCH_H_INCLUDED
