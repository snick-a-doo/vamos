//  Copyright (C) 2002-2022 Sam Varner
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

#ifndef VAMOS_BODY_FUEL_TANK_H_INCLUDED
#define VAMOS_BODY_FUEL_TANK_H_INCLUDED

#include "particle.h"

namespace Vamos_Geometry
{
class Three_Vector;
}
namespace Vamos_Body
{
/// A container of fuel for the engine. The mass of this particle is the mass of the fuel.
class Fuel_Tank : public Particle
{
public:
    Fuel_Tank(Vamos_Geometry::Three_Vector const& position,
              double capacity, double volume, double density);

    /// Fill the tank to a specific level. Negative values are clipped to zero, positive
    /// value are clipped to tank capacity.
    void fill(double volume);
    /// Fill the tank to capacity.
    void fill();
    /// Decrease the volume of fuel.
    /// @param amount The volume to remove.
    /// @return The volume remaining.
    double consume(double amount);
    /// @return The volume of fuel remaining.
    double fuel() const { return m_volume; }
    /// @return True if the tank is empty.
    bool empty() const { return m_volume <= 0.0; }

private:
    double m_capacity; ///< The volume of the tank.
    double m_volume; ///< The remaining volume of fuel.
    double m_density; ///< The density of the fuel.
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_FUEL_TANK_H_INCLUDED
