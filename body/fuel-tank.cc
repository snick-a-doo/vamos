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

#include "fuel-tank.h"

#include "../geometry/three-vector.h"

using namespace Vamos_Geometry;
using namespace Vamos_Body;

Fuel_Tank::Fuel_Tank(Three_Vector const& position,
                     double capacity, double volume, double density)
    : Particle{volume * density, position},
      m_capacity{capacity},
      m_volume{volume},
      m_density{density}
{
}

void Fuel_Tank::fill(double volume)
{
    m_volume = volume;
    set_mass(m_volume * m_density);
}

void Fuel_Tank::fill()
{
    fill(m_capacity);
}

double Fuel_Tank::consume(double amount)
{
    m_volume = amount >= m_volume ? 0.0 : m_volume - amount;
    set_mass(m_volume * m_density);
    return m_volume;
}
