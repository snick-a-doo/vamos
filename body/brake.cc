//  Copyright (C) 2001-2022  Sam Varner
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

#include "brake.h"

#include "../geometry/numeric.h"

#include <cmath>

using Vamos_Geometry::sign;

Vamos_Body::Brake::Brake(double sliding, double radius, double area, double max_pressure,
                         double bias)
    : m_friction(sliding),
      m_radius(radius),
      m_area(area),
      m_max_pressure(max_pressure * bias),
      m_bias(bias)
{
}

double Vamos_Body::Brake::torque(double factor, double rotational_speed)
{
    auto pressure{factor * m_bias * m_max_pressure};
    auto normal{pressure * m_area};
    auto torque = m_friction * normal * m_radius * sign(rotational_speed);
    auto velocity = m_radius * rotational_speed;

    m_is_locked = std::abs(velocity) < m_threshold * normal;
    return m_is_locked ? 0.0 : torque;
}
