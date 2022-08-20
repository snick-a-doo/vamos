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

#include "../geometry/numeric.h"
#include "clutch.h"

#include <cmath>

/// The clutch is fully engaged when engine speed - transmission speed <
/// normal force x @p lock_threshold
double constexpr lock_threshold{0.01};

using namespace Vamos_Body;

Clutch::Clutch(double sliding, double radius, double area, double max_pressure)
    : m_sliding_friction{sliding},
      m_radius{radius},
      m_area{area},
      m_max_pressure{max_pressure}
{
}

double Clutch::drag(double engine_speed, double drive_speed)
{
    using Vamos_Geometry::sign;
    double normal{m_pressure * m_area};
    double rel_speed{engine_speed - drive_speed};
    if (std::abs(rel_speed) < lock_threshold * normal)
        m_engaged = true;
    return m_engaged ? 0.0 : m_sliding_friction * normal * m_radius * sign(rel_speed);
}

void Clutch::set_position(double factor)
{
    m_engaged = factor >= 1.0;
    m_pressure = std::min(factor, 1.0) * m_max_pressure;
}
