//  Copyright (C) 2001--2002 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#include "transmission.h"

#include <cassert>

using namespace Vamos_Body;

Transmission::Transmission(int forward_gears, double first_ratio, double last_ratio)
    : m_forward_gears{forward_gears},
      m_reverse_gears{1}
{
    assert(forward_gears > 1);
    auto first_inv{1.0 / first_ratio};
    auto last_inv{1.0 / last_ratio};
    auto increment{(first_inv - last_inv) / (forward_gears - 1)};
    m_gear_ratios[0] = 0.0;
    for (auto i{0}; i <= forward_gears; ++i)
        m_gear_ratios[i + 1] = 1.0 / (first_inv - increment * i);
    // Put the ratio for reverse midway between first and second.
    m_gear_ratios[-1] = -0.5 * (m_gear_ratios[1] + m_gear_ratios[2]);
}

Transmission::Transmission()
{
    m_gear_ratios[0] = 0.0;
}

void Transmission::shift(int gear)
{
    m_gear = gear;
}

void Transmission::set_gear_ratio(int gear, double ratio)
{
    // One driveshaft revolution yields `ratio' engine revolutions.  ratio == 0.0
    // indicates neutral.
    m_gear_ratios[gear] = ratio;

    // Find the number of consecutive forward gears.
    m_forward_gears = 0;
    int key{0};
    while (m_gear_ratios.find(++key) != m_gear_ratios.end())
        ++m_forward_gears;

    // Find the number of consecutive reverse gears.
    m_reverse_gears = 0;
    key = 0;
    while (m_gear_ratios.find(--key) != m_gear_ratios.end())
        ++m_reverse_gears;
}

double Transmission::torque(double drag)
{
    return drag * m_gear_ratios[m_gear];
}

void Transmission::set_driveshaft_speed(double driveshaft_speed)
{
    m_clutch_speed = driveshaft_speed * m_gear_ratios[m_gear];
}
