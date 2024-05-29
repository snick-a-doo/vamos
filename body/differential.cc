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

#include "differential.h"

#include <numeric>

using namespace Vamos_Body;

Differential::Differential(double final_drive, double anti_slip)
    : m_final_drive{final_drive},
      m_anti_slip(anti_slip)
{
}

double Differential::get_driveshaft_speed(double left_speed, double right_speed)
{
    m_left_speed = left_speed;
    m_right_speed = right_speed;
    return m_final_drive * std::midpoint(left_speed, right_speed);
}

std::tuple<double, double> Differential::get_torque() const
{
    return {m_left_torque, m_right_torque};
}

void Differential::find_torques(double driveshaft_torque)
{
    auto torque{driveshaft_torque * m_final_drive / 2.0};
    auto drag{m_anti_slip * (m_left_speed - m_right_speed)};
    m_left_torque = torque - drag;
    m_right_torque = torque + drag;
}
