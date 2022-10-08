//  Copyright (C) 2003-2022 Sam Varner
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
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#include "controls.h"
#include "../geometry/numeric.h"

using namespace Vamos_Geometry;
using namespace Vamos_World;

// SDL reports joystick positions from -32768 to 32767.  We'll pretend it's -32767 to
// 32767.
auto constexpr RAW_MOTION_RANGE{32767};

//-----------------------------------------------------------------------------
double Callback_List::Callback::transform(double value) const
{
    // Handle half-range controls.
    if (!calibration.negative)
        value = std::max(value, 0.0);
    if (!calibration.positive)
        value = std::min(value, 0.0);

    // Handle deadbands.
    if (std::abs(value) < calibration.deadband)
        return calibration.offset;
    if (value < -1.0 + calibration.upper_deadband)
        return -calibration.factor + calibration.offset;
    if (value > 1.0 - calibration.upper_deadband)
        return calibration.factor + calibration.offset;

    auto m{calibration.factor / (1.0 - calibration.upper_deadband - calibration.deadband)};
    auto b{calibration.offset - sign(value) * m * calibration.deadband};
    return m * value + b;
}

void Callback_List::add(int index, Callback_Fn function, Calibration const& calib, double arg)
{
    m_callbacks.emplace_back(index, function, calib, arg);
}

void Callback_List::call(int index, double value)
{
    // Call any methods that match the index until one returns true or we exhaust the list.
    for (auto const& cb : m_callbacks)
        if (cb.index == index)
            if (cb.function(cb.transform(value), cb.argument))
                break;
}

//-----------------------------------------------------------------------------
void Control::bind_action(int index, Direct direction, Callback_Fn function, double time)
{
    auto& cbs{direction == Direct::up ? m_release_callbacks : m_press_callbacks};
    cbs.add(index, function, {}, time);
}

void Control::bind_motion(int axis, Callback_Fn function, Calibration const& calib)
{
    m_motion_callbacks.add(axis, function, calib, 0.0);
    if (m_ranges.find(axis) == m_ranges.end())
        set_axis_range(axis, -RAW_MOTION_RANGE, RAW_MOTION_RANGE);
}

void Control::bind_incremental(int index, Callback_Fn function)
{
    m_incremental_callbacks.add(index, function, {}, 0.0);
}

void Control::press(int index)
{
    m_press_callbacks.call(index, 1.0);
}

void Control::release(int index)
{
    m_release_callbacks.call(index, 0.0);
}

void Control::move(int axis, int position)
{
    m_motion_callbacks.call(axis, normalize(axis, position));
}

void Control::increment(int index, int delta)
{
    m_incremental_callbacks.call(index, delta);
}

void Control::set_axis_range(int axis, int low_raw_value, int high_raw_value)
{
    m_ranges[axis] = {low_raw_value, high_raw_value};
}

double Control::normalize(int axis, int value) const
{
    auto range{m_ranges.find(axis)->second};
    return 1.0 - 2.0 * (value - range.first) / (range.second - range.first);
}
