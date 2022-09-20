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
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_WORLD_CONTROLS_H_INCLUDED
#define VAMOS_WORLD_CONTROLS_H_INCLUDED

#include <functional>
#include <map>
#include <vector>

namespace Vamos_World
{
enum class Direct{none, up, down, left, right, forward, backward, in, out};

/// Convert a scoped enum to its integral type. See https://stackoverflow.com/a/14589519.
template <typename E> constexpr auto to_integral(E e) -> typename std::underlying_type<E>::type
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}

class Control;

struct Calibration
{
    bool negative{true};  ///< True if output can be negative
    bool positive{true};  ///< True if output can be positive
    double factor{1.0};   ///< Scaling of normalized value to output.
    double offset{0.0};   ///< Shift of normalized value to output.
    double deadband{0.0}; ///< Output is zero when abs(value) < this value.
    /// Output is maximum when value is > this value. Similarly for minimum value.
    double upper_deadband{0.0};
};

/// A member function of a Control_Handler.
using Callback_Fn = std::function<bool(double, double)>;

/// A class for managing callbacks
class Callback_List
{
public:
    /// Register a callback.
    void add(int index, Callback_Fn function, Calibration const& calib, double arg);
    /// Call a callback by index.
    void call(int index, double value);

private:
    struct Callback
    {
        /// @return The result of applying the calibration to @p value.
        double transform(double value) const;
        int index{0}; ///< The control key, button, or axis ID.
        Callback_Fn function; ///< The function to call when actuated.
        Calibration calibration{}; ///< The calibration to apply to the input.
        double argument{0.0}; ///< The value to pass to the callback.
    };
    std::vector<Callback> m_callbacks;
};

class Control
{
public:
    /// Bind a discrete control to callback.
    void bind_action(int index, Direct direction, Callback_Fn function, double time);
    /// Bind a continuous control to a callback.
    void bind_motion(int axis, Callback_Fn func, Calibration const& calib);
    /// Change the raw range of a continuous control.
    void set_axis_range(int axis, int low_raw_value, int high_raw_value);

    /// Call callbacks registered for axis movement.
    void move(int axis, int position);
    /// Call callbacks registered for key or button press.
    void press(int index);
    /// Call callbacks registered for key or button release.
    void release(int index);

private:
    /// Convert a raw value to a normalized value according to @p m_ranges.
    double normalize(int axis, int value) const;

    Callback_List m_press_callbacks;
    Callback_List m_release_callbacks;
    Callback_List m_motion_callbacks;
    std::map<int, std::pair<int, int>> m_ranges;
};

/// The base class for classes that can set control callbacks
class Control_Handler
{
public:
    Control& joystick() { return m_joystick; }
    Control& keyboard() { return m_keyboard; }
    Control& mouse() { return m_mouse; }

private:
    Control m_joystick;
    Control m_keyboard;
    Control m_mouse;
};
} // namespace Vamos_World

#endif // VAMOS_WORLD_CONTROLS_H_INCLUDED
