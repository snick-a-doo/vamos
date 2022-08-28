//  Copyright (C) 2008-2022 Sam Varner
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

#ifndef VAMOS_GEOMETRY_PID_H_INCLUDED
#define VAMOS_GEOMETRY_PID_H_INCLUDED

#include <iosfwd>

/// A PID controller
///
/// The controller provides a value to be used to as an input parameter for
/// some process.  The output value is calculated to minimize the difference
/// between a 'process variable' and its desired value -- the 'setpoint'.
///
/// For example, during acceleration, we wish to keep tire slip at a value that
/// gives maximum acceleration, say, 10%.  The process variable is the actual
/// slip ratio and the setpoint is 10%.  The output of the controller is the
/// throttle setting.  The P, I, and D parameters must be chosen so that the
/// throttle is set appropriately when the slip ratio is not at the setpoint.
namespace Vamos_Geometry
{
class PID
{
public:
    /// Set the proportional, integral, and derivative gains. Optionally, set the time
    /// constant for the integral term.
    PID(double p, double i, double d, double integral_decay = 0.0);
    /// Give a new setpoint.
    void set(double setpoint);
    /// @return The current setpoint.
    double setpoint() const { return m_setpoint; }
    /// Zero the integral and cumulative time.
    void reset();
    /// @param input The process variable that we are trying to control at the setpoint.
    /// @param dt The time since the last reading. We assume @p input has been at its
    /// current value for this long.
    /// @return The new control parameter output.
    double propagate(double input, double dt);
    /// @return The controller output.
    double operator()() const;

private:
    double m_kp; //< Proportional gain
    double m_ki; //< Integral gain
    double m_kd; //< Derivative gain

    double m_proportional{0.0}; //< Proportional term
    double m_integral{0.0};     //< Integral term
    double m_derivative{0.0};   //< Derivative term

    /// Time constant for decay of the integral term.  This reduces the influence of the
    /// accumulated integral term on the output at later times.
    double m_integral_decay{0.0};

    double m_previous_error{0.0}; ///< The previous error for the derivative term.
    double m_setpoint{0.0}; ///< The most recent setpoint.
    double m_cumulative_time{0.0}; /// Total time since construction or the last reset().

    friend std::ostream& operator << (std::ostream& os, PID const& pid);
};

std::ostream& operator << (std::ostream& os, PID const& pid);
} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_PID_H_INCLUDED
