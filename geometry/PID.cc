//  PID.cc - a proportional, integral, derivative controller.
//
//  Copyright (C) 2008 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#include "PID.h"

#include <algorithm>
#include <iostream>

using namespace Vamos_Geometry;

PID::PID (double p, double i, double d, double integral_decay)
: m_kp (p),
  m_ki (i),
  m_kd (d),
  m_proportional (0.0),
  m_integral (0.0),
  m_derivative (0.0),
  m_integral_decay (integral_decay)
{
  set (0.0);
  reset ();
}

void
PID::set (double setpoint)
{
  m_setpoint = setpoint;
}

void
PID::reset ()
{
  m_integral = 0.0;
  m_cumulative_time = 0.0;
}

double PID::propagate (double signal, double dt)
{
  const double error = m_setpoint - signal;
  m_proportional = m_kp * error;
  m_integral += m_ki * error * dt - m_integral * dt * m_integral_decay;
  m_integral = std::max (m_integral, 0.0);
  m_derivative = ((m_cumulative_time == 0.0) || (dt == 0.0))
    ? 0.0
    : m_kd * (error - m_previous_error) / dt;

  m_previous_error = error;
  m_cumulative_time += dt;

  return (*this)();
}

double PID::operator () () const
{
  return m_proportional + m_integral + m_derivative;
}

std::ostream& PID::output (std::ostream& os) const
{
  os << m_setpoint << ' ' << m_proportional << ' ' << m_integral << ' ' << m_derivative;
  return os;
}
