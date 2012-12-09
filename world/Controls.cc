//  Controls.cc - a class for handling control events.
//
//	Vamos Automotive Simulator
//  Copyright (C) 2003 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#include "Controls.h"
#include "../geometry/Numeric.h"

using namespace Vamos_Geometry;
using namespace Vamos_World;

const int RAW_MOTION_RANGE = 32767;
// SDL reports joystick positions from -32768 to 32767.  We'll pretend
// it's -32767 to 32767.

//-----------------------------------------------------------------------------
Callback_List::Callback::Callback (int i,
                                   Control_Handler* obj,
                                   Callback_Function func, 
                                   const Calibration& cal,
                                   double arg)
  : index (i),
    object (obj),
    function (func), 
    calibration (cal),
    argument (arg)
{
}

double
Callback_List::Callback::transform (double value) const
{
  // Handle half-range controls.
  if (!calibration.negative)
    value = std::max (value, 0.0);
  if (!calibration.positive)
    value = std::min (value, 0.0);

  // Handle deadbands.
  if (std::abs (value) < calibration.deadband) return calibration.offset;
  if (value < -1.0 + calibration.upper_deadband) 
    return -calibration.factor + calibration.offset; 
  if (value > 1.0 - calibration.upper_deadband) 
    return calibration.factor + calibration.offset; 

  double m = calibration.factor 
    / (1.0 - calibration.upper_deadband - calibration.deadband);
  double b = calibration.offset - sign (value) * m * calibration.deadband;
  return m * value + b;
}

void 
Callback_List::add (int index,
                    Control_Handler* object,
                    Callback_Function function,
                    const Calibration& calibration,
                    double argument)
{
  m_callbacks.push_back (Callback (index, object, function, calibration, argument));
}

void
Callback_List::call (int index, double value)
{
  bool done_calling = false;
  for (std::vector <Callback>::const_iterator it = m_callbacks.begin ();
	   (it != m_callbacks.end ()) && !done_calling;
	   it++)
	{
      if (index == it->index)
        done_calling = ((it->object)->*(it->function)) 
          (it->transform (value), it->argument);
	}
}

//-----------------------------------------------------------------------------
void 
Control::bind_action (int index, 
                      Direction direction,
                      Control_Handler* object,
                      Callback_Function function,
                      double time)
{
  if (direction == UP)
    m_release_callbacks.add (index, object, function, Calibration (), time);
  else
    m_press_callbacks.add (index, object, function, Calibration (), time);
} 

void 
Control::bind_motion (int axis,
                      Direction direction,
                      Control_Handler* object, 
                      Callback_Function function,
                      double factor,
                      double offset,
                      double deadband,
                      double upper_deadband)
{
  const bool negative = (direction != FORWARD && direction != RIGHT);
  const bool positive = (direction != BACKWARD && direction != LEFT);
  m_motion_callbacks.add (axis, object, function,
                          Calibration (negative,
                                       positive, 
                                       factor,
                                       offset,
                                       deadband,
                                       upper_deadband));
  if (m_ranges.find (axis) == m_ranges.end ())
    set_axis_range (axis, -RAW_MOTION_RANGE, RAW_MOTION_RANGE);
}

void 
Control::press (int index)
{
  m_press_callbacks.call (index, 1.0);
}

void 
Control::release (int index)
{
  m_release_callbacks.call (index, 0.0);
}

void
Control::move (int axis, int position)
{
  m_motion_callbacks.call (axis, transform (axis, position));
}

void
Control::set_axis_range (int axis, int low_raw_value, int high_raw_value)
{
  m_ranges [axis] = 
    std::pair <int, int> (low_raw_value, high_raw_value);  
}

double
Control::transform (int axis, int value) const
{
  std::pair <int, int> range = m_ranges.find (axis)->second;
  return 1.0 - 2.0 * (value - range.first) / (range.second - range.first);
}
