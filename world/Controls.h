//  Controls.h - a class for handling control events.
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

#ifndef _CONTROLS_H_
#define _CONTROLS_H_

#include "../geometry/Constants.h"

#include <vector>
#include <map>
#include <iostream>

namespace Vamos_World
{
  class Control;

  // The base class for classes that can set control callbacks
  class Control_Handler 
  {
  public:
    virtual Control& joystick () = 0;
    virtual Control& keyboard () = 0;
    virtual Control& mouse () = 0;
  };

  // The callback function pointer type
  typedef bool (Control_Handler::* Callback_Function) (double, double);

  struct Calibration
  {
    Calibration (bool neg = true,
                 bool pos = true,
                 double fact = 1.0,
                 double off = 0.0,
                 double dead = 0.0,
                 double upper = 0.0)
      : negative (neg),
        positive (pos),
        factor (fact), 
        offset (off), 
        deadband (dead),
        upper_deadband (upper)
    {};

    bool negative;
    bool positive;
    double factor;
    double offset;
    double deadband;
    double upper_deadband;
  };

  // A class for managing callbacks
  class Callback_List
  {
  public:
    void add (int index,
              Control_Handler* object,
              Callback_Function function,
              const Calibration& calibration,
              double argument = 0.0);
    void call (int index, double value);

  private:
    struct Callback
    {
      Callback (int i,
                Control_Handler* obj,
                Callback_Function func,
                const Calibration& cal,
                double arg = 0.0);
      
      int index;
      Control_Handler* object;
      Callback_Function function;
      Calibration calibration;
      double argument;

      double transform (double value) const;
    };
    std::vector <Callback> m_callbacks;
  };

  class Control
  {
  public:
    void bind_action (int index, 
                      Vamos_Geometry::Direction direction,
                      Control_Handler* object,
                      Callback_Function function,
                      double time);

    void bind_motion (int axis,
                      Vamos_Geometry::Direction direction,
                      Control_Handler* object, 
                      Callback_Function func,
                      double factor,
                      double offset,
                      double deadband,
                      double upper_deadband);

    void move (int axis, int position);
    void press (int index);
    void release (int index);

    void set_axis_range (int axis, int low_raw_value, int high_raw_value);

  protected:
    double transform (int axis, int value) const;

  private:
    Callback_List m_press_callbacks;
    Callback_List m_release_callbacks;
    Callback_List m_motion_callbacks;
    std::map <int, std::pair <int, int> > m_ranges;
  };
}

#endif // not _CONTROLS_H_
