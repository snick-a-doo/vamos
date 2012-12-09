//  Interactive_Driver.h - a human-controlled driver
//
//	Vamos Automotive Simulator
//  Copyright (C) 2008 Sam Varner
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

#ifndef _INTERACTIVE_DRIVER_H_
#define _INTERACTIVE_DRIVER_H_

#include "Controls.h"
#include "Driver.h"

namespace Vamos_World
{
  class Interactive_Driver : public Driver, public Control_Handler
  {
  public:
    Interactive_Driver (Vamos_Body::Car* car_in);

    virtual Control& joystick () { return m_joystick; }
    virtual Control& keyboard () { return m_keyboard; }
    virtual Control& mouse () { return m_mouse; }
    
    Control m_joystick;
    Control m_keyboard;
    Control m_mouse;
    
	bool start_engine (double, double);// Start the car's engine.
	bool fill_tank (double, double);   // Fill the car's gas tank.

    bool gas (double, double);
    bool brake (double, double);
    bool steer (double, double);
	bool steer_left (double, double);
	bool steer_right (double, double);

	bool initial_shift_up (double, double);
	bool initial_shift_down (double, double);
	bool shift_up (double, double);
	bool shift_down (double, double);
	bool initial_shift_up_disengage (double, double);
	bool initial_shift_down_disengage (double, double);
	bool shift_up_disengage (double, double);
	bool shift_down_disengage (double, double);
	bool initial_clutch (double, double);
	bool clutch (double, double);
	bool initial_engage_clutch (double, double);
	bool initial_disengage_clutch (double, double);
	bool engage_clutch (double, double);
	bool disengage_clutch (double, double);

	bool pan_left (double, double);
	bool pan_right (double, double);
  };
}

#endif // not _INTERACTIVE_DRIVER_H_
