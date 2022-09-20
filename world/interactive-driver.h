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

#ifndef VAMOS_WORLD_INTERACTIVE_DRIVER_H_INCLUDED
#define VAMOS_WORLD_INTERACTIVE_DRIVER_H_INCLUDED

#include "controls.h"
#include "driver.h"

namespace Vamos_World
{
class Interactive_Driver : public Driver, public Control_Handler
{
public:
    Interactive_Driver(std::shared_ptr<Vamos_Body::Car> car);

    /// Control callbacks
    /// @{
    bool start_engine(double, double);
    bool fill_tank(double, double);

    bool gas(double, double);
    bool brake(double, double);
    bool steer(double, double);
    bool steer_left(double, double);
    bool steer_right(double, double);

    bool initial_shift_up(double, double);
    bool initial_shift_down(double, double);
    bool shift_up(double, double);
    bool shift_down(double, double);
    bool initial_shift_up_disengage(double, double);
    bool initial_shift_down_disengage(double, double);
    bool shift_up_disengage(double, double);
    bool shift_down_disengage(double, double);
    bool initial_clutch(double, double);
    bool clutch(double, double);
    bool initial_engage_clutch(double, double);
    bool initial_disengage_clutch(double, double);
    bool engage_clutch(double, double);
    bool disengage_clutch(double, double);

    bool pan_left(double, double);
    bool pan_right(double, double);
    /// @}
};
} // namespace Vamos_World

#endif // VAMOS_WORLD_INTERACTIVE_DRIVER_H_INCLUDED
