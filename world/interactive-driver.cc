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

#include "interactive-driver.h"

#include "../body/car.h"
#include "../body/fuel-tank.h"

using namespace Vamos_World;

Interactive_Driver::Interactive_Driver(std::shared_ptr<Vamos_Body::Car> car)
    : Driver{car}
{
}

bool Interactive_Driver::start_engine(double, double)
{
    mp_car->start_engine();
    return true;
}

bool Interactive_Driver::fill_tank(double, double)
{
    mp_car->fuel_tank()->fill();
    return true;
}

bool Interactive_Driver::gas(double value, double time)
{
    mp_car->gas(value, time);
    return false;
}

bool Interactive_Driver::brake(double value, double time)
{
    mp_car->brake(value, time);
    return false;
}

bool Interactive_Driver::steer(double value, double time)
{
    mp_car->steer(value, time);
    return false;
}

bool Interactive_Driver::steer_left(double value, double time)
{
    mp_car->steer(value, time);
    return true;
}

bool Interactive_Driver::steer_right(double value, double time)
{
    mp_car->steer(-value, time);
    return true;
}

bool Interactive_Driver::initial_shift_up(double, double)
{
    if (mp_car->gear() != 0)
        return false;

    mp_car->shift_up();
    return true;
}

bool Interactive_Driver::initial_shift_down(double, double)
{
    if (mp_car->gear() != 0)
        return false;

    mp_car->shift_down();
    return true;
}

bool Interactive_Driver::shift_up(double, double)
{
    if (mp_car->gear() == 0)
        return false;

    mp_car->shift_up();
    return true;
}

bool Interactive_Driver::shift_down(double, double)
{
    if (mp_car->gear() == 0)
        return false;

    mp_car->shift_down();
    return true;
}

bool Interactive_Driver::initial_shift_up_disengage(double, double time)
{
    if (mp_car->gear() != 0)
        return false;

    mp_car->disengage_clutch(time);
    mp_car->shift_up();
    return true;
}

bool Interactive_Driver::initial_shift_down_disengage(double, double time)
{
    if (mp_car->gear() != 0)
        return false;

    mp_car->disengage_clutch(time);
    mp_car->shift_down();
    return true;
}

bool Interactive_Driver::shift_up_disengage(double, double time)
{
    if (mp_car->gear() == 0)
        return false;

    if (!mp_car->fast_shift())
        mp_car->disengage_clutch(time);
    mp_car->shift_up();
    return true;
}

bool Interactive_Driver::shift_down_disengage(double, double time)
{
    if (mp_car->gear() == 0)
        return false;

    if (!mp_car->fast_shift())
        mp_car->disengage_clutch(time);
    mp_car->shift_down();
    return true;
}

bool Interactive_Driver::initial_clutch(double value, double)
{
    if (mp_car->last_gear() != 0)
        return false;

    mp_car->clutch(value);
    return true;
}

bool Interactive_Driver::clutch(double value, double)
{
    if (mp_car->last_gear() == 0)
        return false;

    mp_car->clutch(value);
    return true;
}

bool Interactive_Driver::initial_engage_clutch(double, double time)
{
    if (mp_car->last_gear() != 0)
        return false;

    mp_car->engage_clutch(time);
    return true;
}

bool Interactive_Driver::initial_disengage_clutch(double, double time)
{
    if (mp_car->last_gear() != 0)
        return false;

    mp_car->disengage_clutch(time);
    return true;
}

bool Interactive_Driver::engage_clutch(double, double time)
{
    if (mp_car->last_gear() == 0)
        return false;

    mp_car->engage_clutch(time);
    return true;
}

bool Interactive_Driver::disengage_clutch(double, double time)
{
    if (mp_car->last_gear() == 0)
        return false;

    mp_car->disengage_clutch(time);
    return true;
}

bool Interactive_Driver::pan_left(double value, double time)
{
    mp_car->pan(value, time);
    return true;
}

bool Interactive_Driver::pan_right(double value, double time)
{
    mp_car->pan(-value, time);
    return true;
}
