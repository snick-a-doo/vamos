//  Interactive_Driver.cc - a human-controlled driver
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

#include "Interactive_Driver.h"
#include "../body/Car.h"
#include "../body/Fuel_Tank.h"

using namespace Vamos_World;

Interactive_Driver::Interactive_Driver (Vamos_Body::Car* car_in)
  : Driver (car_in)
{
}

bool 
Interactive_Driver::start_engine (double, double)
{
  mp_car->start_engine ();
  return true;
}

bool 
Interactive_Driver::fill_tank (double, double)
{
  mp_car->fuel_tank ()->fill ();
  return true;
}

bool 
Interactive_Driver::gas (double value, double time)
{
  mp_car->gas (value, time);
  return false;
}

bool 
Interactive_Driver::brake (double value, double time)
{
  mp_car->brake (value, time);
  return false;
}

bool 
Interactive_Driver::steer (double value, double time)
{
  mp_car->steer (value, time);
  return false;
}

bool 
Interactive_Driver::steer_left (double value, double time)
{
  mp_car->steer (value, time);
  return true;
}

bool 
Interactive_Driver::steer_right (double value, double time)
{
  mp_car->steer (-value, time);
  return true;
}

bool 
Interactive_Driver::initial_shift_up (double, double)
{
  if (mp_car->gear () == 0)
    {
      mp_car->shift_up ();
      return true;
    }
  return false;
}

bool 
Interactive_Driver::initial_shift_down (double, double)
{
  if (mp_car->gear () == 0)
    {
      mp_car->shift_down ();
      return true;
    }
  return false;
}

bool 
Interactive_Driver::shift_up (double, double)
{
  if (mp_car->gear () != 0)
    {
      mp_car->shift_up ();
      return true;
    }
  return false;
}

bool 
Interactive_Driver::shift_down (double, double)
{
  if (mp_car->gear () != 0)
    {
      mp_car->shift_down ();
      return true;
    }
  return false;
}

bool 
Interactive_Driver::initial_shift_up_disengage (double, double time)
{
  if (mp_car->gear () == 0)
    {
      mp_car->disengage_clutch (time);
      mp_car->shift_up ();
      return true;
    }
  return false;
}

bool 
Interactive_Driver::initial_shift_down_disengage (double, double time)
{
  if (mp_car->gear () == 0)
    {
      mp_car->disengage_clutch (time);
      mp_car->shift_down ();
      return true;
    }
  return false;
}

bool 
Interactive_Driver::shift_up_disengage (double, double time)
{
  if (mp_car->gear () != 0)
    {
      if (!mp_car->fast_shift ())
        {
          mp_car->disengage_clutch (time);
        }
      mp_car->shift_up ();
      return true;
    }
  return false;
}

bool 
Interactive_Driver::shift_down_disengage (double, double time)
{
  if (mp_car->gear () != 0)
    {
      if (!mp_car->fast_shift ())
        {
          mp_car->disengage_clutch (time);
        }
      mp_car->shift_down ();
      return true;
    }
  return false;
}

bool 
Interactive_Driver::initial_clutch (double value, double)
{
  if (mp_car->last_gear () == 0)
    {
      mp_car->clutch (value);
      return true;
    }
  return false;
}

bool 
Interactive_Driver::clutch (double value, double)
{
  if (mp_car->last_gear () != 0)
    {
      mp_car->clutch (value);
      return true;
    }
  return false;
}

bool 
Interactive_Driver::initial_engage_clutch (double, double time)
{
  if (mp_car->last_gear () == 0)
    {
      mp_car->engage_clutch (time);
      return true;
    }
  return false;
}

bool 
Interactive_Driver::initial_disengage_clutch (double, double time)
{
  if (mp_car->last_gear () == 0)
    {
      mp_car->disengage_clutch (time);
      return true;
    }
  return false;
}

bool 
Interactive_Driver::engage_clutch (double, double time)
{
  if (mp_car->last_gear () != 0)
    {
      mp_car->engage_clutch (time);
      return true;
    }
  return false;
}

bool 
Interactive_Driver::disengage_clutch (double, double time)
{
  if (mp_car->last_gear () != 0)
    {
      mp_car->disengage_clutch (time);
      return true;
    }
  return false;
}

bool 
Interactive_Driver::pan_left (double value, double time)
{
  mp_car->pan (value, time);
  return true;
}

bool 
Interactive_Driver::pan_right (double value, double time)
{
  mp_car->pan (-value, time);
  return true;
}
