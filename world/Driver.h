//  Copyright (C) 2008--2014 Sam Varner
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

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "Controls.h"

#include <vector>

namespace Vamos_Body
{
  class Car;
}

namespace Vamos_World
{
  struct Car_Information;

  class Driver
  {
  public:
    Driver (Vamos_Body::Car& car_in) 
      : mp_car (&car_in)
    {
    }

    virtual ~Driver () {}

    virtual void set_cars (const std::vector <Car_Information>* cars) {}
    /// Start driving.
    virtual void start (double to_go) {}
    virtual void finish () {}
    /// True if the driver is driving.
    virtual bool is_driving () const { return true; }

    virtual void reset () {}
    virtual void propagate (double time_step) {}
    virtual void draw () {}

    virtual bool is_interactive () const { return true; }

    Vamos_Body::Car* mp_car;
  };
}

#endif // not _DRIVER_H_
