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
    Driver (Vamos_Body::Car* car_in) 
      : mp_car (car_in)
    {
    }

    virtual ~Driver () {}

    virtual void set_cars (const std::vector <Car_Information>* cars) {}
    /// Start driving.
    virtual void start (double to_go) {}
    /// True if the driver is driving.
    virtual bool is_started () const { return true; }

    virtual void reset () {}
    virtual void propagate (double time_step) {}
    virtual void draw () {}

    virtual void is_interactive () const { return true; }

    Vamos_Body::Car* mp_car;
  };
}

#endif // not _DRIVER_H_
