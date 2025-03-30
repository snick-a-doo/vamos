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

#ifndef VAMOS_WORLD_DRIVER_H_INCLUDED
#define VAMOS_WORLD_DRIVER_H_INCLUDED

#include <memory>
#include <vector>

namespace Vamos_Body
{
class Car;
}
namespace Vamos_Geometry
{
class Three_Vector;
}

namespace Vamos_World
{
struct Car_Info;

class Driver
{
public:
    Driver(std::shared_ptr<Vamos_Body::Car> car)
        : mp_car{car}
    {}
    virtual ~Driver() = default;

    /// Specify the gravitational acceleration vector.
    virtual void set_gravity(Vamos_Geometry::Three_Vector const&) {}
    /// Give access to the full set of cars.
    virtual void set_cars(std::vector<Car_Info> const* /* cars */) {}
    /// Start driving.
    virtual void start(double /* to_go */) {}
    /// End the event.
    virtual void finish() {}
    /// True if the driver is driving.
    virtual bool is_driving() const { return true; }
    /// Advance in time.
    virtual void propagate(double /* time_step */) {}
    /// Render the driver.
    virtual void draw() {}
    /// True for human-controlled drivers.
    virtual bool is_interactive() const { return true; }

protected:
    std::shared_ptr<Vamos_Body::Car> mp_car; ///< The controlled car.
};
} // namespace Vamos_World

#endif // VAMOS_WORLD_DRIVER_H_INCLUDED
