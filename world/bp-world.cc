//  bp-world.cc - Boost.Python wrapper for libvamos-world.
//
//  Copyright (C) 2014 Sam Varner
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

#include "../body/Car.h"
#include "../geometry/Three_Vector.h"
#include "../track/Strip_Track.h"
#include "Atmosphere.h"
#include "Interactive_Driver.h"
#include "Robot_Driver.h"
#include "Gl_World.h"
#include "Sounds.h"

#include <boost/python.hpp>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Track;
using namespace Vamos_World;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (world)
{
  bp::class_<Atmosphere> ("Atmosphere", bp::init <double, const Three_Vector&>());

  bp::class_<World> ("World", bp::init <Strip_Track&, Atmosphere&>())
    .def ("write_results", &World::write_results);

  bp::class_<Gl_World, bp::bases <World> > 
    ("Gl_World", bp::init <Strip_Track&, Atmosphere&, Sounds&, bool>())
    .def ("read", &Gl_World::read)
    .def ("start", &Gl_World::start)
    .def ("add_car", &Gl_World::add_car)
    .def ("set_focused_car", &Gl_World::set_focused_car)
    .def ("get_gravity", &Gl_World::get_gravity)
    .def ("cars_can_interact", &Gl_World::cars_can_interact);

  bp::class_<Driver> ("Driver", bp::init <Car&>());

  bp::class_<Interactive_Driver, bp::bases <Driver> >
    ("Interactive_Driver", bp::init <Car&>());

  bp::class_<Robot_Driver, bp::bases <Driver> >
    ("Robot_Driver", bp::init <Car&, Strip_Track&, double>())
    .def ("interact", &Robot_Driver::interact)
    .def ("show_steering_target", &Robot_Driver::show_steering_target)
    .def ("qualify", &Robot_Driver::qualify);

  bp::class_<Sounds> ("Sounds", bp::init <double>())
    .def ("read", &Sounds::read);
}