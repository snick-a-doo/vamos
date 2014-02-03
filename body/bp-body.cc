//  bp-body.cc - Boost.Python wrapper for libvamos-body.
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

#include "Gl_Car.h"
#include "../geometry/Three_Vector.h"
#include "../geometry/Three_Matrix.h"

#include <boost/python.hpp>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (body)
{
  bp::class_<Car> ("Car", bp::init <const Three_Vector&, const Three_Matrix&>());

  bp::class_<Gl_Car, bp::bases <Car> >
    ("Gl_Car", bp::init <const Three_Vector&, const Three_Matrix&>())
    .def ("read", &Gl_Car::read)
    .def ("start_engine", &Gl_Car::start_engine)
    .def ("adjust_robot_parameters", &Gl_Car::adjust_robot_parameters);
}
