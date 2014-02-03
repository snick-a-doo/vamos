//  bp-geometry.cc - Boost.Python wrapper for libvamos-geometry.
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

#include "Three_Matrix.h"
#include "Three_Vector.h"

#include <boost/python.hpp>

using namespace Vamos_Geometry;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (geometry)
{
  bp::class_<Three_Matrix> ("Three_Matrix", bp::init <>());
  bp::class_<Three_Vector> ("Three_Vector", bp::init <double, double, double>())
    .def (bp::init <>());
}
