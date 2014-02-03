//  bp-track.cc - Boost.Python wrapper for libvamos-track.
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

#include "../geometry/Three_Vector.h"
#include "Strip_Track.h"

#include <boost/python.hpp>

using namespace Vamos_Track;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (track)
{
  bp::class_<Road> ("Road", bp::init <>());

  bp::class_<Strip_Track> ("Strip_Track", bp::init <>())
    .def ("read", &Strip_Track::read)
    .def ("get_road", &Strip_Track::get_road,
          bp::return_value_policy <bp::copy_const_reference>())
    .def ("grid_position", &Strip_Track::grid_position,
          bp::return_value_policy <bp::return_by_value>())
    .def ("build_racing_line", &Strip_Track::build_racing_line)
    .def ("show_racing_line", &Strip_Track::show_racing_line);
}
