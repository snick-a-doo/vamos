//  bp-media.cc - Boost.Python wrapper for libvamos-media.
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

#include "../body/Gl_Car.h"
#include "XML_Parser.h"

#include <boost/python.hpp>

using namespace Vamos_Media;
namespace bp = boost::python;

void translate (XML_Exception const& e)
{
  PyErr_SetString (PyExc_RuntimeError, e.message().c_str ());
}

BOOST_PYTHON_MODULE (media)
{
  bp::register_exception_translator <XML_Exception> (&translate);

  bp::class_<XML_Exception> ("XML_Exception", bp::init <std::string, int, std::string>())
    .def ("message", &XML_Exception::message);
}
