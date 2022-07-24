//  Parameter.h - numbers passed from the command line.
//
//  Copyright (C) 2011 Sam Varner
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

#include "parameter.h"

using namespace Vamos_Geometry;

std::vector <double> Parameter::m_values;

double 
Parameter::get (size_t i, double fallback)
{ 
  return i < size () ? m_values [i] : fallback; 
}
