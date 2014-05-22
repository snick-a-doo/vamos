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

#ifndef PARAMETER_H
#define PARAMETER_H

#include <vector>
#include <cstddef>

/// A container for numbers passed from the command line.
namespace Vamos_Geometry
{
  class Parameter
  {
  public:
    /// Save a passed-in number.
    static void set (double value) { m_values.push_back (value); }

    /// Get a parameter.
    /// @param i the index of the parameter
    /// @fallback the value to return if the ith parameter was not set.
    static double get (size_t i, double fallback = 0.0);

    /// Return the number of parameters set.
    static double size ()
    { return m_values.size (); }

  private:
    static std::vector <double> m_values;
  };
}
  
#endif
