//  Parameter.h - numbers passed from the command line.
//
//	Vamos Automotive Simulator
//  Copyright (C) 2011 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef PARAMETER_H
#define PARAMETER_H

#include <vector>

/// A container for numbers passed from the command line.
namespace Vamos_Geometry
{
  class Parameter
  {
  public:
    /// Save the passed-in numbers
    static void set (const std::vector <double>& values)
    { m_values = values; }

    /// Get a parameter.
    /// @pamam i the index of the parameter
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
