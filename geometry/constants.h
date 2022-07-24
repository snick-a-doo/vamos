//  Copyright (C) 2003 Sam Varner
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

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <cmath>

namespace Vamos_Geometry
{
#ifdef M_PI
  const double pi = M_PI;
#else
  const double pi = 3.14159265358979323846;
#endif
  const double two_pi = 2.0 * pi;

#ifdef M_SQRT2
  const double root_2 = M_SQRT2;
#else
  const double root_2 = 1.41421356237309504880;
#endif

  const double inv_root_2 = 1.0 / root_2;

  enum Direction { NONE, IN, OUT, UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT };

  enum Axis { X, Y, Z };
}

#endif
