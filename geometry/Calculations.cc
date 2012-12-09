//  Calculations.cc - useful geometry functions.
//
//  Copyright 2009 Sam Varner
//
//	This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 3 of the License, or
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

#include "Calculations.h"
#include "Three_Vector.h"

using namespace Vamos_Geometry;

double 
Vamos_Geometry::closest_approach (const Three_Vector& r1, const Three_Vector& v1,
                                  const Three_Vector& r2, const Three_Vector& v2)
{
  const Three_Vector delta_r = r2 - r1;
  return delta_r.magnitude () * sin (delta_r.angle (v2 - v1));
}

double
Vamos_Geometry::closing_speed (const Three_Vector& r1, const Three_Vector& v1,
                               const Three_Vector& r2, const Three_Vector& v2)
{
  const Three_Vector delta_r = r2 - r1;
  return v1.component (delta_r) - v2.component (delta_r);
}

