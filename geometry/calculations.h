//  Calculations.h - useful geometry functions.
//
//  Copyright 2009 Sam Varner
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

#ifndef _CALCULATIONS_H_
#define _CALCULATIONS_H_

namespace Vamos_Geometry
{
  struct Three_Vector;

  /// Return the minimum distance (in the past or future) between two
  /// particles. 
  /// \param r1 Position of particle 1.
  /// \palam v1 Velocity of particle 1.
  /// \param r2 Position of particle 2.
  /// \palam v2 Velocity of particle 2.
  double closest_approach (const Three_Vector& r1, const Three_Vector& v1,
                           const Three_Vector& r2, const Three_Vector& v2);

  /// Return a particle 2's speed relative to particle 1.
  /// \param r1 Position of particle 1.
  /// \palam v1 Velocity of particle 1.
  /// \param r2 Position of particle 2.
  /// \palam v2 Velocity of particle 2.
  double closing_speed (const Three_Vector& r1, const Three_Vector& v1,
                        const Three_Vector& r2, const Three_Vector& v2);

}

#endif // not _CALCULATIONS_H_
