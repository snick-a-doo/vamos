//  Copyright (C) 2001--2002 Sam Varner
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

#ifndef _INTERPOLATOR_H_
#define _INTERPOLATOR_H_

#include "Two_Vector.h"
#include <vector>

namespace Vamos_Geometry
{
  class Interpolator
  {
  public:
    Interpolator ();
    Interpolator (const std::vector <Two_Vector>& points);
	virtual ~Interpolator ();

	// Add a point to the curve.
	virtual void load (const Two_Vector& point) = 0;
	virtual void load (double x, double y) { load (Two_Vector (x, y)); }

	// Add multiple points to the curve.
	virtual void load (const std::vector <Two_Vector>& points) = 0;

	// Replace all points on the curve.
	virtual void replace (const std::vector <Two_Vector>& points) = 0;

	// Remove all points from the curve.
	virtual void clear () = 0;

	// Remove points with x > LIMIT.
	virtual void remove_greater (double limit) = 0;

	// Scale all of the x values by FACTOR.
	virtual void scale (double factor) = 0;

	// Return the y value at the x value DIST
	virtual double interpolate (double dist) const;

	// Return the normal to the tanget at DIST.
	virtual Two_Vector normal (double dist) const = 0;

    // Add 'delta' to all points.
    virtual void shift (double delta) = 0;

    // Return the number of control points.
    virtual size_t size () const = 0;

    virtual Two_Vector& operator [] (size_t i) = 0;
    virtual const Two_Vector& operator [] (size_t i) const = 0;

  protected:
    size_t low_index (double dist) const;
    size_t cached_low_index () const { return m_cached_low_index; }
    void clear_cache () { m_cached_low_index = 0; }

    // The array of points to interpolate between.
    std::vector <Two_Vector> m_points;

  private:
    size_t get_new_low_index (double dist, size_t low, size_t high) const;

    mutable size_t m_cached_low_index;

    // for gathering cache statistics
    mutable int m_interpolations;
    mutable int m_cache_hits;
  };
}

#endif
