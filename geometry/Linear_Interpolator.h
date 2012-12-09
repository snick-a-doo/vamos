// Linear_Interpolator.h - a piecewise-linear interpolator.
//
//	Vamos Automotive Simulator
//  Copyright (C) 2003 Sam Varner
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

#ifndef _LINEAR_INTERPOLATOR_H_
#define _LINEAR_INTERPOLATOR_H_

#include "Interpolator.h"
#include "Two_Vector.h"

#include <vector>

namespace Vamos_Geometry
{
  class Linear_Interpolator : public Interpolator
  {
  public:
	// Construct an empty curve.
	Linear_Interpolator ();

	// Construct a cuvre from an array of points.
	Linear_Interpolator (const std::vector <Two_Vector>& points);

	// Add a point to the curve.
	virtual void load (const Two_Vector& point);
	virtual void load (double x, double y) { load (Two_Vector (x, y)); }

	// Add multiple points to the curve.
	virtual void load (const std::vector <Two_Vector>& points);

	// Replace all points on the curve.
	virtual void replace (const std::vector <Two_Vector>& points)
    {
      clear ();
      load (points);
    }

	// Remove all points from the curve.
	virtual void clear ();

	// Remove points with x > LIMIT.
	virtual void remove_greater (double limit);

	// Scale all of the x values by FACTOR.
	virtual void scale (double factor);

	// Return the y value at the x value DIST
	virtual double interpolate (double dist) const;

	// Return the normal to the tanget at DIST.
	virtual Two_Vector normal (double dist) const;

    // Add 'delta' to all points.
    virtual void shift (double delta);

    // Return the number of control points.
    virtual size_t size () const { return m_points.size (); }

    virtual Two_Vector& operator [] (size_t i) { return m_points [i]; }
    virtual const Two_Vector& operator [] (size_t i) const { return m_points [i]; }
  };
}

#endif
