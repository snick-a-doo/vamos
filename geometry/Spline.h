//  Copyright (C) 2001--2004 Sam Varner
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

#ifndef _SPLINE_H_
#define _SPLINE_H_

#include "Interpolator.h"
#include "Three_Vector.h"
#include "Two_Vector.h"

#include <vector>

namespace Vamos_Geometry
{
  class Spline : public Interpolator
  {
  public:
    // Construct an empty curve.
    Spline ();
    Spline (double first_slope, double last_slope);

    // Construct a cuvre from an array of points.
    Spline (const std::vector <Two_Vector>& points);
    Spline (const std::vector <Two_Vector>& points,
            double first_slope, 
            double last_slope);

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

    void set_periodic (double period_end);

    // Remove points with x > LIMIT.
    virtual void remove_greater (double limit);

    // Scale all of the x values by FACTOR.
    virtual void scale (double factor);

    // Return the y value at the x value DIST
    virtual double interpolate (double dist) const;

    double slope (double distance) const;

    double second_derivative (double distance) const;

    // Return the normal to the tanget at DIST.
    virtual Two_Vector normal (double dist) const;

    // Add 'delta' to all points.
    virtual void shift (double delta);

    // Return the number of control points.
    virtual size_t size () const { return m_points.size (); }

    virtual Two_Vector& operator [] (size_t i) { return m_points [i]; }
    virtual const Two_Vector& operator [] (size_t i) const { return m_points [i]; }

  private:
    // The array of calculated second derivatives.
    mutable std::vector <double> m_second_deriv;

    // The first derivative of the spline at the first point.
    double m_first_slope;
    bool m_first_slope_is_set;

    // The first derivative of the spline at the last point.
    double m_last_slope;
    bool m_last_slope_is_set;

    // True if the second derivatives have been calculated.
    mutable bool m_calculated;

    // The 1st and 2nd derivatives at the interpolated point
    // calculated during the last call to interpolate().
    mutable double m_slope;
    mutable double m_second_derivative;

    // Calculate the coefficients for interpolation.
    void calculate () const;

    // The segment index from the previous interpolation.
    mutable size_t m_last_index;

    double m_period_end;
    bool m_periodic;

    void check (double* a, double* b, double* r) const;
  };

  class Parametric_Spline
  {
  public:
    Parametric_Spline ();

    Parametric_Spline (double first_x_slope, double last_x_slope,
                       double first_y_slope, double last_y_slope);

	// Add a point to the curve.
	void load (double parameter, const Two_Vector& point);
	void load (double parameter, double x, double y) 
    { load (parameter, Two_Vector (x, y)); }

	// Remove all points from the curve.
	void clear ();

    void set_periodic (double period_end);

	// Return the point at PARAMETER.
	Two_Vector interpolate (double parameter) const;

    // Return the number of control points.
    size_t size () const;

    Two_Vector operator [] (size_t i) const;

    double parameter (size_t i) const;

  private:
    Spline m_x;
    Spline m_y;
  };

  class Vector_Spline
  {
  public:
    Vector_Spline ();
    Vector_Spline (double first_x_slope, double last_x_slope,
                   double first_y_slope, double last_y_slope,
                   double first_z_slope, double last_z_slope);

	// Add a point to the curve.
	void load (double parameter, const Vamos_Geometry::Three_Vector& point);
	void load (double parameter, double x, double y, double z) 
    { load (parameter, Three_Vector (x, y, z)); }

	// Remove all points from the curve.
	void clear ();

    void set_periodic (double period_end);

	// Return the point at PARAMETER.
    Vamos_Geometry::Three_Vector interpolate (double parameter) const;

    // Return the number of control points.
    size_t size () const;

    Vamos_Geometry::Three_Vector operator [] (size_t i) const;

    double parameter (size_t i) const;

  private:
    Spline m_x;
    Spline m_y;
    Spline m_z;
  };
}

#endif
