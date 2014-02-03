// Spline.cc - a cubic spline interpolator.
//
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

#include "Spline.h"
#include "Numeric.h"

#include <cmath>
#include <cassert>

using namespace Vamos_Geometry;

Spline::Spline ()
  : m_first_slope_is_set (false),
    m_last_slope_is_set (false),
    m_calculated (false),
    m_slope (0.0),
    m_periodic (false)
{
}

// Construct an empty curve.
Spline::Spline (double first_slope, double last_slope) 
  : m_first_slope (first_slope),
    m_first_slope_is_set (true),
    m_last_slope (last_slope),
    m_last_slope_is_set (true),
    m_calculated (false),
    m_slope (0.0),
    m_periodic (false)
{
}

// Construct a cuvre from an array of points.
Spline::Spline (const std::vector <Two_Vector>& points)
  : Interpolator (points),
    m_first_slope_is_set (false),
    m_last_slope_is_set (false),
    m_calculated (false),
    m_slope (0.0),
    m_periodic (false)
{
}

// Construct a cuvre from an array of points.
Spline::Spline (const std::vector <Two_Vector>& points,
                double first_slope, double last_slope) 
  : Interpolator (points),
    m_first_slope (first_slope),
    m_first_slope_is_set (true),
    m_last_slope (last_slope),
    m_last_slope_is_set (true),
    m_calculated (false),
    m_slope (0.0),
    m_periodic (false)
{
}

// Add a point to the curve.
void 
Spline::load (const Two_Vector& point)
{
  m_points.push_back (point);
  m_calculated = false;
}

// Add multiple points to the curve.
void 
Spline::load (const std::vector <Two_Vector>& points)
{
  for (std::vector <Two_Vector>::const_iterator it = points.begin ();
       it != points.end ();
       it++)
    {
      m_points.push_back (*it);
    }
  m_calculated = false;
}

// Remove all points from the curve.
void 
Spline::clear ()
{
  m_points.clear ();
  clear_cache ();
  m_calculated = false;
}

void
Spline::set_periodic (double end)
{
  load (end, (m_points.size () == 0) ? 0.0 : m_points [0].y);
  m_periodic = true;
}

// Remove points with x > LIMIT.
void 
Spline::remove_greater (double limit)
{
  clear_cache ();
  for (size_t size = 0; size < m_points.size (); size++)
    {
      if (m_points [size].x > limit)
        {
          m_points.resize (size);
          m_calculated = false;
          return;
        }
    }
}

// Scale all of the x values by FACTOR.
void 
Spline::scale (double factor)
{
  for (std::vector <Two_Vector>::iterator it = m_points.begin ();
       it != m_points.end ();
       it++)
    {
      it->x *= factor;
    }

  m_calculated = false;
}

// calculate() and interpolate() follow the discussion on cubic
// splines found in Numerical Recipes.  The implementation here is
// original. 

// Return the y value at the x value DISTANCE
double 
Spline::interpolate (double distance) const
{
  Interpolator::interpolate (distance);

  const size_t n = m_points.size ();

  if ((n < 2) || ((n == 2) && m_periodic))
    {
      m_slope = (m_periodic || !m_first_slope_is_set) ? 0.0 : m_first_slope;
      m_second_derivative = 0.0;
      Two_Vector p0 = (m_points.empty ()) ? Two_Vector (0.0, 0.0) : m_points [0];
      return p0.y + m_slope * (distance - p0.x);
    }
  if ((n == 2) && (!m_first_slope_is_set || !m_last_slope_is_set))
    {
      // Fall back to linear interpolation.
      m_slope = (m_points [1].y - m_points [0].y)
        / (m_points [1].x - m_points [0].x);
      return Vamos_Geometry::interpolate (distance, 
                                          m_points [0].x, m_points [0].y,
                                          m_points [1].x, m_points [1].y);
    }

  if (m_periodic)
    distance = wrap (distance, m_points [0].x, m_points [n-1].x);

  // calculate() only needs to be called once for a given set of
  // points.
  if (!m_calculated)
    calculate ();

  const size_t low = low_index (distance);
  const size_t high = low + 1;
  const double diff = m_points [high].x - m_points [low].x;

  // Evaluate the coefficients for the cubic spline equation.
  const double a = (m_points [high].x - distance) / diff;
  const double b = 1.0 - a;
  const double sq = diff*diff / 6.0;
  const double a2 = a*a;
  const double b2 = b*b;

  // Find the first derivative.
  m_slope =
    (m_points [high].y - m_points [low].y)/diff
    - (3.0 * a2 - 1.0) / 6.0 * diff * m_second_deriv [low]
    + (3.0 * b2 - 1.0) / 6.0 * diff * m_second_deriv [high];

  m_second_derivative = 
    Vamos_Geometry::interpolate (distance, 
                                 m_points [low].x, m_second_deriv [low],
                                 m_points [high].x, m_second_deriv [high]);

  // Return the interpolated value.
  return a * m_points [low].y 
    + b * m_points [high].y 
    + a * (a2 - 1.0) * sq * m_second_deriv [low] 
    + b * (b2 - 1.0) * sq * m_second_deriv [high];
}

double 
Spline::slope (double distance) const
{
  // The slope is calculated and stored when interpolate() is called.
  interpolate (distance);
  return m_slope;
}

double
Spline::second_derivative (double distance) const
{
  // The slope is calculated and stored when interpolate() is called.
  interpolate (distance);
  return m_second_derivative;
}

void
solve_symmetric_tridiagonal (const double* a, 
                             const double* b_in, 
                             const double* r_in, 
                             double* x, 
                             size_t n)
{
  double* b = new double [n];
  double* r = new double [n];

  b [0] = b_in [0];
  r [0] = r_in [0];

  // Gauss-Jordan Elimination
  for (size_t i = 1; i < n; i++)
    {
      // Replace row i with row i - k * row (i-1) such that A_{i,i-1} = 0.0.
      double factor = a [i-1] / b [i-1];
      // A_{i,i-1} is not used again, so it need not be calculated.
      b [i] = b_in [i] - factor * a [i-1];
      // A_{i,i+1} is unchanged because A_{i-1,i+1} = 0.0.
      r [i] = r_in [i] - factor * r [i-1];
    }
  
  // Back-Substitution
  x [n-1] = r [n-1] / b [n-1];
  for (int i = n - 2; i >= 0; i--)
    {
      // Use the solution for x[i+1] to find x[i].
      x [i] = (r [i] - a [i] * x [i+1]) / b [i];
    }

  delete [] r;
  delete [] b;
}

// Calculate the coefficients for interpolation.
void 
Spline::calculate () const
{
  m_calculated = true;

  size_t n = m_points.size ();
  if (n < 2)
    return;

  m_second_deriv.resize (n);

  if ((n == 2) && m_first_slope_is_set && m_last_slope_is_set)
    {
      Two_Vector delta = m_points [1] - m_points [0];
      double m3 = 3.0 * delta.y / delta.x;
      double a = delta.x / 2.0;
      m_second_deriv [0] = -(2.0 * m_first_slope + m_last_slope - m3) / a;
      m_second_deriv [1] = (m_first_slope + 2.0 * m_last_slope - m3) / a;
      return;
    }

  double* a = new double [n-1];
  double* b = new double [n-1];
  double* r = new double [n-1];
  double* x = new double [n-1];

  for (size_t i = 0; i < n - 2; i++)
    {
      double diff_low = m_points [i+1].x - m_points [i].x;
      double diff_high = m_points [i+2].x - m_points [i+1].x;

      a [i] = diff_high / 6.0;
      b [i] = (diff_low + diff_high) / 3.0;
      r [i] = (m_points [i+2].y - m_points [i+1].y) / diff_high
        - (m_points [i+1].y - m_points [i].y) / diff_low;
    }

  if (m_periodic)
    {
      double diff_low = m_points [n-1].x - m_points [n-2].x;
      double diff_high = m_points [1].x - m_points [0].x;

      a [n-2] = diff_high / 6.0;
      b [n-2] = (diff_low + diff_high) / 3.0;
      r [n-2] = (m_points [1].y - m_points [0].y) / diff_high
        - (m_points [n-1].y - m_points [n-2].y) / diff_low;
      
      const double alpha = a [n-2];
      const double gamma = -b [0];
      b [0] -= gamma;
      b [n-2] -= alpha * alpha / gamma;

      solve_symmetric_tridiagonal (a, b, r, x, n-1);

      double* u = new double [n-1];
      u [0] = gamma;
      for (size_t i = 1; i < n-2; i++)
        u [i] = 0.0;
      u [n-2] = alpha;

      double* z = new double [n-1];
      solve_symmetric_tridiagonal (a, b, u, z, n-1);

      const double factor = (x [0] + x [n-2] * alpha / gamma)
        / (1.0 + z [0] + z [n-2] * alpha / gamma);

      for (size_t i = 1; i < n; i++)
        m_second_deriv [i] = x [i-1] - factor * z [i-1];
      m_second_deriv [0] = m_second_deriv [n-1];

      delete [] z;
      delete [] u;
    }
  else
    {
      solve_symmetric_tridiagonal (a, b, r, x, n - 2);

      for (size_t i = 1; i < n-1; i++)
        m_second_deriv [i] = x [i-1];
      m_second_deriv [0] = 0.0;
      m_second_deriv [n-1] = 0.0;

      if (m_first_slope_is_set)
        {
          const double dy = m_points [1].y - m_points [0].y;
          const double dx = m_points [1].x - m_points [0].x;
          m_second_deriv [0] = 3.0/dx 
            * (dy/dx - m_first_slope - dx/6.0 * m_second_deriv [1]);
        }
      if (m_last_slope_is_set)
        {
          const double dy = m_points [n-1].y - m_points [n-2].y;
          const double dx = m_points [n-1].x - m_points [n-2].x;
          m_second_deriv [n-1] = -3.0/dx 
            * (dy/dx - m_last_slope + dx/6.0 * m_second_deriv [n-2]);
        }
    }

  delete [] x;
  delete [] r;
  delete [] b;
  delete [] a;
}

// Return the normal to the tanget at DISTANCE.
Two_Vector 
Spline::normal (double distance) const
{
  interpolate (distance);
  double theta = std::atan (m_slope);
  return Two_Vector (-std::sin (theta), std::cos (theta));
}

// Add 'delta' to all points.
void
Spline::shift (double delta)
{
  for (std::vector <Two_Vector>::iterator it = m_points.begin ();
	   it != m_points.end ();
	   it++)
	{
	  it->y += delta;
	}
}

//-----------------------------------------------------------------------------
Parametric_Spline::Parametric_Spline ()
{
}

Parametric_Spline::Parametric_Spline (double first_x_slope, double last_x_slope,
                                      double first_y_slope, double last_y_slope)
  : m_x (first_x_slope, last_x_slope),
    m_y (first_y_slope, last_y_slope)
{
}

void
Parametric_Spline::load (double parameter, const Two_Vector& point)
{
  m_x.load (Two_Vector (parameter, point.x));
  m_y.load (Two_Vector (parameter, point.y));
}

void
Parametric_Spline::clear ()
{
  m_x.clear ();
  m_y.clear ();
}

void
Parametric_Spline::set_periodic (double end)
{
  m_x.set_periodic (end);
  m_y.set_periodic (end);
}

Two_Vector
Parametric_Spline::interpolate (double parameter) const
{
  return Two_Vector (m_x.interpolate (parameter),
                     m_y.interpolate (parameter));
}

size_t
Parametric_Spline::size () const
{
  assert (m_x.size () == m_y.size ());
  return m_x.size ();
}

Two_Vector 
Parametric_Spline::operator [] (size_t i) const
{
  return Two_Vector (m_x [i].y, m_y [i].y);
}

double
Parametric_Spline::parameter (size_t i) const
{
  return m_x [i].x;
}

//-----------------------------------------------------------------------------
Vector_Spline::Vector_Spline ()
{
}

Vector_Spline::Vector_Spline (double first_x_slope, double last_x_slope,
                              double first_y_slope, double last_y_slope,
                              double first_z_slope, double last_z_slope)
  : m_x (first_x_slope, last_x_slope),
    m_y (first_y_slope, last_y_slope),
    m_z (first_z_slope, last_z_slope)
{
}

void
Vector_Spline::load (double parameter, const Three_Vector& point)
{
  m_x.load (Two_Vector (parameter, point.x));
  m_y.load (Two_Vector (parameter, point.y));
  m_z.load (Two_Vector (parameter, point.z));
}

void
Vector_Spline::clear ()
{
  m_x.clear ();
  m_y.clear ();
  m_z.clear ();
}

void
Vector_Spline::set_periodic (double end)
{
  m_x.set_periodic (end);
  m_y.set_periodic (end);
  m_z.set_periodic (end);
}

Three_Vector
Vector_Spline::interpolate (double parameter) const
{
  return Three_Vector (m_x.interpolate (parameter),
                       m_y.interpolate (parameter),
                       m_z.interpolate (parameter));
}

size_t
Vector_Spline::size () const
{
  assert (m_x.size () == m_y.size ());
  assert (m_x.size () == m_z.size ());
  return m_x.size ();
}

Three_Vector 
Vector_Spline::operator [] (size_t i) const
{
  return Three_Vector (m_x [i].y, m_y [i].y, m_z [i].y);
}

double
Vector_Spline::parameter (size_t i) const
{
  return m_x [i].x;
}
