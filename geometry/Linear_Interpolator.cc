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

#include "Linear_Interpolator.h"
#include "Numeric.h"

#include <cmath>
#include <cassert>

Vamos_Geometry::
Linear_Interpolator::Linear_Interpolator () 
{
}

Vamos_Geometry::
Linear_Interpolator::
Linear_Interpolator (const std::vector <Two_Vector>& points) 
  : Interpolator (points)
{
  m_points = points;
}

void Vamos_Geometry::
Linear_Interpolator::load (const Two_Vector& point)
{
  m_points.push_back (point);
}

void Vamos_Geometry::
Linear_Interpolator::load (const std::vector <Two_Vector>& points)
{
  for (std::vector <Two_Vector>::const_iterator it = points.begin ();
	   it != points.end ();
	   it++)
	{
	  m_points.push_back (*it);
	}
}

void Vamos_Geometry::
Linear_Interpolator::clear ()
{
  m_points.clear ();
  clear_cache ();
}

// Remove points with x > LIMIT.
void Vamos_Geometry::
Linear_Interpolator::remove_greater (double limit)
{
  clear_cache ();
  size_t size = 0;
  for (std::vector <Two_Vector>::const_iterator it = m_points.begin ();
	   it != m_points.end ();
	   it++)
	{
	  if (it->x > limit)
		{
		  m_points.resize (size);
		  break;
		}
	  size++;
	}
}

void Vamos_Geometry::
Linear_Interpolator::scale (double factor)
{
  for (std::vector <Two_Vector>::iterator it = m_points.begin ();
	   it != m_points.end ();
	   it++)
	{
	  it->x *= factor;
	}
}

double Vamos_Geometry::
Linear_Interpolator::interpolate (double dist) const
{
  Interpolator::interpolate (dist);

  assert (m_points.size () > 0);

  if (m_points.size () == 1)
	return m_points [0].y;

  if (dist < m_points.begin ()->x)
    return m_points.begin ()->y;
  if (dist > (m_points.end () - 1)->x)
    return (m_points.end () - 1)->y;

  const size_t low = low_index (dist);
  const size_t high = low + 1;

  return Vamos_Geometry::interpolate (dist, 
                                      m_points [low].x, m_points [low].y, 
                                      m_points [high].x, m_points [high].y);
}

Vamos_Geometry::Two_Vector Vamos_Geometry::
Linear_Interpolator::normal (double dist) const
{
  if ((m_points.size () == 1)
      || (dist < m_points.begin ()->x)
      || (dist > (m_points.end () - 1)->x))
	return Two_Vector (0.0, 1.0);

  const size_t low = low_index (dist);
  const size_t high = low + 1;

  return Two_Vector (m_points [low].y - m_points [high].y, 
                     m_points [high].x - m_points [low].x).unit ();
}

// Add 'delta' to all points.
void Vamos_Geometry::
Linear_Interpolator::shift (double delta)
{
  for (std::vector <Two_Vector>::iterator it = m_points.begin ();
	   it != m_points.end ();
	   it++)
	{
	  it->y += delta;
	}
}
