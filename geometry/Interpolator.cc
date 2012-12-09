//	Vamos Automotive Simulator
//  Copyright (C) 2009 Sam Varner
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

#include "Interpolator.h"

using namespace Vamos_Geometry;

// Set to true to print out cache information.
const bool show_statistics = false;
const int statistics_interval = 1000000;

Interpolator::Interpolator () 
  : m_cached_low_index (0),
    m_interpolations (0),
    m_cache_hits (0)
{
}

Interpolator::Interpolator (const std::vector <Two_Vector>& points) 
  : m_points (points),
    m_cached_low_index (0),
    m_interpolations (0),
    m_cache_hits (0)
{
}

Interpolator::~Interpolator () 
{
}

double
Interpolator::interpolate (double distance) const
{
  m_interpolations++;
  if (show_statistics && (m_interpolations % statistics_interval == 0))
    {
      std::cout << this << " calls: " << m_interpolations 
                << "  hits: " << m_cache_hits 
                << "  " << 100.0 * double (m_cache_hits) / m_interpolations << "%" 
                << std::endl;
    }

  return 0.0;
}

size_t
Interpolator::low_index (double dist) const
{
  const size_t low = m_cached_low_index;
  const size_t high = low + 1;

  if (dist > m_points [high].x)
    return get_new_low_index (dist, low, m_points.size () - 1);
  if (dist <= m_points [low].x)
    return get_new_low_index (dist, 0, low);

  // On same interval as last time.  No need to recalculate.
  m_cache_hits++;
  return m_cached_low_index;
}

size_t
Interpolator::get_new_low_index (double dist, size_t low, size_t high) const
{
  // Bisect to find the interval that dist is on.
  // distance (low) < dist <= distance (high)
  while ((high - low) > 1)
    {
      size_t index = size_t ((high + low) / 2.0);
      if (m_points [index].x > dist)
        high = index;
      else
        low = index;
    }
  m_cached_low_index = low;
  return low;
}
