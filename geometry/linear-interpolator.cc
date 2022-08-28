//  Copyright (C) 2003-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#include "linear-interpolator.h"
#include "numeric.h"

#include <cmath>
#include <cassert>

using namespace Vamos_Geometry;

Linear_Interpolator::Linear_Interpolator()
{
}

Linear_Interpolator::Linear_Interpolator(std::vector<Two_Vector> const& points)
    : Interpolator(points)
{
}

double Linear_Interpolator::interpolate(double x) const
{
    assert(!m_points.empty());
    Interpolator::interpolate(x);

    if (m_points.size() == 1)
        return m_points[0].y;
    // Clamp to endpoints.
    if (x <= m_points.front().x)
        return m_points.front().y;
    if (x >= m_points.back().x)
        return m_points.back().y;

    auto low{low_index(x)};
    return Vamos_Geometry::interpolate(x, m_points[low].x, m_points[low].y,
                                       m_points[low + 1].x, m_points [low + 1].y);
}

Two_Vector Linear_Interpolator::normal(double x) const
{
    if (m_points.size() == 1 || x <= m_points.front().x || x > m_points.back().x)
        return {0.0, 1.0};

    auto low{low_index(x)};
    return Two_Vector{m_points[low].y - m_points[low + 1].y,
                      m_points[low + 1].x - m_points[low].x}.unit();
}
