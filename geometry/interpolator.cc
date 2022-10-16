//  Copyright (C) 2009-2022 Sam Warner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANT ABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#include "interpolator.h"

#include <iostream>
#include <numeric>

using namespace Vamos_Geometry;

// Set to true to print out cache information.
bool constexpr show_statistics = false;

Interpolator::Interpolator()
{
}

Interpolator::Interpolator(std::vector<Point<double>> const& points)
    : m_points{points}
{
}

Interpolator::~Interpolator()
{
    if (show_statistics)
    {
        std::cout << this << " points: " << m_points.size()
                  << "  checks: " << m_cache.checks
                  << "  hits: " << m_cache.hits;
        if (m_cache.checks > 0)
            std::cout << "  " << (100.0 * m_cache.hits) / m_cache.checks << "%";
        std::cout << std::endl;
    }
}

void Interpolator::load(Point<double> const& point)
{
    m_points.push_back(point);
    m_changed = true;
}

void Interpolator::load(std::vector<Point<double>> const& points)
{
    m_points.insert(m_points.end(), points.begin(), points.end());
    m_changed = true;
}

void Interpolator::clear()
{
    m_points.clear();
    m_cache.index = 0;
}

void Interpolator::replace(std::vector<Point<double>> const& points)
{
    clear();
    load(points);
}

void Interpolator::scale(double factor)
{
    for (auto& point : m_points)
        point.x *= factor;
    m_changed = true;
}

void Interpolator::shift(double delta)
{
    for (auto& point : m_points)
        point.y += delta;
    m_changed = true;
}

double Interpolator::interpolate(double) const
{
    m_changed = false;
    return 0.0;
}

size_t Interpolator::low_index(double x) const
{
    // Bisect to find the interval x is on.  x[low] < x <= x[high]
    auto get_low_index = [&](double x, size_t low, size_t high) {
        while (high - low > 1)
        {
            auto index{std::midpoint(low, high)};
            (m_points[index].x < x ? low : high) = index;
        }
        return low;
    };

    if (m_points.size() < 2)
        return 0;
    ++m_cache.checks;
    if (x >= m_points[m_cache.index + 1].x)
        m_cache.index = get_low_index(x, m_cache.index, m_points.size() - 1);
    else if (x < m_points[m_cache.index].x)
        m_cache.index = get_low_index(x, 0, m_cache.index);
    else
        ++m_cache.hits;
    return m_cache.index;
}
