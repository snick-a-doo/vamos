//  Copyright (C) 2005-2022 Sam Varner
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

#include "rectangle.h"

#include <algorithm>
#include <numeric>

using namespace Vamos_Geometry;

Rectangle::Rectangle()
{
}

Rectangle::Rectangle(double x, double y, double width, double height, bool inverted)
    : m_left(x),
      m_top(inverted ? y : y + height),
      m_right(x + width),
      m_bottom(inverted ? y + height : y)
{
}

Rectangle::Rectangle(const Two_Vector& upper_left, const Two_Vector& lower_right)
    : m_left(upper_left.x),
      m_top(upper_left.y),
      m_right(lower_right.x),
      m_bottom(lower_right.y)
{
}

Vamos_Geometry::Two_Vector Rectangle::center() const
{
    return {std::midpoint(m_left, m_right), std::midpoint(m_top, m_bottom)};
}

Rectangle& Rectangle::enclose(const Rectangle& other)
{
    auto inverted{m_bottom > m_top};
    m_left = std::min(m_left, other.m_left);
    m_top = inverted ? std::min(m_top, other.m_top) : std::max(m_top, other.m_top);
    m_right = std::max(m_right, other.m_right);
    m_bottom = inverted
        ? std::max(m_bottom, other.m_bottom)
        : std::min(m_bottom, other.m_bottom);
    return *this;
}

Rectangle& Rectangle::clip(const Rectangle& other)
{
    auto inverted{m_bottom > m_top};
    m_left = std::max(m_left, other.m_left);
    m_top = inverted ? std::max(m_top, other.m_top) : std::min(m_top, other.m_top);
    m_right = std::min(m_right, other.m_right);
    m_bottom = inverted
        ? std::min(m_bottom, other.m_bottom)
        : std::max(m_bottom, other.m_bottom);
    return *this;
}

Rectangle& Rectangle::scale(double x_factor, double y_factor)
{
    auto mid{center()};
    m_left = (m_left - mid.x) * x_factor + mid.x;
    m_top = (m_top - mid.y) * y_factor + mid.y;
    m_right = (m_right - mid.x) * x_factor + mid.x;
    m_bottom = (m_bottom - mid.y) * y_factor + mid.y;
    return *this;
}

Rectangle& Rectangle::scale(double factor)
{
    return scale(factor, factor);
}

Rectangle& Rectangle::move(const Two_Vector& delta)
{
    m_left += delta.x;
    m_top += delta.y;
    m_right += delta.x;
    m_bottom += delta.y;
    return *this;
}

namespace Vamos_Geometry
{
std::ostream& operator<<(std::ostream& os, Rectangle const& rect)
{
    return os << "(" << rect.left() << ", " << rect.top()
              << ") (" << rect.right() << ", " << rect.bottom() << ")";
}
}
