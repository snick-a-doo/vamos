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

#ifndef VAMOS_GEOMETRY_RECTANGLE_H_INCLUDED
#define VAMOS_GEOMETRY_RECTANGLE_H_INCLUDED

#include "../geometry/two-vector.h"

#include <iostream>
#include <numeric>

namespace Vamos_Geometry
{
template<typename T> class Rectangle
{
public:
    /// Construct a rectangle with zero size.
    Rectangle() {};
    /// Construct a rectangle from the top-left corner position and dimensions. If @p
    /// inverted is true, the bottom position is greater than the top.
    Rectangle(T x, T y, T width, T height, bool inverted = false);
    /// Construct a rectangle from the top-left and bottom-right coordinates.
    Rectangle(Vamos_Geometry::Point<T> const& upper_left,
              Vamos_Geometry::Point<T> const& lower_right);

    T left() const { return m_left; }
    T top() const { return m_top; }
    T right() const { return m_right; }
    T bottom() const { return m_bottom; }
    T width() const { return std::abs(m_right - m_left); }
    T height() const { return std::abs(m_top - m_bottom); }
    /// @return The position of the midpoint of the rectangle.
    Vamos_Geometry::Point<T> center() const
    {
        return {std::midpoint(m_left, m_right), std::midpoint(m_top, m_bottom)};
    };
    /// @return The aspect ratio.
    double aspect() const { return static_cast<double>(width()) / height(); }

    friend bool operator==(Rectangle const& r1, Rectangle const& r2) = default;

    /// Enlarge the rectangle if necessary so that another rectangle is completely
    /// enclosed.
    Rectangle<T>& enclose(const Rectangle& other);
    /// Shrink the rectangle if necessary so that it's completely enclosed by another.
    Rectangle<T>& clip(const Rectangle& other);

    /// Multiply width by @p x_factor and height by @p y_factor. Center is unchanged.
    Rectangle<T>& scale(double x_factor, double y_factor);
    /// Multiply width and height by @p factor.
    Rectangle<T>& scale(double factor) { return scale(factor, factor); }
    /// Translate the rectangle by the x and y components of @p delta.
    Rectangle<T>& move(Vamos_Geometry::Point<T> const& delta);

private:
    T m_left{0};
    T m_top{0};
    T m_right{0};
    T m_bottom{0};
};

template <typename T> Rectangle<T>::Rectangle(T x, T y, T width, T height, bool inverted)
    : m_left(x),
      m_top(inverted ? y : y + height),
      m_right(x + width),
      m_bottom(inverted ? y + height : y)
{
}

template <typename T>
Rectangle<T>::Rectangle(const Point<T>& upper_left, const Point<T>& lower_right)
    : m_left(upper_left.x),
      m_top(upper_left.y),
      m_right(lower_right.x),
      m_bottom(lower_right.y)
{
}

template <typename T> Rectangle<T>& Rectangle<T>::enclose(const Rectangle<T>& other)
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

template <typename T> Rectangle<T>& Rectangle<T>::clip(const Rectangle<T>& other)
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

template <typename T> Rectangle<T>& Rectangle<T>::scale(double x_factor, double y_factor)
{
    auto mid{center()};
    m_left = (m_left - mid.x) * x_factor + mid.x;
    m_top = (m_top - mid.y) * y_factor + mid.y;
    m_right = (m_right - mid.x) * x_factor + mid.x;
    m_bottom = (m_bottom - mid.y) * y_factor + mid.y;
    return *this;
}

template <typename T>Rectangle<T>& Rectangle<T>::move(const Point<T>& delta)
{
    m_left += delta.x;
    m_top += delta.y;
    m_right += delta.x;
    m_bottom += delta.y;
    return *this;
}

template <typename T>
std::ostream& operator << (std::ostream& os, Vamos_Geometry::Rectangle<T> const& rect)
{
    return os << "(" << rect.left() << ", " << rect.top()
              << ") (" << rect.right() << ", " << rect.bottom() << ")";
}

} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_RECTANGLE_H_INCLUDED
