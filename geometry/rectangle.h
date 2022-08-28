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

#include <iosfwd>

namespace Vamos_Geometry
{
class Rectangle
{
public:
    /// Construct a rectangle with zero size.
    Rectangle();
    /// Construct a rectangle from the top-left corner position and dimensions. If @p
    /// inverted is true, the bottom position is greater than the top.
    Rectangle(double x, double y, double width, double height, bool inverted = false);
    /// Construct a rectangle from the top-left and bottom-right coordinates.
    Rectangle(Vamos_Geometry::Two_Vector const& upper_left,
              Vamos_Geometry::Two_Vector const& lower_right);

    double left() const { return m_left; }
    double top() const { return m_top; }
    double right() const { return m_right; }
    double bottom() const { return m_bottom; }
    double width() const;
    double height() const;
    /// @return The position of the midpoint of the rectangle.
    Vamos_Geometry::Two_Vector center() const;
    /// @return The aspect ratio.
    double aspect() const;

    friend bool operator==(Rectangle const& r1, Rectangle const& r2) = default;

    /// Enlarge the rectangle if necessary so that another rectangle is completely
    /// enclosed.
    Rectangle& enclose(const Rectangle& other);
    /// Shrink the rectangle if necessary so that it's completely enclosed by another.
    Rectangle& clip(const Rectangle& other);

    /// Multiply width by @p x_factor and height by @p y_factor. Center is unchanged.
    Rectangle& scale(double x_factor, double y_factor);
    /// Multiply width and height by @p factor.
    Rectangle& scale(double factor);
    /// Translate the rectangle by the x and y components of @p delta.
    Rectangle& move(Vamos_Geometry::Two_Vector const& delta);

private:
    double m_left{0.0};
    double m_top{0.0};
    double m_right{0.0};
    double m_bottom{0.0};
};

std::ostream& operator << (std::ostream& os, Vamos_Geometry::Rectangle const& rect);
} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_RECTANGLE_H_INCLUDED
