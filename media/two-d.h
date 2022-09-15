//  Copyright (C) 2013-2022 Sam Varner
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

#ifndef VAMOS_MEDIA_TWO_D_H_INCLUDED
#define VAMOS_MEDIA_TWO_D_H_INCLUDED

#include <iomanip>
#include <sstream>
#include <string>

namespace Vamos_Geometry
{
class Rectangle;
class Two_Vector;
}
namespace Vamos_Media
{
/// RGB colors.
struct Color
{
    double red{0.0};
    double green{0.0};
    double blue{0.0};
};

/// Some common pre-defined colors
/// @{
Color constexpr red{1.0, 0.0, 0.0};
Color constexpr cyan{0.0, 1.0, 1.0};
Color constexpr magenta{1.0, 0.0, 1.0};
Color constexpr gray80{0.8, 0.8, 0.8};
/// @}

/// Draw 2D text. Positions are expressed as percentage of the viewport width and height.
class Two_D
{
    using V2 = Vamos_Geometry::Two_Vector;

public:
    /// Set up for 2D drawing.
    Two_D();
    /// Restore the graphics state.
    ~Two_D();

    /// Draw a string at screen position @p.
    void text(V2 p, std::string const& label);
    /// Draw a formatted value with label and units.
    template <typename T, typename U>
    void text(V2 const& p, const T& label, const U& value, const std::string& units = "",
              int precision = 0);
    /// Draw a vertical bar @p fraction of the way to the top of @p box.
    void bar(Vamos_Geometry::Rectangle const& box, const Color& color, double fraction);
    /// Draw @p n circles of radius @p r starting at at @p. The first @p n_an are
    /// drawn in @p on_color, the rest in off_color.
    void lights(V2 p, double r, int n, int n_on,
                Color const& on_color, Color const& off_color);
    /// Draw a dot displaced by @p v from the center of a ring of radius @p r.
    void vector(V2 p, double r, Color const& ring_color, Color const& dot_corol, V2 const& v);

private:
    double m_width; ///< The width of the viewport.
    double m_height; ///< The width of the viewport.
};

template <typename T, typename U>
void Two_D::text(V2 const& p, T const& label, U const& value, std::string const& units,
                 int precision)
{
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(precision) << label << ' ' << value << ' ' << units;
    text(p, os.str());
}

} // namespace Vamos_Media

#endif // VAMOS_MEDIA_TWO_D_H_INCLUDED
