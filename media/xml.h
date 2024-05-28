//  Copyright (C) 2022 Sam Varner
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

#ifndef VAMOS_MEDIA_XML_H_INCLUDED
#define VAMOS_MEDIA_XML_H_INCLUDED

#include "model.h"

#include "../geometry/two-vector.h"

#include <pugixml.hpp>

#include <array>
#include <sstream>
#include <string>
#include <vector>

namespace Vamos_Media
{
/// @return The value in the given tag under the given node. If no such tag, @p fallback
/// is returned.
template <typename T> T get_value(pugi::xml_node node, std::string const& tag, T fallback)
{
    if (auto child{node.child(tag.c_str())})
    {
        std::istringstream is{child.text().as_string()};
        is >> fallback;
    }
    return fallback;
}

/// @return True if @p tag is present.
bool get_flag(pugi::xml_node node, std::string const& tag);

/// @return A std::array of doubles.
template <size_t N> std::array<double, N> get_array(pugi::xml_node node,
                                                    std::string const& tag,
                                                    std::array<double, N>& fallback)
{
    if (auto child{node.child(tag.c_str())})
    {
        std::istringstream is{child.text().as_string()};
        char delim;
        // Each element is assumed to be preceded by an unspecified delimiter. E.g. [1.1,
        // 2.2, 3.3].
        for (size_t i{0}; i < N; ++i)
            is >> delim >> fallback[i];
    }
    return fallback;
}

/// Get a vector of points from a single XML tag.
/// @param ordered If true, new points with x <= previous overwrite the previous point.
std::vector<Vamos_Geometry::Point<double>> get_points(
    pugi::xml_node node,
    char const* tag,
    bool ordered,
    std::vector<Vamos_Geometry::Point<double>> points = {});

/// Read the model tags and return an Ac3d object.
Model get_model(pugi::xml_node node, std::string const& dir);
} // namespace Vamos_Media

#endif // VAMOS_MEDIA_XML_H_INCLUDED
