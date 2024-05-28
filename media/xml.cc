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

#include "xml.h"

#include "../geometry/conversions.h"

#include <sstream>

using namespace Vamos_Geometry;
using namespace Vamos_Media;

namespace Vamos_Media
{
bool get_flag(pugi::xml_node node, std::string const& tag)
{
    return node.child(tag.c_str());
}

std::vector<Point<double>> get_points(pugi::xml_node node,
                                      char const* tag,
                                      bool ordered,
                                      std::vector<Point<double>> points)
{
    std::istringstream is{node.child_value(tag)};
    while (is)
    {
        Point<double> p;
        is >> p;
        if (!is)
            break;
        // Overwrite a point at the same or greater x value.
        if (ordered && !points.empty() && p.x <= points.back().x)
            points.back() = p;
        else
            points.push_back(p);
    }
    return points;
}

Model get_model(pugi::xml_node node, std::string const& dir)
{
    return Model{dir + get_value(node, "file", std::string()),
                 get_value(node, "scale", 1.0),
                 get_value(node, "translate", null_v),
                 get_value(node, "rotate", null_v) * deg_to_rad(1.0)};
}
}
