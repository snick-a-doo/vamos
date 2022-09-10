//  Copyright (C) 2001-2022 Sam Varner
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
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#include "frame.h"

#include "../geometry/conversions.h"

#include <cassert>
#include <cmath>

using namespace Vamos_Body;
using namespace Vamos_Geometry;

Frame::Frame(Three_Vector const& position, Three_Matrix const& orientation, Frame const* parent)
    : mp_parent(parent),
      m_orientation(orientation),
      m_position(position)
{
}

Frame::Frame(Three_Vector const& position, Frame const* parent)
    : mp_parent(parent),
      m_position(position)
{
}

Frame::Frame(Frame const* parent)
    : mp_parent(parent)
{
}

Three_Vector Frame::transform_from_parent(Three_Vector const& r) const
{
    return rotate_from_parent(r - m_position);
}

Three_Vector Frame::transform_velocity_from_parent(Three_Vector const& v) const
{
    return rotate_from_parent(v - m_velocity);
}

Three_Vector Frame::transform_from_world(Three_Vector const& r) const
{
    auto in{transform_from_parent(r)};
    return mp_parent ? mp_parent->transform_from_world(in) : in;
}

Three_Vector Frame::transform_velocity_from_world(Three_Vector const& v) const
{
    auto in{transform_velocity_from_parent(v)};
    return mp_parent ? mp_parent->transform_velocity_from_world(in) : in;
}

Three_Vector Frame::transform_to_parent(Three_Vector const& r) const
{
    return rotate_to_parent(r) + m_position;
}

Three_Vector Frame::transform_velocity_to_parent(Three_Vector const& v) const
{
    return rotate_to_parent(v) + m_velocity;
}

Three_Vector Frame::transform_to_world(Three_Vector const& r) const
{
    auto out{transform_to_parent(r)};
    return mp_parent ? mp_parent->transform_to_world(out) : out;
}

Three_Vector Frame::transform_velocity_to_world(Three_Vector const& v) const
{
    auto out{transform_velocity_to_parent(v)};
    return mp_parent ? mp_parent->transform_velocity_to_world(out) : out;
}

Three_Vector Frame::rotate_to_parent(Three_Vector const& vector) const
{
    return m_orientation * vector;
}

Three_Vector Frame::rotate_to_world(Three_Vector const& vector) const
{
    auto out{rotate_to_parent(vector)};
    return mp_parent ? mp_parent->rotate_to_world(out) : out;
}

Three_Vector Frame::rotate_from_parent(Three_Vector const& vector) const
{
    return transpose(m_orientation) * vector;
}

Three_Vector Frame::rotate_from_world(Three_Vector const& vector) const
{
    auto in{rotate_from_parent(vector)};
    return mp_parent ? mp_parent->rotate_from_world(in) : in;
}

Three_Vector Frame::axis_angle(double& angle) const
{
    // To convert the rotation matrix representation of the body's orientation
    // to an axis-angle orientation, we transform first to a quaternion
    // representation.  The matrix-to-quaternion and quaternion-to-axis-angle
    // transformations are described in the Matrix and Quaternion FAQ
    // (matrixfaq.htm) in the doc directory.

    // Make a local reference to the tranformation matrix for brevity.
    const auto& omat{m_orientation};

    // Convert from matrix to quaternion
    auto trace{omat[0][0] + omat[1][1] + omat[2][2] + 1.0};
    double s, w, x, y, z;
    s = w = x = y = z = 0.0;
    if (trace > 0.0)
    {
        s = 0.5 / sqrt(trace);
        w = 0.25 / s;
        x = (omat[2][1] - omat[1][2]) * s;
        y = (omat[0][2] - omat[2][0]) * s;
        z = (omat[1][0] - omat[0][1]) * s;
    }
    else
    {
        // Find the largest diagonal element and do the appropriate transformation.
        auto largest = omat[0][0];
        auto index{0};
        if (omat[1][1] > largest)
        {
            largest = omat[1][1];
            index = 1;
        }

        if (omat[2][2] > largest)
        {
            largest = omat[2][2];
            s = sqrt(1.0 - omat[0][0] - omat[1][1] + omat[2][2]) * 2.0;
            w = (omat[0][1] + omat[1][0]) / s;
            x = (omat[0][2] + omat[2][0]) / s;
            y = (omat[1][2] + omat[2][1]) / s;
            z = 0.5 / s;
        }
        else if (index == 0)
        {
            s = sqrt(1.0 + omat[0][0] - omat[1][1] - omat[2][2]) * 2.0;
            w = (omat[1][2] + omat[2][1]) / s;
            x = 0.5 / s;
            y = (omat[0][1] + omat[1][0]) / s;
            z = (omat[0][2] + omat[2][0]) / s;
        }
        else
        {
            assert(index == 1);
            s = sqrt(1.0 - omat[0][0] + omat[1][1] - omat[2][2]) * 2.0;
            w = (omat[0][2] + omat[2][0]) / s;
            x = (omat[0][1] + omat[1][0]) / s;
            y = 0.5 / s;
            z = (omat[1][2] + omat[2][1]) / s;
        }
    }

    // Convert from quaternion to angle-axis.
    angle = Vamos_Geometry::rad_to_deg(acos(w) * 2.0);
    // The return value would be divided by sin (angle) to give a unit vector, but
    // glRotate*() doesn't care about the length so we'll leave it as is.
    return {x, y, z};
}
