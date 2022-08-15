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

#ifndef VAMOS_GEOMETRY_CONTACT_INFO_H_INCLUDED
#define VAMOS_GEOMETRY_CONTACT_INFO_H_INCLUDED

#include "material.h"
#include "three-vector.h"

namespace Vamos_Geometry
{
/// Information about a collision between a point and a surface.
struct Contact_Info
{
    /// True if a collision occurred, false otherwise.
    bool contact{false};
    /// How far the point had penetrated the surface when the collision was detected.
    double depth{0.0};
    /// The vector normal to the surface.
    Three_Vector normal;
    /// The material of the surface.
	Material material;
};
}

#endif // VAMOS_GEOMETRY_CONTACT_INFO_H_INCLUDED
