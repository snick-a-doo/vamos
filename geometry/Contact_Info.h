//  Contact_Info.h - information about collisions
//
//	Vamos Automotive Simulator
//  Copyright (C) 2005 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef _CONTACT_INFO_H_
#define _CONTACT_INFO_H_

#include "Material.h"
#include "Three_Vector.h"

namespace Vamos_Geometry
{
  /// Information about a collision between a point and a surface.
  struct Contact_Info
  {
	Contact_Info (bool contact_in, 
                  double depth_in,
                  const Vamos_Geometry::Three_Vector& normal_in,
                  const Vamos_Geometry::Material& material_in) :
	  contact (contact_in),
	  depth (depth_in),
	  normal (normal_in.unit ()),
	  material (material_in)
	{};

	bool contact; 
    //< True if a collision occurred, false otherwise. 
	double depth; 
    //< How far the point had penetrated the surface when the collision was
    //< detected.
	Vamos_Geometry::Three_Vector normal;
    //< The vector normal to the surface.
	Vamos_Geometry::Material material;
  };
}

#endif // not _CONTACT_INFO_H_
