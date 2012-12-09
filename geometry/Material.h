//	Vamos Automotive Simulator
//  Copyright (C) 2001--2002 Sam Varner
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

#ifndef _MATERIAL_H_
#define _MATERIAL_H_

#include "../geometry/Three_Vector.h"
#include "../geometry/Two_Vector.h"

#include <string>

namespace Vamos_Geometry
{
  class Material
  {
  public:
	enum Material_Type
	  {
		RUBBER,
		METAL,
		ASPHALT,
		CONCRETE,
        KERB,
		GRASS,
		GRAVEL,
		DIRT,
        AIR,
		UNKNOWN
	  };

  private:
	Material_Type m_type;

	// The frictional properties of the car are muliplied by these factors
	// for this particular surface.  They should all be 1.0 for pavement.
	double m_friction_factor;
	double m_restitution_factor;
	double m_rolling_resistance_factor;
	double m_drag_factor;

	Two_Vector m_bump_amplitude;
	double m_bump_wavelength;

    std::string m_texture_file_name;

    bool m_smooth;
    bool m_mip_map;
    double m_width;
    double m_height;

  public:
	Material (Material_Type type,
              double friction, 
              double restitution,
			  double rolling, 
              double drag, 
			  const Two_Vector& bump_amplitude,
              double bump_wavelength,
              std::string texture_file_name,
              bool smooth, 
              bool mip_map,
              double width, 
              double height);

    Material (Material_Type = UNKNOWN,
              double friction = 1.0,
              double restitution = 1.0);

    std::string texture_file_name () const { return m_texture_file_name; }
	double friction_factor () const { return m_friction_factor; }
	double rolling_resistance_factor () const 
	{ return m_rolling_resistance_factor; }
	double drag_factor () const { return m_drag_factor; }
	double restitution_factor () const { return m_restitution_factor; }
	Three_Vector bump (double x, double y) const;
	Material_Type type () const { return m_type; }
    bool smooth () const { return m_smooth; }
    bool mip_map () const { return m_mip_map; }
    double width () const { return m_width; }
    double height () const { return m_height; }
  };
}

#endif
