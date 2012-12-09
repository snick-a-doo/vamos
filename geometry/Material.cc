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

#include "Material.h"
#include "Constants.h"

using namespace Vamos_Geometry;

Material::Material (Material_Type type, 
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
                    double height)
  : m_type (type),
	m_friction_factor (friction),
	m_restitution_factor (restitution),
	m_rolling_resistance_factor (rolling),
	m_drag_factor (drag),
	m_bump_amplitude (bump_amplitude),
	m_bump_wavelength (bump_wavelength),
    m_texture_file_name (texture_file_name),
    m_smooth (smooth),
    m_mip_map (mip_map),
    m_width (width),
    m_height (height)
{
}

Material::Material (Material_Type type,
                    double friction,
                    double restitution)
  : m_type (type),
    m_friction_factor (friction),
	m_restitution_factor (restitution),
	m_rolling_resistance_factor (0.0),
	m_drag_factor (0.0),
	m_bump_wavelength (1.0),
    m_texture_file_name (""),
    m_smooth (false),
    m_mip_map (false),
    m_width (1.0),
    m_height (1.0)
{
}

inline double bump_function (double x, double y)
{
  const double phase = two_pi * (x + y);
  return 0.25 * (sin (phase) + sin (root_2 * phase));
}

Three_Vector 
Material::bump (double x, double y) const
{
  if (m_bump_wavelength == 0)
    return Three_Vector();

  // To avoid interfering with the initial placement of the car, don't
  // return a positive number.
  const double bump = bump_function (x / m_bump_wavelength,
                                     y / m_bump_wavelength);
  const double side = m_bump_amplitude.y * bump;
  const double up = m_bump_amplitude.x * (bump - 0.5);
  return Three_Vector (side, side, up);
}
