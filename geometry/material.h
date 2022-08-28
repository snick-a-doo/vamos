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
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_GEOMETRY_MATERIAL_H_INCLUDED
#define VAMOS_GEOMETRY_MATERIAL_H_INCLUDED

#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"

#include <string>

namespace Vamos_Geometry
{
class Material
{
public:
    enum Composition{
        RUBBER, METAL, ASPHALT, CONCRETE, KERB, GRASS, GRAVEL, DIRT, AIR, UNKNOWN
    };

    Material(Composition composition, double friction, double restitution,
             double rolling,  double drag,
             Two_Vector const& bump_amplitude, double bump_wavelength,
             std::string const& texture_file_name, bool smooth, bool mip_map,
             double width, double height);
    Material(Composition = UNKNOWN, double friction = 1.0, double restitution = 1.0);

    /// @return A small displacement due to unevenness in the material.
    Three_Vector bump(double x, double y) const;

    /// Read-only access to material properties.
    /// @{
    std::string const& texture_file_name() const { return m_texture_file_name; }
    double friction_factor() const { return m_friction_factor; }
    double rolling_resistance_factor() const { return m_rolling_resistance_factor; }
    double drag_factor() const { return m_drag_factor; }
    double restitution_factor() const { return m_restitution_factor; }
    Composition composition() const { return m_composition; }
    bool smooth() const { return m_smooth; }
    bool mip_map() const { return m_mip_map; }
    double width() const { return m_width; }
    double height() const { return m_height; }
    /// @}

private:
    Composition m_composition{UNKNOWN};

    // Corresponding object properties are muliplied by these factors. They should all be
    // 1.0 for pavement.
    double m_friction_factor{1.0};
    double m_restitution_factor{1.0};
    double m_rolling_resistance_factor{0.0};
    double m_drag_factor{0.0};

    Two_Vector m_bump_amplitude;
    double m_bump_wavelength{1.0};

    /// Texture image properties
    /// @{
    std::string m_texture_file_name;
    bool m_smooth{false};
    bool m_mip_map{false};
    double m_width{1.0};
    double m_height{1.0};
    /// @}
};
} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_MATERIAL_H_INCLUDED
