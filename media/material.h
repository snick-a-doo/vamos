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

#include <memory>
#include <string>

namespace Vamos_Media
{
class Texture_Image;

class Material
{
public:
    enum Composition{
        rubber, metal, asphalt, concrete, kerb, grass, gravel, dirt, air, unknown
    };

    Material(Composition composition, double friction, double restitution,
             double rolling,  double drag,
             Vamos_Geometry::Two_Vector const& bump_amplitude, double bump_wavelength,
             std::shared_ptr<Texture_Image> texture);
    Material(Composition = unknown, double friction = 1.0, double restitution = 1.0);

    /// @return A small displacement due to unevenness in the material.
    Vamos_Geometry::Three_Vector bump(double x, double y) const;

    /// Read-only access to material properties.
    /// @{
    double friction_factor() const { return m_friction_factor; }
    double rolling_resistance_factor() const { return m_rolling_resistance_factor; }
    double drag_factor() const { return m_drag_factor; }
    double restitution_factor() const { return m_restitution_factor; }
    Composition composition() const { return m_composition; }
    Texture_Image const* texture() const { return mp_texture.get(); }
    /// @}

private:
    Composition m_composition{unknown};

    // Corresponding object properties are muliplied by these factors. They should all be
    // 1.0 for pavement.
    double m_friction_factor{1.0};
    double m_restitution_factor{1.0};
    double m_rolling_resistance_factor{0.0};
    double m_drag_factor{0.0};
    Vamos_Geometry::Two_Vector m_bump_amplitude;
    double m_bump_wavelength{1.0};
    std::shared_ptr<Texture_Image> mp_texture;
};

/// Information about a collision between a point and a surface.
struct Contact_Info
{
    bool contact{false}; ///< True if a collision occurred, false otherwise.
    double depth{0.0}; ///< Penetration depth.
    Vamos_Geometry::Three_Vector normal; ///< The vector normal to the surface.
    Material material; ///< The material of the surface.
};
} // namespace Vamos_Media

#endif // VAMOS_GEOMETRY_MATERIAL_H_INCLUDED
