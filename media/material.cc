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

#include "material.h"
#include "texture-image.h"

#include <cmath>
#include <numbers>

using namespace Vamos_Geometry;
using namespace Vamos_Media;

Material::Material(Composition composition, double friction, double restitution,
                   double rolling, double drag,
                   Two_Vector const& bump_amplitude, double bump_wavelength,
                   std::shared_ptr<Texture_Image> texture)
    : m_composition{composition},
      m_friction_factor{friction},
      m_restitution_factor{restitution},
      m_rolling_resistance_factor{rolling},
      m_drag_factor{drag},
      m_bump_amplitude{bump_amplitude},
      m_bump_wavelength{bump_wavelength},
      mp_texture{std::move(texture)}
{
}

Material::Material(Composition composition, double friction, double restitution)
    : m_composition(composition),
      m_friction_factor(friction),
      m_restitution_factor(restitution)
{
}

Three_Vector Material::bump(double x, double y) const
{
    if (m_bump_wavelength == 0)
        return Three_Vector();

    using namespace std::numbers;
    auto bump_function = [](double x, double y) {
        auto phase{pi * (x + y)};
        return 0.25 * (std::sin(phase) + sin(sqrt2 * phase));
    };

    // To avoid interfering with the initial placement of the car, don't return a positive
    // number.
    auto bump{bump_function(x / m_bump_wavelength, y / m_bump_wavelength)};
    auto side{m_bump_amplitude.y * bump};
    auto up{m_bump_amplitude.x * (bump - 0.5)};
    return {side, side, up};
}
