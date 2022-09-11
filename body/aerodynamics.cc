//  Copyright (C) 2002-2022 Sam Varner
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
//  If not, see <http://www.gnu.org/licenses/>.  along with this program; if not, write to
//  the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
//  USA

#include "aerodynamics.h"

#include <cmath>

using namespace Vamos_Geometry;
using namespace Vamos_Body;

//-----------------------------------------------------------------------------
Drag::Drag(Three_Vector const& position,
           double frontal_area,
           double drag_coefficient)
    : Particle{0.0, position},
      m_frontal_area{frontal_area},
      m_drag_coefficient{drag_coefficient}
{
}

void Drag::wind(Three_Vector const& wind_vector, double density)
{
    m_wind_vector = wind_vector;
    m_density = density;
}

double Drag::drag_factor () const
{
    return 0.5 * m_density * m_drag_coefficient * m_frontal_area;
}

void Drag::find_forces()
{
    // Calculate drag and lift forces.
    set_force(drag_factor() * m_wind_vector * m_wind_vector.magnitude());
}

//-----------------------------------------------------------------------------
Wing::Wing(Three_Vector const& position,
           double frontal_area,
           double surface_area,
           double lift_coefficient,
           double efficiency)
    : Drag {position, frontal_area, std::abs(lift_coefficient * (1.0 - efficiency))},
      m_surface_area{surface_area},
      m_lift_coefficient{lift_coefficient * efficiency}
{
}

void Wing::find_forces()
{
    const auto wind_speed = std::abs(wind_vector().dot(Three_Vector::X));
    const auto lift = lift_factor() * wind_speed * wind_speed;

    // Add lift to drag to get the total force.
    Drag::find_forces ();
    set_force(force() + lift * Three_Vector::Z);
}

double Wing::lift_factor() const
{
    return 0.5 * air_density() * m_lift_coefficient * m_surface_area;
}

