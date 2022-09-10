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

#include "particle.h"

using namespace Vamos_Body;
using namespace Vamos_Geometry;

Particle::Particle(double mass, Three_Vector const& position,
                   Material const& material, Frame const* parent)
    : Frame{position, parent},
      m_mass{mass},
      m_material{material}
{
}

Particle::Particle(double mass, Three_Vector const& position, Frame const* parent)
    : Particle{mass, position, Material(), parent}
{
}

Particle::Particle(double mass, Frame const* parent)
    : Particle{mass, Three_Vector::ZERO, Material(), parent}
{
}

bool Particle::can_contact() const
{
    return m_material.composition() != Material::UNKNOWN;
}

double Particle::contact(Three_Vector const& impulse,
                         Three_Vector const&, // velocity
                         double,              // distance
                         Three_Vector const&, // normal
                         Three_Vector const&, // ang_vel
                         Material const&)     // material
{
    if (can_contact())
    {
        set_impulse(rotate_from_parent(impulse));
        m_contact = true;
    }
    return 0.0;
}

void Particle::find_forces()
{
    if (!m_contact)
        reset();
}

void Particle::end_timestep()
{
    m_contact = false;
}

void Particle::reset()
{
    m_force.zero();
    m_impulse.zero();
    m_torque.zero();
}
