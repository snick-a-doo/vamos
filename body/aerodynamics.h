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
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_BODY_AERODYNAMICS_H_INCLUDED
#define VAMOS_BODY_AERODYNAMICS_H_INCLUDED

#include "particle.h"
#include "../geometry/three-vector.h"

namespace Vamos_Body
{
/// A particle that produces drag.
class Drag : public Particle
{
public:
    Drag(Vamos_Geometry::Three_Vector const& position,
         double frontal_area,
         double drag_coefficient,
         Frame const* parent = nullptr);

    /// Find and store the forces, impulses, and torques for the current configuration.
    virtual void find_forces();
    /// Calculate the drag and lift due to @p wind_vector. @p wind_vector is supplied by
    /// the Body in the Body's frame.  @p density is the density of the atmosphere.
    void wind(Vamos_Geometry::Three_Vector const& wind_vector, double density);
    /// @return The value that gets multiplied by v^2 to get the drag force.
    virtual double drag_factor() const;
    /// @return The value that gets multiplied by v^2 to get the lift.
    virtual double lift_factor() const { return 0.0; }

protected:
    /// @return The current wind velocity vector.
    Vamos_Geometry::Three_Vector const& wind_vector() const { return m_wind_vector; }
    /// @return The current air density.
    double air_density() const { return m_density; }

private:
    /// The current wind velocity vector.
    Vamos_Geometry::Three_Vector m_wind_vector;
    /// The current air density.
    double m_density{0.0};
    /// The frontal area of the particle.
    double m_frontal_area{0.0};
    /// The coefficient of drag.
    double m_drag_coefficient{0.0};
  };

/// A particle that produces lift or downforce.
class Wing : public Drag
{
public:
    Wing(Vamos_Geometry::Three_Vector const& position,
         double frontal_area,
         double surface_area,
         double lift_coefficient,
         double efficiency,
         const Frame* parent = nullptr);

    virtual void find_forces() override;
    virtual double lift_factor() const override;

private:
    /// The area of the surface of the wing.
    double m_surface_area{0.0};
    /// The coefficient of lift.
    double m_lift_coefficient{0.0};
};
}

#endif // VAMOS_BODY_AERODYNAMICS_H_INCLUDED
