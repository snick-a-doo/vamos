//  Aerodynamic_Device.cc - a particle that produces drag and lift.
//
//  Copyright (C) 2002 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#include "aerodynamics.h"
#include "../geometry/constants.h"
#include <cmath>

using namespace Vamos_Geometry;
using namespace Vamos_Body;

//-----------------------------------------------------------------------------
// Class Drag

Drag::Drag (const Three_Vector& position,
			double frontal_area, 
            double drag_coefficient,
            const Frame* parent) 
  : Particle (0.0, position, parent),
    m_frontal_area (frontal_area),
    m_drag_coefficient (drag_coefficient)
{
}

// Calculate the drag and lift due to WIND_VECTOR.  WIND_VECTOR is
// supplied by the Body in the Body's frame.
void
Drag::wind (const Three_Vector& wind_vector, double density)
{
  m_wind_vector = wind_vector;
  m_density = density;
}

// Return the value that gets multiplied by v^2 to get the drag force.
double
Drag::drag_factor () const
{
  return 0.5 * m_density * m_drag_coefficient * m_frontal_area;
}

// Find and store the forces, impulses, and torques for the current
// configuration.
void
Drag::find_forces ()
{
  // Calculate the drag and lift forces.
  set_force (drag_factor () * m_wind_vector * m_wind_vector.magnitude ());
}

//-----------------------------------------------------------------------------
// Class Wing

Wing::Wing (const Three_Vector& position,
			double frontal_area,  
			double surface_area, 
            double lift_coefficient,
			double efficiency,
            const Frame* parent) 
: Drag (position, 
        frontal_area, 
        std::abs (lift_coefficient * (1.0 - efficiency)), 
        parent),
  m_surface_area (surface_area),
  m_lift_coefficient (lift_coefficient * efficiency)
{
}

// Find and store the forces, impulses, and torques for the current
// configuration.
void
Wing::find_forces ()
{
  const double wind_speed = std::abs (wind_vector ().dot (Three_Vector::X));
  const double lift = lift_factor () * wind_speed * wind_speed;

  // Add the lift to the drag to get the total force.
  Drag::find_forces ();
  set_force (force () + lift * Three_Vector::Z);
}

// Return the value that gets multiplied by v^2 to get the lift.
double
Wing::lift_factor () const
{
  return 0.5 * air_density () * m_lift_coefficient * m_surface_area;
}

