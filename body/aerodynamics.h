//  Aerodynamic_Device.h - a particle that produces drag and lift.
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

#ifndef _AERODYNAMIC_DEVICE_H_
#define _AERODYNAMIC_DEVICE_H_

#include "particle.h"
#include "../geometry/three-vector.h"

namespace Vamos_Body
{
  /// An Aerodynamic_Device that produces drag.
  class Drag : public Particle
  {
  public:
	Drag (const Vamos_Geometry::Three_Vector& position,
		  double frontal_area, 
          double drag_coefficient,
          const Frame* parent = 0);

	// Find and store the forces, impulses, and torques for the
	// current configuration.
	virtual void find_forces ();

	// Calculate the drag and lift due to WIND_VECTOR.  WIND_VECTOR is
	// supplied by the Body in the Body's frame.  DENSITY is the denisty
	// of the atmosphere.
	void wind (const Vamos_Geometry::Three_Vector& wind_vector, 
			   double density);

    /// Return the value that gets multiplied by v^2 to get the drag force.
    virtual double drag_factor () const;

    /// Return the value that gets multiplied by v^2 to get the lift.
    virtual double lift_factor () const { return 0.0; }

  protected:
	// The current wind velocity vector.
	const Vamos_Geometry::Three_Vector& wind_vector () const { return m_wind_vector; }

	// The current air density.
	double air_density () const { return m_density; }

  private:
	// The current wind velocity vector.
	Vamos_Geometry::Three_Vector m_wind_vector;

	// The current air density.
	double m_density;

	// The frontal area of the particle.
	double m_frontal_area;

	// The coefficient of drag.
	double m_drag_coefficient;
  };

  /// A device that produces lift or downforce.
  class Wing : public Drag
  {
  public:
	Wing (const Vamos_Geometry::Three_Vector& position,
		  double frontal_area, 
		  double surface_area, 
          double lift_coefficient,
		  double efficiency,
          const Frame* parent = 0);

	// Find and store the forces, impulses, and torques for the
	// current configuration.
	virtual void find_forces ();

    /// Return the value that gets multiplied by v^2 to get the lift.
    virtual double lift_factor () const;

  private:
	// The area of the surface of the wing.
	double m_surface_area;

	// The coefficient of lift.
	double m_lift_coefficient;
  };
}

#endif // not _AERODYNAMIC_DEVICE_H_
