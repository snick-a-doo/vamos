//	Particle.h - a massive particle which is part of a rigid body.
//
//  Copyright (C) 2001--2002 Sam Varner
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

#ifndef _PARTICLE_H_
#define _PARTICLE_H_

#include "../geometry/Three_Vector.h"
#include "../geometry/Three_Matrix.h"
#include "../geometry/Inertia_Tensor.h"
#include "../geometry/Material.h"
#include "Frame.h"

namespace Vamos_Body
{
  // Particle is a point mass which may be part of a rigid body.  It
  // has position and orientation information inherited from Frame.  A
  // Particle can exert forces and torques.
  class Particle : public Frame
  {
  public:
	// Specify position and orientation.
	Particle (double mass, 
              const Vamos_Geometry::Three_Vector& position, 
			  const Vamos_Geometry::Three_Matrix& orientation,
              const Frame* parent = 0);

	// Take the parent's orientation.
	Particle (double mass, 
              const Vamos_Geometry::Three_Vector& position,
              const Frame* parent = 0);

	// The particle's frame is coincident with the parent's.
	Particle (double mass = 0.0,
              const Frame* parent = 0);

	virtual ~Particle () {}

	// Return the force exerted on the rigid body in the body's frame.
	virtual Vamos_Geometry::Three_Vector force () const
	{ return rotate_to_parent (m_force); }

	// Return the impulse exerted on the rigid body in the body's
	// frame.
	virtual Vamos_Geometry::Three_Vector impulse () const
	{ return rotate_to_parent (m_impulse); }

	// Return the torque exerted on the rigid body in the body's frame.
	virtual Vamos_Geometry::Three_Vector torque () const
	{ return rotate_to_parent (m_torque); }

	// Classes derived from Particle may lie about their positions
	// for collisions...
	virtual Vamos_Geometry::Three_Vector contact_position () const 
	{ return position (); }

	// ...for exerting forces and impulses...
	virtual Vamos_Geometry::Three_Vector force_position () const 
	{ return position (); }

	// ...for exerting torques...
	virtual Vamos_Geometry::Three_Vector torque_position () const 
	{ return position (); }

	// ...or for constructing the inertia tensor of the rigid body.
	virtual Vamos_Geometry::Three_Vector mass_position () const 
	{ return position (); }

	virtual double contact (const Vamos_Geometry::Three_Vector& impulse,
                            const Vamos_Geometry::Three_Vector& velocity, 
                            double distance,
                            const Vamos_Geometry::Three_Vector& normal,
                            const Vamos_Geometry::Three_Vector& angular_velocity,
                            const Vamos_Geometry::Material& material)
	{ return 0.0; }

	// Return the particle's mass.
	double mass () const { return m_mass; }

	virtual bool single_contact () const { return true; }

	// Return the material properties.
	const Vamos_Geometry::Material& material () const 
	{ return m_material; }

	// Find and store the forces, impulses, and torques for the
	// current configuration.
	virtual void find_forces () {}

	// Propagate the Particle forward in time by TIME.
	virtual void propagate (double time) {};

	// Undo the last propagation.
	virtual void rewind () {}

	// Do any necessary cleanup at the end of a time step.
	virtual void end_timestep () {}

	// Set the force, impulse and torque to zero;
	virtual void reset ();

  protected:
    void set_mass (double new_mass) { m_mass = new_mass; }

    void set_material (const Vamos_Geometry::Material& new_material)
    { m_material = new_material; }

    void set_force (const Vamos_Geometry::Three_Vector& new_force) 
    { m_force = new_force; }

    void set_impulse (const Vamos_Geometry::Three_Vector& new_impulse) 
    { m_impulse = new_impulse; }

    void set_torque (const Vamos_Geometry::Three_Vector& new_torque) 
    { m_torque = new_torque; }

  private:
	// The mass of the particle.
	double m_mass;

	// Material properties for the particle.
	Vamos_Geometry::Material m_material;

	// The resultant force exerted by the component.
	Vamos_Geometry::Three_Vector m_force;

	// The impulse exerted by the component.
	Vamos_Geometry::Three_Vector m_impulse;

	// The resultant torque exerted by the component.
	Vamos_Geometry::Three_Vector m_torque;
  };
}

#endif // not _PARTICLE_H_
