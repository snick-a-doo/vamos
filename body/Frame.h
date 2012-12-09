// Frame.h - a coordinate system.
//
//  Copyright (C) 2001--2004 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
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

#ifndef _FRAME_H_
#define _FRAME_H_

#include "../geometry/Three_Vector.h"
#include "../geometry/Three_Matrix.h"

namespace Vamos_Body
{
  // A Frame describes a coordinate system.
  class Frame
  {
  public:
    // Specify the position and orientation.
    Frame (const Vamos_Geometry::Three_Vector& position, 
           const Vamos_Geometry::Three_Matrix& orientation,
           const Frame* parent = 0);

    // Take the parent's orientation.
    Frame (const Vamos_Geometry::Three_Vector& position,
           const Frame* parent = 0);

    // Make a frame that's coincident with the parent frame.
    Frame (const Frame* parent = 0);

    bool is_world_frame () const { return mp_parent == 0; }

    Vamos_Geometry::Three_Matrix orientation () const
    { return m_orientation; }

    // Return the position of the origin in the parent frame.
    Vamos_Geometry::Three_Vector position () const 
    { return m_position; }

    // Return the velocity of the origin relative to the parent frame.
    Vamos_Geometry::Three_Vector velocity () const 
    { return m_velocity; }

    // Return the angular velocity of the frame relative to the parent
    // frame.
    Vamos_Geometry::Three_Vector angular_velocity () const 
    { return m_angular_velocity; }


    // Give this frame an absolute orientation of NEW_ORIENTATION.
    void set_orientation (const Vamos_Geometry::Three_Matrix& new_orientation)
    { m_orientation = new_orientation; }

    // Rotate the frame about the vector delta_theta, by an angle equal to
    // the magnitude of DELTA_THETA.
    void rotate (const Vamos_Geometry::Three_Vector& delta_theta)
    { m_orientation.rotate (delta_theta); }

    // VEC is a vector in the parent's frame.  The representation of
    // VEC in this frame is returned.
    Vamos_Geometry::Three_Vector 
    transform_from_parent (const Vamos_Geometry::Three_Vector& vec) const;

    Vamos_Geometry::Three_Vector 
    transform_from_world (const Vamos_Geometry::Three_Vector& vec) const;

    // VEC is a vector in this frame.  The representation of VEC in
    // the parent's frame is returned.
    Vamos_Geometry::Three_Vector 
    transform_to_parent (const Vamos_Geometry::Three_Vector& vec) const;

    Vamos_Geometry::Three_Vector 
    transform_to_world (const Vamos_Geometry::Three_Vector& vec) const;


    // VELOCITY is a velocity vector in the world frame.  The
    // representation of VELOCITY in this frame is returned.
    Vamos_Geometry::Three_Vector 
    transform_velocity_from_world
    (const Vamos_Geometry::Three_Vector& velocity) const;

    // VELOCITY is a vector in this frame.  The representation of
    // VELOCITY in the world frame is returned.
    Vamos_Geometry::Three_Vector 
    transform_velocity_to_world 
    (const Vamos_Geometry::Three_Vector& velocity) const;

    // VELOCITY is a velocity vector in the parent's frame.  The
    // representation of VELOCITY in this frame is returned.
    Vamos_Geometry::Three_Vector 
    transform_velocity_from_parent
    (const Vamos_Geometry::Three_Vector& velocity) const;

    // VELOCITY is a vector in this frame.  The representation of
    // VELOCITY in the parent's frame is returned.
    Vamos_Geometry::Three_Vector 
    transform_velocity_to_parent
    (const Vamos_Geometry::Three_Vector& velocity) const;

    // Same as transform_in (VEC) above, except that translation is
    // not performed.
    Vamos_Geometry::Three_Vector 
    rotate_from_parent (const Vamos_Geometry::Three_Vector& vec) const;
    Vamos_Geometry::Three_Vector 
    rotate_from_world (const Vamos_Geometry::Three_Vector& vec) const;

    // Same as transform_out (VEC) above, except that translation is
    // not performed.
    Vamos_Geometry::Three_Vector 
    rotate_to_parent (const Vamos_Geometry::Three_Vector& vec) const;
    Vamos_Geometry::Three_Vector 
    rotate_to_world (const Vamos_Geometry::Three_Vector& vec) const;

    void set_position (const Vamos_Geometry::Three_Vector& r) 
    { m_position = r; }

    // Change the position by DELTA_R.
    void translate (const Vamos_Geometry::Three_Vector& delta_r) 
    { m_position += delta_r; }

    void set_velocity (const Vamos_Geometry::Three_Vector& v) 
    { m_velocity = v; }

    void accelerate (const Vamos_Geometry::Three_Vector& delta_v) 
    { m_velocity += delta_v; }

    void set_angular_velocity (const Vamos_Geometry::Three_Vector& omega) 
    { m_angular_velocity = omega; }

    void angular_accelerate (const Vamos_Geometry::Three_Vector& delta_omega) 
    { m_angular_velocity += delta_omega; }


    // Express the orientation of this frame as a vector in the parent
    // frame and a rotation about that vector.  ANGLE holds the
    // rotation angle when the function returns.  The returned vector
    // has a magnitude of sin (ANGLE).  The values returned are
    // suitable for use with the glRotate functions.
    Vamos_Geometry::Three_Vector axis_angle (double* angle) const;

  private:
    const Frame* mp_parent;

    Vamos_Geometry::Three_Matrix m_orientation;

    // The position of the origin of this frame relative to the parent frame.
    Vamos_Geometry::Three_Vector m_position;

    // The position of the origin of this frame relative to the parent
    // frame.  This is the vector from the parent frame's origin to
    // this frame's origin, expressed in the coordinates of the parent
    // frame.
    Vamos_Geometry::Three_Vector m_velocity;

    // The angular velocity vector of this frame expressed in the
    // coordinates of the parent's frame.
    Vamos_Geometry::Three_Vector m_angular_velocity;
  };
}

#endif // not _FRAME_H_
