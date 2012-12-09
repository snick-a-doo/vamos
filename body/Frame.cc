//  Frame.cc - a coordinate system.
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

#include "Frame.h"
#include "../geometry/Conversions.h"

#include <cassert>
#include <cmath>

using Vamos_Geometry::Three_Vector;
using Vamos_Geometry::Three_Matrix;
using namespace Vamos_Body;

//* Class Frame

//** Constructors

// Specify the position and orientation.
Frame::Frame (const Three_Vector& position, 
              const Three_Matrix& orientation,
              const Frame* parent) 
  : mp_parent (parent),
    m_orientation (orientation),
    m_position (position)
{
}

// Take the parent's orientation.
Frame::Frame (const Three_Vector& position,
              const Frame* parent) 
  : mp_parent (parent),
    m_position (position)
{
}

// Make a frame that's coincident with the parent frame.
Frame::Frame (const Frame* parent)
  : mp_parent (parent)
{
}


Three_Vector 
Frame::transform_from_parent (const Three_Vector& r) const
{
  return rotate_from_parent (r - m_position);
}

Three_Vector 
Frame::transform_velocity_from_parent (const Three_Vector& v) const
{
  return rotate_from_parent (v - m_velocity);
}

Three_Vector 
Frame::transform_from_world (const Three_Vector& r) const
{
  Three_Vector in = transform_from_parent (r);
  if (is_world_frame ())
    return in;
  else
    return mp_parent->transform_from_world (in);
}

Three_Vector 
Frame::transform_velocity_from_world (const Three_Vector& v) const
{
  Three_Vector in = transform_velocity_from_parent (v);
  if (is_world_frame ())
    return in;
  else
    return mp_parent->transform_velocity_from_world (in);
}


Three_Vector 
Frame::transform_to_parent (const Three_Vector& r) const
{
  return rotate_to_parent (r) + m_position;
}

Three_Vector 
Frame::transform_velocity_to_parent (const Three_Vector& v) const
{
  return rotate_to_parent (v) + m_velocity;
}


Three_Vector 
Frame::transform_to_world (const Three_Vector& r) const
{
  Three_Vector out = transform_to_parent (r);
  if (is_world_frame ())
    return out;
  else
    return mp_parent->transform_to_world (out);
}

Three_Vector 
Frame::transform_velocity_to_world (const Three_Vector& v) const
{
  Three_Vector out = transform_velocity_to_parent (v);
  if (is_world_frame ())
    return out;
  else
    return mp_parent->transform_velocity_to_world (out);
}



Three_Vector
Frame::rotate_to_parent (const Three_Vector& vector) const
{
  return m_orientation * vector;
}

Three_Vector
Frame::rotate_to_world (const Three_Vector& vector) const
{
  Three_Vector out = rotate_to_parent (vector);
  if (is_world_frame ())
    return out;
  else
    return mp_parent->rotate_to_world (out);
}

Three_Vector
Frame::rotate_from_parent (const Three_Vector& vector) const
{
  return m_orientation.transpose() * vector;
}

Three_Vector
Frame::rotate_from_world (const Three_Vector& vector) const
{
  Three_Vector in = rotate_from_parent (vector);
  if (is_world_frame ())
    return in;
  else
    return mp_parent->rotate_from_world (in);
}



// Express the orientation of this frame as a vector in the parent
// frame and a rotation about that vector.  ANGLE holds the rotation
// angle when the function returns.  The returned vector has a
// magnitude of sin (ANGLE).  The values returned are suitable for use
// with the glRotate functions.
Three_Vector 
Frame::axis_angle (double* angle) const
{
  // To convert the rotation matrix representation of the body's orientation
  // to an axis-angle orientation, we transform first to a quaternion
  // representation.  The matrix-to-quaternion and quaternion-to-axis-angle
  // transformations are described in the Matrix and Quaternion FAQ 
  // (matrixfaq.htm) in the doc directory.

  // Make a local reference to the tranformation matrix for brevity.
  const Three_Matrix& omat = m_orientation;

  // Convert from matrix to quaternion
  double trace = omat [0][0] + omat [1][1] + omat [2][2] + 1.0;
  double s, w, x, y, z;
  s = w = x = y = z = 0.0;
  if (trace > 0.0)
    {
      s = 0.5 / sqrt (trace);
      w = 0.25 / s;
      x = (omat [2][1] - omat [1][2]) * s;
      y = (omat [0][2] - omat [2][0]) * s;
      z = (omat [1][0] - omat [0][1]) * s;
    }
  else
    {
      // Find the largest diagonal element and do the appropriate
      // transformation.
      double largest = omat [0][0];
      int index = 0;
      if (omat [1][1] > largest)
        {
          largest = omat [1][1];
          index = 1;
        }

      if (omat [2][2] > largest)
        {
          largest = omat [2][2];
          s = sqrt (1.0 - omat [0][0] - omat [1][1] + omat [2][2]) * 2.0;
          w = (omat [0][1] + omat [1][0]) / s;
          x = (omat [0][2] + omat [2][0]) / s;
          y = (omat [1][2] + omat [2][1]) / s;
          z = 0.5 / s;
        }
      else if (index == 0)
        {
          s = sqrt (1.0 + omat [0][0] - omat [1][1] - omat [2][2]) * 2.0;
          w = (omat [1][2] + omat [2][1]) / s;
          x = 0.5 / s;
          y = (omat [0][1] + omat [1][0]) / s;
          z = (omat [0][2] + omat [2][0]) / s;
        }
      else
        {
          assert (index == 1);
          s = sqrt (1.0 - omat [0][0] + omat [1][1] - omat [2][2]) * 2.0;
          w = (omat [0][2] + omat [2][0]) / s;
          x = (omat [0][1] + omat [1][0]) / s;
          y = 0.5 / s;
          z = (omat [1][2] + omat [2][1]) / s;
        }
    }

  // Convert from quaternion to angle-axis.
  *angle = Vamos_Geometry::rad_to_deg (acos (w) * 2.0);

  // The return value would be divided by sin (angle) to give a unit
  // vector, but glRotate* () does not care about the length so we'll
  // leave it as is.
  return Three_Vector (x, y, z);
}
