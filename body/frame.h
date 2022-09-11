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

#ifndef VAMOS_BODY_FRAME_H_INCLUDED
#define VAMOS_BODY_FRAME_H_INCLUDED

#include "../geometry/three-matrix.h"
#include "../geometry/three-vector.h"

namespace Vamos_Body
{
/// A Frame describes a coordinate system.
class Frame
{
public:
    /// Specify the position and orientation.
    Frame(Vamos_Geometry::Three_Vector const& position,
          Vamos_Geometry::Three_Matrix const& orientation);
    /// Take the parent's orientation.
    Frame(Vamos_Geometry::Three_Vector const& position);
    /// Make a frame that's coincident with the parent frame.
    Frame();

    /// @return The orientation of the origin in the parent frame.
    Vamos_Geometry::Three_Matrix const& orientation() const { return m_orientation; }
    /// @return The position of the origin in the parent frame.
    Vamos_Geometry::Three_Vector const& position() const { return m_position; }
    /// @return the velocity of the origin relative to the parent frame.
    Vamos_Geometry::Three_Vector const& velocity() const { return m_velocity; }
    /// @return the angular velocity of the frame relative to the parent frame.
    Vamos_Geometry::Three_Vector const& angular_velocity() const { return m_angular_velocity; }

    // Give this frame an absolute orientation.
    void set_orientation(Vamos_Geometry::Three_Matrix const& o) { m_orientation = o; }
    // Rotate the frame about a vector by an angle equal to its magnitude.
    void rotate(Vamos_Geometry::Three_Vector const& delta_theta)
    { m_orientation.rotate(delta_theta); }

    /// @return The representation of an absolute position in this frame.
    Vamos_Geometry::Three_Vector transform_in(Vamos_Geometry::Three_Vector const& vec) const;
    /// @return The absolute position of a position in this frame.
    Vamos_Geometry::Three_Vector transform_out(Vamos_Geometry::Three_Vector const& vec) const;
    /// @return The representation of an absolute velocity in this frame.
    Vamos_Geometry::Three_Vector transform_velocity_in(
        Vamos_Geometry::Three_Vector const& v) const;
    /// @return The absolute velocity of a velocity in this frame.
    Vamos_Geometry::Three_Vector transform_velocity_out(
        Vamos_Geometry::Three_Vector const& v) const;
    /// @return A vector in this frame that's parallel to the given displacement vector.
    Vamos_Geometry::Three_Vector rotate_in(Vamos_Geometry::Three_Vector const& vec) const;
    /// @return An absolute vector that's parallel to a vector in this frame.
    Vamos_Geometry::Three_Vector rotate_out(Vamos_Geometry::Three_Vector const& vec) const;

    /// Set the absolute position of this frame's origin.
    void set_position(Vamos_Geometry::Three_Vector const& r) { m_position = r; }
    /// Move the origin by an absolute amount.
    void translate(Vamos_Geometry::Three_Vector const& delta_r) { m_position += delta_r; }
    /// Set the absolute velocity of this frame's origin.
    void set_velocity(Vamos_Geometry::Three_Vector const& v) { m_velocity = v; }
    /// Change the velocity by an absolute amount.
    void accelerate(Vamos_Geometry::Three_Vector const& delta_v) { m_velocity += delta_v; }
    /// Set the angular velocity about an axis through this frame's origin.
    void set_angular_velocity(Vamos_Geometry::Three_Vector const& omega)
    { m_angular_velocity = omega; }
    /// Change the angular velocity about an axis through this frame's origin.
    void angular_accelerate(Vamos_Geometry::Three_Vector const& delta_omega)
    { m_angular_velocity += delta_omega; }

    /// Express the orientation of this frame as a vector in the parent frame and a
    /// rotation about that vector. The values returned are suitable for use with the
    /// glRotate functions.
    /// @param angle Filled in with the rotation angle.
    /// @return The rotation axis. The magnitude sin(angle) because that's what the
    /// calculation gives. It's left unmodified for efficiency.
    Vamos_Geometry::Three_Vector axis_angle(double& angle) const;

private:
    Frame const* mp_parent{nullptr};
    Vamos_Geometry::Three_Matrix m_orientation{1.0};
    Vamos_Geometry::Three_Vector m_position;
    Vamos_Geometry::Three_Vector m_velocity;
    Vamos_Geometry::Three_Vector m_angular_velocity;
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_FRAME_H_INCLUDED
