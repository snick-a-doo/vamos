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
    Frame(Vamos_Geometry::Three_Vector const& position, Vamos_Geometry::Three_Matrix const& orientation, Frame const* parent = nullptr);
    /// Take the parent's orientation.
    Frame(Vamos_Geometry::Three_Vector const& position, Frame const* parent = nullptr);
    /// Make a frame that's coincident with the parent frame.
    Frame(Frame const* parent = nullptr);

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
    void rotate(Vamos_Geometry::Three_Vector const& delta_theta) { m_orientation.rotate(delta_theta); }

    Vamos_Geometry::Three_Vector transform_from_parent(Vamos_Geometry::Three_Vector const& vec) const;
    Vamos_Geometry::Three_Vector transform_from_world(Vamos_Geometry::Three_Vector const& vec) const;

    Vamos_Geometry::Three_Vector transform_to_parent(Vamos_Geometry::Three_Vector const& vec) const;
    Vamos_Geometry::Three_Vector transform_to_world(Vamos_Geometry::Three_Vector const& vec) const;

    Vamos_Geometry::Three_Vector transform_velocity_from_world(Vamos_Geometry::Three_Vector const& velocity) const;
    Vamos_Geometry::Three_Vector transform_velocity_to_world(Vamos_Geometry::Three_Vector const& velocity) const;

    Vamos_Geometry::Three_Vector transform_velocity_from_parent(Vamos_Geometry::Three_Vector const& velocity) const;
    Vamos_Geometry::Three_Vector transform_velocity_to_parent(Vamos_Geometry::Three_Vector const& velocity) const;

    Vamos_Geometry::Three_Vector rotate_from_parent(Vamos_Geometry::Three_Vector const& vec) const;
    Vamos_Geometry::Three_Vector rotate_from_world(Vamos_Geometry::Three_Vector const& vec) const;

    Vamos_Geometry::Three_Vector rotate_to_parent(Vamos_Geometry::Three_Vector const& vec) const;
    Vamos_Geometry::Three_Vector rotate_to_world(Vamos_Geometry::Three_Vector const& vec) const;

    void set_position(Vamos_Geometry::Three_Vector const& r) { m_position = r; }
    void translate(Vamos_Geometry::Three_Vector const& delta_r) { m_position += delta_r; }
    void set_velocity(Vamos_Geometry::Three_Vector const& v) { m_velocity = v; }
    void accelerate(Vamos_Geometry::Three_Vector const& delta_v) { m_velocity += delta_v; }
    void set_angular_velocity(Vamos_Geometry::Three_Vector const& omega) { m_angular_velocity = omega; }
    void angular_accelerate(Vamos_Geometry::Three_Vector const& delta_omega) { m_angular_velocity += delta_omega; }

    // Express the orientation of this frame as a vector in the parent frame and a
    // rotation about that vector.  ANGLE holds the rotation angle when the function
    // returns.  The returned vector has a magnitude of sin (ANGLE).  The values returned
    // are suitable for use with the glRotate functions.
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
