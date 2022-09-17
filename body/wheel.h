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
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_BODY_WHEEL_H_INCLUDED
#define VAMOS_BODY_WHEEL_H_INCLUDED

#include "brake.h"
#include "particle.h"
#include "suspension.h"
#include "tire.h"

#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"
#include "../media/material.h"

#include <GL/gl.h>

#include <memory>
#include <string>

namespace Vamos_Body
{
class Wheel : public Particle
{
public:
    Wheel(double mass, Vamos_Geometry::Three_Vector const& position, double tire_offset,
          double roll_height, double restitution, std::shared_ptr<Suspension> suspension,
          Tire const& tire, Brake const& brake, bool steered, bool driven, Side side);

    virtual void reset() override;
    virtual bool single_contact() const override { return false; }
    virtual void find_forces() override;
    virtual void propagate(double time) override;

    // Handle collisions.  The return value is how much the wheel was displaced by the
    // collision.
    virtual double contact(const Vamos_Geometry::Three_Vector& impulse,
                           const Vamos_Geometry::Three_Vector& velocity, double distance,
                           const Vamos_Geometry::Three_Vector& normal,
                           const Vamos_Geometry::Three_Vector& angular_velocity,
                           const Vamos_Geometry::Material& surface_material) override;

    /// Apply torque if it's a driven wheel.
    void set_drive_torque(double torque);
    /// Apply brakes.
    void brake(double factor);
    // Set the steering angle if it's a steered wheel.
    void steer(double degree_angle);
    /// Set the 3D models for rendering the wheel.
    void set_models(std::string const& slow_file, std::string const& fast_file,
                    double transition_speed, std::string const& stator_file,
                    double stator_offset, double scale,
                    Vamos_Geometry::Three_Vector const& translation,
                    Vamos_Geometry::Three_Vector const& rotation);
    /// Render the wheel.
    void draw();

    /// @return The side of the car the wheel is on.
    Side side() const { return m_side; }
    /// @return The wheel's rotational speed in radians per second.
    double rotational_speed() const { return m_tire.rotational_speed(); }
    /// @return The linear speed of the tire tread. This is the speed that a speedometer on
    /// this wheel would read.
    double speed() const { return m_tire.speed(); }
    /// @return the position relative to the body where the wheel exerts its force.
    Vamos_Geometry::Three_Vector force_position() const;
    /// @return the position of the wheel relative to the body for the purpose of detecting
    /// collisions.
    Vamos_Geometry::Three_Vector contact_position() const;
    /// @return The tire attached to the wheel.
    const Tire& get_tire() const { return m_tire; }

private:
    // Return the display list for the specified model file.
    GLuint make_model(std::string const& file, double scale,
                      Vamos_Geometry::Three_Vector const& translation,
                      Vamos_Geometry::Three_Vector const& rotation);

    /// The initial position of the wheel.
    Vamos_Geometry::Three_Vector m_original_position;
    /// The distance from the steering pivot to the effective center of the contact patch.
    double m_tire_offset;
    /// How far off the road lateral forces are applied to the chassis.
    double m_roll_height;
    /// The suspension that the wheel is attached to.
    std::shared_ptr<Suspension> mp_suspension;
    Tire m_tire; ///< The tire that is attached to the wheel.
    Brake m_brake; ///< The brake for the wheel.
    /// The velocity of the ground relative to the wheel.
    Vamos_Geometry::Three_Vector m_ground_velocity;
    // The current normal vector for the road.
    Vamos_Geometry::Three_Vector m_normal;
    // The angular velocity of the wheel.
    Vamos_Geometry::Three_Vector m_angular_velocity;
    // The surface we're currently on
    Vamos_Geometry::Material m_surface_material;
    double m_drive_torque{0.0}; ///< The last applied drive torque.
    double m_braking_torque{0.0}; ///< The last applied braking torque.
    bool m_is_steered{false}; ///< True if the wheel responds to steering.
    bool m_is_driven{false}; ///< True if the wheel is powered by the drivetrain.
    Side m_side; ///< The side of the car that the wheel is on.
    double m_rotation{0.0}; ///< The current rotation angle about the axle.
    /// The speed where we start using rendering the wheel as non-rotating.
    double m_transition_speed{10.0};

    /// Display list IDs
    /// @{
    GLuint m_slow_wheel_list{0};
    GLuint m_fast_wheel_list{0};
    GLuint m_stator_list{0};
    /// @}
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_WHEEL_H_INCLUDED
