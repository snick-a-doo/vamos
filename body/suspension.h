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

#ifndef VAMOS_BODY_SUSPENSION_H_INCLUDED
#define VAMOS_BODY_SUSPENSION_H_INCLUDED

#include "particle.h"

#include "../geometry/three-vector.h"

#include <memory>
#include <string>
#include <vector>

namespace Vamos_Body
{
struct Suspension_Model;

enum class Side{left, right};

class Hinge : public Particle
{
public:
    Hinge(Vamos_Geometry::Three_Vector const& position);

    void input(Vamos_Geometry::Three_Vector const& torque,
               Vamos_Geometry::Three_Vector const& radius);
};

/// The suspension component for a wheel.
class Suspension : public Particle
{
public:
    Suspension(Vamos_Geometry::Three_Vector const& position,
               Vamos_Geometry::Three_Vector const& center_of_translation, Side side_of_car,
               double spring_constant, double bounce, double rebound, double travel,
               double max_compression_velocity);
    virtual ~Suspension();

    /// Set the steering angle.
    void steer(double degree_angle);
    /// Set the camber angle.
    void camber(double degree_angle);
    /// Set the caster angle.
    void caster(double degree_angle);
    /// Set the toe angle.
    void toe(double degree_angle);

    std::shared_ptr<Hinge> get_hinge() const { return mp_hinge; }

    void input(Vamos_Geometry::Three_Vector const& normal);

    void set_torque(double wheel_torque);

    virtual void propagate(double time) override;
    virtual void reset() override;

    // Specify the suspension component that is attached to this one with an anti-roll
    // bar.  The anti-roll bar will have a spring constant of SPRING_CONSTANT.
    void anti_roll(std::shared_ptr<Suspension> other, double spring_constant);

    /// Displace this suspension component. Compression is positive.
    void displace(double distance);
    /// @return the current displacement.
    double get_displacement() const { return m_displacement; }
    // @return true if the suspension behaves as rigid.
    bool is_bottomed_out() const { return m_bottomed_out; }

    double current_camber(double normal_y) const;
    /// Set the 3D model for rendering the suspension.
    void set_model(std::string file_name, double scale,
                   Vamos_Geometry::Three_Vector const& translation,
                   Vamos_Geometry::Three_Vector const& rotation);
    /// Render the suspension.
    void draw();

private:
    void find_forces();
    // Return the suspension position for the current displacement.
    Vamos_Geometry::Three_Vector get_position() const;

    std::shared_ptr<Hinge> mp_hinge;
    Vamos_Geometry::Three_Vector m_radius;
    Vamos_Geometry::Three_Vector m_initial_radius;
    double m_radius_magnitude;
    double m_initial_z;
    double m_spring_constant;
    double m_bounce; ///< Damping for compression.
    double m_rebound; ///< Damping for decompression.
    double m_travel; ///< How far the suspension can be displaced before bottoming out.
    // The current displacement of the suspension. Compression is positive.
    double m_displacement{0.0};
    // The size of the last time step.
    double m_time_step{0.0};
    // How fast the suspension is being compressed.
    double m_compression_speed{0.0};
    // How fast the suspension can be compressed before the damper locks up.
    double m_max_compression_velocity;
    // true if the displacement has exceeded `m_travel', false otherwise.
    bool m_bottomed_out{false};
    // The spring constant for the anti-roll bar that connects this suspension component
    // with another.
    double m_anti_roll_k{0.0};
    // The suspension component that this one is connected to with an anti-roll bar.
    Suspension* m_anti_roll_suspension = nullptr;

    // The deflection of the tire in radians due to steering.
    double m_steer_angle{0.0};
    // The static camber angle in radians.  Positive camber is a rotation that moves the
    // tops of the tires away from the center of the car.  So the direction of the
    // rotation depends on the value of m_side.
    double m_camber{0.0};
    // The caster angle in radians.
    double m_caster{0.0};
    // The toe angle in radians.  Positive is toe-in.
    double m_toe{0.0};
    // The side of the car the suspension is on. This is used to choose the right
    // direction for camber, caster and toe adjustments.
    Side m_side;
    // The orientation of the wheel in the absence of steering and displacement.
    Vamos_Geometry::Three_Matrix m_static_orientation{1.0};
    Vamos_Geometry::Three_Vector m_normal{0.0, 0.0, 1.0};
    Vamos_Geometry::Three_Vector m_hinge_axis;
    std::vector<std::unique_ptr<Suspension_Model>> m_models;
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_SUSPENSION_H_INCLUDED
