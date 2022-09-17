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

#ifndef VAMOS_BODY_TIRE_H_INCLUDED
#define VAMOS_BODY_TIRE_H_INCLUDED

#include "particle.h"

#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"
#include "../media/material.h"

#include <array>
#include <tuple>

namespace Vamos_Body
{
using Longi_Params = std::array<double, 11>;
using Trans_Params = std::array<double, 15>;
using Align_Params = std::array<double, 10>;

/// Return the longitudinal (first) and transverse (second) slip ratios.
std::pair<double, double> get_slip(double patch_speed,
                                   Vamos_Geometry::Three_Vector const& hub_velocity);

/// The frictional properties of the tire.
class Tire_Friction
{
public:
    Tire_Friction(Longi_Params const& longi_parameters,
                  Trans_Params const& trans_parameters,
                  Align_Params const& align_parameters);

    // Return the friction vector calculated from the magic formula.
    // HUB_VELOCITY is the velocity vector of the wheel's reference
    // frame.  PATCH_SPEED is the rearward speed of the contact pacth
    // with respect to the wheel's frame.
    Vamos_Geometry::Three_Vector friction_forces(double normal_force, double friction_factor,
                                                 const Vamos_Geometry::Three_Vector& hub_velocity,
                                                 double patch_speed, double current_camber);

    /// @return A measure of the overall amount of tire slip.
    double total_slip() const { return m_total_slip; }
    /// @return The slip ratio that gives maximum force.
    double peak_slip_ratio() const { return m_peak_slip; }
    /// @return The slip angle that gives maximum force.
    double peak_slip_angle() const { return m_peak_slip_angle; }

private:
    /// The parameters of the longitudinal magic equation.
    Longi_Params m_longitudital_parameters;
    /// The parameters of the transverse magic equation.
    Trans_Params m_transverse_parameters;
    /// The parameters of the magic equation for aligning torque.
    Align_Params m_aligning_parameters;
    /// The slip that gives the maximum longitudinal force.
    double m_peak_slip{0.0};
    /// The slip angle that gives the maximum transverse force.
    double m_peak_slip_angle{0.0};
    /// The slip angle that gives the maximum aligning torque.
    double m_peak_align_angle{0.0};
    /// The vector length of longitudital and lateral slip.
    double m_total_slip{0.0};
};

/// The tire for a wheel.
class Tire : public Particle
{
public:
    Tire(double radius, double rolling_resistance_1, double rolling_resistance_2,
         Tire_Friction const& friction, double hardness, double inertia);

    // Called by the wheel to update the tire.
    void input(Vamos_Geometry::Three_Vector const& velocity, double normal_angular_velocity,
               Vamos_Geometry::Three_Vector const& normal_force, double camber, double torque,
               bool is_locked, Vamos_Geometry::Material const& surface_material);

    void find_forces();

    // Advance the tire in time by TIME.
    void propagate(double time);

    void rewind();

    // Return the radius of the tire
    double radius() const { return m_radius; }

    // Return the rotational speed of the tire.
    double rotational_speed() const { return m_rotational_speed; }

    // Return the linear speed of the tread.
    double speed() const { return m_rotational_speed * m_radius; }

    // Fill in the longitudinal (sigma) and transverse (alpha) slip ratios as x and y.
    Vamos_Geometry::Two_Vector slip() const;

    double temperature() const { return m_temperature - 273.15; }
    /// A traction factor that depends on tire temperature and wear.
    double wear() const { return m_wear; }
    double grip() const;

    // Return the linear sliding speed.
    double slide() const { return m_slide; }

    // Return the slip ratio that gives maximum force.
    double peak_slip_ratio() const { return m_tire_friction.peak_slip_ratio(); }
    // Return the slip angle that gives maximum force.
    double peak_slip_angle() const { return m_tire_friction.peak_slip_angle(); }

    // Return the position of the contact patch in the wheel's
    // coordinate system.
    Vamos_Geometry::Three_Vector contact_position() const;

    // Return the normal force on this tire.
    double normal_force() const { return m_normal_force; }

    // Set the tire to its initial conditions.
    void reset();

private:
    /// The radius of the tire.
    double m_radius;
    /// Linear and quadratic rolling resistance on a hard surface.
    std::tuple<double, double> m_rolling_resistance;
    /// Object for calculating friction.
    Tire_Friction m_tire_friction;
    /// The rotational inertia of the tire.
    double m_inertia;
    /// The rotational speed of the tire in radians per second.
    double m_rotational_speed{0.0};
    /// How fast the tire is sliding.
    double m_slide{0.0};
    /// How much the tire heats up due to applied forces.
    double m_hardness;
    /// The temperature of the tire.
    double m_temperature{345.0};
    /// How much grip has been lost to tire wear.
    double m_wear{0.0};

    /// Input paremeters provided by the wheel.
    /// @{
    /// The velocity of the wheel relative to the road.
    Vamos_Geometry::Three_Vector m_velocity;
    /// The angular velocity about the normal to the surface.
    double m_normal_angular_velocity{0.0};
    /// The force perpendicular to the surface.
    double m_normal_force{0.0};
    /// The current camber, supplied by the wheel.
    double m_camber{0.0};
    /// The torque on the wheel due to acceleration or braking.
    double m_applied_torque{0.0};
    /// True if the brake for this wheel is locked.
    bool m_is_locked{false};
    /// The surface that the tire is currently on.
    Vamos_Geometry::Material m_surface_material;
    /// @}
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_TIRE_H_INCLUDED
