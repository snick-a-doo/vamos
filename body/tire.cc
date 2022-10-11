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

#include "tire.h"

#include "../geometry/conversions.h"
#include "../geometry/numeric.h"
#include "../geometry/parameter.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

auto constexpr epsilon{1.0e-12};
auto constexpr ambient_temperature{300.0};

namespace Vamos_Body
{
std::pair<double, double> get_slip(double patch_speed, Three_Vector const& hub_velocity)
{
    // Limit the denominator to keep ratios from getting out of hand at low speeds.
    auto denom{std::max(std::abs(hub_velocity.x), 3.0)};
    return {100.0 * (patch_speed - hub_velocity.x) / denom,
            -rad_to_deg(atan2(hub_velocity.y, denom))};
}
}

/// The magic equation.  Slip is a percentage for longitudinal force, an angle in degrees
/// for transverse force and aligning torque.
static inline double pacejka_equation(double slip, double B, double C, double D, double E,
                                      double Sh, double Sv)
{
    return D * sin(C * atan(B * (1.0 - E) * (slip + Sh) + E * atan(B * (slip + Sh)))) + Sv;
}

/// Return the slip value for which the Pacejka function is maximized.
static double peak_slip(double B, double C, double E, double Sh, double guess)
{
    // The Pacejka function is maximized when this function is zero...
    auto function = [](double x, double B, double C, double E, double Sh) {
        return B * (1.0 - E) * (x + Sh) + E * atan(B * (x + Sh)) - tan(pi / (2.0 * C));
    };
    auto derivative = [](double x, double B, double E, double Sh) {
        return B * (1.0 - E) + E * B / (1.0 + B * B * (x + Sh) * (x + Sh));
    };

    // Newton's method. The shape of the function makes it converge quickly.
    auto x{guess};
    for (auto i{0}; i < 10; ++i)
    {
        auto y{function(x, B, C, E, Sh)};
        if (std::abs(y) < 0.001)
            return x;
        x -= y / derivative(x, B, E, Sh);
    }
    //   std::cerr << "peak_slip() failed x=" << x << " y=" << y <<
    //     " B=" << B << " C=" << C << " E=" << E << " Sh=" << Sh << std::endl;
    return guess;
}

//----------------------------------------------------------------------------------------
Tire_Friction::Tire_Friction(Longi_Params const& long_parameters,
                             Trans_Params const& trans_parameters,
                             Align_Params const& align_parameters)
    : m_longitudital_parameters(long_parameters),
      m_transverse_parameters(trans_parameters),
      m_aligning_parameters(align_parameters)
{
}

Three_Vector Tire_Friction::friction_forces(double normal_force, double friction_factor,
                                            Three_Vector const& hub_velocity,
                                            double patch_speed, double current_camber)
{
    auto Fz{1.0e-3 * normal_force};
    auto Fz2{Fz * Fz};

    // Evaluate the longitudinal parameters.
    const auto& b{m_longitudital_parameters};
    auto Cx{b[0]};
    auto Dx{friction_factor * (b[1] * Fz2 + b[2] * Fz)};
    auto Bx{(b[3] * Fz2 + b[4] * Fz) * exp(-b[5] * Fz) / (Cx * Dx)};
    auto Ex{b[6] * Fz2 + b[7] * Fz + b[8]};
    auto Shx{b[9] * Fz + b[10]};

    // Evaluate the transverse parameters.
    const auto& a{m_transverse_parameters};
    auto gamma{rad_to_deg(current_camber)};
    auto Cy{a[0]};
    auto Dy{friction_factor * (a[1] * Fz2 + a[2] * Fz)};
    auto By{a[3] * sin(2.0 * atan(Fz / a[4])) * (1.0 - a[5] * std::abs(gamma)) / (Cy * Dy)};
    auto Ey{a[6] * Fz + a[7]};
    auto Shy{a[8] * gamma + a[9] * Fz + a[10]};
    auto Svy{(a[11] * Fz + a[12]) * gamma * Fz + a[13] * Fz + a[14]};

    // Evaluate the aligning parameters.
    const auto& c{m_aligning_parameters};
    auto Cz{c[0]};
    auto Dz{friction_factor * (c[1] * Fz2 + c[2] * Fz)};
    auto Bz{(c[3] * Fz2 + c[4] * Fz) * (1.0 - c[6] * std::abs(gamma)) * exp(-c[5] * Fz)
            / (Cz * Dz)};
    auto Ez{(c[7] * Fz2 + c[8] * Fz + c[9]) * (1.0 - c[10] * std::abs(gamma))};
    auto Shz{c[11] * gamma + c[12] * Fz + c[13]};
    auto Svz{(c[14] * Fz2 + c[15] * Fz) * gamma + c[16] * Fz + c[17]};

    // Use the previous peak as the guess.
    m_peak_slip = peak_slip(Bx, Cx, Ex, Shx, m_peak_slip);
    m_peak_slip_angle = peak_slip(By, Cy, Ey, Shy, m_peak_slip_angle);
    m_peak_align_angle = peak_slip(Bz, Cz, Ez, Shz, m_peak_align_angle);

    auto [sigma, alpha] = get_slip(patch_speed, hub_velocity);
    Three_Vector slip{m_peak_slip > epsilon ? slip.x = sigma / m_peak_slip : 0.0,
                      m_peak_slip_angle > epsilon ? slip.y = alpha / m_peak_slip_angle : 0.0,
                      m_peak_align_angle > epsilon ? slip.z = alpha / m_peak_align_angle : 0.0};

    m_total_slip = Three_Vector(slip.x, slip.y, 0.0).magnitude();
    auto Fx{pacejka_equation(sign(slip.x) * m_total_slip * m_peak_slip, Bx, Cx, Dx, Ex, Shx, 0.0)};
    auto Fy{pacejka_equation(sign(slip.y) * m_total_slip
                             * m_peak_slip_angle, By, Cy, Dy, Ey, Shy, Svy)};
    auto slip_xz{Three_Vector(slip.x, 0.0, slip.z).magnitude()};
    auto Mz{pacejka_equation(slip_xz * m_peak_align_angle, Bz, Cz, Dz, Ez, Shz, Svz)};

    if (m_total_slip > epsilon)
    {
        Fx *= std::abs(slip.x) / m_total_slip;
        Fy *= std::abs(slip.y) / m_total_slip;
    }
    if (slip_xz > epsilon)
        Mz *= slip.z / slip_xz;
    return {Fx, Fy, Mz};
}

//----------------------------------------------------------------------------------------
Vamos_Body::Tire::Tire(double radius, double rolling_resistance_1, double rolling_resistance_2,
                       Tire_Friction const& friction, double hardness, double inertia)
    : m_radius{radius},
      m_rolling_resistance{rolling_resistance_1, rolling_resistance_2},
      m_tire_friction{friction},
      m_inertia{inertia},
      m_hardness{hardness}
{
}

void Tire::input(Three_Vector const& velocity, double normal_angular_velocity,
                 Three_Vector const& normal_force, double camber, double torque,
                 bool is_locked, Material const& surface_material)
{
    // Return the axis and angle to rotate about to make the z-axis coincide with the
    // passed-in unit vector
    auto orient = [](Three_Vector const& unit) {
        auto axis{Three_Vector(-unit.y, unit.x, 0.0)};
        auto angle{asin(axis.magnitude())};
        return angle * axis.unit();
    };
    // Orient the tire's z-axis with the normal force.
    set_orientation(Three_Matrix{1.0});
    rotate(orient(normal_force.unit()));

    m_velocity = rotate_in(velocity);
    m_normal_angular_velocity = normal_angular_velocity;
    m_normal_force = normal_force.magnitude();
    m_camber = camber;
    m_applied_torque = torque;
    m_is_locked = is_locked;
    m_surface_material = surface_material;
}

void Tire::find_forces()
{
    // m_force is the force of the road on the tire.  The force of the tire on the body
    // must be calculated.  The transverse component won't change, but the longitudial
    // component will be affected by the tire's ability to move in that direction, and by
    // applied foces (acceleration and braking).

    // Skip this step if we don't have a surface yet.
    if (m_surface_material.composition() == Material::UNKNOWN)
        return;

    m_slide = 0.0;
    if (m_normal_force <= 0.0)
    {
        Particle::reset();
        return;
    }

    // Get the friction force (components 0 and 1) and the aligning torque (component 2).
    auto friction_force{m_tire_friction.friction_forces(
            m_normal_force * grip(), m_surface_material.friction_factor(),
            m_velocity, speed(), m_camber)};

    // the frictional force opposing the motion of the contact patch.
    set_force({friction_force.x, friction_force.y, 0.0});

    // Apply the reaction torque from acceleration or braking.  In normal conditions this
    // torque comes from friction.  However, the frictional force can sometimes be large
    // when no acceleration or braking is applied.  For instance, when you run into a
    // gravel trap.  In any case, the reaction torque should never be larger than the
    // applied accelerating or braking torque.
    auto reaction{force().x * m_radius};
    if ((m_applied_torque > 0.0 && reaction > m_applied_torque)
        || (m_applied_torque < 0.0 && reaction < m_applied_torque))
        reaction = m_applied_torque;

    set_torque(Three_Vector(0.0, reaction, friction_force.z));

    if (!m_is_locked)
    {
        auto [roll1, roll2] = m_rolling_resistance;
        if (speed() < 0.0)
            roll1 *= -1.0;
        // Include constant and quadratic rolling resistance.
        auto rolling{m_surface_material.rolling_resistance_factor()
                     * (roll1 + roll2 * speed() * speed())};
        m_applied_torque -= (force().x + rolling) * m_radius;
    }

    // Add the suface drag.
    set_force(force()
              - m_surface_material.drag_factor() * Three_Vector(m_velocity.x, m_velocity.y, 0.0));

    m_slide = m_tire_friction.total_slip();
}

void Tire::propagate(double time)
{
    find_forces();
    m_rotational_speed
        = m_is_locked ? 0.0 : m_rotational_speed + m_applied_torque * time / m_inertia;

    // My made-up model of tire heating and wear.
    static auto constexpr stress_heating{2e-4};
    static auto constexpr abrasion_heating{1e-1};
    static auto constexpr wear{1e-8};
    static auto constexpr cooling{5e-3};

    const auto dT{m_temperature - ambient_temperature};
    if (m_slide > 0.0)
    {
        // Forces applied through the tire flex, stretch, and compress the rubber
        // heating it up. Slipping results in heating due to sliding friction.
        auto F{(force() + m_normal_force * z_hat).magnitude()};
        auto friction{m_slide * m_surface_material.friction_factor()};
        auto heat{time * (stress_heating * F / m_hardness + abrasion_heating * friction)};
        m_temperature += heat;

        // Slipping wears the tire through abrasion. Tires wear more quickly at high
        // temperature.
        m_wear += time * wear * friction * pow(dT, 2);
    }
    // Cooling is proportional to difference from ambient.
    m_temperature -= time * cooling * dT;
}

double Tire::grip() const
{
    return std::max(m_temperature / 380.0 - m_wear, 0.3);
}

Two_Vector Tire::slip() const
{
    auto [x, y] = get_slip(speed(), m_velocity);
    return{x, y};
}

Three_Vector Tire::contact_position() const
{
    return {0.0, 0.0, -m_radius};
}

// Set the tire to its initial conditions.
void Tire::reset()
{
    Particle::reset();
    m_rotational_speed = 0.0;
}
