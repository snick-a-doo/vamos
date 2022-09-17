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

#include "spline.h"
#include "numeric.h"

#include <cassert>
#include <cmath>
#include <vector>

using namespace Vamos_Geometry;

Spline::Spline(std::optional<double> first_slope, std::optional<double> last_slope)
    : Spline({}, first_slope, last_slope)
{
}

Spline::Spline(std::vector<Two_Vector> const& points,
               std::optional<double> first_slope,
               std::optional<double> last_slope)
    : Interpolator{points},
      mo_first_slope{first_slope},
      mo_last_slope{last_slope}
{
}

void Spline::set_periodic(double end)
{
    load({end, m_points.empty() ? 0.0 : m_points.front().y});
    m_periodic = true;
}

// calculate() and interpolate() follow the discussion on cubic splines found in Numerical
// Recipes. The implementation here is original.

double Spline::interpolate(double x) const
{
    // Save the state of m_changed before it's reset by base interpolate().
    auto changed{m_changed};
    Interpolator::interpolate(x);

    size_t const n{m_points.size()};

    if (n < 2 || (n == 2 && m_periodic))
    {
        m_slope = m_periodic || !mo_first_slope ? 0.0 : *mo_first_slope;
        auto p0{m_points.empty() ? Two_Vector{0.0, 0.0} : m_points[0]};
        return p0.y + m_slope * (x - p0.x);
    }
    if (n == 2 && (!mo_first_slope || !mo_last_slope))
    {
        // Fall back to linear interpolation.
        m_slope = (m_points[1].y - m_points[0].y) / (m_points[1].x - m_points[0].x);
        return Vamos_Geometry::interpolate(x, m_points[0].x, m_points[0].y,
                                           m_points[1].x, m_points[1].y);
    }

    if (m_periodic)
        x = wrap(x, m_points.front().x, m_points.back().x);

    // calculate() only needs to be called once for a given set of points.
    if (changed)
        calculate();

    auto const low{low_index(x)};
    auto const high{low + 1};
    auto const diff{m_points[high].x - m_points[low].x};

    // Evaluate the coefficients for the cubic spline equation.
    auto const a{(m_points[high].x - x) / diff};
    auto const b{1.0 - a};
    auto const sq{diff * diff / 6.0};
    auto const a2{a * a};
    auto const b2{b * b};

    // Find the first derivative.
    m_slope = (m_points[high].y - m_points[low].y) / diff
              - (3.0 * a2 - 1.0) / 6.0 * diff * m_second_deriv[low]
              + (3.0 * b2 - 1.0) / 6.0 * diff * m_second_deriv[high];

    // Return the interpolated value.
    return a * (m_points[low].y + (a2 - 1.0) * sq * m_second_deriv[low])
        + b * (m_points[high].y + (b2 - 1.0) * sq * m_second_deriv[high]);
}

double Spline::slope(double x) const
{
    // The slope is calculated and stored when interpolate() is called.
    interpolate(x);
    return m_slope;
}

Two_Vector Spline::normal(double x) const
{
    interpolate(x);
    auto theta{std::atan(m_slope)};
    return {-std::sin(theta), std::cos(theta)};
}

std::vector<double> solve_symmetric_tridiagonal(std::vector<double> const& a,
                                                std::vector<double> const& b_in,
                                                std::vector<double> const& r_in)
{
    auto const n{a.size()};
    assert(n > 0);
    std::vector<double> b(n);
    std::vector<double> r(n);

    // Gauss-Jordan Elimination
    b[0] = b_in[0];
    r[0] = r_in[0];
    for (size_t i{1}; i < n; ++i)
    {
        // Replace row i with row i - k * row (i-1) such that A_{i,i-1} = 0.0.
        auto factor{a[i - 1] / b[i - 1]};
        // A_{i,i-1} is not used again, so it need not be calculated.
        b[i] = b_in[i] - factor * a[i - 1];
        // A_{i,i+1} is unchanged because A_{i-1,i+1} = 0.0.
        r[i] = r_in[i] - factor * r[i - 1];
    }
    // Back-Substitution
    std::vector<double> x(n);
    x[n - 1] = r[n - 1] / b[n - 1];
    // Test incremented index j to avoid underflow and narrowing.
    for (size_t i{n - 2}, j{0}; j < n - 1; --i, ++j)
        // Use the solution for x[i+1] to find x[i].
        x[i] = (r[i] - a[i] * x[i + 1]) / b[i];
    return x;
}

void Spline::calculate() const
{
    auto n{m_points.size()};
    if (n < 2)
        return;

    if (n == 2 && mo_first_slope && mo_last_slope)
    {
        auto delta{m_points[1] - m_points[0]};
        auto m3{3.0 * delta.y / delta.x};
        auto a{delta.x / 2.0};
        m_second_deriv = {-(2.0 * *mo_first_slope + *mo_last_slope - m3) / a,
                          (*mo_first_slope + 2.0 * *mo_last_slope - m3) / a};
        return;
    }

    std::vector<double> a(n - 2);
    std::vector<double> b(n - 2);
    std::vector<double> r(n - 2);
    std::vector<double> x(n - 2);
    m_second_deriv.resize(n);

    for (size_t i{0}; i < n - 2; ++i)
    {
        auto diff_low{m_points[i + 1].x - m_points[i].x};
        auto diff_high{m_points[i + 2].x - m_points[i + 1].x};
        a[i] = diff_high / 6.0;
        b[i] = (diff_low + diff_high) / 3.0;
        r[i] = (m_points[i + 2].y - m_points[i + 1].y) / diff_high
               - (m_points[i + 1].y - m_points[i].y) / diff_low;
    }

    if (m_periodic)
    {
        auto diff_low{m_points[n - 1].x - m_points[n - 2].x};
        auto diff_high{m_points[1].x - m_points[0].x};
        a.push_back(diff_high / 6.0);
        b.push_back((diff_low + diff_high) / 3.0);
        r.push_back((m_points[1].y - m_points[0].y) / diff_high
                    - (m_points[n - 1].y - m_points[n - 2].y) / diff_low);

        const auto alpha{a.back()};
        const auto gamma{-b.front()};
        b.front() -= gamma;
        b.back() -= alpha * alpha / gamma;

        x = solve_symmetric_tridiagonal(a, b, r);

        std::vector<double> u(n - 1, 0.0);
        u.front() = gamma;
        u.back() = alpha;

        std::vector<double> z{solve_symmetric_tridiagonal(a, b, u)};

        auto factor{(x[0] + x[n - 2] * alpha / gamma)
                    / (1.0 + z[0] + z[n - 2] * alpha / gamma)};
        for (size_t i{1}; i < n; ++i)
            m_second_deriv[i] = x[i - 1] - factor * z[i - 1];
        m_second_deriv.front() = m_second_deriv.back();
    }
    else
    {
        x = solve_symmetric_tridiagonal(a, b, r);

        m_second_deriv.front() = 0.0;
        for (size_t i{1}; i < n - 1; ++i)
            m_second_deriv[i] = x[i - 1];
        m_second_deriv.back() = 0.0;

        if (mo_first_slope)
        {
            auto dy{m_points[1].y - m_points[0].y};
            auto dx{m_points[1].x - m_points[0].x};
            m_second_deriv.front()
                = 3.0 / dx * (dy / dx - *mo_first_slope - dx / 6.0 * m_second_deriv[1]);
        }
        if (mo_last_slope)
        {
            auto dy{m_points[n - 1].y - m_points[n - 2].y};
            auto dx{m_points[n - 1].x - m_points[n - 2].x};
            m_second_deriv.back()
                = -3.0 / dx * (dy / dx - *mo_last_slope + dx / 6.0 * m_second_deriv[n - 2]);
        }
    }
}

//-----------------------------------------------------------------------------
Vector_Spline::Vector_Spline(std::optional<double> first_x_slope,
                             std::optional<double> last_x_slope,
                             std::optional<double> first_y_slope,
                             std::optional<double> last_y_slope,
                             std::optional<double> first_z_slope,
                             std::optional<double> last_z_slope)
    : m_x{first_x_slope, last_x_slope},
      m_y{first_y_slope, last_y_slope},
      m_z{first_z_slope, last_z_slope}
{
}

void Vector_Spline::load(double t, Three_Vector const& point)
{
    m_x.load({t, point.x});
    m_y.load({t, point.y});
    m_z.load({t, point.z});
}

void Vector_Spline::clear()
{
    m_x.clear();
    m_y.clear();
    m_z.clear();
}

void Vector_Spline::set_periodic(double end)
{
    m_x.set_periodic(end);
    m_y.set_periodic(end);
    m_z.set_periodic(end);
}

Three_Vector Vector_Spline::interpolate(double t) const
{
    return {m_x.interpolate(t), m_y.interpolate(t), m_z.interpolate(t)};
}

size_t Vector_Spline::size() const
{
    assert(m_x.size() == m_y.size());
    assert(m_x.size() == m_z.size());
    return m_x.size();
}

Three_Vector Vector_Spline::operator[](size_t i) const
{
    return {m_x[i].y, m_y[i].y, m_z[i].y};
}
