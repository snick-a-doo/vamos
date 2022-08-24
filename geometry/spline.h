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

#ifndef VAMOS_GEOMETRY_SPLINE_H_INCLUDED
#define VAMOS_GEOMETRY_SPLINE_H_INCLUDED

#include "interpolator.h"
#include "three-vector.h"
#include "two-vector.h"

#include <optional>
#include <vector>

namespace Vamos_Geometry
{
/// A smooth cubic spline interpolator.
class Spline : public Interpolator
{
public:
    /// Construct an empty curve. Slopes of the endpoints may be specified or left free.
    /// @param first_slope The slope at the beginning of the curve.
    /// @param last_slope The slope at the end of the curve.
    Spline(std::optional<double> first_slope = std::nullopt,
           std::optional<double> last_slope = std::nullopt);

    /// Construct a cuvre from an array of points. Slopes of the endpoints may be
    /// specified or left free.
    /// @param first_slope The slope at the beginning of the curve.
    /// @param last_slope The slope at the end of the curve.
    Spline(std::vector<Two_Vector> const& points,
           std::optional<double> first_slope = std::nullopt,
           std::optional<double> last_slope = std::nullopt);

    /// Make the spline repeat for x-values greater than @p end.
    void set_periodic(double end);

    /// @return the y value at x.
    virtual double interpolate(double x) const override;
    /// @return the first derivative at x.
    double slope(double x) const;
    /// @return the normal to the tangent at x.
    virtual Two_Vector normal(double x) const;

private:
    // Calculate the coefficients for interpolation.
    void calculate() const;

    /// The first derivative of the spline at the first point.
    std::optional<double> mo_first_slope;
    /// The first derivative of the spline at the last point.
    std::optional<double> mo_last_slope;
    /// True if the spline repeats for x-values outside the range of control points.
    bool m_periodic{false};
    /// The array of calculated second derivatives. Calculated once for a given set of
    /// control points. Used for each interpolation.
    mutable std::vector<double> m_second_deriv;
    // The 1st derivative at the interpolated point calculated during the last call to
    // interpolate().
    mutable double m_slope{0.0};

};

//----------------------------------------------------------------------------------------
/// A three-dimensional parametric spline. x, y, and z coordinates are found by spline
/// interpolation. This class is not derived from Interpolator and doesn't have exactly
/// the same interface.
class Vector_Spline
{
public:
    /// Construct an empty curve. Slopes of the endpoints may be specified or left free.
    Vector_Spline(std::optional<double> first_x_slope = std::nullopt,
                  std::optional<double> last_x_slope = std::nullopt,
                  std::optional<double> first_y_slope = std::nullopt,
                  std::optional<double> last_y_slope = std::nullopt,
                  std::optional<double> first_z_slope = std::nullopt,
                  std::optional<double> last_z_slope = std::nullopt);

    /// Add a point to the curve.
    void load(double t, Three_Vector const& point);
    /// Remove all points from the curve.
    void clear();
    /// Make the spline repeat for x-values greater than @p end.
    void set_periodic(double period_end);
    /// @return the point at the parameter value @p t.
    Three_Vector interpolate(double t) const;
    /// @return the number of control points.
    size_t size() const;
    /// @return A control point by index.
    Vamos_Geometry::Three_Vector operator[](size_t i) const;

private:
    Spline m_x;
    Spline m_y;
    Spline m_z;
};
} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_SPLINE_H_INCLUDED
