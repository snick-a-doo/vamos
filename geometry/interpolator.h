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

#ifndef VAMOS_GEOMETRY_INTERPOLATOR_H_INCLUDED
#define VAMOS_GEOMETRY_INTERPOLATOR_H_INCLUDED

#include "two-vector.h"

#include <optional>
#include <vector>

namespace Vamos_Geometry
{
class Interpolator
{
public:
    /// Construct an empty curve.
    Interpolator();
    /// Construct a cuvre from an array of points.
    Interpolator(std::vector<Point<double>> const& points);
    virtual ~Interpolator();

    /// Add a point to the curve.
    virtual void load(Point<double> const& point);
    virtual void load(double x, double y) { load({x, y}); }
    /// Add multiple points to the curve.
    virtual void load(std::vector<Point<double>> const& points);
    /// Replace all points on the curve.
    virtual void replace(std::vector<Point<double>> const& points);
    /// Remove all points from the curve.
    virtual void clear();
    /// Scale the x values.
    virtual void scale(double factor);
    /// Add an offset to the y values.
    virtual void shift(double delta);

    /// @return the y value at the given x.
    virtual double interpolate(double x) const;
    /// @return the normal to the tangent at x.
    virtual Two_Vector normal(double x) const = 0;

    /// Container-like interface
    /// @{
    size_t size() const { return m_points.size(); }
    bool empty() const { return m_points.empty(); }
    Point<double>& operator [] (size_t i) { return m_points[i]; }
    Point<double> const& operator [] (size_t i) const { return m_points[i]; }
    /// @}

protected:
    /// @return The
    size_t low_index(double x) const;
    std::vector<Point<double>> m_points; ///< The array of points to interpolate between.
    /// True if the control points have changed since the last interpolation.
    /// The index of the nearest control point whose x-value is <= the last interpolate()
    /// argument.
    bool mutable m_changed{true};

private:
    struct Cache
    {
        size_t index{0};
        int mutable checks{0}; ///< Number of interpolations done.
        int mutable hits{0}; ///< Number of times the interval was reused.
    };
    Cache mutable m_cache;
};
}

#endif // VAMOS_GEOMETRY_INTERPOLATOR_H_INCLUDED
