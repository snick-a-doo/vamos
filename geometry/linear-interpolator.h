//  Copyright (C) 2003-2022 Sam Varner
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

#ifndef VAMOS_GEOMETRY_LINEAR_INTERPOLATOR_H_INCLUDED
#define VAMOS_GEOMETRY_LINEAR_INTERPOLATOR_H_INCLUDED

#include "interpolator.h"
#include "two-vector.h"

#include <vector>

namespace Vamos_Geometry
{
class Linear_Interpolator : public Interpolator
{
public:
    /// Construct an empty curve.
    Linear_Interpolator();
    /// Construct a cuvre from an array of points.
    Linear_Interpolator(std::vector<Two_Vector> const& points);

    virtual double interpolate(double x) const override;
    /// Normals are discontinuous at the control points. If x is at a control point, use
    /// the lower interval.  x_i < x <= x_{i+1}.
    virtual Two_Vector normal(double x) const override;
};
} // namespace Vamos_Geometry

#endif // VAMOS_GEOMETRY_LINEAR_INTERPOLATOR_H_INCLUDED
