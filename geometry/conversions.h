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

/// @file Unit conversions.
#ifndef VAMOS_GEOMETRY_CONVERSIONS_H_INCLUDED
#define VAMOS_GEOMETRY_CONVERSIONS_H_INCLUDED

#include <numbers>

using namespace std::numbers;

namespace Vamos_Geometry
{
     constexpr double rad_to_deg(double rad) { return rad * (180.0 / pi); }
     constexpr double deg_to_rad(double deg) { return deg * (pi / 180.0); }

     constexpr double rad_s_to_rpm(double rad_s) { return rad_s * 30.0 / pi; }
     constexpr double rpm_to_rad_s(double rpm) { return rpm * pi / 30.0; }

     constexpr double m_s_to_km_h(double m_s) { return m_s * 3.6; }
     constexpr double km_h_to_m_s(double km_h) { return km_h / 3.6; }
}

#endif // VAMOS_GEOMETRY_CONVERSIONS_H_INCLUDED
