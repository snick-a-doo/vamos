//  Conversions.h - unit conversions.
//
//	Vamos Automotive Simulator
//  Copyright (C) 2001 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef _CONVERSIONS_H_
#define _CONVERSIONS_H_

#include "Constants.h"

namespace Vamos_Geometry
{
  template <typename T> T rad_to_deg (T rad) { return rad * (180.0 / pi); }
  template <typename T> T deg_to_rad (T deg) { return deg * (pi / 180.0); }

  inline double rad_s_to_rpm (double rad_s) { return rad_s * 30.0 / pi; }
  inline double rpm_to_rad_s (double rpm) { return rpm * pi / 30.0; }

  inline double m_s_to_km_h (double m_s) { return m_s * 3.6; } 
  inline double km_h_to_m_s (double km_h) { return km_h / 3.6; } 
}

#endif
