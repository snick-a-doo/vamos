//  Sphere_Sky.cc - Spherical image mapping for Caelum.
//
//  Copyright (C) 2003 Sam Varner
//
//  This file is part of Caelum.
//
//  Caelum is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  Caelum is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with Caelum.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _SPHERE_SKY_H_
#define _SPHERE_SKY_H_

#include <string>
#include <vector>

#include "sky.h"

class Sphere_Sky : public Sky
{
  // Pre-calculated angles.
  std::vector <double> m_theta;
  std::vector <double> m_phi;

  // Plot a vertex and its texture coordinates.
  void vertex (double x, double y, double z, double theta, double phi);

  // Render a view.
  void draw ();

public:
  // divisions - # of latitude divisions.
  // image - the name of the image file.
  // width - the width of each viewport.
  // height - the height of each viewport.
  Sphere_Sky (int divisions, std::string image, int width, int height);
  
  // Respond to keystrokes.
  void key_press (int key);
};

#endif // not _SPHERE_SKY_H_
