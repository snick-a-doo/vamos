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

#ifndef _CYLINDER_SKY_H_
#define _CYLINDER_SKY_H_

#include <string>

#include "Sky.h"

class Cylinder_Sky : public Sky
{
public:
  // divisions - # of latitude divisions.
  // image - the name of the image file.
  // width - the width of each viewport.
  // height - the height of each viewport.
  Cylinder_Sky (int divisions, std::string image, int width, int height);

private:
  // Render a view.
  void draw ();

  int m_z_divisions;
};

#endif // not _CYLINDER_SKY_H_
