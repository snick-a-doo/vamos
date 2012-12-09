//  Sphere_Sky.cc - Spherical image mapping for Caelum.
//
//  Copyright (C) 2003 Sam Varner
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
