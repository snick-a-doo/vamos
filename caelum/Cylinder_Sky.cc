//  Cylinder_Sky.cc - Cylindrical image mapping for Caelum.
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

#include <cassert>

#include <GL/gl.h>

#include "../geometry/Constants.h"

#include "Cylinder_Sky.h"

using Vamos_Geometry::pi;

Cylinder_Sky::Cylinder_Sky (int divisions, std::string image, 
							int width, int height) :
  Sky (image, width, height),
  m_z_divisions (divisions)
{
  assert (m_z_divisions > 1);
}

void
Cylinder_Sky::draw ()
{
  // Draw the cylinder.
  for (int i = 0; i < m_z_divisions; i++)
	{
	  glBegin (GL_QUAD_STRIP);
	  for (int j = 0; j <= m_z_divisions * 2; j++)
		{
		  double tex_x = j / (m_z_divisions * 2.0);
		  double x = cos (j * pi / m_z_divisions);
		  double y = sin (j * pi / m_z_divisions);
		  double tex_z = 1.0 - double (i) / m_z_divisions;
		  double z = (2.0 * i) / m_z_divisions - 1.0;

		  z = (z + z_pos ()) / z_mag ();
		  tex_x += z_rot ();

		  glTexCoord2d (tex_x, tex_z);
		  glVertex3d (x, y, z);

		  tex_z = 1.0 - (i + 1.0) / m_z_divisions;
		  z = (2.0 * (i + 1.0)) / m_z_divisions - 1.0;
		  z = (z + z_pos ()) / z_mag ();

		  glTexCoord2d (tex_x, tex_z);
		  glVertex3d (x, y, z);
		}
	  glEnd ();
	}
}
