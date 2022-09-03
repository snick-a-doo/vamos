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

#include "sphere-sky.h"

#include <GL/gl.h>

#include <cassert>
#include <cmath>
#include <numbers>

using namespace std::numbers;

Sphere_Sky::Sphere_Sky (int divisions, std::string image,
						int width, int height) :
  Sky (image, width, height)
{
  assert (divisions > 1);

  // Calculate the angles of the vertices.
  m_theta.resize (divisions + 1);
  double delta (pi / divisions);
  for (size_t i = 0; i < m_theta.size (); i++)
	{
	  m_theta [i] = delta * i;
	}

  m_phi.resize (divisions * 2 + 1);
  for (size_t i = 0; i < m_phi.size (); i++)
	{
	  m_phi [i] = delta * i;
	}
}

void
Sphere_Sky::draw ()
{
  // Draw the globe.
  for (std::vector <double>::iterator theta_it = m_theta.begin ();
	   theta_it != m_theta.end () - 1;
	   theta_it++)
 	{
	  glBegin (GL_QUAD_STRIP);
	  for (std::vector <double>::iterator phi_it = m_phi.begin ();
		   phi_it != m_phi.end ();
		   phi_it++)
		{
		  double x = sin (*theta_it) * cos (*phi_it);
		  double y = sin (*theta_it) * sin (*phi_it);
		  double z = cos (*theta_it);
		  vertex (x, y, z, *theta_it, *phi_it);
		  
		  theta_it++;
		  x = sin (*theta_it) * cos (*phi_it);
		  y = sin (*theta_it) * sin (*phi_it);
		  z = cos (*theta_it);
		  vertex (x, y, z, *theta_it, *phi_it);
		  theta_it--;
		}
	  glEnd ();
	}
}

void
Sphere_Sky::vertex (double x, double y, double z, double theta, double phi)
{
  // Set the texture coordinates.
  double tex_x = phi / (2.0 * pi) + z_rot ();
  double tex_y = (theta / pi + z_pos ()) / z_mag ();

  glTexCoord2d (tex_x, tex_y);
  glVertex3d (x, y, z);
}