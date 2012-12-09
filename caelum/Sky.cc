//  Sky.cc - Helper class for Caelum.
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

#include "Sky.h"

#include <cassert>
#include <cstdlib>

#include <SDL/SDL.h>
#include <GL/glut.h>

#include "../geometry/Constants.h"
#include "../media/Texture_Image.h"

Sky::Sky (std::string image, int width, int height) : 
  m_width (width),
  m_height (height),
  m_z_pos (0.0),
  m_z_mag (1.0),
  m_z_rot (0.0)
{
  // Load the image.
  mp_image = new Vamos_Media::Texture_Image (image, true, true);

  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

  // Repeat in the east-west direction.
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  // Clamp in the north-south direction.
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

Sky::~Sky ()
{
  delete mp_image;
}

void
Sky::key_press (int key)
{
  static const double r_inc = 0.01;
  static const double z_inc = 0.01;
  static const double m_inc = 0.01;

  switch (key)
	{
	case SDLK_UP:
	  // Raise the image.
	  m_z_pos += z_inc;
	  break;
	case SDLK_DOWN:
	  // Lower the image.
	  m_z_pos -= z_inc;
	  break;
	case SDLK_RIGHT:
	  // Shift the image to the right.
	  m_z_rot += r_inc;
	  break;
	case SDLK_LEFT:
	  // Shift the image to the left.
	  m_z_rot -= r_inc;
	  break;
	case SDLK_PAGEUP:
	  // Magnify in the z-direction.
	  m_z_mag += m_inc;
	  break;
	case SDLK_PAGEDOWN:
	  // Shrink in the z-direction.
	  m_z_mag -= m_inc;
	  if (m_z_mag <= 0.0)
		m_z_mag = 1.0;
	  break;
	case 'q': // fall through
	case 27:  // escape
	  std::exit (0);
	}
  display ();
}

void
Sky::display ()
{
  // Clear the entire window.
  glViewport (0, 0, m_width * 4, m_height * 3);
  glClear (GL_COLOR_BUFFER_BIT);

  // Enable sub-windows.
  glEnable (GL_SCISSOR_TEST);

  // Draw the front view.
  int x = 0;
  int y = m_height;
  glViewport (x, y, m_width, m_height);
  glScissor (x, y, m_width, m_height);

  // Set the camera.
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective (90, 1.0, 0.1, 1.1);
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();
  gluLookAt (0.0, 0.0, 0.0, // camera position 
			 0.1, 0.0, 0.0,  // target 
			 0.0, 0.0, 0.1); // up direction
  draw ();

  // Draw the right view.
  x += m_width;
  glViewport (x, y, m_width, m_height);
  glScissor (x, y, m_width, m_height);
  glRotated (90.0, 0.0, 0.0, 1.0);
  draw ();

  // Draw the back view.
  x += m_width;
  glViewport (x, y, m_width, m_height);
  glScissor (x, y, m_width, m_height);
  glRotated (90.0, 0.0, 0.0, 1.0);
  draw ();

  // Draw the left view.
  x += m_width;
  glViewport (x, y, m_width, m_height);
  glScissor (x, y, m_width, m_height);
  glRotated (90.0, 0.0, 0.0, 1.0);
  draw ();

  // Draw the up view.
  x = 0;
  y = 0;
  glViewport (x, y, m_width, m_height);
  glScissor (x, y, m_width, m_height);
  glRotated (90.0, 0.0, 0.0, 1.0);
  glRotated (90.0, 0.0, -1.0, 0.0);
  draw ();

  // Draw the down view.
  y = m_height*2;
  glViewport (x, y, m_width, m_height);
  glScissor (x, y, m_width, m_height);
  glRotated (180.0, 0.0, -1.0, 0.0);
  draw ();

  glFlush ();
  SDL_GL_SwapBuffers ();
}
