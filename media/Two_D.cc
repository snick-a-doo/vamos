//  2D - Convenience functions for 2D OpenGl rendering. 
//
//	Vamos Automotive Simulator
//  Copyright (C) 2013 Sam Varner
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

#include "Two_D.h"

#include <GL/glut.h>

using namespace Vamos_Media;

Two_D::Two_D (int width, int height)
{
  glMatrixMode (GL_PROJECTION);
  glPushMatrix ();
  glLoadIdentity ();

  gluOrtho2D (0, width, 0, height);
  glMatrixMode (GL_MODELVIEW);
  glPushMatrix ();
  glLoadIdentity ();
  
  glDisable (GL_DEPTH_TEST);
  glDisable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);
}

Two_D::~Two_D ()
{
  glEnable (GL_DEPTH_TEST);
  glEnable (GL_LIGHTING);
  glEnable (GL_TEXTURE_2D);

  glMatrixMode (GL_PROJECTION);
  glPopMatrix ();
  glMatrixMode (GL_MODELVIEW);
  glPopMatrix ();
}

void Two_D::draw_string (const std::string& str, double x, double y)
{
  glColor3f (1.0, 1.0, 1.0);
  glRasterPos2d (x, y);
  for (std::string::const_iterator it = str.begin (); it != str.end (); ++it)
	{
	  glutBitmapCharacter (GLUT_BITMAP_8_BY_13, *it);
	}
}

void Two_D::bar (const Vamos_Geometry::Rectangle& box, 
                 double red, double green, double blue,
                 double fraction)
{
  glColor3f (red, green, blue);

  // border
  glBegin (GL_LINE_LOOP);
  glVertex2f (box.left (), box.bottom ());
  glVertex2f (box.left (), box.top ());
  glVertex2f (box.right (), box.top ());
  glVertex2f (box.right (), box.bottom ());
  glEnd ();

  // fill
  glBegin (GL_QUADS);
  const double top = fraction * box.height () + box.bottom ();
  glVertex2f (box.left (), box.bottom ());
  glVertex2f (box.left (), top);
  glVertex2f (box.right (), top);
  glVertex2f (box.right (), box.bottom ());
  glEnd ();
}

void Two_D::lights (double x, double y, double r, int n, int n_on,
                    double red_on, double green_on, double blue_on,
                    double red_off, double green_off, double blue_off)
{
  glTranslatef (x + 6*r, y, 0.0);

  GLUquadricObj* disk = gluNewQuadric();
  gluQuadricDrawStyle (disk, GLU_FILL);

  glColor3f (red_off, green_off, blue_off);
  for (int light = 1; light <= n; light++)
    {
      if (light >= n_on)
        glColor3f (red_on, green_on, blue_on);
      glTranslatef (-2.5*r, 0.0, 0.0);
      gluDisk (disk, 0.0, r, 32, 32);
    }

  gluDeleteQuadric (disk);
}
