//  Two_D.cc - Convenience functions for 2D OpenGl rendering. 
//
//  Copyright (C) 2013 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//  
//  Vamos is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//  
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#include "two-d.h"

#include <GL/glut.h>

using namespace Vamos_Media;

Two_D::Two_D (int width, int height)
  : m_width (width),
    m_height (height)
{
  initialize ();
}

Two_D::Two_D ()
{
  GLint viewport [4];
  glGetIntegerv (GL_VIEWPORT, viewport);
  m_width = viewport [2] - viewport [0];
  m_height = viewport [3] - viewport [1];
  initialize ();
}

void Two_D::initialize ()
{
  glMatrixMode (GL_PROJECTION);
  glPushMatrix ();
  glLoadIdentity ();

  gluOrtho2D (0, m_width, 0, m_height);
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
  x *= m_width/100.0;
  y *= m_height/100.0;

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
  double x1 = box.left () * m_width/100;
  double y1 = box.top () * m_height/100;
  double x2 = box.right () * m_width/100;
  double y2 = box.bottom () * m_height/100;

  glColor3f (red, green, blue);

  // border
  glBegin (GL_LINE_LOOP);
  glVertex2f (x1, y2);
  glVertex2f (x1, y1);
  glVertex2f (x2, y1);
  glVertex2f (x2, y2);
  glEnd ();

  // fill
  glBegin (GL_QUADS);
  const double top = y2 + fraction * (y1 - y2);
  glVertex2f (x1, y2);
  glVertex2f (x1, top);
  glVertex2f (x2, top);
  glVertex2f (x2, y2);
  glEnd ();
}

void Two_D::lights (double x, double y, double r, int n, int n_on,
                    double red_on, double green_on, double blue_on,
                    double red_off, double green_off, double blue_off)
{
  x *= m_width/100.0;
  y *= m_height/100.0;
  r *= m_height/100.0;

  glPushMatrix();
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
  glPopMatrix();
}

void Two_D::vector (double x, double y, double r,
                    double red, double green, double blue,
                    double red_dot, double green_dot, double blue_dot,
                    const Vamos_Geometry::Two_Vector& v)
{
  x *= m_width/100.0;
  y *= m_height/100.0;
  r *= m_height/100.0;

  glPushMatrix ();
  glTranslatef (x + r, y, 0.0);

  {
    // Draw the indicator under the scale.
    glPushMatrix ();
    GLUquadricObj* dot = gluNewQuadric();
    gluQuadricDrawStyle (dot, GLU_FILL);
    glColor3f (red_dot, green_dot, blue_dot);
    glTranslatef (r*v.x, r*v.y, 0.0);
    gluDisk (dot, 0.0, 0.1*r, 32, 1);
    gluDeleteQuadric (dot);
    glPopMatrix ();
  }

  // Draw the rings.
  GLUquadricObj* circle = gluNewQuadric();
  gluQuadricDrawStyle (circle, GLU_LINE);
  glColor3f (red, green, blue);
  gluDisk (circle, r, r, 32, 1);
  gluDisk (circle, r/2, r/2, 32, 1);
  gluDeleteQuadric (circle);

  // Draw the axes.
  glBegin (GL_LINES);
  glVertex3d (-r, 0, 0);
  glVertex3d (r, 0, 0);
  glVertex3d (0, -r, 0);
  glVertex3d (0, r, 0);
  glEnd();

  glPopMatrix ();
}

