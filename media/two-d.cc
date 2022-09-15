//  Copyright (C) 2013-2022 Sam Varner
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

#include "two-d.h"

#include "../geometry/rectangle.h"
#include "../geometry/two-vector.h"

#include <GL/glut.h>

using namespace Vamos_Geometry;
using namespace Vamos_Media;

Two_D::Two_D()
{
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    m_width = viewport[2] - viewport[0];
    m_height = viewport[3] - viewport[1];

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    gluOrtho2D(0, m_width, 0, m_height);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
}

Two_D::~Two_D()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void Two_D::text(Two_Vector p, std::string const& str)
{
    p.x *= m_width / 100.0;
    p.y *= m_height / 100.0;
    glColor3f(1.0, 1.0, 1.0);
    glRasterPos2d(p.x, p.y);
    for (auto c : str)
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
}

void Two_D::bar(Vamos_Geometry::Rectangle const& box, Color const& color, double fraction)
{
    auto x1{box.left() * m_width / 100};
    auto y1{box.top() * m_height / 100};
    auto x2{box.right() * m_width / 100};
    auto y2{box.bottom() * m_height / 100};

    glColor3f(color.red, color.green, color.blue);

    // border
    glBegin(GL_LINE_LOOP);
    glVertex2f(x1, y2);
    glVertex2f(x1, y1);
    glVertex2f(x2, y1);
    glVertex2f(x2, y2);
    glEnd();

    // fill
    glBegin(GL_QUADS);
    auto top{y2 + fraction * (y1 - y2)};
    glVertex2f(x1, y2);
    glVertex2f(x1, top);
    glVertex2f(x2, top);
    glVertex2f(x2, y2);
    glEnd();
}

void Two_D::lights(Two_Vector p, double r, int n, int n_on,
                   Color const& on, Color const& off)
{
    p.x *= m_width / 100.0;
    p.y *= m_height / 100.0;
    r *= m_height / 100.0;

    glPushMatrix();
    glTranslatef(p.x + 6 * r, p.y, 0.0);
    auto* disk = gluNewQuadric();
    gluQuadricDrawStyle(disk, GLU_FILL);
    glColor3f(off.red, off.green, off.blue);
    for (auto light{1}; light <= n; ++light)
    {
        if (light == n_on)
            glColor3f(on.red, on.green, on.blue);
        glTranslatef(-2.5 * r, 0.0, 0.0);
        gluDisk(disk, 0.0, r, 32, 32);
    }

    gluDeleteQuadric(disk);
    glPopMatrix();
}

void Two_D::vector(Two_Vector p, double r, Color const& ring_color,
                   Color const& dot_color, Two_Vector const& v)
{
    p.x *= m_width / 100.0;
    p.y *= m_height / 100.0;
    r *= m_height / 100.0;

    glPushMatrix();
    glTranslatef(p.x + r, p.y, 0.0);
    {
        // Draw the indicator under the scale.
        glPushMatrix();
        auto* dot{gluNewQuadric()};
        gluQuadricDrawStyle(dot, GLU_FILL);
        glColor3f(dot_color.red, dot_color.green, dot_color.blue);
        glTranslatef(r * v.x, r * v.y, 0.0);
        gluDisk(dot, 0.0, 0.1 * r, 32, 1);
        gluDeleteQuadric(dot);
        glPopMatrix();
    }

    // Draw the rings.
    auto* circle = gluNewQuadric();
    gluQuadricDrawStyle(circle, GLU_LINE);
    glColor3f(ring_color.red, ring_color.green, ring_color.blue);
    gluDisk(circle, r, r, 32, 1);
    gluDisk(circle, r / 2, r / 2, 32, 1);
    gluDeleteQuadric(circle);

    // Draw the axes.
    glBegin(GL_LINES);
    glVertex3d(-r, 0, 0);
    glVertex3d(r, 0, 0);
    glVertex3d(0, -r, 0);
    glVertex3d(0, r, 0);
    glEnd();

    glPopMatrix();
}
