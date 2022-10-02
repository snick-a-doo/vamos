//  Copyright (C) 2003-2022 Sam Varner
//
//  This file is part of Caelum.
//
//  Caelum is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Caelum is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Caelum.
//  If not, see <http://www.gnu.org/licenses/>.

#include "sky.h"

#include <GL/glut.h>
#include <SDL/SDL.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <numbers>

using namespace std::numbers;

Sky::Sky(const std::string& file, int width, int height, int divisions, Shape projetion)
    : m_image{file, true, true},
      m_width{width},
      m_height{height},
      m_divisions{divisions},
      m_projection{projetion}
{
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    // Repeat in the east-west direction.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    // Clamp in the north-south direction.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void Sky::key_press(int key)
{
    static auto constexpr r_inc{0.01};
    static auto constexpr z_inc{0.01};
    static auto constexpr m_inc{0.01};

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
    case 'p':
        m_projection = m_projection == Shape::sphere ? Shape::cylinder : Shape::sphere;
        break;
    case 'q':
    case 27:  // escape
        std::exit(0);
    }
    display();
}

void Sky::display() const
{
    // Clear the entire window.
    glViewport(0, 0, m_width * 4, m_height * 3);
    glClear(GL_COLOR_BUFFER_BIT);

    // Draw the front view.
    auto x{0};
    auto y{m_height};
    glViewport(x, y, m_width, m_height);

    // Set the camera.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90, 1.0, 0.01, 1000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 0.0,  // camera position
              1.0, 0.0, 0.0,  // target
              0.0, 0.0, 1.0); // up direction
    draw();

    // Draw the right view.
    x += m_width;
    glViewport(x, y, m_width, m_height);
    glRotated(90.0, 0.0, 0.0, 1.0);
    draw();

    // Draw the back view.
    x += m_width;
    glViewport(x, y, m_width, m_height);
    glRotated(90.0, 0.0, 0.0, 1.0);
    draw();

    // Draw the left view.
    x += m_width;
    glViewport(x, y, m_width, m_height);
    glRotated(90.0, 0.0, 0.0, 1.0);
    draw();

    // Draw the up view.
    x = 0;
    y = 0;
    glViewport(x, y, m_width, m_height);
    glRotated(90.0, 0.0, 0.0, 1.0);
    glRotated(90.0, 0.0, -1.0, 0.0);
    draw();

    // Draw the down view.
    y = m_height * 2;
    glViewport(x, y, m_width, m_height);
    glRotated(180.0, 0.0, -1.0, 0.0);
    draw();

    glFlush();
    SDL_GL_SwapBuffers();
}

void draw_cylinder(int divisions, double z_pos, double z_rot, double z_mag)
{
    // Plot a vertex and its texture coordinates.
    auto vertex = [&](double phi, double z) {
        auto tex_x{phi/2.0/pi + z_rot};
        auto tex_y{((z + 1.0)/2.0 + z_pos)/z_mag};
        glColor3d(tex_x, tex_y, -z);
        glTexCoord2d(tex_x, tex_y);
        glVertex3d(cos(phi), sin(phi), -z);
    };

    // Draw the cylinder.
    auto delta_phi{pi/divisions};
    auto delta_z{2.0/divisions};
    for (auto i{0}; i < divisions; ++i)
    {
        glBegin(GL_QUAD_STRIP);
        auto z{i*delta_z - 1.0}; // -1 to 1-Δz
        for (auto j{0}; j <= 2*divisions; ++j)
        {
            auto phi{j*delta_phi}; // 0 to 2π
            vertex(phi, z);
            vertex(phi, z + delta_z);
        }
        glEnd();
    }
}

void draw_sphere(int divisions, double z_pos, double z_rot, double z_mag)
{
    // Plot a vertex and its texture coordinates.
    auto vertex = [&](double phi, double theta) {
        auto tex_x{phi/2.0/pi + z_rot};
        auto tex_y{(theta/pi + z_pos)/z_mag};
        glTexCoord2d(tex_x, tex_y);
        glVertex3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
    };

    // Draw the globe.
    auto delta_angle{pi/divisions};
    for (auto i{0}; i < divisions; ++i)
    {
        glBegin(GL_QUAD_STRIP);
        auto theta{i*delta_angle};
        for (auto j{0}; j <= 2*divisions; ++j)
        {
            auto phi{j*delta_angle};
            vertex(phi, theta);
            vertex(phi, theta + delta_angle);
        }
        glEnd();
    }
}

void Sky::draw() const
{
    if (m_projection == Shape::sphere)
        draw_sphere(m_divisions, m_z_pos, m_z_rot, m_z_mag);
    else
        draw_cylinder(m_divisions, m_z_pos, m_z_rot, m_z_mag);
}
