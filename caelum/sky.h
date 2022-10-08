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

#ifndef CAELUM_CAELUM_SKY_H_INCLUDED
#define CAELUM_CAELUM_SKY_H_INCLUDED

#include "../media/texture-image.h"

#include <memory>
#include <string>
#include <vector>

class SDL_Window;

namespace Vamos_Media
{
class Texture_Image;
}

class Sky
{
public:
    enum class Shape{sphere, cylinder};

    /// @param image The name of the image file.
    /// @param width The width of each viewport.
    /// @param height The height of each viewport.
    Sky(SDL_Window* window, const std::string& image_name,
        int width, int height, int divisions, Shape projetion);

    /// Render the scene.
    void display() const;
    /// Respond to input.
    void key_press(int key);

private:
    /// Render a view.
    void draw() const;

    SDL_Window* mp_window;
    Vamos_Media::Texture_Image const m_image;
    int const m_width; ///< Viewport width.
    int const m_height; ///< Viewport height
    int const m_divisions; ///< The number of latitude divisions.

    Shape m_projection; ///< The shape the image is projected onto.
    double m_z_pos{0.0}; ///< The z-position of the image.
    double m_z_mag{1.0}; ///< The z-magnification of the image.
    double m_z_rot{0.0}; ///< The orientation of the image around the axis.
};

#endif // CAELUM_CAELUM_SKY_H_INCLUDED
