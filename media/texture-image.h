//  Copyright (C) 2001-2022 Sam Warner
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

#ifndef VAMOS_MEDIA_TEXTURE_IMAGE_H_INCLUDED
#define VAMOS_MEDIA_TEXTURE_IMAGE_H_INCLUDED

#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"

#include <GL/gl.h>

#include <memory>
#include <stdexcept>
#include <string>

namespace Vamos_Media
{
// Exception class for texture files that can't be found or read.
class Missing_Texture_File : std::runtime_error
{
public:
    Missing_Texture_File(std::string const& file)
        : std::runtime_error{"Can't find the texture file \"" + file + "\""}
    {}
};

/// Texture data read from a PNG file.
class Texture_Image
{
    using Data_Ptr = std::unique_ptr<unsigned char[]>;

public:
    /// @param file_name The PNG file name.
    /// @param smooth Smooth the image for display if true.
    /// @param mip_map Blur when small if true.
    /// @param width Coordinate width of the texture.
    /// @param height Coordinate height of the texture.
    /// @param texture_wrap What to do at the edge of the image.
    Texture_Image(std::string const& file_name, bool smooth = false, bool mip_map = false,
                  Vamos_Geometry::Point<double> size = {1.0, 1.0},
                  int texture_wrap = GL_REPEAT);
    ~Texture_Image();

    /// @return The width of the source image in pixels.
    int width_pixels() const { return m_width_pixels; }
    /// @return The height of the source image in pixels.
    int height_pixels() const { return m_height_pixels; }
    /// @return The aspect ratio of the source image.
    double image_aspect_ratio() const;

    /// Set the coordinate width and height of the texture.
    void set_size(double width, double height);
    /// @return The coordinate width of the texture.
    double width() const { return m_size.x; }
    /// @return The coordinate height of the texture.
    double height() const { return m_size.y; }

    /// Make this texture the one currently rendered.
    void activate() const;

private:
    std::string m_file_name; ///< PNG file name
    int m_channels{3};       ///< 3 for RGB, 4 for RGBA
    int m_width_pixels{0};   ///< Source image width
    int m_height_pixels{0};  ///< Source image height
    Vamos_Geometry::Point<double> m_size{1.0, 1.0}; ///< Texture width and height
    GLuint m_texture_id{0};  ///< Resource handle

    Data_Ptr read_png_file(std::string const& file_name); ///< Image data.
};

//----------------------------------------------------------------------------------------
/// A rectangle width a texture image.
class Facade : public Texture_Image
{
public:
    /// @param draw_back If true, draw a solid-color rectangle for the back of the image.
    Facade(std::string const& image_name,
           Vamos_Geometry::Three_Vector const& position,
           Vamos_Geometry::Point<double> size,
           bool draw_back);
    Facade(std::string const& image_name,
           Vamos_Geometry::Three_Vector const& center,
           double radius, bool draw_back);

    /// @return The coordinates of the center of the image.
    Vamos_Geometry::Three_Vector get_center() const;
    /// Render the facade.
    void draw() const;

private:
    Vamos_Geometry::Three_Vector m_position; ///< Lower-left corner.
    bool m_draw_back{false}; ///< True if the reverse side should be filled in.
};
} // namespace Vamos_Media

#endif // VAMOS_MEDIA_TEXTURE_IMAGE_H_INCLUDED
