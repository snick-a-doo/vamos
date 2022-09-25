//  Copyright (C) 2001-2022 Sam Varner
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

#include "texture-image.h"

#include <png.h>

#include <GL/glu.h>

#include <cassert>
#include <cstdio> // libpng uses FILE.
#include <map>

using namespace Vamos_Media;

/// Reference to an already-read texture file.
struct Cached_Image
{
    GLuint texure_id{0};
    int width_pixels{0};
    int height_pixels{0};
    int reference_count{1};
};
/// The already-read textures.
static std::map<std::string, Cached_Image> image_cache;

Texture_Image::Texture_Image(std::string const& file_name, bool smooth, bool mip_map, double width,
                             double height, int texture_wrap)
    : m_file_name(file_name),
      m_width(width),
      m_height(height)
{
    if (m_file_name.empty())
        return;
    else if (image_cache.count(m_file_name))
    {
        auto& image(image_cache[m_file_name]);
        m_texture_id = image.texure_id;
        m_width_pixels = image.width_pixels;
        m_height_pixels = image.height_pixels;
        ++image.reference_count;
        activate();
        return;
    }
    auto data{read_png_file(m_file_name)};
    assert(m_channels == 1 || m_channels == 3 || m_channels == 4);
    auto format{m_channels == 1 ? GL_LUMINANCE
                : m_channels == 3 ? GL_RGB
                : GL_RGBA};

    glGenTextures(1, &m_texture_id);
    glBindTexture(GL_TEXTURE_2D, m_texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, texture_wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, texture_wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, smooth ? GL_LINEAR : GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, smooth
                    ? (mip_map ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR)
                    : (mip_map ? GL_NEAREST_MIPMAP_NEAREST : GL_NEAREST));
    if (mip_map)
        gluBuild2DMipmaps(GL_TEXTURE_2D, format, m_width_pixels, m_height_pixels, format,
                          GL_UNSIGNED_BYTE, data.get());
    else
        glTexImage2D(GL_TEXTURE_2D, 0, format, m_width_pixels, m_height_pixels, 0, format,
                     GL_UNSIGNED_BYTE, data.get());

    image_cache[m_file_name] = {m_texture_id, m_width_pixels, m_height_pixels};
}

Texture_Image::~Texture_Image()
{
    if (image_cache.count(m_file_name) && --image_cache[m_file_name].reference_count == 0)
        glDeleteTextures(1, &m_texture_id);
}

Texture_Image::Data_Ptr Texture_Image::read_png_file(std::string const& file_name)
{
    // See if the file is a PNG file.
    auto* fp{fopen(file_name.c_str(), "rb")};
    if (!fp)
        throw Missing_Texture_File(file_name);
    png_byte header[8];
    auto bytes_read{fread(header, 1, 8, fp)};
    if (bytes_read != 8)
        throw Missing_Texture_File(file_name);
    auto is_png{!png_sig_cmp(header, 0, 8)};
    if (!is_png)
        throw Missing_Texture_File(file_name);
    // Initialize the structures.
    auto* png_ptr{png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0)};
    if (!png_ptr)
        throw Missing_Texture_File(file_name);
    auto* info_ptr{png_create_info_struct(png_ptr)};
    if (!info_ptr)
    {
        png_destroy_read_struct(&png_ptr, 0, 0);
        throw Missing_Texture_File(file_name);
    }
    auto end_info{png_create_info_struct(png_ptr)};
    if (!end_info)
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, 0);
        throw Missing_Texture_File(file_name);
    }
    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, 8);
    png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, 0);
    fclose(fp);

    m_width_pixels = png_get_image_width(png_ptr, info_ptr);
    m_height_pixels = png_get_image_height(png_ptr, info_ptr);
    m_channels = png_get_channels(png_ptr, info_ptr);
    auto row_size{m_width_pixels * m_channels};
    auto data_size{row_size * m_height_pixels};
    auto data{std::make_unique<unsigned char[]>(data_size)};
    png_bytep* row_pointers{png_get_rows(png_ptr, info_ptr)};
    for (auto i{0}; i < m_height_pixels; ++i)
        for (auto j{0}; j < row_size; ++j)
            data[i * row_size + j] = row_pointers[i][j];
    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
    return data;
}

void Texture_Image::activate() const
{
    glBindTexture(GL_TEXTURE_2D, m_texture_id);
}

//----------------------------------------------------------------------------------------
Facade::Facade(std::string const& image_name, bool draw_back)
    : Texture_Image(image_name, true, true, 1.0, 1.0, GL_CLAMP_TO_EDGE),
      m_draw_back(draw_back)
{
}

void Facade::set_radius(double radius)
{
    set_width(2.0 * radius * aspect_ratio());
    set_height(2.0 * radius);
    m_offset = {-width() / 2.0, -height() / 2.0, m_offset.z};
}

void Facade::draw() const
{
    activate();
    glColor3d(1.0, 1.0, 1.0);
    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);
    glNormal3f(0.0, 0.0, 1.0);
    glTexCoord2d(0.0, 1.0);
    glVertex3d(m_offset.x, m_offset.y, m_offset.z);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(m_offset.x + width(), m_offset.y, m_offset.z);
    glTexCoord2d(1.0, 0.0);
    glVertex3d(m_offset.x + width(), m_offset.y + height(), m_offset.z);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(m_offset.x, m_offset.y + height(), m_offset.z);
    if (m_draw_back)
    {
        glNormal3f(0.0, 0.0, -1.0);
        glVertex3d(m_offset.x, m_offset.y, m_offset.z);
        glVertex3d(m_offset.x, m_offset.y + height(), m_offset.z);
        glVertex3d(m_offset.x + width(), m_offset.y + height(), m_offset.z);
        glVertex3d(m_offset.x + width(), m_offset.y, m_offset.z);
    }
    glEnd();
    glDisable(GL_CULL_FACE);
    glBindTexture(GL_TEXTURE_2D, 0);
}
