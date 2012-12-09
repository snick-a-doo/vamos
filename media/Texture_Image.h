// Texture_Image.cc - 2D textures from PNG images.
//
//	Vamos Automotive Simulator
//  Copyright (C) 2001--2005 Sam Varner
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

#ifndef _TEXTURE_IMAGE_H_
#define _TEXTURE_IMAGE_H_

#ifdef WIN32
# define WINDOWS_LEAN_AND_MEAN 1
# define NOMINMAX 1               // Do not define MS' min()/max() macros.
# include <windows.h>
#endif

#include <GL/gl.h>

#include <string>
#include <map>

namespace Vamos_Media
{
  // Exception class for texture files that can't be found.
  class Missing_Texture_File
  {
	std::string m_file;
  public:
	Missing_Texture_File (std::string file) : m_file (file) {};
	std::string message () const { return m_file; }
  };

  class Texture_Image
  {
    std::string m_file_name;
	int m_channels;
	int m_width_pixels;
	int m_height_pixels;
    double m_width;
    double m_height;
	GLuint m_texture_name;

    void initialize (bool smooth, bool mip_map, int texture_wrap);
    unsigned char* read_png_file (std::string file_name);
    void set_gl_parameters (unsigned char* data,
                            bool smooth, 
                            bool mip_map, 
                            int texture_wrap);

  public:
	Texture_Image (std::string file_name, 
                   bool smooth = false, 
                   bool mip_map = false,
                   double width = 1.0,
                   double height = 1.0,
                   int texture_wrap = GL_REPEAT);

	Texture_Image (std::string file_name, 
                   bool smooth, 
                   bool mip_map,
                   int texture_wrap);

    Texture_Image () {};
    ~Texture_Image ();

    int width_pixels () const { return m_width_pixels; }
    int height_pixels () const { return m_height_pixels; }
    double aspect_ratio () const 
    { return double (m_width_pixels) / m_height_pixels; }

    void set_width (double width) { m_width = width; }
    void set_height (double height) { m_height = height; }
    double width () const { return m_width; }
    double height () const { return m_height; }

	void activate () const;
  };

  class Facade : public Texture_Image
  {
    bool m_draw_back;
    double m_x_offset;
    double m_y_offset;
    double m_z_offset;

  public:
    Facade (std::string image_name, bool draw_back = false);
    // If `draw_back' is true, draw a solid-color rectangle for the
    // back of the image.

    void set_radius (double radius);
    void set_x_offset (double offset) { m_x_offset = offset; }
    void set_y_offset (double offset) { m_y_offset = offset; }
    void set_z_offset (double offset) { m_z_offset = offset; }
    void draw () const;
  };
}

#endif // not _TEXTURE_IMAGE_H_
