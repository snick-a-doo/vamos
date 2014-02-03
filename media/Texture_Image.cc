// Texture_Image.cc - 2D textures from PNG images.
//
//  Copyright (C) 2001--2005 Sam Varner
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

#include "Texture_Image.h"

#include <cassert>
#include <stdio.h> // libpng uses FILE.
#include <png.h>

// Windows puts OpenGL 1.2 and 1.3 stuff in glext.h.
#if HAVE_GL_GLEXT_H
# include <GL/glext.h>
#endif
#include <GL/glu.h>
#include <iostream>

using namespace Vamos_Media;

struct Cached_Image
{
  GLuint texure_name;
  size_t width_pixels;
  size_t height_pixels;
  size_t reference_count;

  Cached_Image (GLuint name = 0, size_t width = 0, size_t height = 0)
    : texure_name (name),
      width_pixels (width),
      height_pixels (height),
      reference_count (1)
  {};
};

static std::map <std::string, Cached_Image> ms_image_cache;

Texture_Image::Texture_Image (std::string file_name, 
                              bool smooth, 
                              bool mip_map,
                              int texture_wrap)
  : m_file_name (file_name),
    m_width (1.0),
    m_height (1.0)
{
  initialize (smooth, mip_map, texture_wrap);
}

Texture_Image::Texture_Image (std::string file_name, 
                              bool smooth, 
                              bool mip_map,
                              double width,
                              double height,
                              int texture_wrap)
  : m_file_name (file_name),
    m_width (width),
    m_height (height),
    m_texture_name (0)
{
  initialize (smooth, mip_map, texture_wrap);
}

void
Texture_Image::initialize (bool smooth, bool mip_map, int texture_wrap)
{
  if (m_file_name.empty ())
    return;
  else if (ms_image_cache.find (m_file_name) != ms_image_cache.end ())
    {
      Cached_Image& image (ms_image_cache [m_file_name]);
      m_texture_name = image.texure_name;
      m_width_pixels = image.width_pixels;
      m_height_pixels = image.height_pixels;
      image.reference_count++;
      activate ();
    }
  else
    {
      unsigned char* data = read_png_file (m_file_name);

      GLuint texture_id;
      glGenTextures (1, &texture_id);
      glBindTexture (GL_TEXTURE_2D, texture_id);
      set_gl_parameters (data, smooth, mip_map, texture_wrap);
      m_texture_name = texture_id;
      delete [] data;
      ms_image_cache [m_file_name] = 
        Cached_Image (m_texture_name, m_width_pixels, m_height_pixels);
    }
}

Texture_Image::~Texture_Image ()
{
  if (ms_image_cache.find (m_file_name) != ms_image_cache.end ())
    {
      Cached_Image& image (ms_image_cache [m_file_name]);
      if (--image.reference_count == 0)
        {
          glDeleteTextures (1, &m_texture_name);
          ms_image_cache.erase (m_file_name);
        }
    }
}

unsigned char*
Texture_Image::read_png_file (std::string file_name)
{
  // See if the file is a PNG file.
  FILE *fp = fopen (file_name.c_str (), "rb");
  if (!fp)
    {
	  throw Missing_Texture_File (file_name);
    }

  png_byte header [8];
  size_t bytes_read = fread (header, 1, 8, fp);
  if (bytes_read != 8)
    throw Missing_Texture_File (file_name);

  bool is_png = !png_sig_cmp (header, 0, 8);
  if (!is_png)
    {
	  throw Missing_Texture_File (file_name);
    }
  
  // Initialize the structures.
  png_structp png_ptr = 
	png_create_read_struct (PNG_LIBPNG_VER_STRING, 0, 0, 0);
  if (png_ptr == 0)
	{
	  throw Missing_Texture_File (file_name);
	}

  png_infop info_ptr = png_create_info_struct (png_ptr);
  if (info_ptr == 0)
	{
	  png_destroy_read_struct (&png_ptr, 0, 0);
	  throw Missing_Texture_File (file_name);
	}

  png_infop end_info = png_create_info_struct (png_ptr);
  if (end_info == 0)
	{
	  png_destroy_read_struct(&png_ptr, &info_ptr, 0);
	  throw Missing_Texture_File (file_name);
	}
  
  png_init_io (png_ptr, fp);
  png_set_sig_bytes (png_ptr, 8);
  png_read_png (png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, 0);

  fclose (fp);

  png_bytep* row_pointers = png_get_rows (png_ptr, info_ptr);

  m_width_pixels = png_get_image_width (png_ptr, info_ptr);
  m_height_pixels = png_get_image_height (png_ptr, info_ptr);
  m_channels = png_get_channels (png_ptr, info_ptr);
  int row_size = m_width_pixels * m_channels;

  size_t data_size = row_size * m_height_pixels;
  unsigned char* data = new unsigned char [data_size];

  for (int i = 0; i < m_height_pixels; i++)
	{
	  for (int j = 0; j < row_size; j++)
		{
		  data [i * row_size + j] = row_pointers [i][j];
		}
	}
  png_destroy_read_struct (&png_ptr, &info_ptr, &end_info);

  return data;
}

void
Texture_Image::set_gl_parameters (unsigned char* data,
                                  bool smooth, 
                                  bool mip_map, 
                                  int texture_wrap)
{
  assert (data != 0);

  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, texture_wrap);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, texture_wrap);

  GLint format = 0;
  switch (m_channels)
	{
	case 1:
	  format = GL_LUMINANCE;
	  break;
	case 3:
	  format = GL_RGB;
	  break;
	case 4:
	  format = GL_RGBA;
	  break;
	default:
	  assert (false);
	}

  if (mip_map)
	{
	  if (smooth)
		{
		  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
						   GL_LINEAR_MIPMAP_LINEAR);
		}
	  else
		{
		  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
						   GL_NEAREST_MIPMAP_NEAREST);
		}
	  gluBuild2DMipmaps (GL_TEXTURE_2D, format, 
						 m_width_pixels, m_height_pixels,
						 format, GL_UNSIGNED_BYTE, 
						 data);
	}
  else
	{
	  if (smooth)
		{
		  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		}
	  else
		{
		  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		}

	  glTexImage2D (GL_TEXTURE_2D, 0, format, 
					m_width_pixels, m_height_pixels,
					0, format, GL_UNSIGNED_BYTE, 
					data);
	}
}

void
Texture_Image::activate () const
{
  glBindTexture (GL_TEXTURE_2D, m_texture_name);
}

//* Class Facade
Facade::Facade (std::string image_name, bool draw_back) 
  : Texture_Image (image_name, true, true, 1.0, 1.0, GL_CLAMP_TO_EDGE),
    m_draw_back (draw_back),
    m_x_offset (0.0),
    m_y_offset (0.0),
    m_z_offset (0.0)
{
}

void
Facade::set_radius (double radius)
{
  set_width (2.0 * radius * aspect_ratio ());
  set_height (2.0 * radius);
  m_x_offset = -width () / 2.0;
  m_y_offset = -height () / 2.0;
}

void
Facade::draw () const
{
  activate ();
  glColor3d (1.0, 1.0, 1.0);
  glEnable (GL_CULL_FACE);
  glBegin (GL_QUADS);
  glNormal3f (0.0, 0.0, 1.0);
  glTexCoord2d (0.0, 1.0);
  glVertex3d (m_x_offset, m_y_offset, m_z_offset);
  glTexCoord2d (1.0, 1.0);
  glVertex3d (m_x_offset + width (), m_y_offset, m_z_offset);
  glTexCoord2d (1.0, 0.0);
  glVertex3d (m_x_offset + width (), m_y_offset + height (), m_z_offset);
  glTexCoord2d (0.0, 0.0);
  glVertex3d (m_x_offset, m_y_offset + height (), m_z_offset);

  if (m_draw_back)
    {
      glNormal3f (0.0, 0.0, -1.0);
      glVertex3d (m_x_offset, m_y_offset, m_z_offset);
      glVertex3d (m_x_offset, m_y_offset + height (), m_z_offset);
      glVertex3d (m_x_offset + width (), m_y_offset + height (), m_z_offset);
      glVertex3d (m_x_offset + width (), m_y_offset, m_z_offset);
    }
  glEnd ();
  glDisable (GL_CULL_FACE);
  glBindTexture (GL_TEXTURE_2D, 0);
}

