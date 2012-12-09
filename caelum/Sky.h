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

#ifndef _SKY_H_
#define _SKY_H_

#include <string>

namespace Vamos_Media
{
  class Texture_Image;
}

class Sky
{
public:
  // image - the name of the image file.
  // width - the width of each viewport.
  // height - the height of each viewport.
  Sky (std::string image, int width, int height);
  virtual ~Sky ();

  // Render the entire scene.
  void display ();

  void key_press (int key);

protected:
  // Render a view.
  virtual void draw () = 0;

  double z_pos () const { return m_z_pos; }
  double z_mag () const { return m_z_mag; }
  double z_rot () const { return m_z_rot; }

private:  
  // The texture image.
  Vamos_Media::Texture_Image* mp_image;

  // The size of the viewports.
  int m_width;
  int m_height;
  
  // The z-position of the image.
  double m_z_pos;
  // The z-magnification of the image.
  double m_z_mag;
  // The orientation of the image around the axis.
  double m_z_rot;
};

#endif // not _SKY_H_
