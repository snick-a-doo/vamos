//  Rectangle.h - a rectangle.
//
//	Vamos Automotive Simulator
//  Copyright (C) 2005 Sam Varner
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

#ifndef _RECTANGLE_H_
#define _RECTANGLE_H_

#include "../geometry/Numeric.h"
#include "../geometry/Two_Vector.h"

#include <iostream>

namespace Vamos_Geometry
{
  class Rectangle
  {
  public:
    Rectangle ();
    Rectangle (double x, double y, double width, double height);
    Rectangle (const Vamos_Geometry::Two_Vector& upper_left,
               const Vamos_Geometry::Two_Vector& lower_right);

    double left () const { return m_left; }
    double top () const { return m_top; }
    double right () const { return m_right; }
    double bottom () const { return m_bottom; }

    double width () const { return m_right - m_left; }
    double height () const { return m_top - m_bottom; }

    inline Vamos_Geometry::Two_Vector center () const;

    double aspect () const { return width () / height (); }

    bool operator == (const Rectangle& other) const;

    // Enlarge the rectangle if necessary so that another rectangle is
    // completely enclosed.
    const Rectangle& enclose (const Rectangle& other);

    void scale (double x_factor, double y_factor);
    void scale (double factor);
    void move (const Vamos_Geometry::Two_Vector& delta);

  private:
    double m_left;
    double m_top;
    double m_right;
    double m_bottom;
  };

  Vamos_Geometry::Two_Vector
  Rectangle::center () const
  { 
    return Vamos_Geometry::Two_Vector (Vamos_Geometry::average (m_left, m_right),
                                       Vamos_Geometry::average (m_top, m_bottom));
  }

  std::ostream& operator << (std::ostream& os, const Rectangle& rectangle);
}

#endif // not _RECTANGLE_H_
