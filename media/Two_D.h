//  2D - Convenience functions for 2D OpenGl rendering. 
//
//	Vamos Automotive Simulator
//  Copyright (C) 2013 Sam Varner
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

#ifndef _TWO_D_H_
#define _TWO_D_H_

#include "../geometry/Rectangle.h"

#include <iomanip>
#include <sstream>
#include <string>

namespace Vamos_Media
{
  class Two_D
  {
  public:
    Two_D ();
    ~Two_D ();

    void text (double x, double y, const std::string& label) { text (x, y, label, ""); }

    template <typename T1, typename T2> void text (double x,
                                                   double y,
                                                   const T1& label, 
                                                   const T2& value,
                                                   const std::string& units = "",
                                                   int precision = 0)
    {
      std::ostringstream os;
      os.setf (std::ios::fixed);
      os << std::setprecision (precision) << label << ' ' << value << ' ' << units;
      draw_string (os.str (), x, y);
    }

    void bar (const Vamos_Geometry::Rectangle& box, 
              double red, double green, double blue,
              double fraction);

  private:
    void draw_string (const std::string& str, double x, double y);
  };
}

#endif // not _TWO_D_H_
