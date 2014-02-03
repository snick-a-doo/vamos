//  Two_D.h2 - Convenience functions for 2D OpenGl rendering. 
//
//  Copyright (C) 2013 Sam Varner
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
    Two_D (int width = 100, int height = 100);
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

    void lights (double x, double y, double r, int n, int n_on,
                 double red_on, double green_on, double blue_on,
                 double red_off, double green_off, double blue_off);

  private:
    void draw_string (const std::string& str, double x, double y);
  };
}

#endif // not _TWO_D_H_
