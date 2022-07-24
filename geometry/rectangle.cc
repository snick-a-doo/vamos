//  Rectangle.cc - a rectangle.
//
//  Copyright (C) 2005 Sam Varner
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

#include "rectangle.h"

#include <algorithm>

using namespace Vamos_Geometry;

Rectangle::Rectangle ()
  : m_left (0.0),
    m_top (0.0),
    m_right (0.0),
    m_bottom (0.0),
    m_inverted (false)
{
}

Rectangle::Rectangle (double x, double y, double width, double height, bool inverted)
  : m_left (x),
    m_top (inverted ? y : y + height),
    m_right (x + width),
    m_bottom (inverted ? y + height : y),
    m_inverted (inverted)
{
}

Rectangle::Rectangle (const Two_Vector& upper_left, const Two_Vector& lower_right)
  : m_left (upper_left.x),
    m_top (upper_left.y),
    m_right (lower_right.x),
    m_bottom (lower_right.y),
    m_inverted (m_bottom > m_top)
{
}

bool
Rectangle::operator == (const Rectangle& other) const
{
  return m_left == other.m_left
    && m_top == other.m_top
    && m_right == other.m_right
    && m_bottom == other.m_bottom;
}

const Rectangle& Rectangle::enclose (const Rectangle& other)
{
  m_left = std::min (m_left, other.m_left);
  m_top = m_inverted ? std::min (m_top, other.m_top) : std::max (m_top, other.m_top);
  m_right = std::max (m_right, other.m_right);
  m_bottom = m_inverted ? std::max (m_bottom, other.m_bottom) : std::min (m_bottom, other.m_bottom);
  return *this;
}

const Rectangle& Rectangle::clip (const Rectangle& other)
{
  m_left = std::max (m_left, other.m_left);
  m_top = m_inverted ? std::max (m_top, other.m_top) : std::min (m_top, other.m_top);
  m_right = std::min (m_right, other.m_right);
  m_bottom = m_inverted ? std::min (m_bottom, other.m_bottom) : std::max (m_bottom, other.m_bottom);
  return *this;
}

void
Rectangle::scale (double x_factor, double y_factor)
{
  const Two_Vector mid = center ();

  m_left = (m_left - mid.x) * x_factor + mid.x;
  m_top = (m_top - mid.y) * y_factor + mid.y;
  m_right = (m_right - mid.x) * x_factor + mid.x;
  m_bottom = (m_bottom - mid.y) * y_factor + mid.y;
}

void
Rectangle::scale (double factor)
{
  scale (factor, factor);
}

void
Rectangle::move (const Two_Vector& delta)
{
  m_left += delta.x;
  m_top += delta.y;
  m_right += delta.x;
  m_bottom += delta.y;
}

std::ostream& operator << (std::ostream& os, const Vamos_Geometry::Rectangle& rectangle)
{
  os << "(" << rectangle.left () << ", " << rectangle.top ()
	 << ") (" << rectangle.right () << ", " << rectangle.bottom () << ")";
  return os;
}
