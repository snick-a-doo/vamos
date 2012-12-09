// Dashboard.cc - dashboard with gauges, dials, and other readouts.
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

#include "Dashboard.h"
#include "../media/Texture_Image.h"

#include <algorithm>
#include <iostream>
#include <functional>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

//* Class Scaler
Scaler::Scaler (double min_in, double min_out, double max_in, double max_out) :
  m_minimum_input (min_in),
  m_maximum_input (max_in),
  m_offset (min_out),
  m_factor ((max_out - min_out) / (max_in - min_in))
{
}

double
Scaler::scale (double value_in)
{
  value_in = std::max (m_minimum_input, value_in);
  value_in = std::min (m_maximum_input, value_in);
  return m_offset + (value_in - m_minimum_input) * m_factor;
}

//* Class Dial
Dial::Dial (double center_x, double center_y, double above, double radius, 
			double min, double min_angle, double max, double max_angle,
			std::string face_image, std::string needle_image) : 
  m_above (above),
  m_scaler (min, min_angle, max, max_angle),
  mp_face (0),
  m_center_x (center_x),
  m_center_y (center_y)
{
  if (face_image != "")
	{
	  mp_face = new Facade (face_image); 
      mp_face->set_radius (radius);
	}
  if (needle_image != "")
	{
	  mp_needle = new Facade (needle_image); 
      mp_needle->set_radius (radius);
	}
}

Dial::~Dial ()
{
  delete mp_needle;
  delete mp_face;
}

void
Dial::set (double value)
{
  m_angle = m_scaler.scale (value);
}

void gauge_transform (double over, 
                      double up, 
                      double in_front, 
                      double rotation = 0.0)
{
  glTranslatef (0.0, -over, up);
  glRotated (rotation, 1.0, 0.0, 0.0);
  glRotatef (-90.0, 0.0, 0.0, 1.0);
  glRotatef (90.0, 1.0, 0.0, 0.0);
  glTranslatef (0.0, 0.0, in_front);
}

void
Dial::draw () const
{
  glPushMatrix ();
  gauge_transform (m_center_x, m_center_y, m_above, 0.0);
  mp_face->draw ();
  glPopMatrix ();

  glPushMatrix ();
  gauge_transform (m_center_x, m_center_y, m_above + 0.01, m_angle);
  mp_needle->draw ();
  glPopMatrix ();
}

//* Class LED_Gauge
LED_Gauge::LED_Gauge (double x, double y, double above, double width, 
					  int elements, double min, double redline, 
					  std::string image, bool on_wheel) :
  m_x (x),
  m_y (y),
  m_above (above),
  m_width (width),
  m_elements (elements),
  m_min (min),
  m_range (redline - min),
  m_leds_on (0),
  m_list_index (glGenLists (1))
{
  m_on_steering_wheel = on_wheel;
  mp_leds = new Texture_Image (image, true, true);

  m_height = m_width / (2.0 * mp_leds->aspect_ratio ());

  glNewList (m_list_index, GL_COMPILE);

  mp_leds->activate ();

  glTranslatef (-m_above, -m_x, m_y);

  glColor3d (1.0, 1.0, 1.0);
  glBegin (GL_QUADS);
  glTexCoord2d (0.0, 0.5);
  glNormal3f (-1.0, 0.0, 0.0);
  glVertex3d (0.0, 0.0, 0.0);
  glTexCoord2d (1.0, 0.5);
  glVertex3d (0.0, -m_width, 0.0);
  glTexCoord2d (1.0, 1.0);
  glVertex3d (0.0, -m_width, m_height);
  glTexCoord2d (0.0, 1.0);
  glVertex3d (0.0, 0.0, m_height);
  glEnd ();

  glEndList ();
}

LED_Gauge::~LED_Gauge ()
{
  delete mp_leds;
}

void
LED_Gauge::set (double value)
{
  m_leds_on = int ((value - m_min)*(m_elements - 1)/m_range + 1.0);
  m_leds_on = std::max (m_leds_on, 0);
  m_leds_on = std::min (m_leds_on, m_elements);
}

void wheel_transform (double z)
{
  glTranslatef (0.0, 0.0, -z);
  glRotatef (90.0, 0.0, 0.0, 1.0);
  glRotatef (90.0, 0.0, 1.0, 0.0);
}

void
LED_Gauge::draw () const
{
  glPushMatrix ();
  if (m_on_steering_wheel)
    wheel_transform (m_above);

  // Draw the LEDs all off...
  glCallList (m_list_index);

  // ...then draw the ones that are on over top.
  double frac = double (m_leds_on) / m_elements;

  mp_leds->activate ();
	  
  glColor3d (1.0, 1.0, 1.0);
  glBegin (GL_QUADS);
  glTexCoord2d (0.0, 0.5);
  glNormal3f (-1.0, 0.0, 0.0);
  glVertex3d (0.0, 0.0, 0.0);
  glTexCoord2d (frac, 0.5);
  glVertex3d (0.0, -m_width * frac, 0.0);
  glTexCoord2d (frac, 0.0);
  glVertex3d (0.0, -m_width * frac, m_height);
  glTexCoord2d (0.0, 0.0);
  glVertex3d (0.0, 0.0, m_height);
  glEnd ();

  glPopMatrix ();
}

//* Class Digital Gauge
Digital_Gauge::Digital_Gauge (double x, double y, double above,
							  double width, double height, size_t places,
							  std::string digits, bool on_wheel) :
  m_x (x),
  m_y (y),
  m_above (above),
  m_width (width),
  m_height (height),
  m_places (places)
{
  m_on_steering_wheel = on_wheel;
  m_digits.resize (places);
  mp_digits = new Texture_Image (digits, true, true);
}

Digital_Gauge::~Digital_Gauge ()
{
  delete mp_digits;
}

void
Digital_Gauge::set (double value)
{
  int n = int (value);
  int denom = 1;
  int sub = 0;
  for (size_t index = 0; index < m_places; index++)
	{
	  int m = denom * 10;
	  int place = (n % m) / denom; 
	  m_digits [m_places - 1 - index] = place;
	  denom = m;
	  sub += place;
	}
}

void
Digital_Gauge::draw () const
{
  glPushMatrix ();
  if (m_on_steering_wheel)
    wheel_transform (m_above);

  mp_digits->activate ();

  bool nonzero = false;
  for (size_t i = 0; i < m_places; i++)
	{
	  int n = m_digits [i];
	  if ((!nonzero) && (n == 0) && (i != (m_places - 1)))
		{
		  continue;
		}
	  nonzero = true;

	  double tex_x1 =  n * 0.1;
	  double tex_x2 = tex_x1 + 0.1;

	  double x1 = i * m_width / m_places;
	  double x2 = x1 + m_width / m_places;

	  glColor3d (1.0, 1.0, 1.0);
	  glBegin (GL_QUADS);
      glNormal3f (-1.0, 0.0, 0.0);
	  glTexCoord2d (tex_x1, 1.0);
	  glVertex3d (-m_above, -m_x - x1, m_y);
	  glTexCoord2d (tex_x2, 1.0);
	  glVertex3d (-m_above, -m_x - x2, m_y);
	  glTexCoord2d (tex_x2, 0.0);
	  glVertex3d (-m_above, -m_x - x2, m_y + m_height);
	  glTexCoord2d (tex_x1, 0.0);
	  glVertex3d (-m_above, -m_x - x1, m_y + m_height);
	  glEnd ();
	}
  glPopMatrix ();
}

//* Class Steering_Wheel
Steering_Wheel::Steering_Wheel (double center_x, double center_y, 
								double above, double radius, 
								double min, double min_angle, 
								double max, double max_angle,
								std::string image) :
  Facade (image),
  m_center_x (center_x),
  m_center_y (center_y),
  m_above (above),
  m_scaler (min, min_angle, max, max_angle)
{
  set_radius (radius);
}

void
Steering_Wheel::set (double value)
{
  m_angle = m_scaler.scale (value);
}

void
Steering_Wheel::draw () const
{
  gauge_transform (m_center_x, m_center_y, m_above, m_angle);
  Facade::draw ();
}

//* Class Gear_Indicator
Gear_Indicator::Gear_Indicator (double x, double y, double above,
								double width, double height, 
								int numbers, std::string image,
								bool on_wheel) :
  m_number_width (1.0 / numbers),
  mp_numbers (0),
  m_x (x),
  m_y (y),
  m_above (above),
  m_width (width),
  m_height (height)
{
  m_on_steering_wheel = on_wheel;
  if (image != "")
	{
	  mp_numbers = new Texture_Image (image, true, true);
	}
}

Gear_Indicator::~Gear_Indicator ()
{
  delete mp_numbers;
}

void
Gear_Indicator::draw () const
{
  glPushMatrix ();
  if (m_on_steering_wheel)
    wheel_transform (m_above);

  mp_numbers->activate ();

  double x1 = m_number_width * (m_gear + 1);
  double x2 = x1 + m_number_width;

  glColor3d (1.0, 1.0, 1.0);
  glBegin (GL_QUADS);
  glNormal3f (-1.0, 0.0, 0.0);
  glTexCoord2d (x2, 1.0);
  glVertex3d (-m_above, -m_x, m_y);
  glTexCoord2d (x1, 1.0);
  glVertex3d (-m_above, -m_x + m_width, m_y);
  glTexCoord2d (x1, 0.0);
  glVertex3d (-m_above, -m_x + m_width, m_y + m_height);
  glTexCoord2d (x2, 0.0);
  glVertex3d (-m_above, -m_x, m_y + m_height);
  glEnd ();

  glPopMatrix ();
}


//* Class Gear_Shift
Gear_Shift::Gear_Shift (double x, double y, double z, 
						double width, double height,
						const Vamos_Geometry::Three_Vector& rotation,
						const std::vector <Vamos_Geometry::Two_Vector>& 
						positions,
						std::string plate_image, std::string stick_image) :
  Gear_Indicator (x, y, z, width, height, 0, "", false),
  m_rotation (rotation),
  m_positions (positions),
  m_top_gear (m_positions.size () - 2),
  m_list_index (glGenLists (2))
{
  mp_gate = new Texture_Image (plate_image, true, true);
  mp_stick = new Texture_Image (stick_image, true, true);

  m_stick_width = 
	m_width * mp_stick->width_pixels () / mp_gate->width_pixels ();
  m_stick_height = 
	m_height * mp_stick->height_pixels () / mp_gate->height_pixels ();

  glNewList (m_list_index, GL_COMPILE);

  mp_gate->activate ();

  glRotated (rotation.x, 0.0, -1.0, 0.0);
  glRotated (rotation.y, 0.0, 0.0, 1.0);
  glRotated (rotation.z, 1.0, 0.0, 0.0);
  glTranslated (-m_above, -m_x, m_y);

  glColor3d (1.0, 1.0, 1.0);
  glBegin (GL_QUADS);
  glTexCoord2d (0.0, 0.0);
  glNormal3f (-1.0, 0.0, 0.0);
  glVertex3d (0.0, 0.0, 0.0);
  glTexCoord2d (1.0, 0.0);
  glVertex3d (0.0, -m_width, 0.0);
  glTexCoord2d (1.0, 1.0);
  glVertex3d (0.0, -m_width, m_height);
  glTexCoord2d (0.0, 1.0);
  glVertex3d (0.0, 0.0, m_height);
  glEnd ();

  glTranslated (0.0, (-m_width + m_stick_width) / 2.0, m_height / 2.0);
  glEndList ();

  glNewList (m_list_index + 1, GL_COMPILE);

  mp_stick->activate ();

  glRotated (-rotation.x, 0.0, -1.0, 0.0);
  glRotated (-rotation.y, 0.0, 0.0, 1.0);
  glRotated (-rotation.z, 1.0, 0.0, 0.0);

  glColor3d (1.0, 1.0, 1.0);
  glBegin (GL_QUADS);
  glTexCoord2d (0.0, 1.0);
  glNormal3f (-1.0, 0.0, 0.0);
  glVertex3d (0.0, 0.0, 0.0);
  glTexCoord2d (1.0, 1.0);
  glVertex3d (0.0, -m_stick_width, 0.0);
  glTexCoord2d (1.0, 0.0);
  glVertex3d (0.0, -m_stick_width, m_stick_height);
  glTexCoord2d (0.0, 0.0);
  glVertex3d (0.0, 0.0, m_stick_height);
  glEnd ();

  glEndList ();
}

Gear_Shift::~Gear_Shift ()
{
  delete mp_stick;
  delete mp_gate;
}

void
Gear_Shift::set (int gear)
{
  m_gear = std::min (m_top_gear, gear);
}

void
Gear_Shift::draw () const
{
  glPushMatrix ();
  glCallList (m_list_index);
  glTranslated (0.0, -m_positions [m_gear + 1].x, m_positions [m_gear + 1].y);
  glCallList (m_list_index + 1);
  glPopMatrix ();
}


//* Class Dashboard
Dashboard::Dashboard (double x, double y, double z, double tilt) :
  m_x (x),
  m_y (y),
  m_z (z),
  m_tilt (tilt),
  mp_tachometer (0),
  mp_speedometer (0),
  mp_fuel_gauge (0),
  mp_gear_indicator (0),
  mp_steering_wheel (0)
{
}

Dashboard::~Dashboard ()
{
  delete mp_steering_wheel;
  delete mp_gear_indicator;
  delete mp_fuel_gauge;
  delete mp_speedometer;
  delete mp_tachometer;
  for (std::vector <Facade*>::iterator it = ma_facades.begin ();
	   it != ma_facades.end ();
	   it++)
	{
	  delete *it;
	}
}

void
Dashboard::add_tachometer (Gauge* tachometer)
{
  delete mp_tachometer;
  mp_tachometer = tachometer;
}

void
Dashboard::add_speedometer (Gauge* speedometer)
{
  delete mp_speedometer;
  mp_speedometer = speedometer;
}

void
Dashboard::add_fuel_gauge (Gauge* fuel_gauge)
{
  delete mp_fuel_gauge;
  mp_fuel_gauge = fuel_gauge;
}

void
Dashboard::add_gear_indicator (Gear_Indicator* gear_indicator)
{
  delete mp_gear_indicator;
  mp_gear_indicator = gear_indicator;
}

void
Dashboard::add_steering_wheel (Steering_Wheel* steering_wheel)
{
  delete mp_steering_wheel;
  mp_steering_wheel = steering_wheel;
}

void
Dashboard::add_facade (Facade* facade)
{
  ma_facades.push_back (facade);
}

void
Dashboard::set_tachometer (double rpm)
{
  if (mp_tachometer != 0)
	{
	  mp_tachometer->set (rpm);
	}
}

void
Dashboard::set_speedometer (double speed)
{
  if (mp_speedometer != 0)
	{
	  mp_speedometer->set (speed);
	}
}

void
Dashboard::set_fuel_gauge (double fuel)
{
  if (mp_fuel_gauge != 0)
	{
	  mp_fuel_gauge->set (fuel);
	}
}

void
Dashboard::set_gear_indicator (int gear)
{
  if (mp_gear_indicator != 0)
	{
	  mp_gear_indicator->set (gear);
	}
}

void
Dashboard::set_steering_wheel (double angle)
{
  if (mp_steering_wheel != 0)
	{
	  mp_steering_wheel->set (angle);
	}
}

void
Dashboard::draw () const
{
  glTranslated (m_x, m_y, m_z);

  for (std::vector <Facade*>::const_iterator it = ma_facades.begin ();
       it != ma_facades.end ();
       it++)
    {
      glPushMatrix ();
      glRotatef (-90.0, 0.0, 0.0, 1.0);
      glRotatef (90.0, 1.0, 0.0, 0.0);
      (*it)->draw ();
      glPopMatrix ();
    }

  glRotated (m_tilt, 0.0, 1.0, 0.0);

  if ((mp_tachometer != 0) && !mp_tachometer->on_steering_wheel ())
    {
      mp_tachometer->draw ();
    }
  if ((mp_speedometer != 0) && !mp_speedometer->on_steering_wheel ())
    {
      mp_speedometer->draw ();
    }
  if ((mp_fuel_gauge != 0) && !mp_fuel_gauge->on_steering_wheel ())
    {
      mp_fuel_gauge->draw ();
    }
  if ((mp_gear_indicator != 0) && !mp_gear_indicator->on_steering_wheel ())
    {
      mp_gear_indicator->draw ();
    }
  if (mp_steering_wheel != 0)
    {
      mp_steering_wheel->draw ();
    }

  glDisable (GL_DEPTH_TEST);
  if ((mp_tachometer != 0) && mp_tachometer->on_steering_wheel ())
    {
      mp_tachometer->draw ();
    }
  if ((mp_speedometer != 0) && mp_speedometer->on_steering_wheel ())
    {
      mp_speedometer->draw ();
    }
  if ((mp_fuel_gauge != 0) && mp_fuel_gauge->on_steering_wheel ())
    {
      mp_fuel_gauge->draw ();
    }
  if ((mp_gear_indicator != 0) && mp_gear_indicator->on_steering_wheel ())
    {
      mp_gear_indicator->draw ();
    }
  glEnable (GL_DEPTH_TEST);
}

