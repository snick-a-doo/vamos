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

#ifndef _DASHBOARD_H_
#define _DASHBOARD_H_

#include "../media/Texture_Image.h"
#include "../geometry/Three_Vector.h"
#include "../geometry/Two_Vector.h"

#include <GL/gl.h>

#include <vector>

namespace Vamos_Body
{
  class Scaler
  {
	double m_minimum_input;
	double m_maximum_input;
	double m_offset;
	double m_factor;
	
  public:
	Scaler (double min_in, double min_out, double max_in, double max_out);
	double scale (double value_in);
  };

  class Steering_Wheel : public Vamos_Media::Facade
  {
	double m_center_x;
	double m_center_y;
    double m_above;
	double m_angle;
	Scaler m_scaler;

  public:
	Steering_Wheel (double center_x, double center_y, double above, 
					double radius, double min, double min_angle, 
					double max, double max_angle,
					std::string image);

	void set (double value);
	void draw () const;
  };

  class Gauge
  {
  protected:
	bool m_on_steering_wheel;

  public:
	Gauge () : m_on_steering_wheel (false) {};
	virtual ~Gauge () {};
	virtual void set (double value) = 0;
	virtual void draw () const = 0;
	bool on_steering_wheel () const { return m_on_steering_wheel; }
  };

  class Dial : public Gauge
  {
	double m_above;
	Scaler m_scaler;
	
	Vamos_Media::Facade* mp_face;
	Vamos_Media::Facade* mp_needle;

  protected:
	double m_center_x;
	double m_center_y;
	double m_angle;

  public:
	Dial (double center_x, double center_y, double above, double radius, 
		  double min, double min_angle, double max, double max_angle,
		  std::string face_image, std::string needle_image);
	
	~Dial ();
	
	void set (double value);
	void draw () const;

	//	double angle () const { return m_angle; }
  };

  class LED_Gauge : public Gauge
  {
	double m_x;
	double m_y;
	double m_above;
	double m_width;
	double m_height;
	int m_elements;
	double m_min;
	double m_range;

	int m_leds_on;

	Vamos_Media::Texture_Image* mp_leds;

	GLuint m_list_index;

  public:
	LED_Gauge (double x, double y, double above, double width, int elements,
			   double min, double redline, std::string image, bool on_wheel);
	~LED_Gauge ();

	void set (double value);
	void draw () const;
  };

  class Digital_Gauge : public Gauge
  {
	double m_x;
	double m_y;
	double m_above;
	double m_width;
	double m_height;
	size_t m_places;

	std::vector <int> m_digits;

	Vamos_Media::Texture_Image* mp_digits;

  public:
	Digital_Gauge (double x, double y, double above, 
				   double width, double height, size_t places,
				   std::string digits, bool on_wheel);
	~Digital_Gauge ();

	void set (double value);
	void draw () const;
  };


  class Gear_Indicator
  {
	bool m_on_steering_wheel;

	double m_number_width;

	Vamos_Media::Texture_Image* mp_numbers;

  protected:
	double m_x;
	double m_y;
	double m_above;
	double m_width;
	double m_height;
	int m_gear;

  public:
	Gear_Indicator (double x, double y, double above, 
					double width, double height,
					int numbers, std::string image, bool on_wheel);
	virtual ~Gear_Indicator ();

	virtual void set (int gear) { m_gear = gear; }
	virtual void draw () const;
	bool on_steering_wheel () const { return m_on_steering_wheel; }
  };


  class Gear_Shift : public Gear_Indicator
  {
	double m_stick_width;
	double m_stick_height;

	Vamos_Geometry::Three_Vector m_rotation;
	std::vector <Vamos_Geometry::Two_Vector> m_positions;
	int m_top_gear;

	Vamos_Media::Texture_Image* mp_gate;
	Vamos_Media::Texture_Image* mp_stick;

	GLuint m_list_index;

  public:
	Gear_Shift (double x, double y, double z, double width, double height,
				const Vamos_Geometry::Three_Vector& rotation,
				const std::vector <Vamos_Geometry::Two_Vector>& positions,
				std::string gate_image, std::string stick_image);
	~Gear_Shift ();

	void set (int gear);
	void draw () const;
  };


  class Dashboard
  {
	double m_x;
	double m_y;
	double m_z;
	double m_tilt;

	Gauge* mp_tachometer;
	Gauge* mp_speedometer;
	Gauge* mp_fuel_gauge;
	Gear_Indicator* mp_gear_indicator;
	Steering_Wheel* mp_steering_wheel;
	std::vector <Vamos_Media::Facade*> ma_facades;

  public:
	Dashboard (double x, double y, double z, double tilt);
	~Dashboard ();

	void add_tachometer (Gauge* tachometer);
	void add_speedometer (Gauge* speedometer);
	void add_fuel_gauge (Gauge* fuel_gauge);
	void add_gear_indicator (Gear_Indicator* gear_indicator);
	void add_steering_wheel (Steering_Wheel* steering_wheel);
	void add_facade (Vamos_Media::Facade* facade);

	void set_tachometer (double rpm);
	void set_speedometer (double speed);
	void set_fuel_gauge (double fuel);
	void set_gear_indicator (int gear);
	void set_steering_wheel (double angle);

	void draw () const;
  };
}

#endif // not _DASHBOARD_H_
