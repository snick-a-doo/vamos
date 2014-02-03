//  Gl_Car.h - a car that handles graphics, sound and input devices.
//
//  Copyright (C) 2001--2002 Sam Varner
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

#ifndef _GL_CAR_H_
#define _GL_CAR_H_

#include "Car.h"
#include "../media/Sample.h"
#include "../media/Texture_Image.h"

#include <GL/glu.h>

#include <string>
#include <vector>

namespace Vamos_Body
{
  class Dashboard;

  //* An exception thrown when a sound file can't be found.
  class Missing_Sound_File
  {
	// The requested file.
	std::string m_file;

  public:
	//** Constructor
	Missing_Sound_File (std::string file) : m_file (file) {};

	// Return the requested file.
	std::string file () const { return m_file; }
  };



  //* Information about rearview mirrors.
  class Rear_View_Mirror
  {
	const Vamos_Geometry::Three_Vector m_position;
	const double m_width;
	const double m_height;
	const double m_direction;
	const double m_field;
	const double m_near_plane;
	const double m_far_plane;
	Vamos_Media::Texture_Image* mp_mask;

	struct Rectangle
	{
	  Rectangle () : x (0), y (0), width (1), height (1) {}; 
	  int x;
	  int y;
	  int width;
	  int height;
	  double aspect () const { return double (width) / height; }
	};

	Rectangle m_viewport;

	int to_pixels (double range, double factor, double coordinate)
	{ return int (0.5 * range * (1.0 - factor * coordinate)); }

	void set_viewport (int window_width, int window_height, 
					   const Vamos_Geometry::Three_Vector& driver_position,
					   double driver_field_of_view,
                       double pan);
	void activate_viewport ();
	void transform_view () const;
	void draw_mask_shape ();
	unsigned char* make_stencil_buffer ();
	void set_stencil (int window_width, int window_height);

  public:
	Rear_View_Mirror (const Vamos_Geometry::Three_Vector& position,
					  double width, double height,
					  double direction, double field,
					  double near_plane, double far_plane,
					  std::string mask_file);
	~Rear_View_Mirror ();

	void make_mask (int window_width, int window_height, 
					const Vamos_Geometry::Three_Vector& driver_position,
					double driver_field_of_view,
                    double pan);
	void set_view ();

	double get_direction () const { return m_direction; }
	Vamos_Geometry::Three_Vector get_center () const;
  };

  //* A car that handles graphics, sound and input devices.
  class Gl_Car : public Car
  {
  public:
	//** Constructor
	Gl_Car (const Vamos_Geometry::Three_Vector& position,
            const Vamos_Geometry::Three_Matrix& orientation);

	//** Destructor
	virtual ~Gl_Car ();

    // Read the car definition file.
    void read (std::string data_dir = "", std::string car_file = "");

	// Define a sound for the engine.
	virtual void engine_sound (std::string file, 
							   double volume, 
							   double throttle_volume_factor, 
							   double engine_speed_volume_factor,
							   double pitch);

	// Set the 3D models.
	virtual void 
	exterior_model (std::string file, double scale,
					const Vamos_Geometry::Three_Vector& translation,
					const Vamos_Geometry::Three_Vector& rotation);
	virtual void 
	interior_model (std::string file, double scale,
					const Vamos_Geometry::Three_Vector& translation,
					const Vamos_Geometry::Three_Vector& rotation);

	void set_perspective (double aspect);

	void set_view (const Vamos_Geometry::Three_Vector& position,
				   double field_of_view,
				   double near_plane, double far_plane,
				   double pan_angle);

	void add_rear_view (const Vamos_Geometry::Three_Vector& position,
						double width, double height,
						double direction, double field,
						double near_plane, double far_plane,
						std::string mask_file);

	// Set the dashboard.
	void dashboard (Dashboard* dash);

	// Render the car according to its current position and
	// orientation.
	void draw ();
	void draw_interior ();
	void draw_rear_view (double aspect, int index);
	void make_rear_view_mask (int window_width, int window_height);
	void update_rear_view_mask (int window_width, int window_height);
	int get_n_mirrors () const { return m_mirrors.size (); }

	// Perform the transformations for the view.
	void view (double pan, const Vamos_Geometry::Three_Vector& view_position);
	void view ();

    virtual void propagate (double time);
    virtual void set_paused (bool is_paused);

  private:
	double m_throttle_volume_factor;
	double m_engine_speed_volume_factor;

	// The engine sound.
	Vamos_Media::Sample* mp_engine_sample;

	// The 3D car models.
	GLuint m_body_list_id;
	GLuint m_interior_list_id;

	// The gauges and steering wheel.
	Dashboard* mp_dashboard;

	std::vector <Rear_View_Mirror*> m_mirrors;

	// Clipping planes.
	double m_near_plane;
	double m_far_plane;

	// Draw the gauges and readouts.
	void draw_dashboard ();

	// Draw detailed information.
	void draw_dashboard_extras ();

	// Perform the modelview transformations for the car.
	void transform_body ();

    void delete_mirrors ();

	// Return the sound parameters.
	virtual double engine_pitch ();
	virtual double engine_volume ();
  };
}

#endif // not _GL_CAR_H_
