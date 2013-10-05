//  Gl_Car.cc - a car that handles graphics, sound and input devices.
//
//  Copyright (C) 2001--2004 Sam Varner
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

#include "Gl_Car.h"
#include "Fuel_Tank.h"
#include "Dashboard.h"
#include "Wheel.h"
#include "../media/Ac3d.h"
#include "../media/Two_D.h"
#include "../geometry/Numeric.h"
#include "../geometry/Rectangle.h"

#include <GL/glut.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

//* Struct Rear_View_Mirror

//** Constructor
Rear_View_Mirror::Rear_View_Mirror (const Three_Vector& position,
									double width, double height,
									double direction,
									double field,
									double near_plane, double far_plane,
									std::string mask_file) :
  m_position (position),
  m_width (width),
  m_height (height),
  m_direction (direction),
  m_field (field),
  m_near_plane (near_plane),
  m_far_plane (far_plane),
  mp_mask (new Texture_Image (mask_file, false, false))
{
}

//** Destructor
Rear_View_Mirror::~Rear_View_Mirror ()
{
  delete mp_mask;
}

void 
Rear_View_Mirror::activate_viewport ()
{
  glViewport (m_viewport.x, m_viewport.y, m_viewport.width, m_viewport.height);
  glScissor (m_viewport.x, m_viewport.y, m_viewport.width, m_viewport.height);
}

Three_Vector 
Rear_View_Mirror::get_center () const
{
  return Three_Vector (m_position.x,
					   m_position.y - m_width / 2.0,
					   m_position.z + m_height / 2.0);
}

void 
Rear_View_Mirror::transform_view () const
{
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  // Reflect the x-axis.
  glScaled (-1.0, 1.0, 1.0);
  gluPerspective (m_field, m_viewport.aspect (), m_near_plane, m_far_plane);
}

void 
Rear_View_Mirror::set_view ()
{
  activate_viewport ();
  glClear (GL_DEPTH_BUFFER_BIT);
  transform_view ();
}

//* Class Gl_Car

//** Constructor
Gl_Car::Gl_Car (const Three_Vector& position, const Three_Matrix& orientation)
  : Car (position, orientation),
	mp_engine_sample (0),
	m_body_list_id (0),
	m_interior_list_id (0),
	mp_dashboard (0)
{
}

//** Destructor
Gl_Car::~Gl_Car ()
{
  delete mp_engine_sample;

  if (m_body_list_id != 0)
	{
	  glDeleteLists (m_body_list_id, 1);
	}
  if (m_interior_list_id != 0)
	{
	  glDeleteLists (m_interior_list_id, 1);
	}

  delete mp_dashboard;
  delete_mirrors ();
}

void 
Gl_Car::delete_mirrors ()
{
  for (std::vector <Rear_View_Mirror*>::iterator it = m_mirrors.begin ();
	   it != m_mirrors.end ();
	   it++)
	{
	  delete *it;
	}
}

void
Gl_Car::read (std::string data_dir, std::string car_file)
{
  delete_mirrors ();
  m_mirrors.clear ();
  Car::read (data_dir, car_file);
}

void 
Gl_Car::exterior_model (std::string file, double scale, 
						const Three_Vector& translation,
						const Three_Vector& rotation)
{
  if (m_body_list_id != 0)
	{
	  glDeleteLists (m_body_list_id, 1);
	}

   Ac3d model (file, scale, translation, rotation);
   m_body_list_id = model.build ();
}

void 
Gl_Car::interior_model (std::string file, double scale, 
						const Three_Vector& translation,
						const Three_Vector& rotation)
{
  if (m_interior_list_id != 0)
	{
	  glDeleteLists (m_interior_list_id, 1);
	}

   Ac3d model (file, scale, translation, rotation);
   m_interior_list_id = model.build ();
}

void 
Gl_Car::set_perspective (double aspect)
{
  gluPerspective (m_field_of_view, aspect, m_near_plane, m_far_plane);
}

void 
Gl_Car::set_view (const Vamos_Geometry::Three_Vector& position,
				  double field_of_view,
				  double near_plane, double far_plane,
				  double pan_angle)
{
  m_driver_view = position;
  m_field_of_view = field_of_view;
  m_near_plane = near_plane;
  m_far_plane = far_plane;
  m_pan_angle = pan_angle;
}

void 
Gl_Car::add_rear_view (const Vamos_Geometry::Three_Vector& position,
					   double width, double height,
					   double direction, double field,
					   double near_plane, double far_plane,
					   std::string mask_file)
{
  m_mirrors.push_back (new Rear_View_Mirror (position, width, height,
											 direction, field, 
											 near_plane, far_plane,
											 mask_file));
}

void
Gl_Car::update_rear_view_mask (int window_width, int window_height)
{
  if (m_pan_key_control.delta () != 0.0)
    make_rear_view_mask (window_width, window_height);
}

// Fill the stencil buffer for masking the rear-view mirrors.
void 
Gl_Car::make_rear_view_mask (int window_width, int window_height)
{
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();

  glViewport (0, 0, window_width, window_height);
  glScissor (0, 0, window_width, window_height);

  glClearColor (0.0, 0.0, 0.0, 0.0);
  glClearStencil (0);
  glClear (GL_COLOR_BUFFER_BIT 
		   | GL_DEPTH_BUFFER_BIT
		   | GL_STENCIL_BUFFER_BIT);

  const double near_plane = 0.2;
  const double far_plane = 10.0;
  gluPerspective (field_of_view (), double (window_width)/window_height, 
				  near_plane, far_plane);
  view ();
  glMatrixMode (GL_MODELVIEW);
  transform_body ();

  for (std::vector <Rear_View_Mirror*>::iterator it = m_mirrors.begin ();
	   it != m_mirrors.end ();
	   it++)
	{
	  (*it)->make_mask (window_width, window_height,
						m_driver_view, field_of_view (),
                        m_pan_key_control.value ());
	}
}


// Put a mask for one rear-view mirror in the stencil buffer.
void 
Rear_View_Mirror::make_mask (int window_width, int window_height,
							 const Three_Vector& driver_position, 
							 double driver_field_of_view,
                             double pan)
{
  glDisable (GL_LIGHTING);
  set_viewport (window_width, window_height, 
				driver_position, driver_field_of_view, pan);
  draw_mask_shape ();
  set_stencil (window_width, window_height);
  glEnable (GL_LIGHTING);
}


// Find the dimensions for a viewport that's just large enough to hold
// the mirror.
void 
Rear_View_Mirror::set_viewport (int window_width, int window_height, 
								const Three_Vector& driver_position,
								double driver_field_of_view,
                                double pan)
{
  Three_Vector pos = (m_position - driver_position).
    rotate (-deg_to_rad (pan) * Three_Vector::Z);
  const double y_factor = 
	-1.0 / (pos.x * tan (0.5 * deg_to_rad (driver_field_of_view)));
  const double aspect = double (window_width) / window_height;
  const double x_factor = -y_factor / aspect;

  const int x0 = to_pixels (window_width, x_factor, pos.y) - 1;
  m_viewport.x = clip (x0, 0, window_width - 1);
  const int y0 = to_pixels (window_height, y_factor, pos.z) - 1;
  m_viewport.y = clip (y0, 0, window_height - 1);

  const int x1 = to_pixels (window_width, x_factor, pos.y - m_width);
  m_viewport.width = clip (x1, 0, window_width - 1) - m_viewport.x;
  const int y1 = to_pixels (window_height, y_factor, pos.z + m_height);
  m_viewport.height = clip (y1, 0, window_height - 1) - m_viewport.y;
}


// Draw the mask.
void 
Rear_View_Mirror::draw_mask_shape ()
{
  glStencilFunc (GL_ALWAYS, 1, 1);
  glStencilOp (GL_KEEP, GL_KEEP, GL_KEEP);

  mp_mask->activate ();

  glColor3d (1.0, 1.0, 1.0);
  glBegin (GL_QUADS);
  glTexCoord2d (0.0, 1.0);
  glVertex3d (m_position.x, m_position.y, m_position.z);
  glTexCoord2d (1.0, 1.0);
  glVertex3d (m_position.x, m_position.y - m_width, m_position.z);
  glTexCoord2d (1.0, 0.0);
  glVertex3d (m_position.x, m_position.y - m_width, 
			  m_position.z + m_height);
  glTexCoord2d (0.0, 0.0);
  glVertex3d (m_position.x, m_position.y, m_position.z + m_height);
  glEnd ();

  glFlush ();
}


// Use the pixels in the viewport to set the stencil buffer.
void 
Rear_View_Mirror::set_stencil (int window_width, int window_height)
{
  unsigned char* stencil_buffer = make_stencil_buffer ();

  glMatrixMode (GL_PROJECTION);
  glPushMatrix ();
  glLoadIdentity ();
  gluOrtho2D (0.0, double (window_width), 0.0, double (window_height));
  glMatrixMode (GL_MODELVIEW);
  glPushMatrix ();
  glLoadIdentity ();
  glStencilFunc (GL_EQUAL, 1, 1);
  glStencilOp (GL_KEEP, GL_REPLACE, GL_REPLACE);
  glRasterPos2i (m_viewport.x, m_viewport.y);
  glDrawPixels (m_viewport.width, m_viewport.height, GL_STENCIL_INDEX,
  				GL_UNSIGNED_BYTE, stencil_buffer);
  glPopMatrix ();

  glMatrixMode (GL_PROJECTION);
  glPopMatrix ();

  glFinish ();

  delete [] stencil_buffer;
}


// Grab the current buffer as an array of pixels.
unsigned char* 
Rear_View_Mirror::make_stencil_buffer ()
{
  glReadBuffer (GL_BACK);
  const size_t elements = m_viewport.width * m_viewport.height;
  unsigned char* rgba_buffer = new unsigned char [4 * elements];

  glReadPixels (m_viewport.x, m_viewport.y, 
 				m_viewport.width, m_viewport.height, 
 				GL_RGBA, GL_UNSIGNED_BYTE, rgba_buffer);

  unsigned char* buffer = new unsigned char [elements];
  for (size_t i = 0; i < elements; i++)
	{
	  buffer [i] = rgba_buffer [4 * i];
	}
  delete [] rgba_buffer;
  return buffer;
}


void 
Gl_Car::draw_rear_view (double aspect, int index)
{
  Rear_View_Mirror* mirror = m_mirrors [index];
  mirror->set_view ();
  view (mirror->get_direction (), mirror->get_center ());
}

// Set the dashboard.
void 
Gl_Car::dashboard (Dashboard* dash)
{
  delete mp_dashboard;
  mp_dashboard = dash;
}

void 
Gl_Car::transform_body ()
{
  // Set the modelview matrix to be the identity matrix
  glLoadIdentity ();
  
  // Translate and rotate the graphical representation of the object
  Three_Vector position = m_chassis.position ();
  glTranslatef (position.x, position.y, position.z);
  
  double angle;
  Three_Vector axis = m_chassis.axis_angle (&angle);
  glRotatef (angle, axis.x, axis.y, axis.z);
}

// Render the car according to its current position and orientation.
void 
Gl_Car::draw ()
{
  if (m_body_list_id != 0)
	{
	  transform_body ();

	  // Draw the body.
      glCallList (m_body_list_id);

	  // Draw the wheels.
	  std::for_each (m_wheels.begin (), m_wheels.end (),
					 std::mem_fun (&Wheel::draw));
    }
}

void 
Gl_Car::draw_interior ()
{
  glCallList (m_interior_list_id);
  if (mp_dashboard)
    draw_dashboard ();
}

void 
Gl_Car::draw_dashboard ()
{
  mp_dashboard->set_tachometer (rad_s_to_rpm (engine ()->rotational_speed ()));
  mp_dashboard->set_speedometer (m_s_to_km_h (wheel (2)->speed ()));
  mp_dashboard->set_fuel_gauge (fuel_tank ()->fuel ());
  mp_dashboard->set_gear_indicator (transmission ()->gear ());
  mp_dashboard->set_steering_wheel (m_steer_key_control.value ());

  mp_dashboard->draw ();
  if (m_show_dashboard_extras)
    {
      draw_dashboard_extras ();
    }
}

void 
Gl_Car::draw_dashboard_extras ()
{ 
  Two_D screen;
  
  const double bottom = 2;
  const double top = 20;

  std::string gear = "N";
  if (transmission ()->gear () == -1)
    gear = "R";
  else if (transmission ()->gear () > 0)
    gear = std::string (1, transmission ()->gear () + '0');

  screen.text (4, 18, "RPM", rad_s_to_rpm (engine ()->rotational_speed ()));
  screen.text (4, 14, "Torque", engine ()->drive_torque (), "Nm");
  screen.text (4, 10, "Speed", m_s_to_km_h (wheel (2)->speed ()), "km/h");
  screen.text (4, 6, "Mass", m_chassis.mass (), "kg");
  screen.text (4, 2, "Gear", gear);

  screen.bar (Rectangle (19, bottom, 2, top - bottom),
              0.0, 1.0, 1.0,
              brake_fraction ());

  screen.bar (Rectangle (22, bottom, 2, top - bottom),
              1.0, 0.0, 1.0,
              throttle_fraction ());

  screen.text (28, 18, "Tire Temperature, Wear, Grip");
  //!fixme Assume wheels are defined in the order: right front, left front,
  // right rear, left rear.
  screen.text (28, 14, "", m_wheels [1]->temperature (), "", 0);
  screen.text (32, 14, "", m_wheels [1]->wear (), "", 2);
  screen.text (36, 14, "", m_wheels [1]->grip (), "", 2);
  screen.text (40, 14, "", m_wheels [0]->temperature (), "", 0);
  screen.text (44, 14, "", m_wheels [0]->wear (), "", 2);
  screen.text (48, 14, "", m_wheels [0]->grip (), "", 2);
  screen.text (28, 10, "", m_wheels [3]->temperature (), "", 0);
  screen.text (32, 10, "", m_wheels [3]->wear (), "", 2);
  screen.text (36, 10, "", m_wheels [3]->grip (), "", 2);
  screen.text (40, 10, "", m_wheels [2]->temperature (), "", 0);
  screen.text (44, 10, "", m_wheels [2]->wear (), "", 2);
  screen.text (48, 10, "", m_wheels [2]->grip (), "", 2);

  screen.text (28, 6, "Fuel", mp_fuel_tank->fuel (), "L", 1);
  screen.text (28, bottom, "Air Density", m_air_density, "kg/m^3", 3);
}

// Perform the transformations for the driver's view.
void
Gl_Car::view ()
{
  // Called for the front view.
  view (m_pan_key_control.value (), m_driver_view);
  const double pan = deg_to_rad (m_pan_key_control.value ());

  Three_Vector z = m_chassis.rotate_to_world (Three_Vector (0.0, 0.0, 1.0));
  Three_Vector x = m_chassis.rotate_to_world
    (Three_Vector (1.0, 0.0, 0.0).rotate (pan * Three_Vector::Z));
  float at_up [6] = { float (x.x), float (x.y), float (x.z),
                      float (z.x), float (z.y), float (z.z) };

  Three_Vector pos = view_position ();
  alListener3f (AL_POSITION, pos.x, pos.y, pos.z);
  alListenerfv (AL_ORIENTATION, at_up); 

  if (engine ())
    {
      Three_Vector v = m_chassis.velocity (engine ()->position ());
      const double v_s = alGetDouble (AL_SPEED_OF_SOUND);
      alListener3f (AL_VELOCITY, v.x / v_s, v.y / v_s, v.z / v_s);
    }
}

void 
Gl_Car::view (double pan, const Three_Vector& view_position)
{
  // Find the angle-axis representation of the car's orientation.
  // Called for the front view and rear views.
  double angle;
  Three_Vector axis = m_chassis.axis_angle (&angle);

  // Rotate the view.
  glRotated (90, 0.0, 1.0, 0.0);
  glRotated (-90, 1.0, 0.0, 0.0);
  glRotated (-angle, axis.x, axis.y, axis.z);

  Three_Vector z = m_chassis.rotate_to_world (Three_Vector (0.0, 0.0, 1.0));
  glRotated (-pan, z.x, z.y, z.z);

  Three_Vector pos = -m_chassis.transform_to_world (view_position);
  glTranslated (pos.x, pos.y, pos.z);
}

void 
Gl_Car::engine_sound (std::string file, 
					  double volume, 
					  double throttle_volume_factor, 
					  double engine_speed_volume_factor,
					  double pitch)
{
  delete mp_engine_sample;
  mp_engine_sample = 0;

  if (file != "")
	{
	  m_throttle_volume_factor = throttle_volume_factor;
	  m_engine_speed_volume_factor = engine_speed_volume_factor;
      mp_engine_sample = new Sample (file, volume, pitch, true);
	}
}

double 
Gl_Car::engine_pitch ()
{
  return engine ()->rotational_speed ();
}

double 
Gl_Car::engine_volume ()
{
  return 1.0 + m_throttle_volume_factor * engine ()->throttle ()
	+ m_engine_speed_volume_factor * engine ()->rotational_speed ();
}

void
Gl_Car::propagate (double time)
{
  Car::propagate (time);

  if (mp_engine_sample != 0)
    {
      mp_engine_sample->pitch (engine_pitch ());
      mp_engine_sample->volume (engine_volume ());
      mp_engine_sample->position (chassis ().position (engine ()));
      mp_engine_sample->velocity (chassis ().velocity (engine ()));
      mp_engine_sample->play ();
    }
}

void
Gl_Car::set_paused (bool is_paused)
{
  if (mp_engine_sample != 0)
    {
      if (is_paused)
        mp_engine_sample->pause ();
      else
        mp_engine_sample->play ();
    }
}
