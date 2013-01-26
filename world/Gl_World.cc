//  Vamos Automotive Simulator
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

#include "../body/Car.h"
#include "../body/Fuel_Tank.h"
#include "../body/Wheel.h"
#include "../geometry/Conversions.h"
#include "../track/Track.h"
#include "Gl_World.h"
#include "Sounds.h"
#include "Interactive_Driver.h"
#include "Timing_Info.h"

#include <SDL.h>
#include <GL/glut.h>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <cctype>
#include <cassert>
#undef min

using namespace Vamos_Geometry;
using namespace Vamos_World;

//-----------------------------------------------------------------------------
static std::string 
format_time (double time)
{
  if (time == Timing_Info::NO_TIME) return "";

  int minutes = int (time / 60.0);
  double seconds = time - 60 * minutes;

  std::ostringstream os;
  os << minutes << ':' 
     << std::fixed << std::setfill ('0') << std::setw (6) << std::setprecision (3)
     << seconds;
  return os.str ();
}

static std::string 
format_time_difference (double delta_time)
{
  if (delta_time == Timing_Info::NO_TIME) return "";

  std::ostringstream os;
  if (delta_time > 0.0)
      os << '+';

  os << std::fixed << std::setprecision (3) << delta_time;
  return os.str ();
}

//-----------------------------------------------------------------------------
Gl_Window::Gl_Window (int width, int height, const char* name, bool full_screen)
  : m_video_flags (SDL_OPENGL | SDL_RESIZABLE | SDL_DOUBLEBUF)
{
  SDL_GL_SetAttribute (SDL_GL_STENCIL_SIZE, 1);
  if (full_screen)
    {
      m_video_flags |= SDL_FULLSCREEN;
      SDL_Rect** modes = SDL_ListModes (0, m_video_flags);
      if ((modes != 0) || (modes [0] != 0))
        {
          width = modes [0]->w;
          height = modes [0]->h;
        }
    }

  SDL_ShowCursor (false);
  SDL_WM_SetCaption (name, name);
  resize (width, height);
}


Gl_Window::~Gl_Window ()
{
}

void 
Gl_Window::resize (int width, int height)
{
  m_width = width;
  m_height = height;
  m_aspect = (m_height == 0) ? 1.0 : double (m_width) / m_height; 
  glViewport (0, 0, m_width, m_height);
  if (SDL_SetVideoMode (width, height, 0, m_video_flags) == 0)
    throw No_SDL_Screen (width, height, 0, m_video_flags);
}

//-----------------------------------------------------------------------------
Timer::Timer (Tick interval, Tick fixed_time_step) :
  m_timeout (interval),
  m_frame_step (0.001),
  m_is_paused (false),
  m_fixed_time_step (fixed_time_step),
  m_use_fixed_time_step (false)
{
  reset ();
}

void
Timer::reset ()
{
  start_averaging ();
  // Pretend that the simulation was paused until now.
  m_pause_ticks = m_start_ticks;
  m_fixed_time = 0;
}

void 
Timer::update ()
{
  if (m_is_paused) return;

  m_current_ticks = SDL_GetTicks ();

  if (m_use_fixed_time_step)
    m_fixed_time += m_fixed_time_step;

  const unsigned elapsed = m_current_ticks - m_start_ticks;
  if ((elapsed > m_timeout) && (m_frames > 0))
    {
      m_frame_step = ticks_to_seconds (elapsed) / m_frames;
      start_averaging ();
    }
}

void 
Timer::start_averaging ()
{
  m_start_ticks = SDL_GetTicks ();
  m_frames = 0;
}

void 
Timer::set_paused (bool is_paused)
{
  m_is_paused = is_paused;
  if (!is_paused)
    {
      start_averaging ();
      m_pause_ticks += m_start_ticks - m_current_ticks; 
      update ();
    }
}

void 
Timer::use_fixed_time_step (bool use) 
{ 
  if (!use)
    {
      start_averaging ();
      update ();
    }
  m_use_fixed_time_step = use; 
}

//-----------------------------------------------------------------------------
Gl_World::Gl_World (int argc, char** argv,
                    Vamos_Track::Strip_Track* track, 
                    Atmosphere* atmosphere,
                    Sounds* sounds,
                    bool full_screen,
                    bool show_mirror_views)
  : World (track, atmosphere),
    m_timer (100, 10),
    m_show_mirror_views (show_mirror_views),
    mp_sounds (sounds),
    mp_window (0),
    m_view (MAP_VIEW),
    m_update_graphics (true),
    m_done (false)
{
  initialize_graphics (&argc, argv);
  mp_window = new Gl_Window (900, 600, argv [0], full_screen);
  reshape (mp_window->width (), mp_window->height ());
  set_attributes ();
  set_paused (true);
}

Gl_World::~Gl_World ()
{
  delete mp_window;
}

void 
Gl_World::initialize_graphics (int* argc, char** argv)
{
  glutInit (argc, argv);
  if (SDL_Init (SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_JOYSTICK) != 0) 
    throw Can_Not_Intialize_SDL (SDL_GetError ());
  atexit (SDL_Quit);
  SDL_JoystickOpen (0);
}

void 
Gl_World::set_attributes ()
{
  // Enable depth testing for hidden line removal
  glEnable (GL_DEPTH_TEST);
  // Allow transparency.
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // Allow textures.
  glEnable (GL_TEXTURE_2D);
  // Allow viewports.
  glEnable (GL_SCISSOR_TEST);
  // Allow masking.
  if (m_show_mirror_views)
    {
      glEnable (GL_STENCIL_TEST);
      glClearStencil (0);
    }
  // Allow lighting.
  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);
  // Create a Directional Light Source (x, y, z, w).
  GLfloat position [] = { 0.0, -1.0, 1.0, 0.0 };
  glLightfv (GL_LIGHT0, GL_POSITION, position);
  // Set the ambient light (R, G, B, A).
  GLfloat ambient [] = { 0.7, 0.7, 0.7, 1.0 };
  glLightfv (GL_LIGHT0, GL_AMBIENT, ambient);
  glClearColor (0.32, 0.65, 0.34, 0.0);
}

void 
Gl_World::add_car (Vamos_Body::Car* car, 
                   Driver* driver,
                   const Vamos_Track::Road& road,
                   bool controlled)
{
  if (car == 0) return;
  World::add_car (car, driver, road, controlled);
  m_view = BODY_VIEW;
}

// Read the definition file.
void 
Gl_World::read (std::string data_dir, 
                std::string world_file,
                std::string controls_file)
{
  // Remember the file name for re-reading.
  if (data_dir != "")
    {
      m_data_dir = data_dir;
    }
  if (world_file != "")
    {
      m_world_file = world_file;
    }
  if (controls_file != "")
    {
      m_controls_file = controls_file;
    }

  World_Reader w_reader (m_world_file, this);
  Controls_Reader c_reader (m_controls_file, this);
}

void 
Gl_World::set_paused (bool is_paused)
{
  m_timer.set_paused (is_paused);
  m_paused = is_paused;

  for (std::vector <Car_Information>::iterator it = m_cars.begin ();
       it != m_cars.end ();
       it++)
    {
      it->car->set_paused (is_paused);
    }

  if ((mp_sounds != 0) && is_paused)
    mp_sounds->pause ();
}

bool 
Gl_World::pause (double, double)
{
  set_paused (!m_paused);
  return true;
}

bool 
Gl_World::quit (double, double)
{
  m_done = true;
  return true;
}

bool 
Gl_World::read_car (double, double)
{
  if (controlled_car () != 0)
    {
      controlled_car ()->car->read ();
      controlled_car ()->car->make_rear_view_mask (mp_window->width (), 
                                                   mp_window->height ());
    }
  return true;
}

bool 
Gl_World::read_track (double, double)
{
  mp_track->read ();
  display ();
  return true;
}

bool 
Gl_World::read_world (double, double)
{
  read ();
  return true;
}

bool 
Gl_World::reset_car (double, double)
{
  World::reset ();
  return true;
}

bool 
Gl_World::restart_car (double, double)
{
  World::restart ();
  return true;
}

bool 
Gl_World::cycle_view (double, double)
{
  switch (m_view)
    {
    case BODY_VIEW:
      m_view = CHASE_VIEW;
      glClearColor (0.32, 0.65, 0.34, 0.0);
      break;
    case CHASE_VIEW:
      m_view = MAP_VIEW;
      break;
    case MAP_VIEW:
      if (focused_car () != 0)
        m_view = WORLD_VIEW;
      break;
    case WORLD_VIEW:
    default:
      m_view = BODY_VIEW;
      break;
    }

  return true;
}

bool
Gl_World::toggle_graphics (double, double)
{
  m_update_graphics = !m_update_graphics;
  m_timer.use_fixed_time_step (!m_update_graphics);
  return true;
}

bool 
Gl_World::focus_next_car (double, double)
{
  focus_other_car (1);
  return true;
}

bool 
Gl_World::focus_previous_car (double, double)
{
  focus_other_car (-1);
  return true;
}

bool
Gl_World::replay (double, double)
{
  set_paused (true);

  if (m_cars [0].m_record.size () == 0)
    return true;

  double real_time = m_cars [0].m_record [0].m_time;
  Tick last_ticks = SDL_GetTicks ();

  for (size_t i = 0; i < m_cars [0].m_record.size (); i++)
    {
      std::vector <Car_Information>::iterator it = m_cars.begin ();
      const double time = it->m_record [i].m_time;

      for (; it != m_cars.end (); it++)
        {
          const Car_Information::Record& record = it->m_record [i];
          it->car->chassis ().set_position (record.m_position);
          it->car->chassis ().set_orientation (record.m_orientation);
        }

      if (time >= real_time)
        display ();
      check_for_events ();

      Tick now = SDL_GetTicks ();
      real_time += 0.001 * (now - last_ticks);
      last_ticks = now;
    }
  return true;
}

// The main loop of the simulation
void 
Gl_World::animate ()
{
  if (focused_car () != 0)
    {
      for (int loop = 0; loop < m_timer.steps_per_frame (); loop++)
        {
          propagate_cars (m_timer.get_time_step ());
        }
      play_sounds ();
      update_car_timing ();
   }
  m_timer.add_frame ();
}

void
Gl_World::update_car_timing ()
{
  for (size_t i = 0; i < m_cars.size (); i++)
    {
      Car_Information& car = m_cars [i];
      if (!car.driver->is_started ())
        car.driver->start (mp_timing->countdown ());
      const double distance = 
        mp_track->track_coordinates (car.car->chassis ().position (), 
                                     car.road_index,
                                     car.segment_index).x;
      const int sector = mp_track->sector (distance);
      mp_timing->update (m_timer.get_current_time (), i, distance, sector);
    }
}

void
Gl_World::play_sounds ()
{
  if (mp_sounds == 0) return;

  double tire_slide = 0.0;
  double kerb_speed = 0.0;
  double grass_speed = 0.0;
  double gravel_speed = 0.0;
  double scrape_speed = 0.0;
  double hard_crash_speed = 0.0;
  double soft_crash_speed = 0.0;
  
  for (std::vector <Interaction_Info>::const_iterator 
         it = m_interaction_info.begin ();
       it != m_interaction_info.end ();
       it++)
    {
      if (it->car != focused_car ()->car) continue;

      switch (it->track_material)
        {
        case Material::ASPHALT:
        case Material::CONCRETE:
        case Material::METAL:
          if (it->car_material == Material::RUBBER)
            {
              tire_slide = it->car->slide ();
            }
          else if (it->car_material == Material::METAL)
            {
              scrape_speed = it->parallel_speed;
              hard_crash_speed = it->perpendicular_speed;
            }
          break;
        case Material::KERB:
          kerb_speed = it->parallel_speed;
          break;
        case Material::GRASS:
          grass_speed = it->parallel_speed;
          break;
        case Material::GRAVEL:
          gravel_speed = it->parallel_speed;
          break;
        case Material::RUBBER:
          soft_crash_speed = it->perpendicular_speed;
          break;
        default:
          break;
        }
    }

  m_interaction_info.clear ();
  const Three_Vector& p = focused_car ()->car->chassis ().position ();

  mp_sounds->play_tire_squeal_sound (tire_slide, p);
  mp_sounds->play_kerb_sound (kerb_speed, p);
  mp_sounds->play_grass_sound (grass_speed, p);
  mp_sounds->play_gravel_sound (gravel_speed, p);
  mp_sounds->play_scrape_sound (scrape_speed, p);
  mp_sounds->play_wind_sound ((focused_car ()->car->chassis ().
                               cm_velocity () 
                               - mp_atmosphere->velocity ()).magnitude (),
                              p);
  mp_sounds->play_hard_crash_sound (hard_crash_speed, p);
  mp_sounds->play_soft_crash_sound (soft_crash_speed, p);
}

void 
Gl_World::show_full_window ()
{
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();

  glViewport (0, 0, mp_window->width (), mp_window->height ());
  glScissor (0, 0, mp_window->width (), mp_window->height ());
  glStencilFunc (GL_ALWAYS, 1, 1);
  glStencilOp (GL_KEEP, GL_KEEP, GL_KEEP);

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void 
Gl_World::draw_mirror_views ()
{
  for (int i = 0; i < focused_car ()->car->get_n_mirrors (); i++)
    {
      focused_car ()->car->draw_rear_view (mp_window->aspect (), i);

      glMatrixMode (GL_MODELVIEW);

      // Enable the rearview mirror mask.
      glStencilFunc (GL_EQUAL, 1, 1);

      // Front and back are reversed in the mirrors, so cull front
      // faces while we're drawing mirror views.
      glPushAttrib (GL_POLYGON_BIT);
      {
        glCullFace (GL_FRONT);
        mp_track->draw_sky (focused_car ()->car->view_position ());
        mp_track->draw ();
        draw_cars (false, false);
      }
      glPopAttrib ();
    }
}

void 
Gl_World::set_car_view (Vamos_Body::Car* car)
{
  assert (car != 0);
  car->set_perspective (mp_window->aspect ());
  car->view ();
}

void 
Gl_World::set_world_view (const Vamos_Geometry::Three_Vector& camera_position,
                          const Vamos_Geometry::Three_Vector& target_position,
                          double vertical_field_angle)
{
  gluPerspective (vertical_field_angle, mp_window->aspect (), 10.0, 1000.0);
  gluLookAt (camera_position.x, camera_position.y, camera_position.z,
             target_position.x, target_position.y, target_position.z,
             0.0, 0.0, 1.0); // up direction

  Three_Vector direction (target_position - camera_position);
  float at_up [6] = { direction.x, direction.y, direction.z, 0.0, 0.0, 1.0 };

  alListener3f (AL_POSITION, 
                camera_position.x, camera_position.y, camera_position.z);
  alListener3f (AL_VELOCITY, 0.0, 0.0, 0.0);
  alListenerfv (AL_ORIENTATION, at_up);
}

void 
Gl_World::set_world_view (const Vamos_Track::Camera& camera)
{
  set_world_view (mp_track->camera_position (camera),
                  camera.fixed ? mp_track->camera_target (camera)
                  : focused_car ()->car->chassis ().cm_position (),
                  camera.vertical_field_angle);
}

void 
Gl_World::draw_track (bool draw_sky, const Three_Vector& view_position)
{
  glMatrixMode (GL_MODELVIEW);
  if (draw_sky)
    {
      assert (focused_car () != 0);
      mp_track->draw_sky (view_position);
    }
  else
    {
      mp_track->draw_map_background ();
    }
  mp_track->draw ();
}

void 
Gl_World::draw_cars (bool draw_interior, bool draw_focused_car)
{
  for (std::vector <Car_Information>::iterator it = m_cars.begin ();
       it != m_cars.end ();
       it++)
    {
      assert (it->car != 0);
      if (it->car != focused_car ()->car)
        {
          it->car->draw ();
          if (it->driver != 0)
            it->driver->draw ();
        }
    }
  if (draw_focused_car)
    {
      focused_car ()->car->draw ();
      if (draw_interior)
        {
          focused_car ()->car->draw_interior ();
        }
      if (focused_car ()->driver != 0)
        focused_car ()->driver->draw ();
    }
}

void 
Gl_World::show_scene ()
{
  glFlush ();
  SDL_GL_SwapBuffers ();
}

void 
Gl_World::display ()
{
  if (m_view == BODY_VIEW)
    focused_car ()->car->update_rear_view_mask (mp_window->width (), 
                                                mp_window->height ());
  show_full_window ();

  switch (m_view)
    {
    case BODY_VIEW:
      set_car_view (focused_car ()->car);
      draw_track (true, focused_car ()->car->view_position ());
      draw_cars (true);
      draw_timing_info ();
      if (m_show_mirror_views)
        draw_mirror_views ();
      break;
    case CHASE_VIEW:
      {
        const Vamos_Body::Car& car = *focused_car ()->car;
        Three_Vector chase_pos = car.chase_position ();
        set_world_view (chase_pos, car.chassis ().cm_position (), 45.0);
        draw_track (true, chase_pos);
        draw_cars (true);
        draw_timing_info ();
      }
      break;
    case MAP_VIEW:
      m_map.set_view ();
      draw_track (false);
      if (focused_car () != 0)
        {
          draw_cars (false);
          draw_timing_info ();
        }
      break;
    case WORLD_VIEW:
      {
        if (focused_car () != 0)
          {
            const Vamos_Track::Camera& camera =
              mp_track->get_camera (mp_timing->timing_at_index (m_focused_car_index)
                                    .lap_distance ());
            set_world_view (camera);
            draw_track (true, mp_track->camera_position (camera));
          }
        draw_cars (true);
        draw_timing_info ();
      }
      break;
    }

  show_scene ();
}

void 
Gl_World::reshape (int width, int height)
{
  mp_window->resize (width, height);
  m_mouse.set_axis_range (X, 0, width);
  m_mouse.set_axis_range (Y, 0, height);
  if (focused_car () != 0)
    {
      focused_car ()->car->make_rear_view_mask (width, height);
    }
  m_map.set_bounds (*mp_track, *mp_window);
}

void 
Gl_World::draw_string (const std::string& str, double x, double y)
{
  glRasterPos2d (x, y);
  for (std::string::const_iterator it = str.begin (); it != str.end (); it++)
    {
      glutBitmapCharacter (GLUT_BITMAP_8_BY_13, *it);
    }
}

void 
Gl_World::draw_timing_info ()
{
  glDisable (GL_DEPTH_TEST);
  glDisable (GL_LIGHTING);
  glDisable (GL_TEXTURE_2D);

  glColor3f (1.0, 1.0, 1.0);
  
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();
  
  std::ostringstream b_stream;
  double x = 55;

  gluOrtho2D (0, 100, 0, 100);

  if (!mp_track->has_starting_lights ())
    set_starting_lights ();

  if (mp_timing->running_order ().size () > 1 )
    draw_leaderboard ();
  else
    draw_lap_times ();

  const Timing_Info::Car_Timing& car = mp_timing->timing_at_index (m_focused_car_index);
  b_stream << "Lap Time " << format_time (car.lap_time ());
  draw_string (b_stream.str (), x, 14);

  b_stream.str ("");
  b_stream << "    Last " << format_time (car.previous_lap_time ()) 
           << " " 
           << format_time_difference (car.lap_time_difference ());
  draw_string (b_stream.str (), x, 10);

  b_stream.str ("");
  b_stream << "    Best " << format_time (car.best_lap_time ());
  draw_string (b_stream.str (), x, 6);

  b_stream.str ("");
  b_stream << int (m_timer.get_frame_rate () + 0.5) << " frame/s";
  draw_string (b_stream.str (), x, 2);

  x = 75;

  size_t sector = car.current_sector ();
  b_stream.str ("");
  b_stream << "   Sector ";
  b_stream << sector << " " << format_time (car.sector_time ());
  draw_string (b_stream.str (), x, 14);

  b_stream.str ("");
  b_stream << "       Best "; 
  if (sector != 0)
    {
      b_stream << format_time (car.best_sector_time ());
    }
  draw_string (b_stream.str (), x, 10);

  b_stream.str ("");
  b_stream << "Last Sector ";
  if (car.previous_sector () != 0)
    {
      b_stream << format_time (car.previous_sector_time ()) << "  " 
               << format_time_difference 
        (car.previous_sector_time_difference ());
    }
  draw_string (b_stream.str (), x, 6);

  b_stream.str ("");
  b_stream << "Distance " << int (car.lap_distance ()) << " m";
  draw_string (b_stream.str (), x, 2);

  glEnable (GL_DEPTH_TEST);
  glEnable (GL_LIGHTING);
  glEnable (GL_TEXTURE_2D);
}

void
Gl_World::set_starting_lights ()
{
  const int count = mp_timing->countdown ();
  if (count == 0)
    return;

  glPushAttrib (GL_CURRENT_BIT);
  glPushMatrix ();
  glLoadIdentity ();

  const int width = mp_window->width ();
  const int height = mp_window->height ();
  gluOrtho2D (0, width, 0, height);

  const double r = 0.01*height;
  const int n_lights = 5;
  glTranslatef (0.5*width + 6*r, 0.6*height, 0.0);

  GLUquadricObj* disk = gluNewQuadric();
  gluQuadricDrawStyle (disk, GLU_FILL);

  glColor3f (0.2, 0.2, 0.2);
  for (int light = 1; light <= n_lights; light++)
    {
      if (light >= count)
        glColor3f (0.9, 0.0, 0.0);
      glTranslatef (-2.5*r, 0.0, 0.0);
      gluDisk (disk, 0.0, r, 32, 32);
    }

  glColor3f (1.0, 0.0, 0.0);

  gluDeleteQuadric (disk);

  glPopMatrix ();
  glPopAttrib ();
}

void
Gl_World::draw_leaderboard ()
{
  double x = 2;
  double y = 95;
  std::ostringstream b_stream;

  const Timing_Info::Running_Order& order = mp_timing->running_order ();
  Timing_Info::Running_Order::const_iterator it = order.begin ();

  const size_t lap = (*it)->current_lap ();
  const size_t total_laps = mp_timing->total_laps ();
  if (lap <= total_laps)
    b_stream << "Lap " << lap << '/' << total_laps;
  else
    b_stream << "Finish";
  draw_string (b_stream.str (), x, y);

  y -= 3;
  b_stream.str ("");
  b_stream << m_cars [(*it)->grid_position () - 1].car->name () << ' ';
  double time = (*it)->previous_lap_time ();
  if (time != Timing_Info::NO_TIME)
    b_stream << format_time (time);
  draw_string (b_stream.str (), x, y);

  while (++it != order.end ())
    {
      y -= 3;
      b_stream.str ("");
      b_stream << m_cars [(*it)->grid_position () - 1].car->name () << ' ';
      double interval = (*it)->interval ();
      if (interval != Timing_Info::NO_TIME)
        b_stream << format_time_difference (interval);
      draw_string (b_stream.str (), x, y);
    }

  y -= 3;
  b_stream.str ("");
  b_stream << "Fastest Lap"; 
  draw_string (b_stream.str (), x, y);
  y -= 2;
  b_stream.str ("");
  const Timing_Info::Car_Timing* p_fastest = mp_timing->fastest_lap_timing ();
  if (p_fastest)
    {
      time = p_fastest->best_lap_time ();
      if (time != Timing_Info::NO_TIME)
        b_stream << m_cars [p_fastest->grid_position () - 1].car->name () << ' '
                 << format_time (p_fastest->best_lap_time ());
    }
  draw_string (b_stream.str (), x, y);
}

void
Gl_World::draw_lap_times ()
{
  double x = 85;
  double y = 95;
  std::ostringstream b_stream;

  const Timing_Info::Running_Order& order = mp_timing->running_order ();
  Timing_Info::Running_Order::const_iterator it = order.begin ();

  static std::vector <double> a_time;
  const double lap_time = (*it)->previous_lap_time ();
  const size_t lap = (*it)->current_lap ();
  if ((lap_time != Timing_Info::NO_TIME) 
      && (lap > 0)
      && (lap - 1) > a_time.size ())
    a_time.push_back (lap_time);

  for (size_t i = 0; i < a_time.size (); i++)
    {
      b_stream.str ("");
      b_stream << i + 1 << ' ' << format_time (a_time [i]);
      draw_string (b_stream.str (), x, y);
      y -= 3;
    }
}

void 
Gl_World::start (size_t laps)
{
  World::start (laps);
  m_map.set_bounds (*mp_track, *mp_window);
  if (!m_cars.empty ())
    set_paused (false);
  m_timer.reset ();

  SDL_Event event;

  // Flush the event queue.
  while (SDL_PollEvent (&event))
    ;

  while (!m_done)
    {
      m_timer.update ();
      check_for_events ();

      if (m_paused)
        {
          if (m_key_repeat.is_active ())
              m_key_repeat.repeat ();
          else
            SDL_Delay (100);
        }
      else
        {
          SDL_Delay (0);
          animate ();
        }

      if (m_update_graphics)
          display ();
    }
}

void
Gl_World::check_for_events ()
{
  SDL_Event event;
  while (SDL_PollEvent (&event))
    {
      Interactive_Driver* driver = 0;
      if (controlled_car () != 0)
        {
          driver = dynamic_cast <Interactive_Driver*> (controlled_car ()->driver);
          if (driver == 0)
            continue;
        }

      switch (event.type) 
        {
        case SDL_JOYAXISMOTION:
          if (driver != 0)
            driver->m_joystick.move (event.jaxis.axis, event.jaxis.value);
          break;
        case SDL_JOYBUTTONDOWN:
          if (driver != 0)
            driver->m_joystick.press (event.jbutton.button + 1);
          break;
        case SDL_JOYBUTTONUP:
          if (driver != 0)
            driver->m_joystick.release (event.jbutton.button + 1);
          break;
        case SDL_KEYDOWN:
          m_keyboard.press (event.key.keysym.sym);
          if (driver != 0)
            driver->m_keyboard.press (event.key.keysym.sym);
          if (m_view == MAP_VIEW)
            {
              m_key_repeat.set (event, 100);
              m_map.m_keyboard.press (event.key.keysym.sym);
            }
          break;                
        case SDL_KEYUP:
          m_keyboard.release (event.key.keysym.sym);
          m_key_repeat.cancel ();
          if (driver != 0)
            driver->m_keyboard.release (event.key.keysym.sym);
          if (m_view == MAP_VIEW)
            m_map.m_keyboard.release (event.key.keysym.sym);
          break;
        case SDL_MOUSEMOTION:
          if (driver != 0)
            {
              driver->m_mouse.move (X, event.motion.x);
              driver->m_mouse.move (Y, event.motion.y);
            }
          if (m_view == MAP_VIEW)
            {
              m_map.m_mouse.move (X, event.motion.x);
              m_map.m_mouse.move (Y, event.motion.y);
            }
          break;
        case SDL_MOUSEBUTTONDOWN:
          if (driver != 0)
            driver->m_mouse.press (event.button.button);
          if (m_view == MAP_VIEW)
            m_map.m_mouse.press (event.key.keysym.sym);
          break;
        case SDL_MOUSEBUTTONUP:
          if (driver != 0)
            driver->m_mouse.release (event.button.button);
          if (m_view == MAP_VIEW)
            m_map.m_mouse.release (event.key.keysym.sym);
          break;
        case SDL_VIDEORESIZE:
          reshape (event.resize.w, event.resize.h);
          break;
        case SDL_QUIT:
          quit ();
          break;
        }
    }
}
void 
Gl_World::set_focused_car (size_t index)
{
  World::set_focused_car (index);

  if (focused_car () != 0)
    {
      focused_car ()->car->make_rear_view_mask (mp_window->width (), 
                                                mp_window->height ());
    }
}

void 
Gl_World::Event_Repeat::set (const SDL_Event& event, int delay_ms)
{
  if (!m_is_active)
    m_repeat_count = 0;

  m_is_active = true;
  m_event = event;
  m_delay_ms = delay_ms;
}

void
Gl_World::Event_Repeat::repeat ()
{
  if (m_is_active)
    {
      m_repeat_count++;
      SDL_Delay (m_delay_ms);
      SDL_PushEvent (&m_event);
    }
}

Map::Map ()
{
  m_keyboard.bind_action (SDLK_RIGHT, DOWN, this, (Callback_Function)&Map::pan, RIGHT);
  m_keyboard.bind_action (SDLK_LEFT, DOWN, this, (Callback_Function)&Map::pan, LEFT);
  m_keyboard.bind_action (SDLK_UP, DOWN, this, (Callback_Function)&Map::pan, UP);
  m_keyboard.bind_action (SDLK_DOWN, DOWN, this, (Callback_Function)&Map::pan, DOWN);

  m_keyboard.bind_action ('=', DOWN, this, (Callback_Function)&Map::zoom, IN);
  m_keyboard.bind_action ('+', DOWN, this, (Callback_Function)&Map::zoom, IN);
  m_keyboard.bind_action ('-', DOWN, this, (Callback_Function)&Map::zoom, OUT);
  m_keyboard.bind_action ('_', DOWN, this, (Callback_Function)&Map::zoom, OUT);

  for (char c = '1'; c <= '9'; c++)
    m_keyboard.bind_action (c, DOWN, this, (Callback_Function)&Map::set_zoom, 
                            c - '1' + 1);
}

void
Map::set_bounds (const Vamos_Track::Strip_Track& track,
                 const Gl_Window& window)
{
  // Adjust the mins and maxes to keep the correct aspect ratio of the
  // track regardless of the window's size.
  m_bounds = track.bounds ();

  double ratio = m_bounds.aspect () / window.aspect ();
  if (ratio < 1.0)
    {
      // The window is wider than the track.  Stretch the x-dimension.
      m_bounds.scale (1.0 / ratio, 1.0);
    }
  else
    {
      // The track is wider than the window.  Stretch the y-dimension.
      m_bounds.scale (1.0, ratio);
    }
  m_initial_bounds = m_bounds;
}

void 
Map::set_view ()
{
  glOrtho (m_bounds.left (), m_bounds.right (), m_bounds.bottom (), m_bounds.top (), 
           -1000, 1000);
}

bool 
Map::pan (double, double direction)
{
  const double delta = 0.05 * std::max (m_bounds.width (), m_bounds.height ());
  switch (int (direction))
    {
    case LEFT:
      m_bounds.move (Two_Vector (-delta, 0));
      break;
    case RIGHT:
      m_bounds.move (Two_Vector (delta, 0));
      break;
    case UP:
      m_bounds.move (Two_Vector (0, delta));
      break;
    case DOWN:
      m_bounds.move (Two_Vector (0, -delta));
      break;
    }
  return true;
}

bool 
Map::zoom (double, double direction)
{
  static const double factor = 1.1;
  switch (int (direction))
    {
    case IN:
      m_bounds.scale (1.0 / factor);
      break;
    case OUT:
      m_bounds.scale (factor);
      break;
    }
  return true;
}

bool
Map::set_zoom (double, double factor)
{
  m_bounds = m_initial_bounds;
  m_bounds.scale (factor);
  return true;
}

//* Class World_Reader

//** Constructor

World_Reader::World_Reader (std::string file_name, Gl_World* world) 
  : mp_world (world)
{
  read (file_name);
}

void 
World_Reader::on_start_tag (const Vamos_Media::XML_Tag& tag)
{
}

void 
World_Reader::on_end_tag (const Vamos_Media::XML_Tag& tag)
{
}

void 
World_Reader::on_data (std::string data)
{
  if (data.size () == 0)
    {
      return;
    }
  std::istringstream is (data.c_str ());

  if (path () == "/world/gravity")
    {
      double grav;
      is >> grav;
      mp_world->gravity (grav);
    }
  else if (path () == "/world/atmosphere/density")
    {
      double density;
      is >> density;
      mp_world->mp_atmosphere->density (density);
    }
  else if (path () == "/world/atmosphere/velocity")
    {
      Three_Vector velocity;
      is >> velocity;
      mp_world->mp_atmosphere->velocity (velocity);
    }
  else if (path () == "/world/atmosphere/speed-of-sound")
    {
      float v_s;
      is >> v_s;
      alSpeedOfSound (v_s);
    }
  else if (path () == "/world/lighting/source-position")
    {
      Three_Vector pos;
      is >> pos;
      GLfloat position [] = { pos.x, pos.y, pos.z, 0.0 };
      glLightfv (GL_LIGHT0, GL_POSITION, position);
    }
  else if (path () == "/world/lighting/ambient")
    {
      Three_Vector amb;
      is >> amb;
      GLfloat ambient [] = { amb.x, amb.y, amb.z, 1.0 };
      glLightfv (GL_LIGHT0, GL_AMBIENT, ambient);
    }
}

//* Class Controls_Reader
static int 
translate_key (std::string key_name)
{
  // If key_name is a single character, return its integer value...
  if (key_name.size () == 1)
    return int (key_name [0]);

  // ...otherwise, try to interpret it as the name of a special key.

  // Downcase the string to do case-insensinive comparisons.
  std::transform (key_name.begin (), key_name.end (), key_name.begin (),
                  ::tolower);

  if (key_name == "escape")
      return 27;
  if (key_name == "delete")
      return 127;
  if (key_name == "up")
      return SDLK_UP;
  if (key_name == "down")
      return SDLK_DOWN;
  if (key_name == "left")
      return SDLK_LEFT;
  if (key_name == "right")
      return SDLK_RIGHT;
  if (key_name == "insert")
      return SDLK_INSERT;
  if (key_name == "home")
      return SDLK_HOME;
  if (key_name == "end")
      return SDLK_END;
  if (key_name == "page up")
      return SDLK_PAGEUP;
  if (key_name == "page down")
      return SDLK_PAGEDOWN;
  if (key_name == "f1")
      return SDLK_F1;
  if (key_name == "f2")
      return SDLK_F2;
  if (key_name == "f3")
      return SDLK_F3;
  if (key_name == "f4")
      return SDLK_F4;
  if (key_name == "f5")
      return SDLK_F5;
  if (key_name == "f6")
      return SDLK_F6;
  if (key_name == "f7")
      return SDLK_F7;
  if (key_name == "f8")
      return SDLK_F8;
  if (key_name == "f9")
      return SDLK_F9;
  if (key_name == "f10")
      return SDLK_F10;
  if (key_name == "f11")
      return SDLK_F11;
  if (key_name == "f12")
      return SDLK_F12;

  assert (false);
  return 0;
}

static int 
mouse_button_to_control (std::string button)
{
  if ((button == "middle") || (button == "Middle") || (button == "MIDDLE"))
      return SDL_BUTTON_MIDDLE;
  if ((button == "right") || (button == "Right") || (button == "RIGHT"))
      return SDL_BUTTON_RIGHT;
  return SDL_BUTTON_LEFT;
}

//** Constructor

Controls_Reader::Controls_Reader (std::string file_name, Gl_World* world) 
  : mp_world (world)
{
  SDL_ShowCursor (false);
  // Turn on if we bind mouse motion events.

  m_world_function_map ["pause"] = (Callback_Function)&Gl_World::pause;
  m_world_function_map ["quit"] = (Callback_Function)&Gl_World::quit;
  m_world_function_map ["read track"] = (Callback_Function)&Gl_World::read_track;
  m_world_function_map ["read world"] = (Callback_Function)&Gl_World::read_world;
  m_world_function_map ["cycle view"] = (Callback_Function)&Gl_World::cycle_view;
  m_world_function_map ["toggle graphics"] = (Callback_Function)&Gl_World::toggle_graphics;;
  m_world_function_map ["reset car"] = (Callback_Function)&Gl_World::reset_car;
  m_world_function_map ["read car"] = (Callback_Function)&Gl_World::read_car;
  m_world_function_map ["restart car"] = (Callback_Function)&Gl_World::restart_car;
  m_world_function_map ["focus previous car"] = (Callback_Function)&Gl_World::focus_previous_car;
  m_world_function_map ["focus next car"] = (Callback_Function)&Gl_World::focus_next_car;
  m_world_function_map ["replay" ] = (Callback_Function)&Gl_World::replay;

  m_driver_function_map ["start engine"] = (Callback_Function)&Interactive_Driver::start_engine;
  m_driver_function_map ["fill tank"] = (Callback_Function)&Interactive_Driver::fill_tank;

  m_driver_function_map ["initial shift up"] = (Callback_Function)&Interactive_Driver::initial_shift_up;
  m_driver_function_map ["initial shift down"] = (Callback_Function)&Interactive_Driver::initial_shift_down;
  m_driver_function_map ["shift up"] = (Callback_Function)&Interactive_Driver::shift_up;
  m_driver_function_map ["shift down"] = (Callback_Function)&Interactive_Driver::shift_down;
  m_driver_function_map ["initial shift up disengage"] 
    = (Callback_Function)&Interactive_Driver::initial_shift_up_disengage;
  m_driver_function_map ["initial shift down disengage"] 
    = (Callback_Function)&Interactive_Driver::initial_shift_down_disengage;
  m_driver_function_map ["shift up disengage"] 
    = (Callback_Function)&Interactive_Driver::shift_up_disengage;
  m_driver_function_map ["shift down disengage"] 
    = (Callback_Function)&Interactive_Driver::shift_down_disengage;
  m_driver_function_map ["initial engage clutch"] 
    = (Callback_Function)&Interactive_Driver::initial_engage_clutch;
  m_driver_function_map ["initial disengage clutch"] 
    = (Callback_Function)&Interactive_Driver::initial_disengage_clutch;
  m_driver_function_map ["engage clutch"] = (Callback_Function)&Interactive_Driver::engage_clutch;
  m_driver_function_map ["disengage clutch"] = (Callback_Function)&Interactive_Driver::disengage_clutch;
  m_driver_function_map ["initial clutch"] = (Callback_Function)&Interactive_Driver::initial_clutch;
  m_driver_function_map ["clutch"] = (Callback_Function)&Interactive_Driver::clutch;

  m_driver_function_map ["steer"] = (Callback_Function)&Interactive_Driver::steer;
  m_driver_function_map ["steer right"] = (Callback_Function)&Interactive_Driver::steer_right;
  m_driver_function_map ["steer left"] = (Callback_Function)&Interactive_Driver::steer_left;
  m_driver_function_map ["gas"] = (Callback_Function)&Interactive_Driver::gas;
  m_driver_function_map ["brake"] = (Callback_Function)&Interactive_Driver::brake;

  m_driver_function_map ["pan left"] = (Callback_Function)&Interactive_Driver::pan_left;
  m_driver_function_map ["pan right"] = (Callback_Function)&Interactive_Driver::pan_right;

  read (file_name);
}

void 
Controls_Reader::on_start_tag (const Vamos_Media::XML_Tag& tag)
{
  if (label () == "bind")
    {
      m_function = "";
      m_control = 0;
      m_direction = NONE;
      m_factor = 1.0;
      m_offset = 0.0;
      m_deadband = 0.0;
      m_upper_deadband = 0.0;
      m_time = 0.0;
    }
}

void
Controls_Reader::register_callback 
(std::map <std::string, Callback_Function>::iterator it,
 Control_Handler* handler)
{
  switch (m_type)
    {
    case KEY:
      handler->keyboard ().bind_action (m_control,
                                        m_direction,
                                        handler,
                                        it->second,
                                        m_time);
      break;
    case JOYSTICK_BUTTON:
      handler->joystick ().bind_action (m_control,
                                        m_direction,
                                        handler,
                                        it->second, 
                                        m_time);
      break;
    case JOYSTICK_AXIS:
      handler->joystick ().bind_motion (m_control, 
                                        m_direction,
                                        handler,
                                        it->second,
                                        m_factor, 
                                        m_offset, 
                                        m_deadband,
                                        m_upper_deadband);
      break;
    case MOUSE_BUTTON:
      handler->mouse ().bind_action (m_control, 
                                     m_direction,
                                     handler,
                                     it->second, 
                                     m_time);
      break;
    case MOUSE_MOTION:
      SDL_ShowCursor (true);
      handler->mouse ().bind_motion (m_control,
                                     m_direction,
                                     handler,
                                     it->second,
                                     m_factor, 
                                     m_offset, 
                                     m_deadband,
                                     m_upper_deadband);
      break;
    default:
      assert (false);
    }
}

void 
Controls_Reader::on_end_tag (const Vamos_Media::XML_Tag& tag)
{
  if (label () == "up")
    m_direction = UP;
  else if (label () == "down")
    m_direction = DOWN;
  else if (label () == "left")
    m_direction = LEFT;
  else if (label () == "right")
    m_direction = RIGHT;
  else if (label () == "forward")
    m_direction = FORWARD;
  else if (label () == "backward")
    m_direction = BACKWARD;

  else if (label () == "bind")
    {
      std::map <std::string, Callback_Function>
        ::iterator it = m_world_function_map.find (m_function);
      if (it != m_world_function_map.end ())
        {
          register_callback (it, mp_world);
        }
      else if ((it = m_driver_function_map.find (m_function))
               != m_driver_function_map.end ())
        {
          if (mp_world->controlled_car () != 0)
            {
              register_callback 
                (it, 
                 static_cast <Control_Handler*> 
                 (dynamic_cast <Interactive_Driver*> (mp_world->controlled_car ()->driver)));
            }
        }
      else
        {
          throw Unknown_Function (m_function);
        }
    }
}

void 
Controls_Reader::on_data (std::string data)
{
  if (data.size () == 0) return;

  std::istringstream is (data.c_str ());
  
  if (label () == "key")
    {
      m_type = KEY;
      std::string key;
      is >> key;
      m_control = translate_key (key);
    }
  else if (label () == "button")
    {
      m_type = JOYSTICK_BUTTON;
      is >> m_control;
    }
  else if (label () == "mouse-button")
    {
      m_type = MOUSE_BUTTON;
      std::string button;
      is >> button;
      m_control = mouse_button_to_control (button);
    }
  else if (label () == "axis")
    {
      m_type = JOYSTICK_AXIS;
      std::string axis;
      is >> m_control;
    }
  else if (label () == "mouse-direction")
    {
      m_type = MOUSE_MOTION;
      std::string axis;
      is >> m_control;
    }
  else if (label () == "function")
      m_function = data;
  else if (label () == "factor")
      is >> m_factor;
  else if (label () == "offset")
      is >> m_offset;
  else if (label () == "deadband")
      is >> m_deadband;
  else if (label () == "upper-deadband")
      is >> m_upper_deadband;
  else if (label () == "time")
      is >> m_time;
}
