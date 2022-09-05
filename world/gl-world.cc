//  Copyright (C) 2001--2004 Sam Varner
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

#include "gl-world.h"
#include "interactive-driver.h"
#include "robot-driver.h"
#include "sounds.h"
#include "timing-info.h"

#include "../body/gl-car.h"
#include "../body/wheel.h"
#include "../geometry/conversions.h"
#include "../media/xml-parser.h"
#include "../media/two-d.h"
#include "../track/strip-track.h"

#include <SDL/SDL.h>
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
using namespace Vamos_Media;;
using namespace Vamos_World;

enum Mouse_Axis{X, Y};

auto constexpr steps_per_frame{1};

namespace Vamos_World
{
/// Reader for the file with bindings for keyboard, mouse, and joystick.
class Controls_Reader : public Vamos_Media::XML_Parser
{
public:
    /// Read the controls file.
    Controls_Reader(std::string const& file_name, Gl_World* world);

private:
    /// XML_Parser methods
    /// @{
    virtual void on_start_tag(Vamos_Media::XML_Tag const& tag) override;
    virtual void on_end_tag(Vamos_Media::XML_Tag const& tag) override;
    virtual void on_data(std::string data_string) override;
    /// @}

    using Call_Map = std::map<std::string, Callback_Fn>;

    /// Register a callback for the given handler.
    void register_callback(Call_Map::iterator it, Control_Handler* handler);

    Gl_World* mp_world; ///< The world object whose methods we're binding.

    Call_Map m_world_function_map;  ///< Map for looking up world functions by name.
    Call_Map m_driver_function_map; ///< Map for looking up driver functions by name.

    enum class Control
    {
        key,
        joystick_button,
        joystick_axis,
        mouse_button,
        mouse_motion
    };

    Control m_type{Control::key}; ///< The kind of control being bound.
    int m_control{0}; ///< ID of the button, key, axis, etc.
    Direct m_direction{Direct::none}; ///< The direction of actuation.
    /// The name of the function to call when the control is actuated. Must be a key in
    /// one of the call maps above.
    std::string m_function;
    Calibration m_calib{}; ///< Scaling, offset, etc. of the control value.
    double m_time{0.0}; ///< How long to fade in the effect of actuation.
};

//----------------------------------------------------------------------------------------
class World_Reader : public Vamos_Media::XML_Parser
{
public:
    World_Reader(std::string const& file_name, Gl_World* world);

    virtual void on_start_tag(const Vamos_Media::XML_Tag&) override {}
    virtual void on_end_tag(const Vamos_Media::XML_Tag&) override {}
    virtual void on_data(std::string data_string) override;

private:
    std::string m_path;

    Gl_World* mp_world;
};

//----------------------------------------------------------------------------------------
using Tick = int;

/// @brief The timekeeper for the simulation.
///
/// Reports simulation time accounting for pauses and non-realtime operation.  Gives the
/// size of the timestep when queried. Time steps are averaged to smooth out variations.
class Timer
{
public:
    /// Initialize the timer.
    /// @param interval the time interval (ms) to average over when determining the
    /// current time step.
    /// @param fixed_time_step the time step (ms) for non-realtime operation.
    Timer(Tick interval, Tick fixed_time_step);
    /// Set the timer to zero.
    void reset();
    /// Advance the timer.
    void update();
    /// Stop time.
    void set_paused(bool is_paused);
    /// Tell the time that a frame has been rendered.
    void add_frame() { ++m_frames; }
    /// Set the time step for non-realtime operation.
    void set_fixed_time_step(double step) { m_fixed_time_step = step; }
    /// Start and stop non-realtime operation.
    void use_fixed_time_step(bool fixed);
    /// Return the time in seconds since the last reset.
    double get_current_time() const;
    /// Return the current time step.
    double get_time_step() const;
    /// Return the current frame rate.
    double get_frame_rate() const;

private:
    /// Start a new averaging interval.
    void start_averaging();

    Tick m_timeout{0}; ///< How long to average the time step.
    double m_frame_step{0.001}; ///< The current interval between rendered frames.
    Tick m_current_ticks{0}; ///< The number of machine ticks since program start.
    Tick m_pause_ticks{0}; ///< How many machine ticks we've been paused for.
    Tick m_start_ticks{0}; ///< When the last averaging cycle was started.
    int m_frames{0}; ///< The total number of frames rendered.
    bool m_is_paused{false};
    Tick m_fixed_time_step{10}; ///< The time step for non-realtime operation.
    bool m_use_fixed_time_step{false};
    Tick m_fixed_time{0}; ///< How many machine ticks we've been using a fixed time step.
};
} // namespace Vamos_World

//-----------------------------------------------------------------------------
static std::string time_str(double time, int precision = 3)
{
  if (time == Timing_Info::no_time)
      return "";

  auto minutes{static_cast<int>(time / 60.0)};
  auto seconds{time - 60 * minutes};
  // Show the leading zero on the seconds. Add 1 for the decimal point if present.
  auto width{precision > 0 ? precision + 3 : 2};

  std::ostringstream os;
  os << minutes << ':'
     << std::fixed << std::setfill('0') << std::setw(width) << std::setprecision(precision)
     << seconds;
  return os.str();
}

static std::string dtime_str(double delta_time, int precision = 3)
{
  if (delta_time == Timing_Info::no_time)
      return "";

  std::ostringstream os;
  if (delta_time > 0.0)
      os << '+';
  os << std::fixed << std::setprecision(precision) << delta_time;
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
/// Convert ticks (integer milliseconds) to seconds.
static double ticks_to_seconds(Tick ticks)
{
    return 0.001 * ticks;
}

Timer::Timer(Tick interval, Tick fixed_time_step)
    : m_timeout{interval},
      m_fixed_time_step{fixed_time_step}
{
    reset();
}

void Timer::reset()
{
    start_averaging();
    // Pretend that the simulation was paused until now.
    m_pause_ticks = m_start_ticks;
    m_fixed_time = 0;
}

void Timer::update()
{
    if (m_is_paused)
        return;

    if (m_use_fixed_time_step)
        m_fixed_time += m_fixed_time_step;
    m_current_ticks = SDL_GetTicks();
    auto elapsed{m_current_ticks - m_start_ticks};
    if (elapsed > m_timeout && m_frames > 0)
    {
        m_frame_step = ticks_to_seconds(elapsed) / m_frames;
        start_averaging();
    }
}

void Timer::start_averaging()
{
    m_start_ticks = SDL_GetTicks();
    m_frames = 0;
}

void Timer::set_paused(bool is_paused)
{
    m_is_paused = is_paused;
    if (!is_paused)
    {
        start_averaging();
        m_pause_ticks += m_start_ticks - m_current_ticks;
        update();
    }
}

void Timer::use_fixed_time_step(bool fixed)
{
    m_use_fixed_time_step = fixed;
    if (!fixed)
    {
        start_averaging();
        update();
    }
}

double Timer::get_current_time() const
{
    return ticks_to_seconds(m_current_ticks - m_pause_ticks + m_fixed_time);
}

double Timer::get_time_step() const
{
    return m_use_fixed_time_step
        ? ticks_to_seconds(m_fixed_time_step)
        : m_frame_step / steps_per_frame;
}

double Timer::get_frame_rate() const
{
    return m_use_fixed_time_step ? 0.0 : 1.0 / m_frame_step;
}

//-----------------------------------------------------------------------------
Gl_World::Gl_World (Vamos_Track::Strip_Track& track, 
                    Atmosphere& atmosphere,
                    Sounds& sounds,
                    bool full_screen)
  : World (track, atmosphere),
    mp_timer{std::make_unique<Timer>(100, 10)},
    m_sounds (sounds),
    mp_window (0),
    m_view (MAP_VIEW),
    m_update_graphics (true),
    m_done (false)
{
  int argc = 0;
  initialize_graphics (&argc, NULL);
  mp_window = new Gl_Window (800, 500, "Vamos", full_screen);
  reshape (mp_window->width (), mp_window->height ());
  set_attributes ();
  set_paused (true);
}

Gl_World::~Gl_World ()
{
  delete mp_window;
  SDL_Quit();
}

void 
Gl_World::initialize_graphics (int* argc, char** argv)
{
  glutInit (argc, argv);
  if (SDL_Init (SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_JOYSTICK) != 0)
    throw Can_Not_Intialize_SDL (SDL_GetError ());
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
  glEnable (GL_STENCIL_TEST);
  glClearStencil (0);
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
Gl_World::add_car (Vamos_Body::Car& car, Driver& driver)
{
  World::add_car (car, driver);

  // If there's a controlled car, show the view from inside it. Otherwise show
  // the view from the trackside cameras.
  if (driver.is_interactive ())
    m_view = BODY_VIEW;
  else if (m_view != BODY_VIEW)
    m_view = WORLD_VIEW;
}

// Read the definition file.
void 
Gl_World::read (std::string world_file,
                std::string controls_file)
{
  // Remember the file names for re-reading.
  m_world_file = world_file;
  m_controls_file = controls_file;

  World_Reader (m_world_file, this);
  Controls_Reader (m_controls_file, this);
}

void 
Gl_World::set_paused (bool is_paused)
{
  mp_timer->set_paused (is_paused);
  m_paused = is_paused;

  for (std::vector <Car_Information>::iterator it = m_cars.begin ();
       it != m_cars.end ();
       it++)
    {
      it->car->set_paused (is_paused);
    }

  if (is_paused)
    m_sounds.pause ();
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
  m_track.read ();
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
  mp_timer->use_fixed_time_step (!m_update_graphics);
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

void Gl_World::animate()
{
    if (focused_car())
    {
        for (int loop = 0; loop < steps_per_frame; ++loop)
            propagate_cars (mp_timer->get_time_step());
        play_sounds();
        update_car_timing();
    }
    mp_timer->add_frame();
}

void
Gl_World::update_car_timing ()
{
  for (size_t i = 0; i < m_cars.size (); i++)
    {
      Car_Information& car = m_cars [i];
      if (!car.driver->is_driving ())
        car.driver->start (mp_timing->countdown ());
      const double distance = car.track_position ().x;
      const int sector = m_track.sector (distance);
      mp_timing->update(mp_timer->get_current_time(), i, distance, sector);
      if (mp_timing->timing_at_index (i).is_finished ())
        car.driver->finish ();
    }
}

void Gl_World::play_sounds()
{
    auto tire_slide{0.0};
    auto kerb_speed{0.0};
    auto grass_speed{0.0};
    auto gravel_speed{0.0};
    auto scrape_speed{0.0};
    auto hard_crash_speed{0.0};
    auto soft_crash_speed{0.0};

    for (auto const& touch : m_interaction_info)
    {
        if (touch.car != focused_car()->car)
            continue;

        switch (touch.track_material)
        {
        case Material::ASPHALT:
        case Material::CONCRETE:
        case Material::METAL:
            if (touch.car_material == Material::RUBBER)
                tire_slide = touch.car->slide();
            else if (touch.car_material == Material::METAL)
            {
                scrape_speed = touch.parallel_speed;
                hard_crash_speed = touch.perpendicular_speed;
            }
            break;
        case Material::KERB:
            kerb_speed = touch.parallel_speed;
            break;
        case Material::GRASS:
            grass_speed = touch.parallel_speed;
            break;
        case Material::GRAVEL:
            gravel_speed = touch.parallel_speed;
            break;
        case Material::RUBBER:
            soft_crash_speed = touch.perpendicular_speed;
            break;
        default:
            break;
        }
    }
    m_interaction_info.clear();

    auto const& chassis{focused_car()->car->chassis()};
    auto const& pos{chassis.position()};
    auto wind_speed{(chassis.cm_velocity() - m_atmosphere.velocity).magnitude()};
    m_sounds.play_tire_squeal(tire_slide, pos);
    m_sounds.play_kerb(kerb_speed, pos);
    m_sounds.play_grass(grass_speed, pos);
    m_sounds.play_gravel(gravel_speed, pos);
    m_sounds.play_scrape(scrape_speed, pos);
    m_sounds.play_wind(wind_speed, pos);
    m_sounds.play_hard_crash(hard_crash_speed, pos);
    m_sounds.play_soft_crash(soft_crash_speed, pos);
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
      Three_Vector pos = focused_car ()->car->draw_rear_view (mp_window->aspect (), i);

      glMatrixMode (GL_MODELVIEW);

      // Enable the rearview mirror mask.
      glStencilFunc (GL_EQUAL, 1, 1);

      // Front and back are reversed in the mirrors, so cull front
      // faces while we're drawing mirror views.
      glPushAttrib (GL_POLYGON_BIT);
      {
        glCullFace (GL_FRONT);
        m_track.draw_sky (pos);
        m_track.draw ();
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
  gluPerspective (vertical_field_angle, mp_window->aspect (), 1.0, 10000.0);
  gluLookAt (camera_position.x, camera_position.y, camera_position.z,
             target_position.x, target_position.y, target_position.z,
             0.0, 0.0, 1.0); // up direction

  Three_Vector direction (target_position - camera_position);
  float at_up [6] = { float (direction.x), float (direction.y), float (direction.z),
                      0.0f, 0.0f, 1.0f };

  alListener3f (AL_POSITION, 
                camera_position.x, camera_position.y, camera_position.z);
  alListener3f (AL_VELOCITY, 0.0f, 0.0f, 0.0f);
  alListenerfv (AL_ORIENTATION, at_up);
}

void 
Gl_World::set_world_view (const Vamos_Track::Camera& camera)
{
  set_world_view (m_track.camera_position (camera),
                  camera.fixed ? m_track.camera_target (camera)
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
      m_track.draw_sky (view_position);
    }
  else
    {
      m_track.draw_map_background ();
    }
  m_track.draw ();
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
      draw_track (true, focused_car ()->car->view_position (true, true));
      draw_cars (true);
      draw_timing_info ();
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
      draw_track (false, Three_Vector::ZERO);
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
              m_track.get_camera (mp_timing->timing_at_index (m_focused_car_index)
                                    .lap_distance ());
            set_world_view (camera);
            draw_track (true, m_track.camera_position (camera));
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
  m_map.set_bounds (m_track, *mp_window);
}

void Gl_World::draw_timing_info() const
{
    Two_D screen;
    auto count{mp_timing->countdown()};
    if (count > 0)
        screen.lights(50.0, 60.0, 1.0, 5, count, 0.9, 0.0, 0.0, 0.23, 0.2, 0.2);

    if (mp_timing->running_order().size() > 1)
        draw_leaderboard(screen);
    else
        draw_lap_times(screen);

    // Draw timing info for the focused car.
    auto car{mp_timing->timing_at_index(m_focused_car_index)};
    auto x{55};
    auto dt{dtime_str(car.lap_time_difference())};
    screen.text(x, 14, "Lap Time", time_str(car.lap_time()));
    screen.text(x, 10, "    Last", time_str(car.previous_lap_time()), dt);
    screen.text(x, 6,  "    Best", time_str(car.best_lap_time()));
    screen.text(x, 2,  "frames/s", static_cast<int>(mp_timer->get_frame_rate() + 0.5));

    x = 75;
    dt = dtime_str(car.previous_sector_time_difference());
    screen.text(x, 14, "     Sector", time_str(car.sector_time()));
    screen.text(x, 10, "       Best", time_str(car.best_sector_time()));
    screen.text(x, 6,  "Last Sector", time_str(car.previous_sector_time()), dt);
    screen.text(x, 2,  "Distance", static_cast<int>(car.lap_distance()), " m");
}

void Gl_World::draw_leaderboard(Vamos_Media::Two_D& screen) const
{
    auto x{2};
    auto y{95};

    auto const& order{mp_timing->running_order()};
    if (m_track.get_road(0).is_closed())
    {
        auto total_laps{mp_timing->total_laps()};
        if (mp_timing->is_finished())
            screen.text(x, y, "Finish");
        else if (mp_timing->is_qualifying() && total_laps == 0)
            screen.text(x, y, "", time_str(mp_timing->time_remaining(), 0));
        else
        {
            std::ostringstream os;
            os << order.front()->current_lap() << '/' << total_laps;
            screen.text(x, y, "Lap", os.str());
        }
    }

    y -= 3;
    // Show absolute lap time for the leader.
    auto time{time_str(mp_timing->is_qualifying()
                       ? order.front()->best_lap_time() : order.front()->previous_lap_time())};
    screen.text(x, y, m_cars[order.front()->grid_position() - 1].car->name(), time);
    y -= 3;
    // Show relative times for the rest.
    for (auto it{order.cbegin()}; it != order.cend(); ++it, y -= 3)
    {
        time = mp_timing->is_qualifying()
            ? time_str((*it)->best_lap_time()) : dtime_str((*it)->interval());
        screen.text(x, y, m_cars[(*it)->grid_position() - 1].car->name(), time);
    }
    if (!mp_timing->is_qualifying() && m_track.get_road(0).is_closed())
        draw_fastest_lap(screen, x, y);
}

void Gl_World::draw_lap_times(Vamos_Media::Two_D& screen) const
{
    const auto& order{mp_timing->running_order()};
    auto it{order.cbegin()};

    std::vector<double> a_time;
    auto lap_time{(*it)->previous_lap_time()};
    auto lap{(*it)->current_lap()};
    if (lap_time != Timing_Info::no_time && lap > 0 && lap - 1 > a_time.size())
        a_time.push_back(lap_time);

    auto x{2};
    auto y{95};
    screen.text(x, y, "Lap", "Time");
    for (auto i = 0; auto time : a_time)
        screen.text(x, y -= 3, ++i, time_str(time));
    // Draw the lap number with no time for the current lap.
    screen.text(x, y, a_time.size() + 1, "");
    draw_fastest_lap(screen, x, y - 3);
}

void Gl_World::draw_fastest_lap(Vamos_Media::Two_D& screen, int x, int y) const
{
    screen.text(x, y, "Fastest Lap");
    const auto* p_fastest{mp_timing->fastest_lap_timing()};
    if (p_fastest && (p_fastest->best_lap_time() != Timing_Info::no_time))
        screen.text(x, y - 3, m_cars[p_fastest->grid_position() - 1].car->name(),
                    time_str(p_fastest->best_lap_time()));
}

void 
Gl_World::start (bool qualifying, size_t laps_or_minutes)
{
  World::start (qualifying, laps_or_minutes);
  m_map.set_bounds (m_track, *mp_window);
  if (!m_cars.empty ())
    set_paused (false);
  mp_timer->reset();

  SDL_Event event;

  // Flush the event queue.
  while (SDL_PollEvent (&event))
    ;

  while (!m_done)
    {
      mp_timer->update();
      check_for_events ();

      if (m_paused)
        {
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
            m_map.m_keyboard.press (event.key.keysym.sym);
          break;
        case SDL_KEYUP:
          m_keyboard.release (event.key.keysym.sym);
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

//-----------------------------------------------------------------------------
Map::Map ()
{
    m_keyboard.bind_action(SDLK_RIGHT, Direct::down, std::bind_front(&Map::pan, this),
                           to_integral(Direct::right));
    m_keyboard.bind_action(SDLK_LEFT, Direct::down, std::bind_front(&Map::pan, this),
                          to_integral(Direct::left));
    m_keyboard.bind_action(SDLK_UP, Direct::down, std::bind_front(&Map::pan, this),
                          to_integral(Direct::up));
    m_keyboard.bind_action(SDLK_DOWN, Direct::down, std::bind_front(&Map::pan, this),
                          to_integral(Direct::down));

    m_keyboard.bind_action('=', Direct::down, std::bind_front(&Map::zoom, this),
                           to_integral(Direct::in));
    m_keyboard.bind_action('+', Direct::down, std::bind_front(&Map::zoom, this),
                           to_integral(Direct::in));
    m_keyboard.bind_action('-', Direct::down, std::bind_front(&Map::zoom, this),
                           to_integral(Direct::out));
    m_keyboard.bind_action('_', Direct::down, std::bind_front(&Map::zoom, this),
                           to_integral(Direct::out));

    for (char c = '1'; c <= '9'; c++)
        m_keyboard.bind_action(c, Direct::down, std::bind_front(&Map::set_zoom, this),
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
  switch (Direct(direction))
    {
    case Direct::left:
      m_bounds.move (Two_Vector (-delta, 0));
      break;
    case Direct::right:
      m_bounds.move (Two_Vector (delta, 0));
      break;
    case Direct::up:
      m_bounds.move (Two_Vector (0, delta));
      break;
    case Direct::down:
      m_bounds.move (Two_Vector (0, -delta));
      break;
    default:
        assert(false);
        break;
    }
  return true;
}

bool 
Map::zoom (double, double direction)
{
  static const double factor = 1.1;
  switch (Direct(direction))
    {
    case Direct::in:
      m_bounds.scale (1.0 / factor);
      break;
    case Direct::out:
      m_bounds.scale (factor);
      break;
    default:
        assert(false);
        break;
    }
  return true;
}

bool
Map::set_zoom (double, double factor)
{
  m_bounds = m_initial_bounds;
  m_bounds.scale (1.0 / factor);
  return true;
}

//----------------------------------------------------------------------------------------
World_Reader::World_Reader(std::string const& file_name, Gl_World* world)
    : mp_world{world}
{
    read(file_name);
}

void World_Reader::on_data(std::string data)
{
    if (data.empty())
        return;

    std::istringstream is(data.c_str());

    if (path() == "/world/gravity")
    {
        double grav;
        is >> grav;
        mp_world->gravity(grav);
    }
    else if (path() == "/world/atmosphere/density")
        is >> mp_world->m_atmosphere.density;
    else if (path() == "/world/atmosphere/velocity")
        is >> mp_world->m_atmosphere.velocity;
    else if (path() == "/world/atmosphere/speed-of-sound")
    {
        float v_s;
        is >> v_s;
        alSpeedOfSound(v_s);
    }
    else if (path() == "/world/lighting/source-position")
    {
        Three_Vector pos;
        is >> pos;
        GLfloat position[] = {float(pos.x), float(pos.y), float(pos.z), 0.0f};
        glLightfv(GL_LIGHT0, GL_POSITION, position);
    }
    else if (path() == "/world/lighting/ambient")
    {
        Three_Vector amb;
        is >> amb;
        GLfloat ambient[] = {float(amb.x), float(amb.y), float(amb.z), 1.0f};
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    }
}

//----------------------------------------------------------------------------------------
std::map<std::string, int> key_map{
    {"escape", 27}, {"delete", 127},
    {"up", SDLK_UP}, {"down", SDLK_DOWN},
    {"left", SDLK_LEFT}, {"right", SDLK_RIGHT},
    {"insert", SDLK_INSERT}, {"home", SDLK_HOME}, {"end", SDLK_END},
    {"page up", SDLK_PAGEUP}, {"page down", SDLK_PAGEDOWN},
    {"f1", SDLK_F1}, {"f2", SDLK_F2}, {"f3", SDLK_F3}, {"f4", SDLK_F4},
    {"f5", SDLK_F5}, {"f6", SDLK_F6}, {"f7", SDLK_F7}, {"f8", SDLK_F8},
    {"f9", SDLK_F9}, {"f10", SDLK_F10}, {"f11", SDLK_F11}, {"f12", SDLK_F12},
    {"left", SDL_BUTTON_LEFT}, {"middle", SDL_BUTTON_MIDDLE}, {"middle", SDL_BUTTON_RIGHT},
};

static int translate_key(std::string key_name)
{
  // If key_name is a single character, return its integer value.
  if (key_name.size() == 1)
      return static_cast<int>(key_name[0]);
  // Downcase the string to do case-insensitive lookup.
  std::transform(key_name.begin(), key_name.end(), key_name.begin(), ::tolower);
  return key_map[key_name];
}

//----------------------------------------------------------------------------------------
Controls_Reader::Controls_Reader(std::string const& file_name, Gl_World* world)
    : mp_world{world}
{
    // Turn on if we bind mouse motion events.
    SDL_ShowCursor(false);

    /// Turn a map of names and callbacks into a Call_Map for the given object.
    auto bindings = [](auto const& fns, auto* obj)
    {
        Call_Map call;
        for (auto fn : fns)
            call[fn.first] = std::bind_front(fn.second, obj);
        return call;
    };

    using World_CBs = std::map<std::string, std::function<bool(Gl_World*, double, double)>>;
    m_world_function_map = bindings(
        World_CBs{{"pause", &Gl_World::pause},
                  {"quit", &Gl_World::quit},
                  {"read track", &Gl_World::read_track},
                  {"read world", &Gl_World::read_world},
                  {"cycle view", &Gl_World::cycle_view},
                  {"toggle graphics", &Gl_World::toggle_graphics},
                  {"reset car", &Gl_World::reset_car},
                  {"read car", &Gl_World::read_car},
                  {"restart car", &Gl_World::restart_car},
                  {"focus previous car", &Gl_World::focus_previous_car},
                  {"focus next car", &Gl_World::focus_next_car},
                  {"replay", &Gl_World::replay}},
        mp_world);

    auto car{mp_world->controlled_car()};
    auto driver{car ? dynamic_cast<Interactive_Driver*>(car->driver) : nullptr};
    using Driver_CBs
        = std::map<std::string, std::function<bool(Interactive_Driver*, double, double)>>;
    m_driver_function_map = bindings(
        Driver_CBs{{"start engine", &Interactive_Driver::start_engine},
                   {"fill tank", &Interactive_Driver::fill_tank},
                   {"initial shift up", &Interactive_Driver::initial_shift_up},
                   {"initial shift down", &Interactive_Driver::initial_shift_down},
                   {"shift up", &Interactive_Driver::shift_up},
                   {"shift down", &Interactive_Driver::shift_down},
                   {"initial shift up disengage", &Interactive_Driver::initial_shift_up_disengage},
                   {"initial shift down disengage", &Interactive_Driver::initial_shift_down_disengage},
                   {"shift up disengage", &Interactive_Driver::shift_up_disengage},
                   {"shift down disengage", &Interactive_Driver::shift_down_disengage},
                   {"initial engage clutch", &Interactive_Driver::initial_engage_clutch},
                   {"initial disengage clutch", &Interactive_Driver::initial_disengage_clutch},
                   {"engage clutch", &Interactive_Driver::engage_clutch},
                   {"disengage clutch", &Interactive_Driver::disengage_clutch},
                   {"initial clutch", &Interactive_Driver::initial_clutch},
                   {"clutch", &Interactive_Driver::clutch},
                   {"steer", &Interactive_Driver::steer},
                   {"steer right", &Interactive_Driver::steer_right},
                   {"steer left", &Interactive_Driver::steer_left},
                   {"gas", &Interactive_Driver::gas},
                   {"brake", &Interactive_Driver::brake},
                   {"pan left", &Interactive_Driver::pan_left},
                   {"pan right", &Interactive_Driver::pan_right}},
        driver);

    read(file_name);
}

void Controls_Reader::on_start_tag(Vamos_Media::XML_Tag const&)
{
    if (label() != "bind")
        return;

    m_function = "";
    m_control = 0;
    m_direction = Direct::none;
    m_calib = {};
    m_time = 0.0;
}

void Controls_Reader::register_callback(Call_Map::iterator it, Control_Handler* handler)
{
    switch (m_type)
    {
    case Control::key:
        handler->keyboard().bind_action(m_control, m_direction, it->second, m_time);
        break;
    case Control::joystick_button:
        handler->joystick().bind_action(m_control, m_direction, it->second, m_time);
        break;
    case Control::joystick_axis:
        handler->joystick().bind_motion(m_control, it->second, m_calib);
        break;
    case Control::mouse_button:
        handler->mouse().bind_action(m_control, m_direction, it->second, m_time);
        break;
    case Control::mouse_motion:
        SDL_ShowCursor(true);
        handler->mouse().bind_motion(m_control, it->second, m_calib);
        break;
    default:
        assert(false);
    }
}

void Controls_Reader::on_end_tag(Vamos_Media::XML_Tag const&)
{
    if (label() == "up")
        m_direction = Direct::up;
    else if (label() == "down")
        m_direction = Direct::down;
    else if (label() == "left")
        m_direction = Direct::left;
    else if (label() == "right")
        m_direction = Direct::right;
    else if (label() == "forward")
        m_direction = Direct::forward;
    else if (label() == "backward")
        m_direction = Direct::backward;
    else if (label() == "bind")
    {
        // Use a one-sided calibration if only one direction was specified.
        m_calib.negative = m_direction != Direct::forward && m_direction != Direct::left;
        m_calib.positive = m_direction != Direct::backward && m_direction != Direct::right;
        auto wit{m_world_function_map.find(m_function)};
        auto dit{m_driver_function_map.find(m_function)};
        auto car{mp_world->controlled_car()};
        auto driver{car ? dynamic_cast<Interactive_Driver*>(car->driver) : nullptr};
        if (wit != m_world_function_map.end())
            register_callback(wit, mp_world);
        else if (dit != m_driver_function_map.end())
        {
            if (driver)
                register_callback(dit, driver);
        }
        else
            throw Unknown_Function(m_function);
    }
}

void Controls_Reader::on_data(std::string data)
{
    if (data.empty())
        return;

    std::istringstream is{data.c_str()};

    if (label() == "key")
    {
        m_type = Control::key;
        std::string key;
        is >> key;
        m_control = translate_key(key);
    }
    else if (label() == "button")
    {
        m_type = Control::joystick_button;
        is >> m_control;
    }
    else if (label() == "mouse-button")
    {
        m_type = Control::mouse_button;
        std::string button;
        is >> button;
        m_control = translate_key(button);
    }
    else if (label() == "axis")
    {
        m_type = Control::joystick_axis;
        std::string axis;
        is >> m_control;
    }
    else if (label() == "mouse-direction")
    {
        m_type = Control::mouse_motion;
        std::string axis;
        is >> m_control;
    }
    else if (label() == "function")
        m_function = data;
    else if (label() == "factor")
        is >> m_calib.factor;
    else if (label() == "offset")
        is >> m_calib.offset;
    else if (label() == "deadband")
        is >> m_calib.deadband;
    else if (label() == "upper-deadband")
        is >> m_calib.upper_deadband;
    else if (label() == "time")
        is >> m_time;
}
