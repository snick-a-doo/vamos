//  Gl_World.h - handles interactions between a car and its environment.
//
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

#ifndef _GL_WORLD_H_
#define _GL_WORLD_H_

#include "world.h"
#include "controls.h"

#include "../geometry/rectangle.h"

#include <GL/gl.h>

#include <memory>

namespace Vamos_Geometry
{
  class Three_Vector;
}

namespace Vamos_Media
{
  class Two_D;
}

namespace Vamos_Track
{
  class Camera;
}

namespace Vamos_World
{
class Controls_Reader;
class Sounds;
class Timer;
class World_Reader;

  class Gl_Window
  {
    int m_width;
    int m_height;
    double m_aspect;
    int m_id;
    int m_video_flags;
    
  public:
    Gl_Window (int width, int height, const char* name, bool full_screen);
    ~Gl_Window ();
    
    double aspect () const { return m_aspect; }
    int width () const { return m_width; }
    int height () const { return m_height; }

    void resize (int width, int height);
  };

  struct Can_Not_Intialize_SDL
  {
    std::string error;
    Can_Not_Intialize_SDL (std::string error_in) : error (error_in) {};
  };

  struct No_SDL_Screen
  {
    int width;
    int height;
    int depth;
    int video_flags;
    No_SDL_Screen (int w, int h, int d, int flags) 
      : width (w), height (h), depth (d), video_flags (flags) {};
  };

  class Map : public Control_Handler
  {
  public:
    Map ();

    void set_bounds (const Vamos_Track::Strip_Track& track,
                     const Gl_Window& window);
    void set_view ();

      virtual Control& joystick() override { return m_joystick; }
      virtual Control& keyboard() override { return m_keyboard; }
      virtual Control& mouse() override { return m_mouse; }

    Control m_joystick;
    Control m_keyboard;
    Control m_mouse;

    bool pan (double, double direction);
    bool zoom (double, double direction);
    bool set_zoom (double, double factor);

  private:
    Vamos_Geometry::Rectangle m_initial_bounds;
    Vamos_Geometry::Rectangle m_bounds;
  };

  class Gl_World : public World, public Control_Handler
  {
  public:
    Gl_World (Vamos_Track::Strip_Track& track, 
              Atmosphere& atmosphere,
              Sounds& sounds,
              bool full_screen);
    virtual ~Gl_World ();

    virtual void add_car (Vamos_Body::Car& car, Driver& driver);

    // Read the world definition file.
    void read (std::string world_file = "", std::string controls_file = "");

    virtual void start (bool qualifying, size_t laps_or_minutes);

    void set_paused (bool is_paused);

    // Methods registered with the controls system.
    // reset() and restart() defined in a base class are also
    // registered. 
    bool pause (double, double);       // Pause the program.
    bool quit (double = false, double = false); // Quit the program.
    bool read_car (double, double);    // Read the car definition file.
    bool read_track (double, double);  // Read the track definition file.
    bool read_world (double, double);  // Read the world definition file.
    bool reset_car (double, double);   // Put the car back on the track.
    bool restart_car (double, double); // Put the car at the starting line.

    bool cycle_view (double, double);  // Change the view.
    bool toggle_graphics (double, double);

    bool focus_previous_car (double, double);
    bool focus_next_car (double, double);
    void set_focused_car (size_t index);

    bool replay (double, double);

    virtual Control& joystick () { return m_joystick; }
    virtual Control& keyboard () { return m_keyboard; }
    virtual Control& mouse () { return m_mouse; }
    void display ();

  private:
    virtual void draw_timing_info () const;
    void draw_leaderboard (Vamos_Media::Two_D& screen) const;
    void draw_lap_times (Vamos_Media::Two_D& screen) const;
    void draw_fastest_lap (Vamos_Media::Two_D& screen, int x, int y) const;

    friend class World_Reader;
    friend class Controls_Reader;

    std::string m_world_file;
    std::string m_controls_file;

      std::unique_ptr<Timer> mp_timer;
    Sounds& m_sounds;

    Control m_keyboard;
    Control m_joystick;
    Control m_mouse;
    Gl_Window* mp_window;
    GLuint m_font;

    enum View
    {
      BODY_VIEW = 0,
      MAP_VIEW,
      WORLD_VIEW,
      CHASE_VIEW
    };

    View m_view;

    int m_frames;
    bool m_paused;
    bool m_update_graphics;
    bool m_done;
    Map m_map;

    void check_for_events ();
    void animate ();
    void update_car_timing ();
    void play_sounds ();

    void reshape (int width, int height);
    void update_track_bounds ();
    void map_key_press (int index); //!! factor out map class?

    void show_full_window ();
    void set_car_view (Vamos_Body::Car* car);
    void set_map_view ();
    void set_world_view (const Vamos_Track::Camera& camera);
    void set_world_view (const Vamos_Geometry::Three_Vector& camera_position,
                         const Vamos_Geometry::Three_Vector& target_position,
                         double vertical_field_angle);
    void draw_track (bool draw_sky, const Vamos_Geometry::Three_Vector& view_position); 
    void draw_cars (bool draw_interior, bool draw_focused_car = true);
    void draw_mirror_views ();
    void show_scene ();

    void initialize_graphics (int* argc, char** argv);
    void set_attributes ();
  };

  struct Unknown_Function
  {
    std::string m_function;
    Unknown_Function (std::string func) : m_function (func) {};
  };
} // namespace Vamos_World

#endif // not _GL_WORLD_
