//  Copyright (C) 2001-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMES_WORLD_GL_WORLD_H_INCLUDED
#define VAMES_WORLD_GL_WORLD_H_INCLUDED

#include "controls.h"
#include "world.h"

#include "../geometry/rectangle.h"

#include <GL/gl.h>

#include <stdexcept>

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
class Sounds;

struct Can_Not_Intialize_SDL : public std::runtime_error
{
    Can_Not_Intialize_SDL(std::string const& mesage)
        : std::runtime_error(mesage)
    {}
};

struct No_SDL_Screen : public std::runtime_error
{
    No_SDL_Screen(std::string const& mesage)
        : std::runtime_error(mesage)
    {}
};
/// Exception thrown when a control callback does not name an available callback function.
struct Unknown_Function : public std::runtime_error
{
    Unknown_Function(std::string const& func_name)
        : std::runtime_error(func_name)
    {}
};

//----------------------------------------------------------------------------------------
/// The window where the scene is rendered.
class Gl_Window
{
public:
    /// Open the window and initialize GL and SDL.
    Gl_Window(int width, int height, char const* name, bool full_screen);

    int width() const { return m_width; }
    int height() const { return m_height; }
    /// @return width/height
    double aspect() const;

    void resize(int width, int height);

private:
    int m_width{1}; ///< Window width in pixels.
    int m_height{1}; ///< Window height in pixels.
    int m_video_flags; ///< Display options
};

//----------------------------------------------------------------------------------------
/// Overhead view of a track.
class Map : public Control_Handler
{
public:
    Map();

    void set_bounds(const Vamos_Track::Strip_Track& track, const Gl_Window& window);
    void set_view();

    /// Incrementally shift the view.
    bool pan(double, double direction);
    /// Incrementally magnify or contract the view about the center.
    bool zoom(double, double direction);
    /// Set a specific magnification factor.
    bool set_zoom(double, double factor);

private:
    Vamos_Geometry::Rectangle<double> m_initial_bounds;
    Vamos_Geometry::Rectangle<double> m_bounds;
};

//----------------------------------------------------------------------------------------
/// @brief The timekeeper for the simulation.
///
/// Reports simulation time accounting for pauses and non-realtime operation.  Gives the
/// size of the timestep when queried. Time steps are averaged to smooth out variations.
class Timer
{
public:
    using Tick = int;

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

    Tick m_timeout{0};          ///< How long to average the time step.
    double m_frame_step{0.001}; ///< The current interval between rendered frames.
    Tick m_current_ticks{0};    ///< The number of machine ticks since program start.
    Tick m_pause_ticks{0};      ///< How many machine ticks we've been paused for.
    Tick m_start_ticks{0};      ///< When the last averaging cycle was started.
    int m_frames{0};            ///< The total number of frames rendered.
    bool m_is_paused{false};    ///< True if time is paused.
    Tick m_fixed_time_step{10}; ///< The time step for non-realtime operation.
    bool m_use_fixed_time_step{false};
    Tick m_fixed_time{0}; ///< How many machine ticks we've been using a fixed time step.
};

//----------------------------------------------------------------------------------------
class Gl_World : public World, public Control_Handler
{
public:
    Gl_World(Vamos_Track::Strip_Track& track, Atmosphere const& atmosphere, Sounds& sounds,
             bool full_screen);
    virtual ~Gl_World() = default;

    virtual void add_car(std::shared_ptr<Vamos_Body::Car> car,
                         std::unique_ptr<Driver> driver) override;
    virtual void start(bool qualifying, size_t laps_or_minutes) override;
    // Read the world definition file.
    void read(std::string world_file = "", std::string controls_file = "");

private:
    /// Methods registered with the controls system.
    /// @{
    bool pause(double, double);                ///> Pause the program.
    bool quit(double = false, double = false); ///> Quit the program.
    bool read_car(double, double);             ///> Read the car definition file.
    bool read_track(double, double);           ///> Read the track definition file.
    bool read_world(double, double);           ///> Read the world definition file.
    bool reset_car(double, double);            ///> Put the car back on the track.
    bool restart_car(double, double);          ///> Put the car at the starting line.
    bool cycle_view(double, double);           ///> Change the view.
    bool toggle_graphics(double, double);      ///> Turn rendering on or off.
    bool focus_previous_car(double, double);   ///> Put the camera on the car ahead on the grid.
    bool focus_next_car(double, double);       ///> Put the camera on the car behind on the grid.
    bool replay(double, double);               ///> Pause an show a replay.
    /// @}

    Car_Info* focused_car();
    void set_focused_car(size_t index);
    void focus_other_car(int delta);

    void animate();
    void display();

    void check_for_events();
    void update_car_timing();
    void play_sounds();
    void set_paused(bool is_paused);

    void draw_timing_info() const;
    void draw_leaderboard(Vamos_Media::Two_D& screen) const;
    void draw_lap_times(Vamos_Media::Two_D& screen) const;
    void draw_fastest_lap(Vamos_Media::Two_D& screen,
                          Vamos_Geometry::Two_Vector const& p) const;

    /// Callback for window resizing.
    void reshape(int width, int height);

    void set_world_view(Vamos_Track::Camera const& camera);
    void set_world_view(Vamos_Geometry::Three_Vector const& camera_position,
                        Vamos_Geometry::Three_Vector const& target_position,
                        double vertical_field_angle);
    void draw_track(bool draw_sky, Vamos_Geometry::Three_Vector const& view_position);
    void draw_cars(bool draw_interior, bool draw_focused_car = true);
    void draw_mirror_views();

    friend class World_Reader;
    friend class Controls_Reader;

    enum class View{body, map, world, chase};

    Gl_Window m_window;
    Timer m_timer;
    Sounds& m_sounds;
    Map m_map;

    size_t m_focused_car_index{0};
    bool m_paused{true};
    bool m_update_graphics{true};
    bool m_done{false};
    View m_view{View::map};

    std::string m_world_file;
    std::string m_controls_file;
};
} // namespace Vamos_World

#endif // VAMES_WORLD_GL_WORLD_H_INCLUDED
