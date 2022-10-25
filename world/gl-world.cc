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

#include "gl-world.h"
#include "interactive-driver.h"
#include "robot-driver.h"
#include "sounds.h"
#include "timing-info.h"

#include "../body/gl-car.h"
#include "../body/wheel.h"
#include "../geometry/conversions.h"
#include "../media/two-d.h"
#include "../track/strip-track.h"

#include <GL/glut.h>
#include <SDL2/SDL.h>
#include <pugixml.hpp>

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_World;

enum Mouse_Axis{X, Y};

auto constexpr steps_per_frame{3};
Color constexpr red_light_on{0.9, 0.0, 0.0};
Color constexpr red_light_off{0.23, 0.2, 0.2};

//----------------------------------------------------------------------------------------
/// Convert ticks (integer milliseconds) to seconds.
static double ticks_to_seconds(Timer::Tick ticks)
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
    if (fixed)
        return;
    start_averaging();
    update();
}

double Timer::get_current_time() const
{
    return ticks_to_seconds(m_current_ticks - m_pause_ticks + m_fixed_time);
}

double Timer::get_time_step() const
{
    return m_use_fixed_time_step ? ticks_to_seconds(m_fixed_time_step)
                                 : m_frame_step / steps_per_frame;
}

double Timer::get_frame_rate() const
{
    return m_use_fixed_time_step ? 0.0 : 1.0 / m_frame_step;
}

//-----------------------------------------------------------------------------
Gl_Window::Gl_Window(int width, int height, char const* name, bool full_screen)
{
    auto argc{0};
    glutInit(&argc, nullptr);

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 1);
    auto video_flags{SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE};
    if (full_screen)
    {
        video_flags |= SDL_WINDOW_FULLSCREEN;
        SDL_DisplayMode mode{SDL_PIXELFORMAT_UNKNOWN, 0, 0, 0, 0};
        if (SDL_GetNumVideoDisplays() < 1)
            throw Can_Not_Intialize_SDL(SDL_GetError());
        if (SDL_GetDisplayMode(0, 0, &mode) != 0)
            throw Can_Not_Intialize_SDL(SDL_GetError());
        width = mode.w;
        height = mode.h;
    }
    mp_window = SDL_CreateWindow(name, 0, 0, width, height, video_flags);
    if (!mp_window)
        throw Can_Not_Intialize_SDL(SDL_GetError());
    mp_context = SDL_GL_CreateContext(mp_window);
    if (!mp_context)
        throw Can_Not_Intialize_SDL(SDL_GetError());

    resize(width, height);
}

Gl_Window::~Gl_Window()
{
    SDL_GL_DeleteContext(mp_context);
}

double Gl_Window::aspect() const
{
    return m_height == 0 ? 1.0 : static_cast<double>(m_width) / m_height;
}

void Gl_Window::resize(int width, int height)
{
    m_width = width;
    m_height = height;
    glViewport(0, 0, m_width, m_height);
}

void Gl_Window::refresh() const
{
    glFlush();
    SDL_GL_SwapWindow(mp_window);
}

//-----------------------------------------------------------------------------
Map::Map()
{
    keyboard().bind_action(SDLK_RIGHT, Direct::down, std::bind_front(&Map::pan, this),
                           to_integral(Direct::right));
    keyboard().bind_action(SDLK_LEFT, Direct::down, std::bind_front(&Map::pan, this),
                           to_integral(Direct::left));
    keyboard().bind_action(SDLK_UP, Direct::down, std::bind_front(&Map::pan, this),
                           to_integral(Direct::up));
    keyboard().bind_action(SDLK_DOWN, Direct::down, std::bind_front(&Map::pan, this),
                           to_integral(Direct::down));

    keyboard().bind_action('=', Direct::down, std::bind_front(&Map::zoom_center, this), 1);
    keyboard().bind_action('+', Direct::down, std::bind_front(&Map::zoom_center, this), 1);
    keyboard().bind_action('-', Direct::down, std::bind_front(&Map::zoom_center, this), -1);
    keyboard().bind_action('_', Direct::down, std::bind_front(&Map::zoom_center, this), -1);

    for (char c = '1'; c <= '9'; c++)
        keyboard().bind_action(c, Direct::down, std::bind_front(&Map::set_zoom, this),
                               c - '1' + 1);

    mouse().bind_action(SDL_BUTTON_LEFT, Direct::down,
                        std::bind_front(&Map::mouse_button, this), 0.0);
    mouse().bind_action(SDL_BUTTON_LEFT, Direct::up,
                        std::bind_front(&Map::mouse_button, this), 0.0);
    mouse().bind_motion(X, std::bind_front(&Map::point_x, this), {});
    mouse().bind_motion(Y, std::bind_front(&Map::point_y, this), {});
    mouse().bind_incremental(0, std::bind_front(&Map::zoom_point, this));
}

void Map::set_bounds(Vamos_Track::Strip_Track const& track, Gl_Window const& window)
{
    auto bounds{m_bounds};
    auto scale{m_initial_bounds.empty() ? 1.0 : m_bounds.width() / m_initial_bounds.width()};
    // Adjust the mins and maxes to keep the correct aspect ratio of the track regardless
    // of the window's size.
    m_bounds = track.bounds();
    if (m_bounds.empty())
        return;
    auto ratio{m_bounds.aspect() / window.aspect()};
    // If the window is wider than the track, stretch x, otherwise stretch y.
    if (ratio < 1.0)
        m_bounds.scale(1.0 / ratio, 1.0);
    else
        m_bounds.scale(1.0, ratio);
    m_initial_bounds = m_bounds;

    mouse().set_axis_range(X, window.width(), -window.width());
    mouse().set_axis_range(Y, window.height(), -window.height());
    m_bounds.scale(scale);
    if (!bounds.empty())
        m_bounds.move(bounds.center() - m_bounds.center());
}

void Map::set_view()
{
    // Use a large z-range to allow for elevation changes.
    glOrtho(m_bounds.left(), m_bounds.right(), m_bounds.bottom(), m_bounds.top(), -1000, 1000);
}

bool Map::pan(double, double direction)
{
    auto delta{0.05 * std::max(m_bounds.width(), m_bounds.height())};
    switch (Direct(direction))
    {
    case Direct::left:
        m_bounds.move({-delta, 0});
        break;
    case Direct::right:
        m_bounds.move({delta, 0});
        break;
    case Direct::up:
        m_bounds.move({0, delta});
        break;
    case Direct::down:
        m_bounds.move({0, -delta});
        break;
    default:
        assert(false);
        break;
    }
    return true;
}

bool Map::zoom_center(double, double direction)
{
    zoom(direction == 1.0 ? 1 : -1, {0.5, 0.5});
    return true;
}

bool Map::zoom_point(double step, double)
{
    zoom(static_cast<int>(step), m_pointer);
    return true;
}

void Map::zoom(int step, Point<double> const& center)
{
    // Find the map coordinate for the normalized pointer position.
    Point<double> coord{std::lerp(m_bounds.left(), m_bounds.right(), center.x),
                        std::lerp(m_bounds.bottom(), m_bounds.top(), center.y)};
    m_bounds.move(-coord);
    m_bounds.scale(pow(1.1, -static_cast<int>(step)));
    m_bounds.move(coord);
}

bool Map::set_zoom(double, double factor)
{
    m_bounds = m_initial_bounds;
    m_bounds.scale(1.0 / factor);
    return true;
}

bool Map::point_x(double value, double)
{
    if (m_drag)
        m_bounds.move({m_bounds.width() * (m_pointer.x - value), 0.0});
    m_pointer.x = value;
    return true;
}

bool Map::point_y(double value, double)
{
    auto new_y{1.0 - value};
    if (m_drag)
        m_bounds.move({0.0, m_bounds.height() * (m_pointer.y - new_y)});
    m_pointer.y = new_y;
    return true;
}

bool Map::mouse_button(double is_presed, double)
{
    m_drag = static_cast<bool>(is_presed);
    return true;
}

//-----------------------------------------------------------------------------
void show_full_window(double width, double height)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, width, height);
    glScissor(0, 0, width, height);
    glStencilFunc(GL_ALWAYS, 1, 1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

Gl_World::Gl_World(Vamos_Track::Strip_Track& track, Atmosphere const& atmosphere, Sounds& sounds,
                   bool full_screen)
    : World(track, atmosphere),
      m_window{800, 500, "Vamos", full_screen},
      m_timer{100, 10},
      m_sounds{sounds}
{
    reshape(m_window.width(), m_window.height());
    set_paused(true);

    SDL_JoystickOpen(0);
    glEnable(GL_DEPTH_TEST);    // Enable depth testing for hidden line removal
    glEnable(GL_BLEND);         // Allow transparency.
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_TEXTURE_2D);    // Allow textures.
    glEnable(GL_SCISSOR_TEST);  // Allow viewports.
    glEnable(GL_STENCIL_TEST);  // Allow masking.
    glClearStencil(0);
    glEnable(GL_LIGHTING);      // Allow lighting.
    glEnable(GL_LIGHT0);
    GLfloat light_pos[] = {0.0, -1.0, 1.0, 0.0};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    GLfloat ambient[] = {0.7, 0.7, 0.7, 1.0}; // R, G, B, A
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glClearColor(0.32, 0.65, 0.34, 0.0);
}

void Gl_World::add_car(std::shared_ptr<Car> car, std::unique_ptr<Driver> driver)
{
    // If there's a controlled car, show the view from inside it. Otherwise show
    // the view from the trackside cameras.
    auto interactive{driver->is_interactive()};
    World::add_car(car, std::move(driver));
    m_view = interactive ? View::body : View::world;
    if (interactive)
        set_focused_car(m_cars.size() - 1);
}

void Gl_World::read(std::string world_file, std::string controls_file)
{
    // Remember the file names for re-reading.
    m_world_file = world_file;
    m_controls_file = controls_file;
    read_world_file(m_world_file);
    read_controls_file(m_controls_file);
}

void Gl_World::set_paused(bool is_paused)
{
    m_timer.set_paused(is_paused);
    m_paused = is_paused;
    for (auto& info : m_cars)
        info.car->set_paused(is_paused);
    if (is_paused)
        m_sounds.pause();
}

bool Gl_World::pause(double, double)
{
    set_paused(!m_paused);
    return true;
}

bool Gl_World::quit(double, double)
{
    m_done = true;
    return true;
}

bool Gl_World::read_car(double, double)
{
    if (controlled_car())
    {
        controlled_car()->car->read();
        controlled_car()->car->make_rear_view_mask(m_window.width(), m_window.height());
    }
    return true;
}

bool Gl_World::read_track(double, double)
{
    m_track.read();
    display();
    return true;
}

bool Gl_World::read_world(double, double)
{
    read();
    return true;
}

bool Gl_World::reset_car(double, double)
{
    World::reset();
    return true;
}

bool Gl_World::restart_car(double, double)
{
    World::restart();
    return true;
}

bool Gl_World::cycle_view(double, double)
{
    switch (m_view)
    {
    case View::body:
        m_view = View::chase;
        glClearColor(0.32, 0.65, 0.34, 0.0);
        break;
    case View::chase:
        m_view = View::map;
        break;
    case View::map:
        if (focused_car())
            m_view = View::world;
        break;
    case View::world:
    default:
        m_view = View::body;
        break;
    }

    return true;
}

bool Gl_World::toggle_graphics(double, double)
{
    m_update_graphics = !m_update_graphics;
    m_timer.use_fixed_time_step(!m_update_graphics);
    return true;
}

bool Gl_World::focus_next_car(double, double)
{
    focus_other_car(1);
    return true;
}

bool Gl_World::focus_previous_car(double, double)
{
    focus_other_car(-1);
    return true;
}

bool Gl_World::replay(double, double)
{
    set_paused(true);
    if (m_cars[0].m_record.empty())
        return true;

    auto const& first{m_cars.front()};
    auto real_time{first.m_record[0].m_time};
    auto last_ticks{SDL_GetTicks()};
    for (size_t i = 0; i < first.m_record.size(); ++i)
    {
        for (auto const& car : m_cars)
        {
            auto const& record{car.m_record[i]};
            car.car->chassis().set_position(record.m_position);
            car.car->chassis().set_orientation(record.m_orientation);
        }

        if (first.m_record[i].m_time >= real_time)
            display();
        check_for_events();
        auto now{SDL_GetTicks()};
        real_time += 0.001 * (now - last_ticks);
        last_ticks = now;
    }
    return true;
}

void Gl_World::animate()
{
    if (focused_car())
    {
        for (auto i{0}; i < steps_per_frame; ++i)
            propagate_cars(m_timer.get_time_step());
        play_sounds();
        update_car_timing();
    }
    m_timer.add_frame();
}

void Gl_World::update_car_timing()
{
    for (size_t i{0}; i < m_cars.size(); ++i)
    {
        auto& car{m_cars[i]};
        if (!car.driver->is_driving())
            car.driver->start(mp_timing->countdown());
        auto distance{car.track_position().x};
        auto sector{m_track.sector(distance)};
        mp_timing->update(m_timer.get_current_time(), i, distance, sector);
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
        if (touch.car != focused_car()->car.get())
            continue;

        switch (touch.track_material)
        {
        case Material::asphalt:
        case Material::concrete:
        case Material::metal:
            if (touch.car_material == Material::rubber)
                tire_slide = touch.car->slide();
            else if (touch.car_material == Material::metal)
            {
                scrape_speed = touch.parallel_speed;
                hard_crash_speed = touch.perpendicular_speed;
            }
            break;
        case Material::kerb:
            kerb_speed = touch.parallel_speed;
            break;
        case Material::grass:
            grass_speed = touch.parallel_speed;
            break;
        case Material::gravel:
            gravel_speed = touch.parallel_speed;
            break;
        case Material::rubber:
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

void Gl_World::draw_mirror_views()
{
    for (int i{0}; i < focused_car()->car->get_n_mirrors(); ++i)
    {
        // Draw the view and save the position.
        auto pos{focused_car()->car->draw_rear_view(m_window.aspect(), i)};
        glMatrixMode(GL_MODELVIEW);
        glStencilFunc(GL_EQUAL, 1, 1); // Enable the rearview mirror mask.
        glPushAttrib(GL_POLYGON_BIT);
        {
            // Front and back are reversed in the mirrors, so cull front faces while
            // drawing mirror views.
            glCullFace(GL_FRONT);
            m_track.draw_sky(pos);
            m_track.draw();
            draw_cars(false, false);
        }
        glPopAttrib();
    }
}

void Gl_World::set_world_view(Vamos_Geometry::Three_Vector const& camera_position,
                              Vamos_Geometry::Three_Vector const& target_position,
                              double vertical_field_angle)
{
    gluPerspective(vertical_field_angle, m_window.aspect(), 1.0, 10000.0);
    gluLookAt(camera_position.x, camera_position.y, camera_position.z, target_position.x,
              target_position.y, target_position.z, 0.0, 0.0, 1.0); // up direction

    auto direction(target_position - camera_position);
    alListener3f(AL_POSITION, camera_position.x, camera_position.y, camera_position.z);
    alListener3f(AL_VELOCITY, 0.0f, 0.0f, 0.0f);
    float at_up[6] = {static_cast<float>(direction.x), static_cast<float>(direction.y),
                      static_cast<float>(direction.z), 0.0f, 0.0f, 1.0f};
    alListenerfv(AL_ORIENTATION, at_up);
}

void Gl_World::set_world_view(Vamos_Track::Camera const& camera)
{
    set_world_view(m_track.camera_position(camera),
                   camera.fixed ? m_track.camera_target(camera)
                                : focused_car()->car->chassis().cm_position(),
                   camera.vertical_field_angle);
}

void Gl_World::draw_track(bool draw_sky, Three_Vector const& view_position)
{
    glMatrixMode(GL_MODELVIEW);
    if (draw_sky)
        m_track.draw_sky(view_position);
    else
        m_track.draw_map_background();
    m_track.draw();
}

void Gl_World::draw_cars(bool draw_interior, bool draw_focused_car)
{
    for (auto const& car : m_cars)
    {
        assert(car.car);
        if (car.car != focused_car()->car)
            car.car->draw();
    }
    if (!draw_focused_car)
        return;

    focused_car()->car->draw();
    if (draw_interior)
        focused_car()->car->draw_interior();
    if (focused_car()->driver)
        focused_car()->driver->draw();
}

void Gl_World::display()
{
    if (m_view == View::body)
        focused_car()->car->make_rear_view_mask(m_window.width(), m_window.height());
    show_full_window(m_window.width(), m_window.height());

    SDL_ShowCursor(m_view == View::map || m_has_mouse_control);
    switch (m_view)
    {
    case View::body:
    {
        auto& car{*focused_car()->car};
        car.set_perspective(m_window.aspect());
        car.view();
        draw_track(true, focused_car()->car->view_position(true, true));
        draw_cars(true);
        draw_timing_info();
        draw_mirror_views();
        break;
    }
    case View::chase:
    {
        auto& car{*focused_car()->car};
        auto chase_pos{car.chase_position()};
        set_world_view(chase_pos, car.chassis().cm_position(), 45.0);
        draw_track(true, chase_pos);
        draw_cars(true);
        draw_timing_info();
        break;
    }
    case View::map:
        m_map.set_view();
        draw_track(false, null_v);
        if (focused_car())
        {
            draw_cars(false);
            draw_timing_info();
        }
        break;
    case View::world:
    {
        if (focused_car())
        {
            auto const& camera{m_track.get_camera(
                    mp_timing->timing_at_index(m_focused_car_index).lap_distance())};
            set_world_view(camera);
            draw_track(true, m_track.camera_position(camera));
        }
        draw_cars(true);
        draw_timing_info();
    }
    break;
    }

    m_window.refresh();
}

/// Format a time as minutes:seconds.
static std::string time_str(double time, int precision = 3)
{
    if (time == Timing_Info::no_time)
        return "";

    auto minutes{static_cast<int>(time / 60.0)};
    auto seconds{time - 60 * minutes};
    // Show the leading zero on the seconds. Add 1 for the decimal point if present.
    auto width{precision > 0 ? precision + 3 : 2};

    std::ostringstream os;
    os << minutes << ':' << std::fixed << std::setfill('0') << std::setw(width)
       << std::setprecision(precision) << seconds;
    return os.str();
}

/// Format a signed difference in seconds.
static std::string dtime_str(double delta_time, int precision = 3)
{
    if (delta_time == Timing_Info::no_time)
        return "";

    std::ostringstream os;
    if (delta_time > 0.0)
        os << '+';
    os << std::fixed << std::setprecision(precision) << delta_time;
    return os.str();
}

/// Set the limits of mouse motion for interactive drivers.
static void set_mouse_range(Car_Info* car, int width, int height)
{
    if (!car)
        return;
    if (auto driver{dynamic_cast<Interactive_Driver*>(car->driver.get())})
    {
        driver->mouse().set_axis_range(X, 0, width);
        driver->mouse().set_axis_range(Y, 0, height);
    }
}

void Gl_World::reshape(int width, int height)
{
    m_window.resize(width, height);
    set_mouse_range(controlled_car(), width, height);
    if (focused_car())
        focused_car()->car->make_rear_view_mask(width, height);
    m_map.set_bounds(m_track, m_window);
}

void Gl_World::draw_timing_info() const
{
    Two_D screen;
    auto count{mp_timing->countdown()};
    if (count > 0)
        screen.lights({50, 60}, 1.0, 5, count, red_light_on, red_light_off);

    if (mp_timing->running_order().size() > 1)
        draw_leaderboard(screen);
    else
        draw_lap_times(screen);

    // Draw timing info for the focused car.
    auto car{mp_timing->timing_at_index(m_focused_car_index)};
    auto dt{dtime_str(car.lap_time_difference())};
    auto down = [](auto& p, auto n) { p.y -= n; return p; };
    Two_Vector p{55, 18};
    screen.text(down(p, 4), "Lap Time", time_str(car.current_lap_time()));
    screen.text(down(p, 4), "    Last", time_str(car.previous_lap_time()), dt);
    screen.text(down(p, 4), "    Best", time_str(car.best_lap_time()));
    screen.text(down(p, 4), "frames/s", static_cast<int>(m_timer.get_frame_rate() + 0.5));

    p = {75, 18};
    dt = dtime_str(car.previous_sector_time_difference());
    screen.text(down(p, 4), "     Sector", time_str(car.sector_time()));
    screen.text(down(p, 4), "       Best", time_str(car.best_sector_time()));
    screen.text(down(p, 4), "Last Sector", time_str(car.previous_sector_time()), dt);
    screen.text(down(p, 4), "   Distance", static_cast<int>(car.lap_distance()), " m");
}

void Gl_World::draw_leaderboard(Vamos_Media::Two_D& screen) const
{
    Two_Vector p{2, 95};
    auto const& order{mp_timing->running_order()};
    if (m_track.get_road(0).is_closed())
    {
        if (mp_timing->is_finished())
            screen.text(p, "Finish");
        else if (mp_timing->is_qualifying() && mp_timing->total_laps() == 0)
            screen.text(p, "", time_str(mp_timing->time_remaining(), 0));
        else
        {
            std::ostringstream os;
            os << order.front()->current_lap() << '/' << mp_timing->total_laps();
            screen.text(p, "Lap", os.str());
        }
    }

    p.y -= 3;
    // Show absolute lap time for the leader.
    auto time{time_str(mp_timing->is_qualifying() ? order.front()->best_lap_time()
                                                  : order.front()->previous_lap_time())};
    screen.text(p, m_cars[order.front()->grid_position() - 1].car->name(), time);
    p.y -= 3;
    // Show relative times for the rest.
    for (auto it{std::next(order.cbegin())}; it != order.cend(); ++it, p.y -= 3)
    {
        time = mp_timing->is_qualifying() ? time_str((*it)->best_lap_time())
                                          : dtime_str((*it)->interval());
        screen.text(p, m_cars[(*it)->grid_position() - 1].car->name(), time);
    }
    if (!mp_timing->is_qualifying() && m_track.get_road(0).is_closed())
        draw_fastest_lap(screen, p);
}

void Gl_World::draw_lap_times(Vamos_Media::Two_D& screen) const
{
    auto info{mp_timing->running_order().front()};
    Two_Vector p{2, 95};
    screen.text(p, "Lap", "Time");
    p.y -= 3;
    auto last_time{0.0};
    for (size_t i{1}; i < info->current_lap(); ++i, p.y -= 3)
    {
        screen.text(p, i, time_str(info->lap_time(i) - last_time));
        last_time = info->lap_time(i);
    }
    // Draw the lap number with no time for the current lap.
    screen.text(p, info->current_lap(), "");
    draw_fastest_lap(screen, {p.x, p.y - 3});
}

void Gl_World::draw_fastest_lap(Vamos_Media::Two_D& screen, Two_Vector const& p) const
{
    screen.text(p, "Fastest Lap");
    auto const* fastest{mp_timing->fastest_lap_timing()};
    if (fastest && (fastest->best_lap_time() != Timing_Info::no_time))
        screen.text({p.x, p.y - 3}, m_cars[fastest->grid_position() - 1].car->name(),
                    time_str(fastest->best_lap_time()));
}

void Gl_World::start(bool qualifying, size_t laps_or_minutes)
{
    World::start(qualifying, laps_or_minutes);
    m_map.set_bounds(m_track, m_window);
    if (!m_cars.empty())
        set_paused(false);
    m_timer.reset();
    SDL_Event event;
    // Flush the event queue.
    while (SDL_PollEvent(&event))
        ;

    while (!m_done)
    {
        m_timer.update();
        check_for_events();
        if (m_paused)
            SDL_Delay(100);
        else
            animate();
        // Don't skip when paused. Need to respond to camera and focus changes.
        if (m_update_graphics)
            display();
    }
}

void Gl_World::check_for_events()
{
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        Interactive_Driver* driver{nullptr};
        if (controlled_car())
        {
            driver = dynamic_cast<Interactive_Driver*>(controlled_car()->driver.get());
            if (!driver)
                continue;
        }

        switch (event.type)
        {
        case SDL_JOYAXISMOTION:
            if (driver)
                driver->joystick().move(event.jaxis.axis, event.jaxis.value);
            break;
        case SDL_JOYBUTTONDOWN:
            if (driver)
                driver->joystick().press(event.jbutton.button + 1);
            break;
        case SDL_JOYBUTTONUP:
            if (driver)
                driver->joystick().release(event.jbutton.button + 1);
            break;
        case SDL_KEYDOWN:
            keyboard().press(event.key.keysym.sym);
            if (driver)
                driver->keyboard().press(event.key.keysym.sym);
            if (m_view == View::map)
                m_map.keyboard().press(event.key.keysym.sym);
            break;
        case SDL_KEYUP:
            keyboard().release(event.key.keysym.sym);
            if (driver)
                driver->keyboard().release(event.key.keysym.sym);
            if (m_view == View::map)
                m_map.keyboard().release(event.key.keysym.sym);
            break;
        case SDL_MOUSEMOTION:
            if (driver)
            {
                driver->mouse().move(X, event.motion.x);
                driver->mouse().move(Y, event.motion.y);
            }
            if (m_view == View::map)
            {
                m_map.mouse().move(X, event.motion.x);
                m_map.mouse().move(Y, event.motion.y);
            }
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (driver)
                driver->mouse().press(event.button.button);
            if (m_view == View::map)
                m_map.mouse().press(event.button.button);
            break;
        case SDL_MOUSEBUTTONUP:
            if (driver)
                driver->mouse().release(event.button.button);
            if (m_view == View::map)
                m_map.mouse().release(event.button.button);
            break;
        case SDL_MOUSEWHEEL:
            if (m_view == View::map)
                m_map.mouse().increment(0, event.wheel.y);
            break;
        case SDL_WINDOWEVENT:
            if (event.window.event == SDL_WINDOWEVENT_RESIZED)
                reshape(event.window.data1, event.window.data2);
            break;
        case SDL_QUIT:
            quit();
            break;
        }
    }
}

void Gl_World::focus_other_car(int delta)
{
    while (delta < 0)
        delta += m_cars.size();
    auto index{m_focused_car_index + static_cast<size_t>(delta)};
    set_focused_car(index % m_cars.size());
}

void Gl_World::set_focused_car(size_t index)
{
    if (m_cars.empty())
        return;
    assert(index < m_cars.size());
    m_focused_car_index = index;
    if (focused_car())
        focused_car()->car->make_rear_view_mask(m_window.width(), m_window.height());
}

Car_Info* Gl_World::focused_car()
{
    return m_cars.empty() ? nullptr : &m_cars[m_focused_car_index];
}

void Gl_World::read_world_file(std::string const& file_name)
{
    pugi::xml_document doc;
    auto result{doc.load_file(file_name.c_str())};
    auto world_node{doc.child("world")};
    set_gravity(world_node.child("gravity").text().as_double());

    auto atm_node{world_node.child("atmosphere")};
    m_atmosphere.density = atm_node.child("density").text().as_double();
    std::istringstream is{atm_node.child_value("velocity")};
    is >> m_atmosphere.velocity;
    alSpeedOfSound(atm_node.child("speed-of-sound").text().as_float());

    auto light_node{world_node.child("lighting")};
    is.str(light_node.child_value("source-position"));
    Three_Vector v;
    is >> v;
    GLfloat position[] = {float(v.x), float(v.y), float(v.z), 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    is.str(light_node.child_value("ambient"));
    is >> v;
    GLfloat ambient[] = {float(v.x), float(v.y), float(v.z), 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
}

/// A map of key names to SDL symbols.
std::map<std::string, int> key_map{
    {"escape", 27},
    {"delete", 127},
    {"up", SDLK_UP},
    {"down", SDLK_DOWN},
    {"left", SDLK_LEFT},
    {"right", SDLK_RIGHT},
    {"insert", SDLK_INSERT},
    {"home", SDLK_HOME},
    {"end", SDLK_END},
    {"page up", SDLK_PAGEUP},
    {"page down", SDLK_PAGEDOWN},
    {"tab", SDLK_TAB},
    {"f1", SDLK_F1},
    {"f2", SDLK_F2},
    {"f3", SDLK_F3},
    {"f4", SDLK_F4},
    {"f5", SDLK_F5},
    {"f6", SDLK_F6},
    {"f7", SDLK_F7},
    {"f8", SDLK_F8},
    {"f9", SDLK_F9},
    {"f10", SDLK_F10},
    {"f11", SDLK_F11},
    {"f12", SDLK_F12},
};

/// A map of mouse button names to SDL symbols.
std::map<std::string, int> button_map{
    {"left", SDL_BUTTON_LEFT},
    {"middle", SDL_BUTTON_MIDDLE},
    {"right", SDL_BUTTON_RIGHT},
};

/// @return A string converted to all lowercase.
static std::string downcase(std::string str)
{
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    return str;
}

/// @return The SDL symbol for the given key name according to @p key_map.
static int translate_key(std::string key_name)
{
    // If key_name is a single character, return its integer value.
    if (key_name.size() == 1)
        return static_cast<int>(key_name[0]);
    // Do case-insensitive lookup.
    return key_map[downcase(key_name)];
}

static Direct get_direction(pugi::xml_node const& binding)
{
    if (binding.child("up"))
        return Direct::up;
    if (binding.child("left"))
        return Direct::left;
    if (binding.child("right"))
        return Direct::right;
    if (binding.child("forward"))
        return Direct::forward;
    if (binding.child("backward"))
        return Direct::backward;
    return Direct::none;
}

static Calibration get_calibration(pugi::xml_node const& binding, Direct direction)
{
    Calibration calib;
    if (binding.child("factor"))
        calib.factor = binding.child("factor").text().as_double();
    if (binding.child("offset"))
        calib.offset = binding.child("offset").text().as_double();
    if (binding.child("deadband"))
        calib.deadband = binding.child("deadband").text().as_double();
    if (binding.child("upper-deadband"))
        calib.upper_deadband = binding.child("upper-deadband").text().as_double();

    // Use a one-sided calibration if only one direction was specified.
    calib.negative = direction != Direct::forward && direction != Direct::left;
    calib.positive = direction != Direct::backward && direction != Direct::right;
    return calib;
}

void Gl_World::read_controls_file(std::string const& file_name)
{
    /// Turn a map of names and callbacks into a Call_Map for the given object.
    auto bindings = [](auto const& fns, auto* obj)
    {
        std::map<std::string, Callback_Fn> call;
        for (auto fn : fns)
            call[fn.first] = std::bind_front(fn.second, obj);
        return call;
    };

    using World_CBs = std::map<std::string, std::function<bool(Gl_World*, double, double)>>;
    auto world_function_map = bindings(
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
        this);

    auto car{controlled_car()};
    set_mouse_range(car, m_window.width(), m_window.height());
    auto driver{car ? dynamic_cast<Interactive_Driver*>(car->driver.get()) : nullptr};
    using Driver_CBs
        = std::map<std::string, std::function<bool(Interactive_Driver*, double, double)>>;
    auto driver_function_map = bindings(
        Driver_CBs{
            {"start engine", &Interactive_Driver::start_engine},
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

    pugi::xml_document doc;
    auto result{doc.load_file(file_name.c_str())};
    auto top{doc.child("controls")};
    for (auto binding : top.children("bind"))
    {
        Control_Handler* handler{this};
        auto function{binding.child_value("function")};
        auto direction{get_direction(binding)};
        auto calib{get_calibration(binding, direction)};
        auto time{binding.child("time") ? binding.child("time").text().as_double() : 0.0};

        auto it{world_function_map.find(function)};
        if (it == world_function_map.end())
        {
            it = driver_function_map.find(function);
            if (it == driver_function_map.end())
                throw Unknown_Function(function);
            if (!driver)
                continue;
            handler = driver;
        }
        auto callback{it->second};

        if (binding.child("key"))
        {
            auto control{translate_key(binding.child_value("key"))};
            handler->keyboard().bind_action(control, direction, callback, time);
        }
        else if (binding.child("button"))
        {
            auto control{binding.child("button").text().as_int()};
            handler->joystick().bind_action(control, direction, callback, time);
        }
        else if (binding.child("axis"))
        {
            auto control{binding.child("axis").text().as_int()};
            handler->joystick().bind_motion(control, callback, calib);
        }
        else if (binding.child("mouse-button"))
        {
            auto control{button_map[downcase(binding.child_value("mouse-button"))]};
            handler->mouse().bind_action(control, direction, callback, time);
        }
        else if (binding.child("mouse-axis"))
        {
            m_has_mouse_control = true;
            auto control{binding.child("mouse-axis").text().as_int()};
            handler->mouse().bind_motion(control, callback, calib);
        }
    }
}
