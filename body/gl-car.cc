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
//  You should have received a copy of the GNU General Public License
//  along with Vamos.  If not, see <http://www.gnu.org/licenses/>.

#include "gl-car.h"

#include "clutch.h"
#include "dashboard.h"
#include "engine.h"
#include "fuel-tank.h"
#include "transmission.h"
#include "wheel.h"

#include "../geometry/conversions.h"
#include "../geometry/numeric.h"
#include "../geometry/rectangle.h"
#include "../media/model.h"
#include "../media/sample.h"
#include "../media/two-d.h"

#include <AL/alut.h>
#include <GL/glut.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

//----------------------------------------------------------------------------------------
namespace Vamos_Body
{
/// Information about rearview mirrors.
class Rear_View_Mirror
{
public:
    Rear_View_Mirror(Vamos_Geometry::Three_Vector const& position,
                     double width, double height, double direction,
                     double field, double near_plane, double far_plane,
                     std::string const& mask_file);

    void make_mask(int window_width, int window_height,
                   Vamos_Geometry::Three_Vector const& driver_position,
                   double driver_field, double pan);
    void set_view();

    double get_direction() const { return m_direction; }
    Vamos_Geometry::Three_Vector get_center() const;

private:
    void set_viewport(int window_width, int window_height,
                      const Vamos_Geometry::Three_Vector& driver_position,
                      double driver_field_of_view, double pan);
    void activate_viewport();
    void transform_view() const;
    void draw_mask_shape();
    void set_stencil(int window_width, int window_height);

    const Vamos_Geometry::Three_Vector m_position;
    const double m_width;
    const double m_height;
    const double m_direction;
    const double m_field;
    const double m_near_plane;
    const double m_far_plane;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_mask;

    Vamos_Geometry::Rectangle m_viewport;
};
} // namespace Vamos_Body

Rear_View_Mirror::Rear_View_Mirror(Three_Vector const& position, double width, double height,
                                   double direction, double field, double near_plane,
                                   double far_plane, std::string const& mask_file)
    : m_position(position),
      m_width(width),
      m_height(height),
      m_direction(direction),
      m_field(field),
      m_near_plane(near_plane),
      m_far_plane(far_plane),
      mp_mask{std::make_unique<Texture_Image>(mask_file, false, false)}
{
}

void Rear_View_Mirror::activate_viewport()
{
    glViewport(m_viewport.left(), m_viewport.top(), m_viewport.width(), m_viewport.height());
    glScissor(m_viewport.left(), m_viewport.top(), m_viewport.width(), m_viewport.height());
}

Three_Vector Rear_View_Mirror::get_center() const
{
    return {m_position.x, m_position.y - m_width / 2.0, m_position.z + m_height / 2.0};
}

void Rear_View_Mirror::transform_view() const
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // Reflect the x-axis.
    glScaled(-1.0, 1.0, 1.0);
    gluPerspective(m_field, m_viewport.aspect(), m_near_plane, m_far_plane);
}

void Rear_View_Mirror::set_view()
{
    activate_viewport();
    glClear(GL_DEPTH_BUFFER_BIT);
    transform_view();
}

void Rear_View_Mirror::make_mask(int window_width, int window_height,
                                 Three_Vector const& driver_position,
                                 double driver_field, double pan)
{
    glDisable(GL_LIGHTING);
    set_viewport(window_width, window_height, driver_position, driver_field, pan);
    draw_mask_shape();
    set_stencil(window_width, window_height);
    glEnable(GL_LIGHTING);
}

void Rear_View_Mirror::set_viewport(int window_width, int window_height,
                                    Three_Vector const& driver_position,
                                    double driver_field, double pan)
{
    auto to_pixels = [](double range, double factor, double coordinate) {
        return std::floor(0.5 * range * (1.0 - factor * coordinate));
    };
    auto pos{rotate(m_position - driver_position, -deg_to_rad(pan) * Three_Vector::Z)};
    auto y_factor{-1.0 / (pos.x * std::tan(0.5 * deg_to_rad(driver_field)))};
    auto aspect{static_cast<double>(window_width) / window_height};
    auto x_factor{-y_factor / aspect};
    //!!! Need to keep pixels as integers.
    Two_Vector p1{to_pixels(window_width, x_factor, pos.y) - 1,
                  to_pixels(window_height, y_factor, pos.z) - 1};
    Two_Vector p2{to_pixels(window_width, x_factor, pos.y - m_width),
                  to_pixels(window_height, y_factor, pos.z + m_height)};
    m_viewport = Rectangle{p1, p2};
    m_viewport.clip(Rectangle{0, 0,
                              static_cast<double>(window_width - 1),
                              static_cast<double>(window_height - 1),
                              true});
}

void Rear_View_Mirror::draw_mask_shape()
{
    glStencilFunc(GL_ALWAYS, 1, 1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

    mp_mask->activate();

    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 1.0);
    glVertex3d(m_position.x, m_position.y, m_position.z);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(m_position.x, m_position.y - m_width, m_position.z);
    glTexCoord2d(1.0, 0.0);
    glVertex3d(m_position.x, m_position.y - m_width, m_position.z + m_height);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(m_position.x, m_position.y, m_position.z + m_height);
    glEnd();

    glFlush();
}

/// Grab the current buffer as an array of pixels.
auto make_stencil_buffer = [](auto const& rect) {
    glReadBuffer(GL_BACK);
    auto size{rect.width() * rect.height()};
    auto rgba_buffer{std::make_unique<unsigned char[]>(4 * size)};
    glReadPixels(rect.left(), rect.top(), rect.width(), rect.height(),
                 GL_RGBA, GL_UNSIGNED_BYTE, rgba_buffer.get());

    auto buffer{std::make_unique<unsigned char[]>(size)};
    for (size_t i{0}; i < size; ++i)
        buffer[i] = rgba_buffer[4 * i];
    return buffer;
};

void Rear_View_Mirror::set_stencil(int window_width, int window_height)
{
    auto buffer{make_stencil_buffer(m_viewport)};

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, double(window_width), 0.0, double(window_height));
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glStencilFunc(GL_EQUAL, 1, 1);
    glStencilOp(GL_KEEP, GL_REPLACE, GL_REPLACE);
    glRasterPos2i(m_viewport.left(), m_viewport.top());
    glDrawPixels(m_viewport.width(), m_viewport.height(),
                 GL_STENCIL_INDEX, GL_UNSIGNED_BYTE,
                 buffer.get());
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glFinish();
}

//----------------------------------------------------------------------------------------
Gl_Car::Gl_Car(Three_Vector const& position, Three_Matrix const& orientation)
    : Car(position, orientation)
{
}

Gl_Car::~Gl_Car()
{
    if (m_body_list_id)
        glDeleteLists(m_body_list_id, 1);
    if (m_interior_list_id)
        glDeleteLists(m_interior_list_id, 1);
}

void Gl_Car::read(std::string const& data_dir, std::string const& car_file)
{
    m_mirrors.clear();
    Car::read(data_dir, car_file);
}

void Gl_Car::set_dashboard(std::unique_ptr<Dashboard> dash)
{
    mp_dashboard = std::move(dash);
}

void Gl_Car::set_exterior_model(std::string const& file, double scale,
                                Three_Vector const& translation,
                                Three_Vector const& rotation)
{
    if (m_body_list_id)
        glDeleteLists(m_body_list_id, 1);
    m_body_list_id = Ac3d(file, scale, translation, rotation).build();
}

void Gl_Car::set_interior_model(std::string const& file, double scale,
                                Three_Vector const& translation,
                                Three_Vector const& rotation)
{
    if (m_interior_list_id)
        glDeleteLists(m_interior_list_id, 1);
    m_interior_list_id = Ac3d(file, scale, translation, rotation).build();
}

void Gl_Car::set_perspective(double aspect)
{
    gluPerspective(m_field_of_view, aspect, m_near_plane, m_far_plane);
}

void Gl_Car::set_view(Vamos_Geometry::Three_Vector const& position, double field,
                      double near_plane, double far_plane, double pan_angle)
{
    m_driver_view = position;
    m_field_of_view = field;
    m_near_plane = near_plane;
    m_far_plane = far_plane;
    m_pan_angle = pan_angle;
}

void Gl_Car::add_rear_view(Vamos_Geometry::Three_Vector const& position,
                           double width, double height, double direction, double field,
                           double near_plane, double far_plane,
                           std::string const& mask_file)
{
    m_mirrors.push_back(
        std::make_unique<Rear_View_Mirror>(position, width, height, direction,
                                           field, near_plane, far_plane, mask_file));
}

void Gl_Car::update_rear_view_mask(int window_width, int window_height)
{
    make_rear_view_mask(window_width, window_height);
}

void transform_body(Rigid_Body const& body)
{
    auto const& pos{body.position()};
    double angle;
    auto axis{body.axis_angle(angle)};
    glLoadIdentity();
    glTranslatef(pos.x, pos.y, pos.z);
    glRotatef(angle, axis.x, axis.y, axis.z);
}

void Gl_Car::make_rear_view_mask(int window_width, int window_height)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, window_width, window_height);
    glScissor(0, 0, window_width, window_height);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearStencil(0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    static auto const near_plane{0.2};
    static auto const far_plane{10.0};
    gluPerspective(field_of_view(), static_cast<double>(window_width) / window_height,
                   near_plane, far_plane);
    view();
    glMatrixMode(GL_MODELVIEW);
    transform_body(m_chassis);
    for (auto const& mirror : m_mirrors)
        mirror->make_mask(window_width, window_height, view_position(false, true),
                          field_of_view(), m_pan_key_control.value());
}

Three_Vector Gl_Car::draw_rear_view(double, int index)
{
    auto& mirror{*m_mirrors[index]};
    mirror.set_view();
    auto pos{m_chassis.position() + m_chassis.rotate_out(mirror.get_center())};
    view(mirror.get_direction(), pos);
    return pos;
}

void Gl_Car::draw()
{
    if (!m_body_list_id)
        return;

    transform_body(m_chassis);
    // Draw the body.
    glCallList(m_body_list_id);
    // Draw the wheels.
    for (auto const& wheel : m_wheels)
        wheel->draw();
}

void Gl_Car::draw_interior()
{
    glCallList(m_interior_list_id);
    if (mp_dashboard)
        draw_dashboard();
}

void Gl_Car::draw_dashboard()
{
    assert(mp_drivetrain);
    auto const& dt{*mp_drivetrain};
    mp_dashboard->set_tachometer(rad_s_to_rpm(dt.engine().rotational_speed()));
    mp_dashboard->set_speedometer(m_s_to_km_h(wheel(2).speed()));
    mp_dashboard->set_fuel_gauge(fuel_tank()->fuel());
    mp_dashboard->set_gear_indicator(dt.transmission().gear());
    mp_dashboard->set_steering_wheel(m_steer_key_control.value());
    mp_dashboard->draw();
    if (m_show_dashboard_extras)
        draw_dashboard_extras();
}

void Gl_Car::draw_dashboard_extras()
{
    assert(mp_drivetrain);
    auto const& dt{*mp_drivetrain};
    Two_D screen;
    static auto constexpr bottom{2};
    static auto constexpr top{20};
    auto const gear{dt.transmission().gear()};
    auto down = [](auto& p, auto n) { p.y -= n; return p; };
    Two_Vector p{4, 22};
    screen.text(down(p, 4), "RPM", rad_s_to_rpm(dt.engine().rotational_speed()));
    screen.text(down(p, 4), "Torque", dt.engine().drive_torque(), "Nm");
    screen.text(down(p, 4), "Speed", m_s_to_km_h(wheel(2).speed()), "km/h");
    screen.text(down(p, 4), "Mass", m_chassis.mass(), "kg");
    screen.text(down(p, 4), "Gear", (gear == -1 ? "R"
                                     : gear == 0 ? "N"
                                     : std::to_string(gear) + '0'));

    screen.bar({19, bottom, 2, top - bottom}, cyan, brake_fraction());
    screen.bar({22, bottom, 2, top - bottom}, magenta, throttle_fraction());

    // Show an accelerometer with a limit of 2 g's.
    auto const& a{acceleration(false) / 9.8};
    auto radius{0.25 * (top - bottom)};
    screen.vector({25, top - radius}, radius, gray80, red, Two_Vector(a.y, -a.x) / 2.0);

#ifdef DEBUG
    screen.text({26 + radius, top - radius}, "", a.y, "", 2);
    screen.text({27, top - 2 * radius - 2}, "", a.x, "", 2);

    screen.text({28, 18}, "Tire Temperature, Wear, Grip");
    //! fixme Assume wheels are defined in the order: right front, left front,
    // right rear, left rear.
    screen.text({28, 14}, "", m_wheels[1]->get_tire().temperature(), "", 0);
    screen.text({32, 14}, "", m_wheels[1]->get_tire().wear(), "", 2);
    screen.text({36, 14}, "", m_wheels[1]->get_tire().grip(), "", 2);
    screen.text({40, 14}, "", m_wheels[0]->get_tire().temperature(), "", 0);
    screen.text({44, 14}, "", m_wheels[0]->get_tire().wear(), "", 2);
    screen.text({48, 14}, "", m_wheels[0]->get_tire().grip(), "", 2);
    screen.text({28, 10}, "", m_wheels[3]->get_tire().temperature(), "", 0);
    screen.text({32, 10}, "", m_wheels[3]->get_tire().wear(), "", 2);
    screen.text({36, 10}, "", m_wheels[3]->get_tire().grip(), "", 2);
    screen.text({40, 10}, "", m_wheels[2]->get_tire().temperature(), "", 0);
    screen.text({44, 10}, "", m_wheels[2]->get_tire().wear(), "", 2);
    screen.text({48, 10}, "", m_wheels[2]->get_tire().grip(), "", 2);

    screen.text({28, 6}, "Air Density", m_air_density, "kg/m^3", 3);
#endif

    screen.text({28, bottom}, "Fuel", mp_fuel_tank->fuel(), "L", 1);
}

void Gl_Car::view()
{
    // Called for the front view.
    auto pos{view_position(true, true)};
    view(m_pan_key_control.value(), pos);
    auto pan{deg_to_rad(m_pan_key_control.value())};

    auto z{m_chassis.rotate_out(Three_Vector::Z)};
    auto x{m_chassis.rotate_out(rotate(Three_Vector::X, pan * Three_Vector::Z))};
    float at_up[6] = {float(x.x), float(x.y), float(x.z), float(z.x), float(z.y), float(z.z)};
    alListener3f(AL_POSITION, pos.x, pos.y, pos.z);
    alListenerfv(AL_ORIENTATION, at_up);
    if (mp_drivetrain)
    {
        auto v{m_chassis.velocity(mp_drivetrain->engine().position())};
        auto v_s{alGetDouble(AL_SPEED_OF_SOUND)};
        alListener3f(AL_VELOCITY, v.x / v_s, v.y / v_s, v.z / v_s);
    }
}

void Gl_Car::view(double pan, Three_Vector const& view_pos)
{
    // Find the angle-axis representation of the car's orientation.
    // Called for the front view and rear views.
    double angle;
    auto axis{m_chassis.axis_angle(angle)};
    auto const& a = acceleration(true);

    // Rotate the view.
    glRotated(90, 0.0, 1.0, 0.0);
    // Tilt the view according to the acceleration.
    static auto constexpr view_tilt_factor{0.1};
    glRotated(-90 - view_tilt_factor * a.y, 1.0, 0.0, 0.0);
    glRotated(view_tilt_factor * a.x, 0.0, 1.0, 0.0);
    glRotated(-angle, axis.x, axis.y, axis.z);

    auto z{m_chassis.rotate_out({0.0, 0.0, 1.0})};
    glRotated(-pan, z.x, z.y, z.z);
    glTranslated(-view_pos.x, -view_pos.y, -view_pos.z);
}

void Gl_Car::set_engine_sound(std::string const& file, double volume,
                              double throttle_volume_factor,
                              double engine_speed_volume_factor, double pitch)
{
    if (file.empty())
    {
        mp_engine_sample.reset();
        return;
    }
    m_throttle_volume_factor = throttle_volume_factor;
    m_engine_speed_volume_factor = engine_speed_volume_factor;
    mp_engine_sample = std::make_unique<Sample>(file, volume, pitch, true);
}

void Gl_Car::propagate(double time)
{
    Car::propagate(time);
    if (!mp_engine_sample)
        return;

    auto engine_volume = [&](auto const& engine) {
        return 1.0 + m_throttle_volume_factor * engine.throttle()
            + m_engine_speed_volume_factor * engine.rotational_speed();
    };

    auto const& engine{mp_drivetrain->engine()};
    mp_engine_sample->pitch(engine.rotational_speed());
    mp_engine_sample->volume(engine_volume(engine));
    mp_engine_sample->position(chassis().position(engine));
    mp_engine_sample->velocity(chassis().velocity(engine));
    mp_engine_sample->play();
}

void Gl_Car::set_paused(bool is_paused)
{
    if (!mp_engine_sample)
        return;
    if (is_paused)
        mp_engine_sample->pause();
    else
        mp_engine_sample->play();
}
