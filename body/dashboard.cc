//  Copyright (C) 2005-2022 Sam Varner
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

#include "dashboard.h"

#include "../geometry/numeric.h"
#include "../media/texture-image.h"

#include <cassert>
#include <algorithm>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;

using namespace std::placeholders;
auto make_scaler = [](double min_in, double min_out, double max_in, double max_out) {
    assert(min_in != min_out);
    return std::bind(interpolate<double>, _1, min_in, min_out, max_in, max_out);
};

static void gauge_transform(Two_Vector const& center, double in_front, double rotation)
{
    glTranslatef(0.0, -center.x, center.y);
    glRotated(rotation, 1.0, 0.0, 0.0);
    glRotatef(-90.0, 0.0, 0.0, 1.0);
    glRotatef(90.0, 1.0, 0.0, 0.0);
    glTranslatef(0.0, 0.0, in_front);
}

//----------------------------------------------------------------------------------------
Steering_Wheel::Steering_Wheel(double center_x, double center_y, double above, double radius,
                               double min_in, double min_out, double max_in, double max_out,
                               std::string const& image)
    : Facade(image),
      m_center{center_x, center_y},
      m_above(above),
      m_angle_fn{make_scaler(min_in, min_out, max_in, max_out)}
{
    set_radius(radius);
}

void Steering_Wheel::set(double value)
{
    m_angle = m_angle_fn(value);
}

void Steering_Wheel::draw() const
{
    gauge_transform(m_center, m_above, m_angle);
    Facade::draw();
}

//----------------------------------------------------------------------------------------
Dial::Dial(double center_x, double center_y,
           double above, double radius,
           double min_in, double min_out, double max_in, double max_out,
           std::string const& face_image, std::string const& needle_image)
    : m_center{center_x, center_y},
      m_above{above},
      m_angle_fn{make_scaler(min_in, min_out, max_in, max_out)}
{
    if (!face_image.empty())
    {
        mp_face = std::make_unique<Facade>(face_image);
        mp_face->set_radius(radius);
    }
    if (!needle_image.empty())
    {
        mp_needle = std::make_unique<Facade>(needle_image);
        mp_needle->set_radius(radius);
    }
}

void Dial::set(double value)
{
    m_angle = m_angle_fn(value);
}

void Dial::draw() const
{
    glPushMatrix();
    gauge_transform(m_center, m_above, 0.0);
    mp_face->draw();
    glPopMatrix();

    glPushMatrix();
    gauge_transform(m_center, m_above + 0.01, m_angle);
    mp_needle->draw();
    glPopMatrix();
}

//----------------------------------------------------------------------------------------
LED_Gauge::LED_Gauge(double x, double y, double above, double width, int elements, double min,
                     double redline, std::string const& image, bool on_wheel)
    : m_x(x),
      m_y(y),
      m_above(above),
      m_width(width),
      m_elements(elements),
      m_min(min),
      m_range(redline - min),
      mp_leds{std::make_unique<Texture_Image>(image, true, true)},
      m_list_index(glGenLists(1))
{
    m_on_steering_wheel = on_wheel;
    m_height = m_width / (2.0 * mp_leds->aspect_ratio());

    glNewList(m_list_index, GL_COMPILE);
    mp_leds->activate();
    glTranslatef(-m_above, -m_x, m_y);
    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 0.5);
    glNormal3f(-1.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glTexCoord2d(1.0, 0.5);
    glVertex3d(0.0, -m_width, 0.0);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(0.0, -m_width, m_height);
    glTexCoord2d(0.0, 1.0);
    glVertex3d(0.0, 0.0, m_height);
    glEnd();
    glEndList();
}

void LED_Gauge::set(double value)
{
    m_leds_on = static_cast<int>((value - m_min) * (m_elements - 1) / m_range + 1.0);
    m_leds_on = std::max(m_leds_on, 0);
    m_leds_on = std::min(m_leds_on, m_elements);
}

void wheel_transform(double z)
{
    glTranslatef(0.0, 0.0, -z);
    glRotatef(90.0, 0.0, 0.0, 1.0);
    glRotatef(90.0, 0.0, 1.0, 0.0);
}

void LED_Gauge::draw() const
{
    glPushMatrix();
    if (m_on_steering_wheel)
        wheel_transform(m_above);

    // Draw the LEDs all off then draw the ones that are on over top.
    glCallList(m_list_index);
    auto frac{static_cast<double>(m_leds_on) / m_elements};

    mp_leds->activate();

    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 0.5);
    glNormal3f(-1.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glTexCoord2d(frac, 0.5);
    glVertex3d(0.0, -m_width * frac, 0.0);
    glTexCoord2d(frac, 0.0);
    glVertex3d(0.0, -m_width * frac, m_height);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(0.0, 0.0, m_height);
    glEnd();

    glPopMatrix();
}

//----------------------------------------------------------------------------------------
Digital_Gauge::Digital_Gauge(double x, double y, double above, double width, double height,
                             size_t places, std::string const& digits, bool on_wheel)
    : m_x{x},
      m_y{y},
      m_above{above},
      m_width{width},
      m_height{height},
      m_places{places},
      mp_digits{std::make_unique<Texture_Image>(digits, true, true)}
{
    m_on_steering_wheel = on_wheel;
    m_digits.resize(places);
}

void Digital_Gauge::set(double value)
{
    auto n{static_cast<int>(value)};
    auto denom{1};
    auto sub{0};
    for (size_t i{0}; i < m_places; ++i)
    {
        auto m{denom * 10};
        auto place{(n % m) / denom};
        m_digits[m_places - 1 - i] = place;
        denom = m;
        sub += place;
    }
}

void Digital_Gauge::draw() const
{
    glPushMatrix();
    if (m_on_steering_wheel)
        wheel_transform(m_above);

    mp_digits->activate();

    auto nonzero{false};
    for (size_t i{0}; i < m_places; ++i)
    {
        auto n{m_digits[i]};
        if (!nonzero && n == 0 && i != (m_places - 1))
            continue;
        nonzero = true;

        auto tex_x1{n * 0.1};
        auto tex_x2{tex_x1 + 0.1};
        auto x1{i * m_width / m_places};
        auto x2{x1 + m_width / m_places};

        glColor3d(1.0, 1.0, 1.0);
        glBegin(GL_QUADS);
        glNormal3f(-1.0, 0.0, 0.0);
        glTexCoord2d(tex_x1, 1.0);
        glVertex3d(-m_above, -m_x - x1, m_y);
        glTexCoord2d(tex_x2, 1.0);
        glVertex3d(-m_above, -m_x - x2, m_y);
        glTexCoord2d(tex_x2, 0.0);
        glVertex3d(-m_above, -m_x - x2, m_y + m_height);
        glTexCoord2d(tex_x1, 0.0);
        glVertex3d(-m_above, -m_x - x1, m_y + m_height);
        glEnd();
    }
    glPopMatrix();
}

//----------------------------------------------------------------------------------------
Gear_Indicator::Gear_Indicator(Rectangle<double> const& size, double z, int numbers,
                               std::string const& image, bool on_wheel)
    : m_size{size},
      m_above{z},
      m_number_width{1.0 / numbers}
{
    m_on_steering_wheel = on_wheel;
    if (!image.empty())
        mp_numbers = std::make_unique<Texture_Image>(image, true, true);
}

void Gear_Indicator::draw() const
{
    glPushMatrix();
    if (m_on_steering_wheel)
        wheel_transform(m_above);

    mp_numbers->activate();

    auto x1{m_number_width * (m_gear + 1)};
    auto x2{x1 + m_number_width};
    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glNormal3f(-1.0, 0.0, 0.0);
    glTexCoord2d(x2, 1.0);
    glVertex3d(-m_above, -m_size.left(), m_size.bottom());
    glTexCoord2d(x1, 1.0);
    glVertex3d(-m_above, -m_size.left() + m_size.width(), m_size.bottom());
    glTexCoord2d(x1, 0.0);
    glVertex3d(-m_above, -m_size.left() + m_size.width(), m_size.top());
    glTexCoord2d(x2, 0.0);
    glVertex3d(-m_above, -m_size.left(), m_size.top());
    glEnd();

    glPopMatrix();
}

//----------------------------------------------------------------------------------------
Gear_Shift::Gear_Shift(Rectangle<double> const& size, double z,
                       Three_Vector const& rotation,
                       std::vector<Two_Vector> const& positions,
                       std::string const& plate_image, std::string const& stick_image)
    : Gear_Indicator(size, z, 0, "", false),
      m_rotation(rotation),
      m_positions(positions),
      m_top_gear(m_positions.size() - 2),
      mp_gate{std::make_unique<Texture_Image>(plate_image, true, true)},
      mp_stick{std::make_unique<Texture_Image>(stick_image, true, true)},
      m_list_index(glGenLists(2))
{
    m_stick_width = m_size.width() * mp_stick->width_pixels() / mp_gate->width_pixels();
    m_stick_height = m_size.height() * mp_stick->height_pixels() / mp_gate->height_pixels();

    glNewList(m_list_index, GL_COMPILE);

    mp_gate->activate();

    glRotated(rotation.x, 0.0, -1.0, 0.0);
    glRotated(rotation.y, 0.0, 0.0, 1.0);
    glRotated(rotation.z, 1.0, 0.0, 0.0);
    glTranslated(-m_above, -m_size.left(), m_size.bottom());

    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 0.0);
    glNormal3f(-1.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glTexCoord2d(1.0, 0.0);
    glVertex3d(0.0, -m_size.width(), 0.0);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(0.0, -m_size.width(), m_size.height());
    glTexCoord2d(0.0, 1.0);
    glVertex3d(0.0, 0.0, m_size.height());
    glEnd();

    glTranslated(0.0, (-m_size.width() + m_stick_width) / 2.0, m_size.height() / 2.0);
    glEndList();

    glNewList(m_list_index + 1, GL_COMPILE);

    mp_stick->activate();

    glRotated(-rotation.x, 0.0, -1.0, 0.0);
    glRotated(-rotation.y, 0.0, 0.0, 1.0);
    glRotated(-rotation.z, 1.0, 0.0, 0.0);

    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 1.0);
    glNormal3f(-1.0, 0.0, 0.0);
    glVertex3d(0.0, 0.0, 0.0);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(0.0, -m_stick_width, 0.0);
    glTexCoord2d(1.0, 0.0);
    glVertex3d(0.0, -m_stick_width, m_stick_height);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(0.0, 0.0, m_stick_height);
    glEnd();

    glEndList();
}

void Gear_Shift::set(int gear)
{
    m_gear = std::min(m_top_gear, gear);
}

void Gear_Shift::draw() const
{
    glPushMatrix();
    glCallList(m_list_index);
    glTranslated(0.0, -m_positions[m_gear + 1].x, m_positions[m_gear + 1].y);
    glCallList(m_list_index + 1);
    glPopMatrix();
}

//----------------------------------------------------------------------------------------
Dashboard::Dashboard(double x, double y, double z, double tilt)
    : m_x{x},
      m_y{y},
      m_z{z},
      m_tilt{tilt}
{
}

void Dashboard::add_tachometer(std::unique_ptr<Gauge> tachometer)
{
    mp_tachometer = std::move(tachometer);
}

void Dashboard::add_speedometer(std::unique_ptr<Gauge> speedometer)
{
    mp_speedometer = std::move(speedometer);
}

void Dashboard::add_fuel_gauge(std::unique_ptr<Gauge> fuel_gauge)
{
    mp_fuel_gauge = std::move(fuel_gauge);
}

void Dashboard::add_gear_indicator(std::unique_ptr<Gear_Indicator> gear_indicator)
{
    mp_gear_indicator = std::move(gear_indicator);
}

void Dashboard::add_steering_wheel(std::unique_ptr<Steering_Wheel> steering_wheel)
{
    mp_steering_wheel = std::move(steering_wheel);
}

void Dashboard::add_facade(std::unique_ptr<Facade> facade)
{
    m_facades.push_back(std::move(facade));
}

void Dashboard::set_tachometer(double rpm)
{
    if (mp_tachometer)
        mp_tachometer->set(rpm);
}

void Dashboard::set_speedometer(double speed)
{
    if (mp_speedometer)
        mp_speedometer->set(speed);
}

void Dashboard::set_fuel_gauge(double fuel)
{
    if (mp_fuel_gauge)
        mp_fuel_gauge->set(fuel);
}

void Dashboard::set_gear_indicator(int gear)
{
    if (mp_gear_indicator)
        mp_gear_indicator->set(gear);
}

void Dashboard::set_steering_wheel(double angle)
{
    if (mp_steering_wheel)
        mp_steering_wheel->set(angle);
}

void Dashboard::draw() const
{
    glTranslated(m_x, m_y, m_z);

    for (auto const& facade : m_facades)
    {
        glPushMatrix();
        glRotatef(-90.0, 0.0, 0.0, 1.0);
        glRotatef(90.0, 1.0, 0.0, 0.0);
        facade->draw();
        glPopMatrix();
    }

    glRotated(m_tilt, 0.0, 1.0, 0.0);
    if (mp_tachometer && !mp_tachometer->on_steering_wheel())
        mp_tachometer->draw();
    if (mp_speedometer && !mp_speedometer->on_steering_wheel())
        mp_speedometer->draw();
    if (mp_fuel_gauge && !mp_fuel_gauge->on_steering_wheel())
        mp_fuel_gauge->draw();
    if (mp_gear_indicator && !mp_gear_indicator->on_steering_wheel())
        mp_gear_indicator->draw();
    if (mp_steering_wheel)
        mp_steering_wheel->draw();

    glDisable(GL_DEPTH_TEST);
    if (mp_tachometer && mp_tachometer->on_steering_wheel())
        mp_tachometer->draw();
    if (mp_speedometer && mp_speedometer->on_steering_wheel())
        mp_speedometer->draw();
    if (mp_fuel_gauge && mp_fuel_gauge->on_steering_wheel())
        mp_fuel_gauge->draw();
    if (mp_gear_indicator && mp_gear_indicator->on_steering_wheel())
        mp_gear_indicator->draw();
    glEnable(GL_DEPTH_TEST);
}
