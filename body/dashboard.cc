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
auto make_scaler = [](auto min, auto max) {
    assert(min.x != max.x);
    return std::bind(interpolate<double>, _1, min.x, min.y, max.x, max.y);
};

//----------------------------------------------------------------------------------------
/// RAII class for conditional isolation of transformations.
class Reset_Transform
{
public:
    Reset_Transform(bool reset);
    ~Reset_Transform();
private:
    bool m_reset;
};

Reset_Transform::Reset_Transform(bool reset)
    : m_reset{reset}
{
    if (m_reset)
        glPushMatrix();
}

Reset_Transform::~Reset_Transform()
{
    if (m_reset)
        glPopMatrix();
}

//----------------------------------------------------------------------------------------
Dial::Dial(Three_Vector const& position, double radius, Point<double> min, Point<double> max,
           std::string const& face_image, std::string const& needle_image)
    : Gauge{false},
      m_angle_fn{make_scaler(min, max)},
      mp_face{std::make_unique<Facade>(face_image, position, radius, false)},
      mp_needle{std::make_unique<Facade>(needle_image, position + 0.01 * Three_Vector::Z,
                                         radius, false)}
{
}

void Dial::draw(bool reset) const
{
    mp_face->draw();

    Reset_Transform resetter{reset};
    auto const& pos{mp_needle->get_center()};
    glTranslatef(pos.x, pos.y, 0.0);
    glRotated(-m_angle_fn(m_value), 0.0, 0.0, 1.0);
    glTranslatef(-pos.x, -pos.y, 0.0);
    mp_needle->draw();
    // Transform back to the center in case we're not resetting the transformation.
    glTranslatef(pos.x, pos.y, 0.0);
}

//----------------------------------------------------------------------------------------
LED_Gauge::LED_Gauge(Three_Vector const& position, double width, int elements,
                     Point<double> range, std::string const& image, bool on_wheel)
    : Gauge{on_wheel},
      m_above{position.z},
      m_width{width},
      m_elements{elements},
      m_min{range.x},
      m_range{range.y - range.x},
      mp_leds{std::make_unique<Texture_Image>(image, true, true)},
      m_list_index{glGenLists(1)}
{
    m_height = m_width / (2.0 * mp_leds->image_aspect_ratio());

    glNewList(m_list_index, GL_COMPILE);
    mp_leds->activate();
    glTranslatef(position.x, position.y, position.z);
    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 0.5);
    glNormal3f(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    glTexCoord2d(1.0, 0.5);
    glVertex3d(m_width, 0.0, 0.0);
    glTexCoord2d(1.0, 1.0);
    glVertex3d(m_width, m_height, 0.0);
    glTexCoord2d(0.0, 1.0);
    glVertex3d(0.0, m_height, 0.0);
    glEnd();
    glEndList();
}

void LED_Gauge::draw(bool reset) const
{
    Reset_Transform resetter{reset};
    // Draw the LEDs all off then draw the ones that are on over top.
    glCallList(m_list_index);
    auto leds_on = static_cast<int>((m_value - m_min) * (m_elements - 1) / m_range + 1.0);
    auto frac{static_cast<double>(clip(leds_on, 0, m_elements)) / m_elements};

    mp_leds->activate();
    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 0.5);
    glNormal3f(0.0, 0.0, 1.0);
    glVertex3d(0.0, 0.0, 0.0);
    glTexCoord2d(frac, 0.5);
    glVertex3d(m_width * frac, 0.0, 0.0);
    glTexCoord2d(frac, 0.0);
    glVertex3d(m_width * frac, m_height, 0.0);
    glTexCoord2d(0.0, 0.0);
    glVertex3d(0.0, m_height, 0.0);
    glEnd();
}

//----------------------------------------------------------------------------------------
Digital_Gauge::Digital_Gauge(Three_Vector const& position, Point<double> size,
                             int places, std::string const& digits, bool on_wheel)
    : Gauge{on_wheel},
      m_position{position},
      m_size{size},
      m_places{places},
      mp_digits{std::make_unique<Texture_Image>(digits, true, true)}
{
}

void Digital_Gauge::draw(bool reset) const
{
    auto n{static_cast<int>(m_value)};
    auto denom{1};
    auto sub{0};
    std::vector<int> digits(m_places, 0);
    for (size_t i{0}; i < digits.size(); ++i)
    {
        auto m{denom * 10};
        auto place{(n % m) / denom};
        digits.rbegin()[i] = place;
        denom = m;
        sub += place;
    }

    Reset_Transform resetter{reset};
    mp_digits->activate();

    auto nonzero{false};
    auto const N{digits.size()};
    auto const& pos{m_position};
    for (size_t i{0}; i < N; ++i)
    {
        auto n{digits[i]};
        if (!nonzero && n == 0 && i != N - 1)
            continue;
        nonzero = true;

        auto tex_x1{n * 0.1};
        auto tex_x2{tex_x1 + 0.1};
        auto x1{i * m_size.x / N};
        auto x2{x1 + m_size.x / N};

        glColor3d(1.0, 1.0, 1.0);
        glBegin(GL_QUADS);
        glNormal3f(-1.0, 0.0, 0.0);
        glTexCoord2d(tex_x1, 1.0);
        glVertex3d(pos.x + x1, pos.y, pos.z);
        glTexCoord2d(tex_x2, 1.0);
        glVertex3d(pos.x + x2, pos.y, pos.z);
        glTexCoord2d(tex_x2, 0.0);
        glVertex3d(pos.x + x2, pos.y + m_size.y, pos.z);
        glTexCoord2d(tex_x1, 0.0);
        glVertex3d(pos.x + x1, pos.y + m_size.y, pos.z);
        glEnd();
    }
}

//----------------------------------------------------------------------------------------
Gear_Indicator::Gear_Indicator(Three_Vector const& position, Point<double>size,
                               int numbers, std::string const& image, bool on_wheel)
    : Gauge{on_wheel},
      m_position{position},
      m_size{size},
      m_number_width{1.0 / numbers},
      mp_numbers{std::make_unique<Texture_Image>(image, true, true)}
{
}

void Gear_Indicator::draw(bool reset) const
{
    Reset_Transform resetter{reset};
    mp_numbers->activate();

    auto x1{m_number_width * (m_value + 1)};
    auto x2{x1 + m_number_width};
    auto const& pos{m_position};
    glColor3d(1.0, 1.0, 1.0);
    glBegin(GL_QUADS);
    glNormal3f(0.0, 0.0, 1.0);
    glTexCoord2d(x1, 1.0);
    glVertex3d(pos.x, pos.y, pos.z);
    glTexCoord2d(x2, 1.0);
    glVertex3d(pos.x + m_size.x, pos.y, pos.z);
    glTexCoord2d(x2, 0.0);
    glVertex3d(pos.x + m_size.x, pos.y + m_size.y, pos.z);
    glTexCoord2d(x1, 0.0);
    glVertex3d(pos.x, pos.y + m_size.y, pos.z);
    glEnd();
}

//----------------------------------------------------------------------------------------
Gear_Shift::Gear_Shift(Vamos_Geometry::Rectangle<double> const& size, double z,
                       Three_Vector const& rotation,
                       std::vector<Two_Vector> const& positions,
                       std::string const& plate_image, std::string const& stick_image)
    : m_rotation{rotation},
      m_positions{positions},
      m_top_gear{m_positions.size() - 2},
      mp_gate{std::make_unique<Texture_Image>(plate_image, true, true)},
      mp_stick{std::make_unique<Texture_Image>(stick_image, true, true)},
      m_list_index{glGenLists(2)}
{
    // m_stick_width = size.x * mp_stick->width_pixels() / mp_gate->width_pixels();
    // m_stick_height = size.y * mp_stick->height_pixels() / mp_gate->height_pixels();

    // glNewList(m_list_index, GL_COMPILE);

    // mp_gate->activate();

    // glRotated(rotation.x, 0.0, -1.0, 0.0);
    // glRotated(rotation.y, 0.0, 0.0, 1.0);
    // glRotated(rotation.z, 1.0, 0.0, 0.0);
    // glTranslated(-m_above, -size.left(), size.bottom());

    // glColor3d(1.0, 1.0, 1.0);
    // glBegin(GL_QUADS);
    // glTexCoord2d(0.0, 0.0);
    // glNormal3f(0.0, 0.0, 1.0);
    // glVertex3d(0.0, 0.0, 0.0);
    // glTexCoord2d(1.0, 0.0);
    // glVertex3d(size.x, 0.0, 0.0);
    // glTexCoord2d(1.0, 1.0);
    // glVertex3d(size.x, size.y, 0.0);
    // glTexCoord2d(0.0, 1.0);
    // glVertex3d(0.0, size.y, 0.0);
    // glEnd();

    // glTranslated(0.0, (-size.x + m_stick_width) / 2.0, size.y / 2.0);
    // glEndList();

    // glNewList(m_list_index + 1, GL_COMPILE);

    // mp_stick->activate();

    // glRotated(-rotation.x, 0.0, -1.0, 0.0);
    // glRotated(-rotation.y, 0.0, 0.0, 1.0);
    // glRotated(-rotation.z, 1.0, 0.0, 0.0);

    // glColor3d(1.0, 1.0, 1.0);
    // glBegin(GL_QUADS);
    // glTexCoord2d(0.0, 1.0);
    // glNormal3f(-1.0, 0.0, 0.0);
    // glVertex3d(0.0, 0.0, 0.0);
    // glTexCoord2d(1.0, 1.0);
    // glVertex3d(0.0, -m_stick_width, 0.0);
    // glTexCoord2d(1.0, 0.0);
    // glVertex3d(0.0, -m_stick_width, m_stick_height);
    // glTexCoord2d(0.0, 0.0);
    // glVertex3d(0.0, 0.0, m_stick_height);
    // glEnd();

    // glEndList();
}

void Gear_Shift::draw() const
{
    // auto gear{std::min(m_top_gear, m_value)};

    // Reset_Transform resetter{reset};
    // glCallList(m_list_index);
    // glTranslated(0.0, -m_positions[gear + 1].x, m_positions[gear + 1].y);
    // glCallList(m_list_index + 1);
}

//----------------------------------------------------------------------------------------
Dashboard::Dashboard(Three_Vector const& position, double tilt)
    : m_position(position),
      m_tilt{tilt}
{
}

void Dashboard::add_tachometer(std::unique_ptr<Gauge<double>> tachometer)
{
    mp_tachometer = std::move(tachometer);
}

void Dashboard::add_speedometer(std::unique_ptr<Gauge<double>> speedometer)
{
    mp_speedometer = std::move(speedometer);
}

void Dashboard::add_fuel_gauge(std::unique_ptr<Gauge<double>> fuel_gauge)
{
    mp_fuel_gauge = std::move(fuel_gauge);
}

void Dashboard::add_gear_indicator(std::unique_ptr<Gear_Indicator> gear_indicator)
{
    mp_gear_indicator = std::move(gear_indicator);
}

void Dashboard::add_steering_wheel(std::unique_ptr<Dial> steering_wheel)
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
    glPushMatrix();
    glTranslated(m_position.x, m_position.y, m_position.z);
    // Car coordinates are x forward, y left, z up. For convenient placement,
    // interior coordinates are x right, y up, z to the rear, toward the driver.
    glRotatef(-90.0, 0.0, 0.0, 1.0);
    glRotatef(90.0, 1.0, 0.0, 0.0);
    for (auto const& facade : m_facades)
        facade->draw();

    glRotated(-m_tilt, 1.0, 0.0, 0.0);
    if (mp_tachometer && !mp_tachometer->on_steering_wheel())
        mp_tachometer->draw();
    if (mp_speedometer && !mp_speedometer->on_steering_wheel())
        mp_speedometer->draw();
    if (mp_fuel_gauge && !mp_fuel_gauge->on_steering_wheel())
        mp_fuel_gauge->draw();
    if (mp_gear_indicator && !mp_gear_indicator->on_steering_wheel())
        mp_gear_indicator->draw();
    if (mp_steering_wheel)
        mp_steering_wheel->draw(false);

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
    glPopMatrix();
}
