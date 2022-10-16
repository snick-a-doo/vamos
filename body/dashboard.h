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

#ifndef VAMOS_BODY_DASHBOARD_H_INCLUDED
#define VAMOS_BODY_DASHBOARD_H_INCLUDED

#include "../geometry/rectangle.h"
#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"
#include "../media/texture-image.h"

#include <GL/gl.h>

#include <functional>
#include <memory>
#include <vector>

namespace Vamos_Body
{
/// Abstract class for analog and digital gauges
template <typename T> class Gauge
{
public:
    Gauge(bool on_wheel) : m_on_steering_wheel{on_wheel} {}
    virtual ~Gauge() = default;
    void set(T value) { m_value = value; }
    virtual void draw(bool reset = true) const = 0;
    bool on_steering_wheel() const { return m_on_steering_wheel; }

protected:
    T m_value{0};

private:
    bool m_on_steering_wheel{false};
};

//----------------------------------------------------------------------------------------
/// A round gauge with a needle.
class Dial : public Gauge<double>
{
public:
    Dial(Vamos_Geometry::Three_Vector const& position, double radius,
         Vamos_Geometry::Point<double> min, Vamos_Geometry::Point<double> max,
         std::string const& face_image, std::string const& needle_image);
    virtual ~Dial() = default;

    virtual void draw(bool reset) const override;

private:
    std::function<double(double)> m_angle_fn;
    std::unique_ptr<Vamos_Media::Facade> mp_face;
    std::unique_ptr<Vamos_Media::Facade> mp_needle;
};

//----------------------------------------------------------------------------------------
/// A row of lights.
class LED_Gauge : public Gauge<double>
{
public:
    LED_Gauge(Vamos_Geometry::Three_Vector const& position, double width, int elements,
              Vamos_Geometry::Point<double> range, std::string const& image, bool on_wheel);
    virtual ~LED_Gauge() = default;

    virtual void draw(bool reset = true) const override;

private:
    double m_above;
    double m_width;
    double m_height;
    int m_elements;
    double m_min;
    double m_range;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_leds;
    bool m_on_steering_wheel;
    GLuint m_list_index;
};

//----------------------------------------------------------------------------------------
/// A gauge that shows images of digits.
class Digital_Gauge : public Gauge<double>
{
public:
    Digital_Gauge(Vamos_Geometry::Three_Vector const& position,
                  Vamos_Geometry::Point<double> size,
                  int places, std::string const& digits, bool on_wheel);
    virtual ~Digital_Gauge() = default;

    virtual void draw(bool reset = true) const override;

private:
    Vamos_Geometry::Three_Vector m_position;
    Vamos_Geometry::Point<double> m_size;
    int m_places;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_digits;
};

//----------------------------------------------------------------------------------------
/// A digital gear indicator
class Gear_Indicator : public Gauge<int>
{
public:
    Gear_Indicator(Vamos_Geometry::Three_Vector const& position,
                   Vamos_Geometry::Point<double> size,
                   int numbers, std::string const& image, bool on_wheel);
    virtual ~Gear_Indicator() = default;

    virtual void draw(bool reset = true) const override;

private:
    Vamos_Geometry::Three_Vector m_position;
    Vamos_Geometry::Point<double> m_size;
    double m_number_width;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_numbers;
};

//----------------------------------------------------------------------------------------
/// A shift lever rendered in the gate.
class Gear_Shift : public Gauge<int>
{
public:
    Gear_Shift(Vamos_Geometry::Three_Vector const& position,
               Vamos_Geometry::Point<double> const& size,
               Vamos_Geometry::Three_Vector const& rotation,
               std::vector<Vamos_Geometry::Point<double>> const& positions,
               std::string const& gate_image,
               std::string const& stick_image);

    virtual void draw(bool reset = true) const override;

private:
    double m_stick_width;
    double m_stick_height;
    Vamos_Geometry::Three_Vector m_rotation;
    std::vector<Vamos_Geometry::Point<double>> m_positions;
    int m_top_gear;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_gate;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_stick;
    GLuint m_list_index;
};

//----------------------------------------------------------------------------------------
/// A collection of gauges and indicators.
class Dashboard
{
public:
    Dashboard(Vamos_Geometry::Three_Vector const& position, double tilt);

    void add_tachometer(std::unique_ptr<Gauge<double>> tachometer);
    void add_speedometer(std::unique_ptr<Gauge<double>> speedometer);
    void add_fuel_gauge(std::unique_ptr<Gauge<double>> fuel_gauge);
    void add_gear_indicator(std::unique_ptr<Gear_Indicator> gear_indicator);
    void add_gear_shift(std::unique_ptr<Gear_Shift> gear_shift);
    void add_steering_wheel(std::unique_ptr<Dial> steering_wheel);
    void add_facade(std::unique_ptr<Vamos_Media::Facade> facade);

    void set_tachometer(double rpm);
    void set_speedometer(double speed);
    void set_fuel_gauge(double fuel);
    void set_gear(int gear);
    void set_steering_wheel(double angle);

    void draw() const;

private:
    Vamos_Geometry::Three_Vector m_position;
    double m_tilt;

    std::unique_ptr<Gauge<double>> mp_tachometer;
    std::unique_ptr<Gauge<double>> mp_speedometer;
    std::unique_ptr<Gauge<double>> mp_fuel_gauge;
    std::unique_ptr<Gear_Indicator> mp_gear_indicator;
    std::unique_ptr<Gear_Shift> mp_gear_shift;
    std::unique_ptr<Dial> mp_steering_wheel;
    std::vector<std::unique_ptr<Vamos_Media::Facade>> m_facades;
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_DASHBOARD_H_INCLUDED
