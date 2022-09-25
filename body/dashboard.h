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
/// A rotating steering wheel image.
class Steering_Wheel : public Vamos_Media::Facade
{
public:
    Steering_Wheel(double center_x, double center_y,
                   double above, double radius,
                   double min_road, double min_wheel, double max_road, double max_wheel,
                   std::string const& image);
    /// Set the angle of the road wheels. The steering wheel angle is calculated.
    void set(double value);
    /// Render the steering wheel;
    void draw() const;

private:
    Vamos_Geometry::Two_Vector m_center; ///< The position of the center of the wheel.
    double m_above; ///< Distance between the wheel and the dashboard.
    double m_angle; ///< The steering wheel angle.
    /// A function that converts road wheel angle to steering wheel angel.
    std::function<double(double)> m_angle_fn;
};

//----------------------------------------------------------------------------------------
/// Abstract class for analog and digital gauges
class Gauge
{
public:
    virtual ~Gauge() = default;
    virtual void set(double value) = 0;
    virtual void draw() const = 0;
    bool on_steering_wheel() const { return m_on_steering_wheel; }

protected:
    bool m_on_steering_wheel{false};
};

//----------------------------------------------------------------------------------------
/// A round gauge with a needle.
class Dial : public Gauge
{
public:
    Dial(double center_x, double center_y, double above, double radius, double min,
         double min_angle, double max, double max_angle,
         std::string const& face_image, std::string const& needle_image);

    void set(double value);
    void draw() const;

protected:
    Vamos_Geometry::Two_Vector m_center;
    double m_angle{0.0};

private:
    double m_above;
    std::function<double(double)> m_angle_fn;

    std::unique_ptr<Vamos_Media::Facade> mp_face;
    std::unique_ptr<Vamos_Media::Facade> mp_needle;
};

//----------------------------------------------------------------------------------------
/// A row of lights.
class LED_Gauge : public Gauge
{
public:
    LED_Gauge(double x, double y, double above, double width, int elements, double min,
              double redline, std::string const& image, bool on_wheel);

    void set(double value);
    void draw() const;

private:
    double m_x;
    double m_y;
    double m_above;
    double m_width;
    double m_height;
    int m_elements;
    double m_min;
    double m_range;
    int m_leds_on{0};
    std::unique_ptr<Vamos_Media::Texture_Image> mp_leds;
    GLuint m_list_index;
};

//----------------------------------------------------------------------------------------
/// A gauge that shows images of digits.
class Digital_Gauge : public Gauge
{
public:
    Digital_Gauge(double x, double y, double above, double width, double heightn,
                  size_t places, std::string const& digits, bool on_wheel);

    void set(double value);
    void draw() const;

private:
    double m_x;
    double m_y;
    double m_above;
    double m_width;
    double m_height;
    size_t m_places;
    std::vector<int> m_digits;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_digits;
};

//----------------------------------------------------------------------------------------
/// A digital gear indicator
class Gear_Indicator
{
public:
    Gear_Indicator(Vamos_Geometry::Rectangle<double> const& size, double z, int numbers,
                   std::string const& image, bool on_wheel);
    virtual ~Gear_Indicator() = default;

    virtual void set(int gear) { m_gear = gear; }
    virtual void draw() const;
    bool on_steering_wheel() const { return m_on_steering_wheel; }

protected:
    Vamos_Geometry::Rectangle<double> m_size;
    double m_above;
    int m_gear;

private:
    bool m_on_steering_wheel;
    double m_number_width;
    std::unique_ptr<Vamos_Media::Texture_Image> mp_numbers;
};

//----------------------------------------------------------------------------------------
/// A shift lever rendered in the gate.
class Gear_Shift : public Gear_Indicator
{
public:
    Gear_Shift(Vamos_Geometry::Rectangle<double> const& size, double z,
               Vamos_Geometry::Three_Vector const& rotation,
               std::vector<Vamos_Geometry::Two_Vector> const& positions,
               std::string const& gate_image, std::string const& stick_image);

    void set(int gear);
    void draw() const;

private:
    double m_stick_width;
    double m_stick_height;
    Vamos_Geometry::Three_Vector m_rotation;
    std::vector<Vamos_Geometry::Two_Vector> m_positions;
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
    Dashboard(double x, double y, double z, double tilt);

    void add_tachometer(std::unique_ptr<Gauge> tachometer);
    void add_speedometer(std::unique_ptr<Gauge> speedometer);
    void add_fuel_gauge(std::unique_ptr<Gauge> fuel_gauge);
    void add_gear_indicator(std::unique_ptr<Gear_Indicator> gear_indicator);
    void add_steering_wheel(std::unique_ptr<Steering_Wheel> steering_wheel);
    void add_facade(std::unique_ptr<Vamos_Media::Facade> facade);

    void set_tachometer(double rpm);
    void set_speedometer(double speed);
    void set_fuel_gauge(double fuel);
    void set_gear_indicator(int gear);
    void set_steering_wheel(double angle);

    void draw() const;

private:
    double m_x;
    double m_y;
    double m_z;
    double m_tilt;

    std::unique_ptr<Gauge> mp_tachometer;
    std::unique_ptr<Gauge> mp_speedometer;
    std::unique_ptr<Gauge> mp_fuel_gauge;
    std::unique_ptr<Gear_Indicator> mp_gear_indicator;
    std::unique_ptr<Steering_Wheel> mp_steering_wheel;
    std::vector<std::unique_ptr<Vamos_Media::Facade>> m_facades;
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_DASHBOARD_H_INCLUDED
