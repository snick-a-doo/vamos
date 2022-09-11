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

#ifndef VAMOS_BODY_CAR_H_INCLUDED
#define VAMOS_BODY_CAR_H_INCLUDED

#include "drivetrain.h"
#include "rigid-body.h"

#include "../geometry/contact-info.h"
#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"
#include "../media/xml-parser.h"

#include <memory>
#include <utility>
#include <vector>

class slSample;
namespace Vamos_Media
{
class Facade;
class Sample;
} // namespace Vamos_Media

namespace Vamos_Body
{
/// A class that handles gradual application of a control that's operated by a button,
/// such as the clutch.  If you're using a keyboard instead of a joystick, it also handles
/// steering, gas and brake.
class Key_Control
{
public:
    /// @param block Wait until hitting target before responding to the next setting.
    Key_Control(bool block = false);

    // Set the target setting of this control.
    /// @param target The desired setting.
    /// @param time How long it should take to ramp setting to go from 0.0 to 1.0.
    /// @param delay How long to wait before ramping.
    void target(double target, double time = 0.0, double delay = 0.0);
    // Update the setting of the control.
    double update(double time);
    // @return the current value of this control.
    double value() const { return m_value; }
    // Go immediately to the target.
    void end();

private:
    bool m_block{false}; ///< True if the next target is not set until the current one is reached.
    bool m_target_pending{false}; ///< True if a target is waiting.
    double m_value{0.0}; ///< The current setting.
    double m_target{0.0}; ///< The desired setting of this control.
    double m_rate{0.0}; ///< How fast the value approaches the target. 0.0 means immediate.
    double m_delay{0.0};    // How long to wait before starting to change the setting.
    double m_time{0.0};    // The time needed for the control to reach its target.

    double m_next_target{0.0};
    double m_next_rate{0.0};
    double m_next_delay{0.0};
    double m_next_time{0.0};
};

class Dashboard;
class Fuel_Tank;
class Particle;
class Wheel;

/// A body with wheels.
class Car
{
    friend class Car_Reader;
    struct Robot_Parameters;

public:
    Car(Vamos_Geometry::Three_Vector const& position,
        Vamos_Geometry::Three_Matrix const& orientation);
    virtual ~Car();

    Rigid_Body& chassis() { return m_chassis; }
    const Rigid_Body& chassis() const { return m_chassis; }

    /// Read the car definition file.
    virtual void read(std::string data_dir = "", std::string car_file = "");

    /// Define a sound for the engine.
    virtual void engine_sound(std::string const&, // file
                              double,             // volume,
                              double,             // throttle_volume_factor,
                              double,             // engine_speed_volume_factor,
                              double)             // pitch
        {};

    /// Set and get the parameters for computer control.
    void set_robot_parameters(double slip_ratio, double deceleration, double lateral_acceleration);

    /// Change the performance parameters by the given fraction of the current
    /// value, i.e. param <- param + factor*param.
    void adjust_robot_parameters(double slip_ratio_factor, double deceleration_factor,
                                 double lateral_acceleration_factor);

    const Robot_Parameters& get_robot_parameters() const { return m_robot_parameters; }

    /// Set the 3D models.
    virtual void exterior_model(std::string const&,                  // file
                                double,                              // scale
                                Vamos_Geometry::Three_Vector const&, // translation
                                Vamos_Geometry::Three_Vector const&) // rotation
    {
    }
    virtual void interior_model(std::string const&,                  // file
                                double,                              // scale
                                Vamos_Geometry::Three_Vector const&, // translation
                                Vamos_Geometry::Three_Vector const&) // rotation
    {
    }

    /// Set the dashboard.
    virtual void dashboard(Dashboard* /* dash */) {}

    virtual void propagate(double time);
    virtual void set_paused(bool){};

    /// Pan the view.
    void pan(double factor, double time = 0.0);
    /// Change the steering angle.
    /// @param direct Ignore non-linearity if true.
    void steer(double angle, double time = 0.0, bool direct = false);
    /// @return The current steering angle.
    double steer_angle() const { return m_steer_key_control.value(); }
    /// Set the throttle.
    void gas(double factor, double time = 0.0);
    /// Set the brakes.
    void brake(double factor, double time = 0.0);
    /// Shift to the next lower gear if possible.
    /// @return The new gear.
    int shift_down();
    /// Shift to the next higher gear if possible.
    /// @return The new gear.
    int shift_up();
    /// Shift directly to a gear.
    /// @return The new gear.
    int shift(int gear);
    /// Set the clutch.
    void clutch(double factor, double time = 0.0);
    /// Fully engage the clutch.
    void engage_clutch(double time);
    /// Fully disengage the clutch.
    void disengage_clutch(double time);
    /// Start the engine.
    void start_engine();

    /// @return The brake setting as a fraction of maximum.
    double brake_fraction() const { return m_brake_key_control.value(); }
    /// @return The throttle setting as a fraction of maximum.
    double throttle_fraction() const { return m_gas_key_control.value(); }

    /// Set the largest possible steering angle.
    void max_steer_angle(double degree_angle) { m_max_steer_angle = degree_angle; }
    /// Set the steering non-linearity.
    void steer_exponent(double exponent) { m_steer_exponent = exponent; }
    /// Set the amount of decrease in sensitivity with speed.
    void steer_speed_sensitivity(double sensitivity) { m_steer_speed_sensitivity = sensitivity; }
    /// Set the amount of time for gear changes.
    void shift_delay(double time) { m_shift_delay = time; }
    /// Set the amount of time for operating the clutch. The first time is for shifting
    /// from nuetral.
    void clutch_time(double from_neutral, double others);
    /// @return The average of the degree of sliding of the tires.
    double slide() const { return m_slide; }
    /// @return A wheel according to index.
    Wheel& wheel(size_t wheel_index) const;
    /// @return A pointer to the drivetrain which can be used to access the engine,
    /// clutch, and transmission. Nullptr if a drivetrain is not present.
    Drivetrain* drivetrain() { return mp_drivetrain.get(); }
    /// @return a pointer to the fuel tank.
    Fuel_Tank* fuel_tank() { return mp_fuel_tank.get(); }
    /// Return the most recent gear selected.  The shift may not have
    /// occurred yet due to the shift delay specified in the call to
    /// `shift_up ()', `shift_down ()', or `shift ()'.
    int gear() const { return m_new_gear; }

    // Retrun the previous gear selected.
    int last_gear() const { return m_last_gear; }

    /// Restore the initial conditions.
    void reset();
    /// Restore the initial conditions and then set position and orientation.
    void reset(Vamos_Geometry::Three_Vector const& position,
               Vamos_Geometry::Three_Matrix const& orientation);
    /// @return the total distance traveled since the start of the simulation.
    double distance_traveled() const { return m_distance_traveled; }
    /// Take ownership of the pointer to the drivetrain.
    void set_drivetrain(std::unique_ptr<Drivetrain> drivetrain);

    virtual void set_view(Vamos_Geometry::Three_Vector const&, // position
                          double,                              // field_of_view
                          double,                              // near_plane
                          double,                              // far_plane
                          double)                              // pan_angle
    {
    }

    virtual void set_perspective(double /* aspect */){};
    /// Add a rearview mirror.
    virtual void add_rear_view(Vamos_Geometry::Three_Vector const&, // position
                               double,                              // width
                               double,                              // height
                               double,                              // direction
                               double,                              // field
                               double,                              // near_plane
                               double,                              // far_plane
                               std::string const&)                  // mask_file
    {
    }
    // Return the driver's field of view in degrees.
    double field_of_view() const { return m_field_of_view; }

    // Return the current pan angle.
    double pan() const { return m_pan_key_control.value(); }

    // Return the position of the viewpont.
    Vamos_Geometry::Three_Vector view_position(bool world, bool bob) const;

    // Render the car according to its current position and orientation.
    virtual void draw(){};
    virtual void draw_interior(){};
    virtual Vamos_Geometry::Three_Vector draw_rear_view(double /* aspect */, int /* index */);
    virtual void make_rear_view_mask(int /* window_width */, int /* window_height */) {}
    virtual void update_rear_view_mask(int /* window_width */, int /* window_height */) {}
    virtual int get_n_mirrors() const { return 0; }

    // Perform the transformations for the driver's view.
    virtual void view(double,                              // pan
                      Vamos_Geometry::Three_Vector const&) // view_position
    {}
    virtual void view() {}

    /// @return true if there is no shift delay.
    bool fast_shift() const { return m_shift_delay <= 0.0; }
    /// @return true if extra info should be shown on screen.
    void show_dashboard_extras(bool show) { m_show_dashboard_extras = show; }

    // Return the contact information for the given position and velocity.  If 'ignore_z'
    // is true, only consider the x- and y-values of 'position'.
    Vamos_Geometry::Contact_Info collision(const Vamos_Geometry::Three_Vector& position,
                                           const Vamos_Geometry::Three_Vector& velocity,
                                           bool ignore_z = false) const;

    struct Crash_Box
    {
        double front;
        double back;
        double left;
        double right;
        double top;
        double bottom;

        bool within(const Vamos_Geometry::Three_Vector& position, bool ignore_z) const;
        Vamos_Geometry::Three_Vector penetration(const Vamos_Geometry::Three_Vector& point,
                                                 const Vamos_Geometry::Three_Vector& velocity,
                                                 bool ignore_z) const;
    };

    // The vector from the car's origin to the center of the crash box.
    Vamos_Geometry::Three_Vector center() const
    {
        return Vamos_Geometry::Three_Vector((m_crash_box.front + m_crash_box.back) / 2.0,
                                            (m_crash_box.left + m_crash_box.right) / 2.0,
                                            (m_crash_box.top + m_crash_box.bottom) / 2.0);
    }

    Vamos_Geometry::Three_Vector front() const
    {
        return Vamos_Geometry::Three_Vector(m_crash_box.front,
                                            (m_crash_box.left + m_crash_box.right) / 2.0,
                                            (m_crash_box.top + m_crash_box.bottom) / 2.0);
    }

    // The center of the crash box in world coordinates.
    Vamos_Geometry::Three_Vector center_position() const
    {
        return m_chassis.transform_to_world(center());
    }

    double width() const { return m_crash_box.left - m_crash_box.right; }
    double length() const { return m_crash_box.front - m_crash_box.back; }

    void wind(const Vamos_Geometry::Three_Vector& wind_vector, double densty);

    /// The position of the chase-view camera.
    Vamos_Geometry::Three_Vector chase_position() const;

    /// The position of the font centerline in world coordinates.
    Vamos_Geometry::Three_Vector front_position() const;

    void set_front_position(const Vamos_Geometry::Three_Vector& pos);

    /// The position of a point ahead of the car.  Used for steering.
    Vamos_Geometry::Three_Vector target_position() const;
    /// How far the target is ahead of the center of the car.
    double target_distance() const;

    const std::string& car_file() const { return m_car_file; }
    const std::string& name() const { return m_name; }

    double grip() const;

    Vamos_Geometry::Three_Vector acceleration(bool smooth) const;

protected:
    Rigid_Body m_chassis;
    std::unique_ptr<Drivetrain> mp_drivetrain;
    // A pointer to the car's fuel tank.
    std::shared_ptr<Fuel_Tank> mp_fuel_tank;
    // The maximum angle for the steered wheels.
    double m_max_steer_angle{15};
    // Steering non-linearity
    double m_steer_exponent{1.0};
    // Set the amount of decrease in sensitivity with speed.
    double m_steer_speed_sensitivity{0.0};
    // The sum of the sliding speeds of the tires.
    double m_slide{0.0};
    // True if a shift has been requested but not yet made due to the
    // delay `m_shift_delay'.
    bool m_shift_pending{false};
    // The amount of time elapsed after the shift request.
    double m_shift_timer{0.0};
    // How long to wait between getting a shift request and actually
    // shifting.
    double m_shift_delay{0.2};
    // The gear to shift to when `m_shift_timer' reaches
    // `m_shift_delay'.
    int m_new_gear{0};
    // The gear we were in before the last shift was requested.
    int m_last_gear{0};
    // The control that gradually applies steering when the keyboard is used.
    Key_Control m_steer_key_control{false};
    // The control that gradually applies the throttle when the keyboard is used.
    Key_Control m_gas_key_control{false};
    // The control that gradually applies braking when the keyboard is used.
    Key_Control m_brake_key_control{false};
    // The control that gradually applies the clutch when the keyboard is used.
    Key_Control m_clutch_key_control{true};
    // The control that gradually pans the view.
    Key_Control m_pan_key_control{false};
    // The total distance traveled since the start of the simulation.
    double m_distance_traveled{0.0};
    std::vector<std::shared_ptr<Wheel>> m_wheels;
    // The position of the driver's eyes.
    Vamos_Geometry::Three_Vector m_driver_view;
    // The driver's field of view.
    double m_field_of_view{60.0};
    // The maximum pan angle.
    double m_pan_angle{90.0};
    // Display additional information if true.
    bool m_show_dashboard_extras{false};
    double m_air_density{0.0};

private:
    // Perform operations common to both reset() methods.
    void private_reset();

    std::string m_data_dir;
    std::string m_car_file;
    std::string m_name;
    Crash_Box m_crash_box;

    struct Robot_Parameters
    {
        double slip_ratio;
        double deceleration;
        double lateral_acceleration;
    };
    Robot_Parameters m_robot_parameters;

    Vamos_Geometry::Three_Vector m_smoothed_acceleration;
};

struct Model_Info
{
    std::string file;
    double scale;
    Vamos_Geometry::Three_Vector translate;
    Vamos_Geometry::Three_Vector rotate;
};

class Gauge;
class Gear_Indicator;
class Steering_Wheel;

class Car_Reader : public Vamos_Media::XML_Parser
{
public:
    Car_Reader(std::string const& data_dir, std::string const& car_file, Car* car);
    ~Car_Reader();

private:
    void on_start_tag(const Vamos_Media::XML_Tag& tag);
    void on_end_tag(const Vamos_Media::XML_Tag& tag);
    void on_data(std::string data_string);

    std::vector<int> m_ints;
    std::vector<double> m_doubles;
    std::vector<std::string> m_strings;
    std::vector<Vamos_Geometry::Three_Vector> m_vectors;
    std::vector<Vamos_Geometry::Two_Vector> m_points;
    std::vector<std::pair<int, double>> m_gears;
    std::vector<bool> m_bools;

    std::vector<double> m_long_parameters;
    std::vector<double> m_trans_parameters;
    std::vector<double> m_align_parameters;

    std::string m_slow_model;
    std::string m_fast_model;
    std::string m_stator_model;
    double m_transition;
    double m_stator_offset;
    double m_scale;
    Vamos_Geometry::Three_Vector m_translation;
    Vamos_Geometry::Three_Vector m_rotation;
    std::vector<Model_Info*> m_models;
    bool m_first_model_for_this_wheel{true};

    std::string m_data_dir;
    Car* mp_car;
    std::shared_ptr<Engine> mp_engine;
    std::unique_ptr<Clutch> mp_clutch;
    std::unique_ptr<Transmission> mp_transmission;
    std::unique_ptr<Differential> mp_differential;

    std::vector<Vamos_Media::Facade*> ma_mirrors;
    Gauge* mp_tachometer{nullptr};
    Gauge* mp_speedometer{nullptr};
    Gauge* mp_fuel_gauge{nullptr};
    Gear_Indicator* mp_gear_indicator{nullptr};
    Steering_Wheel* mp_steering_wheel{nullptr};

    std::string m_tachometer_type{"dial"};
    std::string m_speedometer_type{"dial"};
    std::string m_fuel_gauge_type{"dial"};
};
} // namespace Vamos_Body

#endif // VAMOS_BODY_CAR_H_INCLUDED
