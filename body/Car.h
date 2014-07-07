//  Car.h - a body with wheels.
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

#ifndef _CAR_H_
#define _CAR_H_

#include "../geometry/Contact_Info.h"
#include "../geometry/Three_Vector.h"
#include "../geometry/Two_Vector.h"
#include "../media/XML_Parser.h"
#include "Rigid_Body.h"
#include "Drivetrain.h"

#include <vector>
#include <utility>

class slSample;
namespace Vamos_Media
{
  class Facade;
  class Sample;
}

namespace Vamos_Body
{
  class Car_Reader;
  class Dashboard;
  class Fuel_Tank;
  class Particle;
  class Wheel;

  //* A class that handles gradual application of a control that's
  // operated by a button, such as the clutch.  If you're using a
  // keyboard instead of a joystick, it also handles steering, gas and
  // brake.
  class Key_Control
  {
    // True if the next target is not set until the current one is
    // reached. 
    bool m_block;

    // True if a target is waiting.
    bool m_target_pending;
    
    // The current setting of this control.
    double m_value;
    // The difference between the current and previous settings.
    double m_delta;

    // The desired setting of this control.
    double m_target;
    double m_next_target;

    // How fast `m_value' approaches `m_target'.
    double m_rate;
    double m_next_rate;

    // How long to wait before starting to change the setting.
    double m_delay;
    double m_next_delay;

    // The time needed for the control to reach its target.
    double m_time;
    double m_next_time;

  public:
    //** Constructor
    Key_Control (bool block = false);
    
    // Set the target setting of this control.  NEW_TARGET is the
    // desired setting.  TIME is how long it should take for the
    // setting to go from 0.0 to 1.0 after waiting for DELAY.
    // `m_rate' is calculated in this function.
    void target (double new_target, double time = 0.0, double delay = 0.0);

    // Update the setting of this control.  The setting move toward
    // `m_target' by the ammount `m_rate' * TIME.
    double update (double time);

    // Return the current value of this control.
    double value () const { return m_value; }

    // Return the difference between the current and previous settings.
    double delta () const { return m_delta; }

    // Go immediately to the target.
    void end ();
  };


  //* A body with wheels.
  class Car
  {
    friend class Car_Reader;

  private:
    struct Robot_Parameters
    {
      double slip_ratio;
      double deceleration;
      double lateral_acceleration;
    };

  public:
    //** Constructor
    Car (const Vamos_Geometry::Three_Vector& position,
         const Vamos_Geometry::Three_Matrix& orientation);

    //** Destructor
    virtual ~Car ();

    Rigid_Body& chassis () { return m_chassis; }
    const Rigid_Body& chassis () const { return m_chassis; }

    // Read the car definition file.
    virtual void read (std::string data_dir = "", std::string car_file = "");

    // Define a sound for the engine.
    virtual void engine_sound (std::string file, 
                               double volume, 
                               double throttle_volume_factor, 
                               double engine_speed_volume_factor, 
                               double pitch) {};

    // Set and get the parameters for computer control.
    void set_robot_parameters (double slip_ratio,
                               double deceleration,
                               double lateral_acceleration);

    // Change the performance parameters by the given fraction of the current
    // value. I.e. param <- param + factor*param.
    void adjust_robot_parameters (double slip_ratio_factor,
                                  double deceleration_factor,
                                  double lateral_acceleration_factor);

    const Robot_Parameters& get_robot_parameters () const
    { return m_robot_parameters; }

    // Set the 3D models.
    virtual void 
    exterior_model (std::string file, double scale,
                    const Vamos_Geometry::Three_Vector& translation,
                    const Vamos_Geometry::Three_Vector& rotation) {};
    virtual void 
    interior_model (std::string file, double scale,
                    const Vamos_Geometry::Three_Vector& translation,
                    const Vamos_Geometry::Three_Vector& rotation) {};

    // Set the dashboard.
    virtual void dashboard (Dashboard* dash) {};

    // Advance the car in time by TIME.
    virtual void propagate (double time);

    virtual void set_paused (bool) {};

    // Pan the view.
    void pan (double factor, double time = 0.0);

    // Change the steering angle to ANGLE with a time constant of TIME.
    void steer (double angle, double time = 0.0, bool direct = false);

    double steer_angle () const { return m_steer_key_control.value (); }

    // Change the throttle to FACTOR with a time constant of TIME.
    void gas (double factor, double time = 0.0);

    // Change the brakes to FACTOR with a time constant of TIME.
    void brake (double factor, double time = 0.0);

    // Shift to the next lower gear.  The chosen gear is returned.
    int shift_down ();

    // Shift to the next higher gear.  The chosen gear is returned.
    int shift_up ();

    // Shift to GEAR.  The chosen gear is returned.
    int shift (int gear);

    void clutch (double factor, double time = 0.0);

    void engage_clutch (double time);
    void disengage_clutch (double time);
    void start_engine ();
  
    // Return the current steer angle as a fraction of maximum.
    double steer_fraction () const
    { return m_steer_key_control.value () / m_max_steer_angle; }

    double brake_fraction () const
    { return m_brake_key_control.value (); }

    double throttle_fraction () const
    { return m_gas_key_control.value (); }

    // Set the largest possible steering angle.
    void max_steer_angle (double degree_angle) 
    { m_max_steer_angle = degree_angle; }

    // Set the steering non-linearity.
    void steer_exponent (double exponent) { m_steer_exponent = exponent; }
    
    // Set the amount of decrease in sensitivity with speed.
    void steer_speed_sensitivity (double sensitivity)
    { m_steer_speed_sensitivity = sensitivity; }

    // Set the amount of time for gear changes.
    void shift_delay (double time) { m_shift_delay = time; }

    // Set the amount of time for operating the clutch.  The first
    // time is for shifting from nuetral.
    void clutch_time (double from_neutral, double others);

    // Return the average of the degree of sliding of the tires.
    double slide () const { return m_slide; }

    // Return the pointer to the WHEEL_INDEXth wheel.
    Wheel* wheel (size_t wheel_index) const;

    // Return the pointer to the front-most particle.
    Particle* front_particle () const { return mp_front_particle; }

    // Return a pointer to the engine.
    Engine* engine () { return mp_drivetrain ? mp_drivetrain->engine () : 0; }

    // Return a pointer to the transmission.
    Transmission* transmission () { return mp_drivetrain->transmission (); }

    // Return a pointer to the transmission.
    Clutch* clutch () { return mp_drivetrain->clutch (); }

    // Return a pointer to the fuel tank.
    Fuel_Tank* fuel_tank () { return mp_fuel_tank; }

    // Return the most recent gear selected.  The shift may not have
    // occurred yet due to the shift delay specified in the call to
    // `shift_up ()', `shift_down ()', or `shift ()'.
    int gear () const { return m_new_gear; }

    // Retrun the previous gear selected.
    int last_gear () const { return m_last_gear; }

    //** Enhance Body's reset methods by re-initializing the engine
    // and gearbox.

    // Restore the initial conditions.
    void reset ();

    // Restore the initial conditions and then set the position to
    // POSITION and the orientation to ORIENTATION.
    void reset (const Vamos_Geometry::Three_Vector& position, 
                const Vamos_Geometry::Three_Matrix& orientation);

    // Return the total distance traveled since the start of the
    // simulation.
    double distance_traveled () const { return m_distance_traveled; }

    void drivetrain (Drivetrain* drive);

    virtual void set_view (const Vamos_Geometry::Three_Vector& position,
                           double field_of_view,
                           double near_plane, double far_plane,
                           double pan_angle) {};

    virtual void set_perspective (double aspect) {};

    // Add a rearview mirror.
    virtual void add_rear_view (const Vamos_Geometry::Three_Vector& position,
                                double width, double height,
                                double direction, double field,
                                double near_plane, double far_plane,
                                std::string mask_file) {};

    // Return the driver's field of view in degrees.
    double field_of_view () const { return m_field_of_view; }

    // Return the current pan angle.
    double pan () const { return m_pan_key_control.value (); }

    // Return the position of the viewpont.
    Vamos_Geometry::Three_Vector view_position (bool world, bool bob) const;

    //* Methods to be defined in derived classes.

    // Render the car according to its current position and
    // orientation.
    virtual void draw () {};
    virtual void draw_interior () {};
    virtual Vamos_Geometry::Three_Vector draw_rear_view (double aspect, int index)
    { return Vamos_Geometry::Three_Vector::ZERO; }
    virtual void make_rear_view_mask (int window_width, int window_height) {}
	virtual void update_rear_view_mask (int window_width, int window_height) {}
    virtual int get_n_mirrors () const { return 0; }

    // Perform the transformations for the driver's view.
    virtual void view (double pan, const Vamos_Geometry::Three_Vector& view_position) {}
    virtual void view () {}

    // Return true if there is no shift delay.
    bool fast_shift () const { return m_shift_delay <= 0.0; }

    void show_dashboard_extras (bool show) { m_show_dashboard_extras = show; }

    // Return the contact information for the given position and
    // velocity.  If 'ignore_z' is true, only consider the x- and
    // y-values of 'position'.
    Vamos_Geometry::Contact_Info 
    collision (const Vamos_Geometry::Three_Vector& position,
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

      bool within (const Vamos_Geometry::Three_Vector& position, bool ignore_z) const;
      Vamos_Geometry::Three_Vector 
      penetration (const Vamos_Geometry::Three_Vector& point,
                   const Vamos_Geometry::Three_Vector& velocity,
                   bool ignore_z) const;
    };

    // The vector from the car's origin to the center of the crash box.
    Vamos_Geometry::Three_Vector center () const
    {
      return Vamos_Geometry::Three_Vector ((m_crash_box.front + m_crash_box.back) / 2.0,
                                           (m_crash_box.left + m_crash_box.right) / 2.0,
                                           (m_crash_box.top + m_crash_box.bottom) / 2.0);
    }

    Vamos_Geometry::Three_Vector front () const
    {
      return Vamos_Geometry::Three_Vector (m_crash_box.front,
                                           (m_crash_box.left + m_crash_box.right) / 2.0,
                                           (m_crash_box.top + m_crash_box.bottom) / 2.0);
    }

    // The center of the crash box in world coordinates.
    Vamos_Geometry::Three_Vector center_position () const
    { return m_chassis.transform_to_world (center ()); }

    double width () const { return m_crash_box.left - m_crash_box.right; }
    double length () const { return m_crash_box.front - m_crash_box.back; }

    void wind (const Vamos_Geometry::Three_Vector& wind_vector, 
               double densty);

    /// The position of the chase-view camera.
    Vamos_Geometry::Three_Vector chase_position () const;

    /// The position of the font centerline in world coordinates.
    Vamos_Geometry::Three_Vector front_position () const;

    void set_front_position (const Vamos_Geometry::Three_Vector& pos);

    /// The position of a point ahead of the car.  Used for steering.
    Vamos_Geometry::Three_Vector target_position () const;
    /// How far the target is ahead of the center of the car.
    double target_distance () const;

    const std::string& car_file () const { return m_car_file; }
    const std::string& name () const { return m_name; }

    double grip () const;

    Vamos_Geometry::Three_Vector acceleration (bool smooth) const;

  protected:
    Rigid_Body m_chassis;

    // A pointer to the car's drivetrain.
    Drivetrain* mp_drivetrain;

    // A pointer to the car's fuel tank.
    Fuel_Tank* mp_fuel_tank;

    // The maximum angle for the steered wheels.
    double m_max_steer_angle;

    // Steering non-linearity
    double m_steer_exponent;

    // Set the amount of decrease in sensitivity with speed.
    double m_steer_speed_sensitivity;

    // The sum of the sliding speeds of the tires.
    double m_slide;

    // True if a shift has been requested but not yet made due to the
    // delay `m_shift_delay'.
    bool m_shift_pending;

    // The amount of time elapsed after the shift request.
    double m_shift_timer;

    // How long to wait between getting a shift request and actually
    // shifting. 
    double m_shift_delay;

    // The gear to shift to when `m_shift_timer' reaches
    // `m_shift_delay'. 
    int m_new_gear;

    // The gear we were in before the last shift was requested.
    int m_last_gear;

    // The control that gradually applies steering when the keyboard
    // is used.
    Key_Control m_steer_key_control;

    // The control that gradually applies the throttle when the
    // keyboard is used.
    Key_Control m_gas_key_control;

    // The control that gradually applies braking when the keyboard
    // is used.
    Key_Control m_brake_key_control;

    // The control that gradually applies the clutch when the keyboard
    // is used.
    Key_Control m_clutch_key_control;

    // The control that gradually pans the view.
    Key_Control m_pan_key_control;

    // A pointer to the frontmost particle of the car.
    Particle* mp_front_particle;

    // The total distance traveled since the start of the simulation.
    double m_distance_traveled;

    std::vector <Wheel*> m_wheels;

    // The position of the driver's eyes.
    Vamos_Geometry::Three_Vector m_driver_view;

    // The driver's field of view.
    double m_field_of_view;

    // The maximum pan angle.
    double m_pan_angle;

    // Display additional information if true.
    bool m_show_dashboard_extras;

    // Perform operations common to both reset() methods. 
    void private_reset ();

    double m_air_density;

  private:
    std::string m_data_dir;
    std::string m_car_file;
    std::string m_name;
    Crash_Box m_crash_box;

    Robot_Parameters m_robot_parameters;

    Vamos_Geometry::Three_Vector m_smoothed_acceleration;
  };

  class Gauge;
  class Gear_Indicator;
  class Steering_Wheel;

  struct Model_Info
  {
    std::string file;
    double scale;
    Vamos_Geometry::Three_Vector translate;
    Vamos_Geometry::Three_Vector rotate;

    Model_Info (std::string file_in, double scale_in,
                const Vamos_Geometry::Three_Vector& translate_in,
                const Vamos_Geometry::Three_Vector& rotate_in)
      : file (file_in), 
        scale (scale_in), 
        translate (translate_in),
        rotate (rotate_in)
    {};
  };

  class Car_Reader : public Vamos_Media::XML_Parser
  {
  public:
    Car_Reader (std::string data_dir, 
                std::string car_file, 
                Car* car);
    ~Car_Reader ();

  private:
    void on_start_tag (const Vamos_Media::XML_Tag& tag); 
    void on_end_tag (const Vamos_Media::XML_Tag& tag); 
    void on_data (std::string data_string);

    std::vector <int> m_ints;
    std::vector <double> m_doubles;
    std::vector <std::string> m_strings;
    std::vector <Vamos_Geometry::Three_Vector> m_vectors;
    std::vector <Vamos_Geometry::Two_Vector> m_points;
    std::vector <std::pair <int, double> > m_gears;
    std::vector <bool> m_bools;

    std::vector <double> m_long_parameters;
    std::vector <double> m_trans_parameters;
    std::vector <double> m_align_parameters;

    std::string m_slow_model;
    std::string m_fast_model;
    std::string m_stator_model;
    double m_transition;
    double m_stator_offset;
    double m_scale;
    Vamos_Geometry::Three_Vector m_translation;
    Vamos_Geometry::Three_Vector m_rotation;
    std::vector <Model_Info*> m_models;
    bool m_first_model_for_this_wheel;

    std::string m_data_dir;
    Car* mp_car;
    Engine* mp_engine;
    Clutch* mp_clutch;
    Transmission* mp_transmission;
    Differential* mp_differential;

    std::vector <Vamos_Media::Facade*> ma_mirrors;
    Gauge* mp_tachometer;
    Gauge* mp_speedometer;
    Gauge* mp_fuel_gauge;
    Gear_Indicator* mp_gear_indicator;
    Steering_Wheel* mp_steering_wheel;

    std::string m_tachometer_type;
    std::string m_speedometer_type;
    std::string m_fuel_gauge_type;
  };
}

#endif // not _CAR_H_
