//  Robot_Driver.h - a computer-controlled driver
//
//  Copyright (C) 2008 Sam Varner
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

#ifndef _ROBOT_DRIVER_H_
#define _ROBOT_DRIVER_H_

#include "controls.h"
#include "driver.h"
#include "../geometry/constants.h"
#include "../geometry/linear-interpolator.h"
#include "../geometry/numeric.h"
#include "../geometry/pid.h"
#include "../geometry/three-vector.h"

namespace Vamos_Body
{
  class Car;
}

namespace Vamos_Geometry
{
  struct Three_Vector;
}

namespace Vamos_Track
{
  class Strip_Track;
  class Road;
  class Gl_Road_Segment;
}

namespace Vamos_World
{
  struct Car_Information;

  /// The robot's interface to the track's racing line. 
  class Robot_Racing_Line
  {
  public:
    Robot_Racing_Line (const Vamos_Track::Road& road,
                       double lateral_acceleration,
                       double gravity);
    Vamos_Geometry::Three_Vector target (double along, double lead) const;
    double maximum_speed (double distance, 
                          double lane_shift,
                          double lift,
                          const Vamos_Geometry::Three_Vector& normal,
                          double mass) const;

    Vamos_Geometry::Three_Vector curvature (double along, double lane_shift) const;
    Vamos_Geometry::Three_Vector tangent (double along) const;
    double from_center (double along, size_t segment) const;

  private:
    const Vamos_Track::Road* mp_road;
    double m_lateral_acceleration;
    double m_gravity;
  };

  /// Information about a braking operation.  A braking operation begins when
  /// the robot determines it needs to slow down to avoid exceeding the maximum
  /// safe speed at some point down the road.  It ends when the robot passes
  /// that point.  During the operation it brakes as necessary to maintain the
  /// speed given by 'maximum_speed()'.
  class Braking_Operation
  {
  public:
    /// Create the braking object
    /// @param track_length The total length of the track.  This is needed to
    ///                     handle the case where the braking operation crosses
    ///                     the end of the track.  
    Braking_Operation (const Vamos_Track::Road& road,
                       double deceleration,
                       double gravity,
                       const Robot_Racing_Line& line);
    ~Braking_Operation ();

    /// True if a braking operation is in progress, i.e. 'start()' has been
    /// called but 'end()' has not.
    bool is_braking () const { return m_is_braking; }

    /// Start a braking operation.
    /// @param start The current position along the track.
    /// @param distance_to_end The distance to the end of the braking
    ///                        operation. 
    void start (double start, double distance_to_end);

    /// See if a braking operation is done.
    /// @param distance How far the car is from the start of the braking
    ///                 operation.
    /// @return True if the braking operation is over, false otherwise.
    bool check_done_braking (double distance);

    double maximum_speed (double speed,
                          double distance,
                          double stretch,
                          double lane_shift,
                          double drag,
                          double lift,
                          double mass);

    void scale (double factor);

  private:
    void end ();
    double distance_from_start (double distance) const;
    bool past_end (double distance) const;

    double delta_braking_speed (double speed,
                                double cornering_speed,
                                double along,
                                double lane_shift,
                                const Vamos_Geometry::Three_Vector& normal,
                                double drag, 
                                double lift, 
                                double mass,
                                double fraction) const;

    Vamos_Geometry::Three_Vector get_normal (double along) const;

    double deceleration (const Vamos_Geometry::Three_Vector& curvature,
                         double speed, 
                         double drag,
                         double lift,
                         const Vamos_Geometry::Three_Vector& normal,
                         const Vamos_Geometry::Three_Vector& tangent,
                         double mass,
                         double fraction) const;

    double m_start;
    double m_length;
    bool m_is_braking;
    const Vamos_Track::Road& m_road;
    double m_deceleration;

    Vamos_Geometry::Linear_Interpolator m_speed_vs_distance;
    double m_gravity;
    const Robot_Racing_Line& m_line;
  };

  /// A computer-controlled driver
  class Robot_Driver : public Driver
  {
  private:
    enum Mode
      {
        QUALIFY,
        RACE
      };

    enum State
      {
        PARKED,
        IDLING,
        REVVING,
        DRIVING,
        COOL_DOWN
      };

    struct Event
    {
      enum Type
        {
          PARK,
          START_ENGINE,
          REV,
          DRIVE,
          DONE,
          NO_EVENT
        };

      Event (Type e_type, double e_delay)
        : type (e_type),
          delay (e_delay),
          time (0.0)
      {}

      void propagate (double time_step) { time += time_step; }
      void reset () { time = 0.0; }
      bool ready () const { return time >= delay; }

      Type type;
      double delay;
      double time;
    };

  public:
    /// Provide pointers to the robot's car and the track.
    Robot_Driver (Vamos_Body::Car& car_in, 
                  Vamos_Track::Strip_Track& track_in,
                  double gravity);
    virtual ~Robot_Driver () {};

    void qualify ();

    /// Provide pointers to the other cars so the robot can pass or avoid
    /// collisions. 
    void set_cars (const std::vector <Car_Information>* cars);

    /// Called to signal the start of the race.
    virtual void start (double to_go);

    virtual void finish ();

    /// Set whether or not the robot should take other cars into account.
    /// Useful for comparing performance with collisions turned off. 
    void interact (bool do_interact) 
    { m_interact = do_interact; }

    /// Step the driver forward in time by 'timestep' seconds.
    virtual void propagate (double timestep);

    /// Turn drawing of the steering target on or off.
    void show_steering_target (bool show) { m_show_steering_target = show; }

    /// Draw the steering target if requested.
    virtual void draw ();

    virtual bool is_interactive () const { return false; }

    virtual bool is_driving () const;

  private:
    const Car_Information& info () const { return (*mp_cars)[m_info_index]; }

    void set_event (Event::Type type, double delay = 0);
    void handle_event ();
    bool has_gap () const;

    enum Track_Side
      {
        NO_SIDE = 0,
        LEFT_SIDE = (1 << 0),
        RIGHT_SIDE = (1 << 1)
      };

    void drive ();
    double get_lane_shift () const;
    double lane_shift (const Vamos_Geometry::Three_Vector& track) const;
    double get_offline_distance () const;
    double offline_distance (double along, double lane_shift) const; 
    /// Return the distance equivalent to n car lengths.
    double lengths (double n) const;

    void steer ();
    void choose_gear ();
    void accelerate ();
    void avoid_collisions ();

    /// Return a vector that points in the direction of the position of the
    /// point that the driver tries to keep on the racing line.
    Vamos_Geometry::Three_Vector pointer_vector () const;
    /// Return a vector that points in the direction of the position of the
    /// point on the racing line that the driver aims for.
    Vamos_Geometry::Three_Vector target_vector ();

    double target_slip_ratio () const;
    double target_slip_angle () const;

    Vamos_Geometry::Direction 
    relative_position (const Vamos_Geometry::Three_Vector& r1_track,
                       const Vamos_Geometry::Three_Vector& r2_track) const;

    Vamos_Geometry::Three_Vector 
    find_gap (const Vamos_Geometry::Three_Vector& r1_track,
              const Vamos_Geometry::Three_Vector& r2_track) const;
    bool maybe_passable (double along, size_t segment) const;
    Vamos_Geometry::Direction get_pass_side (double along, 
                                             double delta_x, 
                                             double delta_v,
                                             size_t segment) const;
    Vamos_Geometry::Direction pass_side (double start, double delta, size_t n,
                                         size_t segment) const;
    Track_Side get_block_side (double along, size_t segment) const;

    void set_steering (double angle);
    void set_gas (double gas);
    void set_brake (double brake);
    void set_speed (double speed);

    const Vamos_Track::Gl_Road_Segment& current_segment () const;
    const Vamos_Track::Gl_Road_Segment& next_curve (int index = -1) const;
    const Vamos_Track::Gl_Road_Segment& previous_curve (int index = -1) const;
    double longitudinal_slip () const;
    double transverse_slip () const;
    double total_slip () const;

    // Return the speed-dependent performance parameters.
    double deceleration () const;

    /// Return a random reaction time in seconds.
    static double reaction_time ();

    Mode m_mode;
    State m_state;
    Event m_event;

    const std::vector <Car_Information>* mp_cars;
    size_t m_info_index;

    Vamos_Geometry::PID m_speed_control;
    Vamos_Geometry::PID m_traction_control;
    Vamos_Geometry::PID m_brake_control;
    Vamos_Geometry::PID m_steer_control;
    Vamos_Geometry::PID m_front_gap_control;

    // Parameters from the car.
    double m_target_slip;

    double m_speed;
    Vamos_Track::Gl_Road_Segment* mp_segment;
    size_t m_target_segment;
    Vamos_Track::Strip_Track* mp_track;
    bool m_reset;
    double m_shift_time;
    double m_timestep;
    double m_lane_shift;
    double m_lane_shift_timer;
    bool m_interact;
    bool m_show_steering_target;

    const Vamos_Track::Road& m_road;
    Robot_Racing_Line m_racing_line;
    Braking_Operation m_braking;

    double m_speed_factor;
    bool m_passing;

    //! debugging
    mutable double m_gap [10];
  };
}

#endif // not _ROBOT_DRIVER_H_