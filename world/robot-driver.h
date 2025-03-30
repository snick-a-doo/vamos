//  Copyright (C) 2008-2022 Sam Varner
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

#ifndef VAMOS_WORLD_ROBOT_DRIVER_H_INCLUDED
#define VAMOS_WORLD_ROBOT_DRIVER_H_INCLUDED

#include "controls.h"
#include "driver.h"
#include "pid.h"

#include "../geometry/linear-interpolator.h"
#include "../geometry/numeric.h"
#include "../geometry/three-vector.h"
#include "../geometry/two-vector.h"

namespace Vamos_Body
{
class Car;
}

namespace Vamos_Track
{
class Strip_Track;
class Road;
class Gl_Road_Segment;
}

namespace Vamos_World
{
struct Car_Info;

/// The robot's interface to the track's racing line. Calculates maximum safe cornering
/// speed.
class Robot_Racing_Line
{
public:
    /// @param friction The coefficient of static friction.
    Robot_Racing_Line(Vamos_Track::Road const& road, double friction);

    /// Set the gravity acceleration vector for determining maximum cornering speed.
    void set_gravity(Vamos_Geometry::Three_Vector const& g) { m_gravity = g; }
    /// @return The maximum cornering speed.
    /// @param distance Distance from the start of the track.
    /// @param lane_shift How far off the racing line the car is.
    /// @param lift Current aerodynamic lift.
    /// @param n_hat The track normal unit vector at the car's position.
    /// @param mass The car's current mass.
    double maximum_speed(double distance, double lane_shift, double lift,
                         Vamos_Geometry::Three_Vector const& n_hat, double mass) const;
    /// Wrappers for track racing line functions.
    /// @{
    Vamos_Geometry::Three_Vector target(double along, double lead) const;
    Vamos_Geometry::Three_Vector curvature(double along, double lane_shift) const;
    Vamos_Geometry::Three_Vector tangent(double along) const;
    double left_width(double along) const;
    double right_width(double along) const;
    /// @}
    /// @return How far the racing line is from the center of the track.
    double from_center(double along, size_t segment) const;

private:
    Vamos_Track::Road const& m_road;
    double const m_friction; ///< Coefficient of static friction.
    Vamos_Geometry::Three_Vector m_gravity; ///< Acceleration due to gravity.
};

//----------------------------------------------------------------------------------------
/// A braking operation begins when the robot determines it needs to slow down to avoid
/// exceeding the maximum safe speed at some point down the road. It ends when the robot
/// passes that point. During the operation it brakes as necessary to maintain the speed
/// given by maximum_speed().
class Braking_Operation
{
public:
    /// @param track_length The total length of the track. This is needed to handle the
    /// case where the braking operation crosses the end of the track.
    Braking_Operation(Vamos_Track::Road const& road, double friction,
                      Robot_Racing_Line const& line);

    /// Set the gravity acceleration vector for determining maximum cornering speed.
    void set_gravity(Vamos_Geometry::Three_Vector const& g) { m_gravity = g; }
    /// Return the speed not to exceed to avoid exceeding the maximum cornering speed for
    /// an upcoming turn.
    /// @param speed The car's current speed.
    /// @param distance Distance from the start of the track.
    /// @param stretch Shift the speed vs. braking curve to avoid the car in front. !!Is
    /// that correct? is it good?
    /// @param lane_shift How far off the racing line the car is.
    /// @param drag Current aerodynamic drag.
    /// @param lift Current aerodynamic lift.
    /// @param mass The car's current mass.
    double maximum_speed(double speed, double distance, double stretch, double lane_shift,
                         double drag, double lift, double mass);
    /// Set the maximum fraction of braking to apply. Can be used to do gentler braking on
    /// the cooldown lap.
    void set_max_fraction(double fraction) { m_max_fraction = fraction; }

private:
    /// Change in speed over predefined distance increment.
    double delta_braking_speed(double speed, double cornering_speed, double along,
                               double lane_shift, const Vamos_Geometry::Three_Vector& normal,
                               double drag, double lift, double mass, double fraction) const;

    Vamos_Track::Road const& m_road;
    double m_friction; ///< Coefficient of static friction.
    Robot_Racing_Line const& m_line;
    Vamos_Geometry::Three_Vector m_gravity; ///< Gravitational acceleration vector.
    double m_start{0.0}; ///< Distance of the start of braking.
    double m_length{0.0}; ///< Distance from start to end of braking. Zero if not braking.
    double m_max_fraction{1.0}; ///< Maximum fraction of braking that can be applied.
    Vamos_Geometry::Linear_Interpolator m_speed_vs_distance; ///< Braking speed vs. distance.
};

//----------------------------------------------------------------------------------------
/// A computer-controlled driver
class Robot_Driver : public Driver
{
private:
    /// The kind of session.
    enum class Mode{qualify, race};
    /// What phase of the session we're in.
    enum class State{parked, idling, revving, driving, cool_down};

    /// A possibly delayed event that may cause a state change.
    struct Event
    {
        /// Add to the elapsed time.
        void propagate(double time_step) { time += time_step; }
        /// Zero elapsed time.
        void reset() { time = 0.0; }
        /// True if we've waited for the delay time.
        bool ready() const { return time >= delay; }

        enum Type{park, start_engine, rev, drive, done};
        Type type; ///< The kind of event.
        double delay{0.0}; ///< How long to wait before ready.
        double time{0.0}; ///< How long we've been waiting.
    };

public:
    /// Provide pointers to the robot's car and the track.
    Robot_Driver(std::shared_ptr<Vamos_Body::Car> car, Vamos_Track::Strip_Track const& track);
    virtual ~Robot_Driver() = default;

    /// Overridden Driver methods
    /// @{
    virtual void set_gravity(Vamos_Geometry::Three_Vector const& g) override;
    virtual void set_cars(const std::vector<Car_Info>* cars) override;
    virtual void start(double to_go) override;
    virtual void finish() override;
    virtual bool is_driving() const override;
    virtual void propagate(double timestep) override;
    virtual void draw() override;
    virtual bool is_interactive() const override { return false; }
    /// @}
    /// Use qualifying behavior.
    void qualify();
    /// Set whether or not the robot should take other cars into account.
    /// Useful for comparing performance with collisions turned off.
    void interact(bool do_interact) { m_interact = do_interact; }
    /// Step the driver forward in time by 'timestep' seconds.
    /// Turn drawing of the steering target on or off.
    void show_steering_target(bool show) { m_show_steering_target = show; }

private:
    /// High-level cor control
    /// @{
    void steer();
    void choose_gear();
    void accelerate();
    void avoid_collisions();
    /// @}

    /// Low-level cor control
    /// @{
    void set_gas(double gas);
    void set_brake(double brake);
    void set_speed(double speed);
    /// @}

    /// @return The world's car info for the controlled car.
    const Car_Info& info() const { return (*mp_cars)[m_info_index]; }
    /// Change the current event state.
    void set_event(Event::Type type, double delay = 0.0);
    /// Advance the state machine depending on the new event.
    void handle_event(Event::Type type);
    /// True if there's room to go on-track for qualifying.
    bool has_gap() const;
    /// @return Set the lane shift according to the current position.
    void set_lane_shift();
    /// @return The distance from the steering pointer to the center of the track.
    double get_offline_distance() const;
    /// @return the distance equivalent to n car lengths.
    double lengths(double n) const;

    /// @return a vector that points in the direction of the position of the point that
    /// the driver tries to keep on the racing line.
    Vamos_Geometry::Three_Vector pointer_vector() const;
    /// @return a vector that points in the direction of the position of the point on the
    /// racing line that the driver aims for.
    Vamos_Geometry::Three_Vector target_vector();
    /// @return A near-optimal slip ration for cornering.
    double target_slip_angle() const;
    /// @return The direction from one position to another. Direc::none if there's no
    /// overlap in the x- or y-direction.
    Direct relative_position(Vamos_Geometry::Three_Vector const& r1_track,
                             Vamos_Geometry::Three_Vector const& r2_track) const;
    /// @return The x- and y-distance between one car and another accounting for car width
    /// and length.
    Vamos_Geometry::Two_Vector find_gap(const Vamos_Geometry::Three_Vector& r1_track,
                                        const Vamos_Geometry::Three_Vector& r2_track) const;
    /// @return True if there may be an overtaking opportunity. Allow approaching a car
    /// ahead more closely if true to get more benefit from the slipstream.
    bool maybe_passable(double along, size_t segment) const;
    // @return Direc::left if we should try to pass on the left, Direc::right if we should
    // try to pass on the right, Direc::none if it's best not to try.
    Direct get_pass_side(double along, double delta_x, double delta_v, size_t segment) const;
    /// @return The side of the racing line that has more room, or Direc::none if neither
    /// side has enough room for a car.
    /// @param start The distance along the track to start checking.
    /// @param delta The spacing between checked points.
    /// @param n The number of points to check.
    Direct get_clear_side(double start, double delta, size_t n, size_t segment) const;
    /// @return The vector length of the largest longitudinal and transverse slip ratios.
    double total_slip() const;

    Vamos_Track::Road const& m_road;
    std::vector<Car_Info> const* mp_cars{nullptr}; ///< All cars in the session.
    size_t m_info_index{0}; ///< The index to this car's info.
    Robot_Racing_Line m_racing_line;
    Braking_Operation m_braking;

    Mode m_mode{Mode::race}; ///< The kind of session: qualifying or race.
    State m_state{State::parked}; ///< The current stage of the session.
    Event m_event{Event::park, 0.0}; ///< The last event that causes a state transition.

    /// PID controllers
    /// @{
    PID m_rev_control;
    PID m_speed_control;
    PID m_traction_control;
    PID m_brake_control;
    PID m_steer_control;
    PID m_front_gap_control;
    /// @}

    double m_target_slip; ///< Acceleration and braking parameter defined by the car.
    double m_speed{0.0}; ///< Current speed
    Vamos_Track::Gl_Road_Segment* mp_segment{nullptr}; ///< The track segment the car is on.
    double m_shift_time{0.0}; ///< Time since last shift.
    double m_timestep{1.0}; ///< The last timestep passed to propagate().
    double m_lane_shift{0.0}; ///< The fraction off the racing line.
    double m_lane_shift_timer{0.0}; ///< How long we've been off the racing line.
    bool m_interact{true}; ///< Avoid collisions if true.
    bool m_show_steering_target{false}; ///< Render the point on the racing line we aim for.
    double m_speed_factor{1.0}; ///< Modulation for speed while following.
    bool m_passing{false}; ///< True if attempting to pass.
};
} // namespace Vamos_World

#endif // VAMOS_WORLD_ROBOT_DRIVER_H_INCLUDED
