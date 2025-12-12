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

#include "robot-driver.h"

#include "world.h"

#include "../body/car.h"
#include "../body/clutch.h"
#include "../body/engine.h"
#include "../body/transmission.h"
#include "../body/wheel.h"
#include "../geometry/parameter.h"
#include "../geometry/three-vector.h"
#include "../media/two-d.h"
#include "../track/strip-track.h"

#include <algorithm>
#include <limits>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_Track;
using namespace Vamos_World;

/// Maximum allowed speed may be scaled. Make sure we don't exceed the exceed
/// the max double if we multiply the maximum speed by a small number.
auto constexpr unlimited_speed{0.01 * std::numeric_limits<double>::max()};

/// Decide whether to keep trying to pass or give up after this many seconds.
auto constexpr pass_time{2.0};

/// Take action if cars will make contact in this many seconds.
auto constexpr crash_time_limit{2.0};

/// Fade the clutch in for this many seconds.
constexpr auto clutch_time{2.0};

static double reaction_time()
{
    return Vamos_Geometry::random_in_range(0.1, 0.3);
}

//-----------------------------------------------------------------------------
Robot_Driver::Robot_Driver(std::shared_ptr<Car> car, Strip_Track const& track)
    : Driver{car},
      m_road(track.get_road(0)),
      m_racing_line{m_road, car->get_robot_parameters().lateral_acceleration},
      m_braking{m_road, car->get_robot_parameters().deceleration, m_racing_line},
      m_rev_control(0.01, 0.0, 0.0),
      m_speed_control(1.0, 0.0, 0.0),
      m_traction_control(0.5, 0.0, 0.0),
      m_brake_control(0.1, 0.0, 0.0),
      m_steer_control(0.5, 0.0, 0.0),
      m_front_gap_control(1.5, 0.0, 0.0),
      m_target_slip{car->get_robot_parameters().slip_ratio}
{
    m_traction_control.set(m_target_slip);
}

void Robot_Driver::set_gravity(Three_Vector const& g)
{
    m_braking.set_gravity(g);
    m_racing_line.set_gravity(g);
}

void Robot_Driver::set_cars(const std::vector<Car_Info>* cars)
{
    mp_cars = cars;
    m_info_index = cars->size();
}

void Robot_Driver::qualify()
{
    m_mode = Mode::qualify;
}

void Robot_Driver::set_event(Event::Type type, double delay)
{
    // Ignore successive events of the same type to avoid resetting the time.
    if (type != m_event.type)
        m_event = {type, delay, 0.0};
};

void Robot_Driver::start(double to_go)
{
    if (m_state == State::parked)
        set_event(Event::start_engine);
    else if (m_mode == Mode::qualify)
        set_event(Event::drive, Vamos_Geometry::random_in_range(10, 60));
    else if (to_go <= 0.0)
        set_event(Event::drive, m_interact ? reaction_time() : 0.0);
    else if (to_go <= 1.0)
        set_event(Event::rev);
}

void Robot_Driver::finish()
{
    set_event(Event::done);
}

void Robot_Driver::propagate(double timestep)
{
    // It's useful for other methods to know the size of the current timestep.
    m_timestep = timestep;
    m_event.propagate(timestep);
    if (m_event.ready())
        handle_event(m_event.type);
    m_speed = mp_car->chassis().cm_velocity().magnitude();
    if (!is_driving())
        return;

    mp_segment = m_road.segments()[info().segment_index].get();
    steer();
    choose_gear();
    accelerate();
    avoid_collisions(); // Do last since it may override steering and braking.
}

void Robot_Driver::handle_event(Event::Type type)
{
    switch (type)
    {
    case Event::park:
        set_brake(1.0);
        mp_car->disengage_clutch(0.0);
        mp_car->shift(0);
        mp_car->start_engine();
        set_gas(0.0);
        m_state = State::parked;
        break;
    case Event::start_engine:
        mp_car->disengage_clutch(0.0);
        mp_car->shift(0);
        mp_car->start_engine();
        set_gas(0.0);
        m_state = State::idling;
        break;
    case Event::rev:
        mp_car->disengage_clutch(0.0);
        mp_car->shift(1);
        set_gas(0.5);
        m_state = State::revving;
        break;
    case Event::drive:
        if (m_mode == Mode::qualify && !has_gap())
            m_event.reset();
        else if (m_state != State::driving)
        {
            if (m_mode != Mode::qualify)
                set_lane_shift();
            mp_car->shift(1);
            mp_car->engage_clutch(clutch_time);
            m_state = State::driving;
        }
        break;
    case Event::done:
        m_state = State::cool_down;
        m_traction_control.set(m_target_slip * 0.5);
        m_braking.set_max_fraction(0.5);
        break;
    default:
        assert(false);
        break;
    }
}

double Robot_Driver::lengths(double n) const
{
    return n * mp_car->length();
}

bool Robot_Driver::has_gap() const
{
    auto along{info().track_position().x};
    for (auto const& car : *mp_cars)
    {
        if (car.driver->is_driving())
        {
            auto delta{car.track_position().x - along};
            if (delta - m_road.length() > lengths(-100.0) || delta < lengths(25.0))
                return false;
        }
    }
    return true;
}

bool Robot_Driver::is_driving() const
{
    return m_state == State::driving || m_state == State::cool_down;
}

void Robot_Driver::set_lane_shift()
{
    auto track{info().track_position()};
    auto line_y{m_racing_line.from_center(track.x, info().segment_index)};
    auto offline{track.y - line_y};
    if (offline > 0.0)
    {
        auto left{m_road.racing_line().left_width(m_road, track.x)};
        m_lane_shift = std::min(offline / (left - line_y), 1.0);
    }
    else
    {
        auto right{m_road.racing_line().right_width(m_road, track.x)};
        m_lane_shift = std::max(offline / (right + line_y), -1.0);
    }
}

double Robot_Driver::get_offline_distance() const
{
    auto along{info().m_pointer_position.x};
    auto line_y{m_racing_line.from_center(along, info().segment_index)};
    if (m_lane_shift > 0.0)
    {
        auto left{m_road.racing_line().left_width(m_road, along)};
        return m_lane_shift * (left - line_y);
    }
    auto right{m_road.racing_line().right_width(m_road, along)};
    return m_lane_shift * (right + line_y);
}

void Robot_Driver::steer()
{
    // Steer to keep the car on the racing line. The target is ahead of the car as if
    // attached to a pole that sticks out in front of the car. Trying to steer to keep the
    // car's position on the racing line leads to instability. Using a target that's ahead
    // of the car is very stable.

    // Steer by an amount that's proportional to the cross product of the actual and
    // desired directions. For small angles steering is approximately proportional to the
    // angle between them. The steering wheel is set according to the output of a PID
    // controller (although only the P term seems to be necessary).
    auto angle{pointer_vector().cross(target_vector()).z};
    // 'angle' is |r_p||r_t|sin theta ~ r_t^2 theta.  The additional angle to the
    // off-line position is sin^-1 (r_o/r_t) ~ r_o/r_t.  After scaling by r_t^2 to
    // agree with 'angle' it's r_o r_t.
    m_steer_control.set(angle + get_offline_distance() * mp_car->target_distance());
    angle = m_steer_control.propagate(mp_car->steer_angle(), m_timestep);
    // Clip to a reasonable range. The last argument says to ignore non-linearity and
    // speed-sensitivity.
    auto max_angle{1.5 * target_slip_angle()};
    mp_car->steer(clip(angle, -max_angle, max_angle), 0.0, true);
}

Three_Vector Robot_Driver::pointer_vector() const
{
    // Return a vector that points in the direction of the point that the driver tries to
    // keep on the racing line.
    return mp_car->target_position() - mp_car->center_position();
}

Three_Vector Robot_Driver::target_vector()
{
    // Return a vector that points in the direction of the position of the point on the
    // track that the driver aims for.
    auto target_dist{3.5 * mp_car->length()};
    auto target{m_racing_line.target(info().track_position().x, target_dist)};
    auto center{mp_car->center_position()};
    return {target.x - center.x, target.y - center.y, 0.0};
}

double Robot_Driver::target_slip_angle() const
{
    //! Can we use a constant instead?
    return abs_max(mp_car->wheel(0).get_tire().peak_slip_angle(),
                   mp_car->wheel(1).get_tire().peak_slip_angle(),
                   mp_car->wheel(2).get_tire().peak_slip_angle(),
                   mp_car->wheel(3).get_tire().peak_slip_angle());
}

void Robot_Driver::choose_gear()
{
    auto& dt{*mp_car->drivetrain()};
    if (!dt.clutch().is_engaged())
        return;

    // Avoid shifting too frequently.
    m_shift_time += m_timestep;
    if (m_shift_time < 0.3)
        return;

    // The selected gear. Use Car::gear() rather than Transmission() because the shift to
    // the selected gear may be delayed.
    auto const gear{mp_car->gear()};

    // Find current the engine power and the power at maximum throttle in 1) the
    // current gear 2) the next gear up 3) 2 gears down. Two gears to prevent
    // downshifting to 1st and to prevent too much engine drag under braking.
    auto omega{dt.engine().rotational_speed()};
    auto up_omega{omega * dt.transmission().gear_ratio(gear + 1)
                  / dt.transmission().gear_ratio(gear)};
    auto down2_omega{omega * dt.transmission().gear_ratio(gear - 2)
                     / dt.transmission().gear_ratio(gear)};

    auto power{dt.engine().power(1.0, omega)};
    auto up_power{dt.engine().power(1.0, up_omega)};
    auto down2_power{dt.engine().power(1.0, down2_omega)};

    // Shift up if there's more power at the current revs in the next higher gear.
    if (up_power > power)
        mp_car->shift_up();
    else if (down2_power > power && gear > (m_state == State::cool_down ? 3 : 2))
        mp_car->shift_down();

    // Reset the timer if the gear actually changed.
    if (mp_car->gear() != gear)
        m_shift_time = 0.0;
}

void Robot_Driver::accelerate()
{
    // Try to achieve the maximum safe speed for this point on the track.

    // Save some values that show up often in this method. The normal vector calculation
    // requires distance along the segment instead of distance along the track.
    auto along_segment{info().track_position().x - mp_segment->start_distance()};
    // The 'true' argument causes kerbs to be ignored when calculating the normal for the
    // racing line speed.
    auto normal{mp_segment->normal(along_segment, info().track_position().y, false)};
    auto lift{mp_car->chassis().lift()};
    auto along{info().track_position().x};
    auto cornering_speed{
        m_racing_line.maximum_speed(along, m_lane_shift, lift, normal, mp_car->chassis().mass())};
    auto braking_speed{m_braking.maximum_speed(
            m_speed, along, m_speed_factor < 1.0 ? lengths(2.0) : 0.0, m_lane_shift,
            mp_car->chassis().drag(), lift, mp_car->chassis().mass())};
    set_speed(std::min(cornering_speed, braking_speed));
}

void Robot_Driver::set_speed(double target_speed)
{
    // Set the gas and brake pedal positions. Three PID controllers are involved in
    // setting the pedal position: 'm_speed_control' and 'm_brake_control' attempt to
    // reach the target speed, 'm_traction_control' attempts to limit wheel spin. Nothing
    // prevents the gas and brake from being applied simultaneously, but with only P terms
    // of the PID controllers being used it does not happen in practice.  Speed may be
    // modulated to avoid running into the back of the car in front.
    target_speed *= mp_car->grip() * m_speed_factor;

    auto const& dt{*mp_car->drivetrain()};

    m_speed_control.set(target_speed);
    auto gas{m_speed_control.propagate(m_speed, m_timestep)};
    gas = std::min(gas, m_traction_control.propagate(total_slip(), m_timestep));

    if (!dt.clutch().is_engaged())
    {
        // Keep the revs in check if the clutch is not fully engaged.
        m_rev_control.set(dt.engine().peak_engine_speed());
        gas = std:: min(gas,
                        m_rev_control.propagate(dt.engine().rotational_speed(), m_timestep));
    }

    if (m_state == State::cool_down)
        gas = std::min(gas, 0.5);
    set_gas(gas * m_speed_factor);

    m_brake_control.set(std::min(target_speed, m_speed));
    auto b1{-m_brake_control.propagate(m_speed, m_timestep)};
    auto b2{m_traction_control.propagate(total_slip(), m_timestep)};
    set_brake(std::min(b1, b2));
}

double Robot_Driver::total_slip() const
{
    std::array slip{mp_car->wheel(0).get_tire().slip(),
                    mp_car->wheel(1).get_tire().slip(),
                    mp_car->wheel(2).get_tire().slip(),
                    mp_car->wheel(3).get_tire().slip()};
    return Two_Vector{abs_max(slip[0].x, slip[1].x, slip[2].x, slip[3].x),
                      abs_max(slip[0].y, slip[1].y, slip[2].y, slip[3].y)}.magnitude();
}

void Robot_Driver::set_gas(double gas)
{
    if (gas <= 0.0)
    {
        // Erase the controllers' history.
        m_speed_control.reset();
        m_traction_control.reset();
    }
    mp_car->gas(clip(gas, 0.0, 1.0), 0.5);
}

void Robot_Driver::set_brake(double brake)
{
    if (brake <= 0.0)
    {
        // Erase the controllers' history.
        m_brake_control.reset();
        m_traction_control.reset();
    }
    mp_car->brake(clip(brake, 0.0, 1.0), 0.2);
}

void Robot_Driver::avoid_collisions()
{
    if (!mp_cars)
        return;

    // Loop through the other cars. Each time through the loop we potentially update all
    // of these variables.
    auto min_forward_gap{100.0};
    auto min_left_gap{100.0};
    auto min_right_gap{100.0};
    auto along{info().track_position().x};
    auto follow_lengths{m_state == State::cool_down ? 3.0
                        : maybe_passable(along, info().segment_index) ? 0.5
                        : 1.5};
    auto crash_time{2.0 * crash_time_limit};
    auto clear_side{Direct::none};
    auto v1{mp_car->chassis().rotate_in(mp_car->chassis().cm_velocity())};

    for (auto const& car : *mp_cars)
    {
        // Break out immediately if the interact flag is false. We still need the lane
        // shift code after the loop so the cars will get to the racing line after the
        // start.
        if (!m_interact)
            break;

        // Don't check for collisions with yourself.
        if (car.car.get() == mp_car.get())
            continue;

        // Don't worry about cars that are far away.
        if (std::abs(car.track_position().x - along) > lengths(10))
            continue;

        auto v2{car.car->chassis().rotate_in(car.car->chassis().cm_velocity())};

        auto delta_v{v2 - v1};
        auto gap{find_gap(info().m_pointer_position, car.m_pointer_position)};
        switch (relative_position(info().track_position(), car.track_position()))
        {
        case Direct::forward:
            if (gap.x < min_forward_gap)
            {
                min_forward_gap = gap.x;
                crash_time = -gap.x / delta_v.x;
                if (min_forward_gap < 0.0 || crash_time < 0.0)
                    m_passing = false;
                else if (m_passing)
                    follow_lengths = -gap.y / mp_car->length();
                clear_side = get_pass_side(along, gap.x, -delta_v.x, info().segment_index);
            }
            break;
        case Direct::left:
            if (gap.y < min_left_gap)
            {
                min_left_gap = gap.y;
                crash_time = -gap.y / delta_v.y;
            }
            break;
        case Direct::right:
            if (gap.y < min_right_gap)
            {
                min_right_gap = gap.y;
                crash_time = gap.y / delta_v.y;
            }
            break;
        default:
            break;
        }
    }

    auto const shift_step{0.4 * m_timestep};

    // Keep track of how line we've been off the racing line.
    if (m_passing || m_lane_shift != 0.0)
        m_lane_shift_timer += m_timestep;

    if (min_right_gap < 0.5 * mp_car->width() && min_left_gap > min_right_gap)
    {
        // Move to the left if the car on the right is too close.
        m_lane_shift = std::min(1.0, m_lane_shift + shift_step);
        m_lane_shift_timer = 0.0;
    }
    else if (min_left_gap < 0.5 * mp_car->width())
    {
        // Move to the right if the car on the left is too close.
        m_lane_shift = std::max(-1.0, m_lane_shift - shift_step);
        m_lane_shift_timer = 0.0;
    }
    else if (m_lane_shift_timer > pass_time)
    {
        // No danger of collision.  Get back to the racing line.
        m_passing = false;
        if (m_lane_shift > 0.0 && min_right_gap > 0.5 * mp_car->width())
            m_lane_shift = std::max(0.0, m_lane_shift - 0.5 * shift_step);
        else if (m_lane_shift < 0.0 && min_left_gap > 0.5 * mp_car->width())
            m_lane_shift = std::min(0.0, m_lane_shift + 0.5 * shift_step);
    }
    else if (m_lane_shift > 0.2 && std::abs(get_offline_distance()) < 0.2 * mp_car->width())
        // Return to the racing line; we're already close.
        m_lane_shift_timer = 2.0 * pass_time;

    if (crash_time < crash_time_limit)
    {
        // Go offline to pass.
        switch (clear_side)
        {
        case Direct::left:
            m_passing = true;
            m_lane_shift = std::min(1.0, m_lane_shift + shift_step);
            m_lane_shift_timer = 0.0;
            break;
        case Direct::right:
            m_passing = true;
            m_lane_shift = std::max(-1.0, m_lane_shift - shift_step);
            m_lane_shift_timer = 0.0;
            break;
        default:
            break;
        }
    }

    // Calculate the speed factor here while we have access to the gap.
    m_front_gap_control.set(std::min(lengths(follow_lengths), 0.5 * m_speed));
    m_speed_factor = 1.0
        - clip(m_front_gap_control.propagate(min_forward_gap, m_timestep), 0.0, 1.0);
}

Direct Robot_Driver::relative_position(Three_Vector const& r1_track,
                                       Three_Vector const& r2_track) const
{
    auto const corner{mp_car->width() / mp_car->length()};
    auto delta{r2_track - r1_track};
    auto const slope{delta.y / delta.x};

    if ((delta.x > mp_car->length() && std::abs(delta.y) > mp_car->width())
        || delta.x < -mp_car->length())
        return Direct::none;
    if (std::abs(slope) < corner)
        return delta.x > 0 ? Direct::forward : Direct::none;
    return delta.y > 0 ? Direct::left : Direct::right;
}

Two_Vector Robot_Driver::find_gap(Three_Vector const& r1_track,
                                    Three_Vector const& r2_track) const
{
    return {m_road.distance(r2_track.x, r1_track.x) - mp_car->length(),
            std::abs(r2_track.y - r1_track.y) - mp_car->width()};
}

bool Robot_Driver::maybe_passable(double along, size_t segment) const
{
    return get_clear_side(along, 0.5 * m_speed, 10, segment) != Direct::none;
}

Direct Robot_Driver::get_pass_side(double along, double delta_x, double delta_v,
                                   size_t segment) const
{
    // Give up immediately if we're not closing at a significant rate. Returning here
    // avoids a possible divide-by-zero below.
    if (delta_v < 1e-6)
        return Direct::none;

    // We'll try to pass if the racing line stays on one side of the track far enough
    // ahead for us to pull alongside the opponent's car at our current rate of closing,
    // 'delta_v'.

    //!! Look ahead by time, not distance to compensate for closing under braking.
    //!! Account for deceleration?
    auto pass_distance{delta_x * m_speed / delta_v};
    if (pass_distance > lengths(100.0))
        return Direct::none;

    return get_clear_side(along + pass_distance / 2, pass_distance / 20, 10, segment);
}

Direct Robot_Driver::get_clear_side(double start, double delta, size_t n, size_t segment) const
{
    auto blocked_side_lr = [&](double along, size_t segment) {
        auto across{m_racing_line.from_center(along, segment)};
        return std::make_pair(across > m_racing_line.left_width(along) - mp_car->width(),
                              across < -m_racing_line.right_width(along) + mp_car->width());
    };
    //!! calls to from_center sometimes increment the segment excessively.

    auto blocked_lr{std::make_pair(false, false)};
    for (size_t i{1}; i <= n; ++i)
    {
        auto lr{blocked_side_lr(start + i * delta, segment)};
        blocked_lr.first |= lr.first;
        blocked_lr.second |= lr.second;
        if (blocked_lr.first && blocked_lr.second)
            return Direct::none;
    }
    // Return the non-blocked side or the side with more room if neither is blocked.
    return blocked_lr.first ? Direct::right
        : blocked_lr.second ? Direct::left
        : m_racing_line.from_center(start, segment) > 0.0 ? Direct::right
        : Direct::left;
}

void Robot_Driver::draw()
{
    // Optionally show the target and vector as green and red squares.
    if (!m_show_steering_target)
        return;

    glLoadIdentity();
    glPointSize(8.0);
    glBegin(GL_POINTS);

    // Draw where the car is currently pointed.
    auto pointer{mp_car->center_position() + pointer_vector()};
    glColor3d(0.0, 0.8, 0.0);
    glVertex3d(pointer.x, pointer.y, pointer.z);

    // Draw the point on the racing line.
    auto goal{mp_car->center_position() + target_vector()};
    glColor3d(8.0, 0.0, 0.0);
    glVertex3d(goal.x, goal.y, pointer.z);

    glEnd();
}

//-----------------------------------------------------------------------------
// The distance resolution of the braking speed calculation
auto constexpr delta_x{5.0};
// Braking is applied gradually.  It reaches its maximum in this many meters.
auto constexpr fade_in{20.0};

Braking_Operation::Braking_Operation(Road const& road, double friction,
                                     Robot_Racing_Line const& line)
    : m_road(road),
      m_friction(friction),
      m_line(line)
{
}

double Braking_Operation::delta_braking_speed(double speed, double cornering_speed,
                                              double along, double lane_shift,
                                              Three_Vector const& n_hat,
                                              double drag, double lift, double mass,
                                              double fraction) const
{
    auto decel = [&](Three_Vector const& K, double v2, Three_Vector const& p_hat) {
        auto mu{fraction * m_friction};
        return -m_gravity.dot(p_hat) - v2 * drag / mass
            - mu * (m_gravity.dot(n_hat) + v2 * (lift / mass + K.dot(n_hat)));
    };

    auto a{decel(m_line.curvature(along, lane_shift), speed * speed, m_line.tangent(along))};
    auto a_par{a * (1.0 - speed / cornering_speed)};
    return a_par * delta_x / speed;
}

double Braking_Operation::maximum_speed(double speed, double distance, double stretch,
                                        double lane_shift, double drag, double lift,
                                        double mass)
{
    auto check_braking = [&](double distance) {
        // Wrap to so the distance from start is positive if start is near the end of the
        // track and distance is across the start.
        auto from_start{wrap(distance - m_start, m_road.length())};
        if (from_start < m_length)
            return true;
        m_length = 0.0;
        return false;
    };

    distance += stretch;
    // If we're in the middle of a braking operation, get the speed from the speed
    // vs. braking curve.
    if (check_braking(distance))
    {
        if (distance < m_speed_vs_distance[0].x)
            distance += m_road.length();
        return m_speed_vs_distance.interpolate(distance);
    }

    auto get_normal = [&](double along) {
        auto const& segment{m_road.segment_at(along)};
        auto along_segment{along - segment.start_distance()};
        return segment.normal(along_segment, 0.0);
    };

    // Calculate the car's speed as a function of distance if braking started now. If the
    // projected speed exceeds the maximum cornering speed calculated from the racing line
    // then braking should start now. When this happens, find the minimum cornering speed
    // and calculate distance vs. speed backwards from there.
    Two_Vector min_speed_vs_distance(0.0, speed);

    // True if projected braking speed exceeds cornering speed anywhere.
    auto too_fast{false};
    // True if a minimum in the cornering speed was found in the distance range
    // where projected braking speed exceeds cornering speed.
    auto found_min{false};
    auto start_speed{speed};
    // Look up to 1000 m ahead.
    for (auto d{0.0}; d < 1000.0; d += delta_x)
    {
        auto along{wrap(distance + d, m_road.length())};
        auto normal{get_normal(along)};
        auto cornering_speed{m_line.maximum_speed(along, lane_shift, lift, normal, mass)};

        // Apply braking gradually.
        auto braking_fraction{std::min(d / fade_in, m_max_fraction)};
        auto dv{delta_braking_speed(speed, cornering_speed, along, lane_shift, normal, drag,
                                    lift, mass, braking_fraction)};
        speed -= dv;
        if (speed <= 0.0)
            break;
        if (speed >= cornering_speed)
        {
            // Already too fast, nothing we can do.
            if (d == 0.0)
                break;
            too_fast = true;
        }
        else if (too_fast)
        {
            // We've gone from a region where braking speed is higher than
            // cornering speed to one where it's lower.  Keep going in case
            // there's another curve up ahead that requires harder braking.
            found_min = true;
            too_fast = false;
        }

        if (too_fast && (cornering_speed < min_speed_vs_distance.y))
            min_speed_vs_distance = {d, cornering_speed};
    }

    // No need to start braking yet.
    if (!found_min)
        return unlimited_speed;

    // Build the speed vs. distance curve by working backwards from the minimum speed.
    // Start one interval beyond the end; end one interval before the beginning to ensure
    // that the interpolations are good slightly beyond the endpoints.
    too_fast = false;
    std::vector<Point<double>> points;
    speed = min_speed_vs_distance.y;
    for (auto d{min_speed_vs_distance.x}; d > -delta_x; d -= delta_x)
    {
        // Use un-wrapped distances so the interpolator's points are in order.
        points.emplace_back(distance + d, speed);

        auto along{wrap(distance + d, m_road.length())};
        auto normal{get_normal(along)};
        auto cornering_speed{m_line.maximum_speed(along, lane_shift, lift, normal, mass)};
        auto braking_fraction{std::min(d / fade_in, 1.0)};
        auto dv{delta_braking_speed(speed, cornering_speed, along, lane_shift, normal, drag,
                                    lift, mass, braking_fraction)};

        if (too_fast && (cornering_speed < min_speed_vs_distance.y))
            min_speed_vs_distance = {d, cornering_speed};

        if (speed >= cornering_speed)
        {
            if (!too_fast)
            {
                min_speed_vs_distance = {d, cornering_speed};
                // Found an earlier curve that requires a lower speed.
                too_fast = true;
            }
        }
        else if (too_fast)
        {
            // Found the new minimum.  Start over from there.
            d = min_speed_vs_distance.x;
            speed = min_speed_vs_distance.y;
            points.clear();
            points.push_back(min_speed_vs_distance + Two_Vector(distance + delta_x, 0.0));
            too_fast = false;
        }
        else
            speed += dv;
    }

    // The interpolator requires ascending x-values.  Reverse the points.
    m_speed_vs_distance.clear();
    std::reverse(points.begin(), points.end());
    m_speed_vs_distance.load(points);

    // Scale speed vs. distance so it matches the passed-in speed at the passed-in
    // distance. This is usually a small adjustment, but can be significant when curves
    // are closely spaced.
    auto delta_speed{start_speed - m_speed_vs_distance[0].y};
    for (size_t i{0}; i < m_speed_vs_distance.size(); ++i)
    {
        //!! Use the interpolator's scale() and shift(). Remove modifying subscript
        //!! operator.
        m_speed_vs_distance[i].x
            -= stretch * (m_speed_vs_distance.size() - i) / m_speed_vs_distance.size();

        auto fraction{(distance + min_speed_vs_distance.x - m_speed_vs_distance[i].x)
                      / (distance + min_speed_vs_distance.x - m_speed_vs_distance[0].x)};
        m_speed_vs_distance[i].y += fraction * delta_speed;
    }

    // Mark the start of a braking operation.
    m_start = distance;
    m_length = min_speed_vs_distance.x;
    // No need to restrict speed yet.  Next call will get it from the newly calculated
    // speed vs. distance curve.
    return unlimited_speed;
}

//-----------------------------------------------------------------------------
Robot_Racing_Line::Robot_Racing_Line(Road const& road, double friction)
    : m_road{road},
      m_friction{friction}
{
}

double Robot_Racing_Line::maximum_speed(double distance, double lane_shift, double lift,
                                        Three_Vector const& n_hat, double mass) const
{
    const auto curve{curvature(distance, lane_shift)};
    auto c{curve.magnitude()};
    auto mu{m_friction};
    auto r_hat{curve.unit()};
    Three_Vector r_perp{-r_hat.y, r_hat.x, 0.0};
    auto q_hat{rotate(n_hat, pi / 2.0 * r_perp.unit())};

    auto upper{-m_gravity.dot(q_hat + mu * n_hat)};
    auto lower{c * (r_hat.dot(q_hat) + mu * r_hat.dot(n_hat)) + mu * lift / mass};
    return lower > 1.0e-9 ? std::sqrt(upper / lower) : unlimited_speed;
}

Three_Vector Robot_Racing_Line::target(double along, double lead) const
{
    return m_road.racing_line().position(along + lead);
}

Three_Vector Robot_Racing_Line::curvature(double along, double lane_shift) const
{
    return m_road.racing_line().curvature(along, lane_shift);
}

Three_Vector Robot_Racing_Line::tangent(double along) const
{
    return m_road.racing_line().tangent(along);
}

double Robot_Racing_Line::left_width(double along) const
{
    return m_road.left_racing_line_width(along);
}

double Robot_Racing_Line::right_width(double along) const
{
    return m_road.right_racing_line_width(along);
}

double Robot_Racing_Line::from_center(double along, size_t segment) const
{
    return m_road.track_coordinates(target(along, 0.0), segment, true).y;
}
