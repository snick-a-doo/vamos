//  Robot_Driver.cc - a computer-controlled driver
//
//	Vamos Automotive Simulator
//  Copyright (C) 2008 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#include "Robot_Driver.h"
#include "../body/Car.h"
#include "../body/Wheel.h"
#include "../geometry/Calculations.h"
#include "../geometry/Constants.h"
#include "../geometry/Parameter.h"
#include "../geometry/Three_Vector.h"
#include "../media/Two_D.h"
#include "../track/Strip_Track.h"
#include "World.h"

#include <algorithm>
#include <limits>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Media;
using namespace Vamos_Track;
using namespace Vamos_World;

namespace
{
  // Maximum allowed speed may be scaled.  Make sure we don't exceed the exceed
  // the max double if we multiply the maximum speed by a small number.
  const double UNLIMITED_SPEED = 0.01 * std::numeric_limits <double>::max ();

  // Decide whether to keep trying to pass or give up after this many seconds.
  const double PASS_TIME = 2.0;

  // Take action if cars will make contact in this many seconds.
  const double CRASH_TIME_LIMIT = 2.0;
}

//-----------------------------------------------------------------------------
Robot_Driver::Robot_Driver (Car* car_in, 
                            Strip_Track* track_in,
                            double gravity)
: Driver (car_in),
  m_mode (RACE),
  m_state (PARKED),
  m_event (Event::PARK, 0.0),
  mp_cars (0),
  m_info_index (0),
  m_speed_control (4.0, 0.0, 0.0),
  m_traction_control (0.5, 0.0, 0.0),
  m_brake_control (0.1, 0.0, 0.0),
  m_steer_control (0.5, 0.0, 0.0),
  m_front_gap_control (1.5, 0.0, 1.5),
  m_target_slip (car_in->get_robot_parameters ().slip_ratio),
  m_speed (0.0),
  m_target_segment (0),
  mp_track (track_in),
  m_shift_time (0.0),
  m_timestep (1.0),
  m_lane_shift (0.0),
  m_lane_shift_timer (0.0),
  m_interact (true),
  m_show_steering_target (false),
  m_road (mp_track->get_road (0)),
  m_racing_line (m_road,
                 car_in->get_robot_parameters ().lateral_acceleration,
                 gravity),
  m_braking (m_road,
             car_in->get_robot_parameters ().deceleration,
             gravity,
             m_racing_line),
  m_speed_factor (1.0),
  m_passing (false)
{
  m_traction_control.set (m_target_slip);
  for (int i = 0; i < 10; i++)
    m_gap [i] = 0;
}

void Robot_Driver::set_cars (const std::vector <Car_Information>* cars)
{ 
  mp_cars = cars;
  m_info_index = cars->size () - 1;
}

double 
Robot_Driver::reaction_time ()
{ 
  return Vamos_Geometry::random_in_range (0.1, 0.3);
}

void Robot_Driver::qualify ()
{
  m_mode = QUALIFY;
}

void
Robot_Driver::start (double to_go)
{
  if (m_state == PARKED)
    set_event (Event::START_ENGINE);
  else if (m_mode == QUALIFY)
    set_event (Event::DRIVE, Vamos_Geometry::random_in_range (10, 60));
  else if (to_go <= 0.0)
    set_event (Event::DRIVE, reaction_time ());
  else if (to_go <= 1.0)
    set_event (Event::REV);
}

void
Robot_Driver::finish ()
{
  set_event (Event::DONE);
}

void
Robot_Driver::set_event (Event::Type type, double delay)
{
  // Ignore successive events of the same type.
  if (type != m_event.type)
    m_event = Event (type, delay);
}

void 
Robot_Driver::propagate (double timestep) 
{
  // It's useful for other methods to know the size of the current timestep.
  m_timestep = timestep;
  handle_event ();
  m_speed = mp_car->chassis ().cm_velocity ().magnitude();

  if (is_driving ())
    drive ();
}

void
Robot_Driver::handle_event ()
{
  m_event.propagate (m_timestep);
  if (!m_event.ready ())
    return;

  switch (m_event.type)
    {
    case Event::PARK:
      set_brake (1.0);
      mp_car->disengage_clutch (0.0);
      mp_car->shift (0);
      mp_car->start_engine ();
      set_gas (0.0);
      m_state = PARKED;
      break;

    case Event::START_ENGINE:
      mp_car->disengage_clutch (0.0);
      mp_car->shift (0);
      mp_car->start_engine ();
      set_gas (0.0);
      m_state = IDLING;
      break;

    case Event::REV:
      mp_car->disengage_clutch (0.0);
      mp_car->shift (1);
      set_gas (0.5);
      m_state = REVVING;
      break;

    case Event::DRIVE:
      static const double clutch_time = 2.0;
      if (m_mode == QUALIFY && !has_gap ())
        m_event.reset ();
      else if (m_state != DRIVING)
        {
          mp_car->shift (1);
          mp_car->engage_clutch (clutch_time);
          m_state = DRIVING;
        }
      break;

    case Event::DONE:
      m_state = COOL_DOWN;
      m_traction_control.set (m_target_slip * 0.5);
      m_braking.scale (0.5);
      break;

    case Event::NO_EVENT:
      break;

    default:
      assert (false);
      break;
    }
}

double Robot_Driver::lengths (double n) const
{
  //! replace n*mp_car->length() everywhere.
  return n * mp_car->length ();
}

bool Robot_Driver::has_gap () const
{
  double along = info ().track_position ().x;
  for (std::vector <Car_Information>::const_iterator it = mp_cars->begin ();
       it != mp_cars->end ();
       it++)
    {
      if (it->driver->is_driving ())
        {
          double delta = (it->track_position ().x - along);
          if ((delta - m_road.length () > lengths (-100.0)) || (delta < lengths (25.0)))
            return false;
        }
    }
  return true;
}

bool Robot_Driver::is_driving () const
{
  return (m_state == DRIVING || m_state == COOL_DOWN);
}

//* Drive
void Robot_Driver::drive ()
{
  mp_segment = m_road.segments ()[info ().segment_index];

  steer ();
  choose_gear ();
  accelerate ();

  // Avoid collisions last since we may override steering and braking.
  avoid_collisions ();
}

double Robot_Driver::get_lane_shift () const
{
  return lane_shift (info ().track_position ());
}

double Robot_Driver::lane_shift (const Three_Vector& track) const
{
  // Return the lane shift parameter for the current position.
  double along = track.x;
  double line_y = m_racing_line.from_center (along, info ().segment_index);
  double offline = track.y - line_y;
  if (offline > 0.0)
    {
      double left = m_road.racing_line ().left_width (m_road, along);
      return std::min (offline / (left - line_y), 1.0);
    }
  else
    {
      double right = m_road.racing_line ().right_width (m_road, along);
      return std::max (offline / (right + line_y), -1.0);
    }
}

double Robot_Driver::get_offline_distance () const
{
  return offline_distance (info ().m_pointer_position.x, m_lane_shift);
}

double Robot_Driver::offline_distance (double along, double lane_shift) const
{
  double line_y = m_racing_line.from_center (along, info ().segment_index);
  if (m_lane_shift > 0.0)
    {
      double left = m_road.racing_line ().left_width (m_road, along);
      return m_lane_shift * (left - line_y);
    }
  else
    {
      double right = m_road.racing_line ().right_width (m_road, along);
      return m_lane_shift * (right + line_y);
    }
}

//** Steer
void
Robot_Driver::steer ()
{
  // Steer to keep the car on the racing line. The target is ahead of the
  // car as if attached to a pole that sticks out in front of the car. Trying to
  // steer to keep the car's position on the racing line leads to
  // instability. Using a target that's ahead of the car is very stable.

  // Steer by an amount that's proportional to the cross product of the actual
  // and desired directions. For small angles steering is approximately
  // proportional to the angle between them. The steering wheel is set according
  // to the output of a PID controller (although only the P term seems to be
  // necessary).
  double angle = pointer_vector ().cross (target_vector ()).z;
  // 'angle' is |r_p||r_t|sin theta ~ r_t^2 theta.  The additional angle to the
  // off-line position is sin^-1 (r_o/r_t) ~ r_o/r_t.  After scaling by r_t^2 to
  // agree with 'angle' it's r_o r_t.
  m_steer_control.set (angle + get_offline_distance () * mp_car->target_distance ());
  set_steering (m_steer_control.propagate (mp_car->steer_angle (), m_timestep));
}

Three_Vector
Robot_Driver::pointer_vector () const
{
  // Return a vector that points in the direction of the point that the driver
  // tries to keep on the racing line.
  return mp_car->target_position () - mp_car->center_position ();
}

Three_Vector
Robot_Driver::target_vector ()
{
  /// Return a vector that points in the direction of the position of the point on
  /// the track that the driver aims for.
  Three_Vector target = m_racing_line.target (info ().track_position ().x,
                                              mp_car->target_distance ());
  Three_Vector center = mp_car->center_position ();
  return Three_Vector (target.x - center.x,
                       target.y - center.y,
                       0.0);
}

void
Robot_Driver::set_steering (double angle)
{
  // Set the steering angle to 'angle'. Clip to a reasonable range. The 'true'
  // argument indicates "direct steering"; non-linearity and speed-sensitivity
  // are ignored.
  const double max_angle = 1.5 * target_slip_angle ();
  mp_car->steer (clip (angle, -max_angle, max_angle), 0.0, true);
}

double
Robot_Driver::target_slip_angle () const
{
  //! Can we use a constant instead?
  return abs_max (mp_car->wheel (0)->peak_slip_angle (),
                  mp_car->wheel (1)->peak_slip_angle (),
                  mp_car->wheel (2)->peak_slip_angle (),
                  mp_car->wheel (3)->peak_slip_angle ());
}

//** Choose Gear
void
Robot_Driver::choose_gear ()
{
  if (!mp_car->clutch ()->engaged ())
    return;

  // Avoid shifting too frequently.
  m_shift_time += m_timestep;
  if (m_shift_time < 0.3)
    return;

  // Save some values that show up often in this method.
  const int gear = mp_car->gear ();
  const double throttle = mp_car->engine ()->throttle ();

  // Find current the engine power and the power at maximum throttle in 1) the
  // current gear 2) the next gear up 3) 2 gears down. Two gears to prevent
  // downshifting to 1st and to prevent too much engine drag under braking.
  double omega = mp_car->engine ()->rotational_speed ();
  double up_omega = omega
    * mp_car->transmission ()->gear_ratio (gear + 1)
    / mp_car->transmission ()->gear_ratio (gear);
  double down2_omega = omega
    * mp_car->transmission ()->gear_ratio (gear - 2)
    / mp_car->transmission ()->gear_ratio (gear);

  double current_power = mp_car->engine ()->power (throttle, omega);
  double power = mp_car->engine ()->power (1.0, omega);
  double up_power = mp_car->engine ()->power (1.0, up_omega);
  double down2_power = mp_car->engine ()->power (1.0, down2_omega);

  // Shift up if there's more power at the current revs in the next higher
  // gear. 
  if (up_power > power)
    {
      mp_car->shift_up ();
    }
  // Shift up if there's enough power in the next higher gear even if there's
  // potentially more power in the current gear. This saves fuel by lowering
  // revs.
  else if ((up_power > current_power)
           && (up_power > 0.95 * power)
           && (throttle > 0.1)
           && (throttle < 0.9))
    {
      mp_car->shift_up ();
    }
  // Shift down if there's more power 2 gears down.
  else if ((down2_power > power)
           && (gear > 2)
           && ((m_state != COOL_DOWN) || (gear > 3)))
    {
      mp_car->shift_down ();
    }

  // Reset the timer if the gear actually changed.
  if (mp_car->gear () != gear)
    m_shift_time = 0.0;
}

//** Accelerate
void
Robot_Driver::accelerate ()
{
  // Try to achieve the maximum safe speed for this point on the track.

  // Save some values that show up often in this method. The normal vector
  // calculation requires distance along the segment instead of distance along
  // the track.
  const double along_segment
    = info ().track_position ().x - mp_segment->start_distance ();
  // The 'true' argument causes kerbs to be ignored when calculating the normal
  // for the racing line speed.
  const Three_Vector normal = mp_segment->normal (along_segment, 
                                                  info ().track_position ().y, 
                                                  false);
  const double drag = mp_car->chassis ().aerodynamic_drag ();
  const double lift = mp_car->chassis ().aerodynamic_lift ();

  double along = info ().track_position ().x;
  double cornering_speed 
    = m_racing_line.maximum_speed (along,
                                   m_lane_shift,
                                   lift,
                                   normal,
                                   mp_car->chassis ().mass ());
  double braking_speed
    = m_braking.maximum_speed (m_speed,
                               along,
                               (m_speed_factor < 1.0) ? lengths (2.0) : 0.0,
                               m_lane_shift,
                               drag,
                               lift,
                               mp_car->chassis ().mass ());

  set_speed (std::min (cornering_speed, braking_speed));
}

void
Robot_Driver::set_speed (double target_speed)
{
  // Set the gas and brake pedal positions. Three PID controllers are involved
  // in setting the pedal position: 'm_speed_control' ant 'm_brake_control'
  // attempt to reach the target speed, 'm_traction_control' attempts to limit
  // wheel spin. Nothing prevents the gas and brake from being applied
  // simultaneously, but with only P terms of the PID controllers being used it
  // does not happen in practice.
  // Speed may be modulated to avoid running into the back of the car in front.
  target_speed *= mp_car->grip () * m_speed_factor;
  m_speed_control.set (target_speed);
  double d1 = m_speed_control.propagate (m_speed, m_timestep);
  double d2 = m_traction_control.propagate (total_slip (), m_timestep);
  double gas = std::min (d1, d2);

  if (!mp_car->clutch ()->engaged ())
    {
      // Keep the revs in check if the clutch is not fully engaged.
      m_speed_control.set (0.0);
      double error = (mp_car->engine ()->rotational_speed ()
                      - mp_car->engine ()->peak_engine_speed ());
      gas = std::min (gas, m_speed_control.propagate (0.01 * error, m_timestep));
    }

  if (gas >= 1.0)
    gas *= m_speed_factor;
  if (m_state == COOL_DOWN)
    gas = std::min (gas, 0.5);
  set_gas (gas * m_speed_factor);

  m_brake_control.set (std::min (target_speed, m_speed));
  double b1 = -m_brake_control.propagate (m_speed, m_timestep);
  double b2 = m_traction_control.propagate (total_slip (), m_timestep);
  set_brake (std::min (b1, b2));
}

double
Robot_Driver::total_slip () const
{
  return Three_Vector (longitudinal_slip (), transverse_slip (), 0.0).magnitude ();
}

double
Robot_Driver::longitudinal_slip () const
{
  return abs_max (mp_car->wheel (0)->slip ().x,
                  mp_car->wheel (1)->slip ().x,
                  mp_car->wheel (2)->slip ().x,
                  mp_car->wheel (3)->slip ().x);
}

double
Robot_Driver::transverse_slip () const
{
  return abs_max (mp_car->wheel (0)->slip ().y,
                  mp_car->wheel (1)->slip ().y,
                  mp_car->wheel (2)->slip ().y,
                  mp_car->wheel (3)->slip ().y);
}

void
Robot_Driver::set_gas (double gas)
{
  if (gas <= 0.0)
    {
      // Erase the controllers' history.
      m_speed_control.reset ();
      m_traction_control.reset ();
    }
  mp_car->gas (clip (gas, 0.0, 1.0));  
}

void
Robot_Driver::set_brake (double brake)
{
  if (brake <= 0.0)
    {
      // Erase the controllers' history.
      m_brake_control.reset ();
      m_traction_control.reset ();
    }
  mp_car->brake (clip (brake, 0.0, 1.0));
}

//** Avoid Collisions
void
Robot_Driver::avoid_collisions ()
{
  // Take action to pass or avoid potential collisions.
  if (mp_cars == 0) return;

  // Loop through the other cars. Each time through the loop we potentially
  // update all of these variables.
  double min_forward_distance_gap = 100;
  double min_left_distance_gap = 100;
  double min_right_distance_gap = 100;
  double along = info ().track_position ().x;
  double follow_lengths = ((m_state == COOL_DOWN)
                           ? 3.0
                           : (maybe_passable (along, info ().segment_index) 
                              ? 0.5 
                              : 1.5));
  double crash_time = 2.0*CRASH_TIME_LIMIT;
  Direction pass_side = NONE;
  Three_Vector v1
    = mp_car->chassis ().rotate_from_world (mp_car->chassis ().cm_velocity ());

  // Break out immediately if the interact flag is false. We still need the lane
  // shift code after the loop so the cars will get to the racing line after the
  // start.  
  for (std::vector <Car_Information>::const_iterator it = mp_cars->begin ();
       it != mp_cars->end () && m_interact;
       it++)
    {
      // Don't check for collisions with yourself.
      if (it->car == mp_car)
        continue;

      // Don't worry about cars that are far away.
      if (std::abs (it->track_position ().x - along) > lengths (10))
        continue;

      Three_Vector v2 
        = it->car->chassis ().rotate_from_world (it->car->chassis ().cm_velocity ());

      Three_Vector delta_v = (v2 - v1);
      Three_Vector distance_gap = find_gap (info ().m_pointer_position,
                                            it->m_pointer_position);
      switch (relative_position (info ().track_position (), it->track_position ()))
        {
        case FORWARD:
          if (distance_gap.x < min_forward_distance_gap)
            {
              min_forward_distance_gap = distance_gap.x;
              crash_time = -distance_gap.x / delta_v.x;

              if ((min_forward_distance_gap < 0.0) || (crash_time < 0.0))
                {
                  m_passing = false;
                }
              else if (m_passing)
                {
                  follow_lengths = -distance_gap.y / mp_car->length ();
                }
              pass_side = get_pass_side (along,
                                         distance_gap.x, 
                                         -delta_v.x,
                                         info ().segment_index);
            }
          break;
        case LEFT:
          if (distance_gap.y < min_left_distance_gap)
            {
              min_left_distance_gap = distance_gap.y;
              crash_time = -distance_gap.y / delta_v.y;
            }
          break;
        case RIGHT:
          if (distance_gap.y < min_right_distance_gap)
            {
              min_right_distance_gap = distance_gap.y;
              crash_time = distance_gap.y / delta_v.y;
            }
          break;
        default:
          break;
        }
    }

  m_gap [4] = crash_time;
  const double shift_step = 0.4 * m_timestep;

  // Keep track of how line we've been off the racing line.
  if (m_passing || m_lane_shift != 0.0)
    m_lane_shift_timer += m_timestep;

  if ((min_right_distance_gap < 0.5*mp_car->width ())
      && (min_left_distance_gap > min_right_distance_gap))
    {
      // Move to the left if the car on the right is too close.
      m_lane_shift = std::min (1.0, m_lane_shift + shift_step);
      m_lane_shift_timer = 0.0;
    }
  else if (min_left_distance_gap < 0.5*mp_car->width ())
    {
      // Move to the right if the car on the left is too close.
      m_lane_shift = std::max (-1.0, m_lane_shift - shift_step);
      m_lane_shift_timer = 0.0;
    }
  else if (m_lane_shift_timer > PASS_TIME)
    { 
      // No danger of collision.  Get back to the racing line.
      m_passing = false;
      if ((m_lane_shift > 0.0) && (min_right_distance_gap > 0.5*mp_car->width ()))
        m_lane_shift = std::max (0.0, m_lane_shift - 0.5*shift_step);
      else if ((m_lane_shift < 0.0) && (min_left_distance_gap > 0.5*mp_car->width ()))
        m_lane_shift = std::min (0.0, m_lane_shift + 0.5*shift_step);
    }
  else if ((m_lane_shift > 0.2)
           && (std::abs (get_offline_distance ()) < 0.2*mp_car->width ()))
    {
      // Get back to the racing line if we're already close.
      m_lane_shift_timer = 2.0*PASS_TIME;
    }

  if (crash_time < CRASH_TIME_LIMIT)
    {
      // Go offline to pass.
      switch (pass_side)
        {
        case LEFT:
          m_passing = true;
          m_lane_shift = std::min (1.0, m_lane_shift + shift_step);
          m_lane_shift_timer = 0.0;
          break;
        case RIGHT:
          m_passing = true;
          m_lane_shift = std::max (-1.0, m_lane_shift - shift_step);
          m_lane_shift_timer = 0.0;
          break;
        default:
          break;
        }
    }

  m_gap [0] = min_forward_distance_gap;
  m_gap [1] = min_left_distance_gap;
  m_gap [2] = min_right_distance_gap;
  m_gap [3] = std::min (lengths (follow_lengths), 0.5*m_speed);

  // Calculate the speed factor here while we have access to the gap.
  m_front_gap_control.set (std::min (lengths (follow_lengths), 0.5*m_speed));
  m_speed_factor = 
    1.0 - clip (m_front_gap_control.propagate (min_forward_distance_gap, m_timestep),
                0.0, 1.0);
}

Direction
Robot_Driver::relative_position (const Three_Vector& r1_track,
                                 const Three_Vector& r2_track) const
{
  const double corner = mp_car->width () / mp_car->length ();
  Three_Vector delta = r2_track - r1_track;
  const double slope = delta.y / delta.x;

  if (((delta.x > mp_car->length ()) && (std::abs (delta.y) > mp_car->width ()))
      || (delta.x < -mp_car->length ()))
    return NONE;
  if (std::abs (slope) < corner)
    return (delta.x > 0) ? FORWARD : NONE;

  return (delta.y > 0) ? LEFT : RIGHT;
}

Three_Vector
Robot_Driver::find_gap (const Three_Vector& r1_track,
                        const Three_Vector& r2_track) const
{
  return Three_Vector (m_road.distance (r2_track.x, r1_track.x) - mp_car->length (),
                       std::abs (r2_track.y - r1_track.y) - mp_car->width (),
                       0.0);
}

bool
Robot_Driver::maybe_passable (double along, size_t segment) const
{
  return (pass_side (along, 0.5*m_speed, 10, segment) != NONE);
}

Direction
Robot_Driver::get_pass_side (double along, double delta_x, double delta_v,
                             size_t segment) const
{
  // Return 'RIGHT' if we should try to pass on the right, 'LEFT' if we should
  // try to pass on the left, 'NONE' if it's better not to try.

  // Give up immediately if we're not closing at a significant rate. Returning
  // here avoids a possible divide-by-zero below. 
  if (delta_v < 1e-6)
    return NONE;

  // We'll try to pass if the racing line stays on one side of the track far
  // enough ahead for us to pull alongside the opponent's car at our current
  // rate of closing, 'delta_v'.

  //!! Look ahead by time, not distance to compensate for closing under braking.
  //!! Account for deceleration? 
  double pass_distance = delta_x * m_speed / delta_v;
  if (pass_distance > lengths (100))
    return NONE;

  return pass_side (along + pass_distance/2, pass_distance/20, 10, segment);
}

Direction
Robot_Driver::pass_side (double start, double delta, size_t n, size_t segment) const
{
  // The logic here requires that *_SIDE are all single bits. Don't use the
  // Direction enum.

  //!! calls to get_block_side and from_center sometime increment the segment
  //!! excessively. 

  int block = 0;
  for (size_t i = 1; i <= n; i++)
    block |= get_block_side (start + i*delta, segment);

  if (block & LEFT_SIDE)
    return (block & RIGHT_SIDE) ? NONE : RIGHT;
  if (block & RIGHT_SIDE)
    return LEFT;

  return (m_racing_line.from_center (start, segment) > 0.0) ? RIGHT : LEFT;
}

Robot_Driver::Track_Side
Robot_Driver::get_block_side (double along, size_t segment) const
{
  // The logic here requires that *_SIDE are all single bits. Don't use the
  // Direction enum.
  const double across = m_racing_line.from_center (along, segment);
  if (across < -m_road.right_racing_line_width (along) + mp_car->width ())
    return RIGHT_SIDE;
  else if (across > m_road.left_racing_line_width (along) - mp_car->width ())
    return LEFT_SIDE;
  return NO_SIDE;
}

//** Draw
void
Robot_Driver::draw ()
{
  {
    double y = 94;
    Two_D screen;
    screen.text (20, y -= 2, "follow ", m_gap [3], "", 3);
    screen.text (20, y -= 2, "crash ", m_gap [4], "s", 1);
    screen.text (20, y -= 2, "pass ", m_passing, "", 0);
    screen.text (20, y -= 2, "forward ", m_gap [0], "m", 3);
    screen.text (20, y -= 2, "speed ", m_speed_factor, "", 3);
    screen.text (20, y -= 2, "left ", m_gap [1], "m", 3);
    screen.text (20, y -= 2, "right ", m_gap [2], "m", 3);
    screen.text (20, y -= 2, "offline ", get_offline_distance (), "m", 3);
    screen.text (20, y -= 2, "shift ", m_lane_shift, "", 3);
  }

  /// Optionally show the target and vector as green and red squares.
  if (!m_show_steering_target)
    return;

  glLoadIdentity ();
  glPointSize (8.0);
  glBegin (GL_POINTS);

  // Draw where the car is currently pointed.
  const Three_Vector pointer (mp_car->center_position () + pointer_vector ());  
  glColor3d (0.0, 0.8, 0.0);
  glVertex3d (pointer.x, pointer.y, pointer.z);

  // Draw the point on the racing line.
  const Three_Vector goal (mp_car->center_position () + target_vector ());
  glColor3d (8.0, 0.0, 0.0);
  glVertex3d (goal.x, goal.y, pointer.z);

  glEnd ();
}

//-----------------------------------------------------------------------------

// The distance resolution of the braking speed calculation
static const double delta_x = 10.0;
// Braking is applied gradually.  It reaches its maximum in this many meters.
static const double fade_in = 50.0;

Braking_Operation::Braking_Operation (const Road& road,
                                      double deceleration,
                                      double gravity,
                                      const Robot_Racing_Line& line)
  : m_start (0.0),
    m_length (0.0),
    m_is_braking (false),
    m_road (road),
    m_deceleration (deceleration),
    m_gravity (gravity),
    m_line (line)
{
}

Braking_Operation::~Braking_Operation ()
{
  // Do the proper cleanup if we were deleted during a braking operation.
  end ();
}

void
Braking_Operation::scale (double factor)
{
  m_deceleration *= factor;
}

void
Braking_Operation::start (double start, double length)
{
  // Use start distance and length instead of start and end to avoid issues with
  // wrapping around the track.
  m_start = start;
  m_length = length;
  m_is_braking = true;
}

void
Braking_Operation::end ()
{
  m_is_braking = false;
}

bool
Braking_Operation::check_done_braking (double distance)
{
  if (past_end (distance))
    end ();
  return !m_is_braking;
}

double
Braking_Operation::distance_from_start (double distance) const
{
  if (distance >= m_start)
    return (distance - m_start);
  else // wrap around the track
    return (distance + m_road.length () - m_start);
}

bool
Braking_Operation::past_end (double distance) const
{
  return (distance_from_start (distance) > m_length);
}

double 
Braking_Operation::deceleration (const Three_Vector& curvature,
                                 double speed, 
                                 double drag,
                                 double lift,
                                 const Three_Vector& n_hat,
                                 const Three_Vector& p_hat,
                                 double mass,
                                 double fraction) const
{
  double c = curvature.magnitude ();
  double mu = m_deceleration * fraction;
  double v2 = speed * speed;
  Three_Vector r_hat = curvature.unit ();
  return (m_gravity*p_hat.z 
          - v2*drag/mass
          + mu * (m_gravity*n_hat.z - v2*(lift/mass + c*r_hat.dot (n_hat))));
}

Three_Vector
Braking_Operation::get_normal (double along) const
{
  const Road_Segment* segment = m_road.segment_at (along);
  double along_segment = along - segment->start_distance ();
  return segment->normal (along_segment, 0.0);
}

double
Braking_Operation::delta_braking_speed (double speed,
                                        double cornering_speed,
                                        double along,
                                        double lane_shift,
                                        const Three_Vector& normal,
                                        double drag, 
                                        double lift, 
                                        double mass,
                                        double fraction) const
{
  Three_Vector curvature = m_line.curvature (along, lane_shift);
  Three_Vector p = m_line.tangent (along);

  double a = deceleration (curvature, speed, drag, lift, normal, p, mass, fraction);
  double a_par = a * (1.0 - speed / cornering_speed);
  return a_par * delta_x / speed;
}

double 
Braking_Operation::maximum_speed (double speed,
                                  double distance,
                                  double stretch,
                                  double lane_shift,
                                  double drag,
                                  double lift,
                                  double mass)
{
  // Return the maximum safe speed under braking.
  distance += stretch;

  // See if we've past the end of a braking operation.
  check_done_braking (distance);

  // If we're in the middle of a braking operation, get the speed from the speed
  // vs. braking curve.
  if (is_braking ())
    {
      if (distance < m_speed_vs_distance [0].x)
        distance += m_road.length ();
      return m_speed_vs_distance.interpolate (distance);
    }

  // Calculate the car's speed as a function of distance if braking started now.
  // If the projected speed exceeds the maximum cornering speed calculated from
  // the racing line then braking should start now.  When this happens, find the
  // minimum cornering speed and calculate distance vs. speed backwards from
  // there. 

  Two_Vector minimum_speed_vs_distance (0.0, speed);

  // True if projected braking speed exceeds cornering speed anywhere.
  bool too_fast = false;
  // True if a minimum in the cornering speed was found in the distance range
  // where projected braking speed exceeds cornering speed.
  bool found_min = false;
  double start_speed = speed;
  // Look up to 1000 m ahead.
  for (double d = 0.0; d < 1000.0; d += delta_x)
    {
      double along = wrap (distance + d, m_road.length ());
      Three_Vector normal = get_normal (along);
      double cornering_speed 
        = m_line.maximum_speed (along, lane_shift, lift, normal, mass);

      // Apply braking gradually.
      double braking_fraction = std::min (d / fade_in, 1.0);
      double dv = delta_braking_speed (speed, cornering_speed, along, lane_shift, 
                                       normal, drag, lift, mass, 
                                       braking_fraction);

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

      if (too_fast && (cornering_speed < minimum_speed_vs_distance.y))
        minimum_speed_vs_distance = Two_Vector (d, cornering_speed);
    }

  // No need to start braking yet.
  if (!found_min)
    return UNLIMITED_SPEED;

  // Build the speed vs. distance curve by working backwards from the minimum
  // speed.  Start one interval beyond the end; end one interval before the
  // beginning to ensure that the interpolations are good slightly beyond the
  // endpoints.
  too_fast = false;
  std::vector <Two_Vector> points;
  speed = minimum_speed_vs_distance.y;
  for (double d = minimum_speed_vs_distance.x; d > -delta_x; d -= delta_x)
    {
      // Use un-wrapped distances so the interpolator's points are in order. 
      points.push_back (Two_Vector (distance + d, speed));

      double along = wrap (distance + d, m_road.length ());
      Three_Vector normal = get_normal (along);
      double cornering_speed 
        = m_line.maximum_speed (along, lane_shift, lift, normal, mass);

      double braking_fraction = std::min (d / fade_in, 1.0);
      double dv = delta_braking_speed (speed, cornering_speed, along, lane_shift, 
                                       normal, drag, lift, mass, braking_fraction);

      if (too_fast && (cornering_speed < minimum_speed_vs_distance.y))
        minimum_speed_vs_distance = Two_Vector (d, cornering_speed);

      if (speed >= cornering_speed)
        {
          if (!too_fast)
            {
              minimum_speed_vs_distance = Two_Vector (d, cornering_speed);
              // Found an earlier curve that requires a lower speed.
              too_fast = true;
            }
        }
      else if (too_fast)
        {
          // Found the new minimum.  Start over from there.
          d = minimum_speed_vs_distance.x;
          speed = minimum_speed_vs_distance.y;
          points.clear ();
          points.push_back (minimum_speed_vs_distance 
                            + Two_Vector (distance + delta_x, 0.0));
          too_fast = false;
        }
      else
        speed += dv;
    }

  // The interpolator requires ascending x-values.  Reverse the points.
  m_speed_vs_distance.clear ();
  std::reverse (points.begin (), points.end ());
  m_speed_vs_distance.load (points);

  // Scale speed vs. distance so it matches the passed-in speed at the passed-in
  // distance.  This is usually a small adjustment, but can be significant when
  // curves are closely spaced.
  double delta_speed = start_speed - m_speed_vs_distance [0].y;
  for (size_t i = 0; i < m_speed_vs_distance.size (); i++)
    {
      m_speed_vs_distance [i].x -= 
        stretch * (m_speed_vs_distance.size () - i)/m_speed_vs_distance.size ();

      double fraction 
        = ((distance + minimum_speed_vs_distance.x - m_speed_vs_distance [i].x)
           / (distance + minimum_speed_vs_distance.x - m_speed_vs_distance [0].x));
      m_speed_vs_distance [i].y += fraction * delta_speed;
    }

  // Mark the start of a braking operation.
  start (distance, minimum_speed_vs_distance.x);
  // No need to restrict speed yet.  Next call will get it from the newly
  // calculated speed vs. distance curve.
  return UNLIMITED_SPEED;
}

//-----------------------------------------------------------------------------
Robot_Racing_Line::Robot_Racing_Line (const Road& road,
                                      double lateral_acceleration,
                                      double gravity)
  : mp_road (&road),
    m_lateral_acceleration (lateral_acceleration),
    m_gravity (gravity)
{
}

Three_Vector
Robot_Racing_Line::curvature (double along, double lane_shift) const
{
  return mp_road->racing_line ().curvature (along, lane_shift);
}

Three_Vector
Robot_Racing_Line::tangent (double along) const
{
  return mp_road->racing_line ().tangent (along);
}

double
Robot_Racing_Line::maximum_speed (double distance, 
                                  double lane_shift,
                                  double lift,
                                  const Three_Vector& n_hat,
                                  double mass) const
{
  const Three_Vector curve = curvature (distance, lane_shift);
  double c = curve.magnitude ();
  double mu = m_lateral_acceleration;
  Three_Vector r_hat = curve.unit ();
  Three_Vector r_perp (-r_hat.y, r_hat.x, 0.0);
  Three_Vector q_hat = n_hat.rotate (0.5 * pi * r_perp.unit ());

  double upper = m_gravity*(q_hat.z + mu*n_hat.z);
  double lower = c*(r_hat.dot(q_hat) + mu*r_hat.dot(n_hat)) + mu*lift/mass;

  if (lower > 1e-9)
    return std::sqrt (upper / lower);
  else
    return UNLIMITED_SPEED;
}

Three_Vector
Robot_Racing_Line::target (double along, double lead) const
{
  return mp_road->racing_line ().position (along + lead);
}

double
Robot_Racing_Line::from_center (double along, size_t segment) const
{
  return mp_road->track_coordinates (target (along, 0.0), segment, true).y;
}
