//  Copyright (C) 2001--2012 Sam Varner
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

#include "../geometry/Numeric.h"

#include "Timing_Info.h"
#include "World.h"

#include <algorithm>
#include <limits>

using namespace Vamos_World;

const double Timing_Info::NO_TIME = std::numeric_limits <double>::max ();
const int N_COUNTDOWN_START = 6;

Timing_Info::Timing_Info (size_t n_cars, size_t n_sectors, bool do_start_sequence )
  : m_sectors (n_sectors),
    m_lap_limit (0),
    m_time_limit (0.0),
    m_qualifying (false),
    m_countdown (do_start_sequence ? N_COUNTDOWN_START : 0),
    m_start_delay (Vamos_Geometry::random_in_range (0.0, 4.0)),
    m_state (do_start_sequence ? STARTING : RUNNING),
    m_total_time (0.0),
    m_state_time (0.0),
    mp_fastest (0),
    m_fastest_lap (NO_TIME)
{
  assert (n_sectors > 0);

  for (size_t i = 0; i < n_cars; i++)
    {
      Car_Timing* p_car = new Car_Timing (i + 1, n_sectors);
      ma_car_timing.push_back (p_car);
      ma_running_order.push_back (p_car);
      if (i == 0)
        mp_fastest = p_car;
    }
}

Timing_Info::~Timing_Info ()
{
  for (size_t i = 0; i < ma_car_timing.size (); i++)
    delete ma_car_timing [i];
}

void Timing_Info::reset ()
{
  m_total_time = 0.0;
  mp_fastest = 0;
  m_fastest_lap = NO_TIME;

  ma_running_order.clear ();
  for (size_t i = 0; i < ma_car_timing.size (); i++)
    {
      Car_Timing* p_car = ma_car_timing [i];
      p_car->reset ();
      ma_running_order.push_back (p_car);
      if (i == 0)
        mp_fastest = p_car;
    }
}

void Timing_Info::update (double current_time,
                          size_t index,
                          double distance,
                          size_t sector)
{
  assert (index < ma_car_timing.size ());
  assert (sector <= m_sectors);

  switch (m_state)
    {
    case STARTING:
      {
        double to_go = N_COUNTDOWN_START - (current_time - m_state_time);
        m_countdown = std::max (int (to_go + 1), 1);
        if (to_go < -m_start_delay)
          {
            m_countdown = 0;
            next_state (current_time);
          }
        break;
      }
    case RUNNING:
    case FINISHED:
      {
        m_total_time = current_time - m_state_time;

        Car_Timing* p_car = ma_car_timing [index];
        const bool new_sector = is_new_sector (index, sector);
        const bool new_lap = new_sector && (sector == 1);
        const bool already_finished = p_car->m_finished;

        p_car->update (m_total_time, distance, sector, new_sector);

        const bool laps_done = ((m_lap_limit > 0)
                                && (ma_car_timing [index]->current_lap () > m_lap_limit));
        const bool time_done = ((m_time_limit > 0.0) && (m_total_time > m_time_limit));

        // A car's race is done when
        // 1. it completes all the laps
        // 2. it completes any lap after any car has completed all laps
        // 3. it completes any lap after time has expired
        if (laps_done || (new_lap && (time_done || (m_state == FINISHED))))
          p_car->set_finished ();
 
       if (new_sector)
          update_position (p_car, m_total_time, sector, already_finished);

        // Go to the FINISHED state when the first car has completed all of the
        // laps or time runs out. Timing must continue for the other cars, so we
        // keep m_state_time.
        if ((m_state == RUNNING) && (laps_done || time_done))
          {
            next_state (m_state_time);
          }
        break;
      }
    }
}

double Timing_Info::time_remaining () const
{
  return (m_time_limit == 0.0 ? 0.0 : std::max (m_time_limit - m_total_time, 0.0));
}

void Timing_Info::next_state (double current_time)
{
  switch (m_state)
   {
   case STARTING:
     m_state = RUNNING;
     break;
   case RUNNING:
     m_state = FINISHED;
     break;
   case FINISHED:
   default:
     assert (false);
     break;
   }

  m_state_time = current_time;
}

void Timing_Info::update_position (Car_Timing* p_car,
                                   double current_time,
                                   size_t sector,
                                   bool finished)
{
  assert ((sector > 0) && (sector <= m_sectors));
  if ((p_car->previous_sector () == 0) || finished)
    return;

  const size_t nth_sector = m_sectors * (p_car->current_lap () - 1) + sector - 1;

  Timing_Info::Running_Order::iterator new_position = ma_running_order.begin ();
  double interval = NO_TIME;
  if (m_qualifying)
    {
      if (sector != 1)
        return;

      Timing_Info::Running_Order::const_iterator it = ma_running_order.begin ();
      while ((it != ma_running_order.end ())
             && ((*it)->best_lap_time () != NO_TIME) 
             && (p_car->best_lap_time () > (*it)->best_lap_time ()))
        {
          ++it;
          ++new_position;
        }
    }
  else if (nth_sector > ma_sector_position.size ())
    {
      ma_sector_position.push_back (1);
      ma_sector_time.push_back (current_time);
    }
  else
    {
      size_t p = ma_sector_position [nth_sector - 1]++;
      for (; p > 0; p--)
          ++new_position;
      interval = current_time - ma_sector_time [nth_sector - 1];
      ma_sector_time [nth_sector - 1] = current_time;
    }

  // If this car has lost positions it will have been pushed down the running
  // order by the cars that have already reached this sector. Thus, old_position
  // is at or after new_position.
  Timing_Info::Running_Order::iterator old_position 
    = std::find (new_position, ma_running_order.end (), p_car);
  if (new_position != old_position)
    {
      ma_running_order.insert (new_position, *old_position);
      ma_running_order.erase (old_position);
    }
  p_car->m_interval = interval;

  const double best = p_car->best_lap_time ();
  if ((best != NO_TIME)
      && ((m_fastest_lap == NO_TIME) || (best < m_fastest_lap)))
    {
      mp_fastest = p_car;
      m_fastest_lap = best;
    }
}

bool Timing_Info::is_new_sector (size_t index, size_t sector) const
{
  const size_t current = ma_car_timing [index]->current_sector ();
  // Do the % before + because sector is 1-based.
  return (sector == (current % m_sectors) + 1);
}

Timing_Info::Car_Timing::Car_Timing (size_t position, size_t sectors)
  : m_grid_position (position),
    m_current_time (0.0),
    m_distance (0.0),
    m_interval (NO_TIME),
    m_sectors (sectors),
    m_sector (0),
    m_last_sector (0),
    m_lap (0),
    m_best_lap_time (NO_TIME),
    m_lap_delta (NO_TIME),
    m_finished (false)
{
  ma_best_sector_time.resize (m_sectors);
  ma_sector_delta.resize (m_sectors);
  for (size_t sector = 0; sector < m_sectors; sector++)
    {
      ma_best_sector_time [sector] = NO_TIME;
      ma_sector_delta [sector] = NO_TIME;
    }
}

void Timing_Info::Car_Timing::reset ()
{
  m_interval = NO_TIME;
  m_sector = 0;
  m_last_sector = 0;
  m_lap = 0;
  m_best_lap_time = NO_TIME;
  m_lap_delta = NO_TIME;
  m_finished = false;

  ma_lap_time.clear ();
  ma_sector_time.clear ();
  for (size_t sector = 0; sector < m_sectors; sector++)
    {
      ma_best_sector_time [sector] = NO_TIME;
      ma_sector_delta [sector] = NO_TIME;
    }
}

void Timing_Info::Car_Timing::update (double current_time, 
                                      double distance, 
                                      size_t sector,
                                      bool new_sector)
{
  m_current_time = current_time;
  m_distance = distance;
  if (!m_finished && new_sector)
    {
      if (is_start_of_lap (sector))
          update_lap_data (current_time);

      update_sector_data (current_time, sector);
    }
}

bool Timing_Info::Car_Timing::is_start_of_lap (size_t sector) const
{
  return (sector == 1);
}

void Timing_Info::Car_Timing::update_lap_data (double current_time)
{
  m_lap++;
  if (m_sector > 0)
    {
      ma_lap_time.push_back (current_time);
      if (m_best_lap_time == NO_TIME)
        m_best_lap_time = previous_lap_time ();
      else
        {
          m_lap_delta = previous_lap_time () - m_best_lap_time;
          if (m_lap_delta < 0.0)
            m_best_lap_time = previous_lap_time ();
        }
    }
}

void Timing_Info::Car_Timing::update_sector_data (double current_time, size_t sector)
{
  if (m_sector > 0)
    ma_sector_time.push_back (current_time);

  m_last_sector = m_sector;
  m_sector = sector;

  if (m_last_sector > 0)
    {
      const size_t index = m_last_sector - 1;
      assert (index < m_sectors);
      double& best = ma_best_sector_time [index];
      if (best == NO_TIME)
        best = ma_sector_time.back ()
          - (ma_sector_time.size () > 1 
             ? *(ma_sector_time.end () - 2)
             : 0);
      else
        {
          ma_sector_delta [index] = previous_sector_time () - best;
          if (ma_sector_delta [index] < 0.0)
            best = previous_sector_time ();
        }
    }
}

double Timing_Info::Car_Timing::lap_time () const
{
  if (m_finished)
    return NO_TIME;
  return m_current_time - (ma_lap_time.size () == 0 ? 0.0 : ma_lap_time.back ());
}

double Timing_Info::Car_Timing::previous_lap_time () const
{
  switch (ma_lap_time.size ())
    {
    case 0:
      return NO_TIME;
    case 1:
      return ma_lap_time.back ();
    default:
      return ma_lap_time.back () - *(ma_lap_time.end () - 2);
    }
}

double Timing_Info::Car_Timing::sector_time () const
{
  if (m_finished)
    return NO_TIME;
  return m_current_time - (ma_sector_time.size () == 0 ? 0.0 : ma_sector_time.back ());
}

double Timing_Info::Car_Timing::best_sector_time () const
{
  // Return the best time on the current sector.
  return (m_sector == 0) ? NO_TIME : ma_best_sector_time [m_sector - 1];
}

double Timing_Info::Car_Timing::previous_sector_time () const
{
  // Return the time spent in the most recently completed sector.
  switch (ma_sector_time.size ())
    {
    case 0:
      return NO_TIME;
    case 1:
      return ma_sector_time.back ();
    default:
      return ma_sector_time.back () - *(ma_sector_time.end () - 2);
    }
}

double Timing_Info::Car_Timing::previous_sector_time_difference () const
{
  return (m_last_sector == 0) ? NO_TIME : ma_sector_delta [m_last_sector - 1];
}
