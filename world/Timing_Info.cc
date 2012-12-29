//  Vamos Automotive Simulator
//  Copyright (C) 2001--2012 Sam Varner
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

#include "Timing_Info.h"
#include "World.h"

#include <algorithm>
#include <limits>

using namespace Vamos_World;

const double Timing_Info::NO_TIME = std::numeric_limits <double>::min ();

Timing_Info::Timing_Info (size_t n_cars, size_t n_sectors, size_t n_laps)
  : m_sectors (n_sectors),
    m_laps (n_laps),
    m_total_time (0.0)
{
  assert (n_sectors > 0);

  // Reserve space if we know the total number of sectors. n_laps may be 0 in
  // which case these vectors will grow indefinitely.
  ma_sector_position.reserve (n_sectors * n_laps);
  ma_sector_time.reserve (n_sectors * n_laps);
  for (size_t i = 0; i < n_cars; i++)
    {
      Car_Timing* p_car = new Car_Timing (i + 1, n_sectors, n_laps);
      ma_car_timing.push_back (p_car);
      ma_running_order.push_back (p_car);
    }
}

Timing_Info::~Timing_Info ()
{
  for (size_t i = 0; i < ma_car_timing.size (); i++)
    delete ma_car_timing [i];
}

void Timing_Info::reset ()
{
  //!todo
}

void Timing_Info::update (double current_time,
                          size_t index,
                          double distance,
                          size_t sector)
{
  assert (index < ma_car_timing.size ());
  assert (sector <= m_sectors);

  m_total_time = current_time;
  const bool new_sector = is_new_sector (index, sector);
  ma_car_timing [index]->update (current_time, distance, sector, new_sector);
  if (new_sector)
    update_position (current_time, index, sector);
}

void Timing_Info::update_position (double current_time, size_t index, size_t sector)
{
  assert ((sector > 0) && (sector <= m_sectors));
  Car_Timing* p_car = ma_car_timing [index];
  const size_t nth_sector = m_sectors * (p_car->current_lap () - 1) + sector;

  Timing_Info::Running_Order::iterator new_position = ma_running_order.begin ();
  double interval;
  if (nth_sector > ma_sector_position.size ())
    {
      interval = NO_TIME;
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
}

bool Timing_Info::is_new_sector (size_t index, size_t sector) const
{
  const size_t current = ma_car_timing [index]->current_sector ();
  // Do the % before + because sector is 1-based.
  return (sector == (current % m_sectors) + 1);
}

Timing_Info::Car_Timing::Car_Timing (size_t position, size_t sectors, size_t laps)
  : m_grid_position (position),
    m_current_time (0.0),
    m_distance (0.0),
    m_interval (NO_TIME),
    m_sectors (sectors),
    m_sector (0),
    m_last_sector (0),
    m_lap (0),
    m_best_lap_time (NO_TIME),
    m_lap_delta (NO_TIME)
{
  //! use same vector for sector, lap times, lap count
  ma_lap_time.reserve (laps);
  ma_sector_time.reserve (m_sectors * laps);
  ma_best_sector_time.resize (m_sectors);
  ma_sector_delta.resize (m_sectors);
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
  if (new_sector)
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
