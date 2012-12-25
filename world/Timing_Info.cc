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

Timing_Info::Timing_Info (size_t n_sectors, size_t n_laps)
  : m_sectors (n_sectors),
    m_laps (n_laps),
    m_total_time (0.0)
{
  // Reserve space if we know the total number of sectors. n_laps may be 0 in
  // which case these vectors will grow indefinitely.
  ma_sector_position.reserve (n_sectors * n_laps);
  ma_sector_time.reserve (n_sectors * n_laps);
}

void Timing_Info::add_car (const Car_Information* p_car)
{
  ma_cars.push_back (p_car);

  ma_sector [p_car] = 0;
  ma_last_sector [p_car] = 0;
  ma_lap [p_car] = 0;
  //! use same vector for sector, lap times, lap count
  maa_lap_time [p_car].reserve (m_laps);
  maa_sector_time [p_car].reserve (m_sectors * m_laps);
  ma_best_lap_time [p_car] = NO_TIME;
  maa_best_sector_time [p_car].resize (m_sectors);
  maa_sector_delta [p_car].resize (m_sectors);
  for (size_t sector = 0; sector < m_sectors; sector++)
    {
      maa_best_sector_time [p_car][sector] = NO_TIME;
      maa_sector_delta [p_car][sector] = NO_TIME;
    }
  ma_lap_delta [p_car] = NO_TIME;
  ma_distance [p_car] = 0.0;
  ma_position [p_car] = ma_cars.size ();
  ma_interval [p_car] = NO_TIME;
}

void Timing_Info::reset ()
{
}

void Timing_Info::update (double current_time,
                          const Car_Information* p_car,
                          double distance,
                          size_t sector)
{
  if (is_new_sector (p_car, sector))
    {
      if (is_start_of_lap (sector))
        update_lap_data (current_time, p_car);

      update_sector_data (current_time, p_car, sector);

      const size_t nth_sector = m_sectors * (ma_lap [p_car] - 1) + sector;
      const size_t old_position = ma_position [p_car];
      assert (ma_cars [old_position - 1] = p_car);
      size_t new_position;
      if (nth_sector > ma_sector_position.size ())
        {
          new_position = 1;
          ma_sector_position.push_back (new_position);
          ma_sector_time.push_back (current_time);
          ma_interval [p_car] = NO_TIME;
        }
      else
        {
          new_position = ++ma_sector_position [nth_sector - 1];
          ma_interval [p_car] = current_time - ma_sector_time [nth_sector - 1];
          ma_sector_time [nth_sector - 1] = current_time;
        }

      std::swap (ma_position [p_car], ma_position [ma_cars [new_position - 1]]);
      std::swap (ma_cars [old_position - 1], ma_cars [new_position - 1]);
      assert (ma_cars [new_position - 1] = p_car);
    }

  m_total_time = current_time;
  ma_distance [p_car] = distance;
}

bool Timing_Info::is_new_sector (const Car_Information* p_car, size_t sector)
{
  return (sector == (ma_sector [p_car] % m_sectors) + 1);
}

bool Timing_Info::is_start_of_lap (size_t sector)
{
  return (sector == 1);
}

void Timing_Info::update_lap_data (double current_time, const Car_Information* p_car)
{
  ma_lap [p_car]++;
  if (ma_sector [p_car] > 0)
    {
      maa_lap_time [p_car].push_back (current_time);
      if (ma_best_lap_time [p_car] == NO_TIME)
        ma_best_lap_time [p_car] = previous_lap_time (p_car);
      else
        {
          ma_lap_delta [p_car] = previous_lap_time (p_car) - ma_best_lap_time [p_car];
          if (ma_lap_delta [p_car] < 0.0)
            ma_best_lap_time [p_car] = previous_lap_time (p_car);
        }
    }
}

void Timing_Info::update_sector_data (double current_time,
                                      const Car_Information* p_car,
                                      size_t sector)
{
  if (ma_sector [p_car] > 0)
    maa_sector_time [p_car].push_back (current_time);

  ma_last_sector [p_car] = ma_sector [p_car];
  ma_sector [p_car] = sector;

  if (ma_last_sector [p_car] > 0)
    {
      const size_t index = ma_last_sector [p_car] - 1;
      assert (index < m_sectors);
      double& best = maa_best_sector_time [p_car][index];
      if (best == NO_TIME)
        best = maa_sector_time [p_car].back ()
          - (maa_sector_time [p_car].size () > 1 
             ? *(maa_sector_time [p_car].end () - 2)
             : 0);
      else
        {
          maa_sector_delta [p_car][index] = previous_sector_time (p_car) - best;
          if (maa_sector_delta [p_car][index] < 0.0)
            best = previous_sector_time (p_car);
        }
    }
}

double Timing_Info::total_time () const
{
  return m_total_time;
}

size_t Timing_Info::total_laps () const 
{ 
  return m_laps;
}

size_t Timing_Info::total_cars () const
{
  return ma_cars.size ();
}

const Car_Information* Timing_Info::car_at_position (size_t position) const
{
  if ((position < 1) || (position > ma_cars.size ()))
    return 0;
  return ma_cars [position - 1];
}

double Timing_Info::interval (const Car_Information* p_car) const
{
  return ma_interval.find (p_car)->second;
}

double Timing_Info::lap_distance (const Car_Information* p_car) const
{
  return ma_distance.find (p_car)->second;
}

size_t Timing_Info::current_lap (const Car_Information* p_car) const
{
  return ma_lap.find (p_car)->second;
}

double Timing_Info::lap_time (const Car_Information* p_car) const
{
  const std::vector <double>& a_time = maa_lap_time.find (p_car)->second;
  return m_total_time - (a_time.size () == 0 ? 0.0 : a_time.back ());
}

double Timing_Info::previous_lap_time (const Car_Information* p_car) const
{
  const std::vector <double>& a_time = maa_lap_time.find (p_car)->second;
  switch (a_time.size ())
    {
    case 0:
      return NO_TIME;
    case 1:
      return a_time.back ();
    default:
      return a_time.back () - *(a_time.end () - 2);
    }
}

double Timing_Info::best_lap_time (const Car_Information* p_car) const
{
  return ma_best_lap_time.find (p_car)->second;
}

double Timing_Info::lap_time_difference (const Car_Information* p_car) const
{
  return ma_lap_delta.find (p_car)->second;
}

size_t Timing_Info::current_sector (const Car_Information* p_car) const
{
  return ma_sector.find (p_car)->second;
}

size_t Timing_Info::previous_sector (const Car_Information* p_car) const
{
  return ma_last_sector.find (p_car)->second;
}

double Timing_Info::sector_time (const Car_Information* p_car) const
{
  const std::vector <double>& a_time = maa_sector_time.find (p_car)->second;
  return m_total_time - (a_time.size () == 0 ? 0.0 : a_time.back ());
}

double Timing_Info::best_sector_time (const Car_Information* p_car) const
{
  // Return the best time on the current sector.
  if (current_sector (p_car) == 0)
    return NO_TIME;
  return maa_best_sector_time.find (p_car)->second [current_sector (p_car) - 1];
}

double Timing_Info::previous_sector_time (const Car_Information* p_car) const
{
  // Return the time spent in the most recently completed sector.
  const std::vector <double>& a_time = maa_sector_time.find (p_car)->second;
  switch (a_time.size ())
    {
    case 0:
      return NO_TIME;
    case 1:
      return a_time.back ();
    default:
      return a_time.back () - *(a_time.end () - 2);
    }
}

double Timing_Info::previous_sector_time_difference (const Car_Information* p_car) const
{
  if (previous_sector (p_car) == 0)
    return NO_TIME;
  return maa_sector_delta.find (p_car)->second [previous_sector (p_car) - 1];
}
