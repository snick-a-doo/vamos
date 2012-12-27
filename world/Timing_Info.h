//  Timing_Info.h - timing information and running order for all cars.
//
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

#ifndef _TIMING_INFO_H_
#define _TIMING_INFO_H_

#include <cstddef>
#include <vector>
#include <map>

namespace Vamos_World
{
  /// Lap times, sector times, running order, and intervals.
  class Timing_Info
  {
  public:
    /// Initialize the timing information.
    /// @param n_cars The number of cars added to the world.
    /// @param n_sectors The number of timing sectors per lap. Must be > 0. 
    /// @param n_laps The number of laps in the session. If 0 laps-to-go
    ///               information is not available. 
    Timing_Info (size_t n_cars, size_t n_sectors, size_t n_laps);

    /// Reset the global timer and the cars' times.
    void reset ();

    /// Update the total time and the timing for the car with the specified index.
    void update (double current_time, size_t index, double distance, size_t sector);

    /// Time since the start of the session. Zero before the session
    /// starts. Updated continuously.
    double total_time () const { return m_total_time; }
    /// Return the total number of laps specified at construction. May be
    /// zero. Fixed. 
    size_t total_laps () const { return m_laps; }
    /// The number of cars added. Updated with 'add_cars()'
    size_t total_cars () const { return ma_car_timing.size (); }

    class Car_Timing
    {
    public:
      Car_Timing (size_t position, size_t sectors, size_t laps);

      void update (double current_time, double distance, size_t sector, bool new_sector);
      void set_position (size_t position, double interval);

      size_t position () const { return m_position; }
      /// Return the time interval to the car ahead. -1 is returned for the
      /// leader, all other intervals are non-negative. Updated each time a car
      /// completes a sector.
      double interval () const { return m_interval; }
      /// The distance traveled on the current lap. Resets to zero each lap at the
      /// finish line. May be > 0 at the start of the session. Updated
      /// continuously. 
      double lap_distance () const { return m_distance; }

      /// The (incomplete) lap the car is currently on. It's 0 before the start
      /// of the session, 1 as soon as the timer starts. Updated each lap.
      size_t current_lap () const { return m_lap; }
      /// Time spent on the current lap so far. Updated continuously.
      double lap_time () const;
      /// Time taken to complete the previous lap. Updated each lap.
      double previous_lap_time () const;
      /// The shortest previous lap time so far. Updated each lap.
      double best_lap_time () const { return m_best_lap_time; }
      /// (previous lap time) - (best lap time). Calculated before best lap time
      /// is updated. Updated each lap.
      double lap_time_difference () const { return m_lap_delta; }

      /// The (incomplete) sector the car is currently on. It's 0 before the start
      /// of the session. It's 1 as soon as the timer starts. Updated each time a
      /// car completes a sector.
      size_t current_sector () const { return m_sector; }
      /// The number of the last sector left. 0 until the 1st sector is
      /// complete. Updated each time a car completes a sector.
      size_t previous_sector () const { return m_last_sector; }
      /// Time spent in the current sector so far. Updated continuously.
      double sector_time () const;
      /// The shortest time on the current sector so far. Updated each sector.
      double best_sector_time () const;
      /// Time taken to complete the previous sector. Updated each sector.
      double previous_sector_time () const;
      /// (previous sector time) - (best previous sector time). Calculated before
      /// best sector time is updated. Updated each sector.
      double previous_sector_time_difference () const;

    private:
      bool is_start_of_lap (size_t sector) const;
      void update_lap_data (double current_time);
      void update_sector_data (double current_time, size_t sector);

      size_t m_position;
      double m_current_time;
      double m_distance;
      double m_interval;
      size_t m_sectors;
      size_t m_sector;
      size_t m_last_sector;
      size_t m_lap;
      std::vector <double> ma_lap_time;
      double m_best_lap_time;
      std::vector <double> ma_best_sector_time;
      double m_lap_delta;
      std::vector <double> ma_sector_delta;
      std::vector <double> ma_sector_time;
    };

    const Car_Timing& timing_at_index (size_t index) const 
    { return ma_car_timing [index]; }
    const Car_Timing& timing_at_position (size_t position) const
    { return timing_at_index (index_at_position (position)); }
    const size_t index_at_position (size_t position) const
    { return ma_index_at_position [position - 1]; }

    static const double NO_TIME; 

  private:
    bool is_new_sector (size_t index, size_t sector) const;
    void update_position (double current_time, size_t index, size_t sector);

    size_t m_sectors;
    size_t m_laps;
    double m_total_time;

    std::vector <Car_Timing> ma_car_timing;
    std::vector <size_t> ma_sector_position;
    std::vector <double> ma_sector_time;
    std::vector <size_t> ma_index_at_position;
  };
}

#endif // not _TIMING_INFO_H_
