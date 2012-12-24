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
  class Car_Information;

  /// Lap times, sector times, running order, and intervals.
  class Timing_Info
  {
  public:
    /// Initialize the timing information.
    /// @param n_sectors The number of timing sectors per lap. Must be > 0. 
    /// @param n_laps The number of laps in the session. If 0 laps-to-go
    ///               information is not available. 
    Timing_Info (size_t n_sectors, size_t n_laps);
    /// Initialize timing for a car. Cars are added in the order they line up on
    /// the grid.
    void add_car (const Car_Information* p_car);
    /// Reset the global timer and the cars' times.
    void reset ();

    void update (double current_time, 
                 const Car_Information* p_car,
                 double distance, 
                 size_t sector);

    /// Time since the start of the session. Zero before the session
    /// starts. Updated continuously.
    double total_time () const;
    /// Return the total number of laps specified at construction. May be
    /// zero. Fixed. 
    size_t total_laps () const;
    /// The number of cars added. Updated with 'add_cars()'
    size_t total_cars () const;

    /// Return the pointer for the car at the specified (1-based) position. Zero
    /// is returned for position < 1 or position > total_cars(). Updated each
    /// time a car completes a sector.
    const Car_Information* car_at_position (size_t position) const;

    /// Return the time interval to the car ahead. -1 is returned for the
    /// leader, all other intervals are non-negative. Updated each time a car
    /// completes a sector.
    double interval (const Car_Information* p_car) const;
    /// The distance traveled on the current lap. Resets to zero each lap at the
    /// finish line. May be > 0 at the start of the session. Updated
    /// continuously. 
    double lap_distance (const Car_Information* p_car) const;

    /// The (incomplete) lap the car is currently on. It's 0 before the start
    /// of the session, 1 as soon as the timer starts. Updated each lap.
    size_t current_lap (const Car_Information* p_car) const;
    /// Time spent on the current lap so far. Updated continuously.
    double lap_time (const Car_Information* p_car) const;
    /// Time taken to complete the previous lap. Updated each lap.
    double previous_lap_time (const Car_Information* p_car) const;
    /// The shortest previous lap time so far. Updated each lap.
    double best_lap_time (const Car_Information* p_car) const;
    /// (previous lap time) - (best lap time). Calculated before best lap time
    /// is updated. Updated each lap.
    double lap_time_difference (const Car_Information* p_car) const;

    /// The (incomplete) sector the car is currently on. It's 0 before the start
    /// of the session. It's 1 as soon as the timer starts. Updated each time a
    /// car completes a sector.
    size_t current_sector (const Car_Information* p_car) const;
    /// The number of the last sector left. 0 until the 1st sector is
    /// complete. Updated each time a car completes a sector.
    size_t previous_sector (const Car_Information* p_car) const;
    /// Time spent in the current sector so far. Updated continuously.
    double sector_time (const Car_Information* p_car) const;
    /// The shortest time on the current sector so far. Updated each sector.
    double best_sector_time (const Car_Information* p_car) const;
    /// Time taken to complete the previous sector. Updated each sector.
    double previous_sector_time (const Car_Information* p_car) const;
    /// (previous sector time) - (best previous sector time). Calculated before
    /// best sector time is updated. Updated each sector.
    double previous_sector_time_difference (const Car_Information* p_car) const;

    static const double NO_TIME; 

  private:
    bool is_new_sector (const Car_Information* p_car, size_t sector);
    bool is_start_of_lap (size_t sector);
    void update_lap_data (double current_time, const Car_Information* p_car);
    void update_sector_data (double current_time, 
                             const Car_Information* p_car,
                             size_t sector);

    size_t m_sectors;
    size_t m_laps;
    double m_total_time;

    std::vector <const Car_Information*> ma_cars;

    std::vector <size_t> ma_sector_position;
    std::vector <double> ma_sector_time;

    std::map <const Car_Information*, size_t> ma_sector;
    std::map <const Car_Information*, size_t> ma_last_sector;
    std::map <const Car_Information*, size_t> ma_lap;
    std::map <const Car_Information*, std::vector <double> > maa_lap_time;
    std::map <const Car_Information*, double> ma_best_lap_time;
    std::map <const Car_Information*, std::vector <double> > maa_best_sector_time;
    std::map <const Car_Information*, double> ma_lap_delta;
    std::map <const Car_Information*, std::vector <double> > maa_sector_delta;
    std::map <const Car_Information*, double> ma_distance;
    std::map <const Car_Information*, size_t> ma_position;
    std::map <const Car_Information*, double> ma_interval;
    std::map <const Car_Information*, std::vector <double> > maa_sector_time;
  };
}

#endif // not _TIMING_INFO_H_
