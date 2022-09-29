//  Copyright (C) 2001-2022 Sam Varner
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

#ifndef VAMOS_WORLD_TIMING_INFO_H_INCLUDED
#define VAMOS_WORLD_TIMING_INFO_H_INCLUDED

#include <cstddef>
#include <limits>
#include <list>
#include <memory>
#include <vector>

namespace Vamos_World
{
class Car_Timing;
/// Lap times, sector times, running order, and intervals.
class Timing_Info
{
    enum class State
    {
        starting, ///< The session has not yet begun.
        running,  ///< The session has started.
        finished  ///< The session is over, but cars may be on track.
    };

public:
    using Running_Order = std::list<std::shared_ptr<Car_Timing>>;

    /// Initialize the timing information.
    /// @param n_cars The number of cars added to the world.
    /// @param n_sectors The number of timing sectors per lap. Must be > 0.
    /// @param do_start_sequence If true, do the countdown to the start of the
    ///                          session. Otherwise start immediately.
    Timing_Info(size_t n_cars, size_t n_sectors, bool do_start_sequence);
    /// Set a limit on the number of laps.
    void set_lap_limit(size_t laps) { m_lap_limit = laps; }
    /// Set a time limit. Race ends on completion of the lap in progress.
    void set_time_limit(double minutes) { m_time_limit = 60.0 * minutes; }
    /// Indicate a qualifying session.
    void set_qualifying() { m_qualifying = true; }
    /// Reset the global timer and the cars' times.
    void reset();
    /// Update the total time and the timing for the car with the specified index.
    void update(double current_time, size_t index, double distance, size_t sector);

    /// @return True if this is a qualifying session.
    bool is_qualifying() const { return m_qualifying; }
    /// @return The number of counts until the start of the session. Intervals
    /// are seconds except for 1 to 0 which has an extra random delay. The race
    /// starts when the count reaches 0 and stays that way through the session.
    int countdown() const { return m_countdown; }
    /// @return The total number of laps specified at construction. Zero if no limit.
    size_t total_laps() const { return m_lap_limit; }
    /// @return The time since the start of the session. Updated continuously.
    double elapsed_time() const { return m_elapsed_time; }
    /// @return The time to the end of a timed race. Zero if no time limit.
    double time_remaining() const;
    /// @return True if the race is over.
    bool is_finished() const { return m_state == State::finished; }
    /// @return Timing info in the order of the positions on track.
    const Running_Order& running_order() const { return m_running_order; }
    /// @return Timing info for the car with a particular starting position.
    const Car_Timing& timing_at_index(size_t index) const { return *m_car_timing[index]; }
    /// @return Timing info for the car with the fastest lap, or nullptr if no laps complete.
    const Car_Timing* fastest_lap_timing() const { return mp_fastest.get(); }
    /// The value for a time that hasn't been measured. All actual times compare as less.
    static constexpr double no_time{std::numeric_limits<double>::max()};

private:
    /// Update the running order and time gaps each time a car enters a new sector.
    void update_position(std::shared_ptr<Car_Timing> p_car,
                         double current_time, size_t sector, bool finished);

    size_t const m_sectors; ///< The number of timed divisions of the track.
    /// The random delay after the last light has been on for a second and before they all
    /// go out to signal the start of the race.
    double const m_start_delay;
    State m_state{State::starting}; ///< The stage of the race.

    size_t m_lap_limit{0}; ///< Race ends after this many laps.
    double m_time_limit{0.0}; ///< Race ends after this much time.
    bool m_qualifying{false}; ///< True for a qualifying session.
    int m_countdown{0}; ///< Counts to

    double m_elapsed_time{0.0}; ///< Time since the start of the race.
    double m_start_time{0.0}; ///< Time when the current state started.
    /// Timing information for each car
    std::vector<std::shared_ptr<Car_Timing>> m_car_timing;
    /// Elements of this vector count the number of cars that have entered a given sector
    /// of a given lap. Used to calculate the position of a car when it enters a sector.
    std::vector<size_t> m_sector_entry_count;
    /// Elements of this vector give the time that a car last entered a given sector of a
    /// given lap. Used to calculate the time gap between positions.
    std::vector<double> m_sector_times;
    /// A list of pointers to car timings in the order of the cars on track.
    Running_Order m_running_order;
    /// Pointer to info for the car with the fastest lap or nullptr if no laps complete.
    std::shared_ptr<Car_Timing> mp_fastest;
    double m_fastest_lap{no_time}; ///< The shortest lap time so far.
};

/// Timing for a single car.
class Car_Timing
{
public:
    /// Create timing info for a car.
    /// @param position The car's 1-based starting position.
    /// @param sectors The number of timing sectors on the track.
    Car_Timing(size_t position, size_t sectors);

    /// Clear the timing info and set it to its initial state.
    void reset();
    /// Called frequently to update the car's info.
    /// @param current_time Elapsed time from the start of the race.
    /// @param distance Distance from the finish line on this lap.
    void update(double current_time, double distance, size_t sector, bool new_sector);
    /// Called when the car finishes the race.
    void set_finished() { m_finished = true; }
    /// Called to give the gap to the car ahead.
    void set_interval(double interval) { m_interval = interval; }

    /// @:return The (incomplete) lap the car is currently on. It's 0 before the start of
    /// the session, 1 as soon as the timer starts. Updated each lap.
    size_t current_lap() const { return m_lap; }
    /// @return The number of completed laps.
    size_t laps_complete() const { return m_lap == 0 ? 0 : m_lap - 1; }
    /// @return The car's position at the start.
    size_t grid_position() const { return m_grid_position; }
    /// @return The time interval to the car ahead. no_time is returned for the leader.
    /// Intervals are non-negative. Updated each time a car completes a sector.
    double interval() const { return m_interval; }
    /// @return The distance traveled on the current lap.
    double lap_distance() const { return m_distance; }
    /// @return Time spent on the current lap so far.
    double current_lap_time() const;
    /// @return Elapsed time at the completion of a lap.
    /// @param lap A lap number from 1 to # of completed laps.
    double lap_time(size_t lap) const;
    /// Time taken to complete the previous lap.
    double previous_lap_time() const;
    /// The shortest previous lap time so far. Updated each lap.
    double best_lap_time() const { return m_best_lap_time; }
    /// (previous lap time) - (best lap time). Calculated before best lap time
    /// is updated. Updated each lap.
    double lap_time_difference() const { return m_lap_delta; }
    /// The (incomplete) sector the car is currently on. It's 0 before the start of the
    /// session. It's 1 as soon as the timer starts. Updated each time a car completes a
    /// sector.
    size_t current_sector() const { return m_sector; }
    /// The number of the last sector left. 0 until the 1st sector is
    /// complete. Updated each time a car completes a sector.
    size_t previous_sector() const { return m_last_sector; }
    /// Time spent in the current sector so far. Updated continuously.
    double sector_time() const;
    /// The shortest time on the current sector so far. Updated each sector.
    double best_sector_time() const;
    /// Time taken to complete the previous sector. Updated each sector.
    double previous_sector_time() const;
    /// (previous sector time) - (best previous sector time). Calculated before best
    /// sector time is updated. Updated each sector.
    double previous_sector_time_difference() const;
    /// True if the car has completed the race.
    bool is_finished() const { return m_finished; }

private:
    /// Do lap variable updates for update().
    void update_lap_data(double current_time);
    /// Do sector variable updates for update().
    void update_sector_data(double current_time, size_t sector);

    // Some methods are called each time a frame is drawn. Values that don't change
    // frequently are stored in member variables rather than calculated on demand.

    size_t const m_grid_position{0}; ///< Starting position.
    size_t const m_sectors{0}; ///< The number of timing sectors on the track.
    double m_current_time{0.0}; ///< Elapsed time from the start of the race.
    double m_distance{0.0}; ///< Distance from the finish line on this lap.
    double m_interval{Timing_Info::no_time}; ///< Time gap to the car ahead.
    bool m_finished{false}; ///< The car's race is complete if true.

    size_t m_lap{0}; ///< Current 1-based lap number.
    std::vector<double> m_lap_times; ///< Time taken to complete each lap.
    double m_best_lap_time{Timing_Info::no_time}; ///< Shortest lap time.
    double m_lap_delta{Timing_Info::no_time}; ///< Difference between last and best lap time.

    size_t m_sector{0}; ///< Current 1-based sector number..
    size_t m_last_sector{0}; ///< Previous sector number.
    /// Time to complete each sector from the start of the race.
    std::vector<double> m_sector_times;
    std::vector<double> m_best_sector_times; ///< Best time so far on each sector.
    /// Difference between last and best time for each sector.
    std::vector<double> m_sector_deltas;
};
} // namespace Vamos_World

#endif // VAMOS_WORLD_TIMING_INFO_H_INCLUDED
