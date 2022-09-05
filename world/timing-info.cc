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

#include "../geometry/numeric.h"

#include "timing-info.h"
#include "world.h"

#include <algorithm>

using namespace Vamos_World;

auto constexpr n_countdown_start{6};

Timing_Info::Timing_Info(size_t n_cars, size_t n_sectors, bool do_start_sequence)
    : m_sectors{n_sectors},
      m_start_delay{Vamos_Geometry::random_in_range(0.0, 4.0)},
      m_state{do_start_sequence ? State::starting : State::running}
{
    assert(n_sectors > 0);
    for (size_t i = 0; i < n_cars; ++i)
    {
        m_car_timing.emplace_back(std::make_shared<Car_Timing>(i + 1, n_sectors));
        m_running_order.push_back(m_car_timing.back());
    }
    if (!m_car_timing.empty())
        mp_fastest = m_car_timing.front();
}

void Timing_Info::reset()
{
    m_elapsed_time = 0.0;
    m_fastest_lap = no_time;
    m_running_order.clear();
    for (auto car : m_car_timing)
    {
        car->reset();
        m_running_order.push_back(car);
    }
    if (!m_car_timing.empty())
        mp_fastest = m_car_timing.front();
}

void Timing_Info::update(double current_time, size_t index, double distance, size_t sector)
{
    assert(index < m_car_timing.size());
    assert(sector <= m_sectors);

    switch (m_state)
    {
    case State::starting:
    {
        auto to_go{n_countdown_start - (current_time - m_start_time)};
        m_countdown = std::max(static_cast<int>(to_go + 1.0), 1);
        if (to_go < -m_start_delay)
        {
            m_countdown = 0;
            m_start_time = current_time;
            m_state = State::running;
        }
        break;
    }
    case State::running:
    case State::finished:
    {
        m_elapsed_time = current_time - m_start_time;

        auto p_car{m_car_timing[index]};
        const auto new_sector{sector == (p_car->current_sector() % m_sectors) + 1};
        const auto new_lap{new_sector && sector == 1};
        const auto already_finished{p_car->is_finished()};

        p_car->update(m_elapsed_time, distance, sector, new_sector);

        const auto laps_done{
            m_lap_limit > 0 && m_car_timing[index]->current_lap() > m_lap_limit};
        const auto time_done{m_time_limit > 0.0 && m_elapsed_time > m_time_limit};

        // A car's race is done when
        // 1. it completes all the laps
        // 2. it completes any lap after any car has completed all laps
        // 3. it completes any lap after time has expired
        if (laps_done || (new_lap && (time_done || m_state == State::finished)))
            p_car->set_finished();

        if (new_sector)
            update_position(p_car, m_elapsed_time, sector, already_finished);

        // Go to the FINISHED state when the first car has completed all of the
        // laps or time runs out. Timing must continue for the other cars, so we
        // keep m_state_time.
        if (m_state == State::running && (laps_done || time_done))
            m_state = State::finished;
        break;
    }
    }
}

double Timing_Info::time_remaining() const
{
    return m_time_limit == 0.0 ? 0.0 : std::max(m_time_limit - m_elapsed_time, 0.0);
}

void Timing_Info::update_position(std::shared_ptr<Car_Timing> p_car,
                                  double current_time, size_t sector, bool finished)
{
    assert(sector > 0 && sector <= m_sectors);
    if (p_car->previous_sector() == 0 || finished)
        return;

    // Lap and sector are 1-based. Subtract 1 from each. The first sector entered is
    // sector 2 of lap 1, so subtract one more to make the initial index 0.
    const auto sector_index{m_sectors * (p_car->current_lap() - 1) + sector - 2};

    // Start with first place and increment as necessary.
    auto new_position{m_running_order.begin()};
    auto interval{no_time};
    if (m_qualifying)
    {
        if (sector != 1)
            return;
        while (p_car->best_lap_time() > (*new_position)->best_lap_time())
            ++new_position;
    }
    else if (sector_index == m_sector_entry_count.size())
    {
        // p_car is the first to enter this sector on this lap. It's in the lead.
        m_sector_entry_count.push_back(1);
        m_sector_times.push_back(current_time);
    }
    else
    {
        std::advance(new_position, m_sector_entry_count[sector_index]);
        ++m_sector_entry_count[sector_index];
        interval = current_time - m_sector_times[sector_index];
        m_sector_times[sector_index] = current_time;
    }
    p_car->set_interval(interval);

    // If this car has lost positions it will have been pushed down the running
    // order by the cars that have already reached this sector. Thus, old_position
    // is at or after new_position.
    auto old_position{std::find(new_position, m_running_order.end(), p_car)};
    if (new_position != old_position)
    {
        m_running_order.insert(new_position, *old_position);
        m_running_order.erase(old_position);
    }

    // Check for fastest lap.
    auto best{p_car->best_lap_time()};
    if (best < m_fastest_lap)
    {
        mp_fastest = p_car;
        m_fastest_lap = best;
    }
}

//----------------------------------------------------------------------------------------
Car_Timing::Car_Timing(size_t position, size_t sectors)
    : m_grid_position{position},
      m_sectors{sectors}
{
    m_best_sector_times.resize(m_sectors);
    m_sector_deltas.resize(m_sectors);
    for (size_t sector = 0; sector < m_sectors; ++sector)
    {
        m_best_sector_times[sector] = Timing_Info::no_time;
        m_sector_deltas[sector] = Timing_Info::no_time;
    }
}

void Car_Timing::reset()
{
    m_interval = Timing_Info::no_time;
    m_sector = 0;
    m_last_sector = 0;
    m_lap = 0;
    m_best_lap_time = Timing_Info::no_time;
    m_lap_delta = Timing_Info::no_time;
    m_finished = false;

    m_lap_times.clear();
    m_sector_times.clear();
    for (size_t sector = 0; sector < m_sectors; ++sector)
    {
        m_best_sector_times[sector] = Timing_Info::no_time;
        m_sector_deltas[sector] = Timing_Info::no_time;
    }
}

void Car_Timing::update(double current_time, double distance, size_t sector, bool new_sector)
{
    m_current_time = current_time;
    m_distance = distance;
    if (m_finished || !new_sector)
        return;
    if (sector == 1)
        update_lap_data(current_time);
    update_sector_data(current_time, sector);
}

void Car_Timing::update_lap_data(double current_time)
{
    ++m_lap;
    if (m_sector == 0)
        return;

    m_lap_times.push_back(current_time);
    if (m_best_lap_time != Timing_Info::no_time)
        m_lap_delta = previous_lap_time() - m_best_lap_time;
    if (m_lap_delta == Timing_Info::no_time || m_lap_delta < 0.0)
        m_best_lap_time = previous_lap_time();
}

void Car_Timing::update_sector_data(double current_time, size_t sector)
{
    if (m_sector > 0)
        m_sector_times.push_back(current_time);

    m_last_sector = m_sector;
    m_sector = sector;
    if (m_last_sector == 0)
        return;

    auto index{m_last_sector - 1};
    assert(index < m_sectors);
    auto& best{m_best_sector_times[index]};
    if (best == Timing_Info::no_time)
    {
        best = m_sector_times.back()
            - (m_sector_times.size() > 1 ? *(m_sector_times.end() - 2) : 0);
        return;
    }
    m_sector_deltas[index] = previous_sector_time() - best;
    if (m_sector_deltas[index] < 0.0)
        best = previous_sector_time();
}

double Car_Timing::lap_time() const
{
    return m_finished ? Timing_Info::no_time
        : m_current_time - (m_lap_times.empty() ? 0.0 : m_lap_times.back());
}

double Car_Timing::previous_lap_time() const
{
    switch (m_lap_times.size())
    {
    case 0:
        return Timing_Info::no_time;
    case 1:
        return m_lap_times.back();
    default:
        return m_lap_times.back() - *(m_lap_times.end() - 2);
    }
}

double Car_Timing::sector_time() const
{
    return m_finished
        ? Timing_Info::no_time
        : m_current_time - (m_sector_times.empty() ? 0.0 : m_sector_times.back());
}

double Car_Timing::best_sector_time() const
{
    return m_sector == 0 ? Timing_Info::no_time : m_best_sector_times[m_sector - 1];
}

double Car_Timing::previous_sector_time() const
{
    return m_sector_times.empty() ? Timing_Info::no_time
        : m_sector_times.size() == 1 ? m_sector_times.back()
        : m_sector_times.back() - *(m_sector_times.end() - 2);
}

double Car_Timing::previous_sector_time_difference() const
{
    return m_last_sector == 0 ? Timing_Info::no_time : m_sector_deltas[m_last_sector - 1];
}
