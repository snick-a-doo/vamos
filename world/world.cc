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

#include "world.h"
#include "driver.h"
#include "timing-info.h"

#include "../body/car.h"
#include "../geometry/three-vector.h"
#include "../track/strip-track.h"

#include <cassert>
#include <limits>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_World;
using namespace Vamos_Track;

double constexpr slipstream_time_constant{0.7};

//-----------------------------------------------------------------------------
Car_Info::Car_Info(std::shared_ptr<Car> car, std::unique_ptr<Driver> driver)
    : car{car},
      driver{std::move(driver)}
{
}

void Car_Info::reset()
{
    road_index = 0;
    segment_index = 0;
    car->reset();
}

void Car_Info::propagate(double time_step, double total_time,
                                Three_Vector const& track_position,
                                Three_Vector const& pointer_position)
{
    m_record.emplace_back(total_time, track_position,
                          car->chassis().position(), car->chassis().orientation());
    m_pointer_position = pointer_position;
    driver->propagate(time_step);
    car->propagate(time_step);
}

const Three_Vector& Car_Info::track_position() const
{
    if (m_record.empty())
        return null_v;
    return m_record.back().m_track_position;
}

//-----------------------------------------------------------------------------
World::World(Vamos_Track::Strip_Track& track, Atmosphere const& atmosphere)
    : m_track{track},
      m_atmosphere{atmosphere}
{
}

void World::start(bool qualify, size_t laps_or_minutes)
{
    mp_timing = std::make_unique<Timing_Info>(m_cars.size(), m_track.timing_lines(),
                                              !qualify && m_cars.size() > 1);
    if (qualify)
    {
        mp_timing->set_qualifying();
        mp_timing->set_time_limit(laps_or_minutes);
    }
    else
        mp_timing->set_lap_limit(laps_or_minutes);
}

Three_Vector rotation_term(Three_Matrix const& I, Three_Vector const& r, Three_Vector const& n)
{
    return (invert(I) * r.cross(n)).cross(r);
}

Three_Vector impulse(Three_Vector const& r1, double m1, Three_Matrix const& I1,
                     Three_Vector const& r2, double m2, Three_Matrix const& I2,
                     Three_Vector const& v, double restitution, double friction,
                     Three_Vector const& normal)
{
    return -normal * (1.0 + restitution) * v.dot(normal)
               / (normal.dot(normal) * (1.0 / m1 + 1.0 / m2)
                  + (rotation_term(I1, r1, normal) + rotation_term(I2, r2, normal)).dot(normal))
           + friction * (v.project(normal) - v);
}

Three_Vector impulse(Three_Vector const& r, Three_Vector const& v, double m,
                     Three_Matrix const& I, double restitution, double friction,
                     Three_Vector const& normal)
{
    return -normal * (1.0 + restitution) * v.dot(normal)
               / (normal.dot(normal) / m + rotation_term(I, r, normal).dot(normal))
           + friction * (v.project(normal) - v);
}

void World::propagate_cars(double time_step)
{
    for (auto& info : m_cars)
    {
        info.propagate(time_step, mp_timing->elapsed_time(),
                       m_track.track_coordinates(info.car->front_position(), info.road_index,
                                                 info.segment_index),
                       m_track.track_coordinates(info.car->target_position(), info.road_index,
                                                 info.segment_index));
        interact(info.car.get(), info.road_index, info.segment_index);

        // Handle air resistance.
        auto slipstream{1.0};
        if (m_cars_interact)
        {
            for (auto& other : m_cars)
            {
                if (other.car == info.car)
                    continue;
                collide(&info, &other);
                slipstream = std::min(slipstream, air_density_factor(info, other));
            }
        }
        info.car->wind(m_atmosphere.velocity, m_atmosphere.density * slipstream);
    }
}

double World::air_density_factor(Car_Info const& car1, Car_Info const& car2)
{
    if (car1.road_index != car2.road_index)
        return 1.0;

    auto const& p1{car1.track_position()};
    auto const& p2{car2.track_position()};
    auto const& road{m_track.get_road(car1.road_index)};
    if (road.distance(p1.x, p2.x) > 0.0)
        return 1.0;

    // Go through car2's history starting with the most recent event to find out how long
    // ago car2 was at car1's position. Calculate the reduction in air density as a
    // function of that time and distance across the track.
    const auto now{mp_timing->elapsed_time()};
    for (size_t i = car2.m_record.size(); i > 0; i--)
    {
        const auto arg{(now - car2.m_record[i - 1].m_time) / slipstream_time_constant};
        // If we're more than 5 time constants behind the density factor would be at least
        // 1-exp(-5) ~ 0.993. Not far enough away from 1.0 to worry about.
        if (arg > 5.0)
            return 1.0;

        auto const& p2{car2.m_record[i - 1].m_track_position};
        if (road.distance(p1.x, p2.x) > 0.0)
        {
            auto longi{std::exp(-arg)};
            auto trans{longi * std::max(1.0 - std::abs(p2.y - p1.y) / car2.car->width(), 0.0)};
            return 1.0 - longi * trans;
        }
    }
    return 1.0;
}

void World::interact(Car* car, size_t road_index, size_t segment_index)
{
    for (auto const& p : car->chassis().particles())
    {
        auto const& pos{car->chassis().contact_position(*p)};
        auto bump_parameter{car->distance_traveled() + p->position().x};
        auto info{m_track.test_for_contact(pos, bump_parameter, road_index, segment_index)};
        auto const& velocity{car->chassis().particle_velocity(*p)};
        if (info.contact)
        {
            auto j{impulse(
                    car->chassis().moment(pos), velocity, car->chassis().mass(),
                    car->chassis().inertia(),
                    p->material().restitution_factor() * info.material.restitution_factor(),
                    p->material().friction_factor() * info.material.friction_factor(), info.normal)};
            car->chassis().contact(*p, j, velocity, info.depth, info.normal, info.material);
            auto v_perp{velocity.project(info.normal)};
            auto v_par{velocity - v_perp};
            m_interaction_info.emplace_back(car, p->material().composition(),
                                            info.material.composition(),
                                            v_par.magnitude(), v_perp.magnitude());
        }
    }

    // Check for contact with track objects.
    for (auto const& object : m_track.objects())
    {
        auto info{car->collision(object.position, null_v, true)};
        if (!info.contact)
            continue;

        auto velocity{
            car->chassis().point_velocity(car->chassis().transform_in(object.position))};
        auto j{impulse(car->chassis().moment(object.position), velocity,
                       car->chassis().mass(), car->chassis().inertia(),
                       object.material.restitution_factor() * info.material.restitution_factor(),
                       object.material.friction_factor() * info.material.friction_factor(),
                       info.normal)};
        car->chassis().temporary_contact(object.position, j, velocity, info.depth, info.normal,
                                         info.material);
        auto v_perp{velocity.project(info.normal)};
        auto v_par{velocity - v_perp};
        m_interaction_info.emplace_back(car, object.material.composition(),
                                        info.material.composition(),
                                        v_par.magnitude(), v_perp.magnitude());
    }
}

void World::collide(Car_Info* car1_info, Car_Info* car2_info)
{
    auto car1{car1_info->car};
    auto car2{car2_info->car};
    assert(car1 != car2);
    auto delta_r{car1->chassis().cm_position() - car2->chassis().cm_position()};

    // Ignore cars that are too far away to make contact.
    if (delta_r.magnitude() > 1.5 * car2->length())
        return;

    auto delta_v{car1->chassis().cm_velocity() - car2->chassis().cm_velocity()};
    // Handle collisions between the contact points of car 1 and the
    // crash box of car 2.
    for (auto const& p : car1->chassis().particles())
    {
        auto const& pos{car1->chassis().contact_position(*p)};
        auto info{car2->collision(pos, car1->chassis().particle_velocity(*p))};
        if (info.contact)
        {
            auto velocity{car1->chassis().particle_velocity(*p)
                          - car2->chassis().particle_velocity(*p)};
            auto j{impulse(
                    car1->chassis().moment(pos), car1->chassis().mass(),
                    car1->chassis().inertia(), car2->chassis().moment(pos),
                    car2->chassis().mass(), car2->chassis().inertia(), delta_v,
                    p->material().restitution_factor() * p->material().restitution_factor(),
                    p->material().friction_factor() * p->material().friction_factor(),
                    info.normal)};
            car1->chassis().contact(*p, j, delta_v, info.depth, info.normal, info.material);
            car2->chassis().temporary_contact(car1->chassis().contact_position(*p), -j, -delta_v,
                                              info.depth, -info.normal, info.material);
            auto v_perp{velocity.project(info.normal)};
            auto v_par{velocity - v_perp};
            m_interaction_info.emplace_back(car1.get(), info.material.composition(),
                                           info.material.composition(),
                                           v_par.magnitude(), v_perp.magnitude());
        }
    }
}

void World::reset()
{
    auto* ccar{controlled_car()};
    if (!ccar)
        return;

    auto car{ccar->car};
    car->reset();
    place_car(car.get(), m_track.reset_position(car->chassis().position(),
                                                ccar->road_index, ccar->segment_index),
              m_track.get_road(ccar->road_index));
}

void World::restart()
{
    mp_timing->reset();
    if (controlled_car())
        controlled_car()->reset();
}

void World::set_gravity(double g)
{
    m_gravity = Three_Vector{0.0, 0.0, -std::abs(g)};
    for (auto& car : m_cars)
    {
        if (!car.car)
            continue;
        car.car->chassis().set_gravity(m_gravity);
        car.driver->set_gravity(m_gravity);
    }
}

void World::place_car(Car* car, Three_Vector const& track_pos, Road const& road)
{
    const auto& segment{road.segment_at(track_pos.x)};
    car->chassis().reset(0.0);
    // Orient the car to be level with the track.
    {
        Three_Matrix orientation{1.0};
        auto along{track_pos.x - segment.start_distance()};
        auto angle{segment.angle(along)};
        orientation.rotate({0.0, 0.0, angle});
        orientation.rotate({-segment.banking().angle(along), 0.0, 0.0});
        auto up{road.elevation().normal(track_pos.x)};
        orientation.rotate({0.0, atan2(up.x, up.y), 0.0});
        car->chassis().set_orientation(orientation);
    }
    // Raise the car to the requested height above the track.
    auto gap{std::numeric_limits<double>::max()};
    for (auto p : car->chassis().particles())
    {
        auto pos{car->chassis().transform_out(p->contact_position())};
        gap = std::min(gap, pos.z - segment.world_elevation(pos));
    }
    // Move the car to its initial x-y position.
    car->set_front_position(road.position(track_pos.x, track_pos.y));
    car->chassis().translate({0.0, 0.0, track_pos.z - gap});
}

void World::add_car(std::shared_ptr<Car> car, std::unique_ptr<Driver> driver)
{
    driver->set_cars(&m_cars);
    if (driver->is_interactive())
        mo_controlled_car_index = m_cars.size();
    car->chassis().set_gravity(m_gravity);
    place_car(car.get(), car->chassis().position(), m_track.get_road(0));
    m_cars.emplace_back(car, std::move(driver));
}

Car_Info* World::controlled_car()
{
    return mo_controlled_car_index ? &m_cars[*mo_controlled_car_index] : nullptr;
}

void World::write_results(const std::string& file) const
{
    const auto* p_fastest{mp_timing->fastest_lap_timing()};
    std::ofstream os(file);
    os << m_track.track_file() << std::endl
       << (p_fastest ? p_fastest->laps_complete() : 0) << std::endl
       << mp_timing->total_laps() << std::endl
       << (p_fastest ? p_fastest->best_lap_time() : 0) << std::endl;

    for (auto const& timing : mp_timing->running_order())
    {
        const auto& info{m_cars[timing->grid_position() - 1]};
        os << info.car->car_file() << '\t' << info.car->name() << '\t'
           << (info.driver->is_interactive() ? "interactive" : "robot") << '\t'
           << timing->laps_complete() << '\t' << timing->best_lap_time() << std::endl;
    }
}
