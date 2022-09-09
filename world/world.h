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

#ifndef VAMES_WORLD_WORLD_H_INCLUDED
#define VAMES_WORLD_WORLD_H_INCLUDED

#include "../geometry/circular-buffer.h"
#include "../geometry/material.h"
#include "../geometry/three-matrix.h"
#include "../geometry/three-vector.h"

#include "timing-info.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Vamos_Body
{
class Car;
}
namespace Vamos_Track
{
class Road;
class Strip_Track;
}
namespace Vamos_World
{
class Atmosphere;
class Driver;
class Track;

/// Atmospheric properties
struct Atmosphere
{
    double density; ///< Gas density in kg/m³.
    Vamos_Geometry::Three_Vector velocity; ///< Wind speed
};

/// Information about each car in the world.
struct Car_Info
{
    Car_Info(Vamos_Body::Car* car_in, Driver* driver_in);

    void reset();
    void propagate(double time_step, double total_time,
                   Vamos_Geometry::Three_Vector const& track_position,
                   Vamos_Geometry::Three_Vector const& pointer_position);
    Vamos_Geometry::Three_Vector const& track_position() const;

    size_t road_index{0};
    size_t segment_index{0};
    Vamos_Body::Car* car{nullptr};
    Driver* driver{nullptr};
    Vamos_Geometry::Three_Vector m_pointer_position;

    /// Recent history for slipstream and replay.
    struct Record
    {
        double m_time{0.0};
        Vamos_Geometry::Three_Vector m_track_position;
        Vamos_Geometry::Three_Vector m_position;
        Vamos_Geometry::Three_Matrix m_orientation{1.0};
    };
    Vamos_Geometry::Circular_Buffer<Record, 5000> m_record;
};

// Information about a collision.
struct Interaction_Info
{
    using Mat_Type = Vamos_Geometry::Material::Composition;
    Vamos_Body::Car* car;
    Mat_Type car_material;
    Mat_Type track_material;
    double parallel_speed; ///< Speed parallel to the surface.
    double perpendicular_speed; ///< Speed normal to the surface.
};

/// The world handles interaction between cars and tracks, air, gravity, etc.
class World
{
public:
    World(Vamos_Track::Strip_Track& track, Atmosphere const& atmosphere);
    virtual ~World() = default;

    /// Put a car and its driver in the world.
    virtual void add_car(Vamos_Body::Car& car, Driver& driver);
    /// Start the session.
    /// @param qualifying Qualifying if true, race if false.
    /// @param laps_or_minutes The time limit, if qualifying, else the number of laps.
    virtual void start(bool qualifying, size_t laps_or_minutes);

    /// @param g Acceleration due to gravity always downward regardless of sign.
    void set_gravity(double g);
    /// Set whether or not cars interact with each other. Defaults to true.
    void cars_interact(bool interact) { m_cars_interact = interact; }
    /// Write the current race state.
    void write_results(std::string const& file) const;

protected:
    void reset();
    void restart();
    void propagate_cars(double time_step);
    Car_Info* controlled_car();

    Vamos_Track::Strip_Track& m_track;
    Atmosphere m_atmosphere;
    std::vector<Car_Info> m_cars;
    /// The times for all cars.
    std::unique_ptr<Timing_Info> mp_timing;
    std::vector<Interaction_Info> m_interaction_info;

private:
    /// Position the car above a point on the track.
    /// @param pos Distance along the track (x), to the left of center (y), and above the
    /// surface (z).
    void place_car(Vamos_Body::Car* car, const Vamos_Geometry::Three_Vector& track_pos,
                   const Vamos_Track::Road& Road);
    /// @return The fraction of air density experienced by @p behind due to the slipstream
    /// of @p in_front.
    double air_density_factor(Car_Info const& behind, Car_Info const& in_front);

    void interact(Vamos_Body::Car* car, size_t road_index, size_t segment_index);
    void collide(Car_Info* car1_info, Car_Info* car2_info);

    /// Cars can collide and create a slipstream if true. Otherwise, they pass through
    /// each other.
    bool m_cars_interact{true};
    /// The index of the car under interactive control or nullopt if no such car.
    std::optional<size_t> mo_controlled_car_index;
    Vamos_Geometry::Three_Vector m_gravity{0.0, 0.0, -9.8};
};
} // namespace Vamos_World

#endif // VAMES_WORLD_WORLD_H_INCLUDED
