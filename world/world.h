//  World.h - handles interactions between a car and its environment.
//
//  Copyright (C) 2001--2004 Sam Varner
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

#ifndef _WORLD_H_
#define _WORLD_H_

#include "../geometry/circular-buffer.h"
#include "../geometry/material.h"
#include "../geometry/three-matrix.h"
#include "../geometry/three-vector.h"

#include "timing-info.h"

#include <string>
#include <vector>

namespace Vamos_Body
{
  class Car;
}

namespace Vamos_Geometry
{
  class Three_Vector;
}

namespace Vamos_Track
{
  class Road;
  class Strip_Track;
}

namespace Vamos_World
{
  class Driver;

  struct Car_Information
  {
    Car_Information (Vamos_Body::Car* car_in, Driver* driver_in);

    void reset ();
    void propagate (double time_step, 
                    double total_time,
                    const Vamos_Geometry::Three_Vector& track_position,
                    const Vamos_Geometry::Three_Vector& pointer_position);
    const Vamos_Geometry::Three_Vector& track_position () const;

    size_t road_index;
    size_t segment_index;
    Vamos_Body::Car* car;
    Driver* driver;
    Vamos_Geometry::Three_Vector m_pointer_position;

    struct Record
    {
      Record () {};
      Record (double time, 
              Vamos_Body::Car* car,
              const Vamos_Geometry::Three_Vector& track_position);

      double m_time;
      Vamos_Geometry::Three_Vector m_track_position;
      Vamos_Geometry::Three_Vector m_position;
        Vamos_Geometry::Three_Matrix m_orientation{1.0};
    };

    Vamos_Geometry::Circular_Buffer <Record> m_record;
  };

  struct Interaction_Info
  {
    typedef Vamos_Geometry::Material::Material_Type Mat_Type;

    Interaction_Info (Vamos_Body::Car* car_in, 
                      Mat_Type car_material_type, 
                      Mat_Type track_material_type,
                      double par_speed, double perp_speed)
      : car (car_in),
        car_material (car_material_type),
        track_material (track_material_type),
        parallel_speed (par_speed),
        perpendicular_speed (perp_speed)
    {}

    Vamos_Body::Car* car;
    Mat_Type car_material;
    Mat_Type track_material;
    double parallel_speed;
    double perpendicular_speed;
  };

  class Atmosphere;
  class Track;

  class World
  {
  public:
    World (Vamos_Track::Strip_Track& track, Atmosphere& atmosphere);
    virtual ~World ();

    void interact (Vamos_Body::Car* car, 
                   size_t road_index,
                   size_t segment_index);
    void collide (Car_Information* car1_info, Car_Information* car2_info);
    void gravity (double g);
    double get_gravity () const { return m_gravity; }
    virtual void add_car (Vamos_Body::Car& car, Driver& driver);
    virtual void set_focused_car (size_t car_index);
    void focus_other_car (int delta);
    void cars_can_interact (bool can) { m_cars_can_interact = can; }
    void propagate_cars (double time_step);
    size_t number_of_cars () const { return m_cars.size (); }
    void write_results (const std::string& file) const;

  protected:
    Vamos_Track::Strip_Track& m_track;
    Atmosphere& m_atmosphere;
    double m_gravity;

    std::vector <Car_Information> m_cars;
    /// The times for all cars.
    Timing_Info* mp_timing;
    std::vector <Interaction_Info> m_interaction_info;
    size_t m_focused_car_index;

    virtual void start (bool qualifying, size_t laps_or_minutes);
    void reset ();
    void restart ();

    Car_Information* focused_car ();
    Car_Information* controlled_car ();

  private:
    void place_car (Vamos_Body::Car* car, 
                    const Vamos_Geometry::Three_Vector& pos,
                    const Vamos_Track::Road& Road);
    void set_controlled_car (size_t car_index);

    bool m_cars_can_interact;
    bool m_has_controlled_car;
    size_t m_controlled_car_index;

    double slipstream_air_density_factor (Car_Information& car1, Car_Information& car2);
  };
}

#endif // not _WORLD_H_
