//  World.h - handles interactions between a car and its environment.
//
//  Vamos Automotive Simulator
//  Copyright (C) 2001--2004 Sam Varner
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

#ifndef _WORLD_H_
#define _WORLD_H_

#include "../body/Car.h"
#include "../geometry/Circular_Buffer.h"
#include "../geometry/Material.h"
#include "../geometry/Three_Vector.h"
#include "../track/Strip_Track.h"
#include "Atmosphere.h"
#include "../track/Road_Segment.h"

#include <vector>

namespace Vamos_World
{
  class Driver;

  class Times
  {
  public:
    Times ();
    void start (double start_time) { m_start = start_time; }
    void update (double current_time) { m_current = current_time; }
    void reset ();

    double time () const { return m_current - m_start; }
    void finalize ();
    double previous () const { return m_previous; }
    double best () const { return m_best; }
    double difference () const { return m_difference; }
    double total () const { return m_current; }

  private:
    double m_start;
    double m_current;
    double m_previous;
    double m_best;
    double m_difference;
  };

  class Timing_Info
  {
  public:
    Timing_Info ();

    void reset ();
    void update (double current_time, double distance, int sector);

    int get_sector () const { return m_sector; }
    int get_previous_sector () const { return m_previous_sector; }
    double get_distance () const { return m_distance; }

    double get_total_time () const { return m_lap_times.total (); }
    double get_lap_time () const { return m_lap_times.time (); }
    double get_previous_lap_time () const { return m_lap_times.previous (); }
    double get_best_lap_time () const { return m_lap_times.best (); }
    double get_lap_time_difference () const { return m_lap_times.difference (); }

    double get_sector_time () const
    { return ma_sector_times [m_sector].time (); }
    double get_previous_sector_time () const
    { return ma_sector_times [m_previous_sector].previous (); }
    double get_previous_sector_time_difference () const 
    { return ma_sector_times [m_previous_sector].difference (); }
    double get_best_sector_time (int sector) const
    { return (size_t (sector) < ma_sector_times.size ()) 
        ? ma_sector_times [sector].best () : 0.0; }

  private:
    Times m_lap_times;
    std::vector <Times> ma_sector_times;

    int m_sector;
    int m_previous_sector;
    double m_distance;

    void update_sector_info (double current_time, int sector);
    void update_times (double current_time, int sector);
  };

  struct Car_Information
  {
    Car_Information (Vamos_Body::Car* car_in, Driver* driver_in);

    Timing_Info timing;
    size_t road_index;
    size_t segment_index;
    Vamos_Body::Car* car;
    Driver* driver;

    void reset ();
    void propagate (double time_step, 
                    const Vamos_Geometry::Three_Vector& track_position);

    struct Record
    {
      Record () {};
      Record (double time, 
              Vamos_Body::Car* car,
              const Vamos_Geometry::Three_Vector& track_position);

      double m_time;
      Vamos_Geometry::Three_Vector m_track_position;
      Vamos_Geometry::Three_Vector m_position;
      Vamos_Geometry::Three_Matrix m_orientation;
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

  class World
  {
  public:
    World (Vamos_Track::Strip_Track* track, Atmosphere* atmosphere);
    virtual ~World ();

    void interact (Vamos_Body::Car* car, 
                   size_t road_index,
                   size_t segment_index);
    void collide (Car_Information* car1_info, Car_Information* car2_info);
    void gravity (double g);
    double get_gravity () const { return m_gravity; }
    virtual void add_car (Vamos_Body::Car* car,
                          Driver* driver,
                          const Vamos_Track::Road& road);
    virtual void set_focused_car (size_t car_index);
    virtual void set_controlled_car (size_t car_index);
    void focus_other_car (int delta);
    void cars_can_interact (bool can) { m_cars_can_interact = can; }
    void propagate_cars (double time_step);
    size_t number_of_cars () const { return m_cars.size (); }

  protected:
    std::vector <Car_Information> m_cars;
    Vamos_Track::Strip_Track* mp_track;
    Atmosphere* mp_atmosphere;

    double m_gravity;

    std::vector <Interaction_Info> m_interaction_info;

    void reset ();
    void restart ();

    Car_Information* focused_car ();
    Car_Information* controlled_car ();

  private:
    void place_car (Vamos_Body::Car* car, 
                    const Vamos_Geometry::Three_Vector& pos,
                    const Vamos_Track::Road& Road);

    size_t m_focused_car_index;
    bool m_cars_can_interact;
    bool m_has_controlled_car;
    size_t m_controlled_car_index;

    double slipstream_air_density_factor (Car_Information& car1, Car_Information& car2);
  };
}

#endif // not _WORLD_H_
