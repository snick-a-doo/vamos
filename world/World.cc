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

#include "World.h"
#include "Driver.h"
#include "../body/Wheel.h"

#include <cassert>
#include <limits>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_World;
using namespace Vamos_Track;

const double slipstream_time_constant = 0.7;

//-----------------------------------------------------------------------------
Times::Times () :
  m_start (0.0),
  m_current (0.0),
  m_previous (0.0),
  m_best (0.0),
  m_difference (0.0)
{
}

void 
Times::finalize ()
{
  const double elapsed = time ();

  if (m_best != 0.0)
    m_difference = elapsed - m_best;

  if ((elapsed < m_best) || (m_best == 0.0))
    m_best = elapsed;

  m_previous = elapsed;
}

void
Times::reset ()
{
  m_start = m_current;
}

//-----------------------------------------------------------------------------

Timing_Info::Timing_Info () :
  m_sector (-1),
  m_previous_sector (-1),
  m_distance (0.0)
{
}

void 
Timing_Info::update_sector_info (double current_time, int sector)
{
  if (sector >= int (ma_sector_times.size ()))
    { // We're entering a sector for the first time.
      ma_sector_times.resize (sector + 1);
    }

  if ((sector != m_sector) && (sector != -1))
    { // We have left a sector.
      if (m_sector != -1)
        {
          ma_sector_times [m_sector].finalize ();
          
          if (sector == 0)
            { // We have finished a lap.
              m_lap_times.finalize ();
              m_lap_times.start (current_time);
            }
        }
      m_previous_sector = m_sector;
      m_sector = sector;
      ma_sector_times [m_sector].start (current_time);
    }
}

void
Timing_Info::update_times (double current_time, int sector)
{
  m_lap_times.update (current_time);
  if (sector != -1)
    {
      assert (sector < int (ma_sector_times.size ()));
      ma_sector_times [sector].update (current_time);
    }
}

void 
Timing_Info::update (double current_time, double distance, int sector)
{
  update_sector_info (current_time, sector);
  update_times (current_time, sector);
  m_distance = distance;
}

void 
Timing_Info::reset ()
{
  m_lap_times.reset ();

  if (m_sector != -1)
    {
      assert (m_sector < int (ma_sector_times.size ()));
      ma_sector_times [m_sector].reset ();
    }
  m_sector = -1;
  m_previous_sector = -1;
  m_distance = 0.0;
}

//-----------------------------------------------------------------------------

Car_Information::Car_Information (Car* car_in, Driver* driver_in) 
 : road_index (0), 
   segment_index (0), 
   car (car_in),
   driver (driver_in),
   m_record (5000)
{
}

//-----------------------------------------------------------------------------
void 
Car_Information::reset ()
{
  timing.reset ();
  road_index = 0;
  segment_index = 0;
  if (driver != 0)
    driver->reset ();
  car->reset ();
}

//-----------------------------------------------------------------------------
void 
Car_Information::propagate (double time_step, const Three_Vector& track_position)
{
  if (driver != 0)
    driver->propagate (time_step);

  car->propagate (time_step);
  m_record.push_back (Record (timing.get_total_time (), car, track_position));
}

//-----------------------------------------------------------------------------
Car_Information::Record::Record (double time, 
                                 Car* car,
                                 const Three_Vector& track_position)
  : m_time (time),
    m_track_position (track_position),
    m_position (car->chassis ().position ()),
    m_orientation (car->chassis ().orientation())
{
}

//-----------------------------------------------------------------------------
World::World (Vamos_Track::Strip_Track* track, Atmosphere* atmosphere) 
  : mp_track (track),
    mp_atmosphere (atmosphere),
    m_gravity (9.8),
    m_focused_car_index (0),
    m_cars_can_interact (true),
    m_has_controlled_car (false),
    m_controlled_car_index (0)
{
}

World::~World ()
{
  delete mp_atmosphere;
  delete mp_track;
  for (std::vector <Car_Information>::iterator it = m_cars.begin ();
       it != m_cars.end ();
       it++)
    {
      delete it->car;
      delete it->driver;
    }
}

inline Three_Vector
rotation_term (const Inertia_Tensor& I,
               const Three_Vector& r,
               const Three_Vector& n)
{
  return (I.inverse () * (r.cross (n))).cross (r);
}

Three_Vector
impulse (const Three_Vector& r1,
         double m1,
         const Inertia_Tensor& I1,
         const Three_Vector& r2,
         double m2,
         const Inertia_Tensor& I2,
         const Three_Vector& v,
         double restitution,
         double friction,
         const Three_Vector& normal)
{
  return -normal * (1.0 + restitution) * v.dot (normal)
    / (normal.dot (normal) * (1.0 / m1 + 1.0 / m2)
       + (rotation_term (I1, r1, normal) 
          + rotation_term (I2, r2, normal)).dot (normal))
    + friction * (v.project (normal) - v);
}

Three_Vector
impulse (const Three_Vector& r,
         const Three_Vector& v,
         double m,
         const Inertia_Tensor& I,
         double restitution,
         double friction,
         const Three_Vector& normal)
{
  return -normal * (1.0 + restitution) * v.dot (normal)
    / (normal.dot (normal) / m + rotation_term (I, r, normal).dot (normal))
    + friction * (v.project (normal) - v);
}

void
World::propagate_cars (double time_step)
{
  for (std::vector <Car_Information>::iterator it = m_cars.begin ();
       it != m_cars.end ();
       it++)
    {
      it->propagate (time_step, mp_track->track_coordinates (it->car->center_position (),
                                                             it->road_index,
                                                             it->segment_index));
      interact (it->car, it->road_index, it->segment_index);

      double air_density_factor = 1.0;
      if (m_cars_can_interact)
        {
          for (std::vector <Car_Information>::iterator other = m_cars.begin ();
               other != m_cars.end ();
               other++)
            {
              if (other == it)
                continue;

              collide (&*it, &*other);
              air_density_factor = std::min (air_density_factor, 
                                             slipstream_air_density_factor (*it, *other));
            }
        }
      
      // Handle air resistance.
      it->car->wind (mp_atmosphere->velocity (), 
                     mp_atmosphere->density () * air_density_factor);
      it->driver->set_air_density_factor (air_density_factor);
    }
}

// Return the fraction of air density at car1 due to the slipstream of car2.
double 
World::slipstream_air_density_factor (Car_Information& car1, Car_Information& car2)
{
  double factor = 1.0;

  if (car1.road_index != car2.road_index)
    return factor;

  Three_Vector p1 = mp_track->track_coordinates (car1.car->center_position (),
                                                 car1.road_index,
                                                 car1.segment_index);
  Three_Vector p2 = mp_track->track_coordinates (car2.car->center_position (),
                                                 car2.road_index,
                                                 car2.segment_index);

  const Vamos_Track::Road& road = mp_track->get_road (car1.road_index);

  if (road.distance (p1.x, p2.x) > 0.0)
    return factor;

  for (size_t i = car2.m_record.size (); i > 0; i--)
    {
      const Three_Vector& p2 = car2.m_record [i - 1].m_track_position;
      if (road.distance (p1.x, p2.x) > 0.0)
        {
          const double now = car2.timing.get_total_time ();
          const double longitudinal 
            = std::exp ((car2.m_record [i - 1].m_time - now)
                        / slipstream_time_constant);
          const double transverse = longitudinal
            * std::max (1.0 - std::abs (p2.y - p1.y) / car2.car->width (),
                        0.0);
          return 1.0 - longitudinal * transverse;
        }
    }

  return factor;
}

void 
World::interact (Car* car, 
                 size_t road_index,
                 size_t segment_index)
{
  size_t i = 0;
  for (std::vector <Particle*>::iterator 
         it = car->chassis ().particles ().begin ();
       it != car->chassis ().particles ().end ();
       it++, i++)
    {
      const Three_Vector& pos = car->chassis ().contact_position (*it);
      double bump_parameter = 
        car->distance_traveled () + (*it)->position ().x;
      const Contact_Info info = mp_track->test_for_contact (pos, 
                                                            bump_parameter, 
                                                            road_index,
                                                            segment_index);

      const Three_Vector& velocity = car->chassis ().velocity (*it);
      if (info.contact)
        {
          Three_Vector j = impulse (car->chassis ().world_moment (pos),
                                    velocity,
                                    car->chassis ().mass (),
                                    car->chassis ().inertia (),
                                    (*it)->material ().restitution_factor ()
                                    * info.material.restitution_factor (),
                                    (*it)->material ().friction_factor ()
                                    * info.material.friction_factor (),
                                    info.normal);

          car->chassis ().contact (*it, 
                                   j, 
                                   velocity,
                                   info.depth, 
                                   info.normal, 
                                   info.material);

          Three_Vector v_perp = velocity.project (info.normal);
          Three_Vector v_par = velocity - v_perp;
          m_interaction_info.
            push_back (Interaction_Info (car,
                                         (*it)->material ().type (), 
                                         info.material.type (),
                                         v_par.magnitude (), 
                                         v_perp.magnitude ()));
        }
    }

  // Check for contact with track objects.
  for (std::vector <Vamos_Track::Track_Object>::const_iterator 
         object = mp_track->objects ().begin ();
       object != mp_track->objects ().end ();
       object++)
    {
      const Contact_Info info = car->collision (object->position, 
                                                Three_Vector (),
                                                true);
      
      if (info.contact)
        {
          Three_Vector velocity = car->chassis ().velocity 
            (car->chassis ().transform_from_world (object->position));
          Three_Vector j = impulse (car->chassis ().world_moment (object->position),
                                    velocity,
                                    car->chassis ().mass (),
                                    car->chassis ().inertia (),
                                    object->material.restitution_factor () 
                                    * info.material.restitution_factor (),
                                    object->material.friction_factor ()
                                    * info.material.friction_factor (),
                                    info.normal);
          
          car->chassis ().temporary_contact 
            (object->position,
             j,
             velocity,
             info.depth,
             info.normal,
             info.material);
          
          Three_Vector v_perp = velocity.project (info.normal);
          Three_Vector v_par = velocity - v_perp;
          m_interaction_info.
            push_back (Interaction_Info (car,
                                         object->material.type (), 
                                         info.material.type (),
                                         v_par.magnitude (), 
                                         v_perp.magnitude ()));
        }
    }
}

void 
World::collide (Car_Information* car1_info, Car_Information* car2_info)
{
  Car* car1 = car1_info->car;
  Car* car2 = car2_info->car;
  assert (car1 != car2);

  const Three_Vector delta_r = car1->chassis ().cm_position () 
    - car2->chassis ().cm_position ();

  // Ignore cars that are too far away to make contact.
  if (delta_r.magnitude () > 1.5 * car2->length ())
    return;

  const Three_Vector delta_v = car1->chassis ().cm_velocity () 
    - car2->chassis ().cm_velocity ();

  // Handle collisions between the contact points of car 1 and the
  // crash box of car 2. 
  for (std::vector <Particle*>::iterator 
         it = car1->chassis ().particles ().begin ();
       it != car1->chassis ().particles ().end ();
       it++)
    {
      const Three_Vector& pos = car1->chassis ().contact_position (*it);
      const Contact_Info info = car2->collision (pos, car1->chassis ().velocity (*it));

      if (info.contact)
        {
          const Three_Vector& velocity = car1->chassis ().velocity (*it)
            - car2->chassis ().velocity (*it);
          Three_Vector j = impulse (car1->chassis ().world_moment (pos),
                                    car1->chassis ().mass (),
                                    car1->chassis ().inertia (),
                                    car2->chassis ().world_moment (pos),
                                    car2->chassis ().mass (),
                                    car2->chassis ().inertia (),
                                    delta_v,
                                    (*it)->material ().restitution_factor ()
                                    * (*it)->material ().restitution_factor (),
                                    (*it)->material ().friction_factor ()
                                    * (*it)->material ().friction_factor (),
                                    info.normal);

          car1->chassis ().contact (*it, 
                                    j,
                                    delta_v,
                                    info.depth,
                                    info.normal, 
                                    info.material);

          car2->chassis ().temporary_contact (car1->chassis ().contact_position (*it),
                                              -j,
                                              -delta_v,
                                              info.depth,
                                              -info.normal, 
                                              info.material);

          Three_Vector v_perp = velocity.project (info.normal);
          Three_Vector v_par = velocity - v_perp;
          m_interaction_info.
            push_back (Interaction_Info (car1,
                                         info.material.type (),
                                         info.material.type (),
                                         v_par.magnitude (), 
                                         v_perp.magnitude ()));
        }
    }
}

// Place the car back on the track at its current position.
void 
World::reset ()
{
  if (!m_has_controlled_car)
    return;

  size_t& segment_index = controlled_car ()->segment_index;
  size_t& road_index = controlled_car ()->road_index;
  Car* car = controlled_car ()->car;
  car->reset ();
  place_car (car,
             mp_track->reset_position (car->chassis ().position (), 
                                       road_index, 
                                       segment_index),
             mp_track->get_road (road_index));
}

// Place the car back on the track at the starting line.
void 
World::restart ()
{
  if (m_has_controlled_car)
    controlled_car ()->reset ();
}

// Set the acceleration due to gravity.  Always downward, regardless
// of sign.
void 
World::gravity (double g)
{
  m_gravity = std::abs (g);
  for (std::vector <Car_Information>::iterator it = m_cars.begin ();
       it != m_cars.end ();
       it++)
    {
      if (it->car != 0)
          it->car->chassis ().gravity (Three_Vector (0.0, 0.0, -m_gravity));
    }
}

void
World::place_car (Car* car, const Three_Vector& track_pos, const Road& road)
{
  const Road_Segment& segment = *road.segment_at (track_pos.x);

  car->chassis ().reset (0.0);

  // Move the car to its initial x-y position.
  car->chassis ().set_position (road.position (track_pos.x, track_pos.y));

  // Orient the car to be level with the track.
  {
    Three_Matrix orientation;
    double along = track_pos.x - segment.start_distance ();
    double angle = segment.angle (along);
    orientation.rotate (Three_Vector (0.0, 0.0, angle));
    orientation.rotate (Three_Vector (-segment.banking ().angle (along), 0.0, 0.0));
    Two_Vector up = road.elevation ().normal (track_pos.x);
    orientation.rotate (Three_Vector (0.0, atan2 (up.x, up.y), 0.0));
    car->chassis ().set_orientation (orientation);
  }

  // Raise the car to the requested height above the track.
  double gap = std::numeric_limits <double>::max ();
  for (std::vector <Particle*>::const_iterator it = car->chassis ().particles ().begin ();
       it != car->chassis ().particles ().end ();
       it++)
    {
      Three_Vector p = car->chassis ().transform_to_world ((*it)->contact_position ());
      gap = std::min (gap, p.z - segment.world_elevation (p));
    }
  car->chassis ().translate (Three_Vector (0.0, 0.0, track_pos.z - gap));
}

void 
World::add_car (Car* car, Driver* driver, const Road& road, bool controlled)
{
  if (driver != 0)
    driver->set_cars (&m_cars);
  car->chassis ().gravity (Three_Vector (0.0, 0.0, -m_gravity));
  m_cars.push_back (Car_Information (car, driver));

  place_car (car, car->chassis ().position (), road);

  set_focused_car (m_cars.size () - 1);
  if (controlled)
    set_controlled_car (m_cars.size () - 1);
}

void 
World::set_focused_car (size_t index)
{
  assert (index < m_cars.size ());
  m_focused_car_index = index;
}

void 
World::focus_other_car (int delta)
{
  set_focused_car ((m_focused_car_index + delta) % m_cars.size ());
}

Car_Information* 
World::focused_car ()
{
  if (m_focused_car_index >= m_cars.size ()) return 0;
  return &m_cars [m_focused_car_index];
}

void 
World::set_controlled_car (size_t index)
{
  assert (index < m_cars.size ());
  m_has_controlled_car = true;
  m_controlled_car_index = index;
}

Car_Information* 
World::controlled_car ()
{
  if (!m_has_controlled_car || (m_controlled_car_index >= m_cars.size ()))
      return 0;
  return &m_cars [m_controlled_car_index];
}
