#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Timing_Info
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Timing_Info.h"

#include <vector>
#include <cassert>

using namespace Vamos_World;

struct Vamos_World::Car_Information
{
};

struct Timing_Fixture
{
  Timing_Fixture (size_t n_sectors, size_t n_laps, size_t n_cars) 
    : timing (n_sectors, n_laps) 
  {
    for (size_t i = 0; i < n_cars; i++)
      {
        Car_Information* car = new Car_Information;
        cars.push_back (car);
        timing.add_car (car);
      }
  }

  ~Timing_Fixture ()
  {
    for (std::vector <Car_Information*>::iterator it = cars.begin ();
         it != cars.end ();
         it++)
      delete *it;
  }

  const Car_Information* car (size_t n) const 
  {
    assert ((n > 0) && (n <= cars.size ()));
    return cars [n-1];
  }

  Timing_Info timing;
  std::vector <Car_Information*> cars;
};

BOOST_AUTO_TEST_CASE (initial_state)
{
  Timing_Fixture f (3, 4, 3);
  BOOST_CHECK_EQUAL (f.timing.total_time (), 0.0);
  BOOST_CHECK_EQUAL (f.timing.total_laps (), 4);
  BOOST_CHECK_EQUAL (f.timing.total_cars (), 3);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (1)), 0);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (2)), 0);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (3)), 0);
  BOOST_CHECK_EQUAL (f.timing.car_at_position (0), (Car_Information*)0);
  BOOST_CHECK_EQUAL (f.timing.car_at_position (1), f.car (1));
  BOOST_CHECK_EQUAL (f.timing.car_at_position (2), f.car (2));
  BOOST_CHECK_EQUAL (f.timing.car_at_position (3), f.car (3));
  BOOST_CHECK_EQUAL (f.timing.car_at_position (4), (Car_Information*)0);
  BOOST_CHECK_EQUAL (f.timing.interval (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.interval (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.interval (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (1)), 0);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (2)), 0);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (3)), 0);
  BOOST_CHECK_EQUAL (f.timing.lap_time (f.car (1)), 0);
  BOOST_CHECK_EQUAL (f.timing.lap_time (f.car (2)), 0);
  BOOST_CHECK_EQUAL (f.timing.lap_time (f.car (3)), 0);
  BOOST_CHECK_EQUAL (f.timing.previous_lap_time (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_lap_time (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_lap_time (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_lap_time (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_lap_time (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_lap_time (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.lap_time_difference (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.lap_time_difference (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.lap_time_difference (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.current_sector (f.car (1)), 0);
  BOOST_CHECK_EQUAL (f.timing.current_sector (f.car (2)), 0);
  BOOST_CHECK_EQUAL (f.timing.current_sector (f.car (3)), 0);
  BOOST_CHECK_EQUAL (f.timing.previous_sector (f.car (1)), 0);
  BOOST_CHECK_EQUAL (f.timing.previous_sector (f.car (2)), 0);
  BOOST_CHECK_EQUAL (f.timing.previous_sector (f.car (3)), 0);
  BOOST_CHECK_EQUAL (f.timing.sector_time (f.car (1)), 0);
  BOOST_CHECK_EQUAL (f.timing.sector_time (f.car (2)), 0);
  BOOST_CHECK_EQUAL (f.timing.sector_time (f.car (3)), 0);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_sector_time (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_sector_time (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_sector_time (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time_difference (f.car (1)),
                     Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time_difference (f.car (2)),
                     Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time_difference (f.car (3)),
                     Timing_Info::NO_TIME);
}

BOOST_AUTO_TEST_CASE (second_sector)
{
  Timing_Fixture f (3, 4, 3);
  // Car 2 finishes the first sector first.
  f.timing.update (10.0, f.car (1), 100.0, 1);
  f.timing.update (21.0, f.car (2), 250.0, 1);
  f.timing.update (32.0, f.car (3), 300.0, 1);
  f.timing.update (40.0, f.car (1), 400.0, 1);
  f.timing.update (51.0, f.car (2), 600.0, 2);
  f.timing.update (62.0, f.car (3), 500.0, 1);
  f.timing.update (70.0, f.car (1), 600.0, 2);
  f.timing.update (81.0, f.car (2), 900.0, 2);
  f.timing.update (92.0, f.car (3), 750.0, 2);
  BOOST_CHECK_EQUAL (f.timing.total_time (), 92.0);
  BOOST_CHECK_EQUAL (f.timing.car_at_position (1), f.car (2));
  BOOST_CHECK_EQUAL (f.timing.car_at_position (2), f.car (1));
  BOOST_CHECK_EQUAL (f.timing.car_at_position (3), f.car (3));
  BOOST_CHECK_EQUAL (f.timing.interval (f.car (1)), 19);
  BOOST_CHECK_EQUAL (f.timing.interval (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.interval (f.car (3)), 22);
  BOOST_CHECK_EQUAL (f.timing.lap_distance (f.car (1)), 600);
  BOOST_CHECK_EQUAL (f.timing.lap_distance (f.car (2)), 900);
  BOOST_CHECK_EQUAL (f.timing.lap_distance (f.car (3)), 750);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (1)), 1);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (2)), 1);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (3)), 1);
  BOOST_CHECK_EQUAL (f.timing.lap_time (f.car (1)), 92);
  BOOST_CHECK_EQUAL (f.timing.lap_time (f.car (2)), 92);
  BOOST_CHECK_EQUAL (f.timing.lap_time (f.car (3)), 92);
  BOOST_CHECK_EQUAL (f.timing.previous_lap_time (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_lap_time (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_lap_time (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_lap_time (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_lap_time (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_lap_time (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.lap_time_difference (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.lap_time_difference (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.lap_time_difference (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.current_sector (f.car (1)), 2);
  BOOST_CHECK_EQUAL (f.timing.current_sector (f.car (2)), 2);
  BOOST_CHECK_EQUAL (f.timing.current_sector (f.car (3)), 2);
  BOOST_CHECK_EQUAL (f.timing.previous_sector (f.car (1)), 1);
  BOOST_CHECK_EQUAL (f.timing.previous_sector (f.car (2)), 1);
  BOOST_CHECK_EQUAL (f.timing.previous_sector (f.car (3)), 1);
  BOOST_CHECK_EQUAL (f.timing.sector_time (f.car (1)), 22);
  BOOST_CHECK_EQUAL (f.timing.sector_time (f.car (2)), 41);
  BOOST_CHECK_EQUAL (f.timing.sector_time (f.car (3)), 0);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time (f.car (1)), 70);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time (f.car (2)), 51);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time (f.car (3)), 92);
  BOOST_CHECK_EQUAL (f.timing.best_sector_time (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_sector_time (f.car (2)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.best_sector_time (f.car (3)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time_difference (f.car (1)),
                     Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time_difference (f.car (2)),
                     Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time_difference (f.car (3)),
                     Timing_Info::NO_TIME);
}

BOOST_AUTO_TEST_CASE (third_lap)
{
  Timing_Fixture f (2, 2, 1);
  f.timing.update (5.0, f.car (1), 100.0, 1);
  f.timing.update (11.0, f.car (1), 200.0, 2);
  f.timing.update (25.0, f.car (1), 2.0, 1);
  f.timing.update (38.0, f.car (1), 201.0, 2);
  f.timing.update (51.0, f.car (1), 3.0, 1);
  f.timing.update (57.0, f.car (1), 100.0, 1);

  BOOST_CHECK_EQUAL (f.timing.total_time (), 57.0);
  BOOST_CHECK_EQUAL (f.timing.car_at_position (1), f.car (1));
  BOOST_CHECK_EQUAL (f.timing.interval (f.car (1)), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.timing.lap_distance (f.car (1)), 100);
  BOOST_CHECK_EQUAL (f.timing.current_lap (f.car (1)), 3);
  BOOST_CHECK_EQUAL (f.timing.lap_time (f.car (1)), 6);
  BOOST_CHECK_EQUAL (f.timing.previous_lap_time (f.car (1)), 26);
  BOOST_CHECK_EQUAL (f.timing.best_lap_time (f.car (1)), 25);
  BOOST_CHECK_EQUAL (f.timing.lap_time_difference (f.car (1)), 1);
  BOOST_CHECK_EQUAL (f.timing.current_sector (f.car (1)), 1);
  BOOST_CHECK_EQUAL (f.timing.previous_sector (f.car (1)), 2);
  // sector 1
  BOOST_CHECK_EQUAL (f.timing.sector_time (f.car (1)), 6);
  BOOST_CHECK_EQUAL (f.timing.best_sector_time (f.car (1)), 11);
  // sector 2
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time (f.car (1)), 13);
  BOOST_CHECK_EQUAL (f.timing.previous_sector_time_difference (f.car (1)), -1);
}
