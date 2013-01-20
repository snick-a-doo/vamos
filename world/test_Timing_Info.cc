#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Timing_Info
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Timing_Info.h"

#include <vector>
#include <cassert>

using namespace Vamos_World;

struct Timing_Fixture
{
  Timing_Fixture (size_t n_cars, size_t n_sectors, size_t n_laps)
    : timing (n_cars, n_sectors, n_laps, false) // Skip the start sequence. 
  {
  }

  const Timing_Info::Car_Timing& car (size_t n) const 
  {
    return timing.timing_at_index (n - 1);
  }

  Timing_Info timing;
};

BOOST_AUTO_TEST_CASE (initial_state)
{
  Timing_Fixture f (3, 3, 4);
  BOOST_CHECK_EQUAL (f.timing.total_time (), 0.0);
  BOOST_CHECK_EQUAL (f.timing.total_laps (), 4);
  BOOST_CHECK_EQUAL (f.car (1).current_lap (), 0);
  BOOST_CHECK_EQUAL (f.car (2).current_lap (), 0);
  BOOST_CHECK_EQUAL (f.car (3).current_lap (), 0);
  BOOST_CHECK_EQUAL (f.car (1).interval (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).interval (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).interval (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).current_lap (), 0);
  BOOST_CHECK_EQUAL (f.car (2).current_lap (), 0);
  BOOST_CHECK_EQUAL (f.car (3).current_lap (), 0);
  BOOST_CHECK_EQUAL (f.car (1).lap_time (), 0);
  BOOST_CHECK_EQUAL (f.car (2).lap_time (), 0);
  BOOST_CHECK_EQUAL (f.car (3).lap_time (), 0);
  BOOST_CHECK_EQUAL (f.car (1).previous_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).previous_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).previous_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).best_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).best_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).best_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).lap_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).lap_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).lap_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).current_sector (), 0);
  BOOST_CHECK_EQUAL (f.car (2).current_sector (), 0);
  BOOST_CHECK_EQUAL (f.car (3).current_sector (), 0);
  BOOST_CHECK_EQUAL (f.car (1).previous_sector (), 0);
  BOOST_CHECK_EQUAL (f.car (2).previous_sector (), 0);
  BOOST_CHECK_EQUAL (f.car (3).previous_sector (), 0);
  BOOST_CHECK_EQUAL (f.car (1).sector_time (), 0);
  BOOST_CHECK_EQUAL (f.car (2).sector_time (), 0);
  BOOST_CHECK_EQUAL (f.car (3).sector_time (), 0);
  BOOST_CHECK_EQUAL (f.car (1).previous_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).previous_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).previous_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).best_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).best_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).best_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).previous_sector_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).previous_sector_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).previous_sector_time_difference (), Timing_Info::NO_TIME);
}

BOOST_AUTO_TEST_CASE (second_sector)
{
  Timing_Fixture f (3, 4, 3);
  // Car 2 finishes the first sector first.
  f.timing.update (10.0, 0, 100.0, 1);
  f.timing.update (21.0, 1, 250.0, 1);
  f.timing.update (32.0, 2, 300.0, 1);
  f.timing.update (40.0, 0, 400.0, 1);
  f.timing.update (51.0, 1, 600.0, 2);
  f.timing.update (62.0, 2, 500.0, 1);
  f.timing.update (70.0, 0, 600.0, 2);
  f.timing.update (81.0, 1, 900.0, 2);
  f.timing.update (92.0, 2, 750.0, 2);
  BOOST_CHECK_EQUAL (f.timing.total_time (), 92.0);
  BOOST_CHECK_EQUAL (f.car (1).interval (), 19);
  BOOST_CHECK_EQUAL (f.car (2).interval (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).interval (), 22);
  BOOST_CHECK_EQUAL (f.car (1).lap_distance (), 600);
  BOOST_CHECK_EQUAL (f.car (2).lap_distance (), 900);
  BOOST_CHECK_EQUAL (f.car (3).lap_distance (), 750);
  BOOST_CHECK_EQUAL (f.car (1).current_lap (), 1);
  BOOST_CHECK_EQUAL (f.car (2).current_lap (), 1);
  BOOST_CHECK_EQUAL (f.car (3).current_lap (), 1);
  BOOST_CHECK_EQUAL (f.car (1).lap_time (), 70);
  BOOST_CHECK_EQUAL (f.car (2).lap_time (), 81);
  BOOST_CHECK_EQUAL (f.car (3).lap_time (), 92);
  BOOST_CHECK_EQUAL (f.car (1).previous_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).previous_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).previous_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).best_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).best_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).best_lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).lap_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).lap_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).lap_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).current_sector (), 2);
  BOOST_CHECK_EQUAL (f.car (2).current_sector (), 2);
  BOOST_CHECK_EQUAL (f.car (3).current_sector (), 2);
  BOOST_CHECK_EQUAL (f.car (1).previous_sector (), 1);
  BOOST_CHECK_EQUAL (f.car (2).previous_sector (), 1);
  BOOST_CHECK_EQUAL (f.car (3).previous_sector (), 1);
  BOOST_CHECK_EQUAL (f.car (1).sector_time (), 0);
  BOOST_CHECK_EQUAL (f.car (2).sector_time (), 30);
  BOOST_CHECK_EQUAL (f.car (3).sector_time (), 0);
  BOOST_CHECK_EQUAL (f.car (1).previous_sector_time (), 70);
  BOOST_CHECK_EQUAL (f.car (2).previous_sector_time (), 51);
  BOOST_CHECK_EQUAL (f.car (3).previous_sector_time (), 92);
  BOOST_CHECK_EQUAL (f.car (1).best_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).best_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).best_sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).previous_sector_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (2).previous_sector_time_difference (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (3).previous_sector_time_difference (), Timing_Info::NO_TIME);
}

BOOST_AUTO_TEST_CASE (third_lap)
{
  Timing_Fixture f (1, 2, 3);
  f.timing.update (5.0, 0, 100.0, 1);
  f.timing.update (11.0, 0, 200.0, 2);
  f.timing.update (25.0, 0, 2.0, 1);
  f.timing.update (38.0, 0, 201.0, 2);
  f.timing.update (51.0, 0, 3.0, 1);
  f.timing.update (57.0, 0, 100.0, 1);

  BOOST_CHECK_EQUAL (f.timing.total_time (), 57.0);
  BOOST_CHECK_EQUAL (f.car (1).interval (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL (f.car (1).lap_distance (), 100);
  BOOST_CHECK_EQUAL (f.car (1).current_lap (), 3);
  BOOST_CHECK_EQUAL (f.car (1).lap_time (), 6);
  BOOST_CHECK_EQUAL (f.car (1).previous_lap_time (), 26);
  BOOST_CHECK_EQUAL (f.car (1).best_lap_time (), 25);
  BOOST_CHECK_EQUAL (f.car (1).lap_time_difference (), 1);
  BOOST_CHECK_EQUAL (f.car (1).current_sector (), 1);
  BOOST_CHECK_EQUAL (f.car (1).previous_sector (), 2);
  // sector 1
  BOOST_CHECK_EQUAL (f.car (1).sector_time (), 6);
  BOOST_CHECK_EQUAL (f.car (1).best_sector_time (), 11);
  // sector 2
  BOOST_CHECK_EQUAL (f.car (1).previous_sector_time (), 13);
  BOOST_CHECK_EQUAL (f.car (1).previous_sector_time_difference (), -1);
}

BOOST_AUTO_TEST_CASE (end_of_race)
{
  Timing_Fixture f (3, 2, 2);
  f.timing.update (10, 0, 100, 1);
  f.timing.update (20, 1, 101, 1);
  f.timing.update (30, 1, 202, 2);
  f.timing.update (40, 0, 203, 2);
  f.timing.update (50, 2, 104, 1);
  f.timing.update (60, 1, 105, 1); // 1 starts 2nd lap
  f.timing.update (70, 0, 106, 1); // 0 starts 2nd lap
  f.timing.update (80, 1, 207, 2);
  f.timing.update (90, 2, 208, 2); // 2 starts sector 2 of 1st lap
  f.timing.update (95, 0, 208.5, 2); 
  f.timing.update (100, 1, 109, 1); // 1 finishes race
  f.timing.update (111, 2, 110, 1); // 2 finishes race 1 lap down
  f.timing.update (120, 0, 111, 1); // 0 finishes race
  // Further timing updates should be ignored.
  f.timing.update (121, 0, 212, 2); // 0 passes after finish
  f.timing.update (122, 1, 213, 2);
  f.timing.update (123, 2, 214, 2);
  f.timing.update (124, 2, 115, 1); // 2 sets fastest lap after finish
  f.timing.update (170, 1, 116, 1);
  f.timing.update (180, 0, 117, 1);

  Timing_Info::Running_Order::const_iterator it = f.timing.running_order ().begin ();

  // Total time keeps running. If race time is needed it could be provided by
  // the winner's Car_Timing.
  BOOST_CHECK_EQUAL (f.timing.total_time (), 180);

  BOOST_CHECK_EQUAL (&(f.timing.timing_at_index (1)), *it);
  BOOST_CHECK_EQUAL ((*it)->current_lap (), 3);
  BOOST_CHECK_EQUAL ((*it)->interval (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL ((*it)->lap_distance (), 116);
  BOOST_CHECK_EQUAL ((*it)->lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL ((*it)->previous_lap_time (), 40); // last race lap
  BOOST_CHECK_EQUAL (f.timing.fastest_lap_timing (), *it);
  BOOST_CHECK_EQUAL ((*it)->best_lap_time (), 40);
  BOOST_CHECK_EQUAL ((*it)->current_sector (), 1);
  BOOST_CHECK_EQUAL ((*it)->previous_sector (), 2);
  BOOST_CHECK_EQUAL ((*it)->sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL ((*it)->previous_sector_time (), 20); // last race sector

  ++it;
  BOOST_CHECK_EQUAL (&(f.timing.timing_at_index (0)), *it);
  BOOST_CHECK_EQUAL ((*it)->current_lap (), 3);
  BOOST_CHECK_EQUAL ((*it)->interval (), 20);
  BOOST_CHECK_EQUAL ((*it)->lap_distance (), 117);
  BOOST_CHECK_EQUAL ((*it)->lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL ((*it)->previous_lap_time (), 50); // last race lap
  BOOST_CHECK_EQUAL ((*it)->current_sector (), 1);
  BOOST_CHECK_EQUAL ((*it)->previous_sector (), 2);
  BOOST_CHECK_EQUAL ((*it)->sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL ((*it)->previous_sector_time (), 25); // last race sector

  ++it;
  BOOST_CHECK_EQUAL (&(f.timing.timing_at_index (2)), *it);
  BOOST_CHECK_EQUAL ((*it)->current_lap (), 2);
  BOOST_CHECK_EQUAL ((*it)->interval (), 41);
  BOOST_CHECK_EQUAL ((*it)->lap_distance (), 115);
  BOOST_CHECK_EQUAL ((*it)->lap_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL ((*it)->previous_lap_time (), 111); // last race lap
  BOOST_CHECK_EQUAL ((*it)->current_sector (), 1);
  BOOST_CHECK_EQUAL ((*it)->previous_sector (), 2);
  BOOST_CHECK_EQUAL ((*it)->sector_time (), Timing_Info::NO_TIME);
  BOOST_CHECK_EQUAL ((*it)->previous_sector_time (), 21); // last race sector
}
