#include "doctest.h"

#include "test.h"

#include <world/timing-info.h>

#include <cassert>
#include <vector>

using namespace Vamos_World;

struct Timing_Fixture
{
    Timing_Fixture(size_t n_cars, size_t n_sectors)
        : timing(n_cars, n_sectors, false) // Skip the start sequence.
    {
    }
    const Car_Timing& car(size_t n) const { return timing.timing_at_index(n - 1); }
    Timing_Info timing;
};

struct Lap_Race_Fixture : public Timing_Fixture
{
    Lap_Race_Fixture(size_t n_cars, size_t n_sectors, size_t n_laps)
        : Timing_Fixture(n_cars, n_sectors)
    {
        timing.set_lap_limit(n_laps);
    }
};

TEST_CASE("initial state")
{
    Lap_Race_Fixture f(3, 3, 4);
    CHECK(f.timing.elapsed_time() == 0.0);
    CHECK(f.timing.total_laps() == 4);
    CHECK(f.car(1).current_lap() == 0);
    CHECK(f.car(2).current_lap() == 0);
    CHECK(f.car(3).current_lap() == 0);
    CHECK(f.car(1).interval() == Timing_Info::no_time);
    CHECK(f.car(2).interval() == Timing_Info::no_time);
    CHECK(f.car(3).interval() == Timing_Info::no_time);
    CHECK(f.car(1).current_lap() == 0);
    CHECK(f.car(2).current_lap() == 0);
    CHECK(f.car(3).current_lap() == 0);
    CHECK(f.car(1).current_lap_time() == 0);
    CHECK(f.car(2).current_lap_time() == 0);
    CHECK(f.car(3).current_lap_time() == 0);
    CHECK(f.car(1).previous_lap_time() == Timing_Info::no_time);
    CHECK(f.car(2).previous_lap_time() == Timing_Info::no_time);
    CHECK(f.car(3).previous_lap_time() == Timing_Info::no_time);
    CHECK(f.car(1).best_lap_time() == Timing_Info::no_time);
    CHECK(f.car(2).best_lap_time() == Timing_Info::no_time);
    CHECK(f.car(3).best_lap_time() == Timing_Info::no_time);
    CHECK(f.car(1).lap_time_difference() == Timing_Info::no_time);
    CHECK(f.car(2).lap_time_difference() == Timing_Info::no_time);
    CHECK(f.car(3).lap_time_difference() == Timing_Info::no_time);
    CHECK(f.car(1).current_sector() == 0);
    CHECK(f.car(2).current_sector() == 0);
    CHECK(f.car(3).current_sector() == 0);
    CHECK(f.car(1).previous_sector() == 0);
    CHECK(f.car(2).previous_sector() == 0);
    CHECK(f.car(3).previous_sector() == 0);
    CHECK(f.car(1).sector_time() == 0);
    CHECK(f.car(2).sector_time() == 0);
    CHECK(f.car(3).sector_time() == 0);
    CHECK(f.car(1).previous_sector_time() == Timing_Info::no_time);
    CHECK(f.car(2).previous_sector_time() == Timing_Info::no_time);
    CHECK(f.car(3).previous_sector_time() == Timing_Info::no_time);
    CHECK(f.car(1).best_sector_time() == Timing_Info::no_time);
    CHECK(f.car(2).best_sector_time() == Timing_Info::no_time);
    CHECK(f.car(3).best_sector_time() == Timing_Info::no_time);
    CHECK(f.car(1).previous_sector_time_difference() == Timing_Info::no_time);
    CHECK(f.car(2).previous_sector_time_difference() == Timing_Info::no_time);
    CHECK(f.car(3).previous_sector_time_difference() == Timing_Info::no_time);
}

TEST_CASE("second sector")
{
    Lap_Race_Fixture f(3, 4, 3);
    // Car 2 finishes the first sector first.
    f.timing.update(10.0, 0, 100.0, 1);
    f.timing.update(21.0, 1, 250.0, 1);
    f.timing.update(32.0, 2, 300.0, 1);
    f.timing.update(40.0, 0, 400.0, 1);
    f.timing.update(51.0, 1, 600.0, 2);
    f.timing.update(62.0, 2, 500.0, 1);
    f.timing.update(70.0, 0, 600.0, 2);
    f.timing.update(81.0, 1, 900.0, 2);
    f.timing.update(92.0, 2, 750.0, 2);
    CHECK(f.timing.elapsed_time() == 92.0);
    CHECK(f.car(1).interval() == 19);
    CHECK(f.car(2).interval() == Timing_Info::no_time);
    CHECK(f.car(3).interval() == 22);
    CHECK(f.car(1).lap_distance() == 600);
    CHECK(f.car(2).lap_distance() == 900);
    CHECK(f.car(3).lap_distance() == 750);
    CHECK(f.car(1).current_lap() == 1);
    CHECK(f.car(2).current_lap() == 1);
    CHECK(f.car(3).current_lap() == 1);
    CHECK(f.car(1).current_lap_time() == 70);
    CHECK(f.car(2).current_lap_time() == 81);
    CHECK(f.car(3).current_lap_time() == 92);
    CHECK(f.car(1).previous_lap_time() == Timing_Info::no_time);
    CHECK(f.car(2).previous_lap_time() == Timing_Info::no_time);
    CHECK(f.car(3).previous_lap_time() == Timing_Info::no_time);
    CHECK(f.car(1).best_lap_time() == Timing_Info::no_time);
    CHECK(f.car(2).best_lap_time() == Timing_Info::no_time);
    CHECK(f.car(3).best_lap_time() == Timing_Info::no_time);
    CHECK(f.car(1).lap_time_difference() == Timing_Info::no_time);
    CHECK(f.car(2).lap_time_difference() == Timing_Info::no_time);
    CHECK(f.car(3).lap_time_difference() == Timing_Info::no_time);
    CHECK(f.car(1).current_sector() == 2);
    CHECK(f.car(2).current_sector() == 2);
    CHECK(f.car(3).current_sector() == 2);
    CHECK(f.car(1).previous_sector() == 1);
    CHECK(f.car(2).previous_sector() == 1);
    CHECK(f.car(3).previous_sector() == 1);
    CHECK(f.car(1).sector_time() == 0);
    CHECK(f.car(2).sector_time() == 30);
    CHECK(f.car(3).sector_time() == 0);
    CHECK(f.car(1).previous_sector_time() == 70);
    CHECK(f.car(2).previous_sector_time() == 51);
    CHECK(f.car(3).previous_sector_time() == 92);
    CHECK(f.car(1).best_sector_time() == Timing_Info::no_time);
    CHECK(f.car(2).best_sector_time() == Timing_Info::no_time);
    CHECK(f.car(3).best_sector_time() == Timing_Info::no_time);
    CHECK(f.car(1).previous_sector_time_difference() == Timing_Info::no_time);
    CHECK(f.car(2).previous_sector_time_difference() == Timing_Info::no_time);
    CHECK(f.car(3).previous_sector_time_difference() == Timing_Info::no_time);
}

TEST_CASE("third lap")
{
    Lap_Race_Fixture f(1, 2, 3);
    f.timing.update(5.0, 0, 100.0, 1);
    f.timing.update(11.0, 0, 200.0, 2);
    f.timing.update(25.0, 0, 2.0, 1);
    f.timing.update(38.0, 0, 201.0, 2);
    f.timing.update(51.0, 0, 3.0, 1);
    f.timing.update(57.0, 0, 100.0, 1);

    CHECK(f.timing.elapsed_time() == 57.0);
    CHECK(f.car(1).interval() == Timing_Info::no_time);
    CHECK(f.car(1).lap_distance() == 100);
    CHECK(f.car(1).current_lap() == 3);
    CHECK(f.car(1).lap_time(2) == 51.0);
    CHECK(f.car(1).previous_lap_time() == 26);
    CHECK(f.car(1).best_lap_time() == 25);
    CHECK(f.car(1).lap_time_difference() == 1);
    CHECK(f.car(1).current_sector() == 1);
    CHECK(f.car(1).previous_sector() == 2);
    // sector 1
    CHECK(f.car(1).sector_time() == 6);
    CHECK(f.car(1).best_sector_time() == 11);
    // sector 2
    CHECK(f.car(1).previous_sector_time() == 13);
    CHECK(f.car(1).previous_sector_time_difference() == -1);
}

TEST_CASE("end of race")
{
    Lap_Race_Fixture f(3, 2, 2);
    f.timing.update(10, 0, 100, 1);
    f.timing.update(20, 1, 101, 1);
    f.timing.update(30, 1, 202, 2);
    f.timing.update(40, 0, 203, 2);
    f.timing.update(50, 2, 104, 1);
    f.timing.update(60, 1, 105, 1); // 1 starts 2nd lap
    f.timing.update(70, 0, 106, 1); // 0 starts 2nd lap
    f.timing.update(80, 1, 207, 2);
    f.timing.update(90, 2, 208, 2); // 2 starts sector 2 of 1st lap
    f.timing.update(95, 0, 208.5, 2);
    f.timing.update(100, 1, 109, 1); // 1 finishes race
    f.timing.update(111, 2, 110, 1); // 2 finishes race 1 lap down
    f.timing.update(120, 0, 111, 1); // 0 finishes race
    // Further timing updates should be ignored.
    f.timing.update(121, 0, 212, 2); // 0 passes after finish
    f.timing.update(122, 1, 213, 2);
    f.timing.update(123, 2, 214, 2);
    f.timing.update(124, 2, 115, 1); // 2 sets fastest lap after finish
    f.timing.update(170, 1, 116, 1);
    f.timing.update(180, 0, 117, 1);

    auto it{f.timing.running_order().cbegin()};

    // Total time keeps running. If race time is needed it could be provided by
    // the winner's Car_Timing.
    CHECK(f.timing.elapsed_time() == 180);

    CHECK(&(f.timing.timing_at_index(1)) == it->get());
    CHECK((*it)->current_lap() == 3);
    CHECK((*it)->interval() == Timing_Info::no_time);
    CHECK((*it)->lap_distance() == 116);
    CHECK((*it)->current_lap_time() == Timing_Info::no_time);
    CHECK((*it)->lap_time(1) == 60.0);
    CHECK((*it)->previous_lap_time() == 40); // last race lap
    CHECK(f.timing.fastest_lap_timing() == it->get());
    CHECK((*it)->best_lap_time() == 40);
    CHECK((*it)->current_sector() == 1);
    CHECK((*it)->previous_sector() == 2);
    CHECK((*it)->sector_time() == Timing_Info::no_time);
    CHECK((*it)->previous_sector_time() == 20); // last race sector

    ++it;
    CHECK(&(f.timing.timing_at_index(0)) == it->get());
    CHECK((*it)->current_lap() == 3);
    CHECK((*it)->interval() == 20);
    CHECK((*it)->lap_distance() == 117);
    CHECK((*it)->current_lap_time() == Timing_Info::no_time);
    CHECK((*it)->lap_time(1) == 70.0);
    CHECK((*it)->previous_lap_time() == 50); // last race lap
    CHECK((*it)->current_sector() == 1);
    CHECK((*it)->previous_sector() == 2);
    CHECK((*it)->sector_time() == Timing_Info::no_time);
    CHECK((*it)->previous_sector_time() == 25); // last race sector

    ++it;
    CHECK(&(f.timing.timing_at_index(2)) == it->get());
    CHECK((*it)->current_lap() == 2);
    CHECK((*it)->interval() == 41);
    CHECK((*it)->lap_distance() == 115);
    CHECK((*it)->current_lap_time() == Timing_Info::no_time);
    CHECK((*it)->lap_time(1) == 111);
    CHECK((*it)->previous_lap_time() == 111); // last race lap
    CHECK((*it)->current_sector() == 1);
    CHECK((*it)->previous_sector() == 2);
    CHECK((*it)->sector_time() == Timing_Info::no_time);
    CHECK((*it)->previous_sector_time() == 21); // last race sector
}

struct Timed_Race_Fixture : public Timing_Fixture
{
    Timed_Race_Fixture(size_t n_cars, size_t n_sectors, double minutes)
        : Timing_Fixture(n_cars, n_sectors)
    {
        timing.set_time_limit(minutes);
    }
};

TEST_CASE("end of timed race")
{
    Timed_Race_Fixture f(3, 2, 99 / 60.0);
    f.timing.update(10, 0, 100, 1);
    f.timing.update(20, 1, 101, 1);
    f.timing.update(30, 1, 202, 2);
    f.timing.update(40, 0, 203, 2);
    f.timing.update(50, 2, 104, 1);
    f.timing.update(60, 1, 105, 1); // 1 starts 2nd lap
    f.timing.update(70, 0, 106, 1); // 0 starts 2nd lap
    f.timing.update(80, 1, 207, 2);
    f.timing.update(90, 2, 208, 2); // 2 starts sector 2 of 1st lap
    f.timing.update(95, 0, 208.5, 2);
    f.timing.update(100, 1, 109, 1); // 1 finishes race
    f.timing.update(111, 2, 110, 1); // 2 finishes race 1 lap down
    f.timing.update(120, 0, 111, 1); // 0 finishes race
    // Further timing updates should be ignored.
    f.timing.update(121, 0, 212, 2); // 0 passes after finish
    f.timing.update(122, 1, 213, 2);
    f.timing.update(123, 2, 214, 2);
    f.timing.update(124, 2, 115, 1); // 2 sets fastest lap after finish
    f.timing.update(170, 1, 116, 1);
    f.timing.update(180, 0, 117, 1);

    Timing_Info::Running_Order::const_iterator it = f.timing.running_order().begin();

    // Total time keeps running. If race time is needed it could be provided by
    // the winner's Car_Timing.
    CHECK(f.timing.elapsed_time() == 180);

    CHECK(&(f.timing.timing_at_index(1)) == it->get());
    CHECK((*it)->current_lap() == 3);
    CHECK((*it)->interval() == Timing_Info::no_time);
    CHECK((*it)->lap_distance() == 116);
    CHECK((*it)->current_lap_time() == Timing_Info::no_time);
    CHECK((*it)->lap_time(1) == 60);
    CHECK((*it)->previous_lap_time() == 40); // last race lap
    CHECK(f.timing.fastest_lap_timing() == it->get());
    CHECK((*it)->best_lap_time() == 40);
    CHECK((*it)->current_sector() == 1);
    CHECK((*it)->previous_sector() == 2);
    CHECK((*it)->sector_time() == Timing_Info::no_time);
    CHECK((*it)->previous_sector_time() == 20); // last race sector

    ++it;
    CHECK(&(f.timing.timing_at_index(0)) == it->get());
    CHECK((*it)->current_lap() == 3);
    CHECK((*it)->interval() == 20);
    CHECK((*it)->lap_distance() == 117);
    CHECK((*it)->current_lap_time() == Timing_Info::no_time);
    CHECK((*it)->lap_time(1) == 70);
    CHECK((*it)->previous_lap_time() == 50); // last race lap
    CHECK((*it)->current_sector() == 1);
    CHECK((*it)->previous_sector() == 2);
    CHECK((*it)->sector_time() == Timing_Info::no_time);
    CHECK((*it)->previous_sector_time() == 25); // last race sector

    ++it;
    CHECK(&(f.timing.timing_at_index(2)) == it->get());
    CHECK((*it)->current_lap() == 2);
    CHECK((*it)->interval() == 41);
    CHECK((*it)->lap_distance() == 115);
    CHECK((*it)->current_lap_time() == Timing_Info::no_time);
    CHECK((*it)->lap_time(1) == 111);
    CHECK((*it)->previous_lap_time() == 111); // last race lap
    CHECK((*it)->current_sector() == 1);
    CHECK((*it)->previous_sector() == 2);
    CHECK((*it)->sector_time() == Timing_Info::no_time);
    CHECK((*it)->previous_sector_time() == 21); // last race sector
}

TEST_CASE("qualifying")
{
    Timed_Race_Fixture f(3, 2, 99 / 60.0);
    f.timing.set_qualifying();
    f.timing.update(10, 0, 100, 1);
    f.timing.update(20, 1, 101, 1);
    f.timing.update(30, 1, 202, 2);
    f.timing.update(40, 0, 203, 2);
    f.timing.update(50, 2, 104, 1);
    f.timing.update(55, 1, 105, 1); // 1 starts 2nd lap (1st lap 35 s, pole)
    CHECK(f.timing.timing_at_index(1).previous_lap_time() == 55);
    f.timing.update(70, 0, 106, 1); // 0 starts 2nd lap (1st lap 60 s)
    CHECK(f.timing.timing_at_index(0).previous_lap_time() == 70);
    f.timing.update(80, 1, 207, 2);
    f.timing.update(90, 2, 208, 2); // 2 starts sector 2 of 1st lap
    f.timing.update(95, 0, 208.5, 2);
    f.timing.update(100, 1, 109, 1); // 1 finishes with 2 laps completed (45 s, 2nd)
    CHECK(f.timing.timing_at_index(1).previous_lap_time() == 45);
    f.timing.update(111, 2, 110, 1); // 2 finishes with 1 lap completed (111 s)
    CHECK(f.timing.timing_at_index(2).previous_lap_time() == 111);
    f.timing.update(120, 0, 111, 1); // 0 finishes with 2 laps completed (50 s, 2rd)
    CHECK(f.timing.timing_at_index(0).previous_lap_time() == 50);
    // Further timing updates should be ignored.
    f.timing.update(121, 0, 212, 2);
    f.timing.update(122, 1, 213, 2);
    f.timing.update(123, 2, 214, 2);
    f.timing.update(124, 2, 115, 1); // 2 gets fastest lap but already finished.
    f.timing.update(170, 1, 116, 1);
    f.timing.update(180, 0, 117, 1);

    Timing_Info::Running_Order::const_iterator it = f.timing.running_order().begin();

    CHECK(f.timing.timing_at_index(0).best_lap_time() == 50);
    CHECK(f.timing.timing_at_index(1).best_lap_time() == 45);
    CHECK(f.timing.timing_at_index(2).best_lap_time() == 111);

    CHECK(&(f.timing.timing_at_index(1)) == (it++)->get());
    CHECK(&(f.timing.timing_at_index(0)) == (it++)->get());
    CHECK(&(f.timing.timing_at_index(2)) == (it++)->get());
}
