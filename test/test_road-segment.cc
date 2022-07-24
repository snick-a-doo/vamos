#include "doctest.h"

#include "test.h"

#include <geometry/conversions.h>
#include <track/road-segment.h>

using namespace Vamos_Geometry;
using namespace Vamos_Track;

TEST_CASE("straight")
{
    Road_Segment segment(100.0, 0.0, 10.0, 10.0, 5.0, 5.0);
    Spline elevation;
    segment.build_elevation(&elevation, 0.0);
    Three_Vector track_pos;

    SUBCASE("start")
    {
        Three_Vector world_pos(0.0, 0.0, 0.0);
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
    SUBCASE("end")
    {
        Three_Vector world_pos(segment.length(), 0.0, 0.0);
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, segment.length(), 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }

    SUBCASE("off start")
    {
        Three_Vector world_pos(-1.0, 1.0, 0.0);
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, -1.0, 1e-6));
        CHECK(close(track_pos.x, -1.0, 1e-6));
        CHECK(close(track_pos.y, 1.0, 1e-6));
    }

    SUBCASE("off end")
    {
        Three_Vector world_pos(segment.length() + 1, 1.0, 0.0);
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 1.0, 1e-6));
        CHECK(close(track_pos.x, segment.length() + 1, 1e-6));
        CHECK(close(track_pos.y, 1.0, 1e-6));
    }

    SUBCASE("moved")
    {
        segment.set_start(Three_Vector(1.0, 2.0, 3.0), 4.0, pi / 2.0, -pi / 4.0,
                          std::vector<double>());

        SUBCASE("start")
        {
            Three_Vector world_pos(1.0, 2.0, 3.0);
            auto off = segment.coordinates(world_pos, track_pos);
            CHECK(close(off, 0.0, 1e-6));
            CHECK(close(track_pos.x, 0.0, 1e-6));
            CHECK(close(track_pos.y, 0.0, 1e-6));
        }
        SUBCASE("end")
        {
            Three_Vector world_pos(1.0, segment.length() + 2.0, 3.0);
            auto off = segment.coordinates(world_pos, track_pos);
            CHECK(close(off, 0.0, 1e-6));
            CHECK(close(track_pos.x, segment.length(), 1e-6));
            CHECK(close(track_pos.y, 0.0, 1e-6));
        }
        SUBCASE("position")
        {
            Three_Vector world_pos = segment.position(100.0, 100.0);
            CHECK(close(world_pos.x, -99.0, 1e-6));
            CHECK(close(world_pos.y, 102.0, 1e-6));
        }
    }
}

TEST_CASE("left turn")
{
    auto initial_length{pi * 50.0};
    auto initial_radius{100.0};
    auto initial_arc{initial_length / initial_radius};
    Three_Vector track_pos;
    Road_Segment segment{initial_length, initial_radius, 10.0, 10.0, 5.0, 5.0};
    Spline elevation;
    segment.build_elevation(&elevation, 0.0);
    segment.set_start(Three_Vector(100.0, 2.0, 3.0), 111, pi / 2.0, 0.5, {});

    CHECK(segment.length() == initial_length);
    CHECK(segment.radius() == initial_radius);
    CHECK(close(segment.length() / segment.radius(), initial_arc, 1e-6));
    CHECK(close(segment.left_width(initial_length / 4.0), 10.0, 1e-6));
    CHECK(close(segment.right_width(initial_length / 2.0), 10.0, 1e-6));
    CHECK(close(segment.left_road_width(initial_length / 4.0), 5.0, 1e-6));
    CHECK(close(segment.right_road_width(initial_length), 5.0, 1e-6));
    CHECK(segment.start_coords().x == 100.0);
    CHECK(segment.start_coords().y == 2.0);
    CHECK(close(segment.end_coords().x, 0.0, 1e-6));
    CHECK(close(segment.end_coords().y, 102.0, 1e-4));

    SUBCASE("change_length")
    {
        segment.set_length(2.0 * initial_length);
        CHECK(segment.length() == 2.0 * initial_length);
        CHECK(segment.radius() == initial_radius);
        CHECK(close(segment.length() / segment.radius(), initial_arc * 2.0, 1e-6));
    }
    SUBCASE("arc")
    {
        segment.set_arc(initial_arc / 2.0);
        CHECK(segment.length() == initial_length / 2);
        CHECK(segment.radius() == initial_radius);
        CHECK(close(segment.length() / segment.radius(), initial_arc / 2.0, 1e-6));
    }
    SUBCASE("scale")
    {
        segment.scale(0.5);
        CHECK(close(segment.length(), initial_length / 2.0, 1e-6));
        CHECK(close(segment.radius(), initial_radius / 2.0, 1e-6));
        CHECK(close(segment.arc(), initial_arc, 1e-6));
    }
    SUBCASE("start")
    {
        Three_Vector world_pos(segment.start_coords());
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
    SUBCASE("start left")
    {
        Three_Vector world_pos(segment.start_coords());
        world_pos.x -= 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("start right")
    {
        Three_Vector world_pos(segment.start_coords());
        world_pos.x += 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
    SUBCASE("end")
    {
        Three_Vector world_pos(segment.end_coords());
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50 * pi, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }

    SUBCASE("end left")
    {
        Three_Vector world_pos(segment.end_coords());
        world_pos.y -= 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50.0 * pi, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("end right")
    {
        Three_Vector world_pos(segment.end_coords());
        world_pos.y += 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50.0 * pi, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
    SUBCASE("position")
    {
        Three_Vector world_pos = segment.position(pi * 25.0, 50.0);
        auto const r{50.0 / sqrt(2.0)};
        CHECK(close(world_pos.x, r, 1e-6));
        CHECK(close(world_pos.y, r + 2.0, 1e-6));
    }
}

TEST_CASE("right turn")
{
    Three_Vector track_pos;
    Road_Segment segment{pi * 50.0, -100.0, 10.0, 10.0, 5.0, 5.0};
    Spline elevation;
    segment.build_elevation(&elevation, 0.0);
    segment.set_start(Three_Vector(100.0, 2.0, 3.0), 111, pi / 2.0, 0.5, {});

    SUBCASE("start")
    {
        Three_Vector world_pos(segment.start_coords());
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
    SUBCASE("start left")
    {
        Three_Vector world_pos(segment.start_coords());
        world_pos.x -= 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("start right")
    {
        Three_Vector world_pos(segment.start_coords());
        world_pos.x += 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
    SUBCASE("end")
    {
        Three_Vector world_pos(segment.end_coords());
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50 * pi, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
}

TEST_CASE("skewed")
{
    Three_Vector track_pos;
    Road_Segment segment{100.0, 0.0, 10.0, 10.0, 5.0, 5.0};
    Spline elevation;
    segment.set_start_skew(-1.0);
    segment.set_end_skew(1.0);
    segment.build_elevation(&elevation, 0.0);

    SUBCASE("start")
    {
        Three_Vector world_pos;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
    SUBCASE("start left")
    {
        Three_Vector world_pos(-10.0, 10.0, 0.0);
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("start right")
    {
        Three_Vector world_pos(10.0, -10.0, 0.0);
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
    SUBCASE("end left")
    {
        Three_Vector world_pos(110.0, 10.0, 0.0);
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 100.0, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("end right")
    {
        Three_Vector world_pos(90.0, -10.0, 0.0);
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 100.0, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
}

TEST_CASE("skewed left turn")
{
    Three_Vector track_pos;
    Road_Segment segment{pi * 50.0, 100.0, 10.0, 10.0, 5.0, 5.0};
    Spline elevation;
    segment.set_start(Three_Vector(100.0, 2.0, 3.0), 111, pi / 2.0, 0.0, {});
    segment.set_start_skew(-0.5);
    segment.set_end_skew(0.5);
    segment.build_elevation(&elevation, 0.0);

    SUBCASE("start")
    {
        Three_Vector world_pos(segment.start_coords());
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
    SUBCASE("start left")
    {
        Three_Vector world_pos(segment.start_coords());
        world_pos.x -= 10.0;
        world_pos.y -= 5.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("start right")
    {
        Three_Vector world_pos(segment.start_coords());
        world_pos.x += 10.0;
        world_pos.y += 5.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
    SUBCASE("end")
    {
        Three_Vector world_pos(segment.end_coords());
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50 * pi, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
    SUBCASE("end left")
    {
        Three_Vector world_pos(segment.end_coords());
        world_pos.x -= 5.0;
        world_pos.y -= 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50.0 * pi, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("end right")
    {
        Three_Vector world_pos(segment.end_coords());
        world_pos.x += 5.0;
        world_pos.y += 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50.0 * pi, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
}

TEST_CASE("skewed right turn")
{
    Three_Vector track_pos;
    Road_Segment segment{pi * 50.0, -100.0, 10.0, 10.0, 5.0, 5.0};
    Spline elevation;
    segment.set_start(Three_Vector(100.0, 2.0, 3.0), 111, pi / 2.0, 0.0, {});
    segment.set_start_skew(0.5);
    segment.set_end_skew(-0.5);
    segment.build_elevation(&elevation, 0.0);

    SUBCASE("start")
    {
        Three_Vector world_pos(segment.start_coords());
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
    SUBCASE("start left")
    {
        Three_Vector world_pos(segment.start_coords());
        world_pos.x -= 10.0;
        world_pos.y += 5.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("start right")
    {
        Three_Vector world_pos(segment.start_coords());
        world_pos.x += 10.0;
        world_pos.y -= 5.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 0.0, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
    SUBCASE("end")
    {
        Three_Vector world_pos(segment.end_coords());
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50 * pi, 1e-6));
        CHECK(close(track_pos.y, 0.0, 1e-6));
    }
    SUBCASE("end left")
    {
        Three_Vector world_pos(segment.end_coords());
        world_pos.x -= 5.0;
        world_pos.y += 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50.0 * pi, 1e-6));
        CHECK(close(track_pos.y, 10.0, 1e-6));
    }
    SUBCASE("end right")
    {
        Three_Vector world_pos(segment.end_coords());
        world_pos.x += 5.0;
        world_pos.y -= 10.0;
        auto off = segment.coordinates(world_pos, track_pos);
        CHECK(close(off, 0.0, 1e-6));
        CHECK(close(track_pos.x, 50.0 * pi, 1e-6));
        CHECK(close(track_pos.y, -10.0, 1e-6));
    }
}
