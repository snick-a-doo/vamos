#include "doctest.h"

#include "test.h"

#include <track/strip-track.h>

using namespace Vamos_Geometry;
using namespace Vamos_Track;

struct Track_Fixture
{
    void add(double length, double radius)
    {
        static std::vector<Two_Vector> left;
        left.push_back(Two_Vector(0.0, 5.0));
        static std::vector<Two_Vector> right;
        right.push_back(Two_Vector(0.0, 5.0));
        static std::vector<Two_Vector> left_road;
        left_road.push_back(Two_Vector(0.0, 10.0));
        static std::vector<Two_Vector> right_road;
        right_road.push_back(Two_Vector(0.0, 10.0));
        static std::vector<Two_Vector> elevation;
        static std::vector<Material> material(7);
        static std::vector<Braking_Marker*> marker;
        track.add_segment(new Gl_Road_Segment(10.0, length, radius, 0.0, left, right, left_road,
                                              right_road, 0, 0, 1.0, 1.0, elevation, 0.0, 0.0,
                                              material, marker));
    }
    Gl_Road_Segment const* segment(int n) const
    {
        const Segment_List& segments = track.get_road(0).segments();
        return *((n >= 0 ? segments.begin() : segments.end()) + n);
    }
    Three_Vector end() const { return segment(-1)->end_coords(); }
    Strip_Track track;
};

struct Oval_Fixture : public Track_Fixture
{
    Oval_Fixture(bool close, int n)
    {
        add(100.0, 0.0);
        add(pi * 50.0, 50.0);
        add(200.0, 0.0);
        add(pi * 50.0, 50.0);
        add(50.0, 0.0);
        track.set_start_direction(90.0);
        track.build(close, n, 0.0, false, 0);
    }
};

TEST_CASE("oval open")
{
    Oval_Fixture f(false, 0);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, -50.0, 1e-6));
    CHECK(close(f.end().z, 0.0, 1e-6));
}

TEST_CASE("oval adjust 0")
{
    Oval_Fixture f(true, 0);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, -50.0, 1e-6));
}

TEST_CASE("oval adjust 1")
{
    Oval_Fixture f(true, 1);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, 0.0, 1e-6));
}

struct Square_Fixture : public Track_Fixture
{
    Square_Fixture(int close)
    {
        add(100.0, 0.0);
        add(pi * 10.0, 20.0);
        add(200.0, 0.0);
        add(pi * 10.0, 20.0);
        add(200.0, 0.0);
        add(pi * 10.0, 20.0);
        add(190.0, 0.0);
        add(pi * 5.0, 20.0);
        add(50.0, 0.0);
        track.set_start_direction(-90.0);
        track.build(true, close, 0.0, false, 0);
    }
};

TEST_CASE("square adjust 3")
{
    Square_Fixture f(3);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, 0.0, 1e-6));
    CHECK(close(f.segment(-3)->length(), 200.0, 1e-6));
    CHECK(close(f.segment(-2)->radius(), 20.0, 1e-6));
    CHECK(close(f.segment(-2)->arc(), pi / 2.0, 1e-6));
    CHECK(close(f.segment(-1)->length(), 100.0, 1e-6));
}

TEST_CASE("square adjust 2")
{
    Square_Fixture f(2);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, 0.0, 1e-6));
    CHECK(close(f.segment(-3)->length(), 190.0, 1e-6));
    CHECK(close(f.segment(-2)->radius(), 30.0, 1e-6));
    CHECK(close(f.segment(-2)->arc(), pi / 2.0, 1e-6));
    CHECK(close(f.segment(-1)->length(), 90.0, 1e-6));
}

struct Triangle_Fixture : public Track_Fixture
{
    Triangle_Fixture(int close)
    {
        add(100.0, 0.0);
        add(pi * 20.0, -30.0);
        add(200.0, 0.0);
        add(pi * 20.0, -30.0);
        add(190.0, 0.0);
        add(pi * 25.0, -30.0);
        add(50.0, 0.0);
        track.set_start_direction(180.0);
        track.build(true, close, 0.0, false, 0);
    }
};

TEST_CASE("triangle adjust 3")
{
    Triangle_Fixture f(3);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, 0.0, 1e-6));
    CHECK(close(f.segment(-3)->length(), 200.0, 1e-6));
    CHECK(close(f.segment(-2)->radius(), -30.0, 1e-6));
    CHECK(close(f.segment(-2)->arc(), -2.0 * pi / 3.0, 1e-6));
    CHECK(close(f.segment(-1)->length(), 100.0, 1e-6));
}

TEST_CASE("triangle adjust 2")
{
    Triangle_Fixture f(2);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, 0.0, 1e-6));
    CHECK(close(f.segment(-3)->length(), 190.0, 1e-6));
    CHECK(close(f.segment(-2)->radius(), -30.0 - 10.0 / sqrt(3.0), 1e-6));
    CHECK(close(f.segment(-2)->arc(), -2.0 * pi / 3.0, 1e-6));
    CHECK(close(f.segment(-1)->length(), 90.0, 1e-6));
}

struct Obtuse_Fixture : public Track_Fixture
{
    Obtuse_Fixture(int close)
    {
        add(100.0, 0.0);
        add(pi * 10.0, 20.0);
        add(200.0, 0.0);
        add(pi * 10.0, 20.0);
        add(400.0, 0.0);
        add(pi * 15.0, 20.0);
        add(200.0, 0.0);
        add(pi * 10.0, 20.0);
        add(50.0, 0.0);
        track.set_start_direction(-90.0);
        track.build(true, close, 0.0, false, 0);
    }
};

TEST_CASE("obtuse adjust 3")
{
    Obtuse_Fixture f(3);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, 0.0, 1e-6));
    CHECK(close(f.segment(-3)->length(), 200.0 * sqrt(2.0), 1e-6));
    CHECK(close(f.segment(-2)->radius(), 20.0, 1e-6));
    CHECK(close(f.segment(-2)->arc(), pi / 4.0, 1e-6));
    CHECK(close(f.segment(-1)->length(), 100.0, 1e-6));
}

TEST_CASE("obtuse adjust 2")
{
    Obtuse_Fixture f(2);
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, 0.0, 1e-6));
    CHECK(close(f.segment(-3)->length(), 200.0, 1e-6));
    CHECK(close(f.segment(-2)->radius(), 220.0, 1e-6));
    CHECK(close(f.segment(-2)->arc(), pi / 4.0, 1e-6));
    CHECK(close(f.segment(-1)->length(), 300.0 - 400.0 / sqrt(2.0), 1e-6));
}

struct Circle_Fixture : public Track_Fixture
{
    Circle_Fixture()
    {
        add(600.0, 100.0);
        track.build(true, 1, 0.0, false, 0);
    }
};

TEST_CASE("circle adjust 1")
{
    Circle_Fixture f;
    CHECK(close(f.end().x, 0.0, 1e-6));
    CHECK(close(f.end().y, 0.0, 1e-6));
    CHECK(close(f.segment(-1)->radius(), 100.0, 1e-6));
    CHECK(close(f.segment(-1)->arc(), 2 * pi, 1e-6));
    CHECK(close(f.segment(-1)->length(), 200.0 * pi, 1e-6));
}
