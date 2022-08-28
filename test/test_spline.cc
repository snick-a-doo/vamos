#include "doctest.h"

#include "test.h"

#include <geometry/spline.h>

#include <cmath>

using namespace Vamos_Geometry;

TEST_CASE("no slope")
{
    Spline spline;

    SUBCASE("interpolate")
    {
        CHECK(spline.size() == size_t(0));
        CHECK(spline.interpolate(-1.0) == 0.0);
        CHECK(spline.interpolate(0.0) == 0.0);
        CHECK(spline.interpolate(1.0) == 0.0);
    }
    SUBCASE("periodic")
    {
        spline.set_periodic(1.0);
        CHECK(spline.interpolate(-1.0) == 0.0);
        CHECK(spline.interpolate(0.0) == 0.0);
        CHECK(spline.interpolate(1.0) == 0.0);
    }

    spline.load(1.0, 2.0);

    SUBCASE("interpolate one")
    {
        CHECK(spline.size() == size_t(1));
        CHECK(spline.interpolate(0.0) == 2.0);
        CHECK(spline.interpolate(1.0) == 2.0);
        CHECK(spline.interpolate(2.0) == 2.0);
    }

    spline.load(2.0, 4.0);

    SUBCASE("interpolate two")
    {
        CHECK(spline.size() == size_t(2));
        CHECK(spline.interpolate(0.0) == 0.0);
        CHECK(spline.interpolate(1.0) == 2.0);
        CHECK(spline.interpolate(1.5) == 3.0);
        CHECK(spline.interpolate(2.0) == 4.0);
        CHECK(spline.interpolate(3.0) == 6.0);
    }
    SUBCASE("interpolate two periodic")
    {
        spline.set_periodic(3.0);
        CHECK(spline.interpolate(0.0) == 4.0);
        CHECK(spline.interpolate(1.0) == 2.0);
        CHECK(spline.interpolate(2.0) == 4.0);
        CHECK(spline.interpolate(3.0) == 2.0);
        CHECK(std::abs(spline.slope(2.999) - spline.slope(3.0)) < 0.02);
        CHECK(std::abs(spline.slope(3.001) - spline.slope(3.0)) < 0.02);
    }

    spline.load(3.0, 5.0);

    SUBCASE("interpolate three")
    {
        CHECK(close(spline.interpolate(1.5), 3.09375, 1e-6));
        CHECK(close(spline.interpolate(2.5), 4.59375, 1e-6));
    }

    SUBCASE("interpolate three periodic")
    {
        spline.set_periodic(4.0);
        CHECK(spline.interpolate(0.5) == 3.375);
        CHECK(spline.interpolate(1.5) == 2.5);
        CHECK(spline.interpolate(2.5) == 5.125);
        CHECK(spline.interpolate(3.5) == 3.375);
        CHECK(std::abs(spline.slope(2.999) - spline.slope(3.0)) < 0.02);
        CHECK(std::abs(spline.slope(3.001) - spline.slope(3.0)) < 0.02);
    }
}

TEST_CASE("slope")
{
    Spline spline(1.0, -1.0);

    SUBCASE("slope")
    {
        CHECK(spline.interpolate(-1.0) == -1.0);
        CHECK(spline.interpolate(0.0) == 0.0);
        CHECK(spline.interpolate(1.0) == 1.0);
    }

    spline.load(1.0, 2.0);

    SUBCASE("interpolate one slope")
    {
        CHECK(spline.interpolate(0.0) == 1.0);
        CHECK(spline.interpolate(1.0) == 2.0);
        CHECK(spline.interpolate(2.0) == 3.0);
    }
    SUBCASE("interpolate one periodic")
    {
        spline.set_periodic(2.0);
        CHECK(spline.interpolate(0.0) == 2.0);
        CHECK(spline.interpolate(1.0) == 2.0);
        CHECK(spline.interpolate(2.0) == 2.0);
    }

    spline.load(2.0, 4.0);

    SUBCASE("interpolate two slope")
    {
        // y = 10 - 21x + 17x^2 - 4x^3
        CHECK(spline.interpolate(0.0) == 10.0);
        CHECK(spline.interpolate(1.0) == 2.0);
        CHECK(spline.interpolate(1.5) == 3.25);
        CHECK(spline.interpolate(2.0) == 4.0);
        CHECK(spline.interpolate(3.0) == -8.0);
        CHECK(close(spline.slope(1.0), 1.0, 1e-6));
        CHECK(close(spline.slope(2.0), -1.0, 1e-6));
    }

    spline.load(3.0, 5.0);

    SUBCASE("interpolate three slope")
    {
        CHECK(close(spline.interpolate(1.5), 2.859375, 1e-6));
        CHECK(close(spline.interpolate(2.5), 4.921875, 1e-6));
        CHECK(close(spline.slope(1.0), 1.0, 1e-6));
        CHECK(close(spline.slope(3.0), -1.0, 1e-6));
    }
}

TEST_CASE("vector spline")
{
    Vector_Spline spline;
    Spline x_spline;
    Spline y_spline;
    Spline z_spline;

    Three_Vector p1;
    Three_Vector p2(1.0, 2.0, 3.0);
    spline.load(1.0, p1);
    spline.load(2.0, p2);
    spline.load(3.0, {2.0, 4.0, 8.0});
    spline.load(4.0, p1);

    x_spline.load(1.0, p1.x);
    x_spline.load(2.0, p2.x);
    x_spline.load(3.0, 2.0);
    x_spline.load(4.0, p1.x);

    y_spline.load(1.0, p1.y);
    y_spline.load(2.0, p2.y);
    y_spline.load(3.0, 4.0);
    y_spline.load(4.0, p1.y);

    z_spline.load(1.0, p1.z);
    z_spline.load(2.0, p2.z);
    z_spline.load(3.0, 8.0);
    z_spline.load(4.0, p1.z);

    for (auto x = 1.5; x < 4.0; x += 1.0)
    {
        auto p = spline.interpolate(x);
        CHECK(p.x == x_spline.interpolate(x));
        CHECK(p.y == y_spline.interpolate(x));
        CHECK(p.z == z_spline.interpolate(x));
    }
}
