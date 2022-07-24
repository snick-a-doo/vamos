#include "doctest.h"

#include "test.h"

#include <geometry/constants.h>
#include <geometry/linear-interpolator.h>

using namespace Vamos_Geometry;

TEST_CASE("angle")
{
    Linear_Interpolator linear{{Two_Vector(0.0, 0.0), Two_Vector(1.0, 1.0)}};

    SUBCASE("interpolate")
    {
        CHECK(linear.interpolate(0.0) == 0.0);
        CHECK(linear.interpolate(0.5) == 0.5);
        CHECK(linear.interpolate(1.0) == 1.0);
    }
    SUBCASE("extrapolate")
    {
        // Extrapolation is clamped to the endpoints.
        CHECK(linear.interpolate(-1.0) == 0.0);
        CHECK(linear.interpolate(100.0) == 1.0);
    }
}

TEST_CASE("zigzag")
{
    Linear_Interpolator linear{{Two_Vector(0.0, 1.0),
            Two_Vector(1.0, 1.0),
            Two_Vector(2.0, 2.0),
            Two_Vector(3.0, 2.0)}};

    SUBCASE("interpolate")
    {
        CHECK(linear.interpolate(0.0) == 1.0);
        CHECK(linear.interpolate(0.5) == 1.0);
        CHECK(linear.interpolate(1.0) == 1.0);
        CHECK(linear.interpolate(1.5) == 1.5);
        CHECK(linear.interpolate(2.0) == 2.0);
        CHECK(linear.interpolate(2.5) == 2.0);
        CHECK(linear.interpolate(3.0) == 2.0);
    }
    SUBCASE("extrapolate")
    {
        CHECK(linear.interpolate(-1.0) == 1.0);
        CHECK(linear.interpolate(100.0) == 2.0);
    }
    SUBCASE("normal")
    {
        const Two_Vector up(0.0, 1.0);
        const Two_Vector slant(-1.0 / root_2, 1.0 / root_2);
        CHECK(linear.normal(-1.0) == up);
        CHECK(linear.normal(0.0) == up);
        CHECK(linear.normal(0.5) == up);
        CHECK(linear.normal(1.0) == up);
        CHECK(linear.normal(1.5) == slant);
        CHECK(linear.normal(2.0) == slant);
        CHECK(linear.normal(2.5) == up);
        CHECK(linear.normal(3.0) == up);
        CHECK(linear.normal(100.0) == up);
    }
}

TEST_CASE("zagzig")
{
    Linear_Interpolator linear{{Two_Vector(0.0, -1.0),
            Two_Vector(1.0, -1.0),
            Two_Vector(2.0, -2.0),
            Two_Vector(3.0, -2.0)}};

    SUBCASE("interpolate")
    {
        CHECK(linear.interpolate(0.0) == -1.0);
        CHECK(linear.interpolate(0.5) == -1.0);
        CHECK(linear.interpolate(1.0) == -1.0);
        CHECK(linear.interpolate(1.5) == -1.5);
        CHECK(linear.interpolate(2.0) == -2.0);
        CHECK(linear.interpolate(2.5) == -2.0);
        CHECK(linear.interpolate(3.0) == -2.0);
    }
    SUBCASE("normal")
    {
        const Two_Vector up(0.0, 1.0);
        const Two_Vector slant(1.0 / root_2, 1.0 / root_2);
        CHECK(linear.normal(-1.0) == up);
        CHECK(linear.normal(0.0) == up);
        CHECK(linear.normal(0.5) == up);
        CHECK(linear.normal(1.0) == up);
        CHECK(linear.normal(1.5) == slant);
        CHECK(linear.normal(2.0) == slant);
        CHECK(linear.normal(2.5) == up);
        CHECK(linear.normal(3.0) == up);
        CHECK(linear.normal(100.0) == up);
    }
}
