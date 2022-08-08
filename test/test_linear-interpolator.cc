#include "doctest.h"

#include "test.h"

#include <geometry/constants.h>
#include <geometry/linear-interpolator.h>
#include <geometry/three-vector.h>

#include <numbers>

using namespace Vamos_Geometry;
using namespace std::numbers;

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
        Three_Vector const slant{-sqrt2 / 2.0, sqrt2 / 2.0, 0.0};
        CHECK(linear.normal(-1.0) == up);
        CHECK(linear.normal(0.0) == up);
        CHECK(linear.normal(0.5) == up);
        CHECK(linear.normal(1.0) == up);
        CHECK(close(Three_Vector(linear.normal(1.5)), slant, 1e-9));
        CHECK(close(Three_Vector(linear.normal(2.0)), slant, 1e-9));
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
        Two_Vector const up{0.0, 1.0};
        Three_Vector const slant{sqrt2 / 2.0, sqrt2 / 2.0, 0.0};
        CHECK(linear.normal(-1.0) == up);
        CHECK(linear.normal(0.0) == up);
        CHECK(linear.normal(0.5) == up);
        CHECK(linear.normal(1.0) == up);
        CHECK(close(Three_Vector(linear.normal(1.5)), slant, 1e-9));
        CHECK(close(Three_Vector(linear.normal(2.0)), slant, 1e-9));
        CHECK(linear.normal(2.5) == up);
        CHECK(linear.normal(3.0) == up);
        CHECK(linear.normal(100.0) == up);
    }
}
