#include "doctest.h"

#include "test.h"

#include <geometry/constants.h>
#include <geometry/linear-interpolator.h>
#include <geometry/three-vector.h>

#include <numbers>

using namespace Vamos_Geometry;
using namespace std::numbers;

TEST_CASE("linear")
{
    Linear_Interpolator linear;
    CHECK(linear.empty());
    linear.load(1.0, 1.0);

    SUBCASE("single")
    {
        CHECK(linear.interpolate(-5.5) == 1.0);
        CHECK(linear.interpolate(0.0) == 1.0);
        CHECK(linear.interpolate(1.0) == 1.0);
        CHECK(linear.interpolate(5.5) == 1.0);
    }

    SUBCASE("angle")
    {
        linear.load(2.0, 3.0);
        CHECK(linear.interpolate(0.0) == 1.0);
        CHECK(linear.interpolate(1.0) == 1.0);
        CHECK(linear.interpolate(1.5) == 2.0);
        CHECK(linear.interpolate(2.0) == 3.0);
        CHECK(linear.interpolate(100.0) == 3.0);
    }

    SUBCASE("zigzag")
    {
        linear.load({Two_Vector{2.0, 2.0}, Two_Vector{3.0, 2.0}});

        CHECK(linear.interpolate(0.0) == 1.0);
        CHECK(linear.interpolate(0.5) == 1.0);
        CHECK(linear.interpolate(1.0) == 1.0);
        CHECK(linear.interpolate(1.5) == 1.5);
        CHECK(linear.interpolate(2.0) == 2.0);
        CHECK(linear.interpolate(2.5) == 2.0);
        CHECK(linear.interpolate(3.0) == 2.0);
        // extrapolate
        CHECK(linear.interpolate(-1.0) == 1.0);
        CHECK(linear.interpolate(100.0) == 2.0);

        Two_Vector up(0.0, 1.0);
        Three_Vector slant{-sqrt2 / 2.0, sqrt2 / 2.0, 0.0};
        CHECK(linear.normal(-1.0) == up);
        CHECK(linear.normal(0.0) == up);
        CHECK(linear.normal(0.5) == up);
        CHECK(linear.normal(1.0) == up);
        CHECK(close(Three_Vector(linear.normal(1.5)), slant, 1e-9));
        CHECK(close(Three_Vector(linear.normal(2.0)), slant, 1e-9));
        CHECK(linear.normal(2.5) == up);
        CHECK(linear.normal(3.0) == up);
        CHECK(linear.normal(100.0) == up);

        linear.scale(2.0);
        linear.shift(-1.0);
        CHECK(linear.interpolate(3.0) == 0.5);
        CHECK(linear.interpolate(1.0) == 0.0);
        Three_Vector slant2{-1.0 / std::sqrt(5.0), 2.0 / std::sqrt(5.0), 0.0};
        CHECK(close(Three_Vector{linear.normal(2.5)}, slant2, 1e-9));

        linear.clear();
        CHECK(linear.empty());
        linear.load({Two_Vector{1.0, 1.0}, Two_Vector{2.0, 2.0}});
        CHECK(linear.interpolate(1.5) == 1.5);
    }

    SUBCASE("zagzig")
    {
        Linear_Interpolator linear{{Two_Vector(0.0, -1.0), Two_Vector(1.0, -1.0),
                                        Two_Vector(2.0, -2.0), Two_Vector(3.0, -3.0)}};
        CHECK(linear.size() == 4);
        CHECK(!linear.empty());
        CHECK(linear[2] == Two_Vector{2.0, -2.0});
        linear[3] = Two_Vector{3.0, -2.0};

        CHECK(linear.interpolate(0.0) == -1.0);
        CHECK(linear.interpolate(0.5) == -1.0);
        CHECK(linear.interpolate(1.0) == -1.0);
        CHECK(linear.interpolate(1.5) == -1.5);
        CHECK(linear.interpolate(2.0) == -2.0);
        CHECK(linear.interpolate(2.5) == -2.0);
        CHECK(linear.interpolate(3.0) == -2.0);

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
