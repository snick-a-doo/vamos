#include "doctest.h"

#include "test.h"

#include <body/frame.h>

#include <numbers>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace std::numbers;

Three_Vector const null_vector(0.0, 0.0, 0.0);
Three_Vector const ones(1.0, 1.0, 1.0);
Three_Matrix const identity{1.0};
// Rotate the identity 120 degrees about [1,1,1].  This moves z to x,
// x to y and y to z.
auto const rotated{Three_Matrix{1.0}.rotate(ones.unit() * 2.0 * pi / 3.0)};

TEST_CASE("defaults")
{
    Frame frame;
    SUBCASE("initial")
    {
        CHECK(frame.position() == null_vector);
        CHECK(frame.velocity() == null_vector);
        CHECK(frame.orientation() == identity);
        CHECK(frame.angular_velocity() == null_vector);
    }
    SUBCASE("transform from world")
    {
        CHECK(frame.transform_in(ones) == ones);
    }
    SUBCASE("transform to world")
    {
        CHECK(frame.transform_out(ones) == ones);
    }
}

TEST_CASE("position")
{
    Frame frame(Three_Vector(1.0, 2.0, -3.0));
    SUBCASE("initial")
    {
        CHECK(frame.position() == Three_Vector(1.0, 2.0, -3.0));
        CHECK(frame.velocity() == null_vector);
        CHECK(frame.orientation() == identity);
        CHECK(frame.angular_velocity() == null_vector);
    }
    SUBCASE("translate")
    {
        frame.translate(Three_Vector(-1.0, 0.0, 1.0));
        CHECK(frame.position() == Three_Vector(0.0, 2.0, -2.0));
        CHECK(frame.velocity() == null_vector);
        CHECK(frame.orientation() == identity);
        CHECK(frame.angular_velocity() == null_vector);
    }
    SUBCASE("set position")
    {
        frame.set_position(Three_Vector(-1.0, 0.0, 1.0));
        CHECK(frame.position() == Three_Vector(-1.0, 0.0, 1.0));
        CHECK(frame.velocity() == null_vector);
        CHECK(frame.orientation() == identity);
        CHECK(frame.angular_velocity() == null_vector);
    }
    SUBCASE("transform")
    {
        CHECK(frame.transform_in(ones) == Three_Vector(0.0, -1.0, 4.0));
    }
    SUBCASE("transform to world position")
    {
        CHECK(frame.transform_out(ones) == Three_Vector(2.0, 3.0, -2.0));
    }
}

TEST_CASE("orientation")
{
    Frame frame(null_vector, rotated);
    SUBCASE("rotate")
    {
        frame.rotate(-ones.unit() * 2.0 * pi / 3.0);
        CHECK(frame.position() == null_vector);
        CHECK(frame.velocity() == null_vector);
        CHECK(frame.angular_velocity() == null_vector);
        CHECK(close(frame.orientation(), Three_Matrix{1.0}, 1e-9));
    }
    SUBCASE("transform to world")
    {
        auto out = frame.transform_out(Three_Vector(1.0, 0.0, 0.0));
        CHECK(close(out.x, 0.0, 1e-4));
        CHECK(close(out.y, 1.0, 1e-4));
        CHECK(close(out.z, 0.0, 1e-4));
    }
}

TEST_CASE("position and orientation")
{
    Frame frame({1.0, 2.0, -3.0}, rotated);

    SUBCASE("initial position and orientation")
    {
        CHECK(frame.position() == Three_Vector(1.0, 2.0, -3.0));
        CHECK(frame.velocity() == null_vector);
        CHECK(frame.orientation() == rotated);
        CHECK(frame.angular_velocity() == null_vector);
    }
    SUBCASE("transform into position and orientation")
    {
        auto in{frame.transform_in(ones)};
        CHECK(close(in.x, -1.0, 1e-4));
        CHECK(close(in.y, 4.0, 1e-4));
        CHECK(close(in.z, 0.0, 1e-4));
    }
    SUBCASE("rotate out position and orientation")
    {
        auto out{frame.rotate_out({1.0, 0.0, 0.0})};
        CHECK(close(out.x, 0.0, 1e-4));
        CHECK(close(out.y, 1.0, 1e-4)); // x' is y
        CHECK(close(out.z, 0.0, 1e-4));
    }
    SUBCASE("copy rotate out position and orientation")
    {
        Frame g(frame);
        auto out{g.rotate_out({1.0, 0.0, 0.0})};
        CHECK(close(out.x, 0.0, 1e-4));
        CHECK(close(out.y, 1.0, 1e-4)); // x' is y
        CHECK(close(out.z, 0.0, 1e-4));
    }
    SUBCASE("rotate in position and orientation")
    {
        auto out{frame.rotate_in({1.0, 0.0, 0.0})};
        CHECK(close(out.x, 0.0, 1e-4));
        CHECK(close(out.y, 0.0, 1e-4));
        CHECK(close(out.z, 1.0, 1e-4)); // x is z'
    }
}
