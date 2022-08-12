#include "doctest.h"

#include "test.h"

#include <body/frame.h>
#include <geometry/constants.h>

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
        CHECK(frame.transform_from_world(ones) == ones);
    }
    SUBCASE("transform to world")
    {
        CHECK(frame.transform_to_world(ones) == ones);
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
        CHECK(frame.transform_from_world(ones) == Three_Vector(0.0, -1.0, 4.0));
    }
    SUBCASE("transform to world position")
    {
        CHECK(frame.transform_to_world(ones) == Three_Vector(2.0, 3.0, -2.0));
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
        auto out = frame.transform_to_world(Three_Vector(1.0, 0.0, 0.0));
        CHECK(close(out.x, 0.0, 1e-4));
        CHECK(close(out.y, 1.0, 1e-4));
        CHECK(close(out.z, 0.0, 1e-4));
    }
}

TEST_CASE("position and orientation")
{
    Frame frame(Three_Vector(1.0, 2.0, -3.0), rotated);

    SUBCASE("initial_position_and_orientation")
    {
        CHECK(frame.position() == Three_Vector(1.0, 2.0, -3.0));
        CHECK(frame.velocity() == null_vector);
        CHECK(frame.orientation() == rotated);
        CHECK(frame.angular_velocity() == null_vector);
    }
    SUBCASE("transform_into_position_and_orientation")
    {
        auto in = frame.transform_from_world(ones);
        CHECK(close(in.x, -1.0, 1e-4));
        CHECK(close(in.y, 4.0, 1e-4));
        CHECK(close(in.z, 0.0, 1e-4));
    }
    SUBCASE("transform_to_world_position_and_orientation")
    {
        auto out = frame.transform_to_world(Three_Vector(1.0, 0.0, 0.0));
        CHECK(close(out.x, 1.0, 1e-4));
        CHECK(close(out.y, 3.0, 1e-4));
        CHECK(close(out.z, -3.0, 1e-4));
    }
    SUBCASE("rotate_out_position_and_orientation")
    {
        auto out = frame.rotate_to_parent(Three_Vector(1.0, 0.0, 0.0));
        CHECK(close(out.x, 0.0, 1e-4));
        CHECK(close(out.y, 1.0, 1e-4)); // x' is y
        CHECK(close(out.z, 0.0, 1e-4));
    }
    SUBCASE("copy_rotate_out_position_and_orientation")
    {
        Frame g(frame);
        Three_Vector out = g.rotate_to_parent(Three_Vector(1.0, 0.0, 0.0));
        CHECK(close(out.x, 0.0, 1e-4));
        CHECK(close(out.y, 1.0, 1e-4)); // x' is y
        CHECK(close(out.z, 0.0, 1e-4));
    }
    SUBCASE("rotate_in_position_and_orientation")
    {
        auto out = frame.rotate_from_parent(Three_Vector(1.0, 0.0, 0.0));
        CHECK(close(out.x, 0.0, 1e-4));
        CHECK(close(out.y, 0.0, 1e-4));
        CHECK(close(out.z, 1.0, 1e-4)); // x is z'
    }
}

TEST_CASE("nested frame")
{
    Frame parent(Three_Vector(1.0, 2.0, 3.0));
    Frame frame(Three_Vector(2.0, 3.0, 4.0), &parent);

    SUBCASE("transform_to_parent")
    {
        CHECK(frame.transform_to_parent(Three_Vector(3.0, 4.0, 5.0))
              == Three_Vector(5.0, 7.0, 9.0));
    }
    SUBCASE("transform_to_world")
    {
        CHECK(frame.transform_to_world(Three_Vector(3.0, 4.0, 5.0))
              == Three_Vector(6.0, 9.0, 12.0));
    }
    SUBCASE("transform_from_parent")
    {
        CHECK(frame.transform_from_parent(Three_Vector(3.0, 4.0, 5.0))
              == Three_Vector(1.0, 1.0, 1.0));
    }
    SUBCASE("transform_from_world")
    {
        CHECK(frame.transform_from_world(Three_Vector(3.0, 4.0, 5.0))
              == Three_Vector(0.0, -1.0, -2.0));
    }
}
