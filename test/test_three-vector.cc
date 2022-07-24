#include "doctest.h"

#include "test.h"

#include <geometry/constants.h>
#include <geometry/three-vector.h>

using namespace Vamos_Geometry;

TEST_CASE("null vector")
{
    Three_Vector null;
    CHECK(null.magnitude() == 0.0);
    CHECK(null.null());
}

TEST_CASE("unit vectors")
{
    Three_Vector x{1.0, 0.0, 0.0};
    Three_Vector y{0.0, 1.0, 0.0};
    Three_Vector z{0.0, 0.0, 1.0};

    SUBCASE("dot products")
    {
        CHECK(x.dot(y) == 0.0);
        CHECK(x.dot(z) == 0.0);
        CHECK(y.dot(x) == 0.0);
        CHECK(y.dot(z) == 0.0);
        CHECK(z.dot(x) == 0.0);
        CHECK(z.dot(y) == 0.0);
    }
    SUBCASE("cross products")
    {
        CHECK(x.cross(y) == z);
        CHECK(x.cross(z) == -y);
        CHECK(y.cross(x) == -z);
        CHECK(y.cross(z) == x);
        CHECK(z.cross(x) == y);
        CHECK(z.cross(y) == -x);
    }
    SUBCASE("add")
    {
        CHECK(x + y + z == Three_Vector{1.0, 1.0, 1.0});
    }
    SUBCASE("subtract")
    {
        CHECK(Three_Vector(1.0, 1.0, 1.0) - x - y == z);
    }
    SUBCASE("projection")
    {
        CHECK(close((x.project(x)).magnitude(), 1.0, 1e-4));
        CHECK(close((x.project(y)).magnitude(), 0.0, 1e-4));
        CHECK(close((x.project(z)).magnitude(), 0.0, 1e-4));
    }
    SUBCASE("rotate z")
    {
        Three_Vector r = x.rotate(Three_Vector(0.0, 0.0, pi / 2.0));
        CHECK(close(r.x, 0.0, 1e-4));
        CHECK(close(x.x, 0.0, 1e-4));
        CHECK(close(r.y, 1.0, 1e-4));
        CHECK(close(x.y, 1.0, 1e-4));
        CHECK(close(r.z, 0.0, 1e-4));
        CHECK(close(x.z, 0.0, 1e-4));
    }
    SUBCASE("rotate y")
    {
        Three_Vector r = z.rotate(Three_Vector(0.0, pi / 2.0, 0.0));
        CHECK(close(r.x, 1.0, 1e-4));
        CHECK(close(r.y, 0.0, 1e-4));
        CHECK(close(r.z, 0.0, 1e-4));
    }
    SUBCASE("rotate x")
    {
        Three_Vector r = y.rotate(Three_Vector(pi / 2.0, 0.0, 0.0));
        CHECK(close(r.x, 0.0, 1e-6));
        CHECK(close(r.y, 0.0, 1e-6));
        CHECK(close(r.z, 1.0, 1e-6));
    }
}

TEST_CASE("1 2 3")
{
    Three_Vector v123{1.0, 2.0, 3.0};
    Three_Vector v321{3.0, 2.0, 1.0};

    SUBCASE("zero")
    {
        v123.zero();
        CHECK(v123.null());
    }
    SUBCASE("negate")
    {
        CHECK(-v123 == Three_Vector(-1.0, -2.0, -3.0));
    }
    SUBCASE("dot")
    {
        CHECK(close(v123.dot(v321), 10.0, 1e-6));
    }
}

TEST_CASE("xy plane")
{
    Three_Vector x{1.0, 0.0, 0.0};
    Three_Vector y{0.0, 1.0, 0.0};
    Three_Vector xy{1.0, 1.0, 0.0};

    SUBCASE("project")
    {
        Three_Vector v = x.project(xy);
        CHECK(close(v.x, 0.5, 1e-6));
        CHECK(close(v.y, 0.5, 1e-6));
        CHECK(close(v.z, 0.0, 1e-6));
    }
    SUBCASE("back project")
    {
        Three_Vector v = x.back_project(xy);
        CHECK(close(v.x, 1.0, 1e-6));
        CHECK(close(v.y, 1.0, 1e-6));
        CHECK(close(v.z, 0.0, 1e-6));
        // Projecting and back-projecting should leave the vector unchanged.
        v = x.back_project(x.project(xy));
        CHECK(close(v.x, xy.x, 1e-6));
        CHECK(close(v.y, xy.y, 1e-6));
        CHECK(close(v.z, xy.z, 1e-6));
    }
}

TEST_CASE("length and angle")
{
    Three_Vector v{2.0, pi / 3.0};

    CHECK(close(v.x, 1.0, 1e-6));
    CHECK(close(v.y, 1.73205, 1e-6));
    CHECK(close(v.z, 0.0, 1e-6));
}

TEST_CASE("vector and point")
{
    Three_Vector v{3.0, 3.0, 0.0};
    Three_Vector p{sqrt(2.0), 0.0, 0.0};

    SUBCASE("perp distance")
    {
        CHECK(close(v.perp_distance(p), 1.0, 1e-6));
        v.y = -3.0;
        CHECK(close(v.perp_distance(p), 1.0, 1e-6));
    }
}
