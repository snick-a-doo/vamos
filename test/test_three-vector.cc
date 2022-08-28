#include "doctest.h"

#include "test.h"

#include <geometry/three-vector.h>
#include <geometry/two-vector.h>

#include <cmath>
#include <numbers>
#include <sstream>

using namespace Vamos_Geometry;
using namespace std::numbers;

TEST_CASE("vectors")
{
    Three_Vector null;
    Three_Vector x{1.0, 0.0, 0.0};
    Three_Vector y{0.0, 1.0, 0.0};
    Three_Vector z{0.0, 0.0, 1.0};
    Three_Vector v123{1.0, 2.0, 3.0};
    Three_Vector vn321{-3.0, -2.0, -1.0};
    Three_Vector vxy{2.0, -pi/4.0};
    Three_Vector v12{Two_Vector{1.0, 2.0}};

    SUBCASE("construct")
    {
        CHECK(close(vxy, Three_Vector{sqrt2, -sqrt2, 0.0}, 1e-9));
        CHECK(v12 == Three_Vector{1.0, 2.0, 0.0});
    }
    SUBCASE("magnitude")
    {
        CHECK(null.magnitude() == 0.0);
        CHECK(x.magnitude() == 1.0);
        CHECK(y.magnitude() == 1.0);
        CHECK(z.magnitude() == 1.0);
        CHECK(v123.magnitude() == sqrt(14.0));
        CHECK(vn321.magnitude() == sqrt(14.0));
        CHECK(vxy.magnitude() == 2.0);
        CHECK(v12.magnitude() == sqrt(5.0));
    }
    SUBCASE("unit")
    {
        CHECK(null.unit() == Three_Vector::Z);
        CHECK(x.unit() == Three_Vector::X);
        CHECK(y.unit() == Three_Vector::Y);
        CHECK(z.unit() == Three_Vector::Z);
        CHECK(v123.unit().magnitude() == 1.0);
        CHECK(vn321.unit().magnitude() == 1.0);
        CHECK(close(vxy.unit(), Three_Vector{sqrt2 / 2.0, -sqrt2 / 2.0, 0.0}, 1e-9));
        CHECK(close(v12.unit().magnitude(), 1.0, 1e-9));
    }
    SUBCASE("null and zero")
    {
        CHECK(!v123.is_null());
        CHECK(null.is_null());
        CHECK(v123.zero() == null);
    }
    SUBCASE("dot product")
    {
        CHECK(null.dot(v123) == 0.0);
        CHECK(x.dot(x) == 1.0);
        CHECK(x.dot(y) == 0.0);
        CHECK(v123.dot(vn321) == -10.0);
        CHECK(v123.dot(z) == 3.0);
        CHECK(vxy.dot(z) == 0.0);
    }
    SUBCASE("cross product")
    {
        CHECK(null.cross(vn321) == Three_Vector::ZERO);
        CHECK(y.cross(y) == Three_Vector::ZERO);
        CHECK(x.cross(y) == Three_Vector::Z);
        CHECK(y.cross(x) == -Three_Vector::Z);
        CHECK(v123.cross(vn321) == Three_Vector{4.0, -8.0, 4.0});
        CHECK(v123.cross(z) == Three_Vector{2.0, -1.0, 0.0});
        CHECK(close(vxy.cross(z), Three_Vector{-sqrt2, -sqrt2, 0.0}, 1e-9));
    }
    SUBCASE("project")
    {
        CHECK(null.project(v123) == Three_Vector::ZERO);
        CHECK(v123.project(null) == Three_Vector::ZERO);
        CHECK(z.project(z) == z);
        CHECK(x.project(y) == Three_Vector::ZERO);
        CHECK(v123.project(z) == 3.0 * Three_Vector::Z);
        CHECK(close(y.project(vxy), Three_Vector(-0.5, 0.5, 0.0), 1e-9));
    }
    SUBCASE("back project")
    {
        CHECK(null.back_project(v123) == Three_Vector::ZERO);
        CHECK(v123.back_project(null) == Three_Vector::ZERO);
        CHECK(z.back_project(z) == z);
        CHECK(x.back_project(y) == Three_Vector::ZERO);
        CHECK(z.back_project(v123) == v123 / 3.0);
        CHECK(close(vxy.back_project(y), -2.0 * sqrt2 * Three_Vector::Y, 1e-9));
    }
    SUBCASE("arithmetic")
    {
        CHECK(null + v123 == v123);
        CHECK(vn321 - null == vn321);
        CHECK(v123 + vn321 == Three_Vector{-2.0, 0.0, 2.0});
        CHECK(v123 - vn321 == Three_Vector{4.0, 4.0, 4.0});
        CHECK(-v123 + v123 == Three_Vector::ZERO);
        CHECK(0.0 * v123 == Three_Vector::ZERO);
        CHECK(v123 * 0.0 == Three_Vector::ZERO);
        CHECK(2.0 * x + y * 3.0 + -5.0 * z == Three_Vector{2.0, 3.0, -5.0});
        CHECK(null / 3.0 == null);
        CHECK(vn321 / -2.0 == Three_Vector(1.5, 1.0, 0.5));
        CHECK_THROWS_AS(vxy / 0.0, std::overflow_error);

        null += v123;
        vn321 -= Three_Vector::ZERO;
        v123 += v12;
        v12 *= 10.0;
        vxy /= -sqrt2;
        CHECK(null == Three_Vector{1.0, 2.0, 3.0});
        CHECK(vn321 == Three_Vector{-3.0, -2.0, -1.0});
        CHECK(v123 == Three_Vector{2.0, 4.0, 3.0});
        CHECK(v12 == Three_Vector{10.0, 20.0, 0.0});
        CHECK(close(vxy, Three_Vector{-1.0, 1.0, 0.0}, 1e-9));
    }
    SUBCASE("input")
    {
        std::istringstream is("[ 1.2,  -3.4; 5]");
        Three_Vector v;
        is >> v;
        CHECK(v == Three_Vector{1.2, -3.4, 5.0});
    }
    SUBCASE("output")
    {
        std::ostringstream os;
        os << v123;
        CHECK(os.str() == "[1, 2, 3]");
    }
}
