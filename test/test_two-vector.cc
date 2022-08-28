#include "doctest.h"

#include "test.h"

#include <geometry/two-vector.h>

#include <sstream>
#include <string>

using Vamos_Geometry::Two_Vector;

TEST_CASE("null")
{
    Two_Vector null;
    CHECK(null.x == 0.0);
    CHECK(null.y == 0.0);
}

TEST_CASE("math")
{
    Two_Vector v1{1.0, 2.0};
    Two_Vector v2{3.0, 4.0};

    SUBCASE("add")
    {
        Two_Vector v3 = v1 + v2;
        CHECK(v3.x == 4.0);
        CHECK(v3.y == 6.0);
    }
    SUBCASE("subtract")
    {
        Two_Vector v3 = v1 - v2;
        CHECK(v3.x == -2.0);
        CHECK(v3.y == -2.0);
    }
    SUBCASE("multiply")
    {
        Two_Vector v3 = v1 * 3;
        CHECK(v3.x == 3.0);
        CHECK(v3.y == 6.0);
        v3 = 3 * v2;
        CHECK(v3.x == 9.0);
        CHECK(v3.y == 12.0);
    }
    SUBCASE("divide")
    {
        Two_Vector v3 = v1 / 2.0;
        CHECK(v3.x == 0.5);
        CHECK(v3.y == 1.0);
    }
}

TEST_CASE("input/output")
{
    Two_Vector v{-2.2, 3.3};

    SUBCASE("input")
    {
        std::string s = "[1.2, -3.4]";
        std::istringstream is(s);
        is >> v;
        CHECK(v.x == 1.2);
        CHECK(v.y == -3.4);
    }
    SUBCASE("output")
    {
        std::ostringstream os;
        os << v;
        CHECK(os.str() == "[-2.2, 3.3]");
    }
}
