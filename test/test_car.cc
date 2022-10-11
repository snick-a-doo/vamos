#include "doctest.h"

#include "test.h"

#include <body/car.h>

using namespace Vamos_Body;
using namespace Vamos_Geometry;

// bool vector_equal (const Three_Vector& v1, const Three_Vector& v2)
// {
//   if ((v1 - v2).magnitude () > 1e-6)
//     {
//       std::cerr << v1 << " != " << v2 << std::endl;
//       // Can't do the obvious std::cerr << v1 << " != " << v2 << std::endl;
//       // because << needs to be overloaded in namespace std for boost.  See
//       // comment at end of Three_Vector.h.
//       return false;
//     }
//   return true;
//}

TEST_CASE("crash box")
{
    Car::Crash_Box crash_box{1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
    Three_Vector v(1.0, 1.0, 1.0);

    SUBCASE("outside")
    {
        Three_Vector x(2.0, 2.0, 2.0);
        CHECK(crash_box.penetration(x, v, false) == null_v);
        x = Three_Vector(-2.0, -2.0, -2.0);
        CHECK(crash_box.penetration(x, v, false) == null_v);
        x = Three_Vector(-2.0, 2.0, 2.0);
        CHECK(crash_box.penetration(x, v, false) == null_v);
        x = Three_Vector(2.0, -2.0, -2.0);
        CHECK(crash_box.penetration(x, v, false) == null_v);

        x = Three_Vector(2.0, 0.5, 0.5);
        CHECK(crash_box.penetration(x, v, false) == null_v);
        x = Three_Vector(-2.0, 0.5, 0.5);
        CHECK(crash_box.penetration(x, v, false) == null_v);
        x = Three_Vector(0.5, 2.0, 0.5);
        CHECK(crash_box.penetration(x, v, false) == null_v);
        x = Three_Vector(0.5, -2.0, 0.5);
        CHECK(crash_box.penetration(x, v, false) == null_v);
        x = Three_Vector(0.5, 0.5, 2.0);
        CHECK(crash_box.penetration(x, v, false) == null_v);
        x = Three_Vector(0.5, 0.5, -2.0);
        CHECK(crash_box.penetration(x, v, false) == null_v);
    }
    SUBCASE("inside")
    {
        Three_Vector x(0.1, 0.5, 0.5);
        CHECK(crash_box.penetration(x, v, false) == Three_Vector(-0.1, 0.0, 0.0));
        x = Three_Vector(0.5, 0.1, 0.5);
        CHECK(crash_box.penetration(x, v, false) == Three_Vector(0.0, -0.1, 0.0));
        x = Three_Vector(0.5, 0.5, 0.1);
        CHECK(crash_box.penetration(x, v, false) == Three_Vector(0.0, 0.0, -0.1));

        x = Three_Vector(0.9, 0.5, 0.5);
        v = Three_Vector(-1.0, -1.0, -1.0);
        CHECK(close(crash_box.penetration(x, v, false), Three_Vector(0.1, 0.0, 0.0), 1e-9));
        x = Three_Vector(0.5, 0.9, 0.5);
        CHECK(close(crash_box.penetration(x, v, false), Three_Vector(0.0, 0.1, 0.0), 1e-9));
        x = Three_Vector(0.5, 0.5, 0.9);
        CHECK(close(crash_box.penetration(x, v, false), Three_Vector(0.0, 0.0, 0.1), 1e-9));
    }
}
