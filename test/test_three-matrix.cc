#include "doctest.h"

#include "test.h"

#include <geometry/constants.h>
#include <geometry/three-matrix.h>

using namespace Vamos_Geometry;

TEST_CASE("matrix")
{
    Three_Matrix m;

    SUBCASE("identity")
    {
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 3; j++)
                CHECK(m[i][j] == (i == j));
    }

    m.rotate(Three_Vector(1.0, 2.0, 3.0));

    SUBCASE("assign")
    {
        Three_Matrix g;
        g = m;
        CHECK(m == g);
    }
    SUBCASE("copy")
    {
        Three_Matrix g(m);
        CHECK(m == g);
    }
    SUBCASE("zero")
    {
        m.zero();
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 3; j++)
                CHECK(m[i][j] == 0.0);
    }
    SUBCASE("set identity")
    {
        Three_Matrix g;
        m.identity();
        CHECK(m == g);
    }
}
