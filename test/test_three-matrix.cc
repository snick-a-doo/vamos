#include "doctest.h"

#include "test.h"

#include <geometry/constants.h>
#include <geometry/three-matrix.h>

#include <numbers>

using namespace Vamos_Geometry;
using namespace std::numbers;

/// Test arrays for approximate equality.
bool close(std::array<double, 3> const& actual,
           std::array<double, 3> const& expected,
           double tol)
{
    for (size_t i{0}; i < 3; ++i)
        if (!close(actual[i], expected[i], tol))
            return false;
    return true;
}

TEST_CASE("matrix")
{
    Three_Matrix m1{1.0};
    Three_Vector axis{1.0, 2.0, 3.0};
    auto mr = Three_Matrix{1.0}.rotate(axis);

    SUBCASE("identity")
    {
        for (size_t i{0}; i < 3; ++i)
            for (size_t j{0}; j < 3; ++j)
                CHECK(m1[i][j] == (i == j));
        CHECK(mr.identity() == m1);
    }
    SUBCASE("zero")
    {
        Three_Matrix m0{0.0};
        for (size_t i{0}; i < 3; ++i)
            for (size_t j{0}; j < 3; ++j)
                CHECK(m0[i][j] == 0.0);
        CHECK(mr.zero() == m0);
    }
    SUBCASE("transpose")
    {
        auto mt(transpose(mr));
        for (size_t i{0}; i < 3; ++i)
            for (size_t j{0}; j < 3; ++j)
                CHECK(mt[i][j] == mr[j][i]);
        CHECK(transpose(mt) == mr);
    }
    SUBCASE("invert")
    {
        auto mi(invert(mr));
        CHECK(close(mi * mr, m1, 1e-9));
    }
    SUBCASE("no rotation")
    {
        auto m = Three_Matrix{1.0}.rotate(Three_Vector::ZERO);
        CHECK(close(m1, m, 1e-6));
    }
    SUBCASE("side")
    {
        m1.rotate(Three_Vector::Z * pi / 2.0);
        CHECK(close(m1[0], std::array{0.0, -1.0, 0.0}, 1e-9));
        CHECK(close(m1[1], std::array{1.0, 0.0, 0.0}, 1e-9));
        CHECK(close(m1[2], std::array{0.0, 0.0, 1.0}, 1e-9));
    }
    SUBCASE("corner")
    {
        Three_Vector ones{1.0, 1.0, 1.0};
        m1.rotate(-ones.unit() * 2.0 * pi / 3.0);
        CHECK(close(m1[0], std::array{0.0, 1.0, 0.0}, 1e-9));
        CHECK(close(m1[1], std::array{0.0, 0.0, 1.0}, 1e-9));
        CHECK(close(m1[2], std::array{1.0, 0.0, 0.0}, 1e-9));
    }
    SUBCASE("axis")
    {
        mr.rotate(-axis);
        CHECK(close(mr, m1, 1e-9));
    }
}
