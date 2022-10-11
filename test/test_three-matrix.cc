#include "doctest.h"

#include "test.h"

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
    Three_Vector v123{1.0, 2.0, 3.0};
    auto mr = Three_Matrix{1.0}.rotate(v123);
    Three_Vector ones{1.0, 1.0, 1.0};
    Three_Matrix m_side{m1};
    m_side.rotate(z_hat * pi / 2.0);
    Three_Matrix m_corner{m1};
    m_corner.rotate(-ones.unit() * 2.0 * pi / 3.0);

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
        auto m = Three_Matrix{1.0}.rotate(null_v);
        CHECK(close(m1, m, 1e-6));
        CHECK(m * ones == ones);
        CHECK(ones * m == ones);
    }
    SUBCASE("side")
    {
        CHECK(close(m_side[0], std::array{0.0, -1.0, 0.0}, 1e-9));
        CHECK(close(m_side[1], std::array{1.0, 0.0, 0.0}, 1e-9));
        CHECK(close(m_side[2], std::array{0.0, 0.0, 1.0}, 1e-9));
        // Rotates vectors about the z-axis by π/2.
        CHECK(close(m_side * v123, Three_Vector{-2.0, 1.0, 3.0}, 1e-9));
        CHECK(close(v123 * m_side, Three_Vector{2.0, -1.0, 3.0}, 1e-9));
    }
    SUBCASE("corner")
    {
        CHECK(close(m_corner[0], std::array{0.0, 1.0, 0.0}, 1e-9));
        CHECK(close(m_corner[1], std::array{0.0, 0.0, 1.0}, 1e-9));
        CHECK(close(m_corner[2], std::array{1.0, 0.0, 0.0}, 1e-9));
        // Rotates about the cube corners by -1/3 of a rotation.
        CHECK(close(m_corner * x_hat, z_hat, 1e-9));
        CHECK(close(m_corner * y_hat, x_hat, 1e-9));
        CHECK(close(m_corner * z_hat, y_hat, 1e-9));
        CHECK(close(x_hat * m_corner, y_hat, 1e-9));
        CHECK(close(y_hat * m_corner, z_hat, 1e-9));
        CHECK(close(z_hat * m_corner, x_hat, 1e-9));
    }
    SUBCASE("side and corner")
    {
        auto rot{m_side * m_corner};
        CHECK(close(rot * v123, m_side * (m_corner * v123), 1e-9));
        CHECK(close(rot * x_hat, z_hat, 1e-9));
        CHECK(close(rot * y_hat, y_hat, 1e-9));
        CHECK(close(rot * z_hat, -x_hat, 1e-9));
        CHECK(close(x_hat * rot, -z_hat, 1e-9));
        CHECK(close(y_hat * rot, y_hat, 1e-9));
        CHECK(close(z_hat * rot, x_hat, 1e-9));

        m_side *= m_corner;
        CHECK(m_side == rot);
    }
    SUBCASE("axis")
    {
        mr.rotate(-v123);
        CHECK(close(mr, m1, 1e-9));
    }
    SUBCASE("arithmetic")
    {
        auto mr2l{2.0 * mr};
        auto mr2r{mr * 2.0};
        for (size_t i{0}; i < 3; ++i)
            for (size_t j{0}; j < 3; ++j)
            {
                CHECK(mr2l[i][j] == 2.0 * mr[i][j]);
                CHECK(mr2r[i][j] == 2.0 * mr[i][j]);
            }
    }
}
