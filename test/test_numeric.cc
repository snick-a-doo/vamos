#include "doctest.h"

#include "test.h"

#include <geometry/numeric.h>

#include <set>

using namespace Vamos_Geometry;

TEST_CASE("sign")
{
    CHECK(sign(-5.0) == -1.0);
    CHECK(sign(0.0) == 0.0);
    CHECK(sign(30.0) == 1.0);
}

TEST_CASE("clip")
{
    CHECK(clip(5.0, 8.0, 10.0) == 8.0);
    CHECK(clip(9.0, 8.0, 10.0) == 9.0);
    CHECK(clip(15.0, 8.0, 10.0) == 10.0);
}

TEST_CASE("is_in_range")
{
    CHECK(is_in_range(5.0, 2.0, 8.0));
    CHECK(!is_in_range(1.0, 2.0, 8.0));
    CHECK(!is_in_range(9.0, 2.0, 8.0));
}

TEST_CASE("wrap")
{
    CHECK(wrap(-1.0, 2.0) == 1.0);
    CHECK(wrap(0.0, 2.0) == 0.0);
    CHECK(wrap(1.0, 2.0) == 1.0);
    CHECK(wrap(2.0, 2.0) == 0.0);
    CHECK(wrap(3.0, 2.0) == 1.0);
    CHECK(wrap(0.0, 1.0, 3.0) == 2.0);
    CHECK(wrap(5.0, 1.0, 3.0) == 1.0);
    CHECK(wrap(6.0, 1.0, 3.0) == 2.0);
    CHECK(wrap(7.0, 1.0, 3.0) == 1.0);

    CHECK_THROWS_AS(wrap(1.0, 0.0), Vamos_Geometry::Bad_Argument<double>);
}

TEST_CASE("branch")
{
    using namespace std::numbers;
    CHECK(branch(2.0, 1.0) == 2.0);
    CHECK(branch(2.0 + 2.0 * pi, 1.0) == 2.0);
    CHECK(branch(2.0 - 2.0 * pi, 1.0) == 2.0);
    CHECK(branch(0.0, 1.0) == 2.0 * pi);
    CHECK(branch(8.0, 1.0) == 8.0 - 2.0 * pi);
}

TEST_CASE("intercept")
{
    CHECK(intercept(0.0, 1.0, 1.0, 1.0) == 0.0);
    CHECK(intercept(0.0, 1.0, 2.0, 1.0) == 1.0);
    CHECK(intercept(0.0, 1.0, 2.0, 4.0) == -2.0);
    CHECK(intercept(2.0, 1.0, 1.0, 1.0) == 2.0);
    CHECK(intercept(2.0, 1.0, 2.0, 1.0) == 3.0);
    CHECK(intercept(2.0, 1.0, 2.0, 4.0) == 6.0);
}

TEST_CASE("interpolate")
{
    CHECK(interpolate(0.0, 0.0, 0.0, 1.0, 1.0) == 0.0);
    CHECK(interpolate(0.5, 0.0, 0.0, 1.0, 1.0) == 0.5);
    CHECK(interpolate(1.0, 0.0, 0.0, 1.0, 1.0) == 1.0);
    CHECK(interpolate(2.0, 0.0, 0.0, 1.0, 1.0) == 2.0);

    CHECK(interpolate(2.0, 1.0, 2.0, 3.0, 5.0) == 3.5);
    CHECK(interpolate(2.0, 1.0, 2.0, 3.0, -5.0) == -1.5);
}

TEST_CASE("random")
{
    // Generate 100 random numbers. Divide the range into 10 bins. Check that there's at
    // least one number in each bin.
    std::set<double> rands;
    for (int i{0}; i < 100; ++i)
        rands.insert(random_in_range(-5.0, 5.0));
    auto low = -5.0;
    auto high = -4.0;
    bool found = false;
    for (auto r : rands)
    {
        if (r > high)
        {
            ++high;
            ++low;
            found = false;
        }
        if (!found)
        {
            CHECK(r < high);
            found = true;
        }
    }
}
