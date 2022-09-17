#include "doctest.h"

#include "test.h"

#include <geometry/rectangle.h>

using namespace Vamos_Geometry;

TEST_CASE("default")
{
    Rectangle<double> r;
    CHECK(r.left() == 0.0);
    CHECK(r.top() == 0.0);
    CHECK(r.right() == 0.0);
    CHECK(r.bottom() == 0.0);
}

TEST_CASE("rectangle")
{
    Rectangle<double> box({1.0, 2.0}, {3.0, -2.0});

    SUBCASE("left")
    {
        box.enclose({{0.0, 1.0}, {1.0, 0.0}});
        CHECK(box.left() == 0.0);
        CHECK(box.top() == 2.0);
        CHECK(box.right() == 3.0);
        CHECK(box.bottom() == -2.0);

        CHECK(box.width() == 3.0);
        CHECK(box.height() == 4.0);
        CHECK(box.center() == Two_Vector{1.5, 0.0});
        CHECK(box.aspect() == 0.75);
    }
    SUBCASE("right")
    {
        box.enclose({{2.0, 1.0}, {4.0, 0.0}});
        CHECK(box.left() == 1.0);
        CHECK(box.top() == 2.0);
        CHECK(box.right() == 4.0);
        CHECK(box.bottom() == -2.0);
    }
    SUBCASE("top")
    {
        box.enclose({{2.0, 3.0}, {3.0, 1.0}});
        CHECK(box.left() == 1.0);
        CHECK(box.top() == 3.0);
        CHECK(box.right() == 3.0);
        CHECK(box.bottom() == -2.0);
    }
    SUBCASE("bottom")
    {
        box.enclose({{2.0, 1.0}, {3.0, -3.0}});
        CHECK(box.left() == 1.0);
        CHECK(box.top() == 2.0);
        CHECK(box.right() == 3.0);
        CHECK(box.bottom() == -3.0);
    }
    SUBCASE("around")
    {
        box.enclose({{0.0, 3.0}, {4.0, -2.0}});
        CHECK(box.left() == 0.0);
        CHECK(box.top() == 3.0);
        CHECK(box.right() == 4.0);
        CHECK(box.bottom() == -2.0);
    }
    SUBCASE("scale down")
    {
        box.scale(0.5);
        CHECK(box == Rectangle<double>({1.5, 1.0}, {2.5, -1.0}));
        CHECK(box.left() == 1.5);
        CHECK(box.top() == 1.0);
        CHECK(box.right() == 2.5);
        CHECK(box.bottom() == -1.0);
    }
    SUBCASE("scale xy")
    {
        box.scale(2.0, 3.0);
        CHECK(box.left() == 0.0);
        CHECK(box.top() == 6.0);
        CHECK(box.right() == 4.0);
        CHECK(box.bottom() == -6.0);
    }
    SUBCASE("clip all")
    {
        box.clip({{1.5, 1.5}, {2.5, -1.5}});
        CHECK(box.left() == 1.5);
        CHECK(box.top() == 1.5);
        CHECK(box.right() == 2.5);
        CHECK(box.bottom() == -1.5);
    }
    SUBCASE("move")
    {
        box.move({1.0, 0.0});
        CHECK(box.left() == 2.0);
        CHECK(box.top() == 2.0);
        CHECK(box.right() == 4.0);
        CHECK(box.bottom() == -2.0);
        box.move({2.0, -3.0});
        CHECK(box.left() == 4.0);
        CHECK(box.top() == -1.0);
        CHECK(box.right() == 6.0);
        CHECK(box.bottom() == -5.0);
    }
}

TEST_CASE("inverted")
{
    // The bottom has a higher value than the top as with pixel coordinates.
    Rectangle<int> box({10, 20}, {30, 60});

    CHECK(box.left() == 10);
    CHECK(box.top() == 20);
    CHECK(box.right() == 30);
    CHECK(box.bottom() == 60);

    CHECK(box.width() == 20);
    CHECK(box.height() == 40);
    CHECK(box.center() == Point{20, 40});
    CHECK(box.aspect() == 0.5);

    SUBCASE("around")
    {
        box.enclose(Rectangle<int>({0, 0}, {40, 80}));
        CHECK(box.left() == 0.0);
        CHECK(box.top() == 0);
        CHECK(box.right() == 40);
        CHECK(box.bottom() == 80);
    }
    SUBCASE("clip lower right")
    {
        box.clip(Rectangle<int>({0, 20}, {20, 30}));
        CHECK(box.left() == 10);
        CHECK(box.top() == 20);
        CHECK(box.right() == 20);
        CHECK(box.bottom() == 30);
    }
    SUBCASE("clip all")
    {
        box.clip(Rectangle<int>({15, 25}, {25, 55}));
        CHECK(box.left() == 15);
        CHECK(box.top() == 25);
        CHECK(box.right() == 25);
        CHECK(box.bottom() == 55);
    }
}
