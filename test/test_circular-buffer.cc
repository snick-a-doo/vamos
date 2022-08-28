#include "doctest.h"

#include "test.h"

#include <geometry/circular-buffer.h>

#include <string>
#include <sstream>

using Vamos_Geometry::Circular_Buffer;

TEST_CASE("circular buffer")
{
    Circular_Buffer<double, 3> buffer;
    CHECK(buffer.size() == 0);

    buffer.emplace_back(1.0);
    buffer.emplace_back(2.0);
    CHECK(buffer.size() == 2);
    CHECK(buffer[0] == 1.0);
    CHECK(buffer[1] == 2.0);

    buffer.emplace_back (3.0);
    buffer.emplace_back (4.0);
    CHECK(buffer.size() == 3);
    CHECK(buffer[0] == 2.0);
    CHECK(buffer[1] == 3.0);
    CHECK(buffer[2] == 4.0);
}
