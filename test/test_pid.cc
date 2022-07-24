#include "doctest.h"

#include "test.h"

#include <geometry/pid.h>

using namespace Vamos_Geometry;

TEST_CASE("proportional")
{
    PID pid(2.0, 0.0, 0.0);
    pid.set(10.0);
    CHECK(close(pid.propagate(0.0, 0.1), 20.0, 1e-6));
    CHECK(close(pid.propagate(0.0, 0.1), 20.0, 1e-6));
    CHECK(close(pid.propagate(5.0, 0.1), 10.0, 1e-6));
    CHECK(close(pid.propagate(5.0, 0.1), 10.0, 1e-6));
    CHECK(close(pid.propagate(10.0, 0.1), 0.0, 1e-6));
    CHECK(close(pid.propagate(15.0, 0.5), -10.0, 1e-6));
    CHECK(close(pid.propagate(10.0, 0.5), 0.0, 1e-6));
}

TEST_CASE("integral")
{
    PID pid(0.0, 2.0, 0.0);
    pid.set(10.0);
    CHECK(close(pid.propagate(0.0, 0.1), 2.0, 1e-6));
    CHECK(close(pid.propagate(0.0, 0.1), 4.0, 1e-6));
    CHECK(close(pid.propagate(5.0, 0.1), 5.0, 1e-6));
    CHECK(close(pid.propagate(5.0, 0.1), 6.0, 1e-6));
    CHECK(close(pid.propagate(10.0, 0.1), 6.0, 1e-6));
    CHECK(close(pid.propagate(15.0, 0.5), 1.0, 1e-6));
    CHECK(close(pid.propagate(10.0, 0.5), 1.0, 1e-6));
}

TEST_CASE("derivative")
{
    PID pid(0.0, 0.0, 2.0);
    pid.set(10.0);
    CHECK(close(pid.propagate(0.0, 0.1), 0.0, 1e-6));
    CHECK(close(pid.propagate(0.0, 0.1), 0.0, 1e-6));
    CHECK(close(pid.propagate(5.0, 0.1), -100.0, 1e-6));
    CHECK(close(pid.propagate(5.0, 0.1), 0.0, 1e-6));
    CHECK(close(pid.propagate(10.0, 0.1), -100.0, 1e-6));
    CHECK(close(pid.propagate(15.0, 0.5), -20.0, 1e-6));
    CHECK(close(pid.propagate(10.0, 0.5), 20.0, 1e-6));
}

TEST_CASE("zero delta t")
{
    PID pid(2.0, 3.0, 4.0);
    pid.set(10.0);
    CHECK(close(pid.propagate(0.0, 0.0), 20.0, 1e-6));
}

TEST_CASE("change setpoint")
{
    PID pid(2.0, 3.0, 4.0);
    pid.set(10.0);
    CHECK(close(pid.propagate(0.0, 1.0), 20.0 + 30.0 + 0.0, 1e-6));
    pid.set(5.0);
    CHECK(close(pid.propagate(0.0, 1.0), 10.0 + 45.0 - 20.0, 1e-6));
}

TEST_CASE("reset")
{
    PID pid(2.0, 3.0, 4.0);
    pid.set(10.0);
    CHECK(close(pid.propagate(0.0, 1.0), 20.0 + 30.0 + 0.0, 1e-6));
    pid.set(5.0);
    pid.reset();
    CHECK(close(pid.propagate(0.0, 1.0), 10.0 + 15.0 + 0.0, 1e-6));
}
