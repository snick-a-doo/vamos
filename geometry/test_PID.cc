#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE PID
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include "PID.h"

#include <fstream>

using namespace Vamos_Geometry;

BOOST_AUTO_TEST_CASE (proportional)
{
  PID pid (2.0, 0.0, 0.0);
  pid.set (10.0);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 0.1),   20.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 0.1),   20.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (5.0, 0.1),   10.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (5.0, 0.1),   10.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (10.0, 0.1),   0.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (15.0, 0.5), -10.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (10.0, 0.5),   0.0, 1e-6);
}

BOOST_AUTO_TEST_CASE (integral)
{
  PID pid (0.0, 2.0, 0.0);
  pid.set (10.0);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 0.1),  2.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 0.1),  4.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (5.0, 0.1),  5.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (5.0, 0.1),  6.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (10.0, 0.1), 6.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (15.0, 0.5), 1.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (10.0, 0.5), 1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE (derivative)
{
  PID pid (0.0, 0.0, 2.0);
  pid.set (10.0);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 0.1),     0.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 0.1),     0.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (5.0, 0.1),  -100.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (5.0, 0.1),     0.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (10.0, 0.1), -100.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (15.0, 0.5),  -20.0, 1e-6);
  BOOST_CHECK_CLOSE (pid.propagate (10.0, 0.5),   20.0, 1e-6);
}

BOOST_AUTO_TEST_CASE (zero_delta_t)
{
  PID pid (2.0, 3.0, 4.0);
  pid.set (10.0);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 0.0), 20.0, 1e-6);
}

BOOST_AUTO_TEST_CASE (change_setpoint)
{
  PID pid (2.0, 3.0, 4.0);
  pid.set (10.0);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 1.0), 20.0 + 30.0 + 0.0, 1e-6);
  pid.set (5.0);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 1.0), 10.0 + 45.0 - 20.0, 1e-6);
}

BOOST_AUTO_TEST_CASE (reset)
{
  PID pid (2.0, 3.0, 4.0);
  pid.set (10.0);
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 1.0), 20.0 + 30.0 + 0.0, 1e-6);
  pid.set (5.0);
  pid.reset ();
  BOOST_CHECK_CLOSE (pid.propagate (0.0, 1.0), 10.0 + 15.0 + 0.0, 1e-6);
}
