#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Rectangle
#include <boost/test/unit_test.hpp>
#include "Numeric.h"

using namespace Vamos_Geometry;

BOOST_AUTO_TEST_CASE (test_sign)
{
  BOOST_CHECK_EQUAL (sign (-5.0), -1.0);
  BOOST_CHECK_EQUAL (sign (0.0), 0.0);
  BOOST_CHECK_EQUAL (sign (30.0), 1.0);
}

BOOST_AUTO_TEST_CASE (test_clip)
{
  BOOST_CHECK_EQUAL (clip (5.0, 8.0, 10.0), 8.0);
  BOOST_CHECK_EQUAL (clip (9.0, 8.0, 10.0), 9.0);
  BOOST_CHECK_EQUAL (clip (15.0, 8.0, 10.0), 10.0);
}

BOOST_AUTO_TEST_CASE (test_is_in_range)
{
  BOOST_CHECK (is_in_range (5.0, 2.0, 8.0));
  BOOST_CHECK (!is_in_range (1.0, 2.0, 8.0));
  BOOST_CHECK (!is_in_range (9.0, 2.0, 8.0));
}

BOOST_AUTO_TEST_CASE (test_closer)
{
  BOOST_CHECK_EQUAL (closer (-5.0, 1.0, 2.0), 1.0);
  BOOST_CHECK_EQUAL (closer (0.9, 1.0, 2.0), 1.0);
  BOOST_CHECK_EQUAL (closer (1.1, 1.0, 2.0), 1.0);
  BOOST_CHECK_EQUAL (closer (1.51, 1.0, 2.0), 2.0);
  BOOST_CHECK_EQUAL (closer (1.9, 1.0, 2.0), 2.0);
  BOOST_CHECK_EQUAL (closer (2.1, 1.0, 2.0), 2.0);
  BOOST_CHECK_EQUAL (closer (21.0, 1.0, 2.0), 2.0);
}

BOOST_AUTO_TEST_CASE (test_average)
{
  BOOST_CHECK_EQUAL (average (1.0, 2.0), 1.5);
  BOOST_CHECK_EQUAL (average (2.0, 1.0), 1.5);
  BOOST_CHECK_EQUAL (average (-2.0, 1.0), -0.5);
}

BOOST_AUTO_TEST_CASE (test_wrap)
{
  BOOST_CHECK_EQUAL (wrap (-1.0, 2.0), 1.0);
  BOOST_CHECK_EQUAL (wrap (0.0, 2.0), 0.0);
  BOOST_CHECK_EQUAL (wrap (1.0, 2.0), 1.0);
  BOOST_CHECK_EQUAL (wrap (2.0, 2.0), 0.0);
  BOOST_CHECK_EQUAL (wrap (3.0, 2.0), 1.0);
  BOOST_CHECK_EQUAL (wrap (0.0, 1.0, 3.0), 2.0);
  BOOST_CHECK_EQUAL (wrap (5.0, 1.0, 3.0), 1.0);
  BOOST_CHECK_EQUAL (wrap (6.0, 1.0, 3.0), 2.0);
  BOOST_CHECK_EQUAL (wrap (7.0, 1.0, 3.0), 1.0);
}

BOOST_AUTO_TEST_CASE (test_branch)
{
  BOOST_CHECK_EQUAL (branch (2.0, 1.0), 2.0);
  BOOST_CHECK_EQUAL (branch (2.0 + 2.0 * pi, 1.0), 2.0);
  BOOST_CHECK_EQUAL (branch (2.0 - 2.0 * pi, 1.0), 2.0);
  BOOST_CHECK_EQUAL (branch (0.0, 1.0), 2.0 * pi);
  BOOST_CHECK_EQUAL (branch (8.0, 1.0), 8.0 - 2.0 * pi);
}

BOOST_AUTO_TEST_CASE (test_intercept)
{
  BOOST_CHECK_EQUAL (intercept (0.0, 1.0, 1.0, 1.0), 0.0);
  BOOST_CHECK_EQUAL (intercept (0.0, 1.0, 2.0, 1.0), 1.0);
  BOOST_CHECK_EQUAL (intercept (0.0, 1.0, 2.0, 4.0), -2.0);
  BOOST_CHECK_EQUAL (intercept (2.0, 1.0, 1.0, 1.0), 2.0);
  BOOST_CHECK_EQUAL (intercept (2.0, 1.0, 2.0, 1.0), 3.0);
  BOOST_CHECK_EQUAL (intercept (2.0, 1.0, 2.0, 4.0), 6.0);
}

BOOST_AUTO_TEST_CASE (test_interpolate)
{
  BOOST_CHECK_EQUAL (interpolate (0.0, 0.0, 0.0, 1.0, 1.0), 0.0);
  BOOST_CHECK_EQUAL (interpolate (0.5, 0.0, 0.0, 1.0, 1.0), 0.5);
  BOOST_CHECK_EQUAL (interpolate (1.0, 0.0, 0.0, 1.0, 1.0), 1.0);
  BOOST_CHECK_EQUAL (interpolate (2.0, 0.0, 0.0, 1.0, 1.0), 2.0);

  BOOST_CHECK_EQUAL (interpolate (2.0, 1.0, 2.0, 3.0, 5.0), 3.5);
  BOOST_CHECK_EQUAL (interpolate (2.0, 1.0, 2.0, 3.0, -5.0), -1.5);
}
