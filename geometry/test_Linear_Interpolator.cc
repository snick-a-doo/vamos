#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Linear_Interpolator
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include "Linear_Interpolator.h"
#include "Constants.h"

using namespace Vamos_Geometry;

struct Angle_Fixture
{
  Angle_Fixture () 
  {
    linear.load (Two_Vector (0.0, 0.0));
    linear.load (Two_Vector (1.0, 1.0));
  }
  Linear_Interpolator linear;
};

struct Zigzag_Fixture
{
  Zigzag_Fixture () 
  {
    linear.load (Two_Vector (0.0, 1.0));
    linear.load (Two_Vector (1.0, 1.0));
    linear.load (Two_Vector (2.0, 2.0));
    linear.load (Two_Vector (3.0, 2.0));
  }
  Linear_Interpolator linear;
};

struct Zagzig_Fixture
{
  Zagzig_Fixture () 
  {
    linear.load (Two_Vector (0.0, -1.0));
    linear.load (Two_Vector (1.0, -1.0));
    linear.load (Two_Vector (2.0, -2.0));
    linear.load (Two_Vector (3.0, -2.0));
  }
  Linear_Interpolator linear;
};

BOOST_AUTO_TEST_CASE (test_angle_interpolation)
{
  Angle_Fixture f;
  BOOST_CHECK_EQUAL (f.linear.interpolate (0.0), 0.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (0.5), 0.5);
  BOOST_CHECK_EQUAL (f.linear.interpolate (1.0), 1.0);
}

BOOST_AUTO_TEST_CASE (test_angle_extrapolation)
{
  Angle_Fixture f;
  BOOST_CHECK_EQUAL (f.linear.interpolate (-1.0), 0.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (100.0), 1.0);
  // Extrapolation gives the values of the endpoints.  We do not
  // extend the line at the current slope.
}

BOOST_AUTO_TEST_CASE (test_zigzag_interpolation)
{
  Zigzag_Fixture f;
  BOOST_CHECK_EQUAL (f.linear.interpolate (0.0), 1.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (0.5), 1.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (1.0), 1.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (1.5), 1.5);
  BOOST_CHECK_EQUAL (f.linear.interpolate (2.0), 2.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (2.5), 2.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (3.0), 2.0);
}

BOOST_AUTO_TEST_CASE (test_zigzag_extrapolation)
{
  Zigzag_Fixture f;
  BOOST_CHECK_EQUAL (f.linear.interpolate (-1.0), 1.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (100.0), 2.0);
}

BOOST_AUTO_TEST_CASE (test_zigzag_normal)
{
  Zigzag_Fixture f;
  const Two_Vector up (0.0, 1.0);
  const Two_Vector slant (-1.0 / root_2, 1.0 / root_2);
  BOOST_CHECK_EQUAL (f.linear.normal (-1.0), up);
  BOOST_CHECK_EQUAL (f.linear.normal (0.0), up);
  BOOST_CHECK_EQUAL (f.linear.normal (0.5), up);
  BOOST_CHECK_EQUAL (f.linear.normal (1.0), up);
  BOOST_CHECK_EQUAL (f.linear.normal (1.5), slant);
  BOOST_CHECK_EQUAL (f.linear.normal (2.0), slant);
  BOOST_CHECK_EQUAL (f.linear.normal (2.5), up);
  BOOST_CHECK_EQUAL (f.linear.normal (3.0), up);
  BOOST_CHECK_EQUAL (f.linear.normal (100.0), up);
}

BOOST_AUTO_TEST_CASE (test_zagzig_interpolation)
{
  Zagzig_Fixture f;
  BOOST_CHECK_EQUAL (f.linear.interpolate (0.0), -1.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (0.5), -1.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (1.0), -1.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (1.5), -1.5);
  BOOST_CHECK_EQUAL (f.linear.interpolate (2.0), -2.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (2.5), -2.0);
  BOOST_CHECK_EQUAL (f.linear.interpolate (3.0), -2.0);
}

BOOST_AUTO_TEST_CASE (test_zagzig_normal)
{
  Zagzig_Fixture f;
  const Two_Vector up (0.0, 1.0);
  const Two_Vector slant (1.0 / root_2, 1.0 / root_2);
  BOOST_CHECK_EQUAL (f.linear.normal (-1.0), up);
  BOOST_CHECK_EQUAL (f.linear.normal (0.0), up);
  BOOST_CHECK_EQUAL (f.linear.normal (0.5), up);
  BOOST_CHECK_EQUAL (f.linear.normal (1.0), up);
  BOOST_CHECK_EQUAL (f.linear.normal (1.5), slant);
  BOOST_CHECK_EQUAL (f.linear.normal (2.0), slant);
  BOOST_CHECK_EQUAL (f.linear.normal (2.5), up);
  BOOST_CHECK_EQUAL (f.linear.normal (3.0), up);
  BOOST_CHECK_EQUAL (f.linear.normal (100.0), up);
}
