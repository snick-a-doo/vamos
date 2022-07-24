#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Spline
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include "../geometry/spline.h"

using namespace Vamos_Geometry;

struct Empty_Fixture
{
  Spline spline;
};

struct Empty_Slope_Fixture
{
  Empty_Slope_Fixture () : spline (1.0, -1.0) {}
  Spline spline;
};

BOOST_AUTO_TEST_CASE (interpolate_empty)
{
  Empty_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (0));
  BOOST_CHECK_EQUAL (f.spline.interpolate (-1.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 0.0);
}

BOOST_AUTO_TEST_CASE (interpolate_empty_slope)
{
  Empty_Slope_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.interpolate (-1.0), -1.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 1.0);
}

BOOST_AUTO_TEST_CASE (interpolate_empty_periodic)
{
  Empty_Fixture f;
  f.spline.set_periodic (1.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (-1.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 0.0);
}

struct One_Point_Fixture : public Empty_Fixture
{
  One_Point_Fixture ()
  {
    spline.load (1.0, 2.0);
  }
};

struct One_Point_Slope_Fixture : public Empty_Slope_Fixture
{
  One_Point_Slope_Fixture ()
  {
    spline.load (1.0, 2.0);
  }
};

BOOST_AUTO_TEST_CASE (interpolate_one)
{
  One_Point_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (1));
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 2.0);
}

BOOST_AUTO_TEST_CASE (interpolate_one_slope)
{
  One_Point_Slope_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 1.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 3.0);
}

BOOST_AUTO_TEST_CASE (interpolate_one_periodic)
{
  One_Point_Slope_Fixture f;
  f.spline.set_periodic (2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 2.0);
}

struct Two_Point_Fixture : public One_Point_Fixture
{
  Two_Point_Fixture ()
  {
    spline.load (2.0, 4.0);
  }
};

struct Two_Point_Slope_Fixture : public One_Point_Slope_Fixture
{
  Two_Point_Slope_Fixture ()
  {
    spline.load (2.0, 4.0);
  }
};

BOOST_AUTO_TEST_CASE (interpolate_two)
{
  Two_Point_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (2));
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.5), 3.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 4.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (3.0), 6.0);
}

BOOST_AUTO_TEST_CASE (interpolate_two_slope)
{
  Two_Point_Slope_Fixture f;
  // y = 10 - 21x + 17x^2 - 4x^3
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 10.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.5), 3.25);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 4.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (3.0), -8.0);

  BOOST_CHECK_CLOSE (f.spline.slope (1.0), 1.0, 1e-6);
  BOOST_CHECK_CLOSE (f.spline.slope (2.0), -1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE (interpolate_two_periodic)
{
  Two_Point_Fixture f;
  f.spline.set_periodic (3.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 4.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 4.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (3.0), 2.0);
  BOOST_CHECK (std::abs (f.spline.slope (2.999) - f.spline.slope (3.0)) < 0.02);
  BOOST_CHECK (std::abs (f.spline.slope (3.001) - f.spline.slope (3.0)) < 0.02);
}

struct Three_Point_Fixture : public Two_Point_Fixture
{
  Three_Point_Fixture ()
  {
    spline.load (3.0, 5.0);
  }
};

struct Three_Point_Slope_Fixture : public Two_Point_Slope_Fixture
{
  Three_Point_Slope_Fixture ()
  {
    spline.load (3.0, 5.0);
  }
};

BOOST_AUTO_TEST_CASE (interpolate_three)
{
  Three_Point_Fixture f;
  BOOST_CHECK_CLOSE (f.spline.interpolate (1.5), 3.09375, 1e-6);
  BOOST_CHECK_CLOSE (f.spline.interpolate (2.5), 4.59375, 1e-6);
}

BOOST_AUTO_TEST_CASE (interpolate_three_slope)
{
  Three_Point_Slope_Fixture f;
  BOOST_CHECK_CLOSE (f.spline.interpolate (1.5), 2.859375, 1e-6);
  BOOST_CHECK_CLOSE (f.spline.interpolate (2.5), 4.921875, 1e-6);

  BOOST_CHECK_CLOSE (f.spline.slope (1.0), 1.0, 1e-6);
  BOOST_CHECK_CLOSE (f.spline.slope (3.0), -1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE (interpolate_three_periodic)
{
  Three_Point_Fixture f;
  f.spline.set_periodic (4.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.5), 3.375);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.5), 2.5);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.5), 5.125);
  BOOST_CHECK_EQUAL (f.spline.interpolate (3.5), 3.375);

  BOOST_CHECK (std::abs (f.spline.slope (2.999) - f.spline.slope (3.0)) < 0.02);
  BOOST_CHECK (std::abs (f.spline.slope (3.001) - f.spline.slope (3.0)) < 0.02);
}

// Uncomment and add -lgsl to test_Spline_LADD in Makefile.am to compare
// with GSL splines.

// #include <gsl/gsl_interp.h>
// #include <gsl/gsl_spline.h>

// BOOST_AUTO_TEST_CASE (draw)
// {
//   const size_t size = 4;
//   double xs [size] = { 1.0, 2.0, 3.0, 4.0 };
//   double ys [size] = { 2.0, 4.0, 5.0, 2.0 };

//   gsl_interp* interp = gsl_interp_alloc (gsl_interp_cspline_periodic, size);
//   gsl_interp_init (interp, xs, ys, size);
//   gsl_interp_accel* accel = gsl_interp_accel_alloc ();

//   Spline s (1.0, -1.0);
//   s.load (1.0, 2.0);
//   s.load (2.0, 4.0);
//   s.load (3.0, 5.0);
//   s.set_periodic (4.0);
//   size_t n = 1000;
//   double interval = 5.0/n;
//   for (size_t i = 0; i <= 1000; i++)
//     {
//       double x = i*interval;
//       std::cout << x << ' ' << s.interpolate (x) << ' ' 
//                 << gsl_interp_eval (interp, xs, ys, x, accel) << std::endl;
//     }
// }

struct Vector_Spline_Fixture
{
  Vector_Spline_Fixture ()
  {
    Three_Vector p1;
    Three_Vector p2 (1.0, 2.0, 3.0);
    spline.load (1.0, p1);
    spline.load (2.0, p2);
    spline.load (3.0, 2.0, 4.0, 8.0);
    spline.load (4.0, p1);

    x_spline.load (1.0, p1.x);
    x_spline.load (2.0, p2.x);
    x_spline.load (3.0, 2.0);
    x_spline.load (4.0, p1.x);

    y_spline.load (1.0, p1.y);
    y_spline.load (2.0, p2.y);
    y_spline.load (3.0, 4.0);
    y_spline.load (4.0, p1.y);

    z_spline.load (1.0, p1.z);
    z_spline.load (2.0, p2.z);
    z_spline.load (3.0, 8.0);
    z_spline.load (4.0, p1.z);
  }
  Vector_Spline spline;
  Spline x_spline;
  Spline y_spline;
  Spline z_spline;
};

BOOST_AUTO_TEST_CASE (vector_spline)
{
  Vector_Spline_Fixture f;
  for (double x = 1.5; x < 4.0; x += 1.0)
    {
      Three_Vector p = f.spline.interpolate (x);
      BOOST_CHECK_EQUAL (p.x, f.x_spline.interpolate (x));
      BOOST_CHECK_EQUAL (p.y, f.y_spline.interpolate (x));
      BOOST_CHECK_EQUAL (p.z, f.z_spline.interpolate (x));
    }
}
