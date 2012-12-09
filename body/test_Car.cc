#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Rigid_Body
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Car.h"

using namespace Vamos_Body;
using namespace Vamos_Geometry;

bool vector_equal (const Three_Vector& v1, const Three_Vector& v2)
{
  if ((v1 - v2).magnitude () > 1e-6)
    {
      std::cerr << v1 << " != " << v2 << std::endl;
      // Can't do the obvious std::cerr << v1 << " != " << v2 << std::endl;
      // because << needs to be overloaded in namespace std for boost.  See
      // comment at end of Three_Vector.h.
      return false;
    }
  return true;
}

struct Crash_Box_Fixture
{
  Crash_Box_Fixture ()
    : v (1.0, 1.0, 1.0)
  {
    crash_box.front = 1.0;
    crash_box.back = 0.0;
    crash_box.left = 1.0;
    crash_box.right = 0.0;
    crash_box.top = 1.0;
    crash_box.bottom = 0.0;
  }

  Car::Crash_Box crash_box;

  Three_Vector v;
  Three_Vector null;
  Three_Vector x;
};

BOOST_AUTO_TEST_CASE (outside_of_box)
{
  Crash_Box_Fixture f;

  f.x = Three_Vector (2.0, 2.0, 2.0);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
  f.x = Three_Vector (-2.0, -2.0, -2.0);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
  f.x = Three_Vector (-2.0, 2.0, 2.0);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
  f.x = Three_Vector (2.0, -2.0, -2.0);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);

  f.x = Three_Vector (2.0, 0.5, 0.5);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
  f.x = Three_Vector (-2.0, 0.5, 0.5);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
  f.x = Three_Vector (0.5, 2.0, 0.5);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
  f.x = Three_Vector (0.5, -2.0, 0.5);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
  f.x = Three_Vector (0.5, 0.5, 2.0);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
  f.x = Three_Vector (0.5, 0.5, -2.0);
  BOOST_CHECK_EQUAL (f.crash_box.penetration (f.x, f.v, false), f.null);
}

BOOST_AUTO_TEST_CASE (inside_box)
{
  Crash_Box_Fixture f;

  f.x = Three_Vector (0.1, 0.5, 0.5);
  BOOST_CHECK (vector_equal (f.crash_box.penetration (f.x, f.v, false),
                             Three_Vector (-0.1, 0.0, 0.0)));
  f.x = Three_Vector (0.5, 0.1, 0.5);
  BOOST_CHECK (vector_equal (f.crash_box.penetration (f.x, f.v, false),
                             Three_Vector (0.0, -0.1, 0.0)));
  f.x = Three_Vector (0.5, 0.5, 0.1);
  BOOST_CHECK (vector_equal (f.crash_box.penetration (f.x, f.v, false),
                             Three_Vector (0.0, 0.0, -0.1)));

  f.x = Three_Vector (0.9, 0.5, 0.5);
  f.v = Three_Vector (-1.0, -1.0, -1.0);
  BOOST_CHECK (vector_equal (f.crash_box.penetration (f.x, f.v, false),
                             Three_Vector (0.1, 0.0, 0.0)));
  f.x = Three_Vector (0.5, 0.9, 0.5);
  BOOST_CHECK (vector_equal (f.crash_box.penetration (f.x, f.v, false),
                             Three_Vector (0.0, 0.1, 0.0)));
  f.x = Three_Vector (0.5, 0.5, 0.9);
  BOOST_CHECK (vector_equal (f.crash_box.penetration (f.x, f.v, false),
                             Three_Vector (0.0, 0.0, 0.1)));
}
