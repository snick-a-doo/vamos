#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Three_Vector
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Constants.h"
#include "Three_Vector.h"

using namespace Vamos_Geometry;

struct Null_Vector_Fixture
{
  Three_Vector vector;
};

BOOST_AUTO_TEST_CASE (zero_length)
{
  Null_Vector_Fixture f;
  BOOST_CHECK_EQUAL (f.vector.magnitude (), 0.0);
  BOOST_CHECK (f.vector.null ());
}

struct Unit_Vector_Fixture
{
  Unit_Vector_Fixture ()
    : x (1.0, 0.0, 0.0),
      y (0.0, 1.0, 0.0),
      z (0.0, 0.0, 1.0)
  {};
  Three_Vector x;
  Three_Vector y;
  Three_Vector z;
};

BOOST_AUTO_TEST_CASE (dot_products)
{
  Unit_Vector_Fixture f;
  BOOST_CHECK_EQUAL (f.x.dot (f.y), 0.0); 
  BOOST_CHECK_EQUAL (f.x.dot (f.z), 0.0); 
  BOOST_CHECK_EQUAL (f.y.dot (f.x), 0.0); 
  BOOST_CHECK_EQUAL (f.y.dot (f.z), 0.0); 
  BOOST_CHECK_EQUAL (f.z.dot (f.x), 0.0); 
  BOOST_CHECK_EQUAL (f.z.dot (f.y), 0.0); 
}

BOOST_AUTO_TEST_CASE (cross_products)
{
  Unit_Vector_Fixture f;
  BOOST_CHECK_EQUAL (f.x.cross (f.y), f.z);
  BOOST_CHECK_EQUAL (f.x.cross (f.z), -f.y);
  BOOST_CHECK_EQUAL (f.y.cross (f.x), -f.z);
  BOOST_CHECK_EQUAL (f.y.cross (f.z), f.x);
  BOOST_CHECK_EQUAL (f.z.cross (f.x), f.y);
  BOOST_CHECK_EQUAL (f.z.cross (f.y), -f.x);
}

BOOST_AUTO_TEST_CASE (add) 
{
  Unit_Vector_Fixture f;
  BOOST_CHECK_EQUAL (f.x + f.y + f.z, Three_Vector (1.0, 1.0, 1.0));
}

BOOST_AUTO_TEST_CASE (subtract)
{
  Unit_Vector_Fixture f;
  BOOST_CHECK_EQUAL (Three_Vector (1.0, 1.0, 1.0) - f.x - f.y, f.z);
}

BOOST_AUTO_TEST_CASE (projection)
{
  Unit_Vector_Fixture f;
  BOOST_CHECK_CLOSE ((f.x.project (f.x)).magnitude (), 1.0, 1e-4);
  BOOST_CHECK_SMALL ((f.x.project (f.y)).magnitude (), 1e-4);
  BOOST_CHECK_SMALL ((f.x.project (f.z)).magnitude (), 1e-4);
}

BOOST_AUTO_TEST_CASE (rotate_z)
{
  Unit_Vector_Fixture f;
  Three_Vector r = f.x.rotate (Three_Vector (0.0, 0.0, pi/2.0));
  BOOST_CHECK_SMALL (r.x, 1e-4);
  BOOST_CHECK_SMALL (f.x.x, 1e-4);
  BOOST_CHECK_CLOSE (r.y, 1.0, 1e-4);
  BOOST_CHECK_CLOSE (f.x.y, 1.0, 1e-4);
  BOOST_CHECK_SMALL (r.z, 1e-4);
  BOOST_CHECK_SMALL (f.x.z, 1e-4);
}

BOOST_AUTO_TEST_CASE (rotate_y)
{
  Unit_Vector_Fixture f;
  Three_Vector r = f.z.rotate (Three_Vector (0.0, pi/2.0, 0.0));
  BOOST_CHECK_CLOSE (r.x, 1.0, 1e-4);
  BOOST_CHECK_SMALL (r.y, 1e-4);
  BOOST_CHECK_SMALL (r.z, 1e-4);
}

BOOST_AUTO_TEST_CASE (rotate_x)
{
  Unit_Vector_Fixture f;
  Three_Vector r = f.y.rotate (Three_Vector (pi/2.0, 0.0, 0.0));
  BOOST_CHECK_SMALL (r.x, 1e-6);
  BOOST_CHECK_SMALL (r.y, 1e-6);
  BOOST_CHECK_CLOSE (r.z, 1.0, 1e-6);
}

struct One_Two_Three_Fixture
{
  One_Two_Three_Fixture ()
    : v123 (1.0, 2.0, 3.0),
      v321 (3.0, 2.0, 1.0)
  {};
  Three_Vector v123;
  Three_Vector v321;
};

BOOST_AUTO_TEST_CASE (zero)
{
  One_Two_Three_Fixture f;
  f.v123.zero ();
  BOOST_CHECK_EQUAL (f.v123.magnitude (), 0.0);
}

BOOST_AUTO_TEST_CASE (negate)
{
  One_Two_Three_Fixture f;
  BOOST_CHECK_EQUAL (-f.v123, Three_Vector (-1.0, -2.0, -3.0));
}

BOOST_AUTO_TEST_CASE (dot)
{
  One_Two_Three_Fixture f;
  BOOST_CHECK_CLOSE (f.v123.dot (f.v321), 10.0, 1e-6);
}

struct X_Y_Plane_Fixture
{
  X_Y_Plane_Fixture ()
    : x (1.0, 0.0, 0.0),
      y (0.0, 1.0, 0.0),
      fourty_five (1.0, 1.0, 0.0)
  {}
  Three_Vector x;
  Three_Vector y;
  Three_Vector fourty_five;
};

BOOST_AUTO_TEST_CASE (xy_projection)
{
  X_Y_Plane_Fixture f;
  Three_Vector v = f.x.project (f.fourty_five);
  BOOST_CHECK_CLOSE (v.x, 0.5, 1e-6);
  BOOST_CHECK_CLOSE (v.y, 0.5, 1e-6);
  BOOST_CHECK_CLOSE (v.z, 0.0, 1e-6);
}

BOOST_AUTO_TEST_CASE (back_projection)
{
  X_Y_Plane_Fixture f;
  Three_Vector v = f.x.back_project (f.fourty_five);
  BOOST_CHECK_CLOSE (v.x, 1.0, 1e-6);
  BOOST_CHECK_CLOSE (v.y, 1.0, 1e-6);
  BOOST_CHECK_CLOSE (v.z, 0.0, 1e-6);

  // Projecting and back-projecting should leave the vector
  // unchanged.
  v = f.x.back_project (f.x.project (f.fourty_five));
  BOOST_CHECK_CLOSE (v.x, f.fourty_five.x, 1e-6);
  BOOST_CHECK_CLOSE (v.y, f.fourty_five.y, 1e-6);
  BOOST_CHECK_CLOSE (v.z, f.fourty_five.z, 1e-6);
}

struct Length_and_Angle_Fixture
{
  Length_and_Angle_Fixture () : v (2.0, pi/3.0) {}; 
  Three_Vector v;
};

BOOST_AUTO_TEST_CASE (construct)
{
  Length_and_Angle_Fixture f;
  BOOST_CHECK_CLOSE (f.v.x, 1.0, 1e-3);
  BOOST_CHECK_CLOSE (f.v.y, 1.73205, 1e-3);
  BOOST_CHECK_CLOSE (f.v.z, 0.0, 1e-3);
}

struct Vector_and_Point_Fixture
{
  Vector_and_Point_Fixture () 
  : v (3.0, 3.0, 0.0), 
    p (sqrt (2.0), 0.0, 0.0) 
  {};

  Three_Vector v;
  Three_Vector p;
};

BOOST_AUTO_TEST_CASE (perp_distance)
{
  Vector_and_Point_Fixture f;
  BOOST_CHECK_CLOSE (f.v.perp_distance (f.p), 1.0, 1e-6);
  f.v.y = -3.0;
  BOOST_CHECK_CLOSE (f.v.perp_distance (f.p), 1.0, 1e-6);
}
