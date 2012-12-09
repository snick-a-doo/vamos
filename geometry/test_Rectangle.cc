#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Rectangle
#include <boost/test/unit_test.hpp>
#include "Rectangle.h"

using namespace Vamos_Geometry;

struct Rectangle_Fixture
{
  Rectangle_Fixture () : box (Two_Vector (1.0, 2.0), Two_Vector (3.0, -2.0)) {};
  Rectangle box;
};

BOOST_AUTO_TEST_CASE (default_constructor)
{
  Rectangle r;
  BOOST_CHECK_EQUAL (r.left (), 0.0);
  BOOST_CHECK_EQUAL (r.top (), 0.0);
  BOOST_CHECK_EQUAL (r.right (), 0.0);
  BOOST_CHECK_EQUAL (r.bottom (), 0.0);
}

BOOST_AUTO_TEST_CASE (left)
{
  Rectangle_Fixture f;
  f.box.enclose (Rectangle (Two_Vector (0.0, 1.0), Two_Vector (1.0, 0.0)));
  BOOST_CHECK_EQUAL (f.box.left (), 0.0);
  BOOST_CHECK_EQUAL (f.box.top (), 2.0);
  BOOST_CHECK_EQUAL (f.box.right (), 3.0);
  BOOST_CHECK_EQUAL (f.box.bottom (), -2.0);

  BOOST_CHECK_EQUAL (f.box.width (), 3.0);
  BOOST_CHECK_EQUAL (f.box.height (), 4.0);
  BOOST_CHECK_EQUAL (f.box.center (), Two_Vector (1.5, 0.0));
  BOOST_CHECK_EQUAL (f.box.aspect (), 0.75);
}

BOOST_AUTO_TEST_CASE (right)
{
  Rectangle_Fixture f;
  f.box.enclose (Rectangle (Two_Vector (2.0, 1.0), Two_Vector (4.0, 0.0)));
  BOOST_CHECK_EQUAL (f.box.left (), 1.0);
  BOOST_CHECK_EQUAL (f.box.top (), 2.0);
  BOOST_CHECK_EQUAL (f.box.right (), 4.0);
  BOOST_CHECK_EQUAL (f.box.bottom (), -2.0);
}

BOOST_AUTO_TEST_CASE (top)
{
  Rectangle_Fixture f;
  f.box.enclose (Rectangle (Two_Vector (2.0, 3.0), Two_Vector (3.0, 1.0)));
  BOOST_CHECK_EQUAL (f.box.left (), 1.0);
  BOOST_CHECK_EQUAL (f.box.top (), 3.0);
  BOOST_CHECK_EQUAL (f.box.right (), 3.0);
  BOOST_CHECK_EQUAL (f.box.bottom (), -2.0);
}

BOOST_AUTO_TEST_CASE (bottom)
{
  Rectangle_Fixture f;
  f.box.enclose (Rectangle (Two_Vector (2.0, 1.0), Two_Vector (3.0, -3.0)));
  BOOST_CHECK_EQUAL (f.box.left (), 1.0);
  BOOST_CHECK_EQUAL (f.box.top (), 2.0);
  BOOST_CHECK_EQUAL (f.box.right (), 3.0);
  BOOST_CHECK_EQUAL (f.box.bottom (), -3.0);
}

BOOST_AUTO_TEST_CASE (around)
{
  Rectangle_Fixture f;
  f.box.enclose (Rectangle (Two_Vector (0.0, 3.0), Two_Vector (4.0, -2.0)));
  BOOST_CHECK_EQUAL (f.box.left (), 0.0);
  BOOST_CHECK_EQUAL (f.box.top (), 3.0);
  BOOST_CHECK_EQUAL (f.box.right (), 4.0);
  BOOST_CHECK_EQUAL (f.box.bottom (), -2.0);
}

BOOST_AUTO_TEST_CASE (scale_down)
{
  Rectangle_Fixture f;
  f.box.scale (0.5);
  BOOST_CHECK_EQUAL (f.box, Rectangle (Two_Vector (1.5, 1.0), Two_Vector (2.5, -1.0)));
  BOOST_CHECK_EQUAL (f.box.left (), 1.5);
  BOOST_CHECK_EQUAL (f.box.top (), 1.0);
  BOOST_CHECK_EQUAL (f.box.right (), 2.5);
  BOOST_CHECK_EQUAL (f.box.bottom (), -1.0);
}

BOOST_AUTO_TEST_CASE (scale_xy)
{
  Rectangle_Fixture f;
  f.box.scale (2.0, 3.0);
  BOOST_CHECK_EQUAL (f.box.left (), 0.0);
  BOOST_CHECK_EQUAL (f.box.top (), 6.0);
  BOOST_CHECK_EQUAL (f.box.right (), 4.0);
  BOOST_CHECK_EQUAL (f.box.bottom (), -6.0);
}

BOOST_AUTO_TEST_CASE (move)
{
  Rectangle_Fixture f;
  f.box.move (Two_Vector (1.0, 0.0));
  BOOST_CHECK_EQUAL (f.box.left (), 2.0);
  BOOST_CHECK_EQUAL (f.box.top (), 2.0);
  BOOST_CHECK_EQUAL (f.box.right (), 4.0);
  BOOST_CHECK_EQUAL (f.box.bottom (), -2.0);
  f.box.move (Two_Vector (2.0, -3.0));
  BOOST_CHECK_EQUAL (f.box.left (), 4.0);
  BOOST_CHECK_EQUAL (f.box.top (), -1.0);
  BOOST_CHECK_EQUAL (f.box.right (), 6.0);
  BOOST_CHECK_EQUAL (f.box.bottom (), -5.0);
}
