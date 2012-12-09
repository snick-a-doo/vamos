#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Two_Vector
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Two_Vector.h"

#include <string>
#include <sstream>

using Vamos_Geometry::Two_Vector;

struct Null_Vector_Fixture
{
  Two_Vector null;
};

BOOST_AUTO_TEST_CASE (components )
{
  Null_Vector_Fixture f;
  BOOST_CHECK_EQUAL (f.null.x, 0.0);
  BOOST_CHECK_EQUAL (f.null.y, 0.0);
}

struct Math_Fixture
{
  Math_Fixture ()
    : v1 (1.0, 2.0),
      v2 (3.0, 4.0)
  {};
  Two_Vector v1;
  Two_Vector v2;
};

BOOST_AUTO_TEST_CASE (add)
{
  Math_Fixture f;
  Two_Vector v3 = f.v1 + f.v2;
  BOOST_CHECK_EQUAL (v3.x, 4.0);
  BOOST_CHECK_EQUAL (v3.y, 6.0);
}

BOOST_AUTO_TEST_CASE (subtract)
{
  Math_Fixture f;
  Two_Vector v3 = f.v1 - f.v2;
  BOOST_CHECK_EQUAL (v3.x, -2.0);
  BOOST_CHECK_EQUAL (v3.y, -2.0);
}

BOOST_AUTO_TEST_CASE (multiply)
{
  Math_Fixture f;
  Two_Vector v3 = f.v1 * 3;
  BOOST_CHECK_EQUAL (v3.x, 3.0);
  BOOST_CHECK_EQUAL (v3.y, 6.0);

  v3 = 3 * f.v2;
  BOOST_CHECK_EQUAL (v3.x, 9.0);
  BOOST_CHECK_EQUAL (v3.y, 12.0);
}

BOOST_AUTO_TEST_CASE (divide)
{
  Math_Fixture f;
  Two_Vector v3 = f.v1 / 2.0;
  BOOST_CHECK_EQUAL (v3.x, 0.5);
  BOOST_CHECK_EQUAL (v3.y, 1.0);
}

struct Input_Output_Fixture
{
  Input_Output_Fixture () : v (-2.2, 3.3) {};
  Two_Vector v;
};

BOOST_AUTO_TEST_CASE (input)
{
  Input_Output_Fixture f;
  std::string s = "[ 1.2, -3.4 ]";
  std::istringstream is (s);
  ::operator >> (is, f.v);
  // Can't use is >> f.v because that would be ambiguous due to the
  // re-definition of >> in std:: for use by Boost::test.  See comment
  // in Two_Vector.h

  BOOST_CHECK_EQUAL (f.v.x, 1.2);
  BOOST_CHECK_EQUAL (f.v.y, -3.4);
}

BOOST_AUTO_TEST_CASE (output)
{
  Input_Output_Fixture f;
  std::ostringstream os;
  ::operator << (os, f.v);
  // Can't use os << f.v because that would be ambiguous due to the
  // re-definition of << in std:: for use by Boost::test.  See comment
  // in Two_Vector.h

  BOOST_CHECK_EQUAL (os.str (), "[ -2.2, 3.3 ]");
}

