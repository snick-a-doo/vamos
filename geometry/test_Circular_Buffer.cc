#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Circula_Buffer
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Circular_Buffer.h"

#include <string>
#include <sstream>

using Vamos_Geometry::Circular_Buffer;

struct Empty_Fixture
{
  Empty_Fixture () : buffer (3) {};
  Circular_Buffer <double> buffer;
};

BOOST_AUTO_TEST_CASE (empty)
{
  Empty_Fixture f;
  BOOST_CHECK_EQUAL (f.buffer.size (), 0);
}

struct Partly_Filled_Fixture : public Empty_Fixture
{
  Partly_Filled_Fixture () 
    {
      buffer.push_back (1.0);
      buffer.push_back (2.0);
    }
};

BOOST_AUTO_TEST_CASE (add)
{
  Partly_Filled_Fixture f;

  BOOST_CHECK_EQUAL (f.buffer.size (), 2);
  BOOST_CHECK_EQUAL (f.buffer [0], 1.0);
  BOOST_CHECK_EQUAL (f.buffer [1], 2.0);
}

struct Overfull_Fixture : public Partly_Filled_Fixture
{
  Overfull_Fixture () 
    {
      buffer.push_back (3.0);
      buffer.push_back (4.0);
    }
};

BOOST_AUTO_TEST_CASE (overfull)
{
  Overfull_Fixture f;

  BOOST_CHECK_EQUAL (f.buffer.size (), 3);
  BOOST_CHECK_EQUAL (f.buffer [0], 2.0);
  BOOST_CHECK_EQUAL (f.buffer [1], 3.0);
  BOOST_CHECK_EQUAL (f.buffer [2], 4.0);
}
