#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Calculations
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "../geometry/Calculations.h"
#include "../geometry/Constants.h"
#include "../geometry/Three_Vector.h"

using namespace Vamos_Geometry;

struct Two_Particle_Fixture
{
  // r1 and v1 are initialized to null vectors.
  Two_Particle_Fixture ()
    : r2 (10.0, 10.0, 0.0),
      v2 (-1.0, 0.0, 0.0)
  {};

  Three_Vector r1;
  Three_Vector v1;
  Three_Vector r2;
  Three_Vector v2;
};

BOOST_AUTO_TEST_CASE (passing)
{
  Two_Particle_Fixture f;
  BOOST_CHECK_CLOSE (closest_approach (f.r1, f.v1, f.r2, f.v2), 10.0, 1e-6);
  BOOST_CHECK_CLOSE (closing_speed (f.r1, f.v1, f.r2, f.v2), inv_root_2, 1e-6);
}

BOOST_AUTO_TEST_CASE (head_on)
{
  Two_Particle_Fixture f;
  f.v2 = Three_Vector (-1.0, -1.0, 0.0);
  BOOST_CHECK_SMALL (closest_approach (f.r1, f.v1, f.r2, f.v2), 1e-6);
  BOOST_CHECK_CLOSE (closing_speed (f.r1, f.v1, f.r2, f.v2), root_2, 1e-6);
}

BOOST_AUTO_TEST_CASE (broadside)
{
  Two_Particle_Fixture f;
  f.v1 = Three_Vector (0.0, 1.0, 0.0);
  BOOST_CHECK_SMALL (closest_approach (f.r1, f.v1, f.r2, f.v2), 1e-6);
  BOOST_CHECK_CLOSE (closing_speed (f.r1, f.v1, f.r2, f.v2), root_2, 1e-6);
}

BOOST_AUTO_TEST_CASE (crossing)
{
  Two_Particle_Fixture f;
  f.v2 = Three_Vector (0.0, -1.0, 0.0);
  BOOST_CHECK_CLOSE (closest_approach (f.r1, f.v1, f.r2, f.v2), 10.0, 1e-6);
  BOOST_CHECK_CLOSE (closing_speed (f.r1, f.v1, f.r2, f.v2), inv_root_2, 1e-6);
}

BOOST_AUTO_TEST_CASE (leaving)
{
  Two_Particle_Fixture f;
  f.v2 = Three_Vector (1.0, 0.0, 0.0);
  BOOST_CHECK_CLOSE (closest_approach (f.r1, f.v1, f.r2, f.v2), 10.0, 1e-6);
  BOOST_CHECK_CLOSE (closing_speed (f.r1, f.v1, f.r2, f.v2), -inv_root_2, 1e-6);
}

