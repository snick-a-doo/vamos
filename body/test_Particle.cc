#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Particle
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Particle.h"
#include "../geometry/Constants.h"

using namespace Vamos_Body;
using namespace Vamos_Geometry;

const Three_Vector null_vector (0.0, 0.0, 0.0);
const Three_Vector ones (1.0, 1.0, 1.0);
const Three_Matrix identity;

// Rotate the identity 120 degrees about [1,1,1].  This moves z to x,
// x to y and y to z.
const Three_Matrix rotated (Three_Matrix ().
                            rotate (ones.unit () * 2.0 * pi / 3.0));

struct Particle_Fixture
{
  Particle_Fixture () 
    : frame (Three_Vector (1.0, 0.0, 0.0), rotated),
      particle (2.0, Three_Vector (0.0, 1.0, 1.0), rotated, &frame)
  {}
  Frame frame;
  Particle particle;
};

BOOST_AUTO_TEST_CASE (positions)
{
  Particle_Fixture f;
  BOOST_CHECK_EQUAL (f.particle.position (), Three_Vector (0.0, 1.0, 1.0));
  BOOST_CHECK_EQUAL (f.particle.contact_position (), Three_Vector (0.0, 1.0, 1.0));
  BOOST_CHECK_EQUAL (f.particle.force_position (), Three_Vector (0.0, 1.0, 1.0));
  BOOST_CHECK_EQUAL (f.particle.torque_position (), Three_Vector (0.0, 1.0, 1.0));
  BOOST_CHECK_EQUAL (f.particle.mass_position (), Three_Vector (0.0, 1.0, 1.0));

  //  BOOST_CHECK_EQUAL (f.particle.transform_to_parent (ones));
}

BOOST_AUTO_TEST_CASE (mass)
{
  Particle_Fixture f;
  BOOST_CHECK_EQUAL (f.particle.mass (), 2.0);
}
