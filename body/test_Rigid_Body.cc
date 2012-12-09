#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Rigid_Body
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Rigid_Body.h"
#include "../geometry/Constants.h"

using namespace Vamos_Body;
using namespace Vamos_Geometry;

Three_Matrix identity;

struct Cube_Fixture
{
  Cube_Fixture ()
    : body (Three_Vector (), Three_Matrix ())
  {
    body.add_particle (new Particle (1.0, Three_Vector (0.0, 0.0, 0.0)));
    body.add_particle (new Particle (1.0, Three_Vector (1.0, 0.0, 0.0)));
    body.add_particle (new Particle (1.0, Three_Vector (0.0, 1.0, 0.0)));
    body.add_particle (new Particle (1.0, Three_Vector (1.0, 1.0, 0.0)));
    body.add_particle (new Particle (1.0, Three_Vector (0.0, 0.0, 1.0)));
    body.add_particle (new Particle (1.0, Three_Vector (1.0, 0.0, 1.0)));
    body.add_particle (new Particle (1.0, Three_Vector (0.0, 1.0, 1.0)));
    body.add_particle (new Particle (1.0, Three_Vector (1.0, 1.0, 1.0)));
    body.update_center_of_mass ();
  }
  Rigid_Body body;
};

BOOST_AUTO_TEST_CASE (cube_properties)
{
  Cube_Fixture f;
  BOOST_CHECK_EQUAL (f.body.mass (), 8.0);
  BOOST_CHECK_EQUAL (f.body.cm_position (), Three_Vector (0.5, 0.5, 0.5));
}
