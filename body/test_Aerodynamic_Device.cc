#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Aerodynamic_Device
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Aerodynamic_Device.h"

using namespace Vamos_Body;
using namespace Vamos_Geometry;

struct Drag_Fixture
{
  Drag_Fixture () 
  : drag (Three_Vector::ZERO, 4.0, 0.5) 
  {
    Three_Vector wind;    
    drag.wind (wind, 0.1);
  }
  Drag drag;
};

BOOST_AUTO_TEST_CASE (drag_factor)
{
  Drag_Fixture f;
  BOOST_CHECK_EQUAL (f.drag.drag_factor (), 0.1);
}

BOOST_AUTO_TEST_CASE (staionary_drag)
{
  Drag_Fixture f;
  f.drag.find_forces ();
  BOOST_CHECK (f.drag.force ().null ());
}

BOOST_AUTO_TEST_CASE (forward_drag)
{
  Drag_Fixture f;
  Three_Vector wind (2.0, 0.0, 0.0);
  f.drag.wind (wind, 0.1);
  f.drag.find_forces ();
  BOOST_CHECK_EQUAL (f.drag.force ().x, 0.4);
}

BOOST_AUTO_TEST_CASE (low_density)
{
  Drag_Fixture f;
  Three_Vector wind (2.0, 0.0, 0.0);
  f.drag.wind (wind, 0.01);
  f.drag.find_forces ();
  BOOST_CHECK_EQUAL (f.drag.force ().x, 0.04);
}

BOOST_AUTO_TEST_CASE (vector_drag)
{
  Drag_Fixture f;
  Three_Vector wind (1.0, 2.0, -3.0);
  f.drag.wind (wind, 0.1);
  f.drag.find_forces ();
  const double length = wind. magnitude ();
  BOOST_CHECK_CLOSE (f.drag.force ().x, 0.1 * 1.0 * length, 1e-6);
  BOOST_CHECK_CLOSE (f.drag.force ().y, 0.1 * 2.0 * length, 1e-6);
  BOOST_CHECK_CLOSE (f.drag.force ().z, 0.1 * -3.0 * length, 1e-6);
}

struct Wing_Fixture
{
  Wing_Fixture () 
    : up_wing (Three_Vector::ZERO, 4.0, 3.0, 0.5, 0.4),
      down_wing (Three_Vector::ZERO, 4.0, 3.0, -0.5, 0.4)
  {
    Three_Vector wind;    
    up_wing.wind (wind, 0.1);
    down_wing.wind (wind, 0.1);
  }
  Wing up_wing;
  Wing down_wing;
};

BOOST_AUTO_TEST_CASE (wing_drag_factor)
{
  Wing_Fixture f;
  BOOST_CHECK_EQUAL (f.up_wing.drag_factor (), 0.06);
  BOOST_CHECK_EQUAL (f.down_wing.drag_factor (), 0.06);
}

BOOST_AUTO_TEST_CASE (wing_staionary_drag)
{
  Wing_Fixture f;
  f.up_wing.find_forces ();
  f.down_wing.find_forces ();
  BOOST_CHECK (f.up_wing.force ().null ());
  BOOST_CHECK (f.down_wing.force ().null ());
}

BOOST_AUTO_TEST_CASE (wing_forward_force)
{
  Wing_Fixture f;
  Three_Vector wind (2.0, 0.0, 0.0);
  f.up_wing.wind (wind, 0.1);
  f.down_wing.wind (wind, 0.1);
  f.up_wing.find_forces ();
  f.down_wing.find_forces ();
  BOOST_CHECK_CLOSE (f.up_wing.force ().x, 0.24, 1e-6);
  BOOST_CHECK_CLOSE (f.up_wing.force ().y, 0.0, 1e-6);
  BOOST_CHECK_CLOSE (f.up_wing.force ().z, 0.12, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().x, 0.24, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().y, 0.0, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().z, -0.12, 1e-6);
}

BOOST_AUTO_TEST_CASE (wing_high_density)
{
  Wing_Fixture f;
  Three_Vector wind (2.0, 0.0, 0.0);
  f.up_wing.wind (wind, 1.0);
  f.down_wing.wind (wind, 1.0);
  f.up_wing.find_forces ();
  f.down_wing.find_forces ();
  BOOST_CHECK_CLOSE (f.up_wing.force ().x, 2.4, 1e-6);
  BOOST_CHECK_CLOSE (f.up_wing.force ().y, 0.0, 1e-6);
  BOOST_CHECK_CLOSE (f.up_wing.force ().z, 1.2, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().x, 2.4, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().y, 0.0, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().z, -1.2, 1e-6);
}

BOOST_AUTO_TEST_CASE (wing_vector_drag)
{
  Wing_Fixture f;
  Three_Vector wind (1.0, 2.0, -3.0);
  f.up_wing.wind (wind, 0.1);
  f.down_wing.wind (wind, 0.1);
  f.up_wing.find_forces ();
  f.down_wing.find_forces ();
  const double length = wind. magnitude ();
  const Three_Vector drag = 0.1 * 0.6 * length * wind;
  const double lift = 0.03;
  BOOST_CHECK_CLOSE (f.up_wing.force ().x, drag.x, 1e-6);
  BOOST_CHECK_CLOSE (f.up_wing.force ().y, drag.y, 1e-6);
  BOOST_CHECK_CLOSE (f.up_wing.force ().z, drag.z + lift, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().x, drag.x, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().y, drag.y, 1e-6);
  BOOST_CHECK_CLOSE (f.down_wing.force ().z, drag.z - lift, 1e-6);
}
