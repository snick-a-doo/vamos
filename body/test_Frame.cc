#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Frame
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Frame.h"
#include "../geometry/Constants.h"

using namespace Vamos_Body;
using namespace Vamos_Geometry;

struct Default_Frame_Fixture
{
  Frame frame;
};

struct Position_Frame_Fixture
{
  Position_Frame_Fixture () : frame (Three_Vector (1.0, 2.0, -3.0)) {};
  Frame frame;
};

const Three_Vector null_vector (0.0, 0.0, 0.0);
const Three_Vector ones (1.0, 1.0, 1.0);
const Three_Matrix identity;

// Rotate the identity 120 degrees about [1,1,1].  This moves z to x,
// x to y and y to z.
const Three_Matrix rotated (Three_Matrix ().
                            rotate (ones.unit () * 2.0 * pi / 3.0));

struct Orientation_Frame_Fixture
{
  Orientation_Frame_Fixture () : frame (null_vector, rotated) {};
  Frame frame;
};

struct Position_and_Orientation_Frame_Fixture
{
  Position_and_Orientation_Frame_Fixture () 
    : frame (Three_Vector (1.0, 2.0, -3.0), rotated)
  {}
  Frame frame;
};

BOOST_AUTO_TEST_CASE (defaults)
{
  Default_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.position (), null_vector);
  BOOST_CHECK_EQUAL (f.frame.velocity (), null_vector);
  BOOST_CHECK_EQUAL (f.frame.orientation (), identity);
  BOOST_CHECK_EQUAL (f.frame.angular_velocity (), null_vector);
}

BOOST_AUTO_TEST_CASE (initial_position)
{
  Position_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.position (), Three_Vector (1.0, 2.0, -3.0));
  BOOST_CHECK_EQUAL (f.frame.velocity (), null_vector);
  BOOST_CHECK_EQUAL (f.frame.orientation (), identity);
  BOOST_CHECK_EQUAL (f.frame.angular_velocity (), null_vector);
}

BOOST_AUTO_TEST_CASE (initial_position_and_orientation)
{
  Position_and_Orientation_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.position (), Three_Vector (1.0, 2.0, -3.0));
  BOOST_CHECK_EQUAL (f.frame.velocity (), null_vector);
  BOOST_CHECK_EQUAL (f.frame.orientation (), rotated);
  BOOST_CHECK_EQUAL (f.frame.angular_velocity (), null_vector);
}

BOOST_AUTO_TEST_CASE (translate)
{
  Position_Frame_Fixture f;
  f.frame.translate (Three_Vector (-1.0, 0.0, 1.0));
  BOOST_CHECK_EQUAL (f.frame.position (), Three_Vector (0.0, 2.0, -2.0));
  BOOST_CHECK_EQUAL (f.frame.velocity (), null_vector);
  BOOST_CHECK_EQUAL (f.frame.orientation (), identity);
  BOOST_CHECK_EQUAL (f.frame.angular_velocity (), null_vector);
}

BOOST_AUTO_TEST_CASE (set_position)
{
  Position_Frame_Fixture f;
  f.frame.set_position (Three_Vector (-1.0, 0.0, 1.0));
  BOOST_CHECK_EQUAL (f.frame.position (), Three_Vector (-1.0, 0.0, 1.0));
  BOOST_CHECK_EQUAL (f.frame.velocity (), null_vector);
  BOOST_CHECK_EQUAL (f.frame.orientation (), identity);
  BOOST_CHECK_EQUAL (f.frame.angular_velocity (), null_vector);
}

BOOST_AUTO_TEST_CASE (rotate)
{
  Orientation_Frame_Fixture f;
  f.frame.rotate (-ones.unit () * 2.0 * pi / 3.0);
  BOOST_CHECK_EQUAL (f.frame.position (), null_vector);
  BOOST_CHECK_EQUAL (f.frame.velocity (), null_vector);
  BOOST_CHECK_EQUAL (f.frame.angular_velocity (), null_vector);

  Three_Matrix o = f.frame.orientation ();
  for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
        {
          if (i == j)
            BOOST_CHECK_CLOSE (o [i][j], 1.0, 1e-4);
          else
            BOOST_CHECK_SMALL (o [i][j], 1e-4);
        }
    }
}

BOOST_AUTO_TEST_CASE (transform_into_default)
{
  Default_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.transform_from_world (ones), ones);
}

BOOST_AUTO_TEST_CASE (transform_into_position)
{
  Position_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.transform_from_world (ones), 
                     Three_Vector (0.0, -1.0, 4.0));
}

BOOST_AUTO_TEST_CASE (transform_into_position_and_orientation)
{
  Position_and_Orientation_Frame_Fixture f;
  Three_Vector in = f.frame.transform_from_world (ones);
  BOOST_CHECK_CLOSE (in.x, -1.0, 1e-4);
  BOOST_CHECK_CLOSE (in.y, 4.0, 1e-4);
  BOOST_CHECK_SMALL (in.z, 1e-4);
}

BOOST_AUTO_TEST_CASE (transform_to_world_default)
{
  Default_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.transform_to_world (ones), ones);
}

BOOST_AUTO_TEST_CASE (transform_to_world_position)
{
  Position_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.transform_to_world (ones), 
                     Three_Vector (2.0, 3.0, -2.0));
}

BOOST_AUTO_TEST_CASE (transform_to_world_orientation)
{
  Orientation_Frame_Fixture f;
  Three_Vector out = f.frame.transform_to_world (Three_Vector (1.0, 0.0, 0.0));
  BOOST_CHECK_SMALL (out.x, 1e-4);
  BOOST_CHECK_CLOSE (out.y, 1.0, 1e-4);
  BOOST_CHECK_SMALL (out.z, 1e-4);
}

BOOST_AUTO_TEST_CASE (transform_to_world_position_and_orientation)
{
  Position_and_Orientation_Frame_Fixture f;
  Three_Vector out = f.frame.transform_to_world (Three_Vector (1.0, 0.0, 0.0));
  BOOST_CHECK_CLOSE (out.x, 1.0, 1e-4);
  BOOST_CHECK_CLOSE (out.y, 3.0, 1e-4);
  BOOST_CHECK_CLOSE (out.z, -3.0, 1e-4);
}

BOOST_AUTO_TEST_CASE (rotate_out_position_and_orientation)
{
  Position_and_Orientation_Frame_Fixture f;
  Three_Vector out = f.frame.rotate_to_parent (Three_Vector (1.0, 0.0, 0.0));
  BOOST_CHECK_SMALL (out.x, 1e-4);
  BOOST_CHECK_CLOSE (out.y, 1.0, 1e-4); // x' is y
  BOOST_CHECK_SMALL (out.z, 1e-4);
}

BOOST_AUTO_TEST_CASE (copy_rotate_out_position_and_orientation)
{
  Position_and_Orientation_Frame_Fixture f;
  Frame g (f.frame);
  Three_Vector out = g.rotate_to_parent (Three_Vector (1.0, 0.0, 0.0));
  BOOST_CHECK_SMALL (out.x, 1e-4);
  BOOST_CHECK_CLOSE (out.y, 1.0, 1e-4); // x' is y
  BOOST_CHECK_SMALL (out.z, 1e-4);
}

BOOST_AUTO_TEST_CASE (rotate_in_position_and_orientation)
{
  Position_and_Orientation_Frame_Fixture f;
  Three_Vector out = f.frame.rotate_from_parent (Three_Vector (1.0, 0.0, 0.0));
  BOOST_CHECK_SMALL (out.x, 1e-4);
  BOOST_CHECK_SMALL (out.y, 1e-4);
  BOOST_CHECK_CLOSE (out.z, 1.0, 1e-4); // x is z'
}

struct Nested_Frame_Fixture
{
  Nested_Frame_Fixture ()
    : parent (Three_Vector (1.0, 2.0, 3.0)),
      frame (Three_Vector (2.0, 3.0, 4.0), &parent)
  {}  
  Frame parent;
  Frame frame;
};

BOOST_AUTO_TEST_CASE (transform_to_parent)
{
  Nested_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.transform_to_parent (Three_Vector (3.0, 4.0, 5.0)), 
                     Three_Vector (5.0, 7.0, 9.0));
}

BOOST_AUTO_TEST_CASE (transform_to_world)
{
  Nested_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.transform_to_world (Three_Vector (3.0, 4.0, 5.0)), 
                     Three_Vector (6.0, 9.0, 12.0));
}

BOOST_AUTO_TEST_CASE (transform_from_parent)
{
  Nested_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.transform_from_parent (Three_Vector (3.0, 4.0, 5.0)), 
                     Three_Vector (1.0, 1.0, 1.0));
}

BOOST_AUTO_TEST_CASE (transform_from_world)
{
  Nested_Frame_Fixture f;
  BOOST_CHECK_EQUAL (f.frame.transform_from_world (Three_Vector (3.0, 4.0, 5.0)), 
                     Three_Vector (0.0, -1.0, -2.0));
}
