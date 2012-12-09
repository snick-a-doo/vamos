#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Three_Matrix
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "Constants.h"
#include "Three_Matrix.h"

using namespace Vamos_Geometry;

struct Identity_Matrix_Fixture
{
  Three_Matrix matrix;
};

struct Rotated_Matrix_Fixture
{
  Rotated_Matrix_Fixture ()
  {
    matrix.rotate (Three_Vector (1.0, 2.0, 3.0));
  }
  Three_Matrix matrix;
};

BOOST_AUTO_TEST_CASE (initial_identity)
{
  Identity_Matrix_Fixture f;
  for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
        {
          if (i == j)
            BOOST_CHECK_CLOSE (f.matrix [i][j], 1.0, 1e-6);
          else
            BOOST_CHECK_SMALL (f.matrix [i][j], 1e-6);
        }
    }
}

BOOST_AUTO_TEST_CASE (assign)
{
  Rotated_Matrix_Fixture f;
  Three_Matrix g;
  g = f.matrix;
  BOOST_CHECK_EQUAL (f.matrix, g);
}

BOOST_AUTO_TEST_CASE (copy)
{
  Rotated_Matrix_Fixture f;
  Three_Matrix g (f.matrix);
  BOOST_CHECK_EQUAL (f.matrix, g);
}

BOOST_AUTO_TEST_CASE (zero)
{
  Rotated_Matrix_Fixture f;
  f.matrix.zero ();
  for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
        {
          BOOST_CHECK_SMALL (f.matrix [i][j], 1e-6);
        }
    }
}

BOOST_AUTO_TEST_CASE (set_identity)
{
  Identity_Matrix_Fixture I;
  Rotated_Matrix_Fixture M;
  M.matrix.identity ();
  BOOST_CHECK_EQUAL (I.matrix, M.matrix);
}
