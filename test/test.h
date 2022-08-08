#ifndef VAMOS_TEST_TEST_HPP_INCLUDED
#define VAMOS_TEST_TEST_HPP_INCLUDED

#include <geometry/three-vector.h>
#include <geometry/two-vector.h>

/// @return True if actual is within tol of expected.
bool close(double const& actual, double const& expected, double tol);

/// @return True if each member of actual is within tol of the corresponding member of
/// expected.
bool close(Vamos_Geometry::Three_Vector const& actual,
           Vamos_Geometry::Three_Vector const& expected,
           double tol);

/// @return True if each member of actual is within tol of the corresponding member of
/// expected.
bool close(Vamos_Geometry::Two_Vector const& actual,
           Vamos_Geometry::Two_Vector const& expected,
           double tol);

#endif // VAMOS_TEST_TEST_HPP_INCLUDED
