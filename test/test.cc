#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "test.h"

#include "doctest.h"

#include <geometry/three-matrix.h>
#include <geometry/three-vector.h>
#include <geometry/two-vector.h>

#include <iostream>

using namespace Vamos_Geometry;

bool close(double const& actual, double const& expected, double tol)
{
    if (std::abs(expected - actual) < tol)
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}

bool close(Three_Vector const& actual, Three_Vector const& expected, double tol)
{
    if (close(actual.x, expected.x, tol)
        && close(actual.y, expected.y, tol)
        && close(actual.z, expected.z, tol))
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}

bool close(Two_Vector const& actual, Two_Vector const& expected, double tol)
{
    if (close(actual.x, expected.x, tol) && close(actual.y, expected.y, tol))
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}

bool close(Three_Matrix const& actual, Three_Matrix const& expected, double tol)
{
    for (size_t i{0}; i < 3; ++i)
        for (size_t j{0}; j < 3; ++j)
            if (!close(actual[i][j], expected[i][j], tol))
            {
                std::cout << actual << " != " << expected << " ±" << tol << std::endl;
                return false;
            }
    return true;
}
