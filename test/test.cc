#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "test.h"

#include "doctest.h"

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
