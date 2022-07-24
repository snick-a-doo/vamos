#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "test.h"

#include "doctest.h"

template<> bool close(double const& actual, double const& expected, double tol)
{
    if (std::abs(expected - actual) < tol)
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}
