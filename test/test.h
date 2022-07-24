#ifndef VAMOS_TEST_TEST_HPP_INCLUDED
#define VAMOS_TEST_TEST_HPP_INCLUDED

#include <iostream>

template<typename T> bool close(T const& actual, T const& expected, double tol)
{
    if (close(actual.x, expected.x, tol)
        && close(actual.y, expected.y, tol)
        && close(actual.z, expected.z, tol))
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}

template<> bool close(double const& actual, double const& expected, double tol);

#endif // VAMOS_TEST_TEST_HPP_INCLUDED
