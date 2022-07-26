#include "doctest.h"

#include "test.h"

#include <body/aerodynamics.h>
#include <geometry/three-vector.h>

using namespace Vamos_Body;
using namespace Vamos_Geometry;

TEST_CASE("drag")
{
    Drag drag(null_v, 4.0, 0.5);
    SUBCASE("stationary")
    {
        drag.wind(null_v, 0.1);
        drag.propagate(1.0);
        CHECK(drag.force() == null_v);
    }
    SUBCASE("forward")
    {
        drag.wind(2.0 * x_hat, 0.1);
        drag.propagate(1.0);
        CHECK(drag.drag_factor() == 0.1); // 0.5 * 0.1 * 4 * 0.5
        CHECK(close(drag.force(), 0.4*x_hat, 1e-9));
    }
    SUBCASE("low density")
    {
        drag.wind(2.0 * x_hat, 0.01);
        drag.propagate(1.0);
        CHECK(drag.drag_factor() == 0.01); // 0.5 * 0.01 * 4 * 0.5
        CHECK(close(drag.force(), 0.04*x_hat, 1e-9));
    }
    SUBCASE("vector")
    {
        Three_Vector wind(1.0, 2.0, -3.0);
        drag.wind(wind, 0.1);
        drag.propagate(1.0);
        CHECK(close(drag.force(), 0.1 * wind.magnitude() * wind, 1e-9));
    }
}

TEST_CASE("wing")
{
    Three_Vector r = y_hat;
    double Af = 4.0;   // frontal area
    double As = 3.0;   // surface area
    double Cl = 0.5;   // lift coefficient
    double eff = 0.4;  // efficiency
    Wing up(r, Af, As, Cl, eff);
    Wing down(r, Af, As, -Cl, eff);
    Three_Vector wind(-2.0, -3.0, -4.0);
    const double speed = wind.magnitude();
    SUBCASE("stationary")
    {
        up.wind(null_v, 0.2);
        down.wind(null_v, 0.4);
        up.propagate(1.0);
        down.propagate(1.0);
        CHECK(up.force() == null_v);
        CHECK(down.force() == null_v);
    }
    SUBCASE("moving")
    {
        up.wind(wind, 0.2);
        down.wind(wind, 0.4);
        up.propagate(1.0);
        down.propagate(1.0);
        // Cd = Cl*(1.0 - eff)  drag coeficcient
        CHECK(up.drag_factor() == 0.12); // kd = 0.5 * rho * Cd * Af
        CHECK(down.drag_factor() == 0.24);
        CHECK(close(up.lift_factor(), 0.06, 1e-9)); // kl = 0.5 * rho * Cl * eff * As
        CHECK(close(down.lift_factor(), -0.12, 1e-9));
        // lift = wind.x^2 * kl * z-hat = 0.24 z-hat, -0.48 z-hat
        // drag = kd * wind * speed
        // force = lift + drag
        CHECK(close(up.force(), 0.12 * wind * speed + 0.24*z_hat, 1e-6));
        CHECK(close(down.force(), 0.24 * wind * speed - 0.48*z_hat, 1e-4));
    }
}
