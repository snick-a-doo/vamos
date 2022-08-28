#include "doctest.h"

#include "test.h"

#include <body/particle.h>

#include <numbers>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace std::numbers;

Three_Vector const ones(1.0, 1.0, 1.0);
// Rotate the identity 120 degrees about [1,1,1].  This moves z to x,
// x to y and y to z.
const Three_Matrix rotated(Three_Matrix{1.0}.rotate(ones.unit() * 2.0 * pi / 3.0));

TEST_CASE("particle")
{
    Frame frame(Three_Vector(1.0, 0.0, 0.0), rotated);
    Particle particle(2.0, Three_Vector(0.0, 1.0, 1.0), rotated, &frame);

    CHECK(particle.position() == Three_Vector(0.0, 1.0, 1.0));
    CHECK(particle.contact_position() == Three_Vector(0.0, 1.0, 1.0));
    CHECK(particle.force_position() == Three_Vector(0.0, 1.0, 1.0));
    CHECK(particle.torque_position() == Three_Vector(0.0, 1.0, 1.0));
    CHECK(particle.mass_position() == Three_Vector(0.0, 1.0, 1.0));
    CHECK(particle.mass() == 2.0);
}
