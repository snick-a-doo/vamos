#include "doctest.h"

#include "test.h"

#include <body/rigid-body.h>

using namespace Vamos_Body;
using namespace Vamos_Geometry;

TEST_CASE("cube")
{
    Rigid_Body body{Three_Vector{}, Three_Matrix{1.0}};
    body.add_particle(std::make_shared<Particle>(1.0, Three_Vector(0.0, 0.0, 0.0)));
    body.add_particle(std::make_shared<Particle>(1.0, Three_Vector(1.0, 0.0, 0.0)));
    body.add_particle(std::make_shared<Particle>(1.0, Three_Vector(0.0, 1.0, 0.0)));
    body.add_particle(std::make_shared<Particle>(1.0, Three_Vector(1.0, 1.0, 0.0)));
    body.add_particle(std::make_shared<Particle>(1.0, Three_Vector(0.0, 0.0, 1.0)));
    body.add_particle(std::make_shared<Particle>(1.0, Three_Vector(1.0, 0.0, 1.0)));
    body.add_particle(std::make_shared<Particle>(1.0, Three_Vector(0.0, 1.0, 1.0)));
    body.add_particle(std::make_shared<Particle>(1.0, Three_Vector(1.0, 1.0, 1.0)));
    body.propagate(1.0);

    CHECK(body.mass() == 8.0);
    CHECK(body.cm_position() == Three_Vector(0.5, 0.5, 0.5));
}
