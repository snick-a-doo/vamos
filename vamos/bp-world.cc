#include "../body/Car.h"
#include "../geometry/Three_Vector.h"
#include "../track/Strip_Track.h"
#include "../world/Atmosphere.h"
#include "../world/Interactive_Driver.h"
#include "../world/Robot_Driver.h"
#include "../world/Gl_World.h"
#include "../world/Sounds.h"

#include <boost/python.hpp>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
using namespace Vamos_Track;
using namespace Vamos_World;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (world)
{
  bp::class_<Atmosphere> ("Atmosphere", bp::init <double, const Three_Vector&>());

  bp::class_<Gl_World> 
    ("Gl_World", bp::init <Strip_Track&, Atmosphere&, Sounds&, bool>())
    .def ("read", &Gl_World::read)
    .def ("start", &Gl_World::start)
    .def ("add_car", &Gl_World::add_car)
    .def ("set_focused_car", &Gl_World::set_focused_car)
    .def ("get_gravity", &Gl_World::get_gravity)
    .def ("cars_can_interact", &Gl_World::cars_can_interact);

  bp::class_<Driver> ("Driver", bp::init <Car&>());

  bp::class_<Interactive_Driver, bp::bases <Driver> >
    ("Interactive_Driver", bp::init <Car&>());

  bp::class_<Robot_Driver, bp::bases <Driver> >
    ("Robot_Driver", bp::init <Car&, Strip_Track&, double>())
    .def ("interact", &Robot_Driver::interact)
    .def ("show_steering_target", &Robot_Driver::show_steering_target);

  bp::class_<Sounds> ("Sounds", bp::init <double>())
    .def ("read", &Sounds::read);
}
