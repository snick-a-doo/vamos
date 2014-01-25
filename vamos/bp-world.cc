#include "../track/Strip_Track.h"
#include "../world/Atmosphere.h"
#include "../world/Gl_World.h"
#include "../world/Sounds.h"

#include <boost/python.hpp>

using namespace Vamos_Track;
using namespace Vamos_World;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (world)
{
  bp::class_<Atmosphere> ("Atmosphere", bp::init <double>());
  bp::class_<Gl_World> 
    ("Gl_World", bp::init <int, char**, Strip_Track*, Atmosphere*, Sounds*, bool>());
  bp::class_<Sounds> ("Sounds", bp::init <double>())
    .def ("read", &Sounds::read);
}
