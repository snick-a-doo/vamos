#include "../body/Gl_Car.h"
#include "../geometry/Three_Vector.h"
#include "../geometry/Three_Matrix.h"

#include <boost/python.hpp>

using namespace Vamos_Body;
using namespace Vamos_Geometry;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (body)
{
  bp::class_<Car> ("Car", bp::init <const Three_Vector&, const Three_Matrix&>());

  bp::class_<Gl_Car, bp::bases <Car> >
    ("Gl_Car", bp::init <const Three_Vector&, const Three_Matrix&>())
    .def ("read", &Gl_Car::read)
    .def ("start_engine", &Gl_Car::start_engine)
    .def ("adjust_robot_parameters", &Gl_Car::adjust_robot_parameters);
}
