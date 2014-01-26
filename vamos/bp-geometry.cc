#include "../geometry/Three_Matrix.h"
#include "../geometry/Three_Vector.h"

#include <boost/python.hpp>

using namespace Vamos_Geometry;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (geometry)
{
  bp::class_<Three_Matrix> ("Three_Matrix", bp::init <>());
  bp::class_<Three_Vector> ("Three_Vector", bp::init <double, double, double>())
    .def (bp::init <>());
}
