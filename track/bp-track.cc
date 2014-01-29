#include "../geometry/Three_Vector.h"
#include "Strip_Track.h"

#include <boost/python.hpp>

using namespace Vamos_Track;
namespace bp = boost::python;

BOOST_PYTHON_MODULE (track)
{
  bp::class_<Road> ("Road", bp::init <>());

  bp::class_<Strip_Track> ("Strip_Track", bp::init <>())
    .def ("read", &Strip_Track::read)
    .def ("get_road", &Strip_Track::get_road,
          bp::return_value_policy <bp::copy_const_reference>())
    .def ("grid_position", &Strip_Track::grid_position,
          bp::return_value_policy <bp::return_by_value>())
    .def ("build_racing_line", &Strip_Track::build_racing_line)
    .def ("show_racing_line", &Strip_Track::show_racing_line);
}
