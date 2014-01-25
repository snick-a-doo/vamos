#include "../body/Gl_Car.h"
#include "../media/XML_Parser.h"

#include <boost/python.hpp>

using namespace Vamos_Media;
namespace bp = boost::python;

void translate (XML_Exception const& e)
{
  PyErr_SetString (PyExc_RuntimeError, e.message().c_str ());
}

BOOST_PYTHON_MODULE (media)
{
  bp::register_exception_translator <XML_Exception> (&translate);

  bp::class_<XML_Exception> ("XML_Exception", bp::init <std::string, int, std::string>())
    .def ("message", &XML_Exception::message);
}
