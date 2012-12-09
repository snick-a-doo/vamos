#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE XML_Parser
#include <boost/test/unit_test.hpp>

#include "XML_Parser.h"

#include <string>

using namespace Vamos_Media;

struct Empty_Path_Fixture
{
  XML_Path path;
};

BOOST_AUTO_TEST_CASE (test_empty)
{
  Empty_Path_Fixture f;
  BOOST_CHECK (f.path.empty ());
  BOOST_CHECK_EQUAL (f.path.path (), std::string (""));
}

BOOST_AUTO_TEST_CASE (test_push)
{
  Empty_Path_Fixture f;
  f.path.push ("element");
  BOOST_CHECK_EQUAL (f.path.path (), std::string ("/element"));
}

BOOST_AUTO_TEST_CASE (test_push_two)
{
  Empty_Path_Fixture f;
  f.path.push ("one");
  f.path.push ("two");
  BOOST_CHECK_EQUAL (f.path.path (), std::string ("/one/two"));
}

BOOST_AUTO_TEST_CASE (test_push_drop)
{
  Empty_Path_Fixture f;
  f.path.push ("element");
  f.path.drop ();
  BOOST_CHECK_EQUAL (f.path.path (), std::string (""));
}

BOOST_AUTO_TEST_CASE (push_two_drop)
{
  Empty_Path_Fixture f;
  f.path.push ("one");
  f.path.push ("two");
  f.path.drop ();
  BOOST_CHECK_EQUAL (f.path.path (), std::string ("/one"));
}

BOOST_AUTO_TEST_CASE (test_top)
{
  Empty_Path_Fixture f;
  f.path.push ("one");
  f.path.push ("two");
  BOOST_CHECK_EQUAL (f.path.top (), std::string ("two"));
}

BOOST_AUTO_TEST_CASE (test_match)
{
  Empty_Path_Fixture f;
  f.path.push ("one");
  f.path.push ("two");
  f.path.push ("three");
  f.path.push ("four");
  BOOST_CHECK (f.path.match ("/one/two/three/four"));
  BOOST_CHECK (!f.path.match ("/one/two/five"));
  BOOST_CHECK (!f.path.match ("/one/two/"));
  BOOST_CHECK (!f.path.match ("three/four"));
  BOOST_CHECK (f.path.match ("/one/two/*"));
  BOOST_CHECK (!f.path.match ("/one/three/*"));
  BOOST_CHECK (f.path.match ("*/three/four"));
  BOOST_CHECK (!f.path.match ("*/one/three"));
  BOOST_CHECK (f.path.match ("/one/t*ree/four"));
  BOOST_CHECK (!f.path.match ("/one/t*six"));
  BOOST_CHECK (!f.path.match ("/one/two/*two/three/four"));
  BOOST_CHECK (f.path.match ("/one/*/thr*our"));
  BOOST_CHECK (f.path.match ("/one/two/th**ree/four"));
  BOOST_CHECK (f.path.match ("/one/t*three/four"));
  BOOST_CHECK (!f.path.match ("*/three/fourteen"));
  BOOST_CHECK (!f.path.match ("*/three/fo"));
}
