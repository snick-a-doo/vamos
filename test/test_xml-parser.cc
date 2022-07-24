#include "doctest.h"

#include "test.h"

#include <media/xml-parser.h>

#include <string>

using namespace Vamos_Media;

TEST_CASE("parse")
{
    XML_Path path;

    SUBCASE("empty")
    {
        CHECK(path.empty());
        CHECK(path.path() == "");
    }
    SUBCASE("push")
    {
        path.push("element");
        CHECK(path.path() == "/element");
    }
    SUBCASE("push two")
    {
        path.push("one");
        path.push("two");
        CHECK(path.path() == "/one/two");
    }
    SUBCASE("push drop")
    {
        path.push("element");
        path.drop();
        CHECK(path.path() == "");
    }
    SUBCASE("push two drop")
    {
        path.push("one");
        path.push("two");
        path.drop();
        CHECK(path.path() == "/one");
    }
    SUBCASE("test top")
    {
        path.push("one");
        path.push("two");
        CHECK(path.top() == "two");
    }
    SUBCASE("test match")
    {
        path.push("one");
        path.push("two");
        path.push("three");
        path.push("four");
        CHECK(path.match("/one/two/three/four"));
        CHECK(!path.match("/one/two/five"));
        CHECK(!path.match("/one/two/"));
        CHECK(!path.match("three/four"));
        CHECK(path.match("/one/two/*"));
        CHECK(!path.match("/one/three/*"));
        CHECK(path.match("*/three/four"));
        CHECK(!path.match("*/one/three"));
        CHECK(path.match("/one/t*ree/four"));
        CHECK(!path.match("/one/t*six"));
        CHECK(!path.match("/one/two/*two/three/four"));
        CHECK(path.match("/one/*/thr*our"));
        CHECK(path.match("/one/two/th**ree/four"));
        CHECK(path.match("/one/t*three/four"));
        CHECK(!path.match("*/three/fourteen"));
        CHECK(!path.match("*/three/fo"));
    }
}
