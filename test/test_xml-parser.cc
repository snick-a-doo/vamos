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
        CHECK(path.match(""));
    }
    SUBCASE("push")
    {
        path.push("element");
        CHECK(path.match("/element"));
    }
    SUBCASE("push two")
    {
        path.push("one");
        path.push("two");
        CHECK(path.match("/one/two"));
    }
    SUBCASE("push drop")
    {
        path.push("element");
        path.drop();
        CHECK(path.match(""));
    }
    SUBCASE("push two drop")
    {
        path.push("one");
        path.push("two");
        path.drop();
        CHECK(path.match("/one"));
    }
    SUBCASE("test top")
    {
        path.push("one");
        path.push("two");
        CHECK(path.match("two"));
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
        CHECK(path.match("three/four"));
        CHECK(!path.match("one/three"));
        CHECK(!path.match("three/fourteen"));
        CHECK(!path.match("three/fo"));
    }
}
