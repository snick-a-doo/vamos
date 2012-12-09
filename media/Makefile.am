lib_LTLIBRARIES = libvamos-media.la

libvamos_media_la_SOURCES = \
	Ac3d.cc \
	Sample.cc \
	Texture_Image.cc \
	XML_Parser.cc

pkginclude_HEADERS = \
	Ac3d.h \
	Sample.h \
	Texture_Image.h \
	XML_Parser.h

pkgincludedir = $(includedir)/@PACKAGE@/media

EXTRA_DIST = $(pkginclude_HEADERS)

AM_CXXFLAGS = -Wall -pedantic -O2 -g -DBOOST_TEST_DYN_LINK -L/usr/lib/boost

if DO_UNIT_TESTS
UNIT_TESTS = test_XML_Parser
else
UNIT_TESTS =
endif

noinst_PROGRAMS = $(UNIT_TESTS)
EXTRA_PROGRAMS = $(UNIT_TESTS)
test_XML_Parser_SOURCES = test_XML_Parser.cc
test_XML_Parser_LDADD = XML_Parser.lo -lboost_unit_test_framework

if DO_UNIT_TESTS
check-local: $(UNIT_TESTS)
	for test in $(UNIT_TESTS); do \
		echo $$test; \
		$(top_srcdir)/media/$$test; \
	done
endif
