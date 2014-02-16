# Library
lib_LTLIBRARIES = libvamos-media.la

libvamos_media_la_SOURCES = \
	Ac3d.cc \
	Sample.cc \
	Texture_Image.cc \
	Two_D.cc \
	XML_Parser.cc

pkginclude_HEADERS = \
	Ac3d.h \
	Sample.h \
	Texture_Image.h \
	Two_D.h \
	XML_Parser.h

pkgincludedir = $(includedir)/@PACKAGE@/media

EXTRA_DIST = $(pkginclude_HEADERS)

AM_CXXFLAGS = -fPIC -Wall -pedantic

# Python Module
if HAVE_PYTHON
pyexec_LTLIBRARIES = media.la
else
pyexec_LTLIBRARIES =
endif

media_la_SOURCES = bp-media.cc
media_la_CXXFLAGS = $(AM_CXXFLAGS) -I$(PYTHON_INCLUDE_DIR)
media_la_LDFLAGS = -avoid-version -module
media_la_LIBADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	-l$(PYTHON_LIB) \
	-l$(BOOST_PYTHON_LIB)

# Unit Tests
if DO_UNIT_TESTS
UNIT_TESTS = test_XML_Parser
else
UNIT_TESTS =
endif

noinst_PROGRAMS = $(UNIT_TESTS)
EXTRA_PROGRAMS = $(UNIT_TESTS)
test_XML_Parser_SOURCES = test_XML_Parser.cc
test_XML_Parser_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_XML_Parser_LDADD = XML_Parser.lo -lboost_unit_test_framework

if DO_UNIT_TESTS
check-local: $(UNIT_TESTS)
	for test in $(UNIT_TESTS); do \
		echo $$test; \
		$(top_srcdir)/media/$$test; \
	done
endif
