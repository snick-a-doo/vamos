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

AM_CXXFLAGS = -fPIC -Wall -pedantic -O2 -g -DBOOST_TEST_DYN_LINK -L/usr/lib/boost

if HAVE_PYTHON
pyexec_LTLIBRARIES = media.la
else
pyexec_LTLIBRARIES =
endif

media_la_SOURCES = bp-media.cc
media_la_CXXFLAGS = -fPIC -Wall -pedantic -I/usr/include/python$(PYTHON_VERSION) `pkg-config --cflags sdl freealut libpng`
media_la_LDFLAGS = -avoid-version -module
media_la_LIBADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	-lpython$(PYTHON_VERSION) \
	-lboost_python-$(PYTHON_VERSION)

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
