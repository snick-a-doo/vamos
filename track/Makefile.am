lib_LTLIBRARIES = libvamos-track.la

LIBS =

libvamos_track_la_SOURCES = \
	Gl_Road_Segment.cc \
	Road_Segment.cc \
	Strip_Track.cc \
	Strip_Track_Reader.cc

pkginclude_HEADERS = \
	Gl_Road_Segment.h \
	Road_Segment.h \
	Strip_Track.h \
	Track.h

pkgincludedir = $(includedir)/@PACKAGE@/track

EXTRA_DIST = $(pkginclude_HEADERS)

AM_CXXFLAGS = -fPIC -Wall -pedantic -O2 -g -DBOOST_TEST_DYN_LINK -L/usr/lib/boost

if HAVE_PYTHON
pyexec_LTLIBRARIES = track.la
else
pyexec_LTLIBRARIES =
endif

track_la_SOURCES = bp-track.cc
track_la_CXXFLAGS = -fPIC -Wall -pedantic -I/usr/include/python$(PYTHON_VERSION) `pkg-config --cflags sdl freealut libpng`
track_la_LDFLAGS = -avoid-version -module
track_la_LIBADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	$(top_srcdir)/track/libvamos-track.la \
	-lpython$(PYTHON_VERSION) \
	-lboost_python-$(PYTHON_VERSION)

if DO_UNIT_TESTS
UNIT_TESTS = \
	test_Road_Segment \
	test_Strip_Track
else
UNIT_TESTS =
endif

noinst_PROGRAMS = $(UNIT_TESTS)
EXTRA_PROGRAMS = $(UNIT_TESTS)
test_Road_Segment_SOURCES = test_Road_Segment.cc
test_Road_Segment_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	Road_Segment.lo \
	-lboost_unit_test_framework
test_Strip_Track_SOURCES = test_Strip_Track.cc
test_Strip_Track_LDFLAGS = \
	`pkg-config --libs sdl freealut libpng`
test_Strip_Track_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	Road_Segment.lo \
	Gl_Road_Segment.lo \
	Strip_Track.lo \
	Strip_Track_Reader.lo \
	-lboost_unit_test_framework

if DO_UNIT_TESTS
check-local: $(UNIT_TESTS)
	for test in $(UNIT_TESTS); do \
		echo $$test; \
		$(top_srcdir)/track/$$test; \
	done
endif
