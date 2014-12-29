include $(top_srcdir)/flymake.mk

# Library
lib_LTLIBRARIES = libvamos-world.la

libvamos_world_la_SOURCES = \
	Atmosphere.cc \
	Controls.cc \
	Gl_World.cc \
	Interactive_Driver.cc \
	Robot_Driver.cc \
	Sounds.cc \
	Timing_Info.cc \
	World.cc

pkginclude_HEADERS = \
	Atmosphere.h \
	Controls.h \
	Driver.h \
	Gl_World.h \
	Interactive_Driver.h \
	Robot_Driver.h \
	Sounds.h \
	Timing_Info.h \
	World.h

pkgincludedir = $(includedir)/@PACKAGE@/world

EXTRA_DIST = $(pkginclude_HEADERS)

AM_CXXFLAGS = -fPIC -Wall -pedantic

# Python Module
if HAVE_PYTHON
pyexec_LTLIBRARIES = world.la
else
pyexec_LTLIBRARIES =
endif

world_la_SOURCES = bp-world.cc
world_la_CXXFLAGS = $(AM_CXXFLAGS) -I$(PYTHON_INCLUDE_DIR)
world_la_LDFLAGS = -avoid-version -module
world_la_LIBADD = \
	$(top_srcdir)/body/libvamos-body.la \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	$(top_srcdir)/track/libvamos-track.la \
	$(top_srcdir)/world/libvamos-world.la \
	-l$(PYTHON_LIB) \
	-l$(BOOST_PYTHON_LIB)

# Unit Tests
if DO_UNIT_TESTS
UNIT_TESTS = test_Controls test_Timing_Info
else
UNIT_TESTS =
endif

noinst_PROGRAMS = $(UNIT_TESTS)
EXTRA_PROGRAMS = $(UNIT_TESTS)
test_Controls_SOURCES = test_Controls.cc
test_Controls_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Controls_LDADD = Controls.lo -lboost_unit_test_framework
test_Timing_Info_SOURCES = test_Timing_Info.cc
test_Timing_Info_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Timing_Info_LDADD = Timing_Info.lo -lboost_unit_test_framework

if DO_UNIT_TESTS
check-local: $(UNIT_TESTS)
	for test in $(UNIT_TESTS); do \
		echo $$test; \
		$(top_srcdir)/world/$$test; \
	done
endif
