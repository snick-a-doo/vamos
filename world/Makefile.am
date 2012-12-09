lib_LTLIBRARIES = libvamos-world.la

libvamos_world_la_SOURCES = \
	Atmosphere.cc \
	Interactive_Driver.cc \
	Robot_Driver.cc \
	World.cc \
	Controls.cc \
	Gl_World.cc \
	Sounds.cc

pkginclude_HEADERS = \
	Atmosphere.h \
	Driver.h \
	Interactive_Driver.h \
	Robot_Driver.h \
	World.h \
	Controls.h \
	Gl_World.h \
	Sounds.h

pkgincludedir = $(includedir)/@PACKAGE@/world

EXTRA_DIST = $(pkginclude_HEADERS)

AM_CXXFLAGS = -Wall -pedantic -O2 -g -DBOOST_TEST_DYN_LINK -L/usr/lib/boost `pkg-config sdl --cflags`

if DO_UNIT_TESTS
UNIT_TESTS = test_Controls
else
UNIT_TESTS =
endif

noinst_PROGRAMS = $(UNIT_TESTS)
EXTRA_PROGRAMS = $(UNIT_TESTS)
test_Controls_SOURCES = test_Controls.cc
test_Controls_LDADD = Controls.lo -lboost_unit_test_framework

if DO_UNIT_TESTS
check-local: $(UNIT_TESTS)
	for test in $(UNIT_TESTS); do \
		echo $$test; \
		$(top_srcdir)/world/$$test; \
	done
endif
