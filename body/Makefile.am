# Library
lib_LTLIBRARIES = libvamos-body.la

libvamos_body_la_SOURCES = \
	Aerodynamic_Device.cc \
	Rigid_Body.cc \
	Brake.cc \
	Car.cc \
	Car_Reader.cc \
	Clutch.cc \
	Contact_Point.cc \
	Dashboard.cc \
	Differential.cc \
	Drivetrain.cc \
	Engine.cc \
	Frame.cc \
	Fuel_Tank.cc \
	Gl_Car.cc \
	Particle.cc \
	Suspension.cc \
	Tire.cc \
	Transmission.cc \
	Wheel.cc

pkginclude_HEADERS = \
	Aerodynamic_Device.h \
	Rigid_Body.h \
	Brake.h \
	Car.h \
	Clutch.h \
	Contact_Point.h \
	Dashboard.h \
	Differential.h \
	Drivetrain.h \
	Engine.h \
	Frame.h \
	Fuel_Tank.h \
	Gl_Car.h \
	Particle.h \
	Suspension.h \
	Tire.h \
	Transmission.h \
	Wheel.h

pkgincludedir = $(includedir)/@PACKAGE@/body

EXTRA_DIST = $(pkginclude_HEADERS)

AM_CXXFLAGS = -fPIC -Wall -pedantic

if DO_UNIT_TESTS
UNIT_TESTS = \
	test_Aerodynamic_Device \
	test_Car \
	test_Frame \
	test_Particle \
	test_Rigid_Body
else
UNIT_TESTS =
endif

noinst_PROGRAMS = $(UNIT_TESTS)
EXTRA_PROGRAMS = $(UNIT_TESTS)
test_Aerodynamic_Device_SOURCES = test_Aerodynamic_Device.cc
test_Aerodynamic_Device_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Aerodynamic_Device_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	Frame.lo \
	Particle.lo \
	Aerodynamic_Device.lo \
	-lboost_unit_test_framework
test_Car_SOURCES = test_Car.cc
test_Car_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Car_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	$(top_srcdir)/body/libvamos-body.la \
	-lboost_unit_test_framework
test_Frame_SOURCES = test_Frame.cc
test_Frame_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Frame_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	Frame.lo \
	-lboost_unit_test_framework
test_Particle_SOURCES = test_Particle.cc
test_Particle_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Particle_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	Frame.lo \
	Particle.lo \
	-lboost_unit_test_framework
test_Rigid_Body_SOURCES = test_Rigid_Body.cc
test_Rigid_Body_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Rigid_Body_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	$(top_srcdir)/body/libvamos-body.la \
	-lboost_unit_test_framework

if DO_UNIT_TESTS
check-local: $(UNIT_TESTS)
	for test in $(UNIT_TESTS); do \
		echo $$test; \
		$(top_srcdir)/body/$$test; \
	done
endif
