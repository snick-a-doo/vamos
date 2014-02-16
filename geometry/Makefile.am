# Library
lib_LTLIBRARIES = \
	libvamos-geometry.la

libvamos_geometry_la_SOURCES = \
	Calculations.cc \
	Inertia_Tensor.cc \
	Interpolator.cc \
	Linear_Interpolator.cc \
	Material.cc \
	PID.cc \
	Rectangle.cc \
	Spline.cc \
	Three_Matrix.cc \
	Three_Vector.cc \
	Two_Vector.cc \
	Parameter.cc

pkginclude_HEADERS = \
	Calculations.h \
	Circular_Buffer.h \
	Constants.h \
	Contact_Info.h \
	Conversions.h \
	Inertia_Tensor.h \
	Interpolator.h \
	Linear_Interpolator.h \
	Material.h \
	Numeric.h \
	PID.h \
	Rectangle.h \
	Spline.h \
	Three_Matrix.h \
	Three_Vector.h \
	Two_Vector.h \
	Parameter.h

pkgincludedir = $(includedir)/@PACKAGE@/geometry

EXTRA_DIST = $(pkginclude_HEADERS)

AM_CXXFLAGS = -fPIC -Wall -pedantic

# Python Module
if HAVE_PYTHON
pyexec_LTLIBRARIES = geometry.la
else
pyexec_LTLIBRARIES =
endif

geometry_la_SOURCES = bp-geometry.cc
geometry_la_CXXFLAGS = $(AM_CXXFLAGS) -I$(PYTHON_INCLUDE_DIR)
geometry_la_LDFLAGS = -avoid-version -module
geometry_la_LIBADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	-l$(PYTHON_LIB) \
	-l$(BOOST_PYTHON_LIB)

# Unit Tests
if DO_UNIT_TESTS
UNIT_TESTS = \
	test_Calculations \
	test_Circular_Buffer \
	test_Linear_Interpolator \
	test_Numeric \
	test_PID \
	test_Rectangle \
	test_Spline \
	test_Three_Matrix \
	test_Three_Vector \
	test_Two_Vector
else
UNIT_TESTS =
endif

noinst_PROGRAMS = $(UNIT_TESTS)
EXTRA_PROGRAMS = $(UNIT_TESTS)
test_Calculations_SOURCES = test_Calculations.cc
test_Calculations_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Calculations_LDADD = Calculations.lo Three_Vector.lo Three_Matrix.lo -lboost_unit_test_framework
test_Circular_Buffer_SOURCES = test_Circular_Buffer.cc
test_Circular_Buffer_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Circular_Buffer_LDADD = -lboost_unit_test_framework
test_Linear_Interpolator_SOURCES = test_Linear_Interpolator.cc
test_Linear_Interpolator_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Linear_Interpolator_LDADD = Linear_Interpolator.lo Interpolator.lo Two_Vector.lo -lboost_unit_test_framework
test_Numeric_SOURCES = test_Numeric.cc
test_Numeric_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Numeric_LDADD = -lboost_unit_test_framework
test_PID_SOURCES = test_PID.cc
test_PID_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_PID_LDADD = PID.o -lboost_unit_test_framework
test_Rectangle_SOURCES = test_Rectangle.cc
test_Rectangle_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Rectangle_LDADD = Rectangle.lo Two_Vector.lo -lboost_unit_test_framework
test_Spline_SOURCES = test_Spline.cc
test_Spline_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Spline_LDADD = Spline.lo Interpolator.lo Two_Vector.lo -lboost_unit_test_framework
test_Three_Matrix_SOURCES = test_Three_Matrix.cc
test_Three_Matrix_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Three_Matrix_LDADD = Three_Vector.lo Three_Matrix.lo -lboost_unit_test_framework
test_Three_Vector_SOURCES = test_Three_Vector.cc
test_Three_Vector_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Three_Vector_LDADD = Three_Vector.lo Three_Matrix.lo -lboost_unit_test_framework
test_Two_Vector_SOURCES = test_Two_Vector.cc
test_Two_Vector_CXXFLAGS = -DBOOST_TEST_DYN_LINK
test_Two_Vector_LDADD = Two_Vector.lo -lboost_unit_test_framework

if DO_UNIT_TESTS
check-local: $(UNIT_TESTS)
	for test in $(UNIT_TESTS); do \
		echo $$test; \
		$(top_srcdir)/geometry/$$test; \
	done
endif
