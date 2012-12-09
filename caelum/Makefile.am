noinst_PROGRAMS = caelum

caelum_SOURCES = \
	caelum.cc \
	Cylinder_Sky.cc \
	Sky.cc \
	Sphere_Sky.cc

INCLUDES = -I$(top_srcdir)/..

LIBS = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	`pkg-config --libs sdl libpng` \
	@LIBS@

EXTRA_DIST = \
	Cylinder_Sky.h \
	Sky.h \
	Sphere_Sky.h

AM_CXXFLAGS = -Wall -pedantic -O2 -g -DDATADIR=\"$(pkgdatadir)\" \
	`pkg-config --cflags sdl libpng`
