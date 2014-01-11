bin_PROGRAMS = vamos

vamos_SOURCES = Options.cc vamos.cc

vamos_LDADD = \
	$(top_srcdir)/world/libvamos-world.la \
	$(top_srcdir)/body/libvamos-body.la \
	$(top_srcdir)/track/libvamos-track.la \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la

LIBS = \
	`pkg-config --libs sdl freealut libpng` -lboost_system -lboost_filesystem \
	@LIBS@

AM_CXXFLAGS = -Wall -pedantic -O2 -g -DDATADIR=\"$(pkgdatadir)\" \
	`pkg-config --cflags sdl freealut libpng`
