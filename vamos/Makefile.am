bin_PROGRAMS = vamos

vamos_SOURCES = vamos.cc

vamos_LDADD = \
	$(top_srcdir)/world/libvamos-world.la \
	$(top_srcdir)/body/libvamos-body.la \
	$(top_srcdir)/track/libvamos-track.la \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la

LIBS = \
	`pkg-config --libs sdl freealut libpng` \
	@LIBS@

AM_CXXFLAGS = -Wall -pedantic -O2 -g -DDATADIR=\"$(pkgdatadir)\" \
	`pkg-config --cflags sdl freealut libpng`
