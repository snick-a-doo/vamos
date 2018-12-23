bin_PROGRAMS = vamos

vamos_SOURCES = Options.cc vamos.cc

EXTRA_DIST = Options.h

vamos_LDADD = \
	$(top_srcdir)/world/libvamos-world.la \
	$(top_srcdir)/body/libvamos-body.la \
	$(top_srcdir)/track/libvamos-track.la \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	-lboost_system \
	-lboost_filesystem

AM_CXXFLAGS = -fPIC -Wall -pedantic -DDATADIR=\"$(pkgdatadir)\"
