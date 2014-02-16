noinst_PROGRAMS = caelum

caelum_SOURCES = \
	caelum.cc \
	Cylinder_Sky.cc \
	Sky.cc \
	Sphere_Sky.cc

EXTRA_DIST = \
	Cylinder_Sky.h \
	Sky.h \
	Sphere_Sky.h

caelum_LDADD = \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la
