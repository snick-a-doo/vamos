vamosexec_PYTHON = vamos.py
vamosexecdir = $(prefix)/bin
pyexec_PYTHON = config.py

install-exec-hook:
	mv -f $(vamosexecdir)/vamos.py $(vamosexecdir)/vamos; \
	chmod 755 $(vamosexecdir)/vamos

uninstall-hook:
	rm -f $(vamosexecdir)/vamos;

noinst_PROGRAMS = cvamos

cvamos_SOURCES = Options.cc vamos.cc

EXTRA_DIST = Options.h

cvamos_LDADD = \
	$(top_srcdir)/world/libvamos-world.la \
	$(top_srcdir)/body/libvamos-body.la \
	$(top_srcdir)/track/libvamos-track.la \
	$(top_srcdir)/geometry/libvamos-geometry.la \
	$(top_srcdir)/media/libvamos-media.la \
	-lboost_system \
	-lboost_filesystem

AM_CXXFLAGS = -fPIC -Wall -pedantic -DDATADIR=\"$(pkgdatadir)\"

