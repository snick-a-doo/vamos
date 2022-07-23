dnl Process this file with autoconf to produce a configure script.
AC_INIT([vamos],[1.0.0],[snick-a-doo@comcast.net])
AC_CONFIG_SRCDIR(world/World.cc)
AC_CONFIG_AUX_DIR(config)

AM_INIT_AUTOMAKE

AC_ARG_ENABLE(unit_tests,
  [  --enable-unit-tests  build and execute unit tests],
    [case "${enableval}" in
       yes) unit_tests=true;;
       no) unit_tests=false;;
       *) AC_MSG_ERROR(bad value ${enableval} for --enable-unit-tests);;
     esac],
  [unit_tests=true])

dnl Checks for programs.
AC_PROG_CXX
AC_PROG_INSTALL
AC_PROG_MAKE_SET

LT_INIT

AC_PATH_X

AM_PATH_PYTHON([3])

dnl Boost
AX_BOOST_BASE([1.46.0],, AC_MSG_ERROR([Boost >= v1.46.0 is required.]))

dnl Checks for libraries.
AC_CHECK_LIB(SDL, SDL_Init,,
	AC_MSG_ERROR([SDL >= v1.2 is required.]))

AC_CHECK_LIB(openal, alGetError,,
	AC_MSG_ERROR([OpenAL >= v1.1 is required.]))

AC_CHECK_LIB(alut, alutGetError,,
	AC_MSG_ERROR([ALUT >= v1.1 is required.]))

AC_CHECK_HEADER(windows.h, AC_DEFINE(WIN32))

dnl Checks for typedefs, structures, and compiler characteristics.

dnl Checks for library functions.

dnl Stolen from plib's configure.in
if test "x$ac_cv_header_windows_h" != "xyes" ; then
    dnl Check for X11
    AC_PATH_XTRA
    x_suffix="$X_LIBS $X_PRE_LIBS -lX11  $X_EXTRA_LIBS"

    AC_CHECK_LIB(dl, dlclose)

    AC_CHECK_LIB(GL, glNewList,,,$x_suffix)
    if test "x$ac_cv_lib_GL_glNewList" = "xno" ; then
        dnl if no GL, check for MesaGL
        AC_CHECK_LIB(MesaGL, glNewList,,,$x_suffix)
    fi

    dnl if using mesa, check for xmesa.h
    if test "x$ac_cv_lib_MesaGL_glNewList" = "xyes" ; then
        AC_CHECK_HEADER(GL/xmesa.h)
        AM_CONDITIONAL(ENABLE_XMESA_FX, \
                       test "x$ac_cv_header_GL_xmesa_h" = "xyes")
    else
        dnl force a failed check
        AM_CONDITIONAL(ENABLE_XMESA_FX, test "no" = "yes")
    fi

    AC_CHECK_LIB(GLU, gluLookAt,,,$x_suffix)
    if test "x$ac_cv_lib_GLU_gluLookAt" = "xno" ; then
        dnl if no GLU, check for MesaGLU
        AC_CHECK_LIB(MesaGLU, gluLookAt,,,$x_suffix)
    fi

    opengl_LIBS="$LIBS $x_suffix"

    dnl check for glut...or freeglut
    AC_CHECK_LIB(freeglut, glutInit,,,$opengl_LIBS)
    if test "x$ac_cv_lib_freeglut_glutInit" = "xno" ; then
        dnl if no freeglut, check for Real GLUT
        AC_CHECK_LIB(glut, glutInit,,,$opengl_LIBS)
    fi
else
  dnl CygWin under Windoze.

  AC_CHECK_HEADERS(windows.h GL/gl.h GL/glext.h,,,
  [[#if HAVE_WINDOWS_H
  # include <windows.h>
  #endif
  #if HAVE_GL_GL_H
  # include <GL/gl.h>
  #endif
  ]])

  dnl force a failed check
  AM_CONDITIONAL(ENABLE_XMESA_FX, test "no" = "yes")

  dnl CygWin doesn't look in this directory
  LIBS="$LIBS -L/usr/local/lib"

  dnl OpenGL libraries
  LIBS="$LIBS -lwinmm -lglut32 -lglu32 -lopengl32 -luser32 -lgdi32"
fi

dnl OpenGL
AC_CHECK_HEADERS(GL/gl.h GL/glu.h,,
	AC_MSG_ERROR([OpenGL is required.]))
AC_CHECK_HEADER(GL/glut.h,,
	AC_MSG_ERROR([Glut >= v3.7 is required.]))

dnl libpng
AC_CHECK_LIB(png, png_sig_cmp,,,$x_suffix)
AC_CHECK_HEADER(png.h,,
	AC_MSG_ERROR([libpng is required.]))

AM_CONDITIONAL(DO_UNIT_TESTS, test x$unit_tests = xtrue)

AC_CONFIG_MACRO_DIR([m4])

AC_CONFIG_FILES([Makefile
				config/Makefile
				scripts/Makefile
				body/Makefile
				geometry/Makefile
				media/Makefile
				track/Makefile
				world/Makefile
				vamos/Makefile
				data/Makefile
				data/cars/Makefile
				data/cars/F1/Makefile
				data/cars/GT/Makefile
				data/controls/Makefile
				data/textures/Makefile
				data/tracks/Makefile
				data/sounds/Makefile
				data/worlds/Makefile
				doc/Makefile
				doc/robot-cars/Makefile
				caelum/Makefile])
AC_OUTPUT
