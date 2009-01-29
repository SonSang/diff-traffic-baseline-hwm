# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.63])
AC_INIT([hwm], [0.1], [jasonsewall@gmail.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([sim/src/network.cpp])
AC_CONFIG_HEADERS([config.h])

AC_LANG([C++])

AC_ARG_ENABLE([visual],
  [AS_HELP_STRING([--enable-visual],
    [build graphical viewer components @<:@default=check@:>@])],
  [enable_visual=$enableval],
  [enable_visual=check])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_RANLIB

AC_ARG_ENABLE([build-docs],
  [AS_HELP_STRING([--enable-build-docs],
    [build Doxygen documentation @<:@default=no@:>@])],
  [try_doxygen=$enableval],
  [try_doxygen=no])

AC_PROG_TRY_DOXYGEN

if test -z "$DOXYGEN" -a x"$try_doxygen" != xno ; then
   echo "-----------------------------------------"
   echo " No Doxygen program found - continuing"
   echo " without Doxygen documentation support."
   echo "-----------------------------------------"
fi

AS_IF([test x"$try_doxygen" != xno -a -n "$DOXYGEN"],
  	    [AC_CHECK_PROG(HAVE_DOT, dot, [YES], [NO])])

AM_CONDITIONAL([HAVE_DOXYGEN], [test x"$try_doxygen" != xno -a -n "$DOXYGEN"])

AC_CHECK_PROG(have_libxml2, xml2-config, [yes], [no])
AS_IF([test x"$have_libxml2" != xno],
            [LIBXML2_CFLAGS=`xml2-config --cflags`
             LIBXML2_LIBS=`xml2-config --libs`
             AC_SUBST([LIBXML2_CFLAGS])
             AC_SUBST([LIBXML2_LIBS])],
      [AC_MSG_ERROR([Could not find xml2-config, please install libxml2!])])

AS_IF([test x"$enable_visual" != xno],
            [AC_CHECK_PROG(have_fltk, fltk-config, [yes], [no])
             PKG_CHECK_MODULES([CAIRO], [cairo > 1.6],
                               [have_cairo=yes],
                               [have_cairo=no])
             AS_IF([test \( x"$have_fltk" = xno -o x"$have_cairo" = xno \) -a x"$enable_visual" != xcheck],
             AC_MSG_FAILURE([--enable-visual given, but test for fltk-config and cairo failed!]))])

AS_IF([test x"$enable_visual" != xno -a x"$have_fltk" = xyes -a x"$have_cairo" = xyes],
            [visual_ok=yes
             FLTK_CXXFLAGS=`fltk-config --cxxflags`
             FLTK_LDFLAGS=`fltk-config --ldflags`
             FLTK_GL_CXXFLAGS=`fltk-config --cxxflags --use-gl`
             FLTK_GL_LDFLAGS=`fltk-config --ldflags --use-gl`
             AC_SUBST([FLTK_CXXFLAGS])
             AC_SUBST([FLTK_LDFLAGS])
             AC_SUBST([FLTK_GL_CXXFLAGS])
             AC_SUBST([FLTK_GL_LDFLAGS])
             AC_SUBST([CAIRO_CFLAGS])
             AC_SUBST([CAIRO_LIBS])],
      [visual_ok=no])

AM_CONDITIONAL(DO_VISUAL, test x"$visual_ok" = xyes)

# Checks for libraries.
AX_BOOST_BASE([1.35])

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T

# Checks for library functions.
AC_FUNC_MALLOC
AC_SEARCH_LIBS([clock_gettime], [rt])
AC_CHECK_FUNCS([floor memset pow sqrt strdup])

AC_CONFIG_FILES([Makefile
		 doc/Doxyfile
		 doc/Makefile
                 sim/Makefile
                 sim/src/Makefile
                 sim/include/Makefile
		 test-progs/Makefile])

AC_OUTPUT

echo "
  $PACKAGE_NAME version $PACKAGE_VERSION
  Prefix.........: $prefix
  Debug Build....: $debug
  Boost CPPFLAGS.: $BOOST_CPPFLAGS
  Boost LDFLAGS..: $BOOST_LDFLAGS
  libxml2 CFLAGS.: $LIBXML2_CFLAGS
  libxml2 LIBS...: $LIBXML2_LIBS"
if test -z $DO_VISUAL; then
echo "
  Building visual: yes
    CAIRO_CFLAGS....: $CAIRO_CFLAGS
    CAIRO_LIBS......: $CAIRO_LIBS
    FLTK_CXXFLAGS...: $FLTK_CXXFLAGS
    FLTK_LDFLAGS....: $FLTK_LDFLAGS
    FLTK_GL_CXXFLAGS: $FLTK_GL_CXXFLAGS
    FLTK_GL_LDFLAGS.: $FLTK_GL_LDFLAGS"
else
echo "
  Building visual: no"
fi
if test x"$try_doxygen" != xno -a -n "$DOXYGEN"; then
echo "
  Building documentation: yes
  Found dot.............: $HAVE_DOT"
else
echo "
  Building documentation: no"
fi
echo "
  C++ Compiler...: $CXX $CXXFLAGS $CPPFLAGS
  Linker.........: $LD $LDFLAGS $LIBS
"