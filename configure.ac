AC_PREREQ(2.59)
AC_INIT(hwm, 0.01, BUG-REPORT-ADDRESS)
AC_CONFIG_SRCDIR([showlane.cpp])

AM_INIT_AUTOMAKE([foreign])
AM_CONFIG_HEADER([config.h])
AM_MAINTAINER_MODE

if test ! x"${CXXFLAGS+set}" = xset; then
# the user set CXXFLAGS; don't override it.
CXXFLAGS=
fi

# Checks for programs.
AC_PROG_CXX

AC_SUBST([WARN_FLAGS])
AC_SUBST([OPTIMIZE_FLAGS])
AC_SUBST([ARCH_FLAGS])
AC_SUBST([TS_CXXFLAGS])
AC_SUBST([TS_LDFLAGS])

AC_LANG([C++])

# Checks for libraries.
AC_ARG_ENABLE([visual],
  [AS_HELP_STRING([--enable-visual],
    [build graphical viewer components @<:@default=check@:>@])],
  [enable_visual=$enableval],
  [enable_visual=check])

FLTK_CXXFLAGS=
FLTK_LDFLAGS=

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
	     AC_SUBST([FLTK_CXXFLAGS])
	     AC_SUBST([FLTK_LDFLAGS])
	     AC_SUBST([CAIRO_CFLAGS])
	     AC_SUBST([CAIRO_LDFLAGS])],
      [visual_ok=no])

AM_CONDITIONAL(DO_VISUAL, test x"$visual_ok" = xyes)

AC_ARG_ENABLE([xml],
  [AS_HELP_STRING([--enable-xml],
    [build components that use xml @<:@default=check@:>@])],
  [enable_xml=$enableval],
  [enable_xml=check])

LIBXML2_CFLAGS=
LIBXML2_LIBS=

AS_IF([test x"$enable_xml" != xno],
	    [AC_CHECK_PROG(have_libxml2, xml2-config, [yes], [no])
	     AS_IF([test x"$have_libxml2" = xno -a x"$enable_xml" != xcheck],
             AC_MSG_FAILURE([--enable-xml given, but test for libxml2 failed!]))])

AS_IF([test x"$enable_xml" != xno -a x"$have_libxml2" = xyes],
	    [xml_ok=yes
	     LIBXML2_CFLAGS=`xml2-config --cflags`
	     LIBXML2_LIBS=`xml2-config --libs`
	     AC_SUBST([LIBXML2_CFLAGS])
	     AC_SUBST([LIBXML2_LIBS])],
      [xml_ok=no])

AM_CONDITIONAL(DO_XML, test x"$xml_ok" = xyes)

# Checks for header files.
AC_HEADER_STDC

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_CONST
AC_C_INLINE
AC_TYPE_SIZE_T

# Checks for library functions.
AC_FUNC_MALLOC
AC_HEADER_STDC
AC_CHECK_FUNCS([floor memset pow])

WARN_FLAGS="-Wall -Werror -Wno-unused"
OPTIMIZE_FLAGS="-O3 -funroll-loops"
ARCH_FLAGS="-march=core2"
TS_CXXFLAGS="-I/home/sewall/unc/euler-tile/grids"
TS_LDFLAGS="/home/sewall/unc/euler-tile/grids/libgrids.a"

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
