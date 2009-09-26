# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.59])
AC_INIT([hwm], [0.1], [jasonsewall@gmail.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([sim/src/network.cpp])
AC_CONFIG_HEADERS([config.h])

AC_ARG_ENABLE([kinematic],
  [AS_HELP_STRING([--enable-kinematic],
    [build kinematic components (depending on gsl) @<:@default=check@:>@])],
  [enable_kinematic=$enableval],
  [enable_kinematic=check])

AS_IF([test x"$enable_kinematic" != xno],
      [AX_PATH_GSL(1.1,
      	           [have_gsl=yes],
		   [have_gsl=no])
       AS_IF([test \( x"$have_gsl" = xno  \) -a x"$enable_kinematic" != xcheck],
             AC_MSG_FAILURE([--enable-kinematic given but test for gsl failed!]))])

AM_CONDITIONAL(DO_KINEMATIC, test \( x"$have_gsl" = xyes  \) -a x"$enable_kinematic" != xno)

AC_LANG([C++])

AX_OPENMP([AC_SUBST(OPENMP_CXXFLAGS)],
	  [AC_MSG_FAILURE([cannot find flag for OpenMP])])

AC_ARG_ENABLE([visual],
  [AS_HELP_STRING([--enable-visual],
    [build graphical viewer components @<:@default=check@:>@])],
  [enable_visual=$enableval],
  [enable_visual=check])

AC_ARG_ENABLE([build-docs],
  [AS_HELP_STRING([--enable-build-docs],
    [build Doxygen documentation @<:@default=no@:>@])],
  [try_doxygen=$enableval],
  [try_doxygen=no])

AC_ARG_ENABLE([ogre],
  [AS_HELP_STRING([--enable-ogre],
    [build Ogre components @<:@default=check@:>@])],
  [enable_ogre=$enableval],
  [enable_ogre=check])

GIT_VERSION=`./GIT-VERSION-GEN`
AC_SUBST(GIT_VERSION)

AH_TEMPLATE([GIT_VERSION],
	    [Git revision of this build])

AC_DEFINE_UNQUOTED(GIT_VERSION, "$GIT_VERSION")

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_DISABLE_FAST_INSTALL
AC_PROG_LIBTOOL

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
             PKG_CHECK_MODULES([CAIRO], [cairo > 1.2	  ],
                               [have_cairo=yes],
                               [have_cairo=no])
             AS_IF([test \( x"$have_fltk" = xno -o x"$have_cairo" = xno \) -a x"$enable_visual" != xcheck],
             AC_MSG_FAILURE([--enable-visual given but test for fltk-config and cairo failed!]))])

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

ogre_ok=no
AS_IF([test x"$enable_ogre" != xno],
	    [PKG_CHECK_MODULES(OGRE,    [OGRE >= 1.2],   [have_ogre=yes],   [have_ogre=no])
             PKG_CHECK_MODULES(OIS,     [OIS >= 1.0],    [have_ois=yes],    [have_ois=no])
             PKG_CHECK_MODULES(CEGUI,     [CEGUI >= 0.6.2],    [have_cegui=yes],    [have_cegui=no])
             PKG_CHECK_MODULES(CEGUI_OGRE,     [CEGUI-OGRE >= 1.6.2],    [have_cegui_ogre=yes],    [have_cegui_ogre=no])
	     PKG_CHECK_MODULES(CAELUM,  [Caelum >= 0.4.0], [have_caelum=yes], [have_caelum=no])
	     AS_IF([test x"have_ogre" != xno -a x"have_ois" != xno -a x"have_cegui" != xno -a x"have_cegui_ogre" != xno -a x"have_caelum" != xno],
	           [ogre_ok=yes],
		   [AC_MSG_FAILURE([--enable-ogre given but test for ogre, ois, cegui, or caelum failed!])])],
      [ogre_ok=no])

AM_CONDITIONAL(DO_OGRE, test x"$ogre_ok" = xyes)

# Checks for libraries.
AX_BOOST_BASE([1.37])
AX_BOOST_FILESYSTEM
AX_BOOST_REGEX

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
 		 ogre/Makefile
 		 ogre/include/Makefile
 		 ogre/src/Makefile
                 sim/Makefile
                 sim/src/Makefile
                 sim/include/Makefile
		 test-progs/Makefile
		 etc/Makefile])

AC_OUTPUT

echo "
  $PACKAGE_NAME version $PACKAGE_VERSION-$GIT_VERSION
  Prefix..............: $prefix
  Debug Build.........: $debug
  Boost CPPFLAGS......: $BOOST_CPPFLAGS
  Boost LDFLAGS.......: $BOOST_LDFLAGS
  Boost FILESYSTEM LIB: $BOOST_FILESYSTEM_LIB
  Boost REGEX LIB.....: $BOOST_REGEX_LIB
  libxml2 CFLAGS......: $LIBXML2_CFLAGS
  libxml2 LIBS........: $LIBXML2_LIBS
  OpenMP CXXFLAGS.....: $OPENMP_CXXFLAGS"
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
if test x"$ogre_ok" = xyes; then
echo "
  Building ogre components: yes
    OGRE_CFLAGS......: $OGRE_CFLAGS
    OGRE_LIBS........: $OGRE_LIBS
    OIS_CFLAGS.......: $OIS_CFLAGS
    OIS_LIBS.........: $OIS_LIBS
    CEGUI_CFLAGS.....: $CEGUI_CFLAGS
    CEGUI_LIBS.......: $CEGUI_LIBS
    CEGUI_OGRE_CFLAGS: $CEGUI_OGRE_CFLAGS
    CEGUI_OGRE_LIBS..: $CEGUI_OGRE_LIBS
    CAELUM_CFLAGS....: $CAELUM_CFLAGS
    CAELUM_LIBS......: $CAELUM_LIBS"
else
echo "
  Building ogre components: no"
fi
if test -z $DO_KINEMATIC; then
echo "
  Building kinematic components: yes
    GSL_CFLAGS...: $GSL_CFLAGS
    GSL_LIBS.....: $GSL_LIBS"
else
echo "
  Building kinematic components: no"
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