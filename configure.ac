# Process this file with autoconf to produce a configure script.
AC_PREREQ(2.63)
AC_INIT([hwm],[0.2],[jasonsewall@gmail.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([-Wall -Werror foreign color-tests])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([libarz/arz.hpp])
AC_CONFIG_HEADERS([config.h])

AC_LANG([C++])

AX_OPENMP([AC_SUBST(OPENMP_CXXFLAGS)],
	  [AC_MSG_FAILURE([cannot find flag for OpenMP])])

AC_ARG_ENABLE([tvmet-optimize],
  [AS_HELP_STRING([--enable-tvmet-optimize],
    [turn on tvmet's optimization flags @<:@default=no@:>@])],
  [tvmet_optimize=$enableval],
  [tvmet_optimize=no])

AC_ARG_ENABLE([boost-optimize],
  [AS_HELP_STRING([--enable-boost-optimize],
    [turn on boost's optimization flags @<:@default=no@:>@])],
  [boost_optimize=$enableval],
  [boost_optimize=no])

AC_ARG_ENABLE([build-docs],
  [AS_HELP_STRING([--enable-build-docs],
    [build Doxygen documentation @<:@default=no@:>@])],
  [try_doxygen=$enableval],
  [try_doxygen=no])

AC_ARG_ENABLE([tests],
  [AS_HELP_STRING([--enable-tests],
    [build test components @<:@default=check@:>@])],
  [enable_tests=$enableval],
  [enable_tests=check])

GIT_VERSION=`./GIT-VERSION-GEN`
AC_SUBST(GIT_VERSION)

AH_TEMPLATE([GIT_VERSION],
	    [Git revision of this build])

AC_DEFINE_UNQUOTED(GIT_VERSION, "$GIT_VERSION")

LT_INIT

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

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

AC_CHECK_PROG(have_tvmet_conf, tvmet-config, [yes], [no])
AS_IF([test x"$have_tvmet_conf" != xno],
	    [TVMET_CFLAGS="`tvmet-config --cxxflags` `tvmet-config --includes `"
	    TVMET_LIBS="`tvmet-config --libs`"
	    AC_SUBST([TVMET_CFLAGS])
	    AC_SUBST([TVMET_LIBS])],
       [AC_MSG_ERROR([Could not find tvmet-config. Please install tvmet!])])

AS_IF([test x"$tvmet_optimize" != xno],
      [TVMET_CFLAGS+=" -finline-limit=5000 --param max-inline-insns-single=5000 --param inline-unit-growth=5000 --param large-function-growth=5000 -DTVMET_OPTIMIZE -O2 -ftemplate-depth-200 -fomit-frame-pointer"],
      [TVMET_CFLAGS+=" -DTVMET_DEBUG"])

# Checks for libraries.
BOOST_REQUIRE([1.37])
BOOST_FOREACH
BOOST_UTILITY
BOOST_TYPETRAITS
BOOST_STATICASSERT

AS_IF([test x"$boost_optimize" != xno],
      [BOOST_CPPFLAGS+=" -DBOOST_DISABLE_ASSERTS"],
      [])

AS_IF([test x"$enable_tests" != xno],
	    [BOOST_TEST
	     AS_IF([test -z "$BOOST_UNIT_TEST_FRAMEWORK_LIBS" -a x"$enable_tests" != xcheck],
	           AC_MSG_FAILURE([--enable-tests given but test for Boost Unit test framework failed!]))])

AM_CONDITIONAL(DO_TESTS, test -n "$BOOST_UNIT_TEST_FRAMEWORK_LIBS")

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
		 doc/Makefile
		 doc/Doxyfile
		 libarz/Makefile])

AC_OUTPUT

echo "
  $PACKAGE_NAME version $PACKAGE_VERSION-$GIT_VERSION
  Prefix..................: $prefix
  Debug Build.............: $debug
  TVMET_CFLAGS............: $TVMET_CFLAGS
  TVMET_LIBS..............: $TVMET_LIBS
  Boost CPPFLAGS..........: $BOOST_CPPFLAGS
  OpenMP CXXFLAGS.........: $OPENMP_CXXFLAGS"
if test -n "$BOOST_UNIT_TEST_FRAMEWORK_LIBS"; then
echo "
  Doing tests: yes
    Boost TEST LDFLAGS....: $BOOST_UNIT_TEST_FRAMEWORK_LDFLAGS
    Boost TEST LIBS.......: $BOOST_UNIT_TEST_FRAMEWORK_LIBS"
else
echo "
  Doing tests: no"
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