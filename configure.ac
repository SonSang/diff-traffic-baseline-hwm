# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.63])
AC_INIT([hwm], [0.1], [jasonsewall@gmail.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([sim/src/network.cpp])
AC_CONFIG_HEADERS([config.h])

AC_LANG([C++])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_RANLIB

AC_CHECK_PROG(have_libxml2, xml2-config, [yes], [no])
AS_IF([test x"$have_libxml2" != xno],
            [LIBXML2_CFLAGS=`xml2-config --cflags`
             LIBXML2_LIBS=`xml2-config --libs`
             AC_SUBST([LIBXML2_CFLAGS])
             AC_SUBST([LIBXML2_LIBS])],
      [AC_MSG_ERROR([Could not find xml2-config, please install libxml2!])])

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
  libxml2 LIBS...: $LIBXML2_LIBS
  C++ Compiler...: $CXX $CXXFLAGS $CPPFLAGS
  Linker.........: $LD $LDFLAGS $LIBS
"