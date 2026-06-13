# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.69])
AC_INIT([cap-fmu], [0.12.0], [sjp@canterburyairpatrol.org])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

m4_include([m4/ax_cxx_compile_stdcxx.m4])

AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIRS([m4])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AM_PROG_AR

AX_CXX_COMPILE_STDCXX([17],[],[mandatory])

LT_INIT([disable-static pic-only])

# Setup so we can install pkg-config
PKG_INSTALLDIR

# Check for required pkg-config
PKG_CHECK_MODULES([FSS_CLIENT_SSL], [fss-client-ssl])
PKG_CHECK_MODULES([FSS_TRANSPORT], [fss-transport])
PKG_CHECK_MODULES([SMM_ASSET], [smm-asset])
PKG_CHECK_MODULES([JSONCPP], [jsoncpp])

AC_CHECK_LIB([pthread], [pthread_create], [
       have_pthread=yes
       pthread_LIBS="-pthread"],
    [AC_MSG_ERROR([pthread is required for this program])])
AC_SUBST(pthread_LIBS)

# Check for optional Catch2 test framework
PKG_CHECK_MODULES([CATCH2], [catch2-with-main],
    [have_catch2=yes], [have_catch2=no])
AM_CONDITIONAL([HAVE_CATCH2], [test "x$have_catch2" = "xyes"])

# Output Makefile files.
AC_CONFIG_FILES([Makefile src/Makefile])
if test "x$have_catch2" = "xyes"; then
  AC_CONFIG_FILES([tests/Makefile])
fi
AC_OUTPUT
