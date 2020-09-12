# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.69])
AC_INIT([cap-fmu], [0.3.2], [sjp@canterburyairpatrol.org])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

m4_include([m4/ax_cxx_compile_stdcxx.m4])

AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIRS([m4])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AM_PROG_AR

AX_CXX_COMPILE_STDCXX([11],[],[mandatory])

LT_INIT([disable-static pic-only])

# Setup so we can install pkg-config
PKG_INSTALLDIR

# Check for required pkg-config
PKG_CHECK_MODULES([FSS_CLIENT], [fss-client])
PKG_CHECK_MODULES([FSS_TRANSPORT], [fss-transport])
PKG_CHECK_MODULES([SMM_ASSET], [smm-asset])

AC_CHECK_LIB([pthread], [pthread_create], [
       have_pthread=yes
       pthread_LIBS="-pthread"],
    [AC_MSG_ERROR([pthread is required for this program])])
AC_SUBST(pthread_LIBS)


# Output Makefile files.
AC_CONFIG_FILES([Makefile src/Makefile])
AC_OUTPUT
