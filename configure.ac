# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.69])
AC_INIT([cap-fmu], [1.0.0], [sjp@canterburyairpatrol.org])
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
PKG_CHECK_MODULES([FSS_CLIENT_SSL], [fss-client-ssl >= 1.1.0])
PKG_CHECK_MODULES([FSS_TRANSPORT], [fss-transport >= 1.1.0])
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

# Dependency headers found via pkg-config are third-party: we cannot fix the
# warnings they emit, but our build uses -Werror. Re-emit their include paths
# with -isystem so the compiler treats them as system headers and does not
# diagnose them. The vendored MAVLink tree is handled the same way, with
# -isystem, in src/Makefile.am and tests/Makefile.am.
for _dep in SMM_ASSET FSS_CLIENT_SSL FSS_TRANSPORT JSONCPP CATCH2; do
  eval "_dep_cflags=\$${_dep}_CFLAGS"
  _dep_cflags=`echo " $_dep_cflags" | sed 's/ -I/ -isystem /g'`
  eval "${_dep}_CFLAGS=\$_dep_cflags"
done

# Optional sanitizers, off by default and intended for CI and local debugging.
# AddressSanitizer (ASan) and ThreadSanitizer (TSan) instrument memory the same
# way and cannot be combined; UndefinedBehaviorSanitizer (UBSan) layers on top
# of either. The collected flags are exported as SANITIZER_FLAGS and applied to
# both compilation and linking in the Makefiles.
AC_ARG_ENABLE([asan],
    [AS_HELP_STRING([--enable-asan], [build with AddressSanitizer])],
    [enable_asan=$enableval], [enable_asan=no])
AC_ARG_ENABLE([ubsan],
    [AS_HELP_STRING([--enable-ubsan], [build with UndefinedBehaviorSanitizer])],
    [enable_ubsan=$enableval], [enable_ubsan=no])
AC_ARG_ENABLE([tsan],
    [AS_HELP_STRING([--enable-tsan], [build with ThreadSanitizer])],
    [enable_tsan=$enableval], [enable_tsan=no])

AS_IF([test "x$enable_asan" = "xyes" && test "x$enable_tsan" = "xyes"],
    [AC_MSG_ERROR([--enable-asan and --enable-tsan cannot be used together])])

SANITIZER_FLAGS=
AS_IF([test "x$enable_asan" = "xyes"],
    [SANITIZER_FLAGS="$SANITIZER_FLAGS -fsanitize=address"])
AS_IF([test "x$enable_tsan" = "xyes"],
    [SANITIZER_FLAGS="$SANITIZER_FLAGS -fsanitize=thread"])
# Make UBSan abort (and so fail CI) instead of just logging and continuing.
AS_IF([test "x$enable_ubsan" = "xyes"],
    [SANITIZER_FLAGS="$SANITIZER_FLAGS -fsanitize=undefined -fno-sanitize-recover=all"])
# Keep frame pointers and debug info so sanitizer reports carry usable traces.
AS_IF([test -n "$SANITIZER_FLAGS"],
    [SANITIZER_FLAGS="$SANITIZER_FLAGS -fno-omit-frame-pointer -g"])
AC_SUBST([SANITIZER_FLAGS])

# Output Makefile files.
AC_CONFIG_FILES([Makefile src/Makefile])
if test "x$have_catch2" = "xyes"; then
  AC_CONFIG_FILES([tests/Makefile])
fi
AC_OUTPUT
