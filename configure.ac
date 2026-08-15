# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.69])
AC_INIT([cap-fmu], [1.3.1], [sjp@canterburyairpatrol.org])
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
#
# The flight-safety-system floor is 1.3.0: the release that broke ABI (all four
# sonames to 4:0:0) by giving each server its own outbound worker. This source
# is written against that release's semantics, not merely built with it ---
# sendMsgAll() and attemptReconnect() are both non-blocking there, and
# connection status counts servers that have *admitted* the client rather than
# ones it has merely connected to. Building against 1.2.x would silently
# reinstate the blocking fan-out this source no longer expects, so the floor is
# a hard requirement rather than a preference.
#
# This was a version *window* (< 1.3) until 1.3.0 shipped, because that break
# was known to be pending. It is a plain floor again now for the same reason
# smm-asset has one: there is no further known-pending ABI break to guard
# against, and a speculative ceiling only fails builds that would have worked.
# When the next break is announced, pin the window again --- and re-check every
# sendMsgAll()/attemptReconnect() call site before moving the floor over it,
# since those are the semantics that changed last time.
# Keep README's build requirements in step with these bounds.
PKG_CHECK_MODULES([FSS_CLIENT_SSL], [fss-client-ssl >= 1.3.0])
PKG_CHECK_MODULES([FSS_TRANSPORT], [fss-transport >= 1.3.0])
PKG_CHECK_MODULES([SMM_ASSET], [smm-asset >= 1.1.0])
PKG_CHECK_MODULES([JSONCPP], [jsoncpp])

AC_CHECK_LIB([pthread], [pthread_create], [
       have_pthread=yes
       pthread_LIBS="-pthread"],
    [AC_MSG_ERROR([pthread is required for this program])])
AC_SUBST(pthread_LIBS)

# The unit test suite needs the Catch2 test framework. Catch2 is optional in
# the sense that cap-fmu itself builds and runs without it -- but a build that
# quietly drops the tests is a trap: with no test directory in SUBDIRS,
# automake's `make check' exits 0 having run nothing, which is
# indistinguishable from a suite that passed. Anyone honouring "run the tests
# before committing" in that configuration is running a no-op and being told it
# succeeded. So a missing Catch2 is reported loudly here (and again at the very
# end of configure), and the top-level check-local target fails instead of
# succeeding empty.
#
# --disable-tests is how to say "I know, and I do not want them": it skips the
# probe entirely and makes `make check' report that no tests were built and
# exit 0. --enable-tests is the opposite -- it turns Catch2 into a hard
# requirement, for a CI or packaging build that must never silently lose its
# test coverage.
AC_ARG_ENABLE([tests],
    [AS_HELP_STRING([--disable-tests],
        [build without the Catch2 unit tests; make check then reports that no tests were built instead of failing])
AS_HELP_STRING([--enable-tests],
        [require Catch2: fail configure if it is missing, rather than building without the tests])],
    [enable_tests=$enableval], [enable_tests=auto])

# Either Catch2 generation will do, because the distributions this is built on
# do not agree on one. Catch2 3 is preferred and is what the suite is written
# against: it advertises catch2-with-main.pc and supplies its own main.
# Catch2 2.x advertises catch2.pc alone and is header-only, so the main has to
# be compiled in (tests/catch2-main.cpp) and the handful of spellings that
# changed between the two are bridged in tests/catch2-compat.hpp.
#
# Supporting 2.x is not nostalgia: Debian bookworm has only 2.13 in main, and
# bookworm is what the aircraft image is built from. The alternative was a
# bookworm package built with no test suite compiled at all, which is precisely
# what the rest of this block exists to prevent.
#
# Order matters. Catch2 3 also installs a catch2.pc (the library without the
# main), so the 2.x probe is constrained to < 3 and only runs once
# catch2-with-main.pc has been ruled out; otherwise a 3.x install with an
# unusual split could be driven down the header-only path and fail to link.
have_catch2=no
catch2_v2=no
AS_IF([test "x$enable_tests" != "xno"],
    [PKG_CHECK_MODULES([CATCH2], [catch2-with-main >= 3],
        [have_catch2=yes],
        [PKG_CHECK_MODULES([CATCH2], [catch2 >= 2.13 catch2 < 3],
            [have_catch2=yes
             catch2_v2=yes],
            [have_catch2=no])])])

AS_IF([test "x$enable_tests" = "xyes" && test "x$have_catch2" = "xno"],
    [AC_MSG_ERROR([--enable-tests was given, but the Catch2 test framework was not found.
    Install Catch2 (Debian: catch2 -- 3.x from trixie onwards, 2.13 on
    bookworm; either builds the tests), or configure with --disable-tests.])])

AM_CONDITIONAL([HAVE_CATCH2], [test "x$have_catch2" = "xyes"])
AM_CONDITIONAL([CATCH2_V2], [test "x$catch2_v2" = "xyes"])

# Which generation was picked is worth stating: the fallback is a fallback, and
# a tree quietly built against 2.x is a tree where a test using anything newer
# will fail to compile for the next person rather than for whoever wrote it.
AS_IF([test "x$catch2_v2" = "xyes"],
    [AC_MSG_NOTICE([using Catch2 2.x (catch2.pc, header-only) -- Catch2 3 is preferred where the distribution has it.])])
AM_CONDITIONAL([TESTS_DISABLED], [test "x$enable_tests" = "xno"])

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

# Say it again after AC_OUTPUT. This is the last thing on the terminal when
# configure finishes, by which point the pkg-config line that reported the
# missing Catch2 has long since scrolled past -- and a build with no tests in
# it is not something to find out about later.
AS_IF([test "x$have_catch2" = "xyes"], [],
  [test "x$enable_tests" = "xno"],
  [AC_MSG_WARN([the unit tests are disabled (--disable-tests): make check will build and run no tests.])],
  [AC_MSG_WARN([
    ****************************************************************
    The Catch2 test framework was not found: neither
    catch2-with-main.pc (Catch2 3) nor catch2.pc (Catch2 2.13 or
    newer) is visible to pkg-config.

    cap-fmu will build, but NO UNIT TESTS will be built or run. An
    empty make check exits 0, so nothing would otherwise tell you the
    suite never ran -- make check is therefore set up to FAIL in this
    configuration.

    Install Catch2 (Debian: catch2 -- 3.x from trixie onwards, 2.13 on
    bookworm; the suite builds against either) and re-run configure,
    or pass --disable-tests to build without the test suite
    deliberately.
    ****************************************************************])])
