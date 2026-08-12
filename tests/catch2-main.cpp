/*
 * Catch2 2.x is header-only: it ships no library to link, so each test
 * programme has to compile Catch2's main itself, in exactly one translation
 * unit. Catch2 3.x supplies it as libCatch2Main (catch2-with-main.pc), so this
 * file is only compiled into the test programmes when configure selected the
 * 2.x path -- see tests/catch2-compat.hpp and tests/Makefile.am.
 */

#ifdef CAP_FMU_CATCH2_V2

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#endif
