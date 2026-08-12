#ifndef CATCH2_COMPAT_HPP
#define CATCH2_COMPAT_HPP

/*
 * One include for the whole suite, so the tests build against either Catch2
 * generation.
 *
 * Catch2 3 is what this is written for, and what CI and any recent Debian
 * provide. Debian bookworm -- the distribution the aircraft image is built
 * from -- has only Catch2 2.13 in main, and the alternative to supporting it
 * was building the bookworm package with no test suite compiled at all. For
 * flight software that is the worse trade, and the gap is small: the suite
 * uses TEST_CASE, SECTION, REQUIRE, REQUIRE_FALSE, REQUIRE_THROWS[_AS] and
 * WARN, which are spelled identically in both, plus Approx, which moved.
 *
 * Everything the two generations disagree about is bridged here, so no test
 * file needs to know which one it is compiled against. Keep it that way: if a
 * new test needs something Catch2 2.13 does not have, bridge it here (or, if
 * it cannot be bridged, that is the point at which dropping 2.x support has to
 * be decided deliberately rather than discovered by a bookworm build failing).
 *
 * CAP_FMU_CATCH2_V2 comes from configure, which picks the generation by which
 * pkg-config module exists: catch2-with-main.pc (3.x, links Catch2's own main)
 * or catch2.pc alone (2.x, header-only, so tests/catch2-main.cpp supplies the
 * main instead).
 */

#ifdef CAP_FMU_CATCH2_V2

#include <catch2/catch.hpp>

namespace Catch
{
/* Catch2 2.x has this as Catch::Detail::Approx (and a using-declaration for it
 * in the global namespace); 3.x promoted it to Catch::Approx. The tests use the
 * 3.x spelling. */
using Detail::Approx;
}

#else

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#endif

#endif
