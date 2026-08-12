AUTOMAKE_OPTIONS = foreign
ACLOCAL_AMFLAGS = -I m4

if HAVE_CATCH2
SUBDIRS = src tests
else
SUBDIRS = src

# With tests/ out of SUBDIRS there is nothing for automake's test harness to
# run, and `make check' then exits 0 having done nothing: same silence, same
# exit status as a suite that passed. That is how a build without Catch2 turns
# "run the tests before you commit" into a no-op that reports success -- so say
# what actually happened rather than exiting quietly.
if TESTS_DISABLED
# Deliberate: --disable-tests was passed at configure time. Report and succeed.
check-local:
	@echo "make check: no tests were built or run -- the test suite was"
	@echo "            disabled at configure time with --disable-tests."
else
# Not deliberate: Catch2 was simply absent. Fail, so an empty run cannot be
# mistaken for a passing one.
check-local:
	@echo "make check: no tests were built, so nothing has been verified." >&2
	@echo "            The Catch2 test framework was not found when" >&2
	@echo "            configure ran: neither catch2-with-main.pc (Catch2 3)" >&2
	@echo "            nor catch2.pc (Catch2 2.13 or newer)." >&2
	@echo "            Install Catch2 (Debian: catch2 -- either generation" >&2
	@echo "            builds the tests) and re-run configure, or configure" >&2
	@echo "            with --disable-tests to build without the test suite" >&2
	@echo "            deliberately." >&2
	@exit 1
endif
endif

# automake distributes README.md and the build machinery on its own; everything
# below has to be named or it silently does not reach the release tarball (and
# so does not reach anyone building from a `make dist` artifact rather than a
# git clone).
#
# LICENSE.md: shipping GPLv2 source without the licence text is a compliance
#   problem, and README.md's licence section links straight at it.
# tools: apconfig_check.py is the ground verification step README documents as
#   the way to confirm the autopilot config every failsafe latch depends on. A
#   release that omits it ships the instructions without the tool. Its unit
#   tests come along, since CI runs them from the same directory.
# check-code.sh/.clang-format/.clang-tidy: the static-analysis gate, so it can
#   be reproduced from a tarball rather than only from a clone.
EXTRA_DIST = mavlink debian docs tools LICENSE.md \
	check-code.sh .clang-format .clang-tidy
