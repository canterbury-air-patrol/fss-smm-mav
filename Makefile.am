AUTOMAKE_OPTIONS = foreign
ACLOCAL_AMFLAGS = -I m4

if HAVE_CATCH2
SUBDIRS = src tests
else
SUBDIRS = src
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
#
# todo/ is deliberately NOT distributed: it tracks in-flight work against this
# repository, not anything a consumer of a release can act on.
EXTRA_DIST = mavlink debian docs tools LICENSE.md \
	check-code.sh .clang-format .clang-tidy
