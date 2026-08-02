#!/usr/bin/env bash

set -euo pipefail

# Static code checks: formatting, cppcheck, clang-tidy, and shellcheck.
#
# Run from the top of the source tree after building with a compile database
# (e.g. `bear -- make`), which clang-tidy needs via compile_commands.json.
#
# File sets are globbed rather than enumerated so new sources are covered by
# the gate automatically. The vendored MAVLink submodule lives outside src/
# and tests/, so it is never picked up here.

# The docker/ scripts are production deployment code -- they are what actually
# launches cap-fmu on an aircraft -- but `bash -n` (all the CI smoke test used
# to do) only proves they parse. It cannot see that
# `DEBUGGER=valgrind --leak-check=full -v` is an assignment *prefix* to the
# command `--leak-check=full` rather than a string: valid syntax that aborted
# the entrypoint with 127 under `set -e`, so RUN_IN_VALGRIND=yes meant cap-fmu
# never started. shellcheck reports exactly that as SC2037.
#
# Scoped to warning-and-above: the tree is clean at that level, while `info`/
# `style` findings here are stylistic (legacy backticks, a false-positive
# "possibly unassigned" on an environment variable) and would make the gate
# noisy without catching defects.
if ! command -v shellcheck >/dev/null 2>&1; then
	echo "check-code.sh: shellcheck not found; install it (Debian: apt install shellcheck)" >&2
	exit 1
fi
# docker/ is globbed, so a script added there is covered automatically. The tree
# root cannot be: autotools drops ltmain.sh next to our own scripts, and it is
# generated, third-party, and not ours to fix (shellcheck finds ~40 problems in
# it). So the two top-level scripts we do own are named explicitly.
mapfile -t sh_files < <(find docker -name '*.sh' | sort)
shellcheck --severity=warning "${sh_files[@]}" autogen.sh check-code.sh

# clang-format output is not stable across major versions. The tree is
# formatted with clang-format 22.1.x; CI pins that exact version and points
# CLANG_FORMAT at it. Locally, set CLANG_FORMAT if your system clang-format
# is a different major version.
clang_format="${CLANG_FORMAT:-clang-format}"
mapfile -t cxx_files < <(find src tests \( -name '*.cpp' -o -name '*.hpp' \) | sort)
"$clang_format" --dry-run -Werror "${cxx_files[@]}"

# clang-format covers everything (src/ and tests/), but cppcheck and clang-tidy
# are scoped to src/ only: it is the production code, and the Catch2-based tests
# trip both tools with false positives (e.g. knownConditionTrueFalse on mock
# state) that are not worth suppressing one by one.
#
# normalCheckLevelMaxBranches is an informational note (cppcheck >= 2.15, as on
# Debian trixie in CI) emitted when branch analysis is capped on a large
# function; it is not a defect but still trips --error-exitcode, so suppress it.
cppcheck --enable=warning,performance,portability,style --error-exitcode=1 \
	--inline-suppr --suppress=normalCheckLevelMaxBranches --std=c++17 -I src src/

# The clang-tidy gate is enforced primarily by WarningsAsErrors in .clang-tidy:
# run-clang-tidy then exits non-zero and `set -o pipefail` fails the pipeline.
# HeaderFilterRegex already restricts diagnostics to our own sources. The grep
# below is a backstop; it matches clang-tidy's "path:line:col: warning:"
# diagnostic format (note the line:col) so it cannot trip on a stray
# "warning:"/"error:" appearing in prose or a suppression summary.
if [ ! -f compile_commands.json ]; then
	echo "check-code.sh: compile_commands.json not found; build with 'bear -- make' first" >&2
	exit 1
fi
# The compile database is captured from g++ and carries GCC-only warning flags
# (-Wlogical-op, -Wduplicated-cond). clang-tidy uses the clang front end, which
# does not know them; with -Werror also recorded it would turn the resulting
# -Wunknown-warning-option into a hard error and abort before running a single
# check. Disable just that diagnostic so clang-tidy's own checks still run.
run-clang-tidy -p . -extra-arg=-Wno-unknown-warning-option 'src/.*\.cpp$' 2>&1 | tee clang-tidy.log
! grep -Eq ':[0-9]+:[0-9]+: (warning|error):' clang-tidy.log
