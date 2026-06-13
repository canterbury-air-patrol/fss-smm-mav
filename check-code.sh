#!/usr/bin/env bash

set -euo pipefail

# Static code checks: formatting, cppcheck, and clang-tidy.
#
# Run from the top of the source tree after building with a compile database
# (e.g. `bear -- make`), which clang-tidy needs via compile_commands.json.
#
# File sets are globbed rather than enumerated so new sources are covered by
# the gate automatically. The vendored MAVLink submodule lives outside src/
# and tests/, so it is never picked up here.

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
run-clang-tidy -p . 'src/.*\.cpp$' 2>&1 | tee clang-tidy.log
! grep -Eq ':[0-9]+:[0-9]+: (warning|error):' clang-tidy.log
