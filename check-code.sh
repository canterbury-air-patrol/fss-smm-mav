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

# normalCheckLevelMaxBranches is an informational note (emitted by cppcheck
# >= 2.15 when it caps branch analysis on a large function); it is not a code
# defect but still trips --error-exitcode, so suppress it explicitly.
cppcheck --enable=warning,performance,portability,style --error-exitcode=1 \
	--inline-suppr --suppress=normalCheckLevelMaxBranches --std=c++17 -I src src/

# The clang-tidy gate is enforced primarily by WarningsAsErrors in .clang-tidy:
# run-clang-tidy then exits non-zero and `set -o pipefail` fails the pipeline.
# HeaderFilterRegex already restricts diagnostics to our own sources. The grep
# below is a backstop that matches clang-tidy's "path:line:col: warning:"
# diagnostic format rather than any stray "warning"/"error" word in the output.
if [ ! -f compile_commands.json ]; then
	echo "check-code.sh: compile_commands.json not found; build with 'bear -- make' first" >&2
	exit 1
fi
run-clang-tidy -p . 'src/.*\.cpp$' 2>&1 | tee clang-tidy.log
! grep -Eq ': (warning|error):' clang-tidy.log
