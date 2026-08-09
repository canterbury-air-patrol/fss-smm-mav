#!/bin/bash -ex

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONFIG_FILE=/home/autopilot/config/fmu-client.json
export CONFIG_FILE

# Validation and JSON generation live in generate-config.sh, which
# CI also runs directly (with sample environment values) to smoke-test the
# generated config without launching cap-fmu itself.
"${SCRIPT_DIR}/generate-config.sh"

# An array, not a string: `DEBUGGER=valgrind --leak-check=full -v` parses as an
# assignment *prefix* to the command `--leak-check=full`, which under this
# script's `set -e` aborted the entrypoint with 127 before cap-fmu ever started.
# An empty array expands to no words at all, so the non-valgrind path runs the
# binary directly.
DEBUGGER=()
if [ "${RUN_IN_VALGRIND}" == "yes" ]
then
    DEBUGGER=(valgrind --leak-check=full -v)
fi

"${DEBUGGER[@]}" /src/src/cap-fmu --terminate-action="${TERMINATE_ACTION}" "${CONFIG_FILE}"
