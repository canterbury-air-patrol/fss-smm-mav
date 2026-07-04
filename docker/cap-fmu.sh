#!/bin/bash -ex

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONFIG_FILE=/home/autopilot/config/fmu-client.json
export CONFIG_FILE

# Validation and JSON generation (todo/56) live in generate-config.sh, which
# CI also runs directly (with sample environment values) to smoke-test the
# generated config without launching cap-fmu itself.
"${SCRIPT_DIR}/generate-config.sh"

DEBUGGER=
if [ "${RUN_IN_VALGRIND}" == "yes" ]
then
    DEBUGGER=valgrind --leak-check=full -v
fi

${DEBUGGER} /src/src/cap-fmu --terminate-action="${TERMINATE_ACTION}" "${CONFIG_FILE}"
