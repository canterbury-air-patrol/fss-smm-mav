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
    # The flight image does not ship valgrind (docker/runtime-setup.sh installs
    # it only under --build-arg INCLUDE_VALGRIND=yes). Say which image to use
    # rather than letting `set -e` abort on a command not found, which is the
    # same silent 127 the SC2037 bug produced.
    if ! command -v valgrind >/dev/null 2>&1
    then
        echo "cap-fmu.sh: RUN_IN_VALGRIND=yes, but this image has no valgrind." >&2
        echo "            The flight image deliberately omits it; rebuild with" >&2
        echo "            docker build --build-arg INCLUDE_VALGRIND=yes and run" >&2
        echo "            that image instead." >&2
        exit 1
    fi
    DEBUGGER=(valgrind --leak-check=full -v)
fi

# /usr/bin/cap-fmu, not /src/src/cap-fmu: the image installs the binary
# (docker/build.sh's `make install' with --prefix=/usr, staged into the runtime
# stage) rather than running it out of a build tree that then has to ship. Same
# path the .deb installs to.
"${DEBUGGER[@]}" /usr/bin/cap-fmu --terminate-action="${TERMINATE_ACTION}" "${CONFIG_FILE}"
