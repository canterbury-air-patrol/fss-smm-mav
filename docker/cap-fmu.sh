#!/bin/bash -ex

# cap-fmu requires a flight-termination action; it is safety-critical and has no
# universally safe default, so it must be set explicitly via TERMINATE_ACTION.
# Validate it before generating any config so a misconfigured container fails
# loudly and early rather than after writing a config it will never use.
case "${TERMINATE_ACTION}" in
    none | disarm | terminate) ;;
    *)
        echo "Error: TERMINATE_ACTION must be set to one of: none, disarm, terminate" >&2
        exit 1
        ;;
esac

# The MAVLink endpoint used to be passed as command-line arguments; it now lives
# in the generated config's "fmu" block. Validate it here so a misconfigured
# container still fails loudly rather than silently falling back to defaults
# (an empty/non-numeric value would otherwise yield malformed JSON).
if [ -z "${MAVPROXY_HOST}" ]; then
    echo "Error: MAVPROXY_HOST must be set (the MAVLink endpoint host)" >&2
    exit 1
fi
case "${MAVPROXY_PORT}" in
    '' | *[!0-9]*)
        echo "Error: MAVPROXY_PORT must be set to a numeric port" >&2
        exit 1
        ;;
esac
# Enforce the TCP port range here too (loadFmuConfig also rejects it), so a
# misconfigured container fails fast rather than writing an out-of-range port.
if [ "${MAVPROXY_PORT}" -lt 1 ] || [ "${MAVPROXY_PORT}" -gt 65535 ]; then
    echo "Error: MAVPROXY_PORT must be between 1 and 65535 (got ${MAVPROXY_PORT})" >&2
    exit 1
fi

CONFIG_FILE=/home/autopilot/config/fmu-client.json

echo "{" > ${CONFIG_FILE}
echo "\"name\": \"${NAME}\"," >> ${CONFIG_FILE}
echo "\"ssl\": {" >> ${CONFIG_FILE}
echo "\"ca_public_key\": \"/certs/ca.public.pem\"," >> ${CONFIG_FILE}
echo "\"client_private_key\": \"/certs/${NAME}.private.pem\"," >> ${CONFIG_FILE}
echo "\"client_public_key\": \"/certs/${NAME}.public.pem\"" >> ${CONFIG_FILE}
echo "}," >>  ${CONFIG_FILE}
echo "\"servers\": [" >> ${CONFIG_FILE}
echo "{" >> ${CONFIG_FILE}
echo "\"address\": \"${SERVER1_ADDR}\"," >> ${CONFIG_FILE}
echo "\"port\": ${SERVER1_PORT}" >> ${CONFIG_FILE}
echo "}" >> ${CONFIG_FILE}
echo "]," >> ${CONFIG_FILE}
echo "\"fmu\": {" >> ${CONFIG_FILE}
echo "\"mav_address\": \"${MAVPROXY_HOST}\"," >> ${CONFIG_FILE}
echo "\"mav_port\": ${MAVPROXY_PORT}" >> ${CONFIG_FILE}
echo "}" >> ${CONFIG_FILE}
echo "}" >> ${CONFIG_FILE}

DEBUGGER=
if [ "${RUN_IN_VALGRIND}" == "yes" ]
then
    DEBUGGER=valgrind --leak-check=full -v
fi

${DEBUGGER} /src/src/cap-fmu --terminate-action="${TERMINATE_ACTION}" /home/autopilot/config/fmu-client.json
