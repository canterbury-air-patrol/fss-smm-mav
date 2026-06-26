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
echo "]" >> ${CONFIG_FILE}
echo "}" >> ${CONFIG_FILE}

DEBUGGER=
if [ "${RUN_IN_VALGRIND}" == "yes" ]
then
    DEBUGGER=valgrind --leak-check=full -v
fi

${DEBUGGER} /src/src/cap-fmu --terminate-action="${TERMINATE_ACTION}" /home/autopilot/config/fmu-client.json $MAVPROXY_HOST $MAVPROXY_PORT
