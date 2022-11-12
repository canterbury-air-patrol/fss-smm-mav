#!/bin/bash -ex

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

${DEBUGGER} /src/src/cap-fmu /home/autopilot/config/fmu-client.json $MAVPROXY_HOST $MAVPROXY_PORT
