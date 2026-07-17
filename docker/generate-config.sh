#!/bin/bash -e

# Generates the FSS client config (fmu-client.json) from environment
# variables using jq, so string values are properly JSON-escaped (todo/56)
# instead of raw echo/interpolation, which could break the config or inject
# additional fields on a value containing a quote, backslash, or newline.
#
# Split out of cap-fmu.sh so CI (and any other caller) can exercise config
# generation alone, with sample environment values, without needing real
# certs or a MAV connection to launch cap-fmu itself.
#
# CONFIG_FILE may be overridden by the caller (CI points it at a scratch
# path); it defaults to the real deployment location.
CONFIG_FILE="${CONFIG_FILE:-/home/autopilot/config/fmu-client.json}"

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

if [ -z "${NAME}" ]; then
    echo "Error: NAME must be set (the FSS asset/client name)" >&2
    exit 1
fi
if [ -z "${SERVER1_ADDR}" ]; then
    echo "Error: SERVER1_ADDR must be set (the FSS server address)" >&2
    exit 1
fi
# SERVER1_PORT previously had no validation at all (todo/56); apply the same
# numeric/range check as MAVPROXY_PORT.
case "${SERVER1_PORT}" in
    '' | *[!0-9]*)
        echo "Error: SERVER1_PORT must be set to a numeric port" >&2
        exit 1
        ;;
esac
if [ "${SERVER1_PORT}" -lt 1 ] || [ "${SERVER1_PORT}" -gt 65535 ]; then
    echo "Error: SERVER1_PORT must be between 1 and 65535 (got ${SERVER1_PORT})" >&2
    exit 1
fi

# SERVER2_ADDR/PORT are optional: a second, independent FSS server for
# redundancy deployments. The FSS client already supports an arbitrary
# "servers" list (each connection managed and reconnected independently),
# this just exposes a second entry the same way SERVER1 does. Unset
# SERVER2_ADDR reproduces the single-server config exactly as before.
if [ -n "${SERVER2_ADDR}" ]; then
    case "${SERVER2_PORT}" in
        '' | *[!0-9]*)
            echo "Error: SERVER2_PORT must be set to a numeric port" >&2
            exit 1
            ;;
    esac
    if [ "${SERVER2_PORT}" -lt 1 ] || [ "${SERVER2_PORT}" -gt 65535 ]; then
        echo "Error: SERVER2_PORT must be between 1 and 65535 (got ${SERVER2_PORT})" >&2
        exit 1
    fi
fi

# LOG_DIR is optional (todo/78): the image creates and owns the cap-fmu
# default (/var/log/cap-fmu) so logging works out of the box, but a
# deployment that wants logs to survive container recreation can set LOG_DIR
# to a mounted volume instead. Omitted entirely when unset, so the default
# in FmuConfig (and this image's directory) is what applies -- not a
# duplicated literal here that could drift from it.
LOG_DIR="${LOG_DIR:-}"

# jq -n builds the document from typed arguments: --arg values are always
# emitted as properly-escaped JSON strings (a NAME/SERVER1_ADDR/MAVPROXY_HOST
# containing a quote, backslash, or newline cannot break the document or
# inject a field), and the ports are passed with --argjson so they land as
# JSON numbers, matching the numeric fields loadFmuConfig expects.
# server2_addr defaults to "" (rather than leaving --arg unset) so the jq
# filter can key the second servers[] entry on it being non-empty; its port
# is only ever read when server2_addr is non-empty, so an unset/zero
# server2_port there is inert.
jq -n \
    --arg name "${NAME}" \
    --arg server_addr "${SERVER1_ADDR}" \
    --argjson server_port "${SERVER1_PORT}" \
    --arg server2_addr "${SERVER2_ADDR:-}" \
    --argjson server2_port "${SERVER2_PORT:-0}" \
    --arg mav_host "${MAVPROXY_HOST}" \
    --argjson mav_port "${MAVPROXY_PORT}" \
    --arg log_dir "${LOG_DIR}" \
    '{
        name: $name,
        ssl: {
            ca_public_key: "/certs/ca.public.pem",
            client_private_key: ("/certs/" + $name + ".private.pem"),
            client_public_key: ("/certs/" + $name + ".public.pem")
        },
        servers: (
            [{ address: $server_addr, port: $server_port }]
            + (if $server2_addr != "" then [{ address: $server2_addr, port: $server2_port }] else [] end)
        ),
        fmu: ({
            mav_address: $mav_host,
            mav_port: $mav_port
        } + (if $log_dir != "" then { log_dir: $log_dir } else {} end))
    }' > "${CONFIG_FILE}"

# Belt-and-braces: confirm the file we just wrote actually parses, rather than
# trusting jq's own output implicitly.
jq empty "${CONFIG_FILE}"
