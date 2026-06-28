#pragma once
#include <cstdint>
#include <string>

#include "fmu-core-types.hpp"

/* Local, per-asset configuration loaded from the "fmu" block of the asset's
 * client.json.
 *
 * Covers both airframe properties (altitude cap, camera geometry) and
 * operational tuning (battery threshold, reconnect interval). Kept separate
 * from the server-pushed SMMSettings: the SMM server tells us where to
 * connect, but these belong to the asset, not the mission. */
struct FmuConfig
{
    /* Regulatory ceiling for derived search altitude, metres AGL
     * (400ft ~= 122m). */
    uint16_t altitude_cap_m{ 122 };
    /* Minimum search altitude, metres AGL. The derived altitude is never
     * flown below this, so a tiny/zero sweep width cannot put the aircraft at
     * ground level. Held <= altitude_cap_m. */
    uint16_t altitude_floor_m{ 10 };
    /* Altitude an FSS "goto" is flown at, metres AGL (relative to home). A goto
     * command carries only a target position, not an altitude, so the FMU
     * supplies one here. Held <= altitude_cap_m (and >= altitude_floor_m) so a
     * goto can never be commanded above the regulatory ceiling or into the
     * ground. */
    uint16_t goto_altitude_m{ 50 };
    /* Camera cross-track (across-flight) total field of view, degrees. Used
     * to derive the flight altitude that yields a desired ground sweep
     * width: width = 2 * altitude * tan(fov / 2). Must be in (0, 180). */
    double camera_fov_deg{ 90.0 };
    /* Battery percentage below which a low-battery RTL is triggered (strict: a
     * reading equal to the threshold is not low — see the `< lowbat_threshold`
     * test in main.cpp). */
    int lowbat_threshold{ 20 };
    /* Interval between FSS/MAV reconnection attempts, seconds. */
    int reconnect_interval_s{ 10 };
    /* Rate at which the autopilot is asked to stream GLOBAL_POSITION_INT,
     * milliseconds between messages (200ms = 5Hz). Faster airframes (fixed-wing)
     * may want a shorter interval than a slow rover. */
    int position_stream_interval_ms{ 200 };
    /* Rate at which the autopilot is asked to stream BATTERY_STATUS,
     * milliseconds between messages. */
    int battery_stream_interval_ms{ 1000 };
    /* Minimum interval between SMM position reports, milliseconds. Throttles the
     * once-per-position HTTP report to the SMM server independently of the
     * (faster) MAVLink position stream above. */
    int smm_position_report_interval_ms{ 1000 };
    /* Connect and total-transfer timeouts for SMM HTTP requests, seconds. The
     * smm-asset library defaults (30s/60s) are far too long for a flight-safety
     * loop: a single slow or hung SMM endpoint would otherwise block the call
     * for the full TCP window. These bound that worst case so SMM latency cannot
     * stall the FMU for tens of seconds (applied via
     * smm_asset_connection_timeouts_set). Held <= the library defaults. */
    int smm_connect_timeout_s{ 5 };
    int smm_transfer_timeout_s{ 10 };
    /* MAVLink autopilot endpoint the FMU connects to (host/IP and TCP port).
     * Previously passed as command-line arguments; now part of the asset config
     * so a deployment is described by one file. Defaults suit a local SITL /
     * mavproxy. */
    std::string mav_address{ "127.0.0.1" };
    int mav_port{ 5760 };
    /* Logging verbosity: error, info (default), or debug. */
    LogLevel log_level{ LogLevel::info };
    /* Directory the rotating fmu.log is written to. Defaults to the system
     * location, but the FMU normally runs as a non-root user, so it is
     * configurable to a path that user can write (the directory is created if
     * missing; logging is skipped with a warning if it cannot be). */
    std::string log_dir{ "/var/log/cap-fmu" };
};

/* Load the optional "fmu" config block from the given client.json. Missing,
 * malformed, or out-of-range values fall back to the FmuConfig defaults. */
auto loadFmuConfig (const std::string &config_file) -> FmuConfig;
