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
    /* Number of consecutive over-/under-cap AGL readings required to trip, or
     * clear, the continuous altitude-cap enforcement latch
     * (FMUStateMachine::setCurrentAltitude). Symmetric: the same count
     * debounces both directions. A single noisy EKF altitude sample must not
     * trip a hard RTL, nor must a single dip back under the cap release one
     * prematurely. Its real-world duration scales with
     * position_stream_interval_ms (default 200ms * 5 = ~1s) -- much tighter
     * than low_battery_latch_count's, since an altitude-ceiling violation is
     * time-critical in a way a battery reading is not. */
    int altitude_breach_latch_count{ 5 };
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
    /* Number of consecutive low-battery readings required to engage the
     * low-battery RTL latch (see FMUStateMachine::setLowBattery). A single
     * noisy/spurious sample must not ground the mission; airframe-dependent
     * battery/sensor noise may want a different debounce. Note the real-world
     * duration this represents scales with battery_stream_interval_ms. */
    int low_battery_latch_count{ 4 };
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
    /* Upper bound (seconds) on a MAV TCP connect attempt: a black-holed or
     * unreachable autopilot endpoint must not stall startup/reconnect beyond
     * this. */
    int mav_connect_timeout_s{ 5 };
    /* Upper bound (seconds) applied to a blocking MAV send (SO_SNDTIMEO and,
     * where available, TCP_USER_TIMEOUT): a peer that stops reading, or a
     * network path that silently disappears, must not pin a sender —
     * including the event-loop thread issuing a safety command — beyond
     * this. Kept tighter than mav_connect_timeout_s by default/range: unlike
     * a slow connect, a blocked send runs on/behind the live event loop, so
     * a large configured value directly extends comms-failure detection
     * latency. */
    int mav_send_timeout_s{ 2 };
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
