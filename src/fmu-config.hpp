#pragma once
#include <cstdint>
#include <string>

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
    /* Camera cross-track (across-flight) total field of view, degrees. Used
     * to derive the flight altitude that yields a desired ground sweep
     * width: width = 2 * altitude * tan(fov / 2). Must be in (0, 180). */
    double camera_fov_deg{ 90.0 };
    /* Battery percentage at or below which a low-battery RTL is triggered. */
    int lowbat_threshold{ 20 };
    /* Interval between FSS/MAV reconnection attempts, seconds. */
    int reconnect_interval_s{ 10 };
};

/* Load the optional "fmu" config block from the given client.json. Missing,
 * malformed, or out-of-range values fall back to the FmuConfig defaults. */
auto loadFmuConfig (const std::string &config_file) -> FmuConfig;
