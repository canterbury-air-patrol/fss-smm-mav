#pragma once
#include <cstdint>
#include <string>

/* Local, per-asset configuration loaded from the asset's client.json.
 *
 * These describe physical/regulatory properties of *this* aircraft and are
 * deliberately kept separate from the server-pushed SMMSettings: the SMM
 * server tells us where to connect, but the regulatory ceiling and the
 * camera geometry are properties of the airframe, not the mission. */
struct AssetConfig
{
    /* Regulatory ceiling for derived search altitude, metres AGL
     * (400ft ~= 122m). */
    uint16_t altitude_cap_m{ 122 };
    /* Camera cross-track (across-flight) total field of view, degrees. Used
     * to derive the flight altitude that yields a desired ground sweep
     * width: width = 2 * altitude * tan(fov / 2). Must be in (0, 180). */
    double camera_fov_deg{ 90.0 };
};

/* Load the optional "fmu" config block from the given client.json. Missing,
 * malformed, or out-of-range values fall back to the AssetConfig defaults. */
auto loadAssetConfig (const std::string &config_file) -> AssetConfig;
