#pragma once
#include <cstdint>

/* Airframe / tuning parameters for the MAV connection, grouped into one struct so
 * the MAV and mav_connection constructors do not take a long list of same-typed
 * positional arguments (which is easy to transpose at a call site). The defaults
 * mirror the FmuConfig defaults; the App builds one of these from the loaded
 * config. */
struct MavParams
{
    /* Altitude a goto is flown at (metres AGL, relative to home), already clamped
     * to [altitude_floor_m, altitude_cap_m]. */
    uint16_t goto_altitude_m{ 50 };
    /* Regulatory altitude bounds (metres AGL) used to clamp commanded altitudes. */
    uint16_t altitude_floor_m{ 10 };
    uint16_t altitude_cap_m{ 122 };
    /* Intervals (microseconds) the autopilot is asked to stream position and
     * battery at. */
    uint32_t position_stream_interval_us{ 200000 };
    uint32_t battery_stream_interval_us{ 1000000 };
};
