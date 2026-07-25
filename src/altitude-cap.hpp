#pragma once
#include <cstdint>

/* Pure breach-decision helper for the continuous altitude-cap enforcement
 * latch (todo/92), factored out of FMUStateMachine::setCurrentAltitude so
 * the safety-critical over-cap comparison is unit-testable without
 * constructing a state machine. Mirrors smm/search-altitude.hpp's style. */

/* True when altitude_agl_m exceeds the regulatory ceiling altitude_cap_m.
 * altitude_agl_m must be AGL (relative to home) — the frame altitude_cap_m
 * and MAVLink's relative_alt share, NOT the MSL frame PositionData::
 * getAltitudeMetres() carries (see todo/99). Strict '>', not '>=': a
 * search/goto/direct-altitude command is legitimately clamped to fly exactly
 * at the cap (clamp_search_altitude, clamp_command_altitude), so "at the cap"
 * must not itself read as a breach. */
inline auto
over_altitude_cap (double altitude_agl_m, uint16_t altitude_cap_m) -> bool
{
    return altitude_agl_m > static_cast<double> (altitude_cap_m);
}
