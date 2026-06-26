#pragma once

#include <cstdint>

/* MAVLink BATTERY_STATUS reports cell voltages in millivolts and uses
 * UINT16_MAX (0xFFFF) in a cell slot to mean "unknown". Reporting that slot
 * verbatim yields a spurious 65.535 V.
 *
 * ArduPilot places the total pack voltage in cell 0 (it generally has no
 * per-cell monitoring), so we report cell 0 as the pack voltage rather than
 * summing the array, which would double-count.
 *
 * The FSS transport packs voltage as a non-negative fixed-point value and
 * treats 0 as its own "unknown" convention (it has no NaN/negative sentinel;
 * negatives clamp to 0 on the wire). So 0.0 is the correct "unknown" value to
 * emit: an unknown UINT16_MAX slot maps to 0.0, and an empty 0 mV slot already
 * coincides with that sentinel. */
inline constexpr double battery_voltage_unknown = 0.0;

inline auto
battery_pack_voltage_v (uint16_t cell0_mv) -> double
{
    if (cell0_mv == UINT16_MAX)
    {
        return battery_voltage_unknown;
    }
    return static_cast<double> (cell0_mv) / 1000.0;
}
