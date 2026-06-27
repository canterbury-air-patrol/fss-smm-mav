#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

/* The canonical internal altitude unit for PositionData is metres. Every
 * protocol boundary converts explicitly through these helpers so the unit
 * contract is unambiguous and cannot silently drift:
 *
 *   - MAVLink GLOBAL_POSITION_INT.alt / ADSB_VEHICLE.altitude are millimetres.
 *   - The FSS wire protocol carries altitude in feet (the FSS web UI labels
 *     both the reported position and the altitude command "ft").
 *   - The SMM asset API expects metres.
 */

/* 1 foot == 0.3048 m exactly, so metres convert to feet by dividing. */
inline constexpr double metres_per_foot = 0.3048;

inline auto
metres_to_feet (double metres) -> double
{
    return metres / metres_per_foot;
}

inline auto
feet_to_metres (double feet) -> double
{
    return feet * metres_per_foot;
}

inline auto
mav_mm_to_metres (int32_t millimetres) -> double
{
    return static_cast<double> (millimetres) / 1000.0;
}

inline auto
metres_to_mav_mm (double metres) -> int32_t
{
    /* std::lround is undefined for non-finite input, and a large magnitude would
     * overflow the int32_t cast. Guard and clamp here so every caller is covered
     * (the MAVLink field is int32_t anyway): a bad altitude yields 0 ("unknown")
     * rather than UB or a wrapped value. */
    if (!std::isfinite (metres))
    {
        return 0;
    }
    const double millimetres = metres * 1000.0;
    if (millimetres >= static_cast<double> (std::numeric_limits<int32_t>::max ()))
    {
        return std::numeric_limits<int32_t>::max ();
    }
    if (millimetres <= static_cast<double> (std::numeric_limits<int32_t>::min ()))
    {
        return std::numeric_limits<int32_t>::min ();
    }
    return static_cast<int32_t> (std::lround (millimetres));
}
