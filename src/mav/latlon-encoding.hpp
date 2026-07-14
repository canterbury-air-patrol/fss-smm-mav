#pragma once
#include <cmath>
#include <cstdint>

/* MAVLink's *_INT messages encode latitude/longitude as whole degrees * 1e7
 * ("degE7") in an int32_t. This conversion is shared by every lat/lon field
 * mav_connection sends (commandGoto's goto_point, send_waypoint's search
 * points, sendADSB) and the one it decodes (GLOBAL_POSITION_INT), so it is
 * factored out here (todo/80) to be unit tested directly rather than only
 * indirectly through MAVLink packing. */
constexpr double LAT_LNG_COV = 0.0000001;

/* Encode a signed decimal-degrees value (latitude in [-90, 90] or longitude in
 * [-180, 180]) as degE7. Truncates toward zero, same as the inline cast this
 * replaced -- sub-degE7 precision is below any meaningful GPS/command
 * resolution, so truncation vs rounding is not a behavioural concern.
 *
 * Every caller is expected to have already validated its input (Point::isValid,
 * todo/85); this clamp is a last-resort backstop so a future caller that skips
 * that validation gets a deterministic, garbage-but-defined int32_t instead of
 * undefined behavior -- converting a NaN, an infinity, or a magnitude beyond
 * int32_t's range via a raw static_cast is UB in C++. Representable inputs
 * (including the just-past-boundary values todo/80 tests) are unaffected. */
inline auto
degrees_to_degE7 (double degrees) -> int32_t
{
    if (!std::isfinite (degrees))
    {
        return 0;
    }
    double degE7 = degrees / LAT_LNG_COV;
    if (degE7 <= static_cast<double> (INT32_MIN))
    {
        return INT32_MIN;
    }
    if (degE7 >= static_cast<double> (INT32_MAX))
    {
        return INT32_MAX;
    }
    return static_cast<int32_t> (degE7);
}

/* Inverse of degrees_to_degE7: decode a wire degE7 value back to decimal
 * degrees. */
inline auto
degE7_to_degrees (int32_t degE7) -> double
{
    return static_cast<double> (degE7) * LAT_LNG_COV;
}
