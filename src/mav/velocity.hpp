#pragma once
#include <cmath>
#include <cstdint>

/* Horizontal ground speed magnitude (cm/s) from the signed component velocities
 * of a MAVLink GLOBAL_POSITION_INT (vx, vy are int16_t cm/s), factored out so the
 * overflow-safe computation can be unit tested without a socket.
 *
 * Computing this as sqrt((vx * vx) + (vy * vy)) does the squares in int (the
 * int16_t operands promote to int), and at the INT16_MIN extreme the sum exceeds
 * INT_MAX and overflows -- signed overflow is undefined behaviour (and UBSan
 * flags it). std::hypot works in double and forms no intermediate square, so it
 * cannot overflow. The magnitude of two int16_t components is at most
 * hypot(32768, 32768) ~= 46341, which always fits in uint16_t, so no saturation
 * is needed; the cast truncates the sub-unit fraction as the original did. */
inline auto
horizontal_velocity (int16_t vx, int16_t vy) -> uint16_t
{
    return static_cast<uint16_t> (std::hypot (static_cast<double> (vx), static_cast<double> (vy)));
}
