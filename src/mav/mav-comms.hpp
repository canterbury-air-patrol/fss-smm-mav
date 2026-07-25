#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

/* Copy `name` into the fixed-width `dest` (size `dest_len`), zero-padding any
 * remainder. For MAVLink fixed-width char[] fields (e.g. param_id) that are
 * NOT NUL-terminated when full — memcpy, not strncpy, since a source exactly
 * dest_len long has no room left for a terminator (strncpy's
 * -Wstringop-truncation flags that as a mistake, which for this field it is
 * not: the field is a fixed-width slot, not a C string). Shared by every
 * caller that packs a MAVLink param_id (checkFailsafeConfig() and the
 * mav_io_test.cpp loopback server's sendParamValue(), todo/91) so the
 * out-of-bounds-read hazard of passing a short literal straight to a
 * mavlink_msg_*_pack_chan() call (which always reads dest_len bytes from its
 * source pointer) is fixed in exactly one place. */
inline void
pack_fixed_width_field (char *dest, std::size_t dest_len, const char *name)
{
    std::memset (dest, 0, dest_len);
    std::memcpy (dest, name, std::min (std::strlen (name), dest_len));
}

/* Pure decision for whether the autopilot (MAVLink) link is currently up,
 * factored out of mav_connection::heartbeat_loop() so the cold-start and
 * heartbeat-timeout transitions can be unit tested without a socket.
 *
 * The link is up only when the socket is open AND a heartbeat has been seen
 * within the timeout window. A closed socket (fd == -1: never connected at cold
 * start, or torn down) is down regardless of the last-heartbeat timestamp.
 *
 * This is what closes the cold-start gap: with no socket there is no heartbeat
 * to time out, so an unestablished link must be reported down rather than left
 * at the state machine's optimistic "comms ok" default. Likewise a freshly
 * (re)connected socket that has not yet produced a heartbeat is down until the
 * autopilot is actually heard from. */
inline auto
mav_comms_is_up (bool fd_open, uint64_t now, uint64_t last_heartbeat_ts, uint64_t timeout_ms) -> bool
{
    if (!fd_open)
    {
        return false;
    }
    /* now is monotonic against last_heartbeat_ts (both current_timestamp_ms()),
     * so now >= last_heartbeat_ts in practice; guard the subtraction anyway so a
     * clock anomaly cannot wrap the unsigned difference into a huge "aged out". */
    if (now < last_heartbeat_ts)
    {
        return true;
    }
    return (now - last_heartbeat_ts) <= timeout_ms;
}
