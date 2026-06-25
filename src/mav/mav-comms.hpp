#pragma once
#include <cstdint>

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
