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
 * mav_io_test.cpp loopback server's sendParamValue()) so the
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

/* Pure decision for whether an open MAVLink socket should be retired — flagged
 * `broken` so the reconnector tears it down and dials a fresh one — factored out
 * of mav_connection::heartbeat_loop() for the same reason as mav_comms_is_up()
 * above.
 *
 * This is deliberately NOT the negation of mav_comms_is_up(). Reporting the link
 * down is cheap and correct the instant a socket has produced no heartbeat (it
 * arms the comms-loss failsafe); retiring the socket costs a teardown, a redial,
 * a stream re-request and a failsafe-param re-check, and drops any mission upload
 * in flight — so it needs the stronger evidence that the socket has genuinely
 * had its chance and stayed silent.
 *
 * Silence is therefore measured from the later of the last heartbeat and the
 * moment the socket was connected. That is what separates the two cases the
 * `broken` flag has to tell apart, which a bare last_heartbeat_ts test cannot:
 *
 *   - Cold start / just reconnected: last_heartbeat_ts is 0 (or belongs to the
 *     previous socket), so the age of the last heartbeat is unbounded even though
 *     this socket has existed for milliseconds. Retiring here tears down a
 *     perfectly healthy link before the autopilot's first heartbeat could
 *     possibly have arrived, and — because the redial leaves fd == -1 for the
 *     join+connect — can itself manufacture the comms failure it was reacting to.
 *   - Half-open: heartbeats arrived and then stopped. The network path is gone
 *     but the fd has not errored, so nothing else would ever retire it. It must
 *     be retired, including the case where the peer accepts TCP but never
 *     forwards autopilot traffic at all (mavproxy up, autopilot serial dead) —
 *     which is why the connect timestamp only defers the decision by one timeout
 *     window rather than exempting a heartbeat-less socket from it.
 *
 * A closed socket (fd == -1) is never "to be retired": there is nothing to tear
 * down, and dialling from that state is attemptReconnect()'s own fd == -1 path.
 *
 * connected_since_ts is expected to be stamped before the fd is published, so an
 * open socket always has one. If that ever stops holding, a zero stamp reads as
 * the distant past and the socket is retired a timeout after its first silent
 * check — noisy, but the safe direction: never retiring a genuinely dead link
 * would leave the FMU talking to a socket the autopilot is not on. */
inline auto
mav_link_should_retire (bool fd_open, uint64_t now, uint64_t last_heartbeat_ts, uint64_t connected_since_ts,
                        uint64_t timeout_ms) -> bool
{
    if (!fd_open)
    {
        return false;
    }
    uint64_t last_contact = std::max (last_heartbeat_ts, connected_since_ts);
    /* Same guard as mav_comms_is_up(): a stamp in the future (clock anomaly)
     * must not wrap the unsigned difference into a huge age and retire a
     * working link. */
    if (now < last_contact)
    {
        return false;
    }
    return (now - last_contact) > timeout_ms;
}

/* How far HEARTBEAT-independent autopilot uptime (time_boot_ms) must go
 * backwards before it counts as a restart rather than stream skew. The two
 * sources feeding it (GLOBAL_POSITION_INT and SYSTEM_TIME) are stamped
 * milliseconds apart at different rates, so a small backwards step between
 * them is ordinary; a reboot resets the counter to ~0, which is orders of
 * magnitude larger. */
inline constexpr uint32_t autopilot_restart_margin_ms = 3000;

/* Pure decision for whether the autopilot restarted, factored out of
 * mav_connection::noteTimeBootMs() so the boundaries can be unit tested without
 * a socket (same reason as mav_comms_is_up() above).
 *
 * This is the detector for the restart shape nothing else on this end can see:
 * an ArduPilot reboot is typically a ~3s heartbeat gap, under heartbeat_loop()'s
 * 5s timeout, so it produces no comms down/up edge at all — the FSS connection
 * never notices either, so the server has nothing to re-dispatch on. A backwards
 * jump in the autopilot's own uptime counter is the evidence that survives.
 *
 * A zero on either side means "no reading", not "uptime zero", and is never a
 * restart. For last_time_boot_ms that is the initial state: nothing has been
 * observed yet, so the first sample cannot itself look like one. For
 * now_time_boot_ms it is MAVLink's convention for a field the sender did not
 * populate — treating that as a jump back to the epoch would report a restart on
 * every such message. Nothing is lost: a real reboot is not observable at uptime
 * exactly 0 (no stream emits its first message that instant), and the next
 * message milliseconds later still reads far below the pre-reboot value.
 *
 * Known false positive, deliberately accepted: time_boot_ms is uint32
 * milliseconds and wraps after ~49.7 days of continuous autopilot uptime, which
 * is indistinguishable here from a reboot. The cost is one redundant re-assert
 * of the current state plus a stream re-request — cheap next to missing a real
 * reboot. */
inline auto
autopilot_restarted (uint32_t last_time_boot_ms, uint32_t now_time_boot_ms, uint32_t margin_ms) -> bool
{
    if (last_time_boot_ms == 0 || now_time_boot_ms == 0 || now_time_boot_ms >= last_time_boot_ms)
    {
        return false;
    }
    return (last_time_boot_ms - now_time_boot_ms) > margin_ms;
}
