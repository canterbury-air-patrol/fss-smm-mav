#pragma once
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

/* Pure mapping of a requested mission sequence number to the kind of mission
 * item the FMU should send for it, factored out of send_waypoint() so the
 * "seq is offset by the two setup items" arithmetic can be unit tested without
 * the MAVLink/socket stack.
 *
 * A search mission is laid out as:
 *   seq 0, 1        two setup/takeoff items (ArduPilot ignores seq 0, so the
 *                   first real item is effectively sent twice)
 *   seq 2 .. N+1    the N search waypoints, points[seq - 2]
 *   seq > N+1       a return-to-launch terminator
 *
 * A goto mission is laid out as:
 *   seq 0, 1        the goto target waypoint
 *   seq >= 2        a return-to-launch terminator
 */
enum class MissionItemKind
{
    takeoff,
    search_point,
    goto_point,
    rtl,
};

/* Which kind of mission is being uploaded — used instead of a bare bool so the
 * call sites read as ...::go_to / ...::search rather than true / false. */
enum class MissionPlanMode
{
    search,
    go_to,
};

struct MissionItem
{
    /* Defaults to rtl: a default-constructed item resolves to the safe
     * return-home terminator rather than an indeterminate kind. */
    MissionItemKind kind{ MissionItemKind::rtl };
    /* Index into the search point list; meaningful only for search_point. */
    std::size_t point_index{ 0 };
};

/* Search waypoints begin at this mission sequence number, after the two
 * setup/takeoff items at seq 0 and 1. */
inline constexpr uint16_t search_first_point_seq = 2;

/* MAVLink mission sequence number for a given search point index. Search point i
 * is uploaded at sequence i + search_first_point_seq, so resuming a search must
 * jump the autopilot to that sequence — not to the bare point index, which would
 * land on the setup/takeoff items. Inverse of the `seq - 2` in mission_item_for. */
inline auto
search_point_mission_seq (int point_index) -> uint16_t
{
    return static_cast<uint16_t> (point_index + search_first_point_seq);
}

/* The search-point index MISSION_ITEM_REACHED(seq) resumes from, for a seq at
 * or past search_first_point_seq (the caller must guard seq <
 * search_first_point_seq itself — see report_reached).
 *
 * This is NOT the inverse of search_point_mission_seq (`seq -
 * search_first_point_seq`, which would give the index of the point just
 * reached): SMMSearch::current_point tracks the *next* point to resume from,
 * one past the item MISSION_ITEM_REACHED names, matching
 * SMMSearch::reachedPoint's `current_point = point` and its `point >=
 * getPointsCount()` search-complete check. So the reached seq for search
 * point index i (= i + search_first_point_seq) maps to i + 1, i.e. `seq -
 * search_first_point_seq + 1`, equivalently `seq - 1` for the current
 * two-setup-item layout — expressed here in terms of search_first_point_seq
 * so a future change to the setup-item count only has to change one place. */
inline auto
next_search_point_after_reached_seq (uint16_t seq) -> int
{
    /* Caller contract, not defensively clamped: report_reached() already
     * guards this immediately before calling in, and silently clamping a
     * violation would hide a real bug (a wrong resume point) rather than
     * surface it. Compiled out under NDEBUG like the rest of this codebase's
     * assert()-based invariant checks. */
    assert (seq >= search_first_point_seq);
    return static_cast<int> (seq) - static_cast<int> (search_first_point_seq) + 1;
}

inline auto
mission_item_for (uint16_t seq, std::size_t num_points, MissionPlanMode mode) -> MissionItem
{
    if (mode == MissionPlanMode::go_to)
    {
        /* seq 0/1 are the goto waypoint; anything past it returns home. */
        if (seq >= 2)
        {
            return { MissionItemKind::rtl };
        }
        return { MissionItemKind::goto_point };
    }
    /* Search mission: the first two items set up the takeoff. */
    if (seq <= 1)
    {
        return { MissionItemKind::takeoff };
    }
    /* Past the last search point, terminate the mission with an RTL. */
    if (seq > num_points + 1)
    {
        return { MissionItemKind::rtl };
    }
    return { MissionItemKind::search_point, static_cast<std::size_t> (seq) - 2 };
}

/* Number of mission items to advertise in MISSION_COUNT for a given mode. The
 * autopilot then requests sequence numbers 0 .. count-1, so the count must be
 * one past the RTL terminator's sequence for send_waypoint()/mission_item_for()
 * to ever be asked for it. By construction mission_item_for(count - 1, ...) is
 * the rtl item; tests assert that invariant. */
inline auto
mission_count_for (std::size_t num_points, MissionPlanMode mode) -> std::size_t
{
    switch (mode)
    {
        case MissionPlanMode::go_to:
            /* seq 0/1 goto waypoint + seq 2 RTL terminator. */
            return 3;
        case MissionPlanMode::search:
            /* seq 0/1 setup/takeoff + the N search waypoints + one RTL terminator. */
            return num_points + 3;
    }
    /* Unreachable: every MissionPlanMode is handled above, and the switch has no
     * default so -Wswitch flags any new mode at compile time. Fail loudly rather
     * than silently miscounting a mission if one is ever reached at runtime. */
    std::abort ();
}
