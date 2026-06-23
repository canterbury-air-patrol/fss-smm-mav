#pragma once
#include <cstddef>
#include <cstdint>

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
    if (mode == MissionPlanMode::go_to)
    {
        /* seq 0/1 goto waypoint + seq 2 RTL terminator. */
        return 3;
    }
    /* seq 0/1 setup/takeoff + the N search waypoints + one RTL terminator. */
    return num_points + 3;
}
