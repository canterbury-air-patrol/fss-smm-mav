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

struct MissionItem
{
    /* Defaults to rtl: a default-constructed item resolves to the safe
     * return-home terminator rather than an indeterminate kind. */
    MissionItemKind kind{ MissionItemKind::rtl };
    /* Index into the search point list; meaningful only for search_point. */
    std::size_t point_index{ 0 };
};

inline auto
mission_item_for (uint16_t seq, std::size_t num_points, bool goto_active) -> MissionItem
{
    if (goto_active)
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
