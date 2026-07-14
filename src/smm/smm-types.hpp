#pragma once
#include <cstddef>
#include <string>
#include <vector>

#include <secure-string.hpp>

#include "../fmu-core-types.hpp"
#include "../ilogger.hpp"
#include "smm-command.hpp"

extern "C"
{
#include <smm-asset.h>
};

class SMMSettings
{
  private:
    std::string url{};
    flight_safety_system::secure_string user{};
    flight_safety_system::secure_string pass{};

  public:
    SMMSettings () = default;
    SMMSettings (std::string t_url, flight_safety_system::secure_string t_user,
                 flight_safety_system::secure_string t_pass)
        : url (std::move (t_url)), user (std::move (t_user)), pass (std::move (t_pass)) {};
    auto
    getURL () -> std::string
    {
        return this->url;
    };
    auto
    getUsername () -> const flight_safety_system::secure_string &
    {
        return this->user;
    };
    auto
    getPassword () -> const flight_safety_system::secure_string &
    {
        return this->pass;
    };
};

/* Result of parse_waypoints(): the decoded points plus whether every one of
 * them was a valid coordinate. Kept separate from SMMSearch so the parsing
 * decision is a pure function, testable directly against a plain
 * smm_waypoint_s array without a live smm_search handle (todo/85). */
struct ParsedWaypoints
{
    std::vector<Point> points{};
    bool all_valid{ true };
};

/* Decode every waypoint into a Point and report whether all of them are valid
 * coordinates (todo/85). One invalid waypoint invalidates the whole batch
 * rather than silently dropping or clamping just that point, so a search
 * containing it is never partially loaded onto the autopilot. */
inline auto
parse_waypoints (smm_waypoints wps, std::size_t count) -> ParsedWaypoints
{
    ParsedWaypoints result;
    for (std::size_t i = 0; i < count; i++)
    {
        Point wp (wps[i]->lat, wps[i]->lon);
        result.points.push_back (wp);
        if (!wp.isValid ())
        {
            result.all_valid = false;
        }
    }
    return result;
}

class SMMSearch
{
  private:
    std::vector<Point> points{};
    int current_point{ 0 };
    int altitude{ 0 };
    smm_search search{ nullptr };
    bool valid{ false };

  public:
    SMMSearch () = default;
    /* altitude_cap/altitude_floor: regulatory ceiling and minimum (m).
     * camera_fov_deg: camera total cross-track field of view (deg), used to
     * derive flight altitude from the search sweep width. */
    SMMSearch (smm_search, uint16_t altitude_cap, uint16_t altitude_floor, double camera_fov_deg, ILogger &logger);
    SMMSearch (SMMSearch &) = delete;
    SMMSearch (SMMSearch &&) = delete;
    auto operator= (SMMSearch &) -> SMMSearch & = delete;
    auto operator= (SMMSearch &&) -> SMMSearch & = delete;
    ~SMMSearch ();
    void
    addPoint (Point p)
    {
        this->points.push_back (p);
    };
    auto
    getCurrentPointIdx () -> int
    {
        return this->current_point;
    };
    auto
    getCurrentPoint () -> Point
    {
        return this->points[this->current_point];
    };
    auto
    getPoints () -> const std::vector<Point>
    {
        return this->points;
    };
    /* Single search waypoint by index, so a caller that needs only one point
     * (e.g. building one mission item) need not copy the whole vector. */
    auto
    getPoint (std::size_t index) -> Point
    {
        return this->points[index];
    };
    auto
    getAltitude () -> uint16_t
    {
        return this->altitude;
    };
    auto getPointsCount () -> int;
    auto reachedPoint (int point) -> bool;
    /* True if the search loaded at least one waypoint from the server. */
    auto
    isValid () const -> bool
    {
        return this->valid;
    };
    /* Commit to this search on the server; call only on a valid search. */
    auto accept () -> bool;
};
