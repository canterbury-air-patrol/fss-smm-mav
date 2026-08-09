#pragma once

#include "../fmu-core-types.hpp"
#include <secure-string.hpp>
#include <string>

class ISMM
{
  public:
    ISMM () = default;
    ISMM (const ISMM &) = delete;
    ISMM (ISMM &&) = delete;
    auto operator= (const ISMM &) -> ISMM & = delete;
    auto operator= (ISMM &&) -> ISMM & = delete;
    virtual ~ISMM () = default;

    virtual void search (Point current_pos) = 0;
    virtual void cancelSearch () = 0;
    /* Outcome-reporting methods used by the event-loop dispatch
     * (EventDispatcher), not by FMUStateMachine (which only needs the two
     * methods above). Part of this interface so both can be driven by the
     * same mock in tests. */
    virtual void connect (const std::string &host, const flight_safety_system::secure_string &user,
                          const flight_safety_system::secure_string &pass, const std::string &asset_name) = 0;
    virtual void reportPosition (PositionData pd) = 0;
    virtual void reachedPoint (int point) = 0;
    virtual auto currentSearchPoints () -> int = 0;
};
