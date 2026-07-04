#pragma once

#include "../fmu-core-types.hpp"
#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include <cstdint>
#include <memory>

enum flight_mode
{
    flight_mode_unknown,
    flight_mode_manual,
    flight_mode_search,
    flight_mode_rtl,
    flight_mode_goto,
    flight_mode_hold,
};

class IMAV
{
  public:
    IMAV () = default;
    IMAV (const IMAV &) = delete;
    IMAV (IMAV &&) = delete;
    auto operator= (const IMAV &) -> IMAV & = delete;
    auto operator= (IMAV &&) -> IMAV & = delete;
    virtual ~IMAV () = default;

    /* The action methods that transmit a command to the autopilot return whether
     * it was actually sent (false when the MAV link is down, or for a mode
     * command, when it was deferred until the autopilot type is known). The state
     * machine uses this to replay a safety-critical action once the link recovers
     * (todo/46). gotoPosition only stashes the target — the goto is transmitted by
     * the following setMode(flight_mode_goto) — so it has nothing to report. */
    virtual auto setMode (flight_mode fm) -> bool = 0;
    virtual auto disarm () -> bool = 0;
    virtual auto terminate () -> bool = 0;
    virtual void gotoPosition (Point to) = 0;
    virtual auto setAltitude (uint32_t alt) -> bool = 0;
    virtual auto getCurrentPosition () -> Point = 0;
    virtual void registerMavCommsStatusCB (notify_mav_comms_cb cb) = 0;
    /* Load an SMM-acquired search mission onto the autopilot, and forward
     * another aircraft's position report for ADS-B rebroadcast. Not used by
     * FMUStateMachine (which only needs the action methods above); part of
     * this interface so the event-loop dispatch (EventDispatcher, todo/76)
     * can be driven by the same mock in tests. */
    virtual void loadSearch (const std::shared_ptr<SMMSearch> &search) = 0;
    virtual void sendADSB (PositionData pd) = 0;
};
