#pragma once

#include "../fmu-core-types.hpp"
#include "../fmu-types.hpp"
#include "../smm/smm-types.hpp"
#include <cstdint>
#include <memory>

/* Deliberately has no "search" member. Entering a search is not a mode command:
 * FMUStateMachine::actionState() drives fmu_state_searching through
 * ISMM::search(), which owns the acquire and the mission upload, and the upload
 * itself ends by commanding AUTO from the MISSION_ACK handler. A
 * flight_mode_search did exist and reached mav_connection::loadSearch() with no
 * search held, dereferencing a null shared_ptr; nothing ever issued it, so it
 * was purely a way for a future caller to crash the FMU. */
enum flight_mode
{
    flight_mode_unknown,
    flight_mode_manual,
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
     * command, when it was deferred until the autopilot type is known). No caller
     * acts on that result any more — FMUStateMachine::actionState() is the only
     * one, and it discards what it collects, because every MAV-link recovery
     * re-applies the current state whether or not the earlier send got through.
     * The implementation side still reads its own copy of the same result:
     * mav_connection::commandGoto() warns when the opening MISSION_COUNT never
     * went out. gotoPosition only stashes the target — the goto is transmitted by
     * the following setMode(flight_mode_goto) — so it has nothing to report. */
    virtual auto setMode (flight_mode fm) -> bool = 0;
    virtual auto disarm () -> bool = 0;
    virtual auto terminate () -> bool = 0;
    virtual void gotoPosition (Point to) = 0;
    virtual auto setAltitude (uint32_t alt) -> bool = 0;
    virtual auto getCurrentPosition () -> Point = 0;
    virtual void registerMavCommsStatusCB (notify_mav_comms_cb cb) = 0;
    /* Report that the autopilot restarted while the link stayed up, so the
     * event loop can re-apply the commanded state to an autopilot that has
     * forgotten it. Distinct from the comms-status callback above: a typical
     * ArduPilot reboot is a ~3s heartbeat gap, which never trips the 5s
     * link-down timeout, so this fires with no comms edge either side. */
    virtual void registerAutopilotRestartCB (notify_autopilot_restart_cb cb) = 0;
    /* Load an SMM-acquired search mission onto the autopilot, and forward
     * another aircraft's position report for ADS-B rebroadcast. Not used by
     * FMUStateMachine (which only needs the action methods above); part of
     * this interface so the event-loop dispatch (EventDispatcher) can be
     * driven by the same mock in tests. */
    virtual void loadSearch (const std::shared_ptr<SMMSearch> &search) = 0;
    virtual void sendADSB (PositionData pd) = 0;
};
