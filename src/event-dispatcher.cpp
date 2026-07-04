#include "event-dispatcher.hpp"

#include <variant>

namespace
{
template <class... Ts> struct overloaded : Ts...
{
    using Ts::operator()...;
};
template <class... Ts> overloaded (Ts...) -> overloaded<Ts...>;
} // namespace

EventDispatcher::EventDispatcher (FMUStateMachine &t_state_machine, IMAV &t_mav, ISMM &t_smm, IFSSReporter &t_fss,
                                  Logger &t_logger, std::string t_asset_name, int t_lowbat_threshold)
    : state_machine (t_state_machine), mav (t_mav), smm (t_smm), fss (t_fss), logger (t_logger),
      asset_name (std::move (t_asset_name)), lowbat_threshold (t_lowbat_threshold)
{
    state_machine.setStateChangeCB (
        [this] (FMUState s)
        {
            logger.log (std::string ("STATE ") + fmu_state_name (s));
            if (s != fmu_state_searching)
            {
                smm_rtl_replay_pending = false;
            }
        });
}

void
EventDispatcher::dispatch (const event &e)
{
    std::visit (
        overloaded{
            [&] (const FSSCommandEvent &ce)
            {
                logger.log (std::string ("CMD fss ") + fss_cmd_name (ce.command));
                FSSCommandResolution res = state_machine.FSSNewCommand (ce.command, ce.target);
                /* Acknowledge the resolved outcome back to FSS (no-op unless the
                 * originating connection negotiated the command-ack feature).
                 * Routed through the FSS send worker rather than sent inline: the
                 * ack ends in a blocking send() and must not stall the event loop
                 * behind a hung FSS peer. */
                if (ce.ack)
                {
                    fss.postAck (ce.ack, res);
                }
            },
            [&] (FSSCommsStatus status)
            {
                logger.log (std::string ("COMMS fss ") + (status == fss_comms_failure ? "failure" : "okay"));
                state_machine.setCommsFailure (status == fss_comms_failure);
            },
            [&] (MavCommsStatus status)
            {
                logger.log (std::string ("COMMS mav ") + (status == MavCommsStatus::failure ? "failure" : "okay"));
                state_machine.setMavCommsFailure (status == MavCommsStatus::failure);
                /* Replay an SmmRtl whose send failed while the link was down
                 * (todo/71), the same recovery point the state machine's own
                 * pending_replay_state uses (todo/46). */
                if (status == MavCommsStatus::ok && smm_rtl_replay_pending && state_machine.isSearching ())
                {
                    logger.log ("CMD smm rtl (replay)");
                    smm_rtl_replay_pending = !mav.setMode (flight_mode_rtl);
                }
            },
            [&] (SMMSettings settings)
            {
                logger.log ("SMM connect " + settings.getURL ());
                smm.connect (settings.getURL (), settings.getUsername (), settings.getPassword (), asset_name);
            },
            [&] (const PositionData &pd)
            {
                fss.reportPosition (pd);
                smm.reportPosition (pd);
            },
            [&] (const ReachedPoint &rp)
            {
                /* A MISSION_ITEM_REACHED is only meaningful as search progress
                 * when the search mission is what is actually loaded on the
                 * autopilot. A goto (or its RTL terminator) can also produce a
                 * reached event while a search is only paused (todo/69); without
                 * this guard that would rewrite the held search's current_point
                 * and report bogus search status to FSS. Same guard as
                 * SmmLoadSearch/SmmRtl. */
                if (state_machine.isSearching ())
                {
                    logger.log ("WAYPOINT " + std::to_string (rp.point));
                    fss.reachedPoint (rp.point, smm.currentSearchPoints ());
                    smm.reachedPoint (rp.point);
                }
            },
            [&] (const SmmLoadSearch &ls)
            {
                /* The SMM worker acquired/resumed a search. Only load it onto the
                 * autopilot if the FMU is still searching: a command/latch that
                 * took over since the acquire started (rtl/terminate/etc.) must
                 * win, and this outcome is dropped rather than overriding it
                 * (todo/33). */
                if (state_machine.isSearching ())
                {
                    mav.loadSearch (ls.search);
                }
            },
            [&] (const SmmRtl &)
            {
                /* SMM could not acquire/accept a search and wants to fly home.
                 * Same guard as SmmLoadSearch: suppress it if a higher-priority
                 * command/latch already took over. This commands the MAV
                 * directly rather than going through the state machine (todo/70
                 * tracks folding it in); current_state intentionally stays
                 * fmu_state_searching for the duration — same as the mission's
                 * own RTL terminator flying home after a completed search
                 * (README: continue mode auto-acquires the next search) — so
                 * this is a deliberate choice, not an oversight. */
                if (state_machine.isSearching ())
                {
                    logger.log ("CMD smm rtl");
                    bool sent = mav.setMode (flight_mode_rtl);
                    if (!sent)
                    {
                        logger.log ("COMMS mav rtl send failed, will replay on link recovery");
                    }
                    smm_rtl_replay_pending = !sent;
                }
            },
            [&] (BatteryData bd)
            {
                auto remaining = bd.getRemaining ();
                /* A remaining of -1 means "unknown"; only a real reading below
                 * the threshold counts as low. The state machine debounces
                 * these, so feed it every reading (low or not) to keep its
                 * consecutive-low counter accurate. */
                bool low = remaining >= 0 && remaining < lowbat_threshold;
                if (low)
                {
                    logger.log ("BATTERY low " + std::to_string (remaining) + "%");
                }
                state_machine.setLowBattery (low);
                fss.reportBatteryStatus (bd);
            },
            [&] (OtherAircraftReport oar)
            {
                if (oar.pd.getCallSign () != asset_name)
                {
                    mav.sendADSB (oar.pd);
                }
            },
            [] (const Nudge &) {},
        },
        e);
}
