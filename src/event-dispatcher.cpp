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
    state_machine.setStateChangeCB ([this] (FMUState s) { logger.log (std::string ("STATE ") + fmu_state_name (s)); });
}

void
EventDispatcher::setNowMsFn (std::function<uint64_t ()> fn)
{
    this->now_ms_fn = std::move (fn);
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
                /* A failed SMM-driven RTL (waiting_for_tasking) send is
                 * replayed here too: it is now tracked in the state
                 * machine's own pending_replay_state like every other
                 * safety-critical state (todo/46, todo/70), so no separate
                 * handling is needed in this handler. */
                state_machine.setMavCommsFailure (status == MavCommsStatus::failure);
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
                /* The SMM worker acquired/resumed a search. A command/latch that
                 * took over since the acquire started (rtl/terminate/etc.) must
                 * win, and this outcome is dropped rather than overriding it
                 * (todo/33) -- neither branch below fires in that case. */
                if (state_machine.isSearching ())
                {
                    mav.loadSearch (ls.search);
                }
                else if (state_machine.isWaitingForTasking ())
                {
                    /* SMM's background retry (kept alive by waiting_for_tasking
                     * not cancelling the searching role, todo/70/77) reacquired
                     * a search with no operator action. Resume via SMMNewCommand
                     * rather than uploading ls.search here: that transitions
                     * waiting_for_tasking -> searching, whose actionState()
                     * re-invokes SMM::doSearch()'s already-held-search resume
                     * path (todo/50) -- the same one "continue after a hold"
                     * uses -- which re-fires load_search_cb with the same
                     * search, landing in the isSearching() branch above. Doing
                     * both here (this upload AND the SMMNewCommand reset) would
                     * double-upload the same mission. */
                    state_machine.SMMNewCommand (smm_cmd_none);
                }
            },
            [&] (const SmmRtl &)
            {
                /* SMM has nothing to search right now -- either a held search's
                 * last waypoint completed with none queued behind it, or the
                 * next acquire attempt failed. Route through the state machine
                 * (todo/70) rather than commanding the MAV directly: its own
                 * priority arbitration already no-ops this call whenever a
                 * higher-priority FSS command or latch is in control (commandedState()
                 * only ever consults the SMM command when the FSS command alone
                 * maps to searching), so the isSearching() guard this used to
                 * need here is now redundant. Maps to fmu_state_waiting_for_tasking,
                 * not fmu_state_rtl: same RTL flight mode, but this must not
                 * cancel SMM's background acquire-retry loop (todo/77). */
                logger.log ("CMD smm rtl");
                state_machine.SMMNewCommand (smm_cmd_mission_complete);
            },
            [&] (const SmmOperatorCommand &oc)
            {
                /* An operator-issued 'AS'/'MC' command (todo/90), routed
                 * through the state machine's own arbitration exactly like
                 * SmmRtl above rather than acted on directly: any conflict
                 * with an in-flight acquire-failure RTL is resolved by
                 * SMMNewCommand's existing priority logic (todo/70). */
                logger.log (std::string ("CMD smm ") + smm_cmd_name (oc.command));
                state_machine.SMMNewCommand (oc.command);
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
                    /* A peer's coordinates are untrusted (todo/85): forwarding a
                     * non-finite or out-of-range position to the autopilot's
                     * collision-avoidance would be worse than not reporting
                     * this contact at all, so drop it before the rate-limit
                     * check even considers it for rebroadcast. */
                    if (!oar.pd.getP ().isValid ())
                    {
                        logger.log (LogLevel::debug, "ADSB: dropping a peer report with an invalid coordinate");
                        return;
                    }
                    /* Rate-limit ADS-B rebroadcast to one per ICAO address per
                     * second (todo/81): forward if this is the first sighting
                     * of this ICAO, or at least 1000ms has passed since the
                     * last forward. */
                    static constexpr uint64_t adsb_forward_interval_ms = 1000;
                    uint32_t icao = oar.pd.getICAOAddress ();
                    uint64_t now = now_ms_fn ();
                    auto it = adsb_last_forwarded_ms.find (icao);
                    if (it == adsb_last_forwarded_ms.end () || now - it->second >= adsb_forward_interval_ms)
                    {
                        mav.sendADSB (oar.pd);
                        adsb_last_forwarded_ms[icao] = now;
                    }
                }
            },
            [] (const Nudge &) {},
        },
        e);
}
