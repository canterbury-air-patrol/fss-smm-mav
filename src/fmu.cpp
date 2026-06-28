#include "fmu.hpp"
#include <iostream>
#include <mutex>

auto
map_smm_state (SMMCommand cmd) -> FMUState
{
    FMUState new_state = fmu_state_rtl;

    switch (cmd)
    {
        case smm_cmd_abandon_search:
        case smm_cmd_none:
            new_state = fmu_state_searching;
            break;
        case smm_cmd_mission_complete:
        default:
            new_state = fmu_state_rtl;
            break;
    }
    return new_state;
}

auto
map_fss_state (FSSCommand cmd) -> FMUState
{
    FMUState new_state = fmu_state_searching;
    switch (cmd)
    {
        case fss_cmd_rtl:
            new_state = fmu_state_rtl;
            break;
        case fss_cmd_goto:
            new_state = fmu_state_goto;
            break;
        case fss_cmd_hold:
            new_state = fmu_state_hold;
            break;
        case fss_cmd_altitude:
            new_state = fmu_state_altitude_adjust;
            break;
        case fss_cmd_disarm:
            new_state = fmu_state_disarmed;
            break;
        case fss_cmd_manual:
            new_state = fmu_state_manual;
            break;
        case fss_cmd_terminate:
            new_state = fmu_state_terminate;
            break;
        case fss_cmd_continue:
        case fss_cmd_unknown:
            new_state = fmu_state_searching;
            break;
    }
    return new_state;
}

auto
FMUStateMachine::commandedState () const -> FMUState
{
    /* fss_cmd_continue and an unknown command map to searching, which then
     * defers to the SMM command (search vs RTL). Every other FSS command maps
     * directly. */
    FMUState desired = map_fss_state (this->fss_command);
    if (desired == fmu_state_searching)
    {
        desired = map_smm_state (this->smm_command);
    }
    return desired;
}

auto
FMUStateMachine::updateState () -> std::optional<FMUState>
{
    /* Priority order: terminate > low battery > comms failure > FSS/SMM command.
     * Comms failure maps to fmu_state_failsafe, which is the default here, so it
     * needs no explicit branch (and the default is never an indeterminate value). */
    FMUState new_state = fmu_state_failsafe;
    if (this->fss_command == fss_cmd_terminate)
    {
        new_state = fmu_state_terminate;
    }
    else if (this->low_battery)
    {
        new_state = fmu_state_low_battery;
    }
    else if (!this->fss_comms_lost && !this->mav_comms_lost)
    {
        new_state = this->commandedState ();
    }

    if (new_state != this->current_state)
    {
        this->current_state = new_state;
        return new_state;
    }
    return std::nullopt;
}

auto
FMUStateMachine::resolveFSSCommand (FMUState desired, const std::optional<FMUState> &changed) -> FSSCommandResolution
{
    FSSCommandResolution res;
    res.transitioned = changed.has_value ();
    if (this->current_state == desired)
    {
        /* The aircraft is in the state the command asked for, whether this
         * command moved it there or it was already there. */
        res.outcome = fss_command_actioned;
    }
    else
    {
        /* updateState() selected a different state than the command alone maps
         * to, which only happens when a higher-priority latch (terminate, low
         * battery, or comms failsafe) is engaged. current_state is that latch's
         * state.
         *
         * Decision (todo/43): report "superseded" even when the command's effect
         * matches the active latch — e.g. an operator RTL while a low-battery or
         * comms-loss latch (which also flies RTL) is engaged. The latch, not the
         * command, is in control, and the ground station relies on this: the FSS
         * web's supersededRtlInEffect() keys off the superseded outcome and the
         * latch reason to show "low-battery/comms-loss RTL in effect" rather than
         * a failure. Reporting "actioned" here would discard that context. The
         * command is still retained (this->fss_command), so it re-applies if the
         * latch is a recoverable one (comms) that later clears. */
        res.outcome = fss_command_superseded;
        res.superseding_state = this->current_state;
    }
    return res;
}

namespace
{
/* States whose MAV command must reach the autopilot for flight safety. If the
 * send fails (link down) these are recorded for replay once the link recovers;
 * a failed manual/hold/goto/altitude is left to the operator to re-issue. */
auto
requires_replay_on_failure (FMUState state) -> bool
{
    switch (state)
    {
        case fmu_state_rtl:
        case fmu_state_failsafe:
        case fmu_state_low_battery:
        case fmu_state_terminate:
            return true;
        default:
            return false;
    }
}
} // namespace

auto
FMUStateMachine::actionState (FMUState state) -> bool
{
    if (state != fmu_state_searching)
    {
        this->smm.cancelSearch ();
    }
    /* searching is driven by SMM (which owns the mission upload), not a direct MAV
     * action whose transmission we track, so it counts as "sent". */
    bool sent = true;
    switch (state)
    {
        case fmu_state_manual:
            /* Tell MAV to exit auto mode */
            sent = this->mav.setMode (flight_mode_manual);
            break;
        case fmu_state_searching:
            /* Tell SMM to implement the search */
            this->smm.search (this->mav.getCurrentPosition ());
            break;
        case fmu_state_rtl:
        case fmu_state_failsafe:
        case fmu_state_low_battery:
            /* Tell MAV to RTL. When the MAV link is down (e.g. the comms-loss
             * failsafe fired precisely because telemetry was lost), the send is
             * skipped (sendMavLinkMsg short-circuits with the link down), so the
             * RTL does not reach the autopilot now. ArduPilot's own comms/GCS
             * failsafe is the immediate backstop; in addition, a failed send here
             * is recorded below and replayed once the MAV link recovers (todo/46). */
            sent = this->mav.setMode (flight_mode_rtl);
            break;
        case fmu_state_goto:
            /* Tell MAV to Goto the fss position. gotoPosition only stashes the
             * target (it transmits nothing), so the goto's transmission result is
             * entirely the following setMode(); that is what `sent` tracks. */
            this->mav.gotoPosition (this->fss.getGoto ());
            sent = this->mav.setMode (flight_mode_goto);
            break;
        case fmu_state_hold:
            /* Tell MAV to Circle/Hold Position */
            sent = this->mav.setMode (flight_mode_hold);
            break;
        case fmu_state_altitude_adjust:
            /* Tell MAV to adjust the altitude */
            sent = this->mav.setAltitude (this->fss.getAltitude ());
            break;
        case fmu_state_disarmed:
            /* Tell MAV to disarm the aircraft */
            sent = this->mav.disarm ();
            break;
        case fmu_state_terminate:
            /* Tell MAV to terminate the flight */
            sent = this->mav.terminate ();
            break;
    }

    {
        /* Remember a safety-critical action that did not make it onto the wire so
         * it can be replayed on MAV recovery; clear the record on any action that
         * did transmit (or any non-safety transition), since the autopilot now has
         * the latest command and there is nothing stale to re-send. */
        std::lock_guard<std::mutex> lk (this->lock);
        if (requires_replay_on_failure (state) && !sent)
        {
            this->pending_replay_state = state;
        }
        else
        {
            this->pending_replay_state.reset ();
        }
    }

    if (this->state_change_cb)
    {
        this->state_change_cb (state);
    }
    return sent;
}

auto
FMUStateMachine::FSSNewCommand (FSSCommand cmd) -> FSSCommandResolution
{
    std::optional<FMUState> changed_to;
    FSSCommandResolution resolution;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->fss_command = cmd;
        /* What the command alone maps to, ignoring the priority latches; shared
         * with updateState()'s comms-okay branch so the two cannot diverge. */
        FMUState desired = this->commandedState ();
        changed_to = this->updateState ();
        resolution = this->resolveFSSCommand (desired, changed_to);
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
    return resolution;
}

void
FMUStateMachine::SMMNewCommand (SMMCommand cmd)
{
    std::optional<FMUState> changed_to;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->smm_command = cmd;
        changed_to = this->updateState ();
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
}

void
FMUStateMachine::setLowBattery (bool low)
{
    std::optional<FMUState> changed_to;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        if (low)
        {
            /* Count consecutive low readings, saturating at the latch count so a
             * long flight cannot overflow the counter. The latch engages once
             * low_battery_latch_count readings have arrived in a row, which
             * rejects a single spurious sample. */
            if (this->low_battery_count < low_battery_latch_count)
            {
                this->low_battery_count++;
            }
            if (this->low_battery_count >= low_battery_latch_count && !this->low_battery)
            {
                this->low_battery = true;
                changed_to = this->updateState ();
            }
        }
        else
        {
            /* A healthy reading resets the run of low samples. The latch itself
             * is deliberately *not* cleared: once a critically low battery has
             * grounded the aircraft, a later optimistic reading must not release
             * it — recovery requires a restart. */
            this->low_battery_count = 0;
        }
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
}

void
FMUStateMachine::setCommsFailure (bool failed)
{
    std::optional<FMUState> changed_to;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->fss_comms_lost = failed;
        changed_to = this->updateState ();
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
}

void
FMUStateMachine::setMavCommsFailure (bool failed)
{
    std::optional<FMUState> changed_to;
    std::optional<FMUState> replay;
    {
        std::lock_guard<std::mutex> lk (this->lock);
        this->mav_comms_lost = failed;
        changed_to = this->updateState ();
        /* The MAV link just recovered. If a safety-critical action could not be
         * transmitted while it was down, re-send it now. A non-clearing latch
         * (low-battery, or a still-current terminate) keeps current_state the
         * same, so updateState() reports no transition and the action would
         * otherwise never be retried. A clearing latch (comms failsafe) instead
         * transitions to the commanded state, handled by the changed_to branch
         * below — which also clears any pending replay. */
        if (!failed && !changed_to && this->pending_replay_state.has_value ())
        {
            replay = this->pending_replay_state;
        }
    }
    if (changed_to)
    {
        this->actionState (*changed_to);
    }
    else if (replay)
    {
        this->actionState (*replay);
    }
}

FMUStateMachine::FMUStateMachine (IMAV &t_mav, ISMM &t_smm, IFSS &t_fss)
    : mav (t_mav), smm (t_smm), fss (t_fss), state_change_cb{}
{
}

void
FMUStateMachine::setStateChangeCB (std::function<void (FMUState)> cb)
{
    std::lock_guard<std::mutex> lk (this->lock);
    this->state_change_cb = std::move (cb);
}