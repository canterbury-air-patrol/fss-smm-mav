#pragma once

#include <fss-transport.hpp>

#include "../fmu.hpp"

/* Translation between the FMU-domain command resolution (FSSCommandResolution,
 * produced by FMUStateMachine) and the transport-domain command-ack fields the
 * FMU sends back to FSS. Kept header-only and free of any I/O so it is unit
 * testable without the SSL transport. */

/* Map a resolved FSS command to the ack outcome FSS expects:
 *   - actioned: the command caused a real transition to the commanded state.
 *   - noop:     the command resolved to the state already current (nothing
 *               transitioned) — distinct from actioned so the operator is not
 *               shown a phantom transition.
 *   - superseded: a higher-priority latch blocked the command. */
inline auto
fss_command_ack_outcome_for (const FSSCommandResolution &res)
    -> flight_safety_system::transport::fss_command_ack_outcome
{
    if (res.outcome == fss_command_superseded)
    {
        return flight_safety_system::transport::command_ack_superseded;
    }
    if (res.transitioned)
    {
        return flight_safety_system::transport::command_ack_actioned;
    }
    return flight_safety_system::transport::command_ack_noop;
}

/* The ack for a stale command — an older, different command from a slow server
 * that a newer operator command has already replaced. It never reaches the state
 * machine (so there is no FSSCommandResolution to map), but it must still be
 * acked rather than dropped: superseded, with the dedicated newer-command reason
 * that distinguishes it from the autonomous safety latches. handleCommandFrom and
 * its test share these so the pairing cannot drift. */
inline constexpr auto stale_command_ack_outcome = flight_safety_system::transport::command_ack_superseded;
inline constexpr auto stale_command_ack_reason = flight_safety_system::transport::supersede_newer_command;

/* Map the superseding latch to the dedicated ack reason. low-battery RTL and
 * comms-loss failsafe both fly an RTL but must stay distinguishable, so the
 * reason is a dedicated enum rather than a command value. Only the autonomous
 * latches that can pre-empt an operator command appear here; anything else
 * (including a non-superseded outcome) is supersede_none. */
inline auto
fss_command_ack_reason_for (const FSSCommandResolution &res) -> flight_safety_system::transport::fss_command_ack_reason
{
    if (res.outcome != fss_command_superseded)
    {
        return flight_safety_system::transport::supersede_none;
    }
    switch (res.superseding_state)
    {
        case fmu_state_low_battery:
            return flight_safety_system::transport::supersede_low_battery;
        case fmu_state_failsafe:
            return flight_safety_system::transport::supersede_comms_loss;
        default:
            /* No other state should reach here: only the low-battery and
             * comms-failsafe latches supersede an FSS command (terminate is a
             * command in its own right, not a latch that drops another). */
            return flight_safety_system::transport::supersede_none;
    }
}
