#pragma once

#include <optional>

/* Core FMU state and command-resolution types. Kept in their own header (free of
 * the MAV/SMM/FSS interface includes that fmu.hpp pulls in) so lightweight
 * consumers — notably fmu-types.hpp, which fmu.hpp transitively includes — can
 * use them without an include cycle. */

enum FMUState
{
    fmu_state_manual,
    fmu_state_searching,
    fmu_state_goto,
    fmu_state_altitude_adjust,
    fmu_state_rtl,
    /* SMM has nothing to search right now (a search just completed with none
     * queued behind it, or the next acquire attempt failed): the aircraft
     * flies the same RTL flight-mode action as fmu_state_rtl, but unlike a
     * real RTL this is not a command to stop searching. actionState()
     * deliberately does not cancel the SMM searching role for this state, so
     * SMM's own background acquire-retry loop keeps running and a freshly
     * reacquired search auto-engages with no operator action. */
    fmu_state_waiting_for_tasking,
    fmu_state_hold,
    fmu_state_low_battery,
    fmu_state_failsafe,
    fmu_state_disarmed,
    fmu_state_terminate,
};

/* How an FSS command resolved once the priority logic ran. This is what the
 * command acknowledgement sent back to FSS reports:
 *   - actioned:   the aircraft is now in the commanded state, whether the
 *                 command caused a transition or it was already there.
 *   - superseded: a higher-priority latch blocked the command
 *                 (terminate > low battery > comms failsafe); superseding_state
 *                 names that latch's state. */
enum FSSCommandOutcome
{
    fss_command_actioned,
    fss_command_superseded,
};

struct FSSCommandResolution
{
    FSSCommandOutcome outcome{ fss_command_actioned };
    /* Set only when outcome == fss_command_superseded: the higher-priority
     * state that blocked the command. Empty otherwise, so an irrelevant value
     * cannot be read for an actioned command. */
    std::optional<FMUState> superseding_state{};
    /* True when the command actually moved the aircraft to a new state (as
     * opposed to confirming a state it was already in). Lets a caller treat a
     * benign no-op distinctly from a real transition if it needs to. */
    bool transitioned{ false };
};
